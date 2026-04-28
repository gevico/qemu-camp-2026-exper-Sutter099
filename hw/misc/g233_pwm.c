/*
 * G233 PWM Controller
 *
 * Copyright 2026 Ze Huang
 *
 * Based on aspeed_pwm.c:
 *
 * Copyright (C) 2017-2021 IBM Corp.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "trace.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/registerfields.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_G233_PWM "g233_pwm"
typedef struct G233PWMState G233PWMState;
DECLARE_INSTANCE_CHECKER(G233PWMState, G233_PWM,
                         TYPE_G233_PWM)

#define G233_PWM_GLB        0x00
/* bit 31-8 is reservied */
#define  PWM_CH3_DONE       BIT(7)
#define  PWM_CH2_DONE       BIT(6)
#define  PWM_CH1_DONE       BIT(5)
#define  PWM_CH0_DONE       BIT(4)
#define  PWM_CH3_EN         BIT(3)
#define  PWM_CH2_EN         BIT(2)
#define  PWM_CH1_EN         BIT(1)
#define  PWM_CH0_EN         BIT(0)

#define G233_PWM_CH_CTRL    0x10
/* bit 31-3 is reservied */
#define  PWM_INTIE          BIT(2)
#define  PWM_POL            BIT(1)
#define  PWM_EN             BIT(0)

#define G233_PWM_CH_PERIOD  0x14
#define G233_PWM_CH_DUTY    0x18
#define G233_PWM_CH_CNT     0x1c

#define G233_GLB_SIZE       0x10
#define G233_CH_OFFSET      0x10
#define G233_PWM_CHANS      4

struct G233PWMReg {
    uint32_t ctrl;
    uint32_t period;
    uint32_t duty;
    uint32_t cnt;
};

struct G233PWMState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    QEMUTimer timer[G233_PWM_CHANS];
    qemu_irq irq;
    qemu_irq output[G233_PWM_CHANS];

    uint32_t glb;
    struct G233PWMReg ch_regs[G233_PWM_CHANS];
    uint64_t freq_hz;
};

// static void update_state(void *opaque)
// {
//     // G233PWMState *s = G233_PWM(opaque);
//
//     // TODO:
//     // 配置通道周期：写 PWM_CHn_PERIOD。
//     // 配置占空比：写 PWM_CHn_DUTY（值须 <= PERIOD）。
//     // 按需配置极性与中断：写 PWM_CHn_CTRL（POL、INTIE）。
//     // 启动通道：置位 PWM_CHn_CTRL.EN。
//     // 周期完成中断处理：读 PWM_GLB 确认哪些通道完成，向对应 DONE 位写 1 清除。
//
//
// }

static inline uint64_t g233_pwm_ticks_to_ns(G233PWMState *s,
                                            uint64_t ticks)
{
    return muldiv64(ticks, NANOSECONDS_PER_SECOND, s->freq_hz);
}

static void g233_pwm_set_alarms(G233PWMState *s)
{
    uint64_t now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t count_ns, time_to_fire, offset = 0;
    int i;

    for (i = 0; i < G233_PWM_CHANS; ++i) {
        count_ns = g233_pwm_ticks_to_ns(s, s->ch_regs[i].cnt);

        /* if not enabled and do not reach trigger time, never */
        time_to_fire = 0xFFFFFFFFFFFFFF;

        /* trigger immeditately */
        if (count_ns >= now_ns) {
            // TODO: trigger int
            time_to_fire = now_ns + 1;

        /* schedule a trigger */
        } else if (s->ch_regs[i].ctrl & PWM_EN) {
            time_to_fire = now_ns + offset;
        }

        timer_mod(&s->timer[i], time_to_fire);
    }
}

static uint64_t g233_pwm_read(void *opaque, hwaddr addr,
                              unsigned int size)
{
    G233PWMState *s = G233_PWM(opaque);
    uint64_t val = 0;
    uint32_t ch = 0;

    if (addr > G233_GLB_SIZE)
        ch = (addr - G233_GLB_SIZE) / G233_CH_OFFSET;

    addr = addr - G233_GLB_SIZE * ch;

    switch (addr) {
    case G233_PWM_GLB:
        val = s->glb;
        break;
    case G233_PWM_CH_CTRL:
        val = s->ch_regs[ch].ctrl;
        break;
    case G233_PWM_CH_PERIOD:
        val = s->ch_regs[ch].period;
        break;
    case G233_PWM_CH_DUTY:
        val = s->ch_regs[ch].duty;
        break;
    case G233_PWM_CH_CNT:
        val = s->ch_regs[ch].cnt;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds read at offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
    }

    // trace_aspeed_pwm_read(addr << 2, val);

    return val;
}

static void g233_pwm_write(void *opaque, hwaddr addr, uint64_t data,
                           unsigned int size)
{
    G233PWMState *s = G233_PWM(opaque);
    uint32_t ch;

    // trace_aspeed_pwm_write(addr, data);

    if (addr > G233_GLB_SIZE)
        ch = (addr - G233_GLB_SIZE) / G233_CH_OFFSET;

    addr = addr - G233_GLB_SIZE * ch;

    switch (addr) {
    case G233_PWM_GLB:
        s->glb = data;
        break;
    case G233_PWM_CH_CTRL:
        s->ch_regs[ch].ctrl = data;
        break;
    case G233_PWM_CH_PERIOD:
        s->ch_regs[ch].period = data;
        break;
    case G233_PWM_CH_DUTY:
        s->ch_regs[ch].duty = data;
        break;
    case G233_PWM_CH_CNT:
        s->ch_regs[ch].cnt = data;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds write at offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
    }

    // FIX:
    g233_pwm_set_alarms(s);

    // TODO: update state?
}

static const MemoryRegionOps g233_pwm_ops = {
    .read = g233_pwm_read,
    .write = g233_pwm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void g233_pwm_interrupt(G233PWMState *s, int num)
{

}

static void g233_pwm_interrupt_0(void *opaque)
{
    G233PWMState *s = opaque;

    g233_pwm_interrupt(s, 0);
}

static void g233_pwm_interrupt_1(void *opaque)
{
    G233PWMState *s = opaque;

    g233_pwm_interrupt(s, 0);
}

static void g233_pwm_interrupt_2(void *opaque)
{
    G233PWMState *s = opaque;

    g233_pwm_interrupt(s, 0);
}

static void g233_pwm_interrupt_3(void *opaque)
{
    G233PWMState *s = opaque;

    g233_pwm_interrupt(s, 0);
}

static void g233_pwm_reset(DeviceState *dev)
{
    G233PWMState *s = G233_PWM(dev);

    s->glb = 0;
    memset(s->ch_regs, 0, sizeof(s->ch_regs));
}

static void g233_pwm_init(Object *obj)
{
    G233PWMState *s = G233_PWM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    sysbus_init_irq(sbd, &s->irq);

    memory_region_init_io(&s->mmio, obj, &g233_pwm_ops, s,
            TYPE_G233_PWM, 0x100);

    sysbus_init_mmio(sbd, &s->mmio);
}

static void g233_pwm_realize(DeviceState *dev, Error **errp)
{
    struct G233PWMState *s = G233_PWM(dev);

    timer_init_ns(&s->timer[0], QEMU_CLOCK_VIRTUAL,
                  g233_pwm_interrupt_0, s);
    timer_init_ns(&s->timer[1], QEMU_CLOCK_VIRTUAL,
                  g233_pwm_interrupt_1, s);
    timer_init_ns(&s->timer[2], QEMU_CLOCK_VIRTUAL,
                  g233_pwm_interrupt_2, s);
    timer_init_ns(&s->timer[3], QEMU_CLOCK_VIRTUAL,
                  g233_pwm_interrupt_3, s);
}

static const VMStateDescription vmstate_g233_pwm_reg = {
    .name = "g233-pwm-reg",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(ctrl, struct G233PWMReg),
        VMSTATE_UINT32(period, struct G233PWMReg),
        VMSTATE_UINT32(duty, struct G233PWMReg),
        VMSTATE_UINT32(cnt, struct G233PWMReg),
        VMSTATE_END_OF_LIST(),
    }
};

static const VMStateDescription vmstate_g233_pwm = {
    .name = TYPE_G233_PWM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(glb, G233PWMState),
        VMSTATE_STRUCT_ARRAY(ch_regs, struct G233PWMState,
                             G233_PWM_CHANS, 1,
                             vmstate_g233_pwm_reg,
                             struct G233PWMReg),
        VMSTATE_END_OF_LIST(),
    }
};

static void g233_pwm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = g233_pwm_realize;
    device_class_set_legacy_reset(dc, g233_pwm_reset);
    dc->desc = "G233 PWM Controller";
    dc->vmsd = &vmstate_g233_pwm;
}

static const TypeInfo g233_pwm_info = {
    .name = TYPE_G233_PWM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(G233PWMState),
    .instance_init = g233_pwm_init,
    .class_init = g233_pwm_class_init,
};

static void g233_pwm_register_types(void)
{
    type_register_static(&g233_pwm_info);
}

type_init(g233_pwm_register_types);
