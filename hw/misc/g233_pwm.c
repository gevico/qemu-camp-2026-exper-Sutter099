/*
 * G233 PWM Controller
 *
 * Copyright 2026 Ze Huang
 *
 * Based on sifive_pwm.c:
 * Copyright (c) 2020 Western Digital
 * Author:  Alistair Francis <alistair.francis@wdc.com>
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

    uint64_t cached_time_ns;
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

static inline uint64_t ticks_to_ns(G233PWMState *s, uint64_t ticks)
{
    return muldiv64(ticks, NANOSECONDS_PER_SECOND, s->freq_hz);
}

static inline uint64_t ns_to_ticks(G233PWMState *s, uint64_t ns)
{
    return muldiv64(ns, s->freq_hz, NANOSECONDS_PER_SECOND);
}

/* only get cnt num, do not trigger timer */
static inline uint32_t g233_pwm_get_cnt(G233PWMState *s, int ch)
{
    struct G233PWMReg *reg = &s->ch_regs[ch];
    uint64_t now_ns, elapsed_ns, elapsed_ticks;

    if (!(reg->ctrl & PWM_EN))
        return reg->cnt;
    if (reg->period == 0)
        return 0;

    now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    elapsed_ns = now_ns - reg->cached_time_ns;
    elapsed_ticks = ns_to_ticks(s, elapsed_ns);

    return (reg->cnt + elapsed_ticks) % reg->period;
}

static void g233_pwm_update_channel(G233PWMState *s, int ch)
{
    struct G233PWMReg *reg = &s->ch_regs[ch];
    uint32_t ticks_to_end;
    uint64_t expire_time;

    if (!(reg->ctrl & PWM_EN) || (reg->period <= 0)) {
        timer_del(&s->timer[ch]);
        return;
    }

    reg->cached_time_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    ticks_to_end = reg->period - reg->cnt;
    expire_time = reg->cached_time_ns + ticks_to_ns(s, ticks_to_end);

    timer_mod(&s->timer[ch], expire_time);
}

static void g233_pwm_interrupt(G233PWMState *s, int ch)
{
    struct G233PWMReg *reg = &s->ch_regs[ch];

    s->glb |= (1 << (4 + ch));

    if (reg->ctrl & PWM_INTIE)
        qemu_irq_raise(s->irq);

    if (reg->ctrl & PWM_EN) {
        reg->cnt = 0;
        g233_pwm_update_channel(s, ch);
    }
}

static uint64_t g233_pwm_read(void *opaque, hwaddr addr,
                              unsigned int size)
{
    G233PWMState *s = G233_PWM(opaque);
    uint64_t val = 0;
    uint32_t ch = 0;

    if (addr == G233_PWM_GLB) {
        for (int i = 0; i < G233_PWM_CHANS; ++i)
            if (s->ch_regs[i].ctrl & PWM_EN)
                s->glb |= (1 << i);

        return s->glb;
    }

    ch = (addr - G233_GLB_SIZE) / G233_CH_OFFSET;
    addr = addr - G233_GLB_SIZE * ch;

    if (ch >= G233_PWM_CHANS) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Invalid channel %d\n", __func__, ch);
        return 0;
    }

    switch (addr) {
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
        val = g233_pwm_get_cnt(s, ch);
        s->ch_regs[ch].cnt = val;
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

    if (addr == G233_PWM_GLB) {
        s->glb &= ~(data & 0xf0);
        return;
    }

    ch = (addr - G233_GLB_SIZE) / G233_CH_OFFSET;
    addr = addr - G233_GLB_SIZE * ch;

    if (ch >= G233_PWM_CHANS) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Invalid channel %d\n", __func__, ch);
        return;
    }

    switch (addr) {
    case G233_PWM_GLB:
        s->glb = data;
        break;
    case G233_PWM_CH_CTRL:
        uint32_t old_ctrl = s->ch_regs[ch].ctrl;
        s->ch_regs[ch].ctrl = data;

        if ((old_ctrl & PWM_EN) == (data & PWM_EN))
            break;

        if (data & PWM_EN) {
            g233_pwm_update_channel(s, ch);
        } else {
            s->ch_regs[ch].cnt = g233_pwm_get_cnt(s, ch);
            timer_del(&s->timer[ch]);
        }
        break;
    case G233_PWM_CH_PERIOD:
        s->ch_regs[ch].period = data;
        if (s->ch_regs[ch].ctrl & PWM_EN)
            g233_pwm_update_channel(s, ch);
        break;
    case G233_PWM_CH_DUTY:
        s->ch_regs[ch].duty = data;
        break;
    case G233_PWM_CH_CNT:
        s->ch_regs[ch].cnt = data;
        if (s->ch_regs[ch].ctrl & PWM_EN)
            g233_pwm_update_channel(s, ch);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds write at offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
    }
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

static void g233_pwm_interrupt_0(void *opaque)
{
    G233PWMState *s = opaque;

    g233_pwm_interrupt(s, 0);
}

static void g233_pwm_interrupt_1(void *opaque)
{
    G233PWMState *s = opaque;

    g233_pwm_interrupt(s, 1);
}

static void g233_pwm_interrupt_2(void *opaque)
{
    G233PWMState *s = opaque;

    g233_pwm_interrupt(s, 2);
}

static void g233_pwm_interrupt_3(void *opaque)
{
    G233PWMState *s = opaque;

    g233_pwm_interrupt(s, 3);
}

static void g233_pwm_reset(DeviceState *dev)
{
    G233PWMState *s = G233_PWM(dev);

    s->glb = 0;
    memset(s->ch_regs, 0, sizeof(s->ch_regs));

    for (int i = 0; i < G233_PWM_CHANS; ++i)
        timer_del(&s->timer[i]);
}

static void g233_pwm_init(Object *obj)
{
    G233PWMState *s = G233_PWM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->freq_hz = 1000000000ULL;

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
        VMSTATE_UINT64(cached_time_ns, struct G233PWMReg),
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
