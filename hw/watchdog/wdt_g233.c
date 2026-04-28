/*
 * G233 WDT Controller
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
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "system/reset.h"

#define TYPE_G233_WDT "g233_wdt"
typedef struct G233WDTState G233WDTState;
DECLARE_INSTANCE_CHECKER(G233WDTState, G233_WDT,
                         TYPE_G233_WDT)

#define G233_WDT_CTRL       0x00
/* bit 31-4 is reservied */
#define  WDT_LOCK           BIT(3)
#define  WDT_RSTEN          BIT(2)
#define  WDT_INTEN          BIT(1)
#define  WDT_EN             BIT(0)
#define  WDT_CTRL_MASK      (WDT_RSTEN | WDT_INTEN | WDT_EN)

#define G233_WDT_LOAD       0x04

#define G233_WDT_VAL        0x08

#define G233_WDT_SR         0x0c
/* bit 31-1 is reservied */
#define  WDT_TIMEOUT        BIT(0)

#define G233_WDT_KEY        0x10
#define  WDT_FEEDDOG        0x5a5a5a5a
#define  WDT_KEY_LOCK       0x1acce551

struct G233WDTState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    QEMUTimer timer;
    qemu_irq irq;

    uint32_t ctrl;
    uint32_t load;
    uint32_t val;
    uint32_t sr;
    uint32_t key;

    uint64_t freq_hz;
    uint64_t cached_time_ns;
};

static inline uint64_t ticks_to_ns(G233WDTState *s, uint64_t ticks)
{
    return muldiv64(ticks, NANOSECONDS_PER_SECOND, s->freq_hz);
}

static inline uint64_t ns_to_ticks(G233WDTState *s, uint64_t ns)
{
    return muldiv64(ns, s->freq_hz, NANOSECONDS_PER_SECOND);
}

/* only get val, do not trigger timer */
static inline uint32_t g233_wdt_get_val(G233WDTState *s)
{
    uint64_t now_ns, elapsed_ns, elapsed_ticks;

    if (!(s->ctrl & WDT_EN))
        return s->val;

    now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    elapsed_ns = now_ns - s->cached_time_ns;
    s->cached_time_ns = now_ns;
    elapsed_ticks = ns_to_ticks(s, elapsed_ns);

    if (s->val > elapsed_ticks)
        return s->val - elapsed_ticks;
    return 0;
}

static void g233_wdt_update_timer(G233WDTState *s)
{
    uint64_t expire_time, now_ns;

    if (!(s->ctrl & WDT_EN)) {
        timer_del(&s->timer);
        return;
    }

    now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->cached_time_ns = now_ns;
    expire_time = now_ns + ticks_to_ns(s, s->val);
    timer_mod(&s->timer, expire_time);
}

static void g233_wdt_interrupt(void *opaque)
{
    G233WDTState *s = G233_WDT(opaque);

    s->sr |= WDT_TIMEOUT;
    if (s->ctrl & WDT_INTEN)
        qemu_set_irq(s->irq, 1);
    if (s->ctrl & WDT_RSTEN)
        qapi_event_send_watchdog(WATCHDOG_ACTION_RESET);
}

// void g233_wdt_perform_action(G233WDTState *s)
// {
//     if (s->ctrl & WDT_INTEN)
//         qemu_set_irq(s->irq, 1);
//     if (s->ctrl & WDT_RSTEN)
//         qapi_event_send_watchdog(WATCHDOG_ACTION_RESET);
// }

static uint64_t g233_wdt_read(void *opaque, hwaddr addr,
                              unsigned int size)
{
    G233WDTState *s = G233_WDT(opaque);
    uint64_t val = 0;

    switch (addr) {
    case G233_WDT_CTRL:
        val = s->ctrl;
        break;
    case G233_WDT_LOAD:
        val = s->load;
        break;
    case G233_WDT_VAL:
        s->val = g233_wdt_get_val(s);
        val = s->val;
        break;
    case G233_WDT_SR:
        val = s->sr;
        break;
    case G233_WDT_KEY:
        val = 0;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds read at offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
    }

    // trace_aspeed_pwm_read(addr << 2, val);

    return val;
}

static void g233_wdt_write(void *opaque, hwaddr addr, uint64_t data,
                           unsigned int size)
{
    G233WDTState *s = G233_WDT(opaque);
    uint32_t old_val;

    // trace_aspeed_pwm_write(addr, data);

    switch (addr) {
    case G233_WDT_CTRL:
        if (s->ctrl & WDT_LOCK)
            break;

        old_val = s->ctrl;
        s->ctrl = (data & WDT_CTRL_MASK) | (old_val & WDT_LOCK);

        if ((data & WDT_EN) == (old_val & WDT_EN))
            break;

        if (data & WDT_EN) {
            s->val = s->load;
        } else {
            s->val = g233_wdt_get_val(s);
        }
        g233_wdt_update_timer(s);
        break;
    case G233_WDT_LOAD:
        s->load = data;
        break;
    case G233_WDT_SR:
        s->sr &= ~(data & WDT_TIMEOUT);
        break;
    case G233_WDT_KEY:
        if (data == WDT_FEEDDOG) {
            s->val = s->load;
            g233_wdt_update_timer(s);
        } else if (data == WDT_KEY_LOCK) {
            s->ctrl |= WDT_LOCK;
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds write at offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
    }
}

static const MemoryRegionOps g233_wdt_ops = {
    .read = g233_wdt_read,
    .write = g233_wdt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void g233_pwm_reset(DeviceState *dev)
{
    G233WDTState *s = G233_WDT(dev);

    s->ctrl = 0;
    s->load = 0;
    s->val  = 0;
    s->sr   = 0;
    s->key  = 0;
    timer_del(&s->timer);
}

static void g233_wdt_init(Object *obj)
{
    G233WDTState *s = G233_WDT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->freq_hz = 1000000000ULL;

    sysbus_init_irq(sbd, &s->irq);

    memory_region_init_io(&s->mmio, obj, &g233_wdt_ops, s,
                          TYPE_G233_WDT, 0x100);

    sysbus_init_mmio(sbd, &s->mmio);
}

static void g233_pwm_realize(DeviceState *dev, Error **errp)
{
    struct G233WDTState *s = G233_WDT(dev);

    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL,
                  g233_wdt_interrupt, s);
}

static const VMStateDescription vmstate_g233_pwm = {
    .name = TYPE_G233_WDT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(ctrl, G233WDTState),
        VMSTATE_UINT32(load, G233WDTState),
        VMSTATE_UINT32(val, G233WDTState),
        VMSTATE_UINT32(sr, G233WDTState),
        VMSTATE_UINT32(key, G233WDTState),
        VMSTATE_UINT64(cached_time_ns, G233WDTState),
        VMSTATE_END_OF_LIST(),
    }
};

static void g233_wdt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = g233_pwm_realize;
    device_class_set_legacy_reset(dc, g233_pwm_reset);
    dc->desc = "G233 WDT Controller";
    dc->vmsd = &vmstate_g233_pwm;
}

static const TypeInfo g233_wdt_info = {
    .name = TYPE_G233_WDT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(G233WDTState),
    .instance_init = g233_wdt_init,
    .class_init = g233_wdt_class_init,
};

static void g233_pwm_register_types(void)
{
    type_register_static(&g233_wdt_info);
}

type_init(g233_pwm_register_types);
