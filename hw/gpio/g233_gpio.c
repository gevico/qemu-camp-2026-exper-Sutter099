/*
 * G233 System-on-Chip general purpose input/output register definition
 *
 * Copyright 2026 Ze Huang
 *
 * Base on nrf51_gpio.c and sifive_gpio.c:
 *
 * Copyright 2019 AdaCore
 * Copyright 2018 Steffen Görtz <contrib@steffen-goertz.de>
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/g233_gpio.h"
#include "migration/vmstate.h"
#include "trace.h"

static void update_output(G233GPIOState *s)
{
    uint32_t pin, i;

    for (i = 0; i < s->ngpio; i++) {
        pin = 1U << i;
        qemu_set_irq(s->output[i], (s->dir & pin) && (s->out & pin));
    }
}

static void update_state(G233GPIOState *s)
{
    uint32_t prev_in = s->in;
    uint32_t output_mask = s->out & s->dir;
    uint32_t input_mask = ~s->dir;
    uint32_t current_in = output_mask | input_mask;
    uint32_t rising = ~prev_in & current_in;
    uint32_t falling = prev_in & ~current_in;
    uint32_t edge_mask = ~s->trig;
    uint32_t level_mask = s->trig;
    uint32_t edge_pending;
    uint32_t level_pending;
    uint32_t pin;

    s->in = current_in;

    edge_pending = ((s->pol & rising) | (~s->pol & falling)) & edge_mask;
    level_pending = ((s->pol & current_in) | (~s->pol & ~current_in)) & level_mask;

    /* Edge interrupts latch until cleared via W1C; level interrupts reflect
     * the current input level and should not stay asserted once the level
     * condition goes away. GPIO_IS only records enabled interrupt sources.
     */
    s->is &= edge_mask;
    s->is |= (edge_pending | level_pending) & s->ie;

    update_output(s);

    for (int i = 0; i < s->ngpio; i++) {
        pin = 1U << i;

        if (s->ie & s->is & pin) {
            qemu_set_irq(s->irq, 1);
            break;
        }
    }
}

static uint64_t g233_gpio_read(void *opaque, hwaddr offset, unsigned int size)
{
    G233GPIOState *s = G233_GPIO(opaque);
    uint64_t r = 0;

    switch (offset) {
    case G233_GPIO_DIR:
        r = s->dir;
        break;
    case G233_GPIO_OUT:
        r = s->out;
        break;
    case G233_GPIO_IN:
        r = s->in;
        break;
    case G233_GPIO_IE:
        r = s->ie;
        break;
    case G233_GPIO_IS:
        r = s->is;
        break;
    case G233_GPIO_TRIG:
        r = s->trig;
        break;
    case G233_GPIO_POL:
        r = s->pol;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
    }

    // trace_sifive_gpio_read(offset, r);

    return r;
}

static void g233_gpio_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned int size)
{
    G233GPIOState *s = G233_GPIO(opaque);

    // trace_sifive_gpio_write(offset, value);

    switch (offset) {
    case G233_GPIO_DIR:
        s->dir = value;
        break;
    case G233_GPIO_OUT:
        s->out = value;
        break;
    case G233_GPIO_IN:
        break;
    case G233_GPIO_IE:
        s->ie = value;
        break;
    case G233_GPIO_IS:
        s->is &= ~value;
        break;
    case G233_GPIO_TRIG:
        s->trig = value;
        break;
    case G233_GPIO_POL:
        s->pol = value;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
    }

    update_state(s);
}

static const MemoryRegionOps gpio_ops = {
    .read =  g233_gpio_read,
    .write = g233_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void g233_gpio_set(void *opaque, int line, int value)
{
    G233GPIOState *s = G233_GPIO(opaque);

    // trace_sifive_gpio_set(line, value);

    assert(line >= 0 && line < G233_GPIO_PINS);

    update_state(s);
}

static void g233_gpio_reset(DeviceState *dev)
{
    G233GPIOState *s = G233_GPIO(dev);

    s->dir = 0;
    s->out = 0;
    s->in = 0;
    s->ie = 0;
    s->is = 0;
    s->trig = 0;
    s->pol = 0;
}

static const VMStateDescription vmstate_g233_gpio = {
    .name = TYPE_G233_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(dir, G233GPIOState),
        VMSTATE_UINT32(out, G233GPIOState),
        VMSTATE_UINT32(in, G233GPIOState),
        VMSTATE_UINT32(ie, G233GPIOState),
        VMSTATE_UINT32(is, G233GPIOState),
        VMSTATE_UINT32(trig, G233GPIOState),
        VMSTATE_UINT32(pol, G233GPIOState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property g233_gpio_properties[] = {
    DEFINE_PROP_UINT32("ngpio", G233GPIOState, ngpio, G233_GPIO_PINS),
};

static void g233_gpio_realize(DeviceState *dev, Error **errp)
{
    G233GPIOState *s = G233_GPIO(dev);

    memory_region_init_io(&s->mmio, OBJECT(dev), &gpio_ops, s,
                          TYPE_G233_GPIO, G233_GPIO_SIZE);

    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    qdev_init_gpio_in(DEVICE(s), g233_gpio_set, s->ngpio);
    qdev_init_gpio_out(DEVICE(s), s->output, s->ngpio);
}

static void g233_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, g233_gpio_properties);
    dc->vmsd = &vmstate_g233_gpio;
    dc->realize = g233_gpio_realize;
    device_class_set_legacy_reset(dc, g233_gpio_reset);
    dc->desc = "G233 GPIO";
}

static const TypeInfo g233_gpio_info = {
    .name = TYPE_G233_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(G233GPIOState),
    .class_init = g233_gpio_class_init
};

static void g233_gpio_register_types(void)
{
    type_register_static(&g233_gpio_info);
}

type_init(g233_gpio_register_types)
