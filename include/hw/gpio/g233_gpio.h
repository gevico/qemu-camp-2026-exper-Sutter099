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

#ifndef G233_GPIO_H
#define G233_GPIO_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_G233_GPIO "g233_gpio"
typedef struct G233GPIOState G233GPIOState;
DECLARE_INSTANCE_CHECKER(G233GPIOState, G233_GPIO,
                         TYPE_G233_GPIO)

#define G233_GPIO_PINS 32

#define G233_GPIO_SIZE 0x100

#define G233_GPIO_DIR       0x00
#define G233_GPIO_OUT       0x04
#define G233_GPIO_IN        0x08
#define G233_GPIO_IE        0x0C
#define G233_GPIO_IS        0x10
#define G233_GPIO_TRIG      0x14
#define G233_GPIO_POL       0x18

struct G233GPIOState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    qemu_irq irq;
    qemu_irq output[G233_GPIO_PINS];

    uint32_t dir;
    uint32_t out;
    uint32_t in;
    uint32_t in_val; /* cache input value */
    uint32_t ie;
    uint32_t is;
    uint32_t trig;
    uint32_t pol;

    /* config */
    uint32_t ngpio;
};

#endif /* G233_GPIO_H */
