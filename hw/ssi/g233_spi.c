/*
 * G233 SPI Controller
 *
 * Copyright (C) 2026 Ze Huang
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/bitops.h"
#include "qom/object.h"
#include "hw/core/irq.h"
#include "hw/ssi/ssi.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"

#define SPI_CR1                 0x00  /* Control Register 1 */
/* bit 31:8 reserved */
#define  SPI_CR1_TXEIE          BIT(7)
#define  SPI_CR1_RXNEIE         BIT(6)
#define  SPI_CR1_ERRIE          BIT(5)
#define  SPI_CR1_MSTR           BIT(2)
/* bit 1 reserved */
#define  SPI_CR1_SPE            BIT(0)
#define  SPI_CR1_MASK           (SPI_CR1_SPE | SPI_CR1_MSTR | SPI_CR1_ERRIE |\
                                 SPI_CR1_RXNEIE | SPI_CR1_TXEIE)

#define SPI_CR2                 0x04
/* bit 31:2 reserved */
#define  SPI_CR2_CS_SEL_MASK    MAKE_64BIT_MASK(0, 2)

#define SPI_SR                  0x08
/* bit 31:5 reserved */
#define  SPI_SR_OVERRUN         BIT(4)
/* bit 3:2 reserved */
#define  SPI_SR_TXE             BIT(1)
#define  SPI_SR_RXNE            BIT(0)

#define SPI_DR                  0x0c
/* bit 31:8 reserved */
#define  SPI_DR_DATA_MASK       MAKE_64BIT_MASK(0, 8)

#define NR_DEVICE               4
#define G233_SPI_SIZE           0x100

#define TYPE_G233_SPI "g233_spi"
typedef struct G233SPIState G233SPIState;
DECLARE_INSTANCE_CHECKER(G233SPIState, G233_SPI,
                         TYPE_G233_SPI)

struct G233SPIState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    qemu_irq irq;
    qemu_irq cs_lines[NR_DEVICE];

    SSIBus *spi;

    uint32_t cr1;
    uint32_t cr2;
    uint32_t sr;
    uint32_t dr;
};

static void g233_spi_update_irq(G233SPIState *s)
{
    bool txe_irq = (s->cr1 & SPI_CR1_TXEIE) && (s->sr & SPI_SR_TXE);
    bool rxne_irq = (s->cr1 & SPI_CR1_RXNEIE) && (s->sr & SPI_SR_RXNE);
    bool err_irq = (s->cr1 & SPI_CR1_ERRIE) && (s->sr & SPI_SR_OVERRUN);
    bool en = s->cr1 & SPI_CR1_SPE;

    if (en && (txe_irq || rxne_irq || err_irq)) {
        qemu_set_irq(s->irq, 1);
    } else {
        qemu_set_irq(s->irq, 0);
    }
}

static uint64_t g233_spi_read(void *opaque, hwaddr offset, unsigned int size)
{
    G233SPIState *s = G233_SPI(opaque);
    uint64_t r = 0;

    switch (offset) {
    case SPI_CR1:
        r = s->cr1;
        break;
    case SPI_CR2:
        r = s->cr2;
        break;
    case SPI_SR:
        r = s->sr;
        break;
    case SPI_DR:
        r = s->dr;
        s->sr &= ~SPI_SR_RXNE;
        g233_spi_update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
    }

    return r;
}

static void g233_spi_write(void *opaque, hwaddr offset,
                           uint64_t value, unsigned int size)
{
    G233SPIState *s = G233_SPI(opaque);
    uint64_t old_val;

    switch (offset) {
    case SPI_CR1:
        old_val = s->cr1;
        s->cr1 = value & SPI_CR1_MASK;
        if ((old_val & SPI_CR1_SPE) && !(s->cr1 & SPI_CR1_SPE)) {
            qemu_set_irq(s->cs_lines[s->cr2], 1);
        } else if (!(old_val & SPI_CR1_SPE) && (s->cr1 & SPI_CR1_SPE)) {
            qemu_set_irq(s->cs_lines[s->cr2], 0);
        }
        g233_spi_update_irq(s);
        break;
    case SPI_CR2:
        old_val = s->cr2;
        s->cr2 = value & SPI_CR2_CS_SEL_MASK;
        if (old_val == s->cr2 || !(s->cr1 & SPI_CR1_SPE))
            break;
        qemu_set_irq(s->cs_lines[old_val], 1);
        qemu_set_irq(s->cs_lines[s->cr2], 0);
        break;
    case SPI_SR:
        s->sr &= ~(value & SPI_SR_OVERRUN);
        g233_spi_update_irq(s);
        break;
    case SPI_DR:
        if (!(s->cr1 & SPI_CR1_SPE))
            break;
        if (s->sr & SPI_SR_RXNE) {
            s->sr |= SPI_SR_OVERRUN;
            g233_spi_update_irq(s);
        }
        s->dr = ssi_transfer(s->spi, value & SPI_DR_DATA_MASK);
        s->sr |= SPI_SR_TXE;
        s->sr |= SPI_SR_RXNE;
        g233_spi_update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
    }
}

static const MemoryRegionOps spi_ops = {
    .read =  g233_spi_read,
    .write = g233_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void g233_spi_reset(DeviceState *dev)
{
    G233SPIState *s = G233_SPI(dev);

    for (int i = 0; i < NR_DEVICE; i++)
        qemu_set_irq(s->cs_lines[i], 1);

    s->cr1 = 0;
    s->cr2 = 0;
    s->sr = 0x2;
    s->dr = 0;

    g233_spi_update_irq(s);
}

static const VMStateDescription vmstate_g233_spi = {
    .name = TYPE_G233_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cr1, G233SPIState),
        VMSTATE_UINT32(cr2, G233SPIState),
        VMSTATE_UINT32(sr, G233SPIState),
        VMSTATE_UINT32(dr, G233SPIState),
        VMSTATE_END_OF_LIST()
    }
};

static void g233_spi_realize(DeviceState *dev, Error **errp)
{
    G233SPIState *s = G233_SPI(dev);

    memory_region_init_io(&s->mmio, OBJECT(dev), &spi_ops, s,
                          TYPE_G233_SPI, G233_SPI_SIZE);

    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->spi = ssi_create_bus(dev, "spi");
    qdev_init_gpio_out_named(DEVICE(s), s->cs_lines, "cs", NR_DEVICE);
}

static void g233_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_g233_spi;
    dc->realize = g233_spi_realize;
    device_class_set_legacy_reset(dc, g233_spi_reset);
    dc->desc = "G233 SPI";
}

static const TypeInfo g233_spi_info = {
    .name = TYPE_G233_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(G233SPIState),
    .class_init = g233_spi_class_init
};

static void g233_spi_register_types(void)
{
    type_register_static(&g233_spi_info);
}

type_init(g233_spi_register_types)
