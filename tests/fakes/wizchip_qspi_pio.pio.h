#ifndef FAKE_WIZCHIP_QSPI_PIO_PROGRAM_H
#define FAKE_WIZCHIP_QSPI_PIO_PROGRAM_H

#include "fake_pico_sdk.h"

static const pio_program_t wiznet_spi_write_read_program = 0u;

enum {
    wiznet_spi_write_read_offset_write_bits = 0u,
    wiznet_spi_write_read_offset_write_end = 1u,
    wiznet_spi_write_read_offset_read_end = 2u
};

static inline pio_sm_config
wiznet_spi_write_read_program_get_default_config(uint offset)
{
    (void)offset;
    return pio_get_default_sm_config();
}

#endif
