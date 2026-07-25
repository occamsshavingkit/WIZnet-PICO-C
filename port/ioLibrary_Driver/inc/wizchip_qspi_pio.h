/*
    Copyright (c) 2023 Raspberry Pi (Trading) Ltd.

    SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef _WIZNET_SPI_FUNCS_H_
#define _WIZNET_SPI_FUNCS_H_

#include <stdint.h>

typedef struct wiznet_spi_funcs** wiznet_spi_handle_t;

typedef enum wiznet_spi_lifecycle_state {
    WIZNET_SPI_UNINIT = 0,
    WIZNET_SPI_OPENING,
    WIZNET_SPI_READY,
    WIZNET_SPI_TRANSFERRING,
    WIZNET_SPI_CLOSING,
    WIZNET_SPI_FAULTED,
    WIZNET_SPI_SLEEPING
} wiznet_spi_lifecycle_state_t;

#define WIZNET_SPI_MAX_TRANSFER_SIZE 16384u

#if   (_WIZCHIP_ == W6300)
typedef struct wiznet_spi_config {
    uint16_t clock_div_major;
    uint8_t clock_div_minor;
    uint8_t clock_pin;
    uint8_t data_io0_pin;
    uint8_t data_io1_pin;
    uint8_t data_io2_pin;
    uint8_t data_io3_pin;
    uint8_t cs_pin;
    uint8_t reset_pin;
    uint8_t irq_pin;
    uint32_t transfer_timeout_us;
    uint32_t abort_timeout_us;
} wiznet_spi_config_t;

typedef struct wiznet_spi_funcs {
    void (*close)(wiznet_spi_handle_t funcs);
    void (*set_active)(wiznet_spi_handle_t funcs);
    void (*set_inactive)(void);
    void (*frame_start)(void);
    void (*frame_end)(void);
    void (*read_byte)(uint8_t opcode, uint16_t addr, uint8_t* pBuf, uint16_t len);
    void (*write_byte)(uint8_t opcode, uint16_t addr, uint8_t* pBuf, uint16_t len);
    void (*read_buffer)(uint8_t *pBuf, uint16_t len);
    void (*write_buffer)(uint8_t *pBuf, uint16_t len);
    void (*reset)(wiznet_spi_handle_t funcs);
    void (*sleep)(wiznet_spi_handle_t funcs);
    void (*wake)(wiznet_spi_handle_t funcs);
} wiznet_spi_funcs_t;
#else
typedef struct wiznet_spi_config {
    uint8_t data_in_pin;
    uint8_t data_out_pin;
    uint8_t cs_pin;
    uint8_t clock_pin;
    uint8_t irq_pin;
    uint8_t reset_pin;
    uint16_t clock_div_major;
    uint8_t clock_div_minor;
    uint8_t spi_hw_instance;
    uint32_t transfer_timeout_us;
    uint32_t abort_timeout_us;
} wiznet_spi_config_t;

typedef struct wiznet_spi_funcs {
    void (*close)(wiznet_spi_handle_t funcs);
    void (*set_active)(wiznet_spi_handle_t funcs);
    void (*set_inactive)(void);
    void (*frame_start)(void);
    void (*frame_end)(void);
    uint8_t (*read_byte)(void);
    void (*write_byte)(uint8_t tx_data);
    void (*read_buffer)(uint8_t *pBuf, uint16_t len);
    void (*write_buffer)(uint8_t *pBuf, uint16_t len);
    void (*reset)(wiznet_spi_handle_t funcs);
    void (*sleep)(wiznet_spi_handle_t funcs);
    void (*wake)(wiznet_spi_handle_t funcs);
} wiznet_spi_funcs_t;
#endif


#endif

#ifndef _WIZNET_SPI_PIO_H_
#define _WIZNET_SPI_PIO_H_

#include "wizchip_spi.h"


int wiznet_spi_pio_open_ex(const wiznet_spi_config_t *spi_config,
                           wiznet_spi_handle_t *handle_out);
wiznet_spi_handle_t wiznet_spi_pio_open(const wiznet_spi_config_t *spi_config);
int wiznet_spi_pio_close_ex(wiznet_spi_handle_t handle);
int wiznet_spi_pio_sleep_ex(wiznet_spi_handle_t handle);
int wiznet_spi_pio_wake_ex(wiznet_spi_handle_t handle);
int wiznet_spi_pio_recover(wiznet_spi_handle_t handle);
void wiznet_spi_pio_sync_initialize(void);
void wiznet_spi_pio_bus_lock(void);
void wiznet_spi_pio_bus_unlock(void);
void wiznet_spi_pio_cris_enter(void);
void wiznet_spi_pio_cris_exit(void);
int wiznet_spi_pio_get_last_error(wiznet_spi_handle_t handle);
void wiznet_spi_pio_clear_last_error(wiznet_spi_handle_t handle);
wiznet_spi_lifecycle_state_t wiznet_spi_pio_get_state(
    wiznet_spi_handle_t handle);
#endif
