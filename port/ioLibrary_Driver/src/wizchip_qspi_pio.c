/*
    Copyright (c) 2023 Raspberry Pi (Trading) Ltd.

    SPDX-License-Identifier: BSD-3-Clause
*/

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/error.h"
#include "pico/critical_section.h"
#include "pico/mutex.h"

#include "hardware/dma.h"
#include "hardware/clocks.h"

#include "wizchip_conf.h"
#include "wizchip_qspi_pio.h"

#include "wizchip_qspi_pio.pio.h"

#ifndef PIO_SPI_PREFERRED_PIO
#define PIO_SPI_PREFERRED_PIO 1
#endif

#define PADS_DRIVE_STRENGTH PADS_BANK0_GPIO0_DRIVE_VALUE_12MA
#define IRQ_SAMPLE_DELAY_NS 100

#if (_WIZCHIP_ == W6300)
#if (_WIZCHIP_QSPI_MODE_ == QSPI_SINGLE_MODE)
#define PIO_PROGRAM_NAME wizchip_pio_spi_single_write_read
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_DUAL_MODE)
#define PIO_PROGRAM_NAME wizchip_pio_spi_dual_write_read
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_QUAD_MODE)
#define PIO_PROGRAM_NAME wizchip_pio_spi_quad_write_read
#endif
#endif

#if   (_WIZCHIP_ == W6300)
#define PIO_PROGRAM_FUNC __CONCAT(PIO_PROGRAM_NAME, _program)
#define PIO_PROGRAM_GET_DEFAULT_CONFIG_FUNC __CONCAT(PIO_PROGRAM_NAME, _program_get_default_config)
#define PIO_OFFSET_WRITE_BITS __CONCAT(PIO_PROGRAM_NAME, _offset_write_bits)
#define PIO_OFFSET_WRITE_BITS_END __CONCAT(PIO_PROGRAM_NAME, _offset_write_bits_end)
#define PIO_OFFSET_READ_BITS_END __CONCAT(PIO_PROGRAM_NAME, _offset_read_bits_end)

#else
#define PIO_PROGRAM_NAME wiznet_spi_write_read
#define PIO_PROGRAM_FUNC __CONCAT(PIO_PROGRAM_NAME, _program)
#define PIO_PROGRAM_GET_DEFAULT_CONFIG_FUNC __CONCAT(PIO_PROGRAM_NAME, _program_get_default_config)
#define PIO_OFFSET_WRITE_BITS __CONCAT(PIO_PROGRAM_NAME, _offset_write_bits)
#define PIO_OFFSET_WRITE_BITS_END __CONCAT(PIO_PROGRAM_NAME, _offset_write_end)
#define PIO_OFFSET_READ_BITS_END __CONCAT(PIO_PROGRAM_NAME, _offset_read_end)
// All wiznet spi operations must start with writing a 3 byte header

#endif

#ifndef PICO_WIZNET_SPI_PIO_INSTANCE_COUNT
#define PICO_WIZNET_SPI_PIO_INSTANCE_COUNT 1
#endif

#define SPI_HEADER_LEN 3
#define RP2040_GPIO_COUNT 30u
#define DEFAULT_TRANSFER_TIMEOUT_US 100000u
#define DEFAULT_ABORT_TIMEOUT_US 1000u

typedef struct spi_pio_ownership {
    bool pio_program_and_sm;
    bool dma_out;
    bool dma_in;
    bool dma_out_quarantined;
    bool dma_in_quarantined;
} spi_pio_ownership_t;

typedef struct spi_pio_state {
    wiznet_spi_funcs_t *funcs;
    wiznet_spi_config_t config;
    int last_error;
    const wiznet_spi_config_t *spi_config;
    wiznet_spi_lifecycle_state_t lifecycle;
    spi_pio_ownership_t ownership;
    pio_hw_t *pio;
    uint8_t pio_func_sel;
    int8_t pio_offset;
    int8_t pio_sm;
    int8_t dma_out;
    int8_t dma_in;
    uint8_t spi_header[SPI_HEADER_LEN];
    uint8_t spi_header_count;
} spi_pio_state_t;



static spi_pio_state_t spi_pio_state[PICO_WIZNET_SPI_PIO_INSTANCE_COUNT];
static spi_pio_state_t *active_state;
static mutex_t spi_bus_mutex;
static critical_section_t state_critical_section;
static critical_section_t spi_cris_cs;
static bool sync_initialized;

static void wiznet_spi_pio_close(wiznet_spi_handle_t handle);
static wiznet_spi_funcs_t *get_wiznet_spi_pio_impl(void);
static void cs_set(spi_pio_state_t *state, bool value);

void wiznet_spi_pio_sync_initialize(void) {
    if (sync_initialized) {
        return;
    }

    mutex_init(&spi_bus_mutex);
    critical_section_init(&state_critical_section);
    critical_section_init(&spi_cris_cs);
    sync_initialized = true;
}

void wiznet_spi_pio_bus_lock(void) {
    mutex_enter_blocking(&spi_bus_mutex);
}

void wiznet_spi_pio_bus_unlock(void) {
    mutex_exit(&spi_bus_mutex);
}

void wiznet_spi_pio_cris_enter(void) {
    critical_section_enter_blocking(&spi_cris_cs);
}

void wiznet_spi_pio_cris_exit(void) {
    critical_section_exit(&spi_cris_cs);
}

static void set_lifecycle(spi_pio_state_t *state,
                          wiznet_spi_lifecycle_state_t lifecycle) {
    critical_section_enter_blocking(&state_critical_section);
    state->lifecycle = lifecycle;
    critical_section_exit(&state_critical_section);
}

static bool dma_channel_abort_bounded(spi_pio_state_t *state, int8_t channel,
                                      bool *quarantined) {
    const uint32_t channel_mask = 1u << (uint)channel;

    if (!dma_channel_is_busy((uint)channel)) {
        /* Idle channel: skip the bounded abort, but still acknowledge a
           stale completion flag exactly as the full abort path would. */
        if ((dma_hw->intr & channel_mask) != 0u) {
            dma_channel_acknowledge_irq0((uint)channel);
        }
        *quarantined = false;
        return true;
    }

    const bool irq0_was_enabled = (dma_hw->inte0 & channel_mask) != 0u;
    const bool irq0_was_asserted = (dma_hw->ints0 & channel_mask) != 0u;
    const uint64_t abort_started_us = time_us_64();

    dma_channel_set_irq0_enabled((uint)channel, false);
    dma_hw->abort = channel_mask;

    while (dma_channel_is_busy((uint)channel) &&
           time_us_64() - abort_started_us < state->config.abort_timeout_us) {
        busy_wait_us_32(1u);
    }

    if (irq0_was_asserted || (dma_hw->intr & channel_mask) != 0u) {
        dma_channel_acknowledge_irq0((uint)channel);
    }
    dma_channel_set_irq0_enabled((uint)channel, irq0_was_enabled);

    *quarantined = dma_channel_is_busy((uint)channel);
    return !*quarantined;
}

static bool prepare_dma_channel(spi_pio_state_t *state, int8_t channel,
                                 bool *quarantined) {
    if (*quarantined ||
        !dma_channel_abort_bounded(state, channel, quarantined)) {
        state->lifecycle = WIZNET_SPI_FAULTED;
        return false;
    }
    return true;
}

static bool dma_channel_has_error(int8_t channel) {
    const uint32_t channel_mask = 1u << (uint)channel;

    return (dma_hw->ints0 & channel_mask) != 0u ||
           (dma_channel_hw_addr((uint)channel)->ctrl_trig &
            DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS) != 0u;
}

static bool pio_fifos_drained(const spi_pio_state_t *state) {
    return pio_sm_is_tx_fifo_empty(state->pio, (uint)state->pio_sm) &&
           pio_sm_is_rx_fifo_empty(state->pio, (uint)state->pio_sm);
}

static void latch_error(spi_pio_state_t *state, int error) {
    if (state != NULL && state->last_error == PICO_OK) {
        state->last_error = error;
    }
}

static void record_transfer_failure(spi_pio_state_t *state, int error,
                                    uint8_t *rx, size_t rx_length) {
    pio_sm_set_enabled(state->pio, (uint)state->pio_sm, false);

    if (state->ownership.dma_in) {
        dma_channel_abort_bounded(
            state, state->dma_in,
            &state->ownership.dma_in_quarantined);
    }
    if (state->ownership.dma_out) {
        dma_channel_abort_bounded(
            state, state->dma_out,
            &state->ownership.dma_out_quarantined);
    }

    pio_sm_clear_fifos(state->pio, (uint)state->pio_sm);
    if (rx != NULL && rx_length != 0u) {
        memset(rx, 0, rx_length);
    }
    state->spi_header_count = 0u;
    cs_set(state, true);
    latch_error(state, error);
    set_lifecycle(state, WIZNET_SPI_FAULTED);
}

static int wait_for_transfer(spi_pio_state_t *state, bool wait_for_dma_in) {
    const uint64_t started_us = time_us_64();

    while (dma_channel_is_busy((uint)state->dma_out) ||
           (wait_for_dma_in &&
            dma_channel_is_busy((uint)state->dma_in)) ||
           !pio_fifos_drained(state)) {
        if (time_us_64() - started_us >=
            state->config.transfer_timeout_us) {
            return PICO_ERROR_TIMEOUT;
        }
        busy_wait_us_32(1u);
    }

    if (time_us_64() - started_us >= state->config.transfer_timeout_us) {
        return PICO_ERROR_TIMEOUT;
    }

    if (dma_channel_has_error(state->dma_out) ||
        (wait_for_dma_in && dma_channel_has_error(state->dma_in))) {
        return PICO_ERROR_IO;
    }

    return PICO_OK;
}

static bool release_owned_resources(spi_pio_state_t *state) {
    if (state->ownership.pio_program_and_sm) {
        pio_sm_set_enabled(state->pio, state->pio_sm, false);
    }

    if (state->ownership.dma_in) {
        if (dma_channel_abort_bounded(
                state, state->dma_in,
                &state->ownership.dma_in_quarantined)) {
            dma_channel_unclaim(state->dma_in);
            state->ownership.dma_in = false;
            state->dma_in = -1;
        }
    }
    if (state->ownership.dma_out) {
        if (dma_channel_abort_bounded(
                state, state->dma_out,
                &state->ownership.dma_out_quarantined)) {
            dma_channel_unclaim(state->dma_out);
            state->ownership.dma_out = false;
            state->dma_out = -1;
        }
    }
    if (state->ownership.pio_program_and_sm && !state->ownership.dma_in &&
        !state->ownership.dma_out) {
        pio_remove_program_and_unclaim_sm(&PIO_PROGRAM_FUNC, state->pio,
                                          (uint)state->pio_sm,
                                          (uint)state->pio_offset);
        state->ownership.pio_program_and_sm = false;
        state->pio = NULL;
        state->pio_sm = -1;
        state->pio_offset = -1;
    }

    return !state->ownership.dma_in && !state->ownership.dma_out;
}

static bool reset_unpublished_state(spi_pio_state_t *state) {
    if (!release_owned_resources(state)) {
        state->lifecycle = WIZNET_SPI_FAULTED;
        return false;
    }

    state->funcs = NULL;
    state->spi_config = NULL;
    state->pio = NULL;
    state->pio_sm = -1;
    state->pio_offset = -1;
    state->dma_out = -1;
    state->dma_in = -1;
    memset(&state->ownership, 0, sizeof(state->ownership));
    memset(&state->config, 0, sizeof(state->config));
    state->spi_header_count = 0;
    state->last_error = PICO_OK;
    set_lifecycle(state, WIZNET_SPI_UNINIT);
    return true;
}

static bool gpio_is_valid(uint8_t pin) {
    return pin < RP2040_GPIO_COUNT;
}

static int wiznet_spi_config_copy_normalize(
    const wiznet_spi_config_t *input,
    wiznet_spi_config_t *output) {
    if (input == NULL || output == NULL) {
        return PICO_ERROR_INVALID_ARG;
    }

#if (_WIZCHIP_ == W6300)
    if (!gpio_is_valid(input->clock_pin) ||
        !gpio_is_valid(input->data_io0_pin) ||
        !gpio_is_valid(input->data_io1_pin) ||
        !gpio_is_valid(input->data_io2_pin) ||
        !gpio_is_valid(input->data_io3_pin) ||
        !gpio_is_valid(input->cs_pin) ||
        !gpio_is_valid(input->reset_pin) ||
        !gpio_is_valid(input->irq_pin) ||
        (input->clock_div_major == 0u && input->clock_div_minor == 0u)) {
        return PICO_ERROR_INVALID_ARG;
    }
#else
    if (!gpio_is_valid(input->data_in_pin) ||
        !gpio_is_valid(input->data_out_pin) ||
        !gpio_is_valid(input->cs_pin) ||
        !gpio_is_valid(input->clock_pin) ||
        !gpio_is_valid(input->irq_pin) ||
        !gpio_is_valid(input->reset_pin) || input->spi_hw_instance > 1u ||
        (input->clock_div_major == 0u && input->clock_div_minor == 0u)) {
        return PICO_ERROR_INVALID_ARG;
    }
#endif

    *output = *input;
    if (output->transfer_timeout_us == 0u) {
        output->transfer_timeout_us = DEFAULT_TRANSFER_TIMEOUT_US;
    }
    if (output->abort_timeout_us == 0u) {
        output->abort_timeout_us = DEFAULT_ABORT_TIMEOUT_US;
    }

    return PICO_OK;
}


static uint16_t mk_cmd_buf(uint8_t *pdst, uint8_t opcode, uint16_t addr) {
#if (_WIZCHIP_QSPI_MODE_ == QSPI_SINGLE_MODE)

    pdst[0] = opcode;
    pdst[1] = (uint8_t)((addr >> 8) & 0xFF);
    pdst[2] = (uint8_t)((addr >> 0) & 0xFF);
    pdst[3] = 0;

    return 3 + 1;
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_DUAL_MODE)
    pdst[0] = ((opcode >> 7 & 0x01) << 6) | ((opcode >> 6 & 0x01) << 4) | ((opcode >> 5 & 0x01) << 2) | ((opcode >> 4 & 0x01) << 0);
    pdst[1] = ((opcode >> 3 & 0x01) << 6) | ((opcode >> 2 & 0x01) << 4) | ((opcode >> 1 & 0x01) << 2) | ((opcode >> 0 & 0x01) << 0);
    pdst[2] = (uint8_t)((addr >> 8) & 0xFF);
    pdst[3] = (uint8_t)((addr >> 0) & 0xFF);

    pdst[4] = 0;

    return 4 + 1;
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_QUAD_MODE)
    pdst[0] = ((opcode >> 7 & 0x01) << 4) | ((opcode >> 6 & 0x01) << 0);
    pdst[1] = ((opcode >> 5 & 0x01) << 4) | ((opcode >> 4 & 0x01) << 0);
    pdst[2] = ((opcode >> 3 & 0x01) << 4) | ((opcode >> 2 & 0x01) << 0);
    pdst[3] = ((opcode >> 1 & 0x01) << 4) | ((opcode >> 0 & 0x01) << 0);

    pdst[4] = ((uint8_t)(addr >> 8) & 0xFF);
    pdst[5] = ((uint8_t)(addr >> 0) & 0xFF);

    pdst[6] = 0;

    return 6 + 1;
#endif
    return 0;
}

// Initialise our gpios
static void pio_spi_gpio_setup(spi_pio_state_t *state) {

#if   (_WIZCHIP_ == W6300)
#if (_WIZCHIP_QSPI_MODE_ == QSPI_SINGLE_MODE)
    // Setup DO and DI
    gpio_init(state->spi_config->data_io0_pin);
    gpio_init(state->spi_config->data_io1_pin);
    gpio_set_dir(state->spi_config->data_io0_pin, GPIO_OUT);
    gpio_set_dir(state->spi_config->data_io1_pin, GPIO_OUT);
    gpio_put(state->spi_config->data_io0_pin, false);
    gpio_put(state->spi_config->data_io1_pin, false);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_DUAL_MODE)
    // Setup DO and DI
    gpio_init(state->spi_config->data_io0_pin);
    gpio_init(state->spi_config->data_io1_pin);
    gpio_set_dir(state->spi_config->data_io0_pin, GPIO_OUT);
    gpio_set_dir(state->spi_config->data_io1_pin, GPIO_OUT);
    gpio_put(state->spi_config->data_io0_pin, false);
    gpio_put(state->spi_config->data_io1_pin, false);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_QUAD_MODE)
    // Setup DO and DI
    gpio_init(state->spi_config->data_io0_pin);
    gpio_init(state->spi_config->data_io1_pin);
    gpio_init(state->spi_config->data_io2_pin);
    gpio_init(state->spi_config->data_io3_pin);
    gpio_set_dir(state->spi_config->data_io0_pin, GPIO_OUT);
    gpio_set_dir(state->spi_config->data_io1_pin, GPIO_OUT);
    gpio_set_dir(state->spi_config->data_io2_pin, GPIO_OUT);
    gpio_set_dir(state->spi_config->data_io3_pin, GPIO_OUT);
    gpio_put(state->spi_config->data_io0_pin, false);
    gpio_put(state->spi_config->data_io1_pin, false);
    gpio_put(state->spi_config->data_io2_pin, false);
    gpio_put(state->spi_config->data_io3_pin, false);
#endif

    // Setup CS
    gpio_init(state->spi_config->cs_pin);
    gpio_set_dir(state->spi_config->cs_pin, GPIO_OUT);
    gpio_put(state->spi_config->cs_pin, true);

    // Setup reset
    gpio_init(state->spi_config->irq_pin);
    gpio_set_dir(state->spi_config->irq_pin, GPIO_IN);
    gpio_set_pulls(state->spi_config->irq_pin, true, false);
    gpio_set_input_hysteresis_enabled(state->spi_config->irq_pin, true);
#else //W55RP20
    // Setup MOSI, MISO and IRQ
    gpio_init(state->spi_config->data_out_pin);
    gpio_set_dir(state->spi_config->data_out_pin, GPIO_OUT);
    gpio_put(state->spi_config->data_out_pin, false);

    // Setup CS
    gpio_init(state->spi_config->cs_pin);
    gpio_set_dir(state->spi_config->cs_pin, GPIO_OUT);
    gpio_put(state->spi_config->cs_pin, true);

    // Setup IRQ
    gpio_init(state->spi_config->irq_pin);
    gpio_set_dir(state->spi_config->irq_pin, GPIO_IN);
    gpio_set_pulls(state->spi_config->irq_pin, true, false);
    gpio_set_input_hysteresis_enabled(state->spi_config->irq_pin, true);
#endif

}

static int wiznet_spi_pio_open_normalized(
    const wiznet_spi_config_t *spi_config,
    wiznet_spi_handle_t *handle_out) {

    spi_pio_state_t *state = NULL;
    critical_section_enter_blocking(&state_critical_section);
    for (size_t i = 0; i < count_of(spi_pio_state); i++) {
        if (spi_pio_state[i].lifecycle == WIZNET_SPI_UNINIT) {
            state = &spi_pio_state[i];
            state->lifecycle = WIZNET_SPI_OPENING;
            break;
        }
    }
    critical_section_exit(&state_critical_section);
    if (!state) {
        return PICO_ERROR_RESOURCE_IN_USE;
    }

    state->config = *spi_config;
    state->spi_config = &state->config;
    state->funcs = NULL;
    state->last_error = PICO_OK;
    state->pio = NULL;
    state->pio_offset = -1;
    state->pio_sm = -1;
    state->dma_in = -1;
    state->dma_out = -1;
    memset(&state->ownership, 0, sizeof(state->ownership));

    uint pio_sm;
    uint pio_offset;
    if (!pio_claim_free_sm_and_add_program(&PIO_PROGRAM_FUNC, &state->pio,
                                            &pio_sm, &pio_offset)) {
        reset_unpublished_state(state);
        return PICO_ERROR_INSUFFICIENT_RESOURCES;
    }
    state->pio_sm = (int8_t)pio_sm;
    state->pio_offset = (int8_t)pio_offset;
    state->ownership.pio_program_and_sm = true;

    state->dma_out = (int8_t)dma_claim_unused_channel(false);
    if (state->dma_out < 0) {
        reset_unpublished_state(state);
        return PICO_ERROR_INSUFFICIENT_RESOURCES;
    }
    state->ownership.dma_out = true;

    state->dma_in = (int8_t)dma_claim_unused_channel(false);
    if (state->dma_in < 0) {
        reset_unpublished_state(state);
        return PICO_ERROR_INSUFFICIENT_RESOURCES;
    }
    state->ownership.dma_in = true;

    pio_spi_gpio_setup(state);

    static_assert(GPIO_FUNC_PIO1 == GPIO_FUNC_PIO0 + 1, "");
    uint pio_index = state->pio == pio1 ? 1u : 0u;
    state->pio_func_sel = GPIO_FUNC_PIO0 + pio_index;


#ifdef WIZCHIP_PIO_TRACE
    uint64_t f_sys = clock_get_hz(clk_sys); // Hz
#if (_WIZCHIP_ == W6300)
    const char *wizchip_pio_clock_str = "PIO QSPI CLOCK SPEED";
#else
    const char *wizchip_pio_clock_str = "PIO SPI CLOCK SPEED";
#endif

    printf("[%s : %.2f MHz] (sys=%.2f MHz)\r\n\r\n",
           wizchip_pio_clock_str,
           (double)f_sys / (2.0 * (state->spi_config->clock_div_major +
                                   state->spi_config->clock_div_minor / 256.0)) / 1e6,
           f_sys / 1e6);
#endif

    pio_sm_config sm_config = PIO_PROGRAM_GET_DEFAULT_CONFIG_FUNC(state->pio_offset);

    sm_config_set_clkdiv_int_frac(&sm_config, state->spi_config->clock_div_major, state->spi_config->clock_div_minor);
    hw_write_masked(&pads_bank0_hw->io[state->spi_config->clock_pin],
                    (uint)PADS_DRIVE_STRENGTH << PADS_BANK0_GPIO0_DRIVE_LSB,
                    PADS_BANK0_GPIO0_DRIVE_BITS
                   );
    hw_write_masked(&pads_bank0_hw->io[state->spi_config->clock_pin],
                    (uint)1 << PADS_BANK0_GPIO0_SLEWFAST_LSB,
                    PADS_BANK0_GPIO0_SLEWFAST_BITS
                   );

#if   (_WIZCHIP_ == W6300)
#if (_WIZCHIP_QSPI_MODE_ == QSPI_SINGLE_MODE)
#ifdef WIZCHIP_PIO_TRACE
    printf("\r\n[QSPI SINGLE MODE]\r\n");
#endif
    sm_config_set_out_pins(&sm_config, state->spi_config->data_io0_pin, 1);
    sm_config_set_in_pins(&sm_config, state->spi_config->data_io1_pin);
    sm_config_set_set_pins(&sm_config, state->spi_config->data_io0_pin, 2);
    sm_config_set_sideset(&sm_config, 1, false, false);
    sm_config_set_sideset_pins(&sm_config, state->spi_config->clock_pin);

    sm_config_set_in_shift(&sm_config, false, true, 8);
    sm_config_set_out_shift(&sm_config, false, true, 8);

    hw_set_bits(&state->pio->input_sync_bypass,
                (1u << state->spi_config->data_io0_pin) | (1u << state->spi_config->data_io1_pin));
    pio_sm_set_config(state->pio, state->pio_sm, &sm_config);
    pio_sm_set_consecutive_pindirs(state->pio, state->pio_sm, state->spi_config->clock_pin, 1, true);

    gpio_set_function(state->spi_config->data_io0_pin, state->pio_func_sel);

    // Set data pin to pull down and schmitt
    gpio_set_pulls(state->spi_config->data_io0_pin, false, true);
    gpio_set_pulls(state->spi_config->data_io1_pin, false, true);
    gpio_set_input_hysteresis_enabled(state->spi_config->data_io0_pin, true);
    gpio_set_input_hysteresis_enabled(state->spi_config->data_io1_pin, true);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_DUAL_MODE)
#ifdef WIZCHIP_PIO_TRACE
    printf("[QSPI DUAL MODE]\r\n\r\n");
#endif
    sm_config_set_out_pins(&sm_config, state->spi_config->data_io0_pin, 2);
    sm_config_set_in_pins(&sm_config, state->spi_config->data_io0_pin);
    sm_config_set_set_pins(&sm_config, state->spi_config->data_io0_pin, 2);
    sm_config_set_sideset(&sm_config, 1, false, false);
    sm_config_set_sideset_pins(&sm_config, state->spi_config->clock_pin);

    sm_config_set_in_shift(&sm_config, false, true, 8);
    sm_config_set_out_shift(&sm_config, false, true, 8);

    hw_set_bits(&state->pio->input_sync_bypass,
                (1u << state->spi_config->data_io0_pin) | (1u << state->spi_config->data_io1_pin));
    pio_sm_set_config(state->pio, state->pio_sm, &sm_config);
    pio_sm_set_consecutive_pindirs(state->pio, state->pio_sm, state->spi_config->clock_pin, 1, true);

    gpio_set_function(state->spi_config->data_io0_pin, state->pio_func_sel);
    gpio_set_function(state->spi_config->data_io1_pin, state->pio_func_sel);

    // Set data pin to pull down and schmitt
    gpio_set_pulls(state->spi_config->data_io0_pin, false, true);
    gpio_set_pulls(state->spi_config->data_io1_pin, false, true);
    gpio_set_input_hysteresis_enabled(state->spi_config->data_io0_pin, true);
    gpio_set_input_hysteresis_enabled(state->spi_config->data_io1_pin, true);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_QUAD_MODE)
#ifdef WIZCHIP_PIO_TRACE
    printf("\r\n[QSPI QUAD MODE]\r\n");
#endif
    sm_config_set_out_pins(&sm_config, state->spi_config->data_io0_pin, 4);
    sm_config_set_in_pins(&sm_config, state->spi_config->data_io0_pin);
    sm_config_set_set_pins(&sm_config, state->spi_config->data_io0_pin, 4);
    sm_config_set_sideset(&sm_config, 1, false, false);
    sm_config_set_sideset_pins(&sm_config, state->spi_config->clock_pin);

    sm_config_set_in_shift(&sm_config, false, true, 8);
    sm_config_set_out_shift(&sm_config, false, true, 8);

    hw_set_bits(&state->pio->input_sync_bypass,
                (1u << state->spi_config->data_io0_pin) | (1u << state->spi_config->data_io1_pin) | (1u << state->spi_config->data_io2_pin) | (1u << state->spi_config->data_io3_pin));
    pio_sm_set_config(state->pio, state->pio_sm, &sm_config);
    pio_sm_set_consecutive_pindirs(state->pio, state->pio_sm, state->spi_config->clock_pin, 1, true);

    gpio_set_function(state->spi_config->data_io0_pin, state->pio_func_sel);
    gpio_set_function(state->spi_config->data_io1_pin, state->pio_func_sel);
    gpio_set_function(state->spi_config->data_io2_pin, state->pio_func_sel);
    gpio_set_function(state->spi_config->data_io3_pin, state->pio_func_sel);

    // Set data pin to pull down and schmitt
    gpio_set_pulls(state->spi_config->data_io0_pin, false, true);
    gpio_set_pulls(state->spi_config->data_io1_pin, false, true);
    gpio_set_pulls(state->spi_config->data_io2_pin, false, true);
    gpio_set_pulls(state->spi_config->data_io3_pin, false, true);
    gpio_set_input_hysteresis_enabled(state->spi_config->data_io0_pin, true);
    gpio_set_input_hysteresis_enabled(state->spi_config->data_io1_pin, true);
    gpio_set_input_hysteresis_enabled(state->spi_config->data_io2_pin, true);
    gpio_set_input_hysteresis_enabled(state->spi_config->data_io3_pin, true);
    /* @todo: Implement to use. */
#endif
#else //W55RP20
    sm_config_set_out_pins(&sm_config, state->spi_config->data_out_pin, 1);
    sm_config_set_in_pins(&sm_config, state->spi_config->data_in_pin);
    sm_config_set_set_pins(&sm_config, state->spi_config->data_out_pin, 1);
    sm_config_set_sideset(&sm_config, 1, false, false);
    sm_config_set_sideset_pins(&sm_config, state->spi_config->clock_pin);

    sm_config_set_in_shift(&sm_config, false, true, 8);
    sm_config_set_out_shift(&sm_config, false, true, 8);
    hw_set_bits(&state->pio->input_sync_bypass, 1u << state->spi_config->data_in_pin);
    pio_sm_set_config(state->pio, state->pio_sm, &sm_config);
    pio_sm_set_consecutive_pindirs(state->pio, state->pio_sm, state->spi_config->clock_pin, 1, true);
    gpio_set_function(state->spi_config->data_out_pin, state->pio_func_sel);
    gpio_set_function(state->spi_config->clock_pin, state->pio_func_sel);

    // Set data pin to pull down and schmitt
    gpio_set_pulls(state->spi_config->data_in_pin, false, true);
    gpio_set_input_hysteresis_enabled(state->spi_config->data_in_pin, true);
#endif

    pio_sm_exec(state->pio, state->pio_sm, pio_encode_set(pio_pins, 1));

    critical_section_enter_blocking(&state_critical_section);
    state->funcs = get_wiznet_spi_pio_impl();
    state->lifecycle = WIZNET_SPI_READY;
    *handle_out = &state->funcs;
    critical_section_exit(&state_critical_section);
    return PICO_OK;

}

int wiznet_spi_pio_open_ex(const wiznet_spi_config_t *spi_config,
                           wiznet_spi_handle_t *handle_out) {
    wiznet_spi_config_t normalized_config;

    wiznet_spi_pio_sync_initialize();

    if (handle_out == NULL) {
        return PICO_ERROR_INVALID_ARG;
    }

    const int result = wiznet_spi_config_copy_normalize(
        spi_config, &normalized_config);
    *handle_out = NULL;
    if (result != PICO_OK) {
        return result;
    }

    return wiznet_spi_pio_open_normalized(&normalized_config, handle_out);
}

wiznet_spi_handle_t wiznet_spi_pio_open(
    const wiznet_spi_config_t *spi_config) {
    wiznet_spi_handle_t handle = NULL;

    return wiznet_spi_pio_open_ex(spi_config, &handle) == PICO_OK ? handle
                                                                  : NULL;
}

static int wiznet_spi_pio_close_locked(spi_pio_state_t *state) {
    if (state->lifecycle == WIZNET_SPI_UNINIT) {
        return PICO_OK;
    }

    set_lifecycle(state, WIZNET_SPI_CLOSING);
    critical_section_enter_blocking(&state_critical_section);
    if (active_state == state) {
        active_state = NULL;
    }
    critical_section_exit(&state_critical_section);

    if (!reset_unpublished_state(state)) {
        latch_error(state, PICO_ERROR_TIMEOUT);
        return PICO_ERROR_TIMEOUT;
    }

    return PICO_OK;
}

int wiznet_spi_pio_close_ex(wiznet_spi_handle_t handle) {
    spi_pio_state_t *state = (spi_pio_state_t *)handle;
    int result;

    if (state == NULL) {
        return PICO_ERROR_INVALID_ARG;
    }

    wiznet_spi_pio_sync_initialize();
    mutex_enter_blocking(&spi_bus_mutex);
    result = wiznet_spi_pio_close_locked(state);
    mutex_exit(&spi_bus_mutex);
    return result;
}

static void wiznet_spi_pio_close(wiznet_spi_handle_t handle) {
    (void)wiznet_spi_pio_close_ex(handle);
}

static void cs_set(spi_pio_state_t *state, bool value) {
    gpio_put(state->spi_config->cs_pin, value);
}

static void ns_delay(uint32_t ns) {
    // cycles = ns * clk_sys_hz / 1,000,000,000
    uint32_t cycles = ns * (clock_get_hz(clk_sys) >> 16u) / (1000000000u >> 16u);
    busy_wait_at_least_cycles(cycles);
}

static void wiznet_spi_pio_frame_start(void) {
    if (!active_state) return;
#if   (_WIZCHIP_ == W6300)
#if (_WIZCHIP_QSPI_MODE_ == QSPI_SINGLE_MODE)
    gpio_set_function(active_state->spi_config->data_io0_pin, active_state->pio_func_sel);
    gpio_set_function(active_state->spi_config->data_io1_pin, active_state->pio_func_sel);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_DUAL_MODE)
    gpio_set_function(active_state->spi_config->data_io0_pin, active_state->pio_func_sel);
    gpio_set_function(active_state->spi_config->data_io1_pin, active_state->pio_func_sel);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_QUAD_MODE)
    gpio_set_function(active_state->spi_config->data_io0_pin, active_state->pio_func_sel);
    gpio_set_function(active_state->spi_config->data_io1_pin, active_state->pio_func_sel);
    gpio_set_function(active_state->spi_config->data_io2_pin, active_state->pio_func_sel);
    gpio_set_function(active_state->spi_config->data_io3_pin, active_state->pio_func_sel);
    /* @todo: Implement to use. */
#endif
    gpio_set_function(active_state->spi_config->clock_pin, active_state->pio_func_sel);
    gpio_pull_down(active_state->spi_config->clock_pin);
#else
    gpio_set_function(active_state->spi_config->data_out_pin, active_state->pio_func_sel);
    gpio_set_function(active_state->spi_config->clock_pin, active_state->pio_func_sel);
    gpio_pull_down(active_state->spi_config->clock_pin);
#endif
    // Pull CS low
    cs_set(active_state, false);
}

static void wiznet_spi_pio_frame_end(void) {
    if (!active_state) return;
    cs_set(active_state, true);
    // we need to wait a bit in case the irq line is incorrectly high
#ifdef IRQ_SAMPLE_DELAY_NS
    ns_delay(IRQ_SAMPLE_DELAY_NS);
#endif
}

#if   (_WIZCHIP_ == W6300)
// To read a byte we must first have been asked to write a 3 byte spi header
void wiznet_spi_pio_read_byte(uint8_t op_code, uint16_t AddrSel, uint8_t *rx, uint16_t rx_length) {
    uint8_t command_buf[8] = {0,};
    uint16_t command_len = mk_cmd_buf(command_buf, op_code, AddrSel);
    uint32_t loop_cnt = 0;

    pio_sm_set_enabled(active_state->pio, active_state->pio_sm, false);
    pio_sm_set_wrap(active_state->pio, active_state->pio_sm, active_state->pio_offset, active_state->pio_offset + PIO_OFFSET_READ_BITS_END - 1);
    //pio_sm_set_wrap(active_state->pio, active_state->pio_sm, active_state->pio_offset, active_state->pio_offset + PIO_OFFSET_READ_BITS_END - 1);
    pio_sm_clear_fifos(active_state->pio, active_state->pio_sm);

#if (_WIZCHIP_QSPI_MODE_ == QSPI_SINGLE_MODE)
    loop_cnt = 8;
    pio_sm_set_pindirs_with_mask(active_state->pio,
                                 active_state->pio_sm,
                                 (1u << active_state->spi_config->data_io0_pin), (1u << active_state->spi_config->data_io0_pin));// | (1u << active_state->spi_config->data_io1_pin));
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_DUAL_MODE)
    loop_cnt = 4;
    pio_sm_set_pindirs_with_mask(active_state->pio,
                                 active_state->pio_sm,
                                 (1u << active_state->spi_config->data_io0_pin) | (1u << active_state->spi_config->data_io1_pin),
                                 (1u << active_state->spi_config->data_io0_pin) | (1u << active_state->spi_config->data_io1_pin));
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_QUAD_MODE)
    loop_cnt = 2;
    pio_sm_set_pindirs_with_mask(active_state->pio,
                                 active_state->pio_sm,
                                 (1u << active_state->spi_config->data_io0_pin) | (1u << active_state->spi_config->data_io1_pin) | (1u << active_state->spi_config->data_io2_pin) | (1u << active_state->spi_config->data_io3_pin),
                                 (1u << active_state->spi_config->data_io0_pin) | (1u << active_state->spi_config->data_io1_pin) | (1u << active_state->spi_config->data_io2_pin) | (1u << active_state->spi_config->data_io3_pin));

    /* @todo: Implement to use. */
#endif

    pio_sm_restart(active_state->pio, active_state->pio_sm);
    pio_sm_clkdiv_restart(active_state->pio, active_state->pio_sm);

    pio_sm_put(active_state->pio, active_state->pio_sm, command_len * loop_cnt - 1);
    pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_out(pio_x, 32));

    pio_sm_put(active_state->pio, active_state->pio_sm, rx_length - 1);
    pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_out(pio_y, 32));

    pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_jmp(active_state->pio_offset));

    if (!prepare_dma_channel(active_state, active_state->dma_out,
                             &active_state->ownership.dma_out_quarantined) ||
        !prepare_dma_channel(active_state, active_state->dma_in,
                             &active_state->ownership.dma_in_quarantined)) {
        return;
    }

    dma_channel_config out_config = dma_channel_get_default_config(active_state->dma_out);
    channel_config_set_transfer_data_size(&out_config, DMA_SIZE_8);
    channel_config_set_bswap(&out_config, true);
    channel_config_set_dreq(&out_config, pio_get_dreq(active_state->pio, active_state->pio_sm, true));
    dma_channel_configure(active_state->dma_out, &out_config, &active_state->pio->txf[active_state->pio_sm], command_buf, command_len, true);

    dma_channel_config in_config = dma_channel_get_default_config(active_state->dma_in);
    channel_config_set_transfer_data_size(&in_config, DMA_SIZE_8);
    channel_config_set_bswap(&in_config, true);
    channel_config_set_dreq(&in_config, pio_get_dreq(active_state->pio, active_state->pio_sm, false));
    channel_config_set_write_increment(&in_config, true);
    channel_config_set_read_increment(&in_config, false);
    dma_channel_configure(active_state->dma_in, &in_config, rx, &active_state->pio->rxf[active_state->pio_sm], rx_length, true);

    pio_sm_set_enabled(active_state->pio, active_state->pio_sm, true);

    __compiler_memory_barrier();

    dma_channel_wait_for_finish_blocking(active_state->dma_out);
    dma_channel_wait_for_finish_blocking(active_state->dma_in);

    __compiler_memory_barrier();

    pio_sm_set_enabled(active_state->pio, active_state->pio_sm, false);
    pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_mov(pio_pins, pio_null));


}

void wiznet_spi_pio_write_byte(uint8_t op_code, uint16_t AddrSel, uint8_t *tx, uint16_t tx_length) {
    uint8_t command_buf[8] = {0,};
    uint16_t command_len = mk_cmd_buf(command_buf, op_code, AddrSel);
    uint32_t loop_cnt = 0;
    tx_length = tx_length + command_len;

    pio_sm_set_enabled(active_state->pio, active_state->pio_sm, false);
    pio_sm_set_wrap(active_state->pio, active_state->pio_sm, active_state->pio_offset, active_state->pio_offset + PIO_OFFSET_WRITE_BITS_END - 1);
    pio_sm_clear_fifos(active_state->pio, active_state->pio_sm);

#if (_WIZCHIP_QSPI_MODE_ == QSPI_SINGLE_MODE)
    loop_cnt = 8;
    pio_sm_set_pindirs_with_mask(active_state->pio,
                                 active_state->pio_sm,
                                 (1u << active_state->spi_config->data_io0_pin), (1u << active_state->spi_config->data_io0_pin));
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_DUAL_MODE)
    loop_cnt = 4;
    pio_sm_set_pindirs_with_mask(active_state->pio,
                                 active_state->pio_sm,
                                 (1u << active_state->spi_config->data_io0_pin) | (1u << active_state->spi_config->data_io1_pin),
                                 (1u << active_state->spi_config->data_io0_pin) | (1u << active_state->spi_config->data_io1_pin));
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_QUAD_MODE)
    loop_cnt = 2;
    pio_sm_set_pindirs_with_mask(active_state->pio,
                                 active_state->pio_sm,
                                 (1u << active_state->spi_config->data_io0_pin) | (1u << active_state->spi_config->data_io1_pin) | (1u << active_state->spi_config->data_io2_pin) | (1u << active_state->spi_config->data_io3_pin),
                                 (1u << active_state->spi_config->data_io0_pin) | (1u << active_state->spi_config->data_io1_pin) | (1u << active_state->spi_config->data_io2_pin) | (1u << active_state->spi_config->data_io3_pin));

#endif

    pio_sm_restart(active_state->pio, active_state->pio_sm);
    pio_sm_clkdiv_restart(active_state->pio, active_state->pio_sm);
    pio_sm_put(active_state->pio, active_state->pio_sm, tx_length * loop_cnt - 1);
    pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_out(pio_x, 32));
    pio_sm_put(active_state->pio, active_state->pio_sm, 0);
    pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_out(pio_y, 32));
    pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_jmp(active_state->pio_offset));
    if (!prepare_dma_channel(active_state, active_state->dma_out,
                             &active_state->ownership.dma_out_quarantined)) {
        return;
    }


    dma_channel_config out_config = dma_channel_get_default_config(active_state->dma_out);
    channel_config_set_transfer_data_size(&out_config, DMA_SIZE_8);
    channel_config_set_bswap(&out_config, true);
    channel_config_set_dreq(&out_config, pio_get_dreq(active_state->pio, active_state->pio_sm, true));

    pio_sm_set_enabled(active_state->pio, active_state->pio_sm, true);

    dma_channel_configure(active_state->dma_out, &out_config, &active_state->pio->txf[active_state->pio_sm], command_buf, command_len, true);
    dma_channel_wait_for_finish_blocking(active_state->dma_out);
    dma_channel_configure(active_state->dma_out, &out_config, &active_state->pio->txf[active_state->pio_sm], tx, tx_length - command_len, true);
    dma_channel_wait_for_finish_blocking(active_state->dma_out);

    const uint32_t fdebug_tx_stall = 1u << (PIO_FDEBUG_TXSTALL_LSB + active_state->pio_sm);
    active_state->pio->fdebug = fdebug_tx_stall;
    {
        uint32_t _poll = 0;
        while (!(active_state->pio->fdebug & fdebug_tx_stall)) {
            if (++_poll > 0xFFFFu) break;
        }
    }
    __compiler_memory_barrier();
#if (_WIZCHIP_QSPI_MODE_ == QSPI_SINGLE_MODE)
    pio_sm_set_consecutive_pindirs(active_state->pio, active_state->pio_sm, active_state->spi_config->data_io0_pin, 1, false);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_DUAL_MODE)
    pio_sm_set_consecutive_pindirs(active_state->pio, active_state->pio_sm, active_state->spi_config->data_io0_pin, 2, false);
#elif (_WIZCHIP_QSPI_MODE_ == QSPI_QUAD_MODE)
    pio_sm_set_consecutive_pindirs(active_state->pio, active_state->pio_sm, active_state->spi_config->data_io0_pin, 4, false);
#endif

    pio_sm_exec(active_state->pio, active_state->pio_sm, pio_encode_mov(pio_pins, pio_null));
    pio_sm_set_enabled(active_state->pio, active_state->pio_sm, false);
  }

#else
// send tx then receive rx
// rx can be null if you just want to send, but tx and tx_length must be valid
static int pio_spi_transfer(spi_pio_state_t *state, const uint8_t *tx,
                            size_t tx_length, uint8_t *rx,
                            size_t rx_length) {
    if (tx_length == 0u && rx_length == 0u) {
        return PICO_OK;
    }

    if (state == NULL || state->lifecycle != WIZNET_SPI_READY) {
        latch_error(state, PICO_ERROR_INVALID_STATE);
        return PICO_ERROR_INVALID_STATE;
    }
    if (tx == NULL || tx_length == 0u ||
        (rx == NULL && rx_length != 0u) ||
        (rx != NULL && rx_length == 0u)) {
        latch_error(state, PICO_ERROR_INVALID_ARG);
        return PICO_ERROR_INVALID_ARG;
    }

    if (tx_length > WIZNET_SPI_MAX_TRANSFER_SIZE ||
        rx_length > WIZNET_SPI_MAX_TRANSFER_SIZE) {
        latch_error(state, PICO_ERROR_INVALID_ARG);
        return PICO_ERROR_INVALID_ARG;
    }
    const size_t payload_length = rx != NULL ? rx_length : tx_length;

    set_lifecycle(state, WIZNET_SPI_TRANSFERRING);

    if (rx != NULL && tx != NULL) {
        pio_sm_set_enabled(state->pio, state->pio_sm, false); // disable sm
        pio_sm_set_wrap(state->pio, state->pio_sm, state->pio_offset + PIO_OFFSET_WRITE_BITS, state->pio_offset + PIO_OFFSET_READ_BITS_END - 1);
        pio_sm_clear_fifos(state->pio, state->pio_sm); // clear fifos from previous run
        pio_sm_set_pindirs_with_mask(state->pio, state->pio_sm, 1u << state->spi_config->data_out_pin, 1u << state->spi_config->data_out_pin);
        pio_sm_restart(state->pio, state->pio_sm);
        pio_sm_clkdiv_restart(state->pio, state->pio_sm);
        pio_sm_put(state->pio, state->pio_sm, tx_length * 8 - 1); // set x
        pio_sm_exec(state->pio, state->pio_sm, pio_encode_out(pio_x, 32));
        pio_sm_put(state->pio, state->pio_sm, rx_length ? (rx_length - 1) : 0); // set y, guard against underflow
        pio_sm_exec(state->pio, state->pio_sm, pio_encode_out(pio_y, 32));
        pio_sm_exec(state->pio, state->pio_sm, pio_encode_jmp(state->pio_offset)); // setup pc
        if (!prepare_dma_channel(
                state, state->dma_out,
                &state->ownership.dma_out_quarantined) ||
            !prepare_dma_channel(
                state, state->dma_in,
                &state->ownership.dma_in_quarantined)) {
            record_transfer_failure(state, PICO_ERROR_IO, rx, rx_length);
            return PICO_ERROR_IO;
        }

        dma_channel_config out_config = dma_channel_get_default_config(state->dma_out);
        channel_config_set_dreq(&out_config, pio_get_dreq(state->pio, state->pio_sm, true));
        channel_config_set_transfer_data_size(&out_config, DMA_SIZE_8);
        dma_channel_configure(state->dma_out, &out_config, &state->pio->txf[state->pio_sm], tx, tx_length, true);

        dma_channel_config in_config = dma_channel_get_default_config(state->dma_in);
        channel_config_set_dreq(&in_config, pio_get_dreq(state->pio, state->pio_sm, false));
        channel_config_set_write_increment(&in_config, true);
        channel_config_set_read_increment(&in_config, false);
        channel_config_set_transfer_data_size(&in_config, DMA_SIZE_8);
        dma_channel_configure(state->dma_in, &in_config, rx, &state->pio->rxf[state->pio_sm], rx_length, true);

        pio_sm_set_enabled(state->pio, state->pio_sm, true);
        __compiler_memory_barrier();

        const int result = wait_for_transfer(state, true);
        if (result != PICO_OK) {
            record_transfer_failure(state, result, rx, rx_length);
            return result;
        }

        __compiler_memory_barrier();
    } else if (tx != NULL) {
        pio_sm_set_enabled(state->pio, state->pio_sm, false);
        pio_sm_set_wrap(state->pio, state->pio_sm, state->pio_offset + PIO_OFFSET_WRITE_BITS, state->pio_offset + PIO_OFFSET_WRITE_BITS_END - 1);
        pio_sm_clear_fifos(state->pio, state->pio_sm);
        pio_sm_restart(state->pio, state->pio_sm);
        pio_sm_clkdiv_restart(state->pio, state->pio_sm);
        pio_sm_put(state->pio, state->pio_sm, tx_length * 8 - 1);
        pio_sm_exec(state->pio, state->pio_sm, pio_encode_out(pio_x, 32));
        pio_sm_put(state->pio, state->pio_sm, tx_length - 1);
        pio_sm_exec(state->pio, state->pio_sm, pio_encode_out(pio_y, 32));
        pio_sm_exec(state->pio, state->pio_sm, pio_encode_set(pio_pins, 0));
        pio_sm_set_consecutive_pindirs(state->pio, state->pio_sm, state->spi_config->data_out_pin, 1, true);
        pio_sm_exec(state->pio, state->pio_sm, pio_encode_jmp(state->pio_offset + PIO_OFFSET_WRITE_BITS));
        if (!prepare_dma_channel(
                state, state->dma_out,
                &state->ownership.dma_out_quarantined)) {
            record_transfer_failure(state, PICO_ERROR_IO, NULL, 0u);
            return PICO_ERROR_IO;
        }

        dma_channel_config out_config = dma_channel_get_default_config(state->dma_out);
        channel_config_set_dreq(&out_config, pio_get_dreq(state->pio, state->pio_sm, true));

        channel_config_set_transfer_data_size(&out_config, DMA_SIZE_8);
        dma_channel_configure(state->dma_out, &out_config, &state->pio->txf[state->pio_sm], tx, tx_length, true);

        pio_sm_set_enabled(state->pio, state->pio_sm, true);
        const int result = wait_for_transfer(state, false);
        if (result != PICO_OK) {
            record_transfer_failure(state, result, NULL, 0u);
            return result;
        }
        __compiler_memory_barrier();
        pio_sm_set_enabled(state->pio, state->pio_sm, false);
        pio_sm_set_consecutive_pindirs(state->pio, state->pio_sm, state->spi_config->data_in_pin, 1, false);
    }
    pio_sm_exec(state->pio, state->pio_sm, pio_encode_mov(pio_pins, pio_null)); // for next time we turn output on

    set_lifecycle(state, WIZNET_SPI_READY);
    return (int)payload_length;
}


static uint8_t wiznet_spi_pio_read_byte(void) {
    assert(active_state);
    assert(active_state->spi_header_count == SPI_HEADER_LEN);
    uint8_t ret;
    if (pio_spi_transfer(active_state, active_state->spi_header,
                         active_state->spi_header_count, &ret, 1) < 0) {
        ret = 0u;
    }
    active_state->spi_header_count = 0;
    return ret;
}

static void wiznet_spi_pio_write_byte(uint8_t wb) {
    uint8_t frame[SPI_HEADER_LEN + 1u];

    assert(active_state);
    if (active_state->spi_header_count == SPI_HEADER_LEN) {
        memcpy(frame, active_state->spi_header, SPI_HEADER_LEN);
        frame[SPI_HEADER_LEN] = wb;
        active_state->spi_header_count = 0u;
        (void)pio_spi_transfer(active_state, frame, sizeof(frame), NULL, 0u);
        return;
    }

    assert(active_state->spi_header_count == 0u);
    (void)pio_spi_transfer(active_state, &wb, 1u, NULL, 0u);
}

// To read a buffer we must first have been asked to write a 3 byte spi header
static void wiznet_spi_pio_read_buffer(uint8_t* pBuf, uint16_t len) {

    assert(active_state);
    assert(active_state->spi_header_count == SPI_HEADER_LEN);
    (void)pio_spi_transfer(active_state, active_state->spi_header,
                           active_state->spi_header_count, pBuf, len);
    active_state->spi_header_count = 0;
}

// If we have been asked to write a spi header already, then write it and the rest of the buffer
// or else if we've been given enough data for just the spi header, save it until the next call
// or we're writing a byte in which case we're given a buffer including the spi header
static void wiznet_spi_pio_write_buffer(uint8_t* pBuf, uint16_t len) {
    assert(active_state);
    if (len == SPI_HEADER_LEN && active_state->spi_header_count == 0) {
        memcpy(active_state->spi_header, pBuf, SPI_HEADER_LEN); // expect another call
        active_state->spi_header_count = SPI_HEADER_LEN;
    } else {
        if (active_state->spi_header_count == SPI_HEADER_LEN) {
            (void)pio_spi_transfer(active_state, active_state->spi_header,
                                   SPI_HEADER_LEN, NULL, 0);
            active_state->spi_header_count = 0;
        }
        assert(active_state->spi_header_count == 0);
        (void)pio_spi_transfer(active_state, pBuf, len, NULL, 0);
    }
}
#endif


static void wiznet_spi_pio_set_active(wiznet_spi_handle_t handle) {
    critical_section_enter_blocking(&state_critical_section);
    active_state = (spi_pio_state_t *)handle;
    critical_section_exit(&state_critical_section);
}

static void wiznet_spi_pio_set_inactive(void) {
    critical_section_enter_blocking(&state_critical_section);
    active_state = NULL;
    critical_section_exit(&state_critical_section);
}

int wiznet_spi_pio_get_last_error(wiznet_spi_handle_t handle) {
    const spi_pio_state_t *state = (const spi_pio_state_t *)handle;

    return state != NULL ? state->last_error : PICO_ERROR_INVALID_ARG;
}

void wiznet_spi_pio_clear_last_error(wiznet_spi_handle_t handle) {
    spi_pio_state_t *state = (spi_pio_state_t *)handle;

    if (state != NULL && state->lifecycle != WIZNET_SPI_FAULTED) {
        state->last_error = PICO_OK;
    }
}

wiznet_spi_lifecycle_state_t wiznet_spi_pio_get_state(
    wiznet_spi_handle_t handle) {
    const spi_pio_state_t *state = (const spi_pio_state_t *)handle;

    return state != NULL ? state->lifecycle : WIZNET_SPI_UNINIT;
}

static void wiznet_spi_pio_reset(wiznet_spi_handle_t handle) {

    spi_pio_state_t *state = (spi_pio_state_t *)handle;
    gpio_set_dir(state->spi_config->reset_pin, GPIO_OUT);
    gpio_put(state->spi_config->reset_pin, 0);
    sleep_ms(100);
    gpio_put(state->spi_config->reset_pin, 1);
    sleep_ms(100);

}

int wiznet_spi_pio_sleep_ex(wiznet_spi_handle_t handle) {
    spi_pio_state_t *state = (spi_pio_state_t *)handle;
    int result = PICO_OK;

    if (state == NULL) {
        return PICO_ERROR_INVALID_ARG;
    }

    wiznet_spi_pio_sync_initialize();
    mutex_enter_blocking(&spi_bus_mutex);
    if (state->lifecycle != WIZNET_SPI_READY) {
        result = PICO_ERROR_INVALID_STATE;
        latch_error(state, result);
    } else {
        pio_sm_set_enabled(state->pio, (uint)state->pio_sm, false);
        set_lifecycle(state, WIZNET_SPI_SLEEPING);
    }
    mutex_exit(&spi_bus_mutex);
    return result;
}

static void wiznet_spi_pio_sleep(wiznet_spi_handle_t handle) {
    (void)wiznet_spi_pio_sleep_ex(handle);
}

int wiznet_spi_pio_wake_ex(wiznet_spi_handle_t handle) {
    spi_pio_state_t *state = (spi_pio_state_t *)handle;
    int result = PICO_OK;

    if (state == NULL) {
        return PICO_ERROR_INVALID_ARG;
    }

    wiznet_spi_pio_sync_initialize();
    mutex_enter_blocking(&spi_bus_mutex);
    if (state->lifecycle != WIZNET_SPI_SLEEPING) {
        result = PICO_ERROR_INVALID_STATE;
        latch_error(state, result);
    } else {
        pio_sm_restart(state->pio, (uint)state->pio_sm);
        pio_sm_set_enabled(state->pio, (uint)state->pio_sm, true);
        set_lifecycle(state, WIZNET_SPI_READY);
    }
    mutex_exit(&spi_bus_mutex);
    return result;
}

static void wiznet_spi_pio_wake(wiznet_spi_handle_t handle) {
    (void)wiznet_spi_pio_wake_ex(handle);
}

int wiznet_spi_pio_recover(wiznet_spi_handle_t handle) {
    spi_pio_state_t *state = (spi_pio_state_t *)handle;
    wiznet_spi_config_t saved_config;
    wiznet_spi_handle_t reopened_handle = NULL;
    bool was_active;
    int result;

    if (state == NULL) {
        return PICO_ERROR_INVALID_ARG;
    }

    wiznet_spi_pio_sync_initialize();
    mutex_enter_blocking(&spi_bus_mutex);
    if (state->lifecycle != WIZNET_SPI_FAULTED) {
        latch_error(state, PICO_ERROR_INVALID_STATE);
        mutex_exit(&spi_bus_mutex);
        return PICO_ERROR_INVALID_STATE;
    }

    saved_config = state->config;
    was_active = active_state == state;
    state->last_error = PICO_OK;

    result = wiznet_spi_pio_close_locked(state);
    if (result == PICO_OK) {
        result = wiznet_spi_pio_open_normalized(&saved_config,
                                                 &reopened_handle);
    }

    if (result == PICO_OK && reopened_handle == handle) {
        if (was_active) {
            critical_section_enter_blocking(&state_critical_section);
            active_state = state;
            critical_section_exit(&state_critical_section);
        }
    } else {
        if (result == PICO_OK) {
            (void)wiznet_spi_pio_close_locked(
                (spi_pio_state_t *)reopened_handle);
            result = PICO_ERROR_RESOURCE_IN_USE;
        }
        state->config = saved_config;
        state->spi_config = &state->config;
        state->funcs = get_wiznet_spi_pio_impl();
        state->last_error = result;
        set_lifecycle(state, WIZNET_SPI_FAULTED);
        if (was_active) {
            critical_section_enter_blocking(&state_critical_section);
            active_state = state;
            critical_section_exit(&state_critical_section);
        }
    }

    mutex_exit(&spi_bus_mutex);
    return result;
}

static wiznet_spi_funcs_t *get_wiznet_spi_pio_impl(void) {
    static const wiznet_spi_funcs_t funcs = {
        .close = wiznet_spi_pio_close,
        .set_active = wiznet_spi_pio_set_active,
        .set_inactive = wiznet_spi_pio_set_inactive,
        .frame_start = wiznet_spi_pio_frame_start,
        .frame_end = wiznet_spi_pio_frame_end,
        .read_byte = wiznet_spi_pio_read_byte,
        .write_byte = wiznet_spi_pio_write_byte,
#if   (_WIZCHIP_ == W5500)
        .read_buffer = wiznet_spi_pio_read_buffer,
        .write_buffer = wiznet_spi_pio_write_buffer,
#endif
        .reset = wiznet_spi_pio_reset,
        .sleep = wiznet_spi_pio_sleep,
        .wake  = wiznet_spi_pio_wake,
    };
    return (wiznet_spi_funcs_t *)&funcs;
}
