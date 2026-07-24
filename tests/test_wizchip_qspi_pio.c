#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "fake_pico_sdk.h"
#undef PIN_INT
#include "wizchip_conf.h"
#include "wizchip_qspi_pio.h"

#define RP2040_INVALID_GPIO 30u
#define DEFAULT_TRANSFER_TIMEOUT_US 100000u
#define DEFAULT_ABORT_TIMEOUT_US 1000u

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            fprintf(stderr, "FAIL: %s:%d: %s\n", __FILE__, __LINE__,          \
                    #condition);                                                \
            return false;                                                       \
        }                                                                       \
    } while (0)

typedef struct {
    wiznet_spi_funcs_t *funcs;
    wiznet_spi_config_t config;
} transport_state_view_t;

static wiznet_spi_config_t valid_config(void)
{
    return (wiznet_spi_config_t){
        .data_in_pin = 16u,
        .data_out_pin = 19u,
        .cs_pin = 17u,
        .clock_pin = 18u,
        .irq_pin = 21u,
        .reset_pin = 20u,
        .clock_div_major = 2u,
        .clock_div_minor = 0u,
        .spi_hw_instance = 0u,
        .transfer_timeout_us = 25000u,
        .abort_timeout_us = 500u,
    };
}

static const wiznet_spi_config_t *stored_config(wiznet_spi_handle_t handle)
{
    const transport_state_view_t *state =
        (const transport_state_view_t *)(const void *)handle;

    return &state->config;
}

static bool configs_equal(const wiznet_spi_config_t *left,
                          const wiznet_spi_config_t *right)
{
    return left->data_in_pin == right->data_in_pin &&
           left->data_out_pin == right->data_out_pin &&
           left->cs_pin == right->cs_pin &&
           left->clock_pin == right->clock_pin &&
           left->irq_pin == right->irq_pin &&
           left->reset_pin == right->reset_pin &&
           left->clock_div_major == right->clock_div_major &&
           left->clock_div_minor == right->clock_div_minor &&
           left->spi_hw_instance == right->spi_hw_instance &&
           left->transfer_timeout_us == right->transfer_timeout_us &&
           left->abort_timeout_us == right->abort_timeout_us;
}

static void close_transport(wiznet_spi_handle_t handle)
{
    if (handle != NULL) {
        (*handle)->close(handle);
    }
}

static bool test_null_config_is_rejected(void)
{
    wiznet_spi_handle_t handle = (wiznet_spi_handle_t)(uintptr_t)1u;

    CHECK(wiznet_spi_pio_open_ex(NULL, &handle) == PICO_ERROR_INVALID_ARG);
    CHECK(handle == NULL);
    return true;
}

static bool test_invalid_pin_numbers_are_rejected(void)
{
    wiznet_spi_config_t config = valid_config();
    uint8_t *pins[] = {
        &config.data_in_pin,
        &config.data_out_pin,
        &config.cs_pin,
        &config.clock_pin,
        &config.irq_pin,
        &config.reset_pin,
    };

    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        wiznet_spi_handle_t handle = (wiznet_spi_handle_t)(uintptr_t)1u;
        const uint8_t original = *pins[i];

        *pins[i] = RP2040_INVALID_GPIO;
        CHECK(wiznet_spi_pio_open_ex(&config, &handle) ==
              PICO_ERROR_INVALID_ARG);
        CHECK(handle == NULL);
        *pins[i] = original;
    }

    return true;
}

static bool test_invalid_pio_instance_and_divider_are_rejected(void)
{
    wiznet_spi_config_t config = valid_config();
    wiznet_spi_handle_t handle = (wiznet_spi_handle_t)(uintptr_t)1u;

    config.spi_hw_instance = NUM_PIOS;
    CHECK(wiznet_spi_pio_open_ex(&config, &handle) == PICO_ERROR_INVALID_ARG);
    CHECK(handle == NULL);

    config = valid_config();
    config.clock_div_major = 0u;
    handle = (wiznet_spi_handle_t)(uintptr_t)1u;
    CHECK(wiznet_spi_pio_open_ex(&config, &handle) == PICO_ERROR_INVALID_ARG);
    CHECK(handle == NULL);
    return true;
}

static bool test_zero_timeouts_select_defaults(void)
{
    wiznet_spi_config_t config = valid_config();
    wiznet_spi_handle_t handle = NULL;

    config.transfer_timeout_us = 0u;
    config.abort_timeout_us = 0u;
    CHECK(wiznet_spi_pio_open_ex(&config, &handle) == PICO_OK);
    CHECK(handle != NULL);
    CHECK(stored_config(handle)->transfer_timeout_us ==
          DEFAULT_TRANSFER_TIMEOUT_US);
    CHECK(stored_config(handle)->abort_timeout_us == DEFAULT_ABORT_TIMEOUT_US);

    close_transport(handle);
    return true;
}

static wiznet_spi_handle_t open_from_stack(int *status)
{
    wiznet_spi_config_t config = valid_config();
    wiznet_spi_handle_t handle = NULL;

    *status = wiznet_spi_pio_open_ex(&config, &handle);
    return handle;
}

static bool test_transport_owns_configuration_copy(void)
{
    wiznet_spi_config_t config = valid_config();
    const wiznet_spi_config_t original = config;
    wiznet_spi_handle_t handle = NULL;

    CHECK(wiznet_spi_pio_open_ex(&config, &handle) == PICO_OK);
    CHECK(handle != NULL);
    memset(&config, 0xff, sizeof(config));
    CHECK(configs_equal(stored_config(handle), &original));
    close_transport(handle);

    int status = PICO_ERROR_GENERIC;
    handle = open_from_stack(&status);
    CHECK(status == PICO_OK);
    CHECK(handle != NULL);
    CHECK(configs_equal(stored_config(handle), &original));
    close_transport(handle);
    return true;
}

static bool test_scalar_write_callback_completes_pending_header(void)
{
    static const uint8_t header[] = {0x00u, 0x24u, 0x14u};
    static const uint8_t expected[] = {0x00u, 0x24u, 0x14u, 0x5au};
    wiznet_spi_config_t config = valid_config();
    wiznet_spi_handle_t handle = NULL;

    fake_pico_sdk_reset();
    CHECK(wiznet_spi_pio_open_ex(&config, &handle) == PICO_OK);
    CHECK(handle != NULL);

    (*handle)->set_active(handle);
    (*handle)->frame_start();
    (*handle)->write_buffer((uint8_t *)(uintptr_t)header, sizeof(header));
    (*handle)->write_byte(0x5au);
    (*handle)->frame_end();

    CHECK(fake_panic_unsupported_calls() == 0u);
    CHECK(fake_dma_tx_trace_length() == sizeof(expected));
    CHECK(memcmp(fake_dma_tx_trace_data(), expected, sizeof(expected)) == 0);

    (*handle)->set_inactive();
    close_transport(handle);
    return true;
}

int main(void)
{
    const struct {
        const char *name;
        bool (*run)(void);
    } tests[] = {
        {"null config rejected", test_null_config_is_rejected},
        {"invalid pins rejected", test_invalid_pin_numbers_are_rejected},
        {"invalid PIO/divider rejected",
         test_invalid_pio_instance_and_divider_are_rejected},
        {"timeout defaults", test_zero_timeouts_select_defaults},
        {"caller-lifetime independence",
          test_transport_owns_configuration_copy},
        {"scalar callback completes pending header",
         test_scalar_write_callback_completes_pending_header},
    };
    int failures = 0;

    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) {
        if (!tests[i].run()) {
            fprintf(stderr, "FAILED: %s\n", tests[i].name);
            ++failures;
        }
    }

    if (failures != 0) {
        fprintf(stderr, "%d transport configuration test(s) failed\n",
                failures);
        return 1;
    }

    puts("PASS: transport configuration");
    return 0;
}
