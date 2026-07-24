#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "fake_pico_sdk.h"
#include "wizchip_conf.h"
#include "wizchip_spi.h"

static unsigned int failures;

#define CHECK(condition, message) do { \
    if (!(condition)) { \
        ++failures; \
        fprintf(stderr, "FAIL [%d]: %s\n", __LINE__, (message)); \
    } \
} while (0)

static void test_initialization_registers_production_callbacks(void)
{
    static const uint8_t expected_write[] = {0x00u, 0x01u, 0x04u, 0x5au};
    const uint8_t *trace;
    size_t trace_length;

    fake_pico_sdk_reset();
    fake_dma_set_rx_value(0x04u);
    CHECK(wizchip_spi_initialize() == PICO_OK,
          "production SPI lifecycle initializes");
    CHECK(WIZCHIP.IF.SPI._read_byte != NULL &&
              WIZCHIP.IF.SPI._write_byte != NULL &&
              WIZCHIP.IF.SPI._read_burst != NULL &&
              WIZCHIP.IF.SPI._write_burst != NULL,
          "production SPI data callbacks are registered");
    CHECK(WIZCHIP.SPISTATUS._check_busy != NULL &&
              WIZCHIP.SPISTATUS._get_error != NULL &&
              WIZCHIP.SPISTATUS._clear_error != NULL,
          "production SPI status callbacks are registered");
    CHECK(WIZCHIP.SPISTATUS._check_busy() == 0u,
          "ready production transport is not busy");
    CHECK(WIZCHIP.SPISTATUS._get_error() == PICO_OK,
          "ready production transport reports no error");

    fake_dma_clear_tx_trace();
    CHECK(wizchip_write8_checked(GAR, 0x5au) == 0,
          "root scalar write dispatches through production SPI callbacks");
    trace = fake_dma_tx_trace_data();
    trace_length = fake_dma_tx_trace_length();
    CHECK(trace_length == sizeof(expected_write) &&
              memcmp(trace, expected_write, sizeof(expected_write)) == 0,
          "production SPI dispatch preserves the complete VDM frame");
}

static void test_transport_errors_are_signed_and_clearable(void)
{
    fake_dma_set_ahb_error(0u, true);
    fake_dma_set_ahb_error(1u, true);
    (void)wizchip_write8_checked(GAR, 0xa5u);
    CHECK(WIZCHIP.SPISTATUS._get_error() == PICO_ERROR_IO,
          "production SPI status preserves signed Pico error values");
    WIZCHIP.SPISTATUS._clear_error();
    CHECK(WIZCHIP.SPISTATUS._get_error() == PICO_ERROR_IO,
          "faulted production transport keeps its error sticky");
    fake_dma_set_ahb_error(0u, false);
    fake_dma_set_ahb_error(1u, false);
}

int main(void)
{
    test_initialization_registers_production_callbacks();
    test_transport_errors_are_signed_and_clearable();

    if (failures != 0u) {
        fprintf(stderr, "%u SPI integration failure(s)\n", failures);
        return 1;
    }
    puts("PASS: production SPI lifecycle, dispatch, and error latching");
    return 0;
}
