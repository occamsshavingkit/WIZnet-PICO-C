#include <stdint.h>
#include <stdio.h>

#include "fake_pico_sdk.h"
#include "wizchip_conf.h"
#include "w5500.h"
#include "w5500_spi_model.h"
#include "wizchip_gpio_irq.h"

static unsigned int failures;
static unsigned int callback_calls;
static uint8_t callback_socket;
static sockint_kind callback_events;
static void *callback_context;
static w5500_model_t model;

#define CHECK(condition, message) do { \
    if (!(condition)) { \
        ++failures; \
        fprintf(stderr, "FAIL [%d]: %s\n", __LINE__, (message)); \
    } \
} while (0)

static void critical_enter(void) {}
static void critical_exit(void) {}
static void chip_select(void) { model_cs_select(&model); }
static void chip_deselect(void) { model_cs_deselect(&model); }
static uint8_t spi_read(void) { return model_spi_read_byte(&model); }
static void spi_write(uint8_t value) { model_spi_write_byte(&model, value); }
static void spi_read_burst(uint8_t *buf, uint16_t len)
{
    model_spi_read_burst(&model, buf, len);
}
static void spi_write_burst(uint8_t *buf, uint16_t len)
{
    model_spi_write_burst(&model, buf, len);
}

static void socket_callback(uint8_t sn, sockint_kind events, void *context)
{
    ++callback_calls;
    callback_socket = sn;
    callback_events = events;
    callback_context = context;
}

static void install_model(void)
{
    model_init(&model);
    reg_wizchip_cris_cbfunc(critical_enter, critical_exit);
    reg_wizchip_cs_cbfunc(chip_select, chip_deselect);
    reg_wizchip_spi_cbfunc(spi_read, spi_write);
    reg_wizchip_spiburst_cbfunc(spi_read_burst, spi_write_burst);
}

static void test_registration_dispatch_and_cleanup(void)
{
    static int context;

    fake_pico_sdk_reset();
    install_model();
    callback_calls = 0u;
    CHECK(wizchip_gpio_interrupt_register(2u,
              (sockint_kind)(SIK_RECEIVED | SIK_TIMEOUT),
              socket_callback, &context) == PICO_OK,
          "socket IRQ registration succeeds");
    CHECK(fake_gpio_raw_handler_add_calls() == 1u,
          "first registration installs one raw handler");
    CHECK((fake_gpio_enabled_events(PIN_INT) & GPIO_IRQ_EDGE_FALL) != 0u,
          "registration enables only the falling-edge source");
    CHECK((model.sir & (1u << 2)) == 0u,
          "registration does not synthesize pending hardware state");
    CHECK((model.sockets[2].ir & (SIK_RECEIVED | SIK_TIMEOUT)) == 0u,
          "registration does not synthesize socket events");

    model.sir = (uint8_t)(1u << 2);
    model.sockets[2].ir = (uint8_t)(SIK_RECEIVED | SIK_TIMEOUT);
    fake_gpio_trigger_irq(PIN_INT, GPIO_IRQ_EDGE_FALL);
    CHECK(wizchip_gpio_interrupt_pending(),
          "raw ISR records deferred task-context work");
    CHECK((fake_gpio_acknowledged_events(PIN_INT) & GPIO_IRQ_EDGE_FALL) != 0u,
          "raw ISR acknowledges the falling edge");
    CHECK((fake_gpio_enabled_events(PIN_INT) & GPIO_IRQ_EDGE_FALL) == 0u,
          "raw ISR disables the source until dispatch");
    CHECK(callback_calls == 0u,
          "raw ISR does not invoke the socket callback");

    CHECK(wizchip_gpio_interrupt_dispatch() == PICO_OK,
          "task-context dispatch succeeds");
    CHECK(callback_calls == 1u && callback_socket == 2u &&
              callback_events == (SIK_RECEIVED | SIK_TIMEOUT) &&
              callback_context == &context,
          "dispatch forwards socket, events, and caller context");
    CHECK(model.sockets[2].ir == 0u,
          "dispatch clears only delivered socket events");
    CHECK((fake_gpio_enabled_events(PIN_INT) & GPIO_IRQ_EDGE_FALL) != 0u,
          "dispatch re-enables registered IRQ delivery");

    CHECK(wizchip_gpio_interrupt_unregister(2u) == PICO_OK,
          "socket IRQ unregister succeeds");
    CHECK(fake_gpio_raw_handler_remove_calls() == 1u,
          "last unregister removes the raw handler");
    CHECK((fake_gpio_enabled_events(PIN_INT) & GPIO_IRQ_EDGE_FALL) == 0u,
          "last unregister disables IRQ delivery");
    fake_gpio_trigger_irq(PIN_INT, GPIO_IRQ_EDGE_FALL);
    CHECK(!wizchip_gpio_interrupt_pending() && callback_calls == 1u,
          "deinitialized IRQ cannot schedule or dispatch callbacks");
}

int main(void)
{
    test_registration_dispatch_and_cleanup();
    if (failures != 0u) {
        fprintf(stderr, "%u GPIO IRQ integration failure(s)\n", failures);
        return 1;
    }
    puts("PASS: production GPIO registration, ISR, dispatch, and cleanup");
    return 0;
}
