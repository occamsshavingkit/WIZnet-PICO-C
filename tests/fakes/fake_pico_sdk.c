#include "fake_pico_sdk.h"
#include <stdlib.h>
#include <string.h>

#define FAKE_DMA_TRACE_CAPACITY 16384u

static int64_t fake_us;
static int claimed_sm[NUM_PIOS][NUM_PIO_STATE_MACHINES];
static int claimed_dma[DMA_CHANNEL_COUNT];
static bool dma_busy[DMA_CHANNEL_COUNT];
static uint8_t dma_tx_trace[FAKE_DMA_TRACE_CAPACITY];
static size_t dma_tx_length;
static unsigned int panic_unsupported_count;
static uint8_t dma_rx_value;
static void (*gpio_raw_handler)(void);
static uint32_t gpio_irq_events[30];
static uint32_t gpio_enabled[30];
static uint32_t gpio_acknowledged[30];
static unsigned int gpio_raw_add_calls;
static unsigned int gpio_raw_remove_calls;

pio_hw_t fake_pio_instances[NUM_PIOS];
dma_hw_t fake_dma_hw;
pads_bank0_hw_t fake_pads_bank0_hw;

int64_t fake_time_us_64(void) { return fake_us; }
uint64_t time_us_64(void) { return (uint64_t)fake_us; }
void fake_time_tick_us(uint64_t us) { fake_us += (int64_t)us; }

void fake_pico_sdk_reset(void)
{
    fake_us = 0;
    memset(claimed_sm, 0, sizeof(claimed_sm));
    memset(claimed_dma, 0, sizeof(claimed_dma));
    memset(dma_busy, 0, sizeof(dma_busy));
    memset(&fake_dma_hw, 0, sizeof(fake_dma_hw));
    memset(fake_pio_instances, 0, sizeof(fake_pio_instances));
    memset(&fake_pads_bank0_hw, 0, sizeof(fake_pads_bank0_hw));
    memset(dma_tx_trace, 0, sizeof(dma_tx_trace));
    dma_tx_length = 0u;
    panic_unsupported_count = 0u;
    dma_rx_value = 0u;
    gpio_raw_handler = NULL;
    memset(gpio_irq_events, 0, sizeof(gpio_irq_events));
    memset(gpio_enabled, 0, sizeof(gpio_enabled));
    memset(gpio_acknowledged, 0, sizeof(gpio_acknowledged));
    gpio_raw_add_calls = 0u;
    gpio_raw_remove_calls = 0u;
}
void fake_dma_set_rx_value(uint8_t value) { dma_rx_value = value; }
void fake_dma_clear_tx_trace(void) { dma_tx_length = 0u; }
void fake_dma_set_ahb_error(uint channel, bool enabled)
{
    if (channel < DMA_CHANNEL_COUNT) {
        if (enabled) {
            fake_dma_hw.ch[channel].ctrl_trig |=
                DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS;
        } else {
            fake_dma_hw.ch[channel].ctrl_trig &=
                ~DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS;
        }
    }
}
unsigned int fake_gpio_raw_handler_add_calls(void) { return gpio_raw_add_calls; }
unsigned int fake_gpio_raw_handler_remove_calls(void) { return gpio_raw_remove_calls; }
uint32_t fake_gpio_enabled_events(uint gpio)
{
    return gpio < 30u ? gpio_enabled[gpio] : 0u;
}
uint32_t fake_gpio_acknowledged_events(uint gpio)
{
    return gpio < 30u ? gpio_acknowledged[gpio] : 0u;
}
void fake_gpio_trigger_irq(uint gpio, uint32_t events)
{
    if (gpio < 30u) {
        gpio_irq_events[gpio] |= events;
        if ((gpio_enabled[gpio] & events) != 0u && gpio_raw_handler != NULL) {
            gpio_raw_handler();
        }
    }
}

size_t fake_dma_tx_trace_length(void) { return dma_tx_length; }
const uint8_t *fake_dma_tx_trace_data(void) { return dma_tx_trace; }
unsigned int fake_panic_unsupported_calls(void)
{
    return panic_unsupported_count;
}

mutex_t *mutex_init(mutex_t *m) { m->locked = 0; return m; }
void mutex_enter_blocking(mutex_t *m) { m->locked = 1; }
void mutex_exit(mutex_t *m) { m->locked = 0; }
int mutex_try_enter(mutex_t *m, uint32_t *owner) { (void)owner; if (!m->locked) { m->locked = 1; return 1; } return 0; }

void critical_section_init(critical_section_t *cs) { cs->entered = 0; }
void critical_section_enter_blocking(critical_section_t *cs) { cs->entered++; }
void critical_section_exit(critical_section_t *cs) { if (cs->entered) cs->entered--; }

uint32_t clock_get_hz(int clk_id) { (void)clk_id; return 125000000; }

bool pio_claim_free_sm_and_add_program(const pio_program_t *prog,
                                       pio_hw_t **pio, uint *sm,
                                       uint *pio_offset)
{
    (void)prog;
    if (pio == NULL || sm == NULL || pio_offset == NULL) {
        return false;
    }
    for (int pi = 0; pi < NUM_PIOS; ++pi) {
        for (int index = 0; index < NUM_PIO_STATE_MACHINES; ++index) {
            if (!claimed_sm[pi][index]) {
                claimed_sm[pi][index] = 1;
                *pio = &fake_pio_instances[pi];
                *sm = (uint)index;
                *pio_offset = 0u;
                return true;
            }
        }
    }
    return false;
}

pio_sm_config pio_get_default_sm_config(void) {
    pio_sm_config c = {0};
    return c;
}

void pio_sm_set_enabled(pio_hw_t *pio, int sm, int enabled) { (void)pio; (void)sm; (void)enabled; }
void pio_sm_clear_enabled(pio_hw_t *pio, int sm) { (void)pio; (void)sm; }
int pio_sm_is_enabled(pio_hw_t *pio, int sm) { (void)pio; (void)sm; return 0; }
void pio_remove_program(pio_hw_t *pio, const pio_program_t *prog, uint offset) { (void)pio; (void)prog; (void)offset; }
int pio_claim_unused_sm(pio_hw_t *pio, int required) { (void)pio; (void)required; return 0; }
void pio_sm_unclaim(pio_hw_t *pio, int sm) { int pi = (pio == pio0) ? 0 : 1; claimed_sm[pi][sm] = 0; }
void pio_sm_claim(pio_hw_t *pio, int sm) { int pi = (pio == pio0) ? 0 : 1; claimed_sm[pi][sm] = 1; }
void sm_config_set_wrap(pio_sm_config *c, uint wrap_target, uint wrap) { (void)c; (void)wrap_target; (void)wrap; }
void sm_config_set_out_pins(pio_sm_config *c, uint out_base, uint out_count) { (void)c; (void)out_base; (void)out_count; }
void sm_config_set_set_pins(pio_sm_config *c, uint set_base, uint set_count) { (void)c; (void)set_base; (void)set_count; }
void sm_config_set_sideset(pio_sm_config *c, uint sideset_count, int optional, int pindirs) { (void)c; (void)sideset_count; (void)optional; (void)pindirs; }
void sm_config_set_clkdiv_int_frac(pio_sm_config *c, uint16_t div_int, uint8_t div_frac) { (void)c; (void)div_int; (void)div_frac; }
void sm_config_set_in_shift(pio_sm_config *c, int shift_right, int auto_push, uint push_threshold) { (void)c; (void)shift_right; (void)auto_push; (void)push_threshold; }
void sm_config_set_out_shift(pio_sm_config *c, int shift_right, int auto_pull, uint pull_threshold) { (void)c; (void)shift_right; (void)auto_pull; (void)pull_threshold; }
void sm_config_set_fifo_join(pio_sm_config *c, int join) { (void)c; (void)join; }
void sm_config_set_out_special(pio_sm_config *c, int sticky, int has_enable_pin, uint enable_pin_index) { (void)c; (void)sticky; (void)has_enable_pin; (void)enable_pin_index; }
void pio_sm_init(pio_hw_t *pio, int sm, uint initial_pc, const pio_sm_config *config) { (void)pio; (void)sm; (void)initial_pc; (void)config; }
uint pio_add_program_at_offset(pio_hw_t *pio, const pio_program_t *prog, uint offset) { (void)pio; (void)prog; return offset; }
void pio_remove_program_and_unclaim_sm(const pio_program_t *prog,
                                       pio_hw_t *pio, uint sm, uint offset)
{
    (void)prog;
    (void)offset;
    pio_sm_unclaim(pio, (int)sm);
}
bool pio_sm_is_tx_fifo_empty(pio_hw_t *pio, uint sm) { (void)pio; (void)sm; return true; }
bool pio_sm_is_rx_fifo_empty(pio_hw_t *pio, uint sm) { (void)pio; (void)sm; return true; }
void pio_sm_clear_fifos(pio_hw_t *pio, uint sm) { (void)pio; (void)sm; }
void pio_sm_set_config(pio_hw_t *pio, uint sm, const pio_sm_config *config) { (void)pio; (void)sm; (void)config; }
void pio_sm_set_wrap(pio_hw_t *pio, uint sm, uint wrap_target, uint wrap) { (void)pio; (void)sm; (void)wrap_target; (void)wrap; }
void pio_sm_set_pindirs_with_mask(pio_hw_t *pio, uint sm, uint32_t values, uint32_t mask) { (void)pio; (void)sm; (void)values; (void)mask; }
void pio_sm_set_consecutive_pindirs(pio_hw_t *pio, uint sm, uint pin_base, uint pin_count, bool is_out) { (void)pio; (void)sm; (void)pin_base; (void)pin_count; (void)is_out; }
void pio_sm_restart(pio_hw_t *pio, uint sm) { (void)pio; (void)sm; }
void pio_sm_clkdiv_restart(pio_hw_t *pio, uint sm) { (void)pio; (void)sm; }
void pio_sm_put(pio_hw_t *pio, uint sm, uint32_t data) { (void)pio; (void)sm; (void)data; }
void pio_sm_exec(pio_hw_t *pio, uint sm, uint16_t instruction) { (void)pio; (void)sm; (void)instruction; }
uint pio_get_dreq(pio_hw_t *pio, uint sm, bool is_tx) { (void)pio; (void)sm; return is_tx ? 0u : 1u; }
uint16_t pio_encode_set(uint destination, uint value) { (void)destination; (void)value; return 0u; }
uint16_t pio_encode_out(uint destination, uint bit_count) { (void)destination; (void)bit_count; return 0u; }
uint16_t pio_encode_jmp(uint address) { (void)address; return 0u; }
uint16_t pio_encode_mov(uint destination, uint source) { (void)destination; (void)source; return 0u; }
void sm_config_set_in_pins(pio_sm_config *config, uint pin_base) { (void)config; (void)pin_base; }
void sm_config_set_sideset_pins(pio_sm_config *config, uint pin_base) { (void)config; (void)pin_base; }

int dma_claim_unused_channel(int required) {
    (void)required;
    for (int i = 0; i < DMA_CHANNEL_COUNT; i++) {
        if (!claimed_dma[i]) { claimed_dma[i] = 1; return i; }
    }
    return PICO_ERROR_INSUFFICIENT_RESOURCES;
}
void dma_channel_unclaim(int channel) { if (channel >= 0 && channel < DMA_CHANNEL_COUNT) claimed_dma[channel] = 0; }
int dma_channel_is_claimed(int channel) { return (channel >= 0 && channel < DMA_CHANNEL_COUNT) ? claimed_dma[channel] : 0; }
bool dma_channel_is_busy(uint channel) { return channel < DMA_CHANNEL_COUNT ? dma_busy[channel] : false; }
void dma_channel_set_irq0_enabled(uint channel, bool enabled) { if (channel < DMA_CHANNEL_COUNT) { if (enabled) fake_dma_hw.inte0 |= 1u << channel; else fake_dma_hw.inte0 &= ~(1u << channel); } }
void dma_channel_acknowledge_irq0(uint channel) { if (channel < DMA_CHANNEL_COUNT) { fake_dma_hw.ints0 &= ~(1u << channel); fake_dma_hw.intr &= ~(1u << channel); } }
dma_channel_hw_t *dma_channel_hw_addr(uint channel) { return channel < DMA_CHANNEL_COUNT ? &fake_dma_hw.ch[channel] : NULL; }
dma_channel_config dma_channel_get_default_config(uint channel) { dma_channel_config config = {0}; (void)channel; return config; }
void channel_config_set_dreq(dma_channel_config *config, uint dreq) { config->dreq = dreq; }
void channel_config_set_transfer_data_size(dma_channel_config *config, uint size) { config->transfer_size = (uint8_t)size; }
void channel_config_set_write_increment(dma_channel_config *config, bool increment) { config->write_increment = increment; }
void channel_config_set_read_increment(dma_channel_config *config, bool increment) { config->read_increment = increment; }
void channel_config_set_bswap(dma_channel_config *config, bool byte_swap) { config->byte_swap = byte_swap; }

static bool is_pio_txf(const volatile void *address)
{
    for (size_t pio = 0u; pio < NUM_PIOS; ++pio) {
        for (size_t sm = 0u; sm < NUM_PIO_STATE_MACHINES; ++sm) {
            if (address == &fake_pio_instances[pio].txf[sm]) {
                return true;
            }
        }
    }
    return false;
}

static bool is_pio_rxf(const volatile void *address)
{
    for (size_t pio = 0u; pio < NUM_PIOS; ++pio) {
        for (size_t sm = 0u; sm < NUM_PIO_STATE_MACHINES; ++sm) {
            if (address == &fake_pio_instances[pio].rxf[sm]) {
                return true;
            }
        }
    }
    return false;
}

void dma_channel_configure(uint channel, const dma_channel_config *config,
                           volatile void *write_addr,
                           const volatile void *read_addr,
                           size_t transfer_count, bool trigger)
{
    (void)config;
    if (channel >= DMA_CHANNEL_COUNT) {
        return;
    }
    fake_dma_hw.ch[channel].write_addr = (uintptr_t)write_addr;
    fake_dma_hw.ch[channel].read_addr = (uintptr_t)read_addr;
    fake_dma_hw.ch[channel].transfer_count = (uint32_t)transfer_count;
    if (is_pio_txf(write_addr) && read_addr != NULL) {
        size_t available = sizeof(dma_tx_trace) - dma_tx_length;
        size_t count = transfer_count < available ? transfer_count : available;
        memcpy(&dma_tx_trace[dma_tx_length], (const void *)read_addr, count);
        dma_tx_length += count;
    } else if (is_pio_rxf(read_addr) && write_addr != NULL) {
        memset((void *)write_addr, dma_rx_value, transfer_count);
    }
    dma_busy[channel] = !trigger;
}

void gpio_init(uint gpio) { (void)gpio; }
void gpio_set_function(uint gpio, int fn) { (void)gpio; (void)fn; }
void gpio_set_dir(uint gpio, int out) { (void)gpio; (void)out; }
void gpio_put(uint gpio, bool value) { (void)gpio; (void)value; }
void gpio_set_pulls(uint gpio, bool up, bool down) { (void)gpio; (void)up; (void)down; }
void gpio_pull_down(uint gpio) { (void)gpio; }
void gpio_pull_up(uint gpio) { (void)gpio; }
void gpio_set_input_hysteresis_enabled(uint gpio, bool enabled) { (void)gpio; (void)enabled; }
void gpio_set_irq_enabled_with_callback(uint gpio, uint32_t event_mask, int enabled,
                                        gpio_irq_callback_t callback) { (void)gpio; (void)event_mask; (void)enabled; (void)callback; }
void gpio_set_irq_enabled(uint gpio, uint32_t event_mask, int enabled)
{
    if (gpio < 30u) {
        if (enabled) gpio_enabled[gpio] |= event_mask;
        else gpio_enabled[gpio] &= ~event_mask;
    }
}
void gpio_add_raw_irq_handler(uint gpio, void (*handler)(void))
{
    (void)gpio;
    gpio_raw_handler = handler;
    ++gpio_raw_add_calls;
}
void gpio_remove_raw_irq_handler(uint gpio, void (*handler)(void))
{
    (void)gpio;
    if (gpio_raw_handler == handler) gpio_raw_handler = NULL;
    ++gpio_raw_remove_calls;
}
uint32_t gpio_get_irq_event_mask(uint gpio)
{
    return gpio < 30u ? gpio_irq_events[gpio] : 0u;
}
void gpio_acknowledge_irq(uint gpio, uint32_t event_mask)
{
    if (gpio < 30u) {
        gpio_acknowledged[gpio] |= event_mask;
        gpio_irq_events[gpio] &= ~event_mask;
    }
}

uint32_t save_and_disable_interrupts(void) { return 0; }
void restore_interrupts(uint32_t status) { (void)status; }
void irq_set_enabled(int irq_num, int enabled) { (void)irq_num; (void)enabled; }

void sleep_ms(uint32_t ms) { fake_us += (int64_t)ms * 1000; }
void busy_wait_us_32(uint32_t us) { fake_us += us; }
void busy_wait_us(uint64_t us) { fake_us += (int64_t)us; }
void sleep_us(uint64_t us) { fake_us += (int64_t)us; }
void busy_wait_at_least_cycles(uint32_t cycles) { (void)cycles; }
void hw_write_masked(volatile uint32_t *address, uint32_t values, uint32_t write_mask) { *address = (*address & ~write_mask) | (values & write_mask); }
void hw_set_bits(volatile uint32_t *address, uint32_t mask) { *address |= mask; }
void panic_unsupported(void) { ++panic_unsupported_count; }
void tight_loop_contents(void) {}
void stdio_init_all(void) {}
int64_t get_absolute_time(void) { return fake_us; }
