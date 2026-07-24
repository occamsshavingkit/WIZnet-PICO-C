#ifndef FAKE_PICO_SDK_H
#define FAKE_PICO_SDK_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

typedef unsigned int uint;

#ifdef __CONCAT
#undef __CONCAT
#endif
#define FAKE_CONCAT_INNER(left, right) left##right
#define __CONCAT(left, right) FAKE_CONCAT_INNER(left, right)
#define count_of(array) (sizeof(array) / sizeof((array)[0]))
#ifndef static_assert
#define static_assert(condition, message) _Static_assert(condition, message)
#endif
#define __compiler_memory_barrier() __asm__ volatile("" ::: "memory")

#define PICO_OK 0
#define PICO_ERROR_NONE 0
#define PICO_ERROR_TIMEOUT (-1)
#define PICO_ERROR_GENERIC (-2)
#define PICO_ERROR_INVALID_ARG (-3)
#define PICO_ERROR_NOT_PERMITTED (-4)
#define PICO_ERROR_INVALID_STATE (-5)
#define PICO_ERROR_NO_DATA (-6)
#define PICO_ERROR_INSUFFICIENT_RESOURCES (-8)
#define PICO_ERROR_RESOURCE_IN_USE (-9)
#define PICO_ERROR_IO (-12)

#define PICO_DEFAULT_SPI_INSTANCE 0

typedef uint32_t pio_program_t;

#define PIO_SM0 0
#define PIO_SM1 1
#define PIO_SM2 2
#define PIO_SM3 3

#define NUM_PIOS 2
#define NUM_PIO_STATE_MACHINES 4
#define PIO_INSTRUCTION_COUNT 32
#define PIO_ORIGIN_ANY ((uint)-1)

typedef struct {
    uint32_t instr;
} pio_asm_program_t;

struct pio_asm_default_config {
    uint32_t wrap_target;
    uint32_t wrap;
    uint32_t out_pins;
    uint32_t set_pins;
    uint32_t sideset_count;
    uint32_t sideset_opt;
    uint32_t clkdiv_int;
    uint32_t clkdiv_frac;
    uint32_t fifo_join;
};

typedef struct pio_asm_default_config pio_sm_config;

typedef struct {
    uint32_t ctrl;
    uint32_t fstat;
    volatile uint32_t txf[NUM_PIO_STATE_MACHINES];
    volatile uint32_t rxf[NUM_PIO_STATE_MACHINES];
    volatile uint32_t fdebug;
    volatile uint32_t input_sync_bypass;
} pio_hw_t;

extern pio_hw_t fake_pio_instances[NUM_PIOS];
#define pio0 (&fake_pio_instances[0])
#define pio1 (&fake_pio_instances[1])

#define PIO_CTRL_SM_ENABLE_LSB 0

typedef int irq_num_t;

#define DMA_CHANNEL_COUNT 12

typedef struct {
    volatile uint32_t read_addr;
    volatile uint32_t write_addr;
    volatile uint32_t transfer_count;
    volatile uint32_t ctrl_trig;
    volatile uint32_t al1_ctrl;
} dma_channel_hw_t;

typedef struct {
    uint32_t dreq;
    uint8_t transfer_size;
    bool read_increment;
    bool write_increment;
    bool byte_swap;
} dma_channel_config;

typedef struct {
    volatile uint32_t inte0;
    volatile uint32_t ints0;
    volatile uint32_t intr;
    volatile uint32_t abort;
    dma_channel_hw_t ch[DMA_CHANNEL_COUNT];
} dma_hw_t;

extern dma_hw_t fake_dma_hw;
#define dma_hw (&fake_dma_hw)

typedef struct {
    volatile uint32_t io[30];
} pads_bank0_hw_t;

extern pads_bank0_hw_t fake_pads_bank0_hw;
#define pads_bank0_hw (&fake_pads_bank0_hw)

#define DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS (1u << 31)
#define DMA_SIZE_8 0u
#define PADS_BANK0_GPIO0_DRIVE_VALUE_12MA 3u
#define PADS_BANK0_GPIO0_DRIVE_LSB 4u
#define PADS_BANK0_GPIO0_DRIVE_BITS (3u << PADS_BANK0_GPIO0_DRIVE_LSB)
#define PADS_BANK0_GPIO0_SLEWFAST_LSB 0u
#define PADS_BANK0_GPIO0_SLEWFAST_BITS 1u
#define clk_sys 0
#define pio_x 0u
#define pio_y 1u
#define pio_pins 2u
#define pio_null 3u

#define DMA_CH0_IRQ_NUM      0
#define DMA_CH0_IRQ_1_NUM    1
#define DMA_CH1_IRQ_2_NUM    2
#define DMA_CH0_IRQ_3_NUM    3

#define GPIO_OUT 1
#define GPIO_IN  0
#define GPIO_FUNC_SPI 1
#define GPIO_FUNC_PIO0 2
#define GPIO_FUNC_PIO1 3
#define GPIO_FUNC_SIO 5

enum gpio_irq_level {
    GPIO_IRQ_LEVEL_LOW = 0x1u,
    GPIO_IRQ_LEVEL_HIGH = 0x2u,
    GPIO_IRQ_EDGE_FALL = 0x4u,
    GPIO_IRQ_EDGE_RISE = 0x8u,
};

typedef void (*gpio_irq_callback_t)(uint gpio, uint32_t event_mask);

#define PICO_DEFAULT_LED_PIN 25

#define bi_decl(...) ((void)0)
#define bi_1pin_with_name(...) 0
#define bi_3pins_with_func(...) 0
#define binary_info_claim(...)

typedef struct {
    uint32_t enable;
} irq_handler_t;

struct repeating_timer {
    uint64_t delay_us;
    void *user_data;
};

typedef bool (*repeating_timer_callback_t)(struct repeating_timer *t);

typedef struct mutex {
    int locked;
} mutex_t;

typedef struct {
    int entered;
} critical_section_t;

int64_t fake_time_us_64(void);
uint64_t time_us_64(void);
void fake_time_tick_us(uint64_t us);
void fake_pico_sdk_reset(void);
size_t fake_dma_tx_trace_length(void);
const uint8_t *fake_dma_tx_trace_data(void);
unsigned int fake_panic_unsupported_calls(void);
void fake_dma_set_rx_value(uint8_t value);
void fake_dma_clear_tx_trace(void);
void fake_dma_set_ahb_error(uint channel, bool enabled);
unsigned int fake_gpio_raw_handler_add_calls(void);
unsigned int fake_gpio_raw_handler_remove_calls(void);
uint32_t fake_gpio_enabled_events(uint gpio);
uint32_t fake_gpio_acknowledged_events(uint gpio);
void fake_gpio_trigger_irq(uint gpio, uint32_t events);

mutex_t *mutex_init(mutex_t *m);
void mutex_enter_blocking(mutex_t *m);
void mutex_exit(mutex_t *m);
int mutex_try_enter(mutex_t *m, uint32_t *owner);

void critical_section_init(critical_section_t *cs);
void critical_section_enter_blocking(critical_section_t *cs);
void critical_section_exit(critical_section_t *cs);

uint32_t clock_get_hz(int clk_id);

bool pio_claim_free_sm_and_add_program(const pio_program_t *prog,
                                       pio_hw_t **pio, uint *sm,
                                       uint *pio_offset);
pio_sm_config pio_get_default_sm_config(void);
void pio_sm_set_enabled(pio_hw_t *pio, int sm, int enabled);
void pio_sm_clear_enabled(pio_hw_t *pio, int sm);
int pio_sm_is_enabled(pio_hw_t *pio, int sm);
void pio_remove_program(pio_hw_t *pio, const pio_program_t *prog, uint offset);
int pio_claim_unused_sm(pio_hw_t *pio, int required);
void pio_sm_unclaim(pio_hw_t *pio, int sm);
void pio_sm_claim(pio_hw_t *pio, int sm);
void sm_config_set_wrap(pio_sm_config *c, uint wrap_target, uint wrap);
void sm_config_set_out_pins(pio_sm_config *c, uint out_base, uint out_count);
void sm_config_set_set_pins(pio_sm_config *c, uint set_base, uint set_count);
void sm_config_set_sideset(pio_sm_config *c, uint sideset_count, int optional, int pindirs);
void sm_config_set_clkdiv_int_frac(pio_sm_config *c, uint16_t div_int, uint8_t div_frac);
void sm_config_set_in_shift(pio_sm_config *c, int shift_right, int auto_push, uint push_threshold);
void sm_config_set_out_shift(pio_sm_config *c, int shift_right, int auto_pull, uint pull_threshold);
void sm_config_set_fifo_join(pio_sm_config *c, int join);
void sm_config_set_out_special(pio_sm_config *c, int sticky, int has_enable_pin, uint enable_pin_index);
void pio_sm_init(pio_hw_t *pio, int sm, uint initial_pc, const pio_sm_config *config);
void pio_remove_program_and_unclaim_sm(const pio_program_t *prog,
                                       pio_hw_t *pio, uint sm, uint offset);
bool pio_sm_is_tx_fifo_empty(pio_hw_t *pio, uint sm);
bool pio_sm_is_rx_fifo_empty(pio_hw_t *pio, uint sm);
void pio_sm_clear_fifos(pio_hw_t *pio, uint sm);
void pio_sm_set_config(pio_hw_t *pio, uint sm,
                       const pio_sm_config *config);
void pio_sm_set_wrap(pio_hw_t *pio, uint sm, uint wrap_target, uint wrap);
void pio_sm_set_pindirs_with_mask(pio_hw_t *pio, uint sm,
                                  uint32_t values, uint32_t mask);
void pio_sm_set_consecutive_pindirs(pio_hw_t *pio, uint sm, uint pin_base,
                                    uint pin_count, bool is_out);
void pio_sm_restart(pio_hw_t *pio, uint sm);
void pio_sm_clkdiv_restart(pio_hw_t *pio, uint sm);
void pio_sm_put(pio_hw_t *pio, uint sm, uint32_t data);
void pio_sm_exec(pio_hw_t *pio, uint sm, uint16_t instruction);
uint pio_get_dreq(pio_hw_t *pio, uint sm, bool is_tx);
uint16_t pio_encode_set(uint destination, uint value);
uint16_t pio_encode_out(uint destination, uint bit_count);
uint16_t pio_encode_jmp(uint address);
uint16_t pio_encode_mov(uint destination, uint source);
void sm_config_set_in_pins(pio_sm_config *config, uint pin_base);
void sm_config_set_sideset_pins(pio_sm_config *config, uint pin_base);

uint pio_add_program_at_offset(pio_hw_t *pio, const pio_program_t *prog, uint offset);

int dma_claim_unused_channel(int required);
void dma_channel_unclaim(int channel);
int dma_channel_is_claimed(int channel);
bool dma_channel_is_busy(uint channel);
void dma_channel_set_irq0_enabled(uint channel, bool enabled);
void dma_channel_acknowledge_irq0(uint channel);
dma_channel_hw_t *dma_channel_hw_addr(uint channel);
dma_channel_config dma_channel_get_default_config(uint channel);
void channel_config_set_dreq(dma_channel_config *config, uint dreq);
void channel_config_set_transfer_data_size(dma_channel_config *config,
                                           uint size);
void channel_config_set_write_increment(dma_channel_config *config,
                                        bool increment);
void channel_config_set_read_increment(dma_channel_config *config,
                                       bool increment);
void channel_config_set_bswap(dma_channel_config *config, bool byte_swap);
void dma_channel_configure(uint channel, const dma_channel_config *config,
                           volatile void *write_addr,
                           const volatile void *read_addr,
                           size_t transfer_count, bool trigger);

void gpio_init(uint gpio);
void gpio_set_function(uint gpio, int fn);
void gpio_set_dir(uint gpio, int out);
void gpio_put(uint gpio, bool value);
void gpio_set_pulls(uint gpio, bool up, bool down);
void gpio_pull_down(uint gpio);
void gpio_pull_up(uint gpio);
void gpio_set_input_hysteresis_enabled(uint gpio, bool enabled);
void gpio_set_irq_enabled_with_callback(uint gpio, uint32_t event_mask, int enabled,
                                        gpio_irq_callback_t callback);
void gpio_set_irq_enabled(uint gpio, uint32_t event_mask, int enabled);
void gpio_add_raw_irq_handler(uint gpio, void (*handler)(void));
void gpio_remove_raw_irq_handler(uint gpio, void (*handler)(void));
uint32_t gpio_get_irq_event_mask(uint gpio);
void gpio_acknowledge_irq(uint gpio, uint32_t event_mask);

uint32_t save_and_disable_interrupts(void);
void restore_interrupts(uint32_t status);

void irq_set_enabled(int irq_num, int enabled);

void sleep_ms(uint32_t ms);
void busy_wait_us_32(uint32_t us);
void busy_wait_us(uint64_t us);
void sleep_us(uint64_t us);
void busy_wait_at_least_cycles(uint32_t cycles);

void hw_write_masked(volatile uint32_t *address, uint32_t values,
                     uint32_t write_mask);
void hw_set_bits(volatile uint32_t *address, uint32_t mask);
void panic_unsupported(void);

void tight_loop_contents(void);

void stdio_init_all(void);

int64_t get_absolute_time(void);

#endif
