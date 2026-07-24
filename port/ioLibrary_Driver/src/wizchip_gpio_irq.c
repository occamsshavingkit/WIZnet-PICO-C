/**
    Copyright (c) 2022 WIZnet Co.,Ltd

    SPDX-License-Identifier: BSD-3-Clause
*/

/**
    ----------------------------------------------------------------------------------------------------
    Includes
    ----------------------------------------------------------------------------------------------------
*/
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

#include "wizchip_conf.h"
#include "socket.h"
#include "wizchip_gpio_irq.h"

/**
    ----------------------------------------------------------------------------------------------------
    Variables
    ----------------------------------------------------------------------------------------------------
*/
typedef struct {
    wizchip_gpio_irq_cb_t callback;
    void *context;
    uint8_t event_mask;
    uint8_t owned_event_mask;
    bool owns_common_mask;
} wizchip_gpio_irq_registration_t;

static wizchip_gpio_irq_registration_t registrations[_WIZCHIP_SOCK_NUM_];
static void (*legacy_callbacks[_WIZCHIP_SOCK_NUM_])(void);
static uint32_t registered_socket_mask;
static uint32_t pending_socket_mask;
static bool raw_handler_installed;

static void wizchip_gpio_interrupt_callback(void);

static intr_kind wizchip_gpio_socket_mask(uint8_t sn) {
    return (intr_kind)(1UL << (sn + 8U));
}

static void wizchip_gpio_legacy_callback(uint8_t sn, sockint_kind events, void *context) {
    void (*callback)(void);

    (void)events;
    (void)context;
    callback = legacy_callbacks[sn];
    if (callback != NULL) {
        callback();
    }
}

/**
    ----------------------------------------------------------------------------------------------------
    Functions
    ----------------------------------------------------------------------------------------------------
*/
int wizchip_gpio_interrupt_register(uint8_t sn, sockint_kind event_mask,
                                    wizchip_gpio_irq_cb_t callback, void *context) {
    wizchip_gpio_irq_registration_t *registration;
    intr_kind common_mask = (intr_kind)0;
    intr_kind socket_mask;
    sockint_kind current_event_mask = (sockint_kind)0;
    uint8_t base_event_mask;
    uint8_t requested_event_mask;
    uint8_t updated_event_mask;
    int ret;

    if (sn >= _WIZCHIP_SOCK_NUM_ || callback == NULL ||
        event_mask == 0 || (((uint32_t)event_mask & ~(uint32_t)SIK_ALL) != 0U)) {
        return PICO_ERROR_INVALID_ARG;
    }

    registration = &registrations[sn];
    requested_event_mask = (uint8_t)event_mask;
    ret = ctlsocket(sn, CS_GET_INTMASK, &current_event_mask);
    if (ret != SOCK_OK) {
        return ret;
    }
    ret = ctlwizchip(CW_GET_INTRMASK, &common_mask);
    if (ret != PICO_OK) {
        return ret;
    }

    base_event_mask = (uint8_t)current_event_mask & (uint8_t)~registration->owned_event_mask;
    updated_event_mask = base_event_mask | requested_event_mask;
    current_event_mask = (sockint_kind)updated_event_mask;
    ret = ctlsocket(sn, CS_SET_INTMASK, &current_event_mask);
    if (ret != SOCK_OK) {
        return ret;
    }

    socket_mask = wizchip_gpio_socket_mask(sn);
    if ((common_mask & socket_mask) == 0) {
        intr_kind updated_common_mask = (intr_kind)(common_mask | socket_mask);

        ret = ctlwizchip(CW_SET_INTRMASK, &updated_common_mask);
        if (ret != PICO_OK) {
            current_event_mask = (sockint_kind)(base_event_mask |
                (registration->event_mask & registration->owned_event_mask));
            (void)ctlsocket(sn, CS_SET_INTMASK, &current_event_mask);
            return ret;
        }
        registration->owns_common_mask = true;
    }

    registration->callback = callback;
    registration->context = context;
    registration->event_mask = requested_event_mask;
    registration->owned_event_mask = requested_event_mask & (uint8_t)~base_event_mask;
    if (callback != wizchip_gpio_legacy_callback) {
        legacy_callbacks[sn] = NULL;
    }

    __atomic_fetch_or(&registered_socket_mask, 1UL << sn, __ATOMIC_RELEASE);
    if (!raw_handler_installed) {
        gpio_add_raw_irq_handler(PIN_INT, wizchip_gpio_interrupt_callback);
        raw_handler_installed = true;
    }
    gpio_set_irq_enabled(PIN_INT, GPIO_IRQ_EDGE_FALL, true);

    return PICO_OK;
}

static void wizchip_gpio_interrupt_callback(void) {
    uint32_t falling_edge = gpio_get_irq_event_mask(PIN_INT) & GPIO_IRQ_EDGE_FALL;

    if (falling_edge == 0U) {
        return;
    }

    gpio_acknowledge_irq(PIN_INT, falling_edge);
    gpio_set_irq_enabled(PIN_INT, GPIO_IRQ_EDGE_FALL, false);
    __atomic_fetch_or(&pending_socket_mask,
                      __atomic_load_n(&registered_socket_mask, __ATOMIC_ACQUIRE),
                      __ATOMIC_RELEASE);
#if defined(__arm__) || defined(__thumb__)
    __sev();
#endif
}

int wizchip_gpio_interrupt_unregister(uint8_t sn) {
    wizchip_gpio_irq_registration_t *registration;
    intr_kind common_mask = (intr_kind)0;
    intr_kind socket_mask;
    sockint_kind current_event_mask = (sockint_kind)0;
    sockint_kind updated_event_mask;
    int ret;

    if (sn >= _WIZCHIP_SOCK_NUM_) {
        return PICO_ERROR_INVALID_ARG;
    }
    registration = &registrations[sn];
    if (registration->callback == NULL) {
        return PICO_ERROR_INVALID_ARG;
    }

    ret = ctlsocket(sn, CS_GET_INTMASK, &current_event_mask);
    if (ret != SOCK_OK) {
        return ret;
    }
    ret = ctlwizchip(CW_GET_INTRMASK, &common_mask);
    if (ret != PICO_OK) {
        return ret;
    }

    updated_event_mask = (sockint_kind)((uint8_t)current_event_mask &
                                        (uint8_t)~registration->owned_event_mask);
    ret = ctlsocket(sn, CS_SET_INTMASK, &updated_event_mask);
    if (ret != SOCK_OK) {
        return ret;
    }

    socket_mask = wizchip_gpio_socket_mask(sn);
    if (registration->owns_common_mask) {
        intr_kind updated_common_mask = (intr_kind)(common_mask & ~socket_mask);

        ret = ctlwizchip(CW_SET_INTRMASK, &updated_common_mask);
        if (ret != PICO_OK) {
            (void)ctlsocket(sn, CS_SET_INTMASK, &current_event_mask);
            return ret;
        }
    }

    registration->callback = NULL;
    registration->context = NULL;
    registration->event_mask = 0U;
    registration->owned_event_mask = 0U;
    registration->owns_common_mask = false;
    legacy_callbacks[sn] = NULL;
    __atomic_fetch_and(&pending_socket_mask, ~(1UL << sn), __ATOMIC_ACQ_REL);
    __atomic_fetch_and(&registered_socket_mask, ~(1UL << sn), __ATOMIC_ACQ_REL);

    if (__atomic_load_n(&registered_socket_mask, __ATOMIC_ACQUIRE) == 0U) {
        gpio_set_irq_enabled(PIN_INT, GPIO_IRQ_EDGE_FALL, false);
        if (raw_handler_installed) {
            gpio_remove_raw_irq_handler(PIN_INT, wizchip_gpio_interrupt_callback);
            raw_handler_installed = false;
        }
    }

    return PICO_OK;
}

bool wizchip_gpio_interrupt_pending(void) {
    return __atomic_load_n(&pending_socket_mask, __ATOMIC_ACQUIRE) != 0U;
}

int wizchip_gpio_interrupt_dispatch(void) {
    uint32_t pending;
    uint32_t asserted;
    uint8_t sir;
    uint8_t sn;

    pending = __atomic_exchange_n(&pending_socket_mask, 0U, __ATOMIC_ACQ_REL);
    if (pending == 0U) {
        return PICO_OK;
    }

    sir = getSIR();
    asserted = pending & sir;
    for (sn = 0U; sn < _WIZCHIP_SOCK_NUM_; ++sn) {
        wizchip_gpio_irq_registration_t registration;
        uint8_t events;

        if ((asserted & (1UL << sn)) == 0U) {
            continue;
        }

        registration = registrations[sn];
        if (registration.callback == NULL) {
            continue;
        }
        events = getSn_IR(sn) & registration.event_mask;
        if (events == 0U) {
            continue;
        }

        setSn_IR(sn, events);
        registration.callback(sn, (sockint_kind)events, registration.context);
    }

    if (__atomic_load_n(&registered_socket_mask, __ATOMIC_ACQUIRE) != 0U) {
        gpio_set_irq_enabled(PIN_INT, GPIO_IRQ_EDGE_FALL, true);
    }

    return PICO_OK;
}

int wizchip_gpio_interrupt_initialize(uint8_t sn, void (*callback)(void)) {
    void (*previous_callback)(void);
    int ret;

    if (sn >= _WIZCHIP_SOCK_NUM_ || callback == NULL) {
        return PICO_ERROR_INVALID_ARG;
    }

    previous_callback = legacy_callbacks[sn];
    legacy_callbacks[sn] = callback;
    ret = wizchip_gpio_interrupt_register(
        sn,
        (sockint_kind)(SIK_CONNECTED | SIK_DISCONNECTED | SIK_RECEIVED | SIK_TIMEOUT),
        wizchip_gpio_legacy_callback,
        NULL);
    if (ret != PICO_OK) {
        legacy_callbacks[sn] = previous_callback;
    }

    return ret;
}
