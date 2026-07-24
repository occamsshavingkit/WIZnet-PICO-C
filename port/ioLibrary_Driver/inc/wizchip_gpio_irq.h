/**
    Copyright (c) 2022 WIZnet Co.,Ltd

    SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef _WIZCHIP_GPIO_IRQ_H_
#define _WIZCHIP_GPIO_IRQ_H_

#include <stdbool.h>

#include "socket.h"
#include "wizchip_spi.h"

/**
    ----------------------------------------------------------------------------------------------------
    Macros
    ----------------------------------------------------------------------------------------------------
*/

/**
    ----------------------------------------------------------------------------------------------------
    Functions
    ----------------------------------------------------------------------------------------------------
*/
typedef void (*wizchip_gpio_irq_cb_t)(uint8_t sn, sockint_kind events, void *context);

/*! \brief Register task-context interrupt dispatch for one socket
    \ingroup wizchip_gpio_irq

    The event mask and chip interrupt masks are merged with masks owned by other users.

    \param sn socket number
    \param event_mask socket interrupt events to dispatch
    \param callback callback invoked by wizchip_gpio_interrupt_dispatch()
    \param context caller context passed to callback
    \return PICO_OK on success, or a negative error code
*/
int wizchip_gpio_interrupt_register(uint8_t sn, sockint_kind event_mask,
                                    wizchip_gpio_irq_cb_t callback, void *context);

/*! \brief Unregister task-context interrupt dispatch for one socket
    \ingroup wizchip_gpio_irq

    \param sn socket number
    \return PICO_OK on success, or a negative error code
*/
int wizchip_gpio_interrupt_unregister(uint8_t sn);

/*! \brief Report whether the raw GPIO ISR recorded pending socket work
    \ingroup wizchip_gpio_irq

    \return true when wizchip_gpio_interrupt_dispatch() should be called
*/
bool wizchip_gpio_interrupt_pending(void);

/*! \brief Dispatch pending WIZchip socket interrupts in task context
    \ingroup wizchip_gpio_irq

    \return PICO_OK on success, or a negative error code
*/
int wizchip_gpio_interrupt_dispatch(void);

/*! \brief Register the legacy no-argument callback for one socket
    \ingroup wizchip_gpio_irq

    \param sn socket number
    \param callback callback invoked by wizchip_gpio_interrupt_dispatch()
    \return PICO_OK on success, or a negative error code
*/
int wizchip_gpio_interrupt_initialize(uint8_t sn, void (*callback)(void));

#endif /* _WIZCHIP_GPIO_IRQ_H_ */
