/**
 *
 * \file
 *
 * \brief WINC1500 configuration.
 *
 * Copyright (c) 2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef CONF_WINC_H_INCLUDED
#define CONF_WINC_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

//#include "stm32f4xx_hal.h"
#include "hal.h"

/*
   ---------------------------------
   ---------- PIN settings ---------
   ---------------------------------
*/
//extern SPI_HandleTypeDef hspi2;

#define CONF_WINC_RESET_PIN				11U
#define CONF_WINC_ENABLE_PIN			9U
#define CONF_WINC_WAKE_PIN				10U
#define CONF_WINC_IRQ_PIN				12U
#define CONF_WINC_RESET_PORT			GPIOE
#define CONF_WINC_ENABLE_PORT			GPIOE
#define CONF_WINC_WAKE_PORT				GPIOE
#define CONF_WINC_IRQ_PORT				GPIOE

#define CONF_WINC_EXTI_IRQN             EXTI15_10_IRQn

/*
   ---------------------------------
   ---------- SPI settings ---------
   ---------------------------------
*/

#define CONF_WINC_USE_SPI				(1)

/** SPI pin and instance settings. */

/** SPI interrupt pin. */
#define CONF_WINC_SPI_INT_PIN			CONF_WINC_IRQ_PIN
#define CONF_WINC_SPI_INT_PORT			CONF_WINC_IRQ_PORT

// CS = PE8
#define SPI_WIFI_CS_PORT				GPIOE
#define SPI_WIFI_CS_PIN					8U

#define SPI_WIFI_MISO_PIN				2U
#define SPI_WIFI_MISO_PORT				GPIOC

#define SPI_WIFI_MOSI_PIN				3U
#define SPI_WIFI_MOSI_PORT				GPIOC

#define SPI_WIFI_SCK_PIN				10U
#define SPI_WIFI_SCK_PORT				GPIOB

#define SPI_WIFI						SPI2


/** SPI clock. */
#define CONF_WINC_SPI_CLOCK				(12000000)

/*
   ---------------------------------
   --------- Debug Options ---------
   ---------------------------------
*/
#define CONF_WINC_DEBUG                 (0)
#define CONF_WINC_PRINTF				myPrintf

#ifdef __cplusplus
}
#endif

#endif /* CONF_WINC_H_INCLUDED */
