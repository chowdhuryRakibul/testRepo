/*
 * ads1282.c
 *
 *  Created on: Sep 16, 2019
 *      Author: MatthewFriesen
 */

/**
 * @file    ads1282.c
 * @brief   ads1282 ADC interface code
 *
 * @addtogroup ads1282
 * @{
 */

#include "ads1282.h"



/*
 * SPI configuration for ADS1282 ADC interface
 * flck/128 clock rate, max rate is approx. 2Mhz
 */

const SPIConfig ads1282ADC_spicfg = {
  false,
  NULL,
  GPIOA,
  4,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 ,
  0
};

void ads1282Init( ads1282Config *config )
{
  /*
   * config ADC spi interface
   */
  //palSetPadMode(GPIOB, 5U, PAL_MODE_ALTERNATE(5)| PAL_STM32_OSPEED_HIGHEST); //SPI1_MOSI held low for now
  palSetPadMode(GPIOB, 5U, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST); //SPI1_MOSI held low for now
  palSetPadMode(GPIOA, 6U, PAL_MODE_ALTERNATE(5)| PAL_STM32_OSPEED_HIGHEST); //SPI1_MISO
  palSetPadMode(GPIOA, 5U, PAL_MODE_ALTERNATE(5)| PAL_STM32_OSPEED_HIGHEST); //SPI1_CLK
  palSetPadMode(GPIOA, 4U, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST); //CS

  spiStart(&SPID1,  &ads1282ADC_spicfg);

  palClearPad(GPIOB, 5U); /* drive SPI_MOSI low for ADC */

  /*
  * Enable hardware interrupt events on rising edge of DRDY of ADC
  */
  palEnablePadEvent(GPIOG, 15U, PAL_EVENT_MODE_FALLING_EDGE);
  /* Assigning a callback to ADC DRDY rising edge event, passing no arguments.*/
  // palSetPadCallback(GPIOA, 0U, ads1282_DRDY_CB, NULL);
  //byte adcReceiveBuff[4];
  uint32_t adcValue;
  while (true) {
    /* wait for ADC DRDY pin to go high */
    palWaitPadTimeout(GPIOG, 15U, TIME_INFINITE);
    /* data ready, so read 4 bytes over SPI */
    spiReceive(&SPID1, sizeof(adcValue), &adcValue);
    //adcValue = adcReceiveBuff[0];
    adcValue = __builtin_bswap32(adcValue); //swap from big-endian received from adc to little endian
    chprintf((BaseSequentialStream*)&SD6, "\ndata 0x%x\n\r", adcValue);
  }
}
