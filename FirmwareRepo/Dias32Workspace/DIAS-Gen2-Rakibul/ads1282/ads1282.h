/*
    Copyright (C) 2019 Matthew Friesn

*/

/**
 * @file    ads1282.h
 * @brief   ads1282 ADC interface header
 *
 * @addtogroup ads1282
 * @{
 */

#ifndef ADS1282_H
#define ADS1282_H


#include "ch.h"
#include "hal.h"


/*==========================================================================*/
/* Driver data structures and types.                                        */
/*==========================================================================*/

/**
 * @brief   ads1282 config structure.
 */
struct ads1282Config {
  SPIConfig *ads1282ADC_spicfg

  /** @brief BaseSensor Virtual Methods Table.                              */
  const struct BaseSensorVMT      *vmt_basesensor;
  /** @brief BaseBarometer Virtual Methods Table.                           */
  const struct BaseBarometerVMT   *vmt_basebarometer;
  /** @brief BaseThermometer Virtual Methods Table.                         */
  const struct BaseThermometerVMT *vmt_basethermometer;
  _bmp085_data;
};

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void ads1282Init(ads1282Config *config);
#ifdef __cplusplus
}
#endif

#endif /* ADS1282_H */
