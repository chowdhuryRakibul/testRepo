/*
* mcp23017.c
*
* modified by Matthew Friesen 2019 based on:
* Copyright (c) 2015, eadf (https://github.com/eadf)
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#include "mcp23017.h"

#define MCP23017_ADDRESS 0x20

// registers

#define MCP23017_IODIRA 0x00
#define MCP23017_IPOLA 0x02
#define MCP23017_GPINTENA 0x04
#define MCP23017_DEFVALA 0x06
#define MCP23017_INTCONA 0x08
#define MCP23017_IOCONA 0x0A
#define MCP23017_GPPUA 0x0C
#define MCP23017_INTFA 0x0E
#define MCP23017_INTCAPA 0x10
#define MCP23017_GPIOA 0x12
#define MCP23017_OLATA 0x14

#define MCP23017_IODIRB 0x01
#define MCP23017_IPOLB 0x03
#define MCP23017_GPINTENB 0x05
#define MCP23017_DEFVALB 0x07
#define MCP23017_INTCONB 0x09
#define MCP23017_IOCONB 0x0B
#define MCP23017_GPPUB 0x0D
#define MCP23017_INTFB 0x0F
#define MCP23017_INTCAPB 0x11
#define MCP23017_GPIOB 0x13
#define MCP23017_OLATB 0x15

#define MCP23017_INT_ERR 255

#define MCP23017_READ_FLAG 0x01


// forward declarations
static uint8_t mcp23017_bitForPin(uint8_t pin);
static uint8_t mcp23017_regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr);
static bool mcp23017_updateRegisterBit(MCP23017_config *self, uint8_t deviceAddr, uint8_t pin, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr);
static uint8_t mcp23017_bitWrite(uint8_t value, uint8_t bitNumber, uint8_t setValue);
//static bool mcp23017_writeRegister(MCP23017_config *self, uint8_t deviceAddr, uint8_t regAddr, uint8_t regValue);
//static bool mcp23017_readRegister(MCP23017_config *self, uint8_t deviceAddr, uint8_t regAddr, uint8_t *rv);

I2CConfig mcp23017_i2ccfg = {
                                   OPMODE_I2C,
                                   400000,
                                   FAST_DUTY_CYCLE_2
};

/**
 * initiates the device. Sets the SCL and SDA pins
 */
bool mcp23017_init(MCP23017_config *self) {
  // i2c_master_gpio_init() and i2c_master_init(void) must already have been used.
  //#if HTS221_SHARED_I2C
    i2cAcquireBus(self->i2cp);
  //#endif /* HTS221_SHARED_I2C */

    /* Intializing the I2C. */
    i2cStart(self->i2cp, &mcp23017_i2ccfg);

  //#if HTS221_SHARED_I2C
      i2cReleaseBus(self->i2cp);
  //#endif /* HTS221_SHARED_I2C */
  return true;
}

/**
 * Initializes the MCP23017 given its HW selected address, see datasheet for Address selection.
 */
bool mcp23017_pinModeAB(MCP23017_config *self, uint8_t deviceAddr, MCP23017_PinMode pinmode) {

  bool rv = true;
  rv &= mcp23017_writeRegister(self, deviceAddr, MCP23017_IODIRA, (pinmode==MCP23017_INPUT)?0xff:0x00);
  rv &= mcp23017_writeRegister(self, deviceAddr, MCP23017_IPOLA, 0x00);
  rv &= mcp23017_writeRegister(self, deviceAddr, MCP23017_IODIRB, (pinmode==MCP23017_INPUT)?0xff:0x00);
  rv &= mcp23017_writeRegister(self, deviceAddr, MCP23017_IPOLB, 0x00);
  return rv;
}

bool mcp23017_digitalWriteAB(MCP23017_config *self, uint8_t deviceAddr, uint16_t data) {
  bool rv = true;
  rv &= mcp23017_writeRegister(self, deviceAddr, MCP23017_GPIOA, data);
  rv &= mcp23017_writeRegister(self, deviceAddr, MCP23017_GPIOB, data>>8);
  return rv;
}

bool mcp23017_digitalWrite(const MCP23017_config *self, uint8_t deviceAddr, uint8_t pin, bool data) {
  uint8_t gpio;
  bool rv = true;
  uint8_t bit=mcp23017_bitForPin(pin);

  // read the current GPIO output latches
  uint8_t regAddr=mcp23017_regForPin(pin,MCP23017_OLATA,MCP23017_OLATB);
  rv &= mcp23017_readRegister(self, deviceAddr, regAddr, &gpio);
  // set the pin and direction
  gpio = mcp23017_bitWrite(gpio,bit,data);

  // write the new GPIO
  regAddr=mcp23017_regForPin(pin,MCP23017_GPIOA,MCP23017_GPIOB);
  rv &= mcp23017_writeRegister(self, deviceAddr, regAddr,gpio);
  return rv;
}

bool mcp23017_digitalRead(MCP23017_config *self, uint8_t deviceAddr, uint8_t pin, bool* data) {
  uint8_t bit=mcp23017_bitForPin(pin);
  uint8_t regValue = 0;
  uint8_t regAddr=mcp23017_regForPin(pin,MCP23017_GPIOA,MCP23017_GPIOB);
  bool rv =mcp23017_readRegister(self, deviceAddr, regAddr, &regValue);
  *data = (regValue >> bit) & 0x1;
  return rv;
}

/**
 * Sets the pin mode to either MCP23017_INPUT or MCP23017_OUTPUT
 */
bool mcp23017_pinMode(MCP23017_config *self, uint8_t deviceAddr, uint8_t pin, MCP23017_PinMode pinmode) {
  return mcp23017_updateRegisterBit(self, deviceAddr, pin,(pinmode==MCP23017_INPUT),MCP23017_IODIRA,MCP23017_IODIRB);
}

/**
 * Sets the pin mode on all the pins on bank A to MCP23017_INPUT or MCP23017_OUTPUT
 */
bool mcp23017_pinModeA(MCP23017_config *self, uint8_t deviceAddr, MCP23017_PinMode pinmode){
  bool rv = true;
  rv &= mcp23017_writeRegister(self, deviceAddr, MCP23017_IODIRA, (pinmode==MCP23017_INPUT)?0xff:0x00);
  rv &= mcp23017_writeRegister(self, deviceAddr, MCP23017_IPOLA, 0x00);
  return rv;
}

/**
 * Sets the output data of whole A bank
 */
bool mcp23017_digitalWriteA(MCP23017_config *self, uint8_t deviceAddr, uint8_t data) {
  return mcp23017_writeRegister(self, deviceAddr, MCP23017_GPIOA, data);
}

/**
 * reads the value of the whole A bank
 */
bool mcp23017_digitalReadA(MCP23017_config *self, uint8_t deviceAddr, uint8_t* data) {
  return mcp23017_readRegister(self, deviceAddr, MCP23017_GPIOA, data);
}

/**
 * Sets the pin mode on all the pins on bank B to MCP23017_INPUT or MCP23017_OUTPUT
 */
bool mcp23017_pinModeB(MCP23017_config *self, uint8_t deviceAddr, MCP23017_PinMode pinmode){
  bool rv = true;
  rv &= mcp23017_writeRegister(self, deviceAddr, MCP23017_IODIRB, (pinmode==MCP23017_INPUT)?0xff:0x00);
  rv &= mcp23017_writeRegister(self, deviceAddr, MCP23017_IPOLB, 0x00);
  return rv;
}

/**
 * reads the value of the whole B bank
 */
bool mcp23017_digitalReadB(MCP23017_config *self, uint8_t deviceAddr, uint8_t* data) {
  return mcp23017_readRegister(self, deviceAddr, MCP23017_GPIOB, data);
}

/**
 * Sets the output data of whole B bank
 */
bool mcp23017_digitalWriteB(MCP23017_config *self, uint8_t deviceAddr, uint8_t data) {
  return mcp23017_writeRegister(self, deviceAddr, MCP23017_GPIOB, data);
}

/**
 * Reads a given register
 */
bool mcp23017_readRegister(MCP23017_config *self, uint8_t deviceAddr, uint8_t regAddr, uint8_t *regValue) {
  bool rv = true;

  deviceAddr &= 0x7;
  deviceAddr |= MCP23017_ADDRESS;
  uint8_t txbuf[1];
  uint8_t rxbuf[1];
//  uint8_t newRegValue;
  msg_t result;

  txbuf[0] = regAddr;

  i2cAcquireBus(self->i2cp);
  result = i2cMasterTransmitTimeout(self->i2cp, deviceAddr, txbuf, 1,rxbuf, 1, TIME_INFINITE);
  i2cReleaseBus(self->i2cp);


  *regValue = rxbuf[0];
  return rv;
}

/**
 * Writes a given register
 */

bool mcp23017_writeRegister(MCP23017_config *self, uint8_t deviceAddr, uint8_t regAddr, uint8_t regValue) {
  bool rv = true;
  deviceAddr &= 0x7;
  deviceAddr |= MCP23017_ADDRESS;
  msg_t result;
//  uint8_t txbuf[2],rxbuf[1];
  uint8_t txbuf[2];
  uint8_t newRegValue;
  txbuf[0] = regAddr;
  txbuf[1] = regValue;

  i2cAcquireBus(self->i2cp);
  result = i2cMasterTransmitTimeout(self->i2cp, deviceAddr, txbuf, 2, NULL, 0, TIME_INFINITE);
  //chThdSleepMicroseconds(50);
  result = i2cMasterTransmitTimeout(self->i2cp, deviceAddr, txbuf, 1,&newRegValue, 1, TIME_INFINITE);
  i2cReleaseBus(self->i2cp);
  if( newRegValue != regValue )
  {
#ifdef debug_mcp
    chprintf((BaseSequentialStream*)&SD6, "\n\rmcp write register failed. Wrote 0x%x to address 0x%x, read 0x%x\n\r", regValue,regAddr,newRegValue);
#endif
    return false;
  }
  else
  {
    return rv;
  }
}

/**
 * Register address, port dependent, for a given PIN
 */
static uint8_t mcp23017_regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr) {
  return(pin<8) ?portAaddr:portBaddr;
}

/**
 * Bit number associated to a give Pin
 */
static uint8_t mcp23017_bitForPin(uint8_t pin) {
  return pin%8;
}

/**
 * Helper to update a single bit of an A/B register.
 * - Reads the current register value
 * - Writes the new register value
 */
static bool mcp23017_updateRegisterBit(MCP23017_config *self, uint8_t deviceAddr, uint8_t pin, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr) {
  uint8_t regValue;
  bool rv = true;
  uint8_t regAddr=mcp23017_regForPin(pin, portAaddr, portBaddr);
  //os_printf("regAddr =%d\n", regAddr);
  uint8_t bit=mcp23017_bitForPin(pin);
  rv &= mcp23017_readRegister(self, deviceAddr, regAddr, &regValue);
  //os_printf("regValue b4=%d\n", regValue);
  // set the value for the particular bit
  regValue = mcp23017_bitWrite(regValue, bit, pValue);
  //os_printf("regValue after =%d\n", regValue);
  rv &= mcp23017_writeRegister(self, deviceAddr, regAddr, regValue);
  return rv;
}

/**
 * returns a copy of the 'value' byte with the 'bitNumber' bit 0 or 1 depending on 'setValue'
 */
static uint8_t mcp23017_bitWrite(uint8_t value, uint8_t bitNumber, uint8_t setValue) {
  setValue = setValue&1;
  if (setValue) {
    // raise the bit
    return value | 1<<bitNumber;
  } else {
    // lower the bit
    return value & ~(1<<bitNumber);
  }
}
