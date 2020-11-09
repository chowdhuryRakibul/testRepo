/*
 * DIAS32.h
 *
 *  Created on: Jan 14, 2020
 *      Author: MatthewFriesen
 */

#ifndef DIAS32_H_
#define DIAS32_H_


#include "stm32_registry.h"

/*
 * for GPS parsing support
 */
#include "GPS.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>


#include "chprintf.h"
#include "shell.h"
#include "ff.h" /* <for FatFS support */

/****************** WiFi Driver *****************/
#include "winc1500/config/conf_winc.h"
#include "winc1500/driver/include/m2m_wifi.h"
#include "winc1500/driver/source/nmasic.h"
#include "winc1500/driver/include/m2m_wifi.h"
#include "winc1500/driver/source/m2m_hif.h"
#include "winc1500/bsp/include/nm_bsp.h"
/************************************************/
/*
 * for i2c mcp23017 io expander
 */
#include "mcp23017/mcp23017.h"


#include "xbee/xbee.h"

/*
 * Node hardware/firmware info
 */

#define FW_TYPE "DIST"
#define FW_VERSION "v2.001"
#define HW_TYPE "ADS"
#define HW_VERSION "v1.40" //need some way to read hardware version

#define STATUS_LED_LINE PAL_LINE(GPIOE, 3U)
#define PPS_LINE PAL_LINE(GPIOE, 0U)    //PPS signal line

#define PWR_CTL_LINE PAL_LINE(GPIOE, 7U) //soft pwr switch control line.
#define MCP_RST_LINE PAL_LINE(GPIOE, 1U) //reset line for mcp i2c expander (active low)

#define RDY_BTN_LINE PAL_LINE(GPIOE, 4U) //ready button input line
#define PWR_BTN_LINE PAL_LINE(GPIOE, 5U) //power button input line

#define PGA_SIG_LINE PAL_LINE(GPIOE, 2U) //PGA control line

#define RDY_LED_PIN_POS 2 // LED1 is connected to GPIO 2 of mcp i2c expander
#define PPS_LED_PIN_POS 3 // LED2 is connected to GPIO 3 of mcp i2c expander

#define SW_5VS_PIN_POS 8+4 // 5Vsw  control pin is gpio 4 of port b of mcp i2c expander
#define SW_5VA_PIN_POS 8+5 // 5V analog control pin is gpio 5 of port b of mcp i2c expander

#define RELAY1_PIN1_POS 8+6 // relay coil pin positions on io expander
#define RELAY1_PIN2_POS 8+7

#define RELAY2_PIN1_POS 8+3
#define RELAY2_PIN2_POS 8+2

#define RELAY3_PIN1_POS 8+0
#define RELAY3_PIN2_POS 8+1

#define RELAY4_PIN1_POS 7
#define RELAY4_PIN2_POS 6

/*
 * node commands
 */
#define CANCEL_CMD 'C' //cancel current state (data or download) and return to idle state
#define START_REC_CMD 'S' //start new recording (with mem value and GPS stop time)
#define ADC_FILER_CMD 'F' //sets the ADC filter type. 0 is no filter, 1 is default, 50 is 50hz, 60 is 60hz.
#define START_DWNLD_CMD 'D' //download new files (same as DIAS12)
#define HIBERNATE_CMD 'H' //reboot into Hibernate firmware (low power mode)
#define READ_CMD 'R' //send latest file via Ymodem
#define REC_RATE_CMD 'f' //set recording rate (frequency)(samples per second)  'this will be required to be
#define TIMEBASE_CMD 'T' //set the signal time base
#define SET_STREAM_CMD 'B' //set what data is broadcast
#define READ_INFO 'I' //return device type/hw version/fw version
#define READ_GPS 'G' //return last received GPS sentence
#define READ_STATE_CMD 'i' //return the current recorder state (shows info of GPS/SD/recording)
#define READ_STATUS_CMD 's' //return current value of status byte in
#define READ_BAT_CMD 'b' //return the ascii decimal representation of the raw battery level
#define RELAY_CMD 'Y' //set/read front end relay state
#define LINE_CMD 'l' //set/read line number cmd
#define STATION_CMD 'n' //set/read station number cmd
#define AVG_CMD 'A' //read the data averages (as configured by the ' SetStreamCMD'
#define RAW_STREAM_CMD 'r' //raw stream data (as configured by the ' SetStreamCMD'
#define END_REC_CMD 'E' //stop recording currently in progress, do not return to idle state
#define CONV_FACTOR_CMD 'M' //get/set the adc conversion factor units/uV or units/mA
#define INPUT_TYPE_CMD 'U' //get/set the adc input type, voltage/current
#define VP_AVG_CMD 'V' //cmd to send VP averages from the last recording (sent automatically at the end of the recording)
#define RES_CHECK_CMD 'R' //cmd to measure and calculate the ground contact resistance (resistance between the 2 inputs, so requires ref wire grounded at another node)
#define ELEC_LOC_CMD 'L' //electrode location cmd. Set/get electrode location. If unset, then electrode is as same location as node, else this buffer contains coordinate of electrode
#define MSG_CMD 'm' //send error/debug message over Xbee
#define PWR_CMD 'P' //command to set/read power level (used to power down parts of the system. '0' is power off
#define SFT_RESET_CMD 'Z' //command to reboot the node
#define GAIN_CMD 'g' //command to config what PGA setting to use. 0=auto,1..n fixed gain
#define BATTERY_CHEM_CMD 'c' //command to get/set battery pack/chemistry used for battgauge code
#define AUTOCAL_CMD 'a' //command to run auto-calibration code. requires an argument with decimal encoded reference voltage in millivolts (eg a2000 for 2000mV)

/*
 * Relay states
 */
#define RELAY_STATE_TEST 'T'
#define RELAY_STATE_CHARGE 'C'
#define RELAY_STATE_SHORT 'S'
#define RELAY_STATE_ADC 'A'
#define RELAY_STATE_OPEN 'O'
#define RELAY_STATE_INT_SHORT 's' //adc shorted, node input open



#define NODE_CFG_EEPROM_ADDR 0x7FFF //offset in eeprom where config is stored

#define LONG_HOLD 0x02 // button held down long press
#define PRESSED 0x01   // button pressed

/*
 * logger recording states:
 */

#define NOT_READY 0
#define READY   1
#define START_RECORDING 2
#define WAIT_START_RECORDING 3
#define RECORDING 4
#define STOP_RECORDING 5

/*
 * node state flags
 */
#define STATE_DOWNLOADING 0x40  //currently unused
#define STATE_RECORDING 0x10    //node is recording
#define STATE_ACTIVE 0x08       //node is in active state ('ready')
#define STATE_BATT_GOOD 0x04    //battery is not low
#define STATE_SD_READY 0x02     //sd card is accessible
#define STATE_GPS_SYNC 0x20     //gps sync achieved
#define STATE_PPS_SYNC 0x01     //pps sync achieved


volatile uint8_t logging_state = 0;
volatile char node_state = 0;
volatile char relay_state = 0;
volatile char battery_soc = 0;

#define INPUT_TYPE_V  'V'
#define INPUT_TYPE_I  'I'
#define INPUT_TYPE_R  'R'
#define DEF_CONV_FACTOR 860000
#define TYPE_20AH_PANASONIC 0x11
#define TYPE_10AH 0xFF

/*
 * logger saved state info
 */
typedef struct __attribute__((__packed__)){
  //all this data is saved on the eeprom
  char device_id[3];    //node id (3 chars)
  int32_t next_file;   //next file number
  uint32_t src_xbee_addr_l; //low 4 bytes of xbee addr
  uint32_t src_xbee_addr_h; //high 4 bytes of xbee addr
  char dev_input_type;      //device input type (voltage/current etc)
  uint32_t input_factor[4]; //input factors ( 1 for each gain)
  char pos_filler_1; //padding
  char bat_chem;    //battery chemistry flag
  char pos_filler_2; //padding
  uint32_t adc_raw_offset;  //adc offset
  char pos_filler[10]; //padding
  uint32_t node_line;   //node line position on survey
  uint32_t node_station;    //node station position in survey

}volatile  DIAS32_config;


volatile xbee_mailbox_t* xbee_mbx;   //mailbox pointer for messages to/from xbee
volatile xbee_packet tx_packet;      //default tx packet

volatile event_source_t evtPPS; //PPS event source to signal new pps
volatile event_source_t evtSindex; //Second marker event source to signal second transition
//power and user button states
volatile uint8_t pwr_btn_state = 0;
volatile uint8_t rdy_btn_state = 0;
volatile uint16_t glbl_last_s_index = 1;// global last s marker
volatile uint16_t glbl_s_index = 1;   //second marker, increments at the start of each second
volatile bool s_index_flag = false; //marks seconds event
volatile bool auto_gain_flag = true; //default to autogain
volatile uint8_t gain_setting = 0; //gain setting index value

//message buffer for debug/error message sending over Xbee
#define MSG_BUFF_LENGTH 100
volatile char message_buffer[MSG_BUFF_LENGTH];
volatile bool msg_flag = false;
#define EVT_NEW_PPS 0x01
#define EVT_NEW_SINDEX 0x02

//ADC values
volatile uint32_t drdy_gpt_cnt;
volatile uint32_t data_gpt_cnt;
volatile int32_t tmp_data_value;
volatile uint8_t byte_index;
volatile bool data_flag;
volatile int32_t data_value;

//recording data
volatile char mem_value[6];

const int32_t FIR_coeffs[] = {-3453,-2996,9719,21008,2250,-44186,-57914,28988,201864,346004,346004,201864,28988,-57914,-44186,2250,21008,9719,-2996,-3453};
const int32_t FIR_coeffs_sum = 1002568;   //sum is 1 million
const uint16_t FIR_length = 20;//30;     //30 coeff long FIR//low pass filter with free notch at 9.6khz


const uint16_t target_sps = 19200;      //target filtered data output rate

char setRelayState( char new_relay_state );


void write_config_eeprom(void);
void read_config_eeprom(void);
void default_config_eeprom(void);
uint8_t ads1282_init(void);
char init_adc(void);
char *new_filename(void);
void button_hw_init(void);
void read_buttons(void);//THD_FUNCTION(button_thread, arg);
void init_i2c(void);


#endif /* DIAS32_H_ */
