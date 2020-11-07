/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
//#define CORTEX_ENABLE_WFI_IDLE TRUE


#include <string.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "shell.h"


/*
 * for logger states
 */
#include "DIAS32.h"

/*
 * for i2c eeprom memory, using community drivers
 */
I2CConfig eeprom_i2ccfg = {
                                   OPMODE_I2C,
                                   400000,
                                   FAST_DUTY_CYCLE_2
};

/*
 * for ads1282 support
 */
//#include "ads1282/ads1282.h"

/*
 * Include for FatFS support. See ffconf.h for options
 */
#include "ff.h"

/*
 * Working area for driver.
 */
static uint8_t sd_scratchpad[512];

/*
 * SDIO configuration.
 */
static const SDCConfig sdccfg = {
  sd_scratchpad,
  SDC_MODE_4BIT
};


/* config data for mcp. Currently just i2c driver pointer. */
const MCP23017_config mcp_cfg = {&I2CD1};

/*
 * node config struct
 */
static DIAS32_config node_config;

/*
 * i2c eeprom related code
 */
#define EEPROM_SIZE 65536
#define EEPROM_PAGE_SIZE 128
#define EEPROM_WRITE_TIME_MS 5 // byte/page write time
//#define EEPROM_SPID SPID1
//#define EEPROM_SPIDCONFIG spi1cfg

uint8_t eeprom_buffer[EEPROM_PAGE_SIZE+2];
static I2CEepromFileConfig eeCfg = {
  0,
  EEPROM_SIZE,
  EEPROM_SIZE,
  EEPROM_PAGE_SIZE,
  TIME_MS2I( EEPROM_WRITE_TIME_MS ),
  &I2CD1,
  0x50,
  eeprom_buffer
};

static I2CEepromFileStream eeFile;
static EepromFileStream *eeFS;

void write_config_eeprom(void);
void read_config_eeprom(void);
void default_config_eeprom(void);
uint8_t ads1282_init(void);
char init_adc(void);
char *new_filename(void);
void button_hw_init(void);
void read_buttons(void);//THD_FUNCTION(button_thread, arg);
void init_i2c(void);

/*
 * helper function to backup config to eeprom
 */
void write_config_eeprom(void)
{
  //return;
  eeFS = I2CEepromFileOpen(&eeFile, &eeCfg, EepromFindDevice(EEPROM_DEV_24XX));
  fileStreamSetPosition(eeFS, NODE_CFG_EEPROM_ADDR);//start at eeprom config offset
  fileStreamWrite(eeFS, &node_config, sizeof(node_config)); //write to config file
  fileStreamClose(eeFS);
}

/*
 * helper function to load config from eeprom
 */
void read_config_eeprom()
{
  char tmpBuff[100];
  eeFS = I2CEepromFileOpen(&eeFile, &eeCfg, EepromFindDevice(EEPROM_DEV_24XX));
  fileStreamSetPosition(eeFS, NODE_CFG_EEPROM_ADDR);//start at eeprom config offse
  fileStreamRead(eeFS, &node_config, sizeof(node_config)); //read in the config file
  fileStreamSetPosition(eeFS, NODE_CFG_EEPROM_ADDR);//start at eeprom config offse
  fileStreamRead(eeFS, tmpBuff, 100); //read in the config file
  fileStreamClose(eeFS);
}

/*
 * write default config
 */
void default_config_eeprom()
{
  DIAS32_config default_config;
  default_config.device_id[0] = 'A';
  default_config.device_id[1] = 'A';
  default_config.device_id[2] = 'A';
  default_config.next_file = 0;
  default_config.dev_input_type = INPUT_TYPE_V;
  default_config.input_factor[0] = DEF_CONV_FACTOR;
  default_config.input_factor[1] = DEF_CONV_FACTOR;
  default_config.bat_chem = TYPE_20AH_PANASONIC;
  default_config.adc_raw_offset = 0;
  //write modified config to eeprom
  eeFS = I2CEepromFileOpen(&eeFile, &eeCfg, EepromFindDevice(EEPROM_DEV_24XX));
  fileStreamSetPosition(eeFS, NODE_CFG_EEPROM_ADDR);//start at eeprom config offset
  fileStreamWrite(eeFS, &default_config, sizeof(default_config)); //read in the config file
  fileStreamClose(eeFS);

  //read back new saved config
  read_config_eeprom();

}

/*
 * Helper function to set the input relay states
 */

char setRelayState( char new_relay_state )
{
  switch ( new_relay_state  )
  {
    case RELAY_STATE_TEST:
      //relay1 (-)
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY1_PIN1_POS, 0);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY1_PIN2_POS, 1);
      //relay2 (+)
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY2_PIN1_POS, 1);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY2_PIN2_POS, 0);
      //relay3 (-)
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY3_PIN1_POS, 0);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY3_PIN2_POS, 1);
      //relay4 (-)
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY4_PIN1_POS, 0);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY4_PIN2_POS, 1);
      relay_state = new_relay_state;
      break;

    case RELAY_STATE_CHARGE:
      //relay1 (N/A)
      //relay2 (N/A)
      //relay3 (+) connect inputs to charge power converter
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY3_PIN1_POS, 1);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY3_PIN2_POS, 0);
      //relay4 (+) isolate internal circuit from inputs
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY4_PIN1_POS, 1);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY4_PIN2_POS, 0);
      relay_state = new_relay_state;
      break;

    case RELAY_STATE_OPEN:
      //relay1 (N/A)
      //relay2 (N/A)
      //relay3 (-) disconnect input from charge circuit
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY3_PIN1_POS, 0);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY3_PIN2_POS, 1);
      //relay4 (+) isolate circuit from inputs
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY4_PIN1_POS, 1);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY4_PIN2_POS, 0);
      relay_state = new_relay_state;
      break;

    case RELAY_STATE_SHORT:
      //relay1 (+)
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY1_PIN1_POS, 1);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY1_PIN2_POS, 0);
      //relay2 (-)
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY2_PIN1_POS, 0);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY2_PIN2_POS, 1);
      //relay3 (-)
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY3_PIN1_POS, 0);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY3_PIN2_POS, 1);
      //relay4 (-)
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY4_PIN1_POS, 0);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY4_PIN2_POS, 1);
      relay_state = new_relay_state;
      break;

    case RELAY_STATE_ADC:
      //relay1 (-)
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY1_PIN1_POS, 0);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY1_PIN2_POS, 1);
      //relay2 (-)
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY2_PIN1_POS, 0);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY2_PIN2_POS, 1);
      //relay3 (-)
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY3_PIN1_POS, 0);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY3_PIN2_POS, 1);
      //relay4 (-)
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY4_PIN1_POS, 0);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY4_PIN2_POS, 1);
      relay_state = new_relay_state;
      break;

    case RELAY_STATE_INT_SHORT:
      //relay1 (+)
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY1_PIN1_POS, 1);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY1_PIN2_POS, 0);
      //relay2 (-)
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY2_PIN1_POS, 0);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY2_PIN2_POS, 1);
      //relay3 (-)
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY3_PIN1_POS, 0);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY3_PIN2_POS, 1);
      //relay4 (+)
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY4_PIN1_POS, 1);
      mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY4_PIN2_POS, 0);
      relay_state = new_relay_state;
      break;

  }

  chThdSleepMilliseconds(20); //wait 20mS for relays to click over
  //turn off all power to relays to save power. They are latching.
  //relay1
  mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY1_PIN1_POS, 0);
  mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY1_PIN2_POS, 0);
  //relay2
  mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY2_PIN1_POS, 0);
  mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY2_PIN2_POS, 0);
  //relay3
  mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY3_PIN1_POS, 0);
  mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY3_PIN2_POS, 0);
  //relay4
  mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY4_PIN1_POS, 0);
  mcp23017_digitalWrite(&mcp_cfg, 0x07, RELAY4_PIN2_POS, 0);

  return new_relay_state;
}

/*
 * LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(wa_blinker_thread, 512);
static THD_FUNCTION(blinker_thread, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  palSetLineMode(STATUS_LED_LINE,PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);    //set status LED line mode
  palSetLineMode(PWR_CTL_LINE,PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);    //set status LED line mode
  palClearLine(PWR_CTL_LINE);

   /* initialize mcp*/
  //hardware reset the mcp
  palClearLine(MCP_RST_LINE);
  chThdSleepMilliseconds(1);
  palSetLine(MCP_RST_LINE);
  chThdSleepMilliseconds(1);
  mcp23017_init(&mcp_cfg);

//  uint8_t reg_val;
#ifdef debug_mcp
  for(i=0;i<23;i++)
  {
    mcp23017_readRegister(&mcp_cfg, 0x07, i, &reg_val);
    chprintf((BaseSequentialStream*)&SD6, "\r\ninit mcp reg addr 0x%02x value: 0x%02x\n\r",i,(char)reg_val);
  }
  mcp23017_writeRegister(&mcp_cfg, 0x07, 0x0A, 0x80);

  for(i=0;i<23;i++)
  {
    mcp23017_readRegister(&mcp_cfg, 0x07, i, &reg_val);
    chprintf((BaseSequentialStream*)&SD6, "\r\ninit mcp reg addr 0x%02x value: 0x%02x\n\r",i,(char)reg_val);
  }
  mcp23017_writeRegister(&mcp_cfg, 0x07, 0x05, 0x00);
#endif

  /* configure outputs for GPS and Status LEDs */
  /* for now, set all as output */
  mcp23017_pinModeAB(&mcp_cfg, 0x07, MCP23017_OUTPUT);
  //bool last_pps = PAL_LOW;

  //register an event listener on the pps event
  event_listener_t ppsListener;
  chEvtRegister(&evtPPS, &ppsListener, EVENT_MASK(0));
  eventflags_t flags;
  flags = chEvtGetAndClearFlags(&ppsListener); //clear flags

  uint8_t cycle_cnt = 0;
  uint32_t rdy_blink_timebase = 500;
  bool pps_led_state = 0;
  bool rdy_led_state = 0;

  //palSetLine(STATUS_LED_LINE);
  mcp23017_digitalWrite(&mcp_cfg, 0x07, RDY_LED_PIN_POS, rdy_led_state);
  mcp23017_digitalWrite(&mcp_cfg, 0x07, PPS_LED_PIN_POS, pps_led_state);

  while (true) {


    if( chEvtWaitAnyTimeout(ALL_EVENTS, chTimeMS2I( rdy_blink_timebase ) ) )
    {
      flags = chEvtGetAndClearFlags(&ppsListener);  //clear flags
      pps_led_state = !pps_led_state;               //toggle pps led if pps event
    }


    if( node_state&STATE_ACTIVE )
    {
      rdy_blink_timebase = 500;
    }
    if( logging_state == RECORDING )
    {
      rdy_blink_timebase = 250;
    }

    if( !( node_state&STATE_ACTIVE ) )
    {
      rdy_led_state = 0;
    }
    else if( cycle_cnt%2 == 0 )
    {
      rdy_led_state = 1;
    }
    else
    {
      rdy_led_state = 0;
    }

    //palSetLine(STATUS_LED_LINE);
    mcp23017_digitalWrite(&mcp_cfg, 0x07, RDY_LED_PIN_POS, rdy_led_state);
    mcp23017_digitalWrite(&mcp_cfg, 0x07, PPS_LED_PIN_POS, pps_led_state);

    cycle_cnt = ++cycle_cnt%4;
/*
    if( ( palReadPad(GPIOE, 0U) == PAL_HIGH ) && ( last_pps == PAL_LOW ) )
    {
      mcp23017_digitalWrite(&mcp_cfg, 0x07, PWR_LED_PIN_POS, 1);
      last_pps = PAL_HIGH;
    }
    else if( ( palReadPad(GPIOE, 0U) == PAL_LOW ) && ( last_pps == PAL_HIGH ) )
    {
      last_pps = PAL_LOW;
      mcp23017_digitalWrite(&mcp_cfg, 0x07, PWR_LED_PIN_POS, 0);
    }
*/
    /*
    //mcp23017_digitalWrite(&mcp_cfg, 0x07, PPS_LED_PIN_POS, 0);
#ifdef debug_mcp
    for(i=0;i<23;i++)
    {
      mcp23017_readRegister(&mcp_cfg, 0x07, i, &reg_val);
      chprintf((BaseSequentialStream*)&SD6, "\r\nLED1 1, LED2, 0 reg addr 0x%02x value: 0x%02x\n\r",i,(char)reg_val);
    }
#endif
    chThdSleepMilliseconds(500);
    //palClearLine(STATUS_LED_LINE);
    mcp23017_digitalWrite(&mcp_cfg, 0x07, PWR_LED_PIN_POS, 0);
    mcp23017_digitalWrite(&mcp_cfg, 0x07, PPS_LED_PIN_POS, 1);

#ifdef debug_mcp
    for(i=0;i<23;i++)
      {
        mcp23017_readRegister(&mcp_cfg, 0x07, i, &reg_val);
        chprintf((BaseSequentialStream*)&SD6, "\r\nLED1 0, LED2 1, reg addr 0x%02x value: 0x%02x\n\r",i,(char)reg_val);
      }
#endif
    chThdSleepMilliseconds(500);
    */
  }
}

/*
 * Test GPS serial interface thread
 */
static const SerialConfig gpsSerialcfg = {
  9600,
  0,
  USART_CR2_STOP1_BITS,
  0
};

static THD_WORKING_AREA(wa_gps_thread, 512);
static THD_FUNCTION(gps_thread, arg) {
   (void)arg;
   chRegSetThreadName("GPSdata");


   RTCDateTime tmpDateTime;


   // Activates the Serial driver 2, PD8(TX) and PD9(RX) are routed to USART3, 9600 8N1.
   palSetPadMode(GPIOD, 8, PAL_MODE_ALTERNATE(7));
   palSetPadMode(GPIOD, 9, PAL_MODE_ALTERNATE(7));
   sdStart(&SD3, &gpsSerialcfg);
   gps_init();
   event_listener_t elGPSdata;
   eventflags_t flags;
   chEvtRegisterMask((event_source_t *)chnGetEventSource(&SD3), &elGPSdata, EVENT_MASK(1)); //register for SD3 event, as EVENT_MASK(1)

   //parsed data events
   event_listener_t gpsListener;
   chEvtRegister(&evtGPS, &gpsListener, EVENT_MASK(0));


   while (TRUE)
   {
      chEvtWaitOne(EVENT_MASK(1));
      //chSysLock();
      flags = chEvtGetAndClearFlags(&elGPSdata);
      //chSysUnlock();

      if (flags & CHN_INPUT_AVAILABLE)
      {
         msg_t charbuf;
         do
         {
            charbuf = chnGetTimeout(&SD3, TIME_IMMEDIATE);
            if ( charbuf != Q_TIMEOUT )
            {
               //chprintf((BaseSequentialStream*)&SD6, "%c", (char)charbuf);
               //chprintf((BaseSequentialStream*)&SD6, "\r\ncall GPSParseChar(char %c)\n", (char)charbuf);
               GPSParseChar((char)charbuf);
            }
         }
         while (charbuf != Q_TIMEOUT);
      }


        //check for any new gps events
      flags = chEvtGetAndClearFlags(&gpsListener);
      if( flags & GPS_NEW_RMC )
      {

        //update rtc
        if( gps_data.fix_status == 'A')
        {
          uint32_t tmp_hour,tmp_minute,tmp_second;
          tmpDateTime.year = (2000-1980)+( gps_data.utc_date[4]-'0' )*10 + ( gps_data.utc_date[5]-'0' ); //years since 1980
          tmpDateTime.month = ( gps_data.utc_date[2]-'0' )*10 + ( gps_data.utc_date[3]-'0' );
          tmpDateTime.day = ( gps_data.utc_date[0]-'0' )*10 + ( gps_data.utc_date[1]-'0' );

          tmp_second = ( gps_data.utc_time[4]-'0' )*10 + ( gps_data.utc_time[5]-'0' );
          tmp_minute = ( gps_data.utc_time[2]-'0' )*10 + ( gps_data.utc_time[3]-'0' );
          tmp_hour = ( gps_data.utc_time[0]-'0' )*10 + ( gps_data.utc_time[1]-'0' );

          tmpDateTime.millisecond = 1000*((tmp_hour)*3600 + tmp_minute*60 + tmp_second); //millisecond since midnight
          tmpDateTime.dstflag = 0;
          rtcSetTime(&RTCD1, &tmpDateTime);
          struct tm tmp_time;
          rtcConvertDateTimeToStructTm(&tmpDateTime,&tmp_time,NULL);

        }/* not supported for now
        if( gps_data.gps_fix )
        {
          node_state |= STATE_GPS_SYNC;
        }
        else
        {
          node_state &= ~STATE_GPS_SYNC;
        }
        */

      }

   }

   return;
}

/*===========================================================================*/
/* FatFs related.                                                            */
/*===========================================================================*/

/**
 * @brief FS object.
 */
//#todo: check pinout for gen2
static FATFS SDC_FS;

/* FS mounted and ready.*/
//static bool fs_ready = FALSE;

/* Generic large buffer.*/
//static uint8_t fbuff[1024];

/*
 * shell test command for adc logging
 */

void cmd_adc(BaseSequentialStream *chp) {
  //toggle
  if( logging_state == RECORDING )
  {
    logging_state = STOP_RECORDING;
    chprintf(chp, "toggle adc stop recording\r\n");
  }
  else
  {
    logging_state = START_RECORDING;
    chprintf(chp, "toggle adc start recording\r\n");
  }
}


static const ShellCommand commands[] = {

  {"adc", cmd_adc},
  {NULL, NULL}
};

//static const ShellConfig shell_cfg1 = {
//  (BaseSequentialStream *)&SD6,
//  commands
//};




/*
 * Data buffer for ADC data
 */
#define ADC_BUFFER_LENGTH 10000
struct adcBuffer {
  mutex_t  adcMutex;        //mutex for controlling sharing the data buffer
  int32_t bufferLength;    //number of samples in the buffer
  int32_t startPos;        //first data postion in the buffer
  int32_t dataEndPos;      //last filled position in the buffer
  int32_t PPSPos;          //position of the next PPS marker if it is in front of the dataEndPos
  int32_t SIndexPos;       //position of the next Sindex marker if it is in front of the dataEndPos
  int32_t dataBuffer[ ADC_BUFFER_LENGTH ]; //actual data buffer, init to 0
  bool freshData;          //flag to show that data has been written to the buffer
}volatile adcBuffer;

//volatile uint32_t adcBuffer_cnt[ ADC_BUFFER_LENGTH ];
//volatile uint16_t adcBuffer_num[ ADC_BUFFER_LENGTH ];

struct rawADCSample{
  uint32_t gpt_cnt;  //timer count value
  int32_t adc_value; //sample value
  uint32_t s_index;  //sIndex value if updated at this sample
  bool pps;         //pps marker if updated at this sample
};

#define RAW_BUFFER_LENGTH 40
volatile struct rawADCSample raw_sample_buffer[RAW_BUFFER_LENGTH];
volatile uint32_t raw_sample_start = 0;
volatile uint32_t raw_sample_end = 0;




/*
 * Test xbee serial interface thread
 */
static const SerialConfig xbeeSerialcfg = {
  115200,
  0,
  USART_CR2_STOP1_BITS,
  0
};

/*
 * xbee setup thread. sets up serial ports, and calls init function
 */
void xbee_setup(void)
{
  palSetPadMode(GPIOD, 5, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOD, 6, PAL_MODE_ALTERNATE(7));
  sdStart(&SD2, &xbeeSerialcfg);
  chprintf((BaseSequentialStream*)&SD6, "\nxbee interface thread\n");
  xbee_mbx = xbee_init( &SD2,&SD6 );

}

#define TIM5_CLK STM32_SYSCLK/2 //run GPT timer5 at half the full speed
/*
* GPT5 callback.
*/
volatile uint16_t adc_last_s_index = 0;    //adc sindex counter
uint32_t gpt_new_target = TIM5_CLK;
static void gpt5cb(GPTDriver *gptp) {

 (void)gptp;
// uint32_t cnt = GPTD5.tim->CNT;
 //s_index_flag = true;
 glbl_s_index++;
 //glbl_s_index = glbl_last_s_index+1;
}
/*
 * GPT5 configuration for ADC sample timing.
 */
static const GPTConfig gpt5cfg = {
  TIM5_CLK,    /* 84Mhz timer clock.*/
  gpt5cb,   /* Timer callback.*/
  0,
  0
};


/*
 * GPS PPS event callback
 */
systime_t last_pps_time,pps_period;
bool pps_lock = 0;
bool gpt_started = false;
bool gpt_synced = false;
uint32_t gpt_last_target, gpt_cnt;
int32_t gpt_error = 0;
int32_t gpt_last_error = 0;
volatile uint32_t glbl_pps_cnt = 0;



/**
 * @brief   EXTI[0] interrupt handler.
 * fast irq means no OS calls
 * @isr for PPS signal
 */

CH_FAST_IRQ_HANDLER(Vector58) {
  uint32_t pr;

  //clear interrupt
  pr = EXTI->PR;
  pr &= EXTI->IMR & (1U << 0);
  EXTI->PR = pr;
  glbl_pps_cnt++;

  //update timer calibration
  if( gpt_started && ( palReadPad(GPIOE, 0U) == PAL_HIGH ) )//filter out interference spikes
  {
    gpt_cnt = GPTD5.tim->CNT;
    GPTD5.tim->CNT = 0;
    gpt_last_target = GPTD5.tim->ARR;


    if( gpt_cnt < gpt_last_target/10 )
    {
      //timer just happend, target too short
      gpt_error = gpt_cnt;
      if( GPTD5.tim->SR & TIM_SR_UIF )
      {
        //if there is already an update event, then we can safely generate a new event by updating the ARR value
        GPTD5.tim->ARR = (gpt_error/5 + 1) + gpt_last_target; //correct to ideal period
      }
      else
      {
        GPTD5.tim->ARR = (gpt_error/5 + 1) + gpt_last_target; //correct to ideal period
        GPTD5.tim->SR &= ~TIM_SR_UIF; //clear the update interrupt flag we just created by updating the ARR value
      }
    }
    else if( gpt_cnt > 9*gpt_last_target/10 )
    {
      //timer hasn't happened, target too long
      gpt_error = ( gpt_last_target - gpt_cnt );
      if( gpt_error < 0 )
      {
        gpt_error = gpt_error;
      }
      GPTD5.tim->ARR = gpt_last_target - (gpt_error/5 + 1); //correct to ideal period
      //s_index_flag = true;
      if(!( GPTD5.tim->SR & TIM_SR_UIF ) )
      {
        glbl_s_index++; //generate an S-marker if there is no pending timer event
      }
      //glbl_s_index++;// = glbl_last_s_index+1;
      //GPTD5.tim->SR &= ~TIM_SR_UIF; //clear any pending interrupt since we already incremented s_index
    }

  }

}


/*
 * Test PPS timing thread
 */
static THD_WORKING_AREA(wa_pps_thread, 512);
static THD_FUNCTION(pps_thread, arg) {

  (void)arg;
  chRegSetThreadName("pps");

  gpt_last_target = TIM5_CLK;
  gptStart(&GPTD5, &gpt5cfg);   //periodic timer to trigger Sindex events
  gptStartContinuous(&GPTD5, gpt_new_target);
  GPTD5.tim->CR1 &= ~TIM_CR1_ARPE; //clear the auto reload preload enable bit so that updates to ARR are immediate
  //GPTD5.tim->CR1 &= ~TIM_CR1_URS;    //clear the update request source so that UG bit triggers an update event
  gpt_started = true;
  palEnableLineEvent( PPS_LINE, PAL_EVENT_MODE_RISING_EDGE );
  //palSetLineCallback( PPS_LINE, pps_cb, NULL );
  //palEnablePadEvent(GPIOE, 0U, PAL_EVENT_MODE_RISING_EDGE); //Port E, pin 0 is pps input

  //palSetPadCallback(GPIOE, 0U, pps_cb, NULL);
  //register an event listener on the pps event
  event_listener_t ppsListener;
  chEvtRegister(&evtPPS, &ppsListener, EVENT_MASK(0));
  chprintf((BaseSequentialStream*)&SD6, "\npps timing thread\n");
  chprintf((BaseSequentialStream*)&SD6, "\n\r GPT clk is %d\n", TIM5_CLK);


  eventflags_t flags = chEvtGetAndClearFlags(&ppsListener);
//  uint32_t pps_ms_timeout = 1500;
//  uint32_t pps_period_ticks,last_pps_ticks;
//  uint32_t max_gpt_error = 0;

  while( true )
  {
    if( chEvtWaitAnyTimeout(ALL_EVENTS, chTimeMS2I( 1500 ) ) )
    {

      //gptChangeInterval(&GPTD5, gpt_new_target); // the target for a long pulse to re-sync to PPS

      chprintf((BaseSequentialStream*)&SD6, "\n\rGPT cur target: %d\n", gptGetIntervalX(&GPTD5));
      chprintf((BaseSequentialStream*)&SD6, "\n\rGPT last target: %d\n", gpt_last_target);
      chprintf((BaseSequentialStream*)&SD6, "\n\rGPT last cnt: %d\n", gpt_cnt);
      chprintf((BaseSequentialStream*)&SD6, "\n\rGPT last error: %d\n", gpt_error);

      //}
      flags = chEvtGetAndClearFlags(&ppsListener); //clear flags
      /*
      pps_period_ticks = chTimeDiffX(last_pps_ticks, last_pps_time);
      last_pps_ticks = last_pps_time;
      if( 990 <  chTimeI2MS ( pps_period_ticks ) && chTimeI2MS ( pps_period_ticks ) < 1010 )
      {
        pps_period = pps_period_ticks; //if period seems reasonable, then update global period
        pps_lock = 1;
      }
      else
      {
        pps_lock = 0;
      }
      */

    }

    palSetLine(STATUS_LED_LINE);
    //chprintf((BaseSequentialStream*)&SD6, "\n\rPPS: gpt_new_target: %d\n", gpt_new_target);
    //chprintf((BaseSequentialStream*)&SD6, "PPS thread PPS\n\r");
    chThdSleepMilliseconds(100);
    palClearLine(STATUS_LED_LINE);
  }

}



volatile bool global_last_pps = 0;
uint32_t adcLatency = 5; //latency between ADC data output and current time
volatile int32_t adcValue;
volatile bool enable_spi_cb = false; //flag to trigger running spi callback
volatile uint32_t adc_last_pps_cnt = 0;
volatile uint16_t raw_buff_overflow = 0; //raw buffer overflow short counter
volatile bool adc_last_s_index_flag = false;
/*
 * ADC DRDY event callback
 */
CH_FAST_IRQ_HANDLER(VectorE0)
{
  //fast interrupt, don't use os calls
  //trigger PendSV exception
  //clear interrupt flag
  uint32_t pr;
  //save the current timestamp for later

  pr = EXTI->PR;
  pr &= EXTI->IMR & ((1U << 10) | (1U << 11) | (1U << 12) | (1U << 13) |
                     (1U << 14) | (1U << 15));
  EXTI->PR = pr;
  tmp_data_value = 0;
  byte_index = 0;
  SPID1.spi->DR = 0xFF; //write to DR to trigger spi exchange
  raw_sample_end = (raw_sample_end+1)%RAW_BUFFER_LENGTH;   //increment index
  raw_sample_buffer[raw_sample_end].gpt_cnt = GPTD5.tim->CNT; //log the counter value for this sample
  //check for pps
  if( adc_last_pps_cnt != glbl_pps_cnt )
  {
    if( glbl_pps_cnt != adc_last_pps_cnt +1 )
    {
      glbl_pps_cnt = adc_last_pps_cnt +1;
    }
    adc_last_pps_cnt = glbl_pps_cnt;
    raw_sample_buffer[raw_sample_end].pps = true;
  }
  else
  {
    raw_sample_buffer[raw_sample_end].pps = false;
  }
  //check for glbl_s_index marker

  if( glbl_s_index != glbl_last_s_index )
  {
    //debug
    /*
    if( glbl_s_index != ( glbl_last_s_index + 1 ) )
    {
      glbl_s_index = glbl_last_s_index +1; //double s_index increment
    }
*/
    glbl_last_s_index = glbl_s_index;
    raw_sample_buffer[raw_sample_end].s_index = glbl_s_index;
  }

  /*
  if( s_index_flag && !adc_last_s_index_flag )
  {
    raw_sample_buffer[raw_sample_end].s_index = glbl_s_index++;
    adc_last_s_index_flag = true;
  }*/
  else
  {
    raw_sample_buffer[raw_sample_end].s_index = 0;
  }
  //adc_last_s_index_flag = s_index_flag;
  //check for buffer over-run;
  if( raw_sample_start == ( ( raw_sample_end+1 )%RAW_BUFFER_LENGTH ) )
  {
    raw_buff_overflow++;
  }
  //should also check to make sure we haven't run out of buffer

}

/*
* ADC spi data received callback
*/
#define DATA_TYPE_CONV 0x84000000     //marker for gain factor
#define DATA_TYPE_SINDEX 0x83000000   //marker for sindex marker in data
#define DATA_TYPE_PPS 0x82000000      //marker for pps marker in data
#define DATA_GPS_START 0x81000000     //start of a GPS sentence
#define DATA_END  0x80000000          //end of non-binary data
volatile uint32_t missed_sample_cnt,buffer_overflows;
uint16_t coef_index = 0;
uint16_t fir_head_index = 0;
//volatile struct rawADCSample fir_buffer[30]; //fir filter buffer
//using separate buffers for fir buff to help debugging
volatile int32_t fir_adc_buffer[30];
volatile int32_t fir_cnt_buffer[30];
volatile int32_t fir_sindex_buffer[30];
volatile bool fir_pps_buffer[30];

volatile int64_t fir_adcValue;
volatile bool missed_adc_flag = false;
//volatile struct rawADCSample filtered_buffer[30];  //fir output buffer
volatile uint8_t filtered_buffer_end_pos = 0;
volatile uint32_t cur_sample_num = 0;  //sub second sample counter
volatile int64_t last_val = 0;
volatile uint64_t last_cnt = 0;
volatile uint8_t error_flag = 0;
volatile int32_t pendsv_last_s_index = 0;
volatile int32_t pendsv_skipped_s_index = 0;
volatile int32_t raw_max_val = 0;
volatile int32_t raw_min_val = 0;


// low priority handler to use os calls
CH_IRQ_HANDLER(PendSV_Handler){//PendSVVector) {

  CH_IRQ_PROLOGUE();
//  palSetPad(GPIOB, 5U); /* drive SPI_MOSI high for debug */
  uint32_t gpt_interval_cnt = gptGetIntervalX(&GPTD5);
  //uint32_t time_cnt = GPTD5.tim->CNT;

  // loop through half of raw data buffer, apply FIR, subsample to new sample rate and write to output buffer
  uint8_t stop_index =  ( raw_sample_start + RAW_BUFFER_LENGTH/2 )%RAW_BUFFER_LENGTH;
  while( raw_sample_start != stop_index )
  {
    //apply FIR filter to data value
    fir_adc_buffer[ fir_head_index ] = raw_sample_buffer[raw_sample_start].adc_value;
    fir_cnt_buffer[ fir_head_index ] = raw_sample_buffer[raw_sample_start].gpt_cnt;
    fir_sindex_buffer[ fir_head_index ] = raw_sample_buffer[raw_sample_start].s_index;
    fir_pps_buffer[ fir_head_index ] = raw_sample_buffer[raw_sample_start].pps;
    //update the min/max value

    if( raw_sample_buffer[raw_sample_start].adc_value < raw_min_val )
    {
      raw_min_val = raw_sample_buffer[raw_sample_start].adc_value;
    }
    if( raw_sample_buffer[raw_sample_start].adc_value > raw_max_val )
    {
      raw_max_val = raw_sample_buffer[raw_sample_start].adc_value;
    }

    raw_sample_start = ( raw_sample_start+1 )%RAW_BUFFER_LENGTH;

    //calculate filtered value
    fir_adcValue = 0;
 //   uint16_t buff_index;
//    int64_t new_val;

    for( int i = 0; i < FIR_length; i++)
    {
      fir_adcValue += (int64_t)fir_adc_buffer[( fir_head_index + i ) % FIR_length ] * (int64_t)FIR_coeffs[ i ];

    }

    //fir_adcValue = fir_adc_buffer[(fir_head_index + FIR_length/2)%FIR_length];//fir_adcValue/FIR_coeffs_sum;
    fir_adcValue = fir_adcValue/FIR_coeffs_sum;

    //get the cnt value from the mid position of the fir buffer
    uint64_t new_cnt = fir_cnt_buffer[(fir_head_index + FIR_length/2)%FIR_length];

    //check for PPS
    if( fir_pps_buffer[(fir_head_index + FIR_length/2)%FIR_length] )
    {
      //write the PPS marker, make sure not to over-write an s-marker
      if( ( adcBuffer.dataBuffer[( adcBuffer.dataEndPos + adcLatency ) % adcBuffer.bufferLength]&0xFF000000 ) == DATA_TYPE_SINDEX  )
      {
        adcBuffer.dataBuffer[( adcBuffer.dataEndPos + adcLatency + 1 ) % adcBuffer.bufferLength] = DATA_TYPE_PPS;
      }
      else
      {
        adcBuffer.dataBuffer[( adcBuffer.dataEndPos + adcLatency ) % adcBuffer.bufferLength] = DATA_TYPE_PPS;
      }


      chSysLockFromISR();
      chEvtBroadcastFlagsI(&evtPPS, EVT_NEW_PPS);    //fire off an event
      chSysUnlockFromISR();
    }
    //check for SIndex
    if( fir_sindex_buffer[(fir_head_index + FIR_length/2)%FIR_length] != 0)
    {
      //write s marker, but skip any filled buffer (from previous PPS marker)

      if( adcBuffer.dataBuffer[( adcBuffer.dataEndPos + adcLatency ) % adcBuffer.bufferLength] == DATA_TYPE_PPS  )
      {
        if( ( adcBuffer.dataBuffer[( adcBuffer.dataEndPos + adcLatency + 1 ) % adcBuffer.bufferLength]&0xFF000000 ) == DATA_TYPE_SINDEX )
        {
          adcBuffer.dataBuffer[( adcBuffer.dataEndPos + adcLatency + 2 ) % adcBuffer.bufferLength] = DATA_TYPE_SINDEX + fir_sindex_buffer[(fir_head_index + FIR_length/2)%FIR_length];
        }
        else
        {
          adcBuffer.dataBuffer[( adcBuffer.dataEndPos + adcLatency + 1 ) % adcBuffer.bufferLength] = DATA_TYPE_SINDEX + fir_sindex_buffer[(fir_head_index + FIR_length/2)%FIR_length];
        }
      }
      else if( ( adcBuffer.dataBuffer[( adcBuffer.dataEndPos + adcLatency ) % adcBuffer.bufferLength]&0xFF000000 ) == DATA_TYPE_SINDEX  )
      {
        adcBuffer.dataBuffer[( adcBuffer.dataEndPos + adcLatency + 1 ) % adcBuffer.bufferLength] = DATA_TYPE_SINDEX + fir_sindex_buffer[(fir_head_index + FIR_length/2)%FIR_length];
      }
      else
      {
        adcBuffer.dataBuffer[( adcBuffer.dataEndPos + adcLatency ) % adcBuffer.bufferLength] = DATA_TYPE_SINDEX + fir_sindex_buffer[(fir_head_index + FIR_length/2)%FIR_length];
      }
      //sanity check for missed s markers
      if( fir_sindex_buffer[(fir_head_index + FIR_length/2)%FIR_length] != ( pendsv_last_s_index+1 ) )
      {
        pendsv_skipped_s_index++;
      }
      pendsv_last_s_index = fir_sindex_buffer[(fir_head_index + FIR_length/2)%FIR_length];

      glbl_last_s_index = fir_sindex_buffer[(fir_head_index + FIR_length/2)%FIR_length];
      //s_index_flag = false;

 //     adcBuffer_cnt[adcBuffer.SIndexPos ] = new_cnt;

      chSysLockFromISR();
      chEvtBroadcastFlagsI(&evtSindex, EVT_NEW_SINDEX);    //fire off an event
      chSysUnlockFromISR();
    }
    //allow new_cnt to go larger than a period to simplify the math
    if( new_cnt <= gpt_interval_cnt/38400 )
    {
     cur_sample_num = 0; //if this is the very first ADC sample after counter reset, then reset sample num

     new_cnt += gpt_interval_cnt;
    }
    //now calculate sub-sample value, linearly averaging 2 values to get the value for desired gpt_cnt
    uint64_t target_sample_cnt = ((uint64_t)(cur_sample_num)*(uint64_t)gpt_interval_cnt)/target_sps;


    //if we padded new_cnt, then need to pad target_sample_cnt also
    if( new_cnt >= gpt_interval_cnt )
    {
      target_sample_cnt += gpt_interval_cnt;
    }

    //calculate and save new value if valid
    if( new_cnt > target_sample_cnt )
    //sampling at 19200, record all values
    //if(true)
      //record every other sample
    if( ( fir_head_index%2 ) == 0)
    {
      //new value is after target cnt, so linearly average new and last values
      //int64_t new_val = new_cnt;//fir_adcValue;
//      int64_t new_val = fir_adcValue;
      //this could probably done clearer, some type of y=mx+b form
      //int64_t sub_adc_val = (new_val*(target_sample_cnt-last_cnt) + last_val*(new_cnt-target_sample_cnt))/(new_cnt-last_cnt);

      //save the sub-sampled value to the output buffer
      adcBuffer.dataBuffer[ adcBuffer.dataEndPos ] = (int32_t)fir_adcValue;//linear interp seems broken(int32_t)sub_adc_val;

      //increment dataEndPos, but skip any filled data (from PPS or S markers)
      adcBuffer.dataEndPos = (adcBuffer.dataEndPos+1)%adcBuffer.bufferLength;

      while( ( adcBuffer.dataBuffer[ adcBuffer.dataEndPos ] != 0 ) && ( ( adcBuffer.dataEndPos+adcLatency+2 )%adcBuffer.bufferLength != adcBuffer.startPos ) )
      {
        adcBuffer.dataEndPos = (adcBuffer.dataEndPos+1)%adcBuffer.bufferLength;
      }

      //test for buffer nearly full
      if( (adcBuffer.dataEndPos+adcLatency+2)%adcBuffer.bufferLength == adcBuffer.startPos  ) //test for full buffer, allow space for PPS
      {
        //buffer is full
        //chprintf((BaseSequentialStream*)&SD6, "\nadcBuffer full\n\r");
        buffer_overflows++;
        adcBuffer.dataBuffer[adcBuffer.startPos] = 0; //clear out the value for fresh data
        adcBuffer.startPos = (adcBuffer.startPos+1)%adcBuffer.bufferLength; //increment startPos (new data over-rules old data)

      }
      adcBuffer.freshData = TRUE;
      cur_sample_num++;
    }
    last_val = new_cnt; //fir_adcValue;
    last_cnt = new_cnt%gpt_interval_cnt;
    fir_head_index = ( fir_head_index + 1 ) % FIR_length;
    //filtered_buffer_end_pos = ( filtered_buffer_end_pos+1 )%FIR_length;

  }
  //measure how long we took processing the data
  //uint32_t end_time_cnt = GPTD5.tim->CNT;
  //time_cnt = end_time_cnt - time_cnt;
  //time_cnt = GPTD5.tim->CNT - time_cnt;
//  palClearPad(GPIOB, 5U); /* drive SPI_MOSI low */
  CH_IRQ_EPILOGUE();
}


/*
 * SPI configuration for ADS1282 ADC interface
 * flck/128 clock rate
 */
const SPIConfig ads1282ADC_spicfg = {
  false,
  NULL,
  GPIOA,
  4,
  SPI_CR1_BR_2, //fclk/64 about 25Mhz
  0
};

#define STM32_SPI1_HANDLER VectorCC
#define STM32_SPI1_NUMBER 35
/*
 * SPI1 ISR handler
 */
uint32_t tmp_data[4];
CH_FAST_IRQ_HANDLER(STM32_SPI1_HANDLER)
{
  //fast interrupt, don't use os calls
  tmp_data_value = ( tmp_data_value << 16 ) + (uint32_t)SPID1.spi->DR;
  if( byte_index++ >=1 )
  {
    raw_sample_buffer[raw_sample_end].adc_value = tmp_data_value;//raw_sample_buffer[raw_sample_end].gpt_cnt;// tmp_data_value;

    //trigger filter code when buffer is half full and full-full
    if( (raw_sample_end+1)%( RAW_BUFFER_LENGTH/2 ) == 0 )
    {
      SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
      raw_sample_start = ( raw_sample_end + 1 + RAW_BUFFER_LENGTH/2 )%RAW_BUFFER_LENGTH; //hard set start to 0 or half
    }

    //check if drdy ahs already gone low, showing we took too long to read thedata
    if( palReadPad(GPIOE, 15U) == PAL_LOW ) //too slow
    {
      //chprintf((BaseSequentialStream*)&SD6, "\nmissed adc read\n\r");
      missed_adc_flag = true;
      missed_sample_cnt++;
    }
  }
  else
  {
    SPID1.spi->DR = 0xFF; //write to DR to intiate another transfer
  }

}

/*
 * non dma spi config code
 */
void spi1_custom_start(SPIDriver *spip, const SPIConfig *config)
{
  spip->config = config;
  /* enable SPI1 clock */
  rccEnableSPI1(true);
  /* SPI setup and enable. Use same BR and POL settings as HAL config */
  spip->spi->CR1 &= ~( SPI_CR1_SPE ); //clear SPE and BIDIOE
  spip->spi->CR1  = spip->config->cr1 | SPI_CR1_MSTR | SPI_CR1_SSM |
                    SPI_CR1_SSI | SPI_CR1_DFF;
  spip->spi->CR2 &= ~( SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN ); //disable dma bufers
  spip->spi->CR2 |= SPI_CR2_RXNEIE;   //enable rx interrupt
  spip->spi->CR1 |= SPI_CR1_SPE; //clear SPE and BIDIOE

}

/*
 * ADC configuration helper function
 */
#define DR_16KSPS 0x08  //16ksps sample rate from sinc filter
#define DR_32KSPS 0x10  //32ksps sample rate from sinc filter
#define FILT_SINC 0x01  //sinc filter only
#define WREG 0x40       //write register command
#define CONFIG0_ADDR 0x01
#define SDATAC 0x11 //stop continuous conversion output command
#define RDATAC 0x10 //start coninuous conv output
#define CONFIG1_DISABLE_CHOP 0x00 //config1 value for chopper disabled
#define CONFIG1_ENABLE_CHOP 0x80 //config1 value for chopper enabled
#define CHOP_BIT 0x08 //chopper enable bit for PGA
char txbuf;
uint8_t ads1282_init()
{
  /*
   * start ADC spi interface
   */
  palSetPadMode(GPIOB, 5U, PAL_MODE_ALTERNATE(5)| PAL_STM32_OSPEED_HIGHEST); //SPI1_MOSI
  palSetPadMode(GPIOA, 6U, PAL_MODE_ALTERNATE(5)| PAL_STM32_OSPEED_HIGHEST); //SPI1_MISO
  palSetPadMode(GPIOA, 5U, PAL_MODE_ALTERNATE(5)| PAL_STM32_OSPEED_HIGHEST); //SPI1_CLK
  palSetPadMode(GPIOA, 4U, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST); //CS

  spiStart(&SPID1,  &ads1282ADC_spicfg);

  /*
   * write config values to adc registers
   */
  chThdSleepMilliseconds(1000);
  txbuf = 0x00; //wakeup
  spiSend(&SPID1, 1, &txbuf);
  chThdSleepMilliseconds(1);
  txbuf = 0x06; //reset
  spiSend(&SPID1, 1, &txbuf);
  chThdSleepMilliseconds(1);
  txbuf = SDATAC; //stop continous conversion
  spiSend(&SPID1, 1, &txbuf);
  chThdSleepMilliseconds(1);
  txbuf = WREG | CONFIG0_ADDR; //write to CONFIG0 address
  spiSend(&SPID1, 1, &txbuf);
  chThdSleepMilliseconds(1);
  txbuf = 0x01; //write 2 bytes, CONFIG0 and CONFIG1
  spiSend(&SPID1, 1, &txbuf);
  chThdSleepMilliseconds(1);
  //txbuf = FILT_SINC | DR_16KSPS | 0x40; //config value to write
  txbuf = FILT_SINC | DR_32KSPS | 0x40; //config value to write
  spiSend(&SPID1, 1, &txbuf);
  chThdSleepMilliseconds(1);
  txbuf = CONFIG1_DISABLE_CHOP; //config1 MUX 000, CHOP disabled PGA gain = 1
  spiSend(&SPID1, 1, &txbuf);
  chThdSleepMilliseconds(1);
  txbuf = RDATAC; //enable continuous conversion
  spiSend(&SPID1, 1, &txbuf);
  chThdSleepMilliseconds(1);
  txbuf = 0x00; //wakeup
  spiSend(&SPID1, 1, &txbuf);

  //configure MOSI as digital out, hold low to enable ADC
  palSetPadMode(GPIOB, 5U, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST); //SPI1_MOSI held low for now
  palClearPad(GPIOB, 5U); /* drive SPI_MOSI low for ADC */
  enable_spi_cb = true;

  //disable current SPI config to disable DMA etc
  spiStop(&SPID1);
  //re-config SPI
  spi1_custom_start(&SPID1, &ads1282ADC_spicfg);

  return true;

}


/*
 * ADC initialization thread
 */
char init_adc()
{

  //bool last_pps = 0;
  //uint32_t adcLatency = 10; //latency between ADC data output and current time
  //uint32_t adcValue;

  /* init ADC interface */
  chMtxObjectInit( &adcBuffer.adcMutex );
  ads1282_init();

  chprintf((BaseSequentialStream*)&SD6, "\n\rads1282 init\n");
  /* show info on system time */
  chprintf((BaseSequentialStream*)&SD6, "\n\r system resolution is %i ticks/second\n",TIME_MS2I(1000));
  //initialize the shared buffer
  chMtxLock( &adcBuffer.adcMutex );  //lock the mutex
  adcBuffer.bufferLength = ADC_BUFFER_LENGTH; //sizeof( adcBuffer.dataBuffer/sizeof(adcValue));
  adcBuffer.dataEndPos = 1;
  adcBuffer.startPos = 0;
  adcBuffer.PPSPos = -1;
  adcBuffer.SIndexPos = -1;
  adcBuffer.freshData = TRUE;
  chMtxUnlock( &adcBuffer.adcMutex );  //lock the mutex

  missed_sample_cnt = 0;
  buffer_overflows = 0;


  /*
  * Enable hardware interrupt events on falling edge of DRDY of ADC
  */
  // enable PendSVR interrupt

  //enable the vector and set priority
  nvicEnableVector(SPI1_IRQn, 0);
  //nvicSetSystemHandlerPriority(STM32_SPI1_HANDLER, 1); //PendSV_Handler HANDLER_PENDSV

  //enable PendSV for handling SPI data
  nvicEnableVector(PendSV_IRQn, 4);
  nvicSetSystemHandlerPriority(HANDLER_PENDSV, 4); //PendSV_Handler HANDLER_PENDSV


  //already configured as an input in board.h
  palEnablePadEvent(GPIOE, 15U, PAL_EVENT_MODE_FALLING_EDGE);
  /* Assigning a callback to ADC DRDY falling  edge event, passing no arguments.*/
  //palSetPadCallback(GPIOE, 15U, drdy_cb, NULL);

  return 1;

}


/*
 * helper function to make the data file filename strin
 */
char data_filename[] = "00000000000"; //AAA00000.BIN";
char *new_filename()
{
//  data_filename = "0000000000";
  char file_number_string[] = "000000";
  //char node_id[] = "   ";
//  uint8_t file_num_length;

  memcpy( data_filename, node_config.device_id , 3); //copy device id to 1st 3 characters
  data_filename[3] = 0; //null terminate to make a string
  chsnprintf( file_number_string, 6, "%d",node_config.next_file++);
  strcat(data_filename,file_number_string); //keep filename length constant. Pad with 0's
  strcat(data_filename,".BIN");

  chprintf((BaseSequentialStream*)&SD6, "\r\nnew data filename: %s \r\n", data_filename);

  return data_filename;
}


/*
 * helper function to open a new data file
 */

FRESULT new_logfile( BaseSequentialStream *chp, FIL *fp)
{
  FRESULT err;
//  uint32_t bytes_written;
//  uint8_t tries = 0;
//  FIL testFP;
  char *path;

  path = new_filename();
  err = f_open (fp, path, (FA_CREATE_NEW | FA_WRITE) ); //open a new data file
  //test to see if the filename already exists
  while( err == FR_EXIST )
  {
    //try again. Note new_filename() auto-increments file number.
    path = new_filename();
    err = f_open (fp, path, (FA_CREATE_NEW | FA_WRITE) ); //open a new data file

  }

  if( err != FR_OK )
  {
    chprintf(chp, "\r\nFile open error: %d\r\n", err);
    chsnprintf( message_buffer, MSG_BUFF_LENGTH, "File open error: %d", err ); //message to buffer
    msg_flag = true; //set message flag to trigger sending
  }
  else
  {
    chprintf(chp, "\r\nFile %s created\r\n", path );
    chsnprintf( message_buffer, MSG_BUFF_LENGTH, "File %s created", path ); //message to buffer
    msg_flag = true; //set message flag to trigger sending
    //save the updated file number to eeprom
    write_config_eeprom();

  }
  return err;
}

/*
 * helper function to format data to add to data buffer
 */
uint32_t format_data( char *tmp_buff, char *tmp_data, uint32_t num_bytes, uint32_t data_type )
{
  uint32_t tmp_buff_end = 0;
  uint32_t data_end_marker = DATA_END;
  memcpy( tmp_buff, &data_type,4);
  tmp_buff_end = 4;
  memcpy( tmp_buff + tmp_buff_end, tmp_data, num_bytes );
  tmp_buff_end += num_bytes;
  while( tmp_buff_end%4 != 0 )
  {
    tmp_buff[tmp_buff_end++] = ' ';//pad with spaces to keep data alignment
  }
  memcpy( tmp_buff + tmp_buff_end, &data_end_marker, 4 ); //append data end marker
  tmp_buff_end += 4;
  return tmp_buff_end;
}
/*
 * function to set pga, returns the new pga value
 */
uint8_t set_pga(uint8_t new_pga_index)
{


  //high gain
  if( new_pga_index == 0 )
  {
    palClearLine(PGA_SIG_LINE); //drive low for high gain
    gain_setting = new_pga_index;
  }
  //low gain
  else
  {
    palSetLine(PGA_SIG_LINE); //drive high for low gain
    gain_setting = new_pga_index;
  }

  return gain_setting;

}


/*
 * data logger thread
 */
//debug vars
int32_t buff_free_size = 0;
int32_t data_end_pos = 0;
uint8_t logger_pos = 0;
uint32_t max_latency = 0;


#define ADC_VREF_MV 5000
#define STRING_BUFFER_LENGTH 128
#define FILE_BUFFER_LENGTH 512+STRING_BUFFER_LENGTH
#define HIGH_GAIN 0
#define LOW_GAIN 1
#define ADC_MAX_VAL 200000000 //2^30 //2^31/2 is max +/- range of 31 bit ADC
#define ADC_EXCESSIVE_LIMIT 1000000000 // - 2^27//approx 90% signal range

static THD_WORKING_AREA(wa_logger_thread, 2048);
static THD_FUNCTION(logger_thread, arg) {

  (void)arg;
  chRegSetThreadName("logger");

  //depending on logger state, receive data from adc, filter it, process it and record it.
  //for now, if the logger is switched to recording, then open a file and start recording
  int32_t tmpADCSample;
//  struct adcSample *tmpSample;
//  bool last_pps = 0;
  uint8_t curBuffer = 1;
  uint32_t bufferEndPos = 0;
  char *curFile_buffer;
  uint32_t sum_missed_sample_cnt = 0;
  uint32_t sum_buffer_overflows = 0;
  uint32_t sum_raw_buff_overflows = 0;
  systime_t write_time_mS = 0;
  uint32_t sync_timer = 0;
  int32_t last_s_index = 0;

  static char file_buffer1[ FILE_BUFFER_LENGTH ];
  static char file_buffer2[ FILE_BUFFER_LENGTH ];
  static char string_buffer[ STRING_BUFFER_LENGTH ]; //tempory string buffer for using sprintf
  uint32_t bytes_written;
  uint32_t sps_count = 0;
  //uint32_t glbl_s_index = 0;
//  char *tmp_buff;

  palSetLineMode(PGA_SIG_LINE,PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
  set_pga(HIGH_GAIN);
  //palSetLine(PGA_SIG_LINE); //drive low for high gain

  FIL fp;
  //initialize SD card
  FRESULT err = sdcConnect(&SDCD1);

//  int32_t adc_bin_to_mV = node_config.input_factor[1]/2;//430;//(uint32_t)(2^32)/(ADC_VREF_MV*1000);
  event_listener_t gpsListener;
  chEvtRegister(&evtGPS, &gpsListener, EVENT_MASK(0));

  if( err != FR_OK )
  {
    chprintf((BaseSequentialStream*)&SD6, "\r\nSDCD1 sdcConnect error: %x", err);
    chsnprintf( message_buffer, MSG_BUFF_LENGTH, "SDCD1 sdcConnect error: %x", err); //write error message to buffer
    msg_flag = true; //set message flag to trigger sending
  }
  else
  {
    chprintf((BaseSequentialStream*)&SD6, "\r\nSDCD1 sdc connected\r\n");

  }
  err = f_mount(&SDC_FS, "/", 1);
  if( err != FR_OK )
  {
    chprintf((BaseSequentialStream*)&SD6, "\r\nSD f_mount error: %x\r\n", err);
    chsnprintf( message_buffer, MSG_BUFF_LENGTH, "SD f_mount error: %x", err); //write error message to buffer
    msg_flag = true; //set message flag to trigger sending
  }
  else
  {

    chprintf((BaseSequentialStream*)&SD6, "\r\nSDCD1 mounted\r\n");
  }

  while (true)
  {
    logger_pos = 0;
    if( missed_adc_flag )
    {
      chprintf((BaseSequentialStream*)&SD6, "\nmissed adc reading\n\r");
      chprintf((BaseSequentialStream*)&SD6, "Missed samples %d buffer overflows: %d\n\r",missed_sample_cnt,buffer_overflows);
      missed_adc_flag = false;
    }
    if( raw_buff_overflow )
    {
      chprintf((BaseSequentialStream*)&SD6, "\nraw buffer overflowed %d times\n\r",raw_buff_overflow);
      sum_raw_buff_overflows+=raw_buff_overflow;
      raw_buff_overflow = 0;
    }
    // check if we are above 20% signal (need to leave headroom for rare high amplitude events
    // only react if node is active, otherwise the signals aren't valid anyways
    if( ( ( abs(raw_min_val) > (ADC_EXCESSIVE_LIMIT/5) ) || (raw_max_val > (ADC_EXCESSIVE_LIMIT/5)) ) && ( logging_state >= READY ))
    {
      //switch to low gain if in first 30s of file recording
      uint8_t tmp_gain_setting = gain_setting;

      if( auto_gain_flag && ( gain_setting != LOW_GAIN ))
      {
        if( ( logging_state == WAIT_START_RECORDING ) || ( logging_state == RECORDING ) )
        {
          if( glbl_s_index < 30 )
          {
            set_pga(LOW_GAIN);
            //add new gain setting to data file
            uint32_t tmp_val = DATA_TYPE_CONV;
            memcpy(curFile_buffer+bufferEndPos,&tmp_val,4);  //conversion value flag
            bufferEndPos += 4;
            tmp_val = node_config.input_factor[1]/2;
            memcpy(curFile_buffer+bufferEndPos,&tmp_val,4);  //copy conversion factor
            bufferEndPos += 4;
            tmp_val = DATA_END;
            memcpy(curFile_buffer+bufferEndPos,&tmp_val,4);  //data end flag
            bufferEndPos += 4;
          }
        }
        else
        {
          //if not recording, then can change gain without doing anything else
          set_pga(LOW_GAIN);
        }
      }
      //if gain wasn't updated, then check for excessive singal and send message if near limit
      if( gain_setting == tmp_gain_setting )
      {
        //send excessive signal message if near limit
        if( ( abs(raw_min_val) > (ADC_EXCESSIVE_LIMIT) ) || (raw_max_val > (ADC_EXCESSIVE_LIMIT) ) )
        {
          chsnprintf( message_buffer, MSG_BUFF_LENGTH, "Excessive Input Voltage!!"); //write error message to buffer
          msg_flag = true; //set message flag to trigger sending
        }
      }
      //send message if updated
      else
      {
        chsnprintf( message_buffer, MSG_BUFF_LENGTH, "Changed PGA To Low Gain"); //write error message to buffer
        msg_flag = true; //set message flag to trigger sending
        chThdSleepMicroseconds(200); //wait 200uS for the signal to settle after PGA change
      }
      raw_max_val = 0;
      raw_min_val = 0;
    }

    if( logging_state == START_RECORDING ) //need to wait until the start of a new period
    {
      //create new data file, and populate header
      FRESULT err = new_logfile((BaseSequentialStream*)&SD6, &fp);
      glbl_s_index = 1;
      glbl_last_s_index = 1;
      sps_count = 0;
      sum_missed_sample_cnt = 0;
      sum_buffer_overflows = 0;
      raw_min_val = 0;
      raw_max_val = 0;
      if( err == FR_OK )//file created successfully
      {
        //logging_state = RECORDING;
        curFile_buffer = file_buffer1; //init the buffer pointer
        bufferEndPos = 0;
        curBuffer = 1;
        uint32_t tmp_val = node_config.input_factor[gain_setting]/2;
        memcpy(curFile_buffer+bufferEndPos,&tmp_val,4);  //copy conversion factor
        bufferEndPos += 4;

        //add new gain setting to data file
        tmp_val = DATA_TYPE_CONV;
        memcpy(curFile_buffer+bufferEndPos,&tmp_val,4);  //conversion value flag
        bufferEndPos += 4;
        tmp_val = node_config.input_factor[gain_setting]/2;
        memcpy(curFile_buffer+bufferEndPos,&tmp_val,4);  //copy conversion factor
        bufferEndPos += 4;
        tmp_val = DATA_END;
        memcpy(curFile_buffer+bufferEndPos,&tmp_val,4);  //data end flag
        bufferEndPos += 4;

        adcBuffer.startPos = adcBuffer.dataEndPos; //reset buffer
        uint32_t i;
        for( i=0;i< adcBuffer.bufferLength;i++)
        {
          adcBuffer.dataBuffer[i] = 0;
        }
        adcBuffer.freshData = false;
        logging_state = WAIT_START_RECORDING; //waiting for start of new second
      }
      else
      {
        logging_state = READY; //couldn't open the file, so can't record.
        chprintf((BaseSequentialStream*)&SD6, "\nfile open failed:  %d\n\r", err);
        chsnprintf( message_buffer, MSG_BUFF_LENGTH, "file open failed:  %d", err); //write error message to buffer
        msg_flag = true; //set message flag to trigger sending
      }
    }
    else if( logging_state == STOP_RECORDING )
    {
      chprintf((BaseSequentialStream*)&SD6, "\nClose the File\n\r");
      //if there is data in the buffer, write it before closing
      if( bufferEndPos > 0 )
      {
        err = f_write(&fp, curFile_buffer, bufferEndPos, &bytes_written);
      }
      f_close(&fp); //close datafile
      logging_state = READY;
    }
    // process fresh data, otherwise sleep
    while( adcBuffer.freshData == TRUE )
    {
      // handle all data in the buffer
      tmpADCSample = adcBuffer.dataBuffer[ adcBuffer.startPos ];
      //clear the buffer value

      adcBuffer.dataBuffer[ adcBuffer.startPos ] = 0;

      buff_free_size = adcBuffer.startPos-adcBuffer.dataEndPos;
      if( buff_free_size <= 0 )
      {
        buff_free_size+=adcBuffer.bufferLength;
      }
      data_end_pos = adcBuffer.dataEndPos;

//      chMtxLock(&adcBuffer.adcMutex);  //lock the mutex
      adcBuffer.startPos = ( adcBuffer.startPos + 1 ) % adcBuffer.bufferLength;
      if( adcBuffer.startPos == adcBuffer.dataEndPos )
      {
        adcBuffer.freshData = FALSE;
        if( logging_state == RECORDING )
        {

          logger_pos = 1;
          write_time_mS = chVTGetSystemTime();
          if( sync_timer > 2 )//sync every 2s, but only if caught up processing the data buffer
            err = f_sync (&fp);   //flush changes every second

          write_time_mS = chTimeDiffX(write_time_mS, chVTGetSystemTime());
          sync_timer = 0;
          if( TIME_I2MS(write_time_mS) > 10 ) //print out if takes more than 10mS to write
            chprintf((BaseSequentialStream*)&SD6, "\n\r f_sync took %d mS\n\r",TIME_I2MS(write_time_mS));
          if( TIME_I2MS(write_time_mS) > max_latency )
            max_latency = TIME_I2MS(write_time_mS);

          if( buffer_overflows )
          {
            chprintf((BaseSequentialStream*)&SD6, "Buffer Overflow: %d\n\r",buffer_overflows);
            sum_buffer_overflows += buffer_overflows;
            buffer_overflows = 0;
          }
          buff_free_size = adcBuffer.startPos-adcBuffer.dataEndPos;
          if( buff_free_size <= 0 )
          {
            buff_free_size+=adcBuffer.bufferLength;
          }
          data_end_pos = adcBuffer.dataEndPos;

          logger_pos = 2;

        }
      }
 //     chMtxUnlock(&adcBuffer.adcMutex);  //unlock the mutex

      //glbl_s_index record
      if( ( tmpADCSample & 0xFF000000 ) == DATA_TYPE_SINDEX )
      {
        /*
        if( adcBuffer.SIndexPos != -1 )
        {

          chprintf((BaseSequentialStream*)&SD6, "S Marker Bug, duplicate sample %d\n\r",glbl_s_index);
        }
        */
        if( logging_state == WAIT_START_RECORDING )
        {
          logging_state = RECORDING;
        }
        if( logging_state == RECORDING )
        {
          memcpy(curFile_buffer+bufferEndPos,&tmpADCSample,4);  //copy Sindex marker to buffer
          bufferEndPos += 4;
          sync_timer++;
        }
        //debug check for missed s_index
        if( ( tmpADCSample & 0xFFFFFF ) != last_s_index+1)
          chprintf((BaseSequentialStream*)&SD6, "Missed S index %d, %d\n\r", ( tmpADCSample & 0xFFFFFF ),last_s_index);
        last_s_index = ( tmpADCSample & 0xFFFFFF );


      }

      // pps record
      else if( tmpADCSample == DATA_TYPE_PPS )
      {
        /*
        if( adcBuffer.PPSPos != -1 )
        {

          chprintf((BaseSequentialStream*)&SD6, "PPS Bug, duplicate sample %d\n\r",glbl_s_index);
        }
        */
        if( logging_state == RECORDING )
        {
          //sprintf( ( string_buffer ),"PPS SPS = %d\n",sps_count);                   //write formatted string to buffer
          //chsnprintf(string_buffer, STRING_BUFFER_LENGTH, "PPS SPS = %d\n",sps_count);
          memcpy(curFile_buffer+bufferEndPos,&tmpADCSample,4);  //copy PPS marker to buffer
          bufferEndPos += 4;
          //err = f_write(&fp, &tmpADCSample, 4, &bytes_written);

          //err = f_write(&fp, string_buffer, strlen(string_buffer), &bytes_written);
          //err = f_printf(&fp, "PPS SPS = %d\r\n",sps_count);
        }
//#ifdef DEBUG_ADC
        chprintf((BaseSequentialStream*)&SD6, "PPS %d samples for glbl_s_index %d\n\r",sps_count,glbl_s_index);
        chprintf((BaseSequentialStream*)&SD6, "Max file latency: %dmS\n\r",max_latency);
        chprintf((BaseSequentialStream*)&SD6, "Missed samples %d, raw overflows: %d, overflows: %d\n\r",sum_missed_sample_cnt,sum_raw_buff_overflows,sum_buffer_overflows);
//#endif

        sum_missed_sample_cnt += missed_sample_cnt;
        missed_sample_cnt = 0;
        sps_count = 0;
        //glbl_s_index++;

      }
      // adc record
      else
      {
        //tmpADCSample = tmpADCSample<<1; //convert back to 32 bit value
        //tmpADCSample = tmpADCSample/adc_bin_to_uV; //convert to uV
        sps_count++;

        if( logging_state == RECORDING )
        {
          //sprintf(string_buffer,"%d\n",tmpADCSample);
          //tmp_buff = ch_ltoa(string_buffer,tmpADCSample,10);
          //strncpy( tmp_buff,"\n",2 );
          //strncat(tmp_buff,"\n",1);
//          chsnprintf(string_buffer, 15, "%d\n", tmpADCSample);

          //memcpy(curFile_buffer+bufferEndPos,&buff_free_size,4);  //write raw data
          memcpy(curFile_buffer+bufferEndPos,&tmpADCSample,4);  //write raw data
          bufferEndPos += 4;
          //err = f_write(&fp, &tmpADCSample, 4, &bytes_written);
          //err = f_write(&fp, &tmpADCSample, sizeof(tmpADCSample), &bytes_written);
          //err = FR_OK;
        }

      }


      logger_pos = 4;
      //check for any new gps events
      eventflags_t flags = chEvtGetAndClearFlags(&gpsListener);
      if( flags )
      {
        if( flags & GPS_NEW_RMC )
        {
          chprintf((BaseSequentialStream*)&SD6, "\n\r %s\n\r",global_GPRMC);
          if( logging_state == RECORDING )
          {
            uint32_t str_buf_len = format_data( string_buffer, global_GPRMC, strlen( global_GPRMC ), DATA_GPS_START );
            memcpy( curFile_buffer + bufferEndPos, string_buffer, str_buf_len );  //copy gps string to data buffer
            bufferEndPos += str_buf_len;
          }
        }
        if( flags & GPS_NEW_GGA )
        {
          chprintf((BaseSequentialStream*)&SD6, "\n\r %s\n\r",global_GPGGA);
          if( logging_state == RECORDING )
          {
            uint32_t str_buf_len = format_data( string_buffer, global_GPGGA, strlen( global_GPGGA ), DATA_GPS_START );
            memcpy( curFile_buffer + bufferEndPos, string_buffer, str_buf_len );  //copy gps string to data buffer
            bufferEndPos += str_buf_len;
          }
        }
      }

      if( logging_state == RECORDING )
      {
        if( bufferEndPos >= ( FILE_BUFFER_LENGTH - STRING_BUFFER_LENGTH ))
        {
          write_time_mS = chVTGetSystemTime();

          logger_pos = 5;
          err = f_write(&fp, curFile_buffer, bufferEndPos, &bytes_written);
          write_time_mS = chTimeDiffX(write_time_mS, chVTGetSystemTime());
          if( TIME_I2MS(write_time_mS) > 10 ) //print out if takes more than 10mS to write
          {
            chprintf((BaseSequentialStream*)&SD6, "\n\r f_write took %d mS\n\r",TIME_I2MS(write_time_mS));
            if( TIME_I2MS(write_time_mS) > max_latency )
              max_latency = TIME_I2MS(write_time_mS);
          }
          if( buffer_overflows )
          {
            chprintf((BaseSequentialStream*)&SD6, "Buffer Overflow: %d\n\r",buffer_overflows);
            sum_buffer_overflows += buffer_overflows;
            buffer_overflows = 0;
          }

          logger_pos = 6;
          bufferEndPos = 0;
          if( curFile_buffer == file_buffer1 )
            curFile_buffer = file_buffer2;
          else
            curFile_buffer = file_buffer1;

          buff_free_size = adcBuffer.startPos-adcBuffer.dataEndPos;
          if( buff_free_size <= 0 )
          {
            buff_free_size+=adcBuffer.bufferLength;
          }
          data_end_pos = adcBuffer.dataEndPos;
        }

      }
      if( err != FR_OK )
      {
        chprintf((BaseSequentialStream*)&SD6, "\nfile recording failed:  %d\n\r", err);
        chsnprintf( message_buffer, MSG_BUFF_LENGTH, "\nfile recording failed:  %d\n\r", err); //write error message to buffer
        msg_flag = true; //set message flag to trigger sending
        logging_state = READY; //show we are no longer recording. Can't close file because there was an error with the card
      }

    }
    //sleep to allow some time for fresh data
    logger_pos = 7;
    chThdSleepMicroseconds(200);
    logger_pos = 8;
    //chThdSleepMilliseconds(1);

  }
}
/* helper function to verify/update node state
 *
 */
char check_node_state(char new_state)
{
  char new_node_state = node_state;
  // currently the only thing that can be set is read/active
  if( new_state&STATE_ACTIVE )
  {
    new_node_state |= STATE_ACTIVE;
  }
  else
  {
    new_node_state &= ~STATE_ACTIVE;
  }
  return new_node_state;

}

/*
 * helper function to verify the new relay state is valid (eg don't allow non ready nodes to set to analog)
 */
bool check_valid_relay(char new_relay_state )
{
  //not valid to set analog relay if node isn't active
  if( new_relay_state == RELAY_STATE_ADC && !( node_state&STATE_ACTIVE ) )
  {
    return 0;
  }
  return 1;
}
/*
 * run_cmd processes node commands. It takes as argument a pointer to a command packet and at transmit mailbox
 * it should work for Wifi or xbee interfaces as long as their packets and mailboxes are compatible (not currently implemented)
 * run_cmd can also be used to send info initiated internally (eg relay change or state change messages)
 */
#define SEND_REPLY 0
#define NO_REPLY -1
msg_t run_cmd(volatile xbee_packet *cmd_packet, xbee_mailbox_t *mbx )
{
  xbee_packet *resp_packet;
  msg_t msg = chMBFetchTimeout(&mbx->tx_free_packets, (msg_t *)&resp_packet,TIME_IMMEDIATE);
  if( msg != MSG_OK)
  {
    chprintf((BaseSequentialStream*)&SD6, "\n\rxbee response mailbox failure\n\r");
    return msg;
  }

  memcpy(resp_packet,cmd_packet,sizeof(xbee_packet)); //copy the address etc.
  resp_packet->data_length = 0; //reset data buffer

  switch ( cmd_packet->data[0] )
  {
    // RX packet type. Assume it is a command.
    case READ_STATUS_CMD:   //node status cmd
      //if argument provided, then update status. New state will auto-send on change
      if( cmd_packet->data_length > 1 )
      {
        node_state = check_node_state( strtoul(&cmd_packet->data[1], NULL, 16) );
      }
      //if no arg provided, just send current status
      else
      {
        resp_packet->data[1] = 0;//null terminate to make like a string
        chsnprintf( &resp_packet->data[strlen(resp_packet->data)], 6, "%02X",node_state);
        resp_packet->data_length = strlen(resp_packet->data);
        (void)chMBPostTimeout(&mbx->tx_filled_packets, (msg_t)resp_packet,TIME_IMMEDIATE); //post the response
        return SEND_REPLY; //flag that the packet needs to be sent
      }
      break;

    case READ_STATE_CMD:    //node state cmd (combined info)
      resp_packet->data[1] = ';';
      resp_packet->data[2] = READ_INFO;
      resp_packet->data[3] = node_config.device_id[0];  //node id
      resp_packet->data[4] = node_config.device_id[1];
      resp_packet->data[5] = 0;//node_config.device_id[2];
//      resp_packet->data[6] = 0;
      chsnprintf( &resp_packet->data[strlen(resp_packet->data)], 100, "%s%s",HW_TYPE,HW_VERSION); //HW data
      chsnprintf( &resp_packet->data[strlen(resp_packet->data)], 100, "%s%s",FW_TYPE,FW_VERSION); //FW data
      chsnprintf( &resp_packet->data[strlen(resp_packet->data)], 100, ";%c%c",INPUT_TYPE_CMD,node_config.dev_input_type); //input type
      chsnprintf( &resp_packet->data[strlen(resp_packet->data)], 100, ";%c%d",REC_RATE_CMD,target_sps); //recording rate
      chsnprintf( &resp_packet->data[strlen(resp_packet->data)], 100, ";%c%i",TIMEBASE_CMD,2000); //timebase
      chsnprintf( &resp_packet->data[strlen(resp_packet->data)], 100, ";%c%i",READ_BAT_CMD,battery_soc); //battery level
      chsnprintf( &resp_packet->data[strlen(resp_packet->data)], 100, ";%c%s,%c,",READ_GPS,gps_data.lat,gps_data.lat_NS); //gps lat
      chsnprintf( &resp_packet->data[strlen(resp_packet->data)], 100, "%s,%c,%s",gps_data.lon,gps_data.lon_EW,gps_data.elev); //gps lon and elev
      resp_packet->data_length = strlen(resp_packet->data);
      (void)chMBPostTimeout(&mbx->tx_filled_packets, (msg_t)resp_packet,TIME_IMMEDIATE); //post the response  msg_t msg = chMBFetchTimeout(&mbx->tx_free_packets, (msg_t *)&resp_packet,TIME_IMMEDIATE);

      msg_t msg = chMBFetchTimeout(&mbx->tx_free_packets, (msg_t *)&resp_packet,TIME_IMMEDIATE); //grab a new packet
      if( msg != MSG_OK)
      {
        chprintf((BaseSequentialStream*)&SD6, "\n\rxbee response mailbox failure\n\r");
        return SEND_REPLY; //still need to send the first packet
      }
      memcpy(resp_packet,cmd_packet,sizeof(xbee_packet)); //copy the address etc.
      resp_packet->data_length = 0; //reset data buffer
      resp_packet->data[1] = 0; //null terminate to make it a string
      chsnprintf( &resp_packet->data[strlen(resp_packet->data)], 100, ";%c%i",LINE_CMD,node_config.node_line); //node line
      chsnprintf( &resp_packet->data[strlen(resp_packet->data)], 100, ";%c%i",STATION_CMD,node_config.node_station); //node station
      chsnprintf( &resp_packet->data[strlen(resp_packet->data)], 100, ";%c%02X",READ_STATUS_CMD,node_state); //status
      chsnprintf( &resp_packet->data[strlen(resp_packet->data)], 100, ";%c%c",RELAY_CMD,relay_state); //recording rate
      resp_packet->data_length = strlen(resp_packet->data);
      (void)chMBPostTimeout(&mbx->tx_filled_packets, (msg_t)resp_packet,TIME_IMMEDIATE); //post the response
      return SEND_REPLY; //flag that the packet needs to be sent
      break;

    case READ_INFO:    //node info cmd
      resp_packet->data[1] = ';';
      resp_packet->data[2] = READ_INFO;
      resp_packet->data[3] = node_config.device_id[0];  //node id
      resp_packet->data[4] = node_config.device_id[1];
      resp_packet->data[5] = node_config.device_id[2];
      resp_packet->data[6] = 0;
      chsnprintf( resp_packet->data[strlen(resp_packet)], 100, "%s%s",HW_TYPE,HW_VERSION); //HW data
      chsnprintf( resp_packet->data[strlen(resp_packet)], 100, "%s%s",FW_TYPE,FW_VERSION); //FW data
      resp_packet->data_length = strlen(resp_packet->data);
      (void)chMBPostTimeout(&mbx->tx_filled_packets, (msg_t)resp_packet,TIME_IMMEDIATE); //post the response
      return SEND_REPLY; //flag that the packet needs to be sent
      break;

    case START_REC_CMD:    //start recording command
       if( cmd_packet->data_length >= 12 ) //6char for endTIme, 6char for memVal
       {
         //flag logger to start recording if the node is ready
         if( ( node_state & STATE_ACTIVE ) && ( logging_state == READY ) )
         {
           memcpy(mem_value,6+cmd_packet->data,6); //copy the address etc.
           logging_state = START_RECORDING;
         }
       }
       //don't send a response, a state change will be sent once recording starts

       break;

     case END_REC_CMD:    //end recording command

       //flag logger to stop recording if recording
       if( logging_state >= START_RECORDING )
       {
         logging_state = STOP_RECORDING;
       }
       //don't send a response, a state change will be sent once recording stops
       break;

    case RELAY_CMD:    //relay command
      //if argument, set relay. relay change will trigger sending new state
      if( cmd_packet->data_length > 1 )//check if cmd has argument
      {
        if( check_valid_relay(cmd_packet->data[1]) )
        {
          setRelayState( cmd_packet->data[1] );
        }
      }
      //if no argument, then send current state
      else
      {
        resp_packet->data[1] = relay_state;
        resp_packet->data[2] = 0;
        resp_packet->data_length = strlen(resp_packet->data);
        (void)chMBPostTimeout(&mbx->tx_filled_packets, (msg_t)resp_packet,TIME_IMMEDIATE); //post the response
        return SEND_REPLY; //flag that the packet needs to be sent
      }
      /*
    case READ_GPS:    //gps command
      //send GPS sentence(s)
      resp_packet->data[1] = 0; //null terminate to make a string
      chsnprintf( &resp_packet->data[strlen(resp_packet->data)], 100, "%s",global_GPRMC); //append GPS sentence
      resp_packet->data_length = strlen(resp_packet->data);
      (void)chMBPostTimeout(&mbx->tx_filled_packets, (msg_t)resp_packet,TIME_IMMEDIATE); //post the response

      msg_t msg = chMBFetchTimeout(&mbx->tx_free_packets, (msg_t *)&resp_packet,TIME_IMMEDIATE); //grab a new packet
      if( msg != MSG_OK)
      {
        chprintf((BaseSequentialStream*)&SD6, "\n\rxbee response mailbox failure\n\r");
        return SEND_REPLY; //still need to send the first packet
      }
      memcpy(resp_packet,cmd_packet,sizeof(xbee_packet)); //copy the address etc.
      resp_packet->data_length = 0; //reset data buffer
      resp_packet->data[1] = 0; //null terminate to make it a string
      chsnprintf( &resp_packet->data[strlen(resp_packet->data)], 100, "%s",global_GPGGA); //append GPS sentence
      resp_packet->data_length = strlen(resp_packet->data);
      (void)chMBPostTimeout(&mbx->tx_filled_packets, (msg_t)resp_packet,TIME_IMMEDIATE); //post the response
      return SEND_REPLY; //flag that the packet needs to be sent
      break;
*/
    case LINE_CMD:   //node line cmd
       //if argument provided, then update line
       if( cmd_packet->data_length > 1 )
       {
         node_config.node_line = strtoul(&cmd_packet->data[1], NULL, 10);
       }

       //if no arg provided, just send current status
       else
       {
         resp_packet->data[1] = 0;//null terminate to make like a string
         chsnprintf( &resp_packet->data[strlen(resp_packet->data)], 100, ";%i",node_config.node_line); //node line
         resp_packet->data_length = strlen(resp_packet->data);
         (void)chMBPostTimeout(&mbx->tx_filled_packets, (msg_t)resp_packet,TIME_IMMEDIATE); //post the response
         return SEND_REPLY; //flag that the packet needs to be sent
       }
       break;

    case STATION_CMD:   //node station cmd
       //if argument provided, then update station
       if( cmd_packet->data_length > 1 )
       {
         node_config.node_station = strtoul(&cmd_packet->data[1], NULL, 10);
       }

       //if no arg provided, just send current status
       else
       {
         resp_packet->data[1] = 0;//null terminate to make like a string
         chsnprintf( &resp_packet->data[strlen(resp_packet->data)], 100, ";%i",node_config.node_station); //node station
         resp_packet->data_length = strlen(resp_packet->data);
         (void)chMBPostTimeout(&mbx->tx_filled_packets, (msg_t)resp_packet,TIME_IMMEDIATE); //post the response
         return SEND_REPLY; //flag that the packet needs to be sent
       }
       break;

    case MSG_CMD:   //message command. Send message buffer
      resp_packet->data[1] = '0'; //null terminate
      chsnprintf( &resp_packet->data[strlen(resp_packet)], 100, "%s",message_buffer); //append message buffer contents
      resp_packet->data_length = strlen(resp_packet->data);
      (void)chMBPostTimeout(&mbx->tx_filled_packets, (msg_t)resp_packet,TIME_IMMEDIATE); //post the response
      break;
  }

 (void)chMBPostTimeout(&xbee_mbx->tx_free_packets, (msg_t)resp_packet,TIME_IMMEDIATE); //if not needed, return to pool
  return NO_REPLY; //don't send response packet
}

/*
 * ctl_thread is the central state and interface thread, receiving commands via
 * xbee/buttons, and updating node state, battery, relay etc
 * in future could add WiFi, USB etc.
 */
static THD_WORKING_AREA(wa_ctl_thread, 256);
static THD_FUNCTION(ctl_thread, arg)
{
  chprintf((BaseSequentialStream*)&SD6, "\nctl thread started\n\r");
  xbee_packet *xbee_cmd;//,*xbee_resp;
  char last_node_state, last_relay_state;
  //init ui buttons
  button_hw_init();
  //setup xbee interface
  xbee_setup();
  //initialize default relay state
  relay_state = setRelayState( RELAY_STATE_OPEN );
  last_relay_state = relay_state;

  while( true )
  {

    //xbee interface
    xbee_com_parse();   //process any new received data from xbee
    //check for new xbee messages
    msg_t msg = chMBFetchTimeout(&xbee_mbx->rx_filled_packets, (msg_t *)&xbee_cmd,TIME_IMMEDIATE);
    if( msg == MSG_OK )
    {
      //process packet
      chprintf((BaseSequentialStream*)&SD6, "\n\rxbee message received\n\r");
      chprintf((BaseSequentialStream*)&SD6, "\n\rcmd: %c\n\r",xbee_cmd->data[0]);
      memcpy(&tx_packet,xbee_cmd,sizeof(xbee_packet)); //save a copy to keep address etc

      //run the command (arguments are mailbox queue and cmd packet
      if( run_cmd(xbee_cmd, xbee_mbx) == SEND_REPLY )
      {
        xbee_send_packet();//send response packet(s) if needed
      }


      (void)chMBPostTimeout(&xbee_mbx->rx_free_packets, (msg_t)xbee_cmd,TIME_IMMEDIATE);    //free the run packet
      //xbee_send_packet();//send response packet if needed
    }
    //check for button input, update button state
    read_buttons();
    //quick and dirty for now
    if( rdy_btn_state == PRESSED )
    {
      if( !( node_state&STATE_ACTIVE ) )
      {
        node_state |= STATE_ACTIVE;
//        logging_state = READY;
//        mcp23017_digitalWrite(&mcp_cfg, 0x07, SW_5VS_PIN_POS, 1); //turn on power to input circuit
        //relay_state = setRelayState( RELAY_STATE_ADC );
      }
      else if( node_state&STATE_ACTIVE )
      {

        node_state &= ~STATE_ACTIVE;
 /*
        if( logging_state == RECORDING )
        {
          logging_state = STOP_RECORDING;
        }
        else
        {
          logging_state = NOT_READY;
        }
        //relay_state = setRelayState( RELAY_STATE_OPEN );
        mcp23017_digitalWrite(&mcp_cfg, 0x07, SW_5VS_PIN_POS, 0); //turn off power to input circuit
        //also stop any recording in progress
        */
      }

    }
    if( rdy_btn_state == LONG_HOLD )
    {
      if( logging_state == READY )
      {
        relay_state = setRelayState( RELAY_STATE_ADC );
        logging_state = START_RECORDING;
      }
    }

    /*
     * update node state
     */
    if( pps_lock )
    {
      node_state |= STATE_PPS_SYNC;
    }
    else
    {
      node_state &= ~STATE_PPS_SYNC;
    }
    /*
     * update node state recording flag based on logger state
     */
    if( logging_state == RECORDING )
    {
      node_state |= STATE_RECORDING;
    }
    else
    {
      node_state &= ~STATE_RECORDING;
    }

    /*
     * if node state, or relay state have changed, then send new value
     */
    if( last_node_state != node_state)
    {
      //check if relay state needs to be changed
      if( ( last_node_state^node_state ) & STATE_ACTIVE ) //if active status changed, then may need to switch relay
      {
        if( node_state & STATE_ACTIVE )//if now enabled, need to be in analog relay
        {
          relay_state = setRelayState( RELAY_STATE_ADC );
          mcp23017_digitalWrite(&mcp_cfg, 0x07, SW_5VS_PIN_POS, 1); //turn on power to input circuit
          set_pga(HIGH_GAIN); //always start with high gain
          raw_min_val = 0;  //reset min/max
          raw_max_val = 0;
          chThdSleepMilliseconds(20);
          raw_min_val = 0;  //reset min/max
          raw_max_val = 0;
          logging_state = READY;
        }
        else
        {
          relay_state = setRelayState( RELAY_STATE_OPEN );
          //also stop any recording in progress
          if( logging_state == RECORDING )
          {
            logging_state = STOP_RECORDING;
          }
          else
          {
            logging_state = NOT_READY;
          }
          //relay_state = setRelayState( RELAY_STATE_OPEN );
          mcp23017_digitalWrite(&mcp_cfg, 0x07, SW_5VS_PIN_POS, 0); //turn off power to input circuit

        }

      }
      //send new node status
      tx_packet.data[0] = READ_STATUS_CMD;
      tx_packet.data[1] = 0;
      tx_packet.data_length = 1;
      run_cmd( &tx_packet, xbee_mbx );
      xbee_send_packet();
      last_node_state = node_state;
    }
    if( last_relay_state != relay_state )
    {
      //send new relay state
      tx_packet.data[0] = RELAY_CMD;
      tx_packet.data[1] = 0;
      tx_packet.data_length = 1;
      run_cmd( &tx_packet, xbee_mbx );
      xbee_send_packet();
      last_relay_state = relay_state;
    }
    //send any message
    if( msg_flag )
    {
      //send new relay state
      tx_packet.data[0] = MSG_CMD;
      tx_packet.data[1] = 0;
      tx_packet.data_length = 1;
      run_cmd( &tx_packet, xbee_mbx );
      xbee_send_packet();
      msg_flag = false;
    }
    chThdSleepMilliseconds(20);

  }



}

//configure buttons as inputs

void button_hw_init()
{
  palSetLineMode(RDY_BTN_LINE,PAL_MODE_INPUT_PULLUP | PAL_STM32_OSPEED_HIGHEST);    //set status LED line mode
  palSetLineMode(PWR_BTN_LINE,PAL_MODE_INPUT_PULLUP | PAL_STM32_OSPEED_HIGHEST);    //set status LED line mode

}

/*
 * button ui thread
 */

#define HOLD_CNT 50 //hold button for 1s for long press
#define PRESS_CNT 2 //hold button for 40mS min for short press
//static THD_WORKING_AREA(wa_button_thread, 256);
//static THD_FUNCTION(button_thread, arg)

uint32_t rdy_btn_cnt = 0;
uint32_t pwr_btn_cnt = 0;
void read_buttons()//THD_FUNCTION(button_thread, arg)
{

/*
#define RDY_BTN_LINE PAL_LINE(GPIOE, 4U) //ready button input line
#define PWR_BTN_LINE PAL_LINE(GPIOE, 5U) //power button input line
*/



//  while (true)
//  {
    //read button states, update logger state
  if( !palReadLine ( RDY_BTN_LINE ) )
  {
    rdy_btn_cnt++;

    if( rdy_btn_cnt >= HOLD_CNT )
    {
      rdy_btn_state = LONG_HOLD;
      rdy_btn_cnt = HOLD_CNT;
      chprintf((BaseSequentialStream*)&SD6, "\nRDY button long hold\n\r");
    }
    else if( rdy_btn_cnt == PRESS_CNT )
    {
      rdy_btn_state = PRESSED;//should broadcast an event
      chprintf((BaseSequentialStream*)&SD6, "\nRDY button pressed\n\r");
    }
    else
    {
      rdy_btn_state = 0;
    }

  }
  else
  {
    rdy_btn_cnt = 0;
    rdy_btn_state = 0;
  }
  if( !palReadLine ( PWR_BTN_LINE ) )
  {
    pwr_btn_cnt++;

    if( pwr_btn_cnt >= HOLD_CNT )
    {
      pwr_btn_state = LONG_HOLD;
      pwr_btn_cnt = HOLD_CNT;
      chprintf((BaseSequentialStream*)&SD6, "\nPWR button hold\n\r");
    }
    else if( pwr_btn_cnt >= PRESS_CNT )
    {
      pwr_btn_state = PRESSED; //should broadcast an event
      chprintf((BaseSequentialStream*)&SD6, "\nPWR button pressed\n\r");
    }
    else
    {
      pwr_btn_state = 0;
    }
  }
  else
  {
    pwr_btn_cnt = 0;
    pwr_btn_state = 0;
  }
    /*

    //quick and dirty for now
    if( rdy_btn_state == PRESSED )
    {
      if( logging_state == NOT_READY )
      {
        logging_state = READY;
        mcp23017_digitalWrite(&mcp_cfg, 0x07, SW_5VS_PIN_POS, 1); //turn on power to input circuit
        setRelayState( RELAY_STATE_ADC );
      }
      else if( ( logging_state == RECORDING ) || ( logging_state == START_RECORDING ))
      {

        logging_state = STOP_RECORDING;
        setRelayState( RELAY_STATE_ADC );
      }
      else if( logging_state == READY )
      {

        logging_state = NOT_READY;
        setRelayState( RELAY_STATE_OPEN );
        mcp23017_digitalWrite(&mcp_cfg, 0x07, SW_5VS_PIN_POS, 0); //turn off power to input circuit
      }

    }
    if( rdy_btn_state == LONG_HOLD )
    {
      if( logging_state == READY )
      {
        setRelayState( RELAY_STATE_ADC );
        logging_state = START_RECORDING;
      }
    }
    */


//    chThdSleepMilliseconds(20);
//  }
}

/*
 * configure i2c
 */

void init_i2c(void)
{
  /* Configuring I2C SCK and I2C SDA related GPIOs .*/
   /* should be already done from board.h */
   palSetLineMode(LINE_I2C1_SCL, PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_OPENDRAIN);
   palSetLineMode(LINE_I2C1_SDA, PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_OPENDRAIN);
   palSetLineMode(MCP_RST_LINE,PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);    //set status LED line mode
//#if HTS221_SHARED_I2C
  i2cAcquireBus(&I2CD1);
//#endif /* HTS221_SHARED_I2C */
  /* Intializing the I2C. */
  i2cStart(&I2CD1, &eeprom_i2ccfg);

//#if HTS221_SHARED_I2C
    i2cReleaseBus(&I2CD1);
//#endif /* HTS221_SHARED_I2C */
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * for testing, set the rtc
   */
  /*
  RTCDateTime tmpDateTime;
  tmpDateTime.year = 40; //years since 1980
  tmpDateTime.month = 1;
  tmpDateTime.day = 30;
  tmpDateTime.dayofweek = 4;
  tmpDateTime.millisecond = 4000; //millisecond since midnight

  rtcSetTime(&RTCD1, &tmpDateTime);
*/
  /*
   * Shell manager initialization.
   */
  shellInit();

  /*
   * Activates the serial driver 6 using the driver default configuration.
   */
  sdStart(&SD6, NULL);

  /*
   * Initializes the SDIO drivers.
   */
  sdcStart(&SDCD1, &sdccfg);

  /*
   * make sure gpio are configured properly for i2c use
   */
  init_i2c();
  /*
   * initialize the pps event source
   */
  chEvtObjectInit( &evtPPS );
  /*
   * initialize the Sindex event source (internal second marker
   */
  chEvtObjectInit( &evtSindex );

  /*
   * load the node config data
   */
  read_config_eeprom(); //load node config
  if( !( '0' <= node_config.device_id[0] <= 'Z') )
  {
    //node_config.device_id[0] = '0';
    default_config_eeprom();    //save default config.
  }

  if( node_config.device_id[0] == 0x00 )
    node_config.device_id[0] = 'a';


  /*
   * Creates the button ui thread.
   */
    //chThdCreateStatic(wa_button_thread, sizeof(wa_button_thread), NORMALPRIO, button_thread, NULL);
  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(wa_blinker_thread, sizeof(wa_blinker_thread), NORMALPRIO, blinker_thread, NULL);
  /*
   * Creates the gps thread.
   */
  chThdCreateStatic(wa_gps_thread, sizeof(wa_gps_thread), NORMALPRIO, gps_thread, NULL);
  /*
   * Creates the adc thread.
   */
  init_adc(); //init adc interface and callbacks
  //chThdCreateStatic(wa_adc_thread, sizeof(wa_adc_thread), HIGHPRIO, adc_thread, NULL);
  /*
   * Creates the xbee thread.
   */
  //chThdCreateStatic(wa_xbee_thread, sizeof(wa_xbee_thread), NORMALPRIO, xbee_thread, NULL);

  /*
   * Creates the PPS timing thread.
   */
  chThdCreateStatic(wa_pps_thread, sizeof(wa_pps_thread), HIGHPRIO-2, pps_thread, NULL);

  /*
   * Creates the logger thread.
   */
  chThdCreateStatic(wa_logger_thread, sizeof(wa_logger_thread), HIGHPRIO-1, logger_thread, NULL);
  /*
   * Creates the node ctl thread.
   */
  chThdCreateStatic(wa_ctl_thread, sizeof(wa_ctl_thread), NORMALPRIO, ctl_thread, NULL);



  /*
   * Normal main() thread activity, spawning shells.
   */


   while (true) {

//    thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
//                                            "shell", NORMALPRIO + 1,
//                                            shellThread, (void *)&shell_cfg1);
//    chThdWait(shelltp);               //Waiting termination.
//*/


    chThdSleepMilliseconds(1000);
  }
}
