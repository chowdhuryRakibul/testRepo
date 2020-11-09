/*
    ChibiCopter - https://github.com/grantmd/ChibiCopter
    A quadcopter platform running under ChibiOS/RT.

    Talks to a GPS receiver over serial
*/

#include <string.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"


#include "GPS.h"

//#define GPS_debug 1

typedef enum GPS_ERROR {
  E_OK                =   0x00,
  E_READ_TIMEOUT      =   0x01,
  E_INVALID_DATA      =   0x02,
  E_LEN_ERROR         =   0x03,
  E_CHKSUM_ERROR      =   0x04,
  E_MSG_TYPE_ERR      =   0x05,
  E_MISSED_CHARS      =   0x06    //wrong data, $ not preceded by \n or \r, ie characters missed

} GPS_ERROR;

mutex_t  gpsMutex;        //mutex for controlling sharing the data buffer

char local_GPRMC[MAX_NMEA_LENGTH];
char local_GPGGA[MAX_NMEA_LENGTH];

/*
 * Parse a character from incoming GPS data using a state machine
 */

/* Appends the char to a working buffer, then when a full sentence has been received, validate the sentence and parses it
 *
 * Returns whether we have completed a valid sentence
 */
uint16_t tmp_buffer_index = 0; //index for gps input buffer
char GPSParseChar(char c){
  char result = E_OK;
  tmp_GPS_buffer[tmp_buffer_index] = c; //save the received char to the buffer
  tmp_GPS_buffer[++tmp_buffer_index] = 0; //null terminate buffer to make a valid string
  //chprintf((BaseSequentialStream*)&SD6, "\r\nin GPSParseChar(char %c)\n", c);

  //start by making sure we don't over-run our buffer
  if( tmp_buffer_index > ( MAX_NMEA_LENGTH-1 ) )
  {
    result = E_LEN_ERROR;
    tmp_buffer_index = 0;
    return result;
  }

  switch(c){
    case '$':
      // start of a sentence
      // if buffer index isn't 1 already, then we missed some input bytes
      if( tmp_buffer_index != 1 )
      {
        result = E_MISSED_CHARS;
        tmp_buffer_index = 0;
        tmp_GPS_buffer[tmp_buffer_index++] = c;
        tmp_GPS_buffer[tmp_buffer_index] = 0; //null terminate
        return result;
      }
      break;

    case '\r':
    case '\n':
      if( tmp_buffer_index > 1 )
      {
        // end of sentence
        //replace \n or \r with Null to make a valid sentence
        tmp_GPS_buffer[tmp_buffer_index-1] = 0; //null terminate

        // verify checksum of sentence (but only if there is data in the buffer)
#ifdef GPS_debug
        chprintf((BaseSequentialStream*)&SD6, "\n\rGPS sentence: %s", tmp_GPS_buffer);
#endif
        result = gps_test_checksum();
        if( result == E_OK )
        {
          //parse the sentence
          gps_parse_sentence();
        }
        else
        {
          return result;
        }
      }
      tmp_buffer_index = 0; //end of sentence, reset the buffer index.
      break;

  }
  return result;

}

void gps_init(void)
{
  chMtxObjectInit( &gpsMutex );    //init gps data sharing mutex
  chEvtObjectInit( &evtGPS );
}

/*
 * identify NMEA sentence type and call appropriate parser
 */
char gps_parse_sentence()
{
  char result;
  const char type_rmc[] = "RMC";
  const char type_gga[] = "GGA";
  //compare the relevant characters in the sentence to check for the sentences we are interested in
#ifdef GPS_debug
  chprintf((BaseSequentialStream*)&SD6, "\n\rGPS type: ");
  streamWrite((BaseSequentialStream*)&SD6, &tmp_GPS_buffer[3], 3 );
  chprintf((BaseSequentialStream*)&SD6, "\n\r");
#endif

  if( strncmp(&tmp_GPS_buffer[3],type_rmc,3) == 0 ) //$GPRMC test
  {
#ifdef GPS_debug
    chprintf((BaseSequentialStream*)&SD6, "\n\rIdentified RMC sentence \n\r");
#endif
    strncpy(local_GPRMC,tmp_GPS_buffer,tmp_buffer_index);
    chMtxLock( &gpsMutex );
    strncpy(global_GPRMC,tmp_GPS_buffer,tmp_buffer_index);
    chMtxUnlock( &gpsMutex );
    chEvtBroadcastFlags(&evtGPS, GPS_NEW_RMC);    //parse some of the data
    char *last_field_pos = local_GPRMC;
    char *cur_field_pos = local_GPRMC;
//    char *tmp_GPRMC = local_GPRMC;
    uint8_t field_index = 1;
    cur_field_pos = strchr(local_GPRMC,',')+1;  //strchr returns position of the ",", so add 1 to get to first pos of next field
    while( field_index < 10 )
    {
      if( field_index == RMC_STAT_POS )
      {
        gps_data.fix_status = cur_field_pos[0];
        if( gps_data.fix_status == 'A' )
        {
          gps_data.gps_fix = true;
        }
        else
        {
          gps_data.gps_fix = false;
        }

      }
      //only parse GPS data if fix is good
      else if( gps_data.fix_status == 'A' )
      {
        if( field_index == RMC_TIME_POS )
          strncpy(gps_data.utc_time,cur_field_pos,9);
        else if( field_index == RMC_DATE_POS )
          strncpy(gps_data.utc_date,cur_field_pos,6);
        else if( field_index == RMC_LAT_POS )
          strncpy(gps_data.lat,cur_field_pos,10);
        else if( field_index == RMC_LON_POS )
          strncpy(gps_data.lon,cur_field_pos,10);
        else if( field_index == RMC_NS_POS )
          gps_data.lat_NS = cur_field_pos[0];
        else if( field_index == RMC_EW_POS )
          gps_data.lon_EW = cur_field_pos[0];
      }
#ifdef GPS_debug
      chprintf((BaseSequentialStream*)&SD6, "token:   \n\r");
#endif
      last_field_pos = cur_field_pos;
      cur_field_pos = strchr(last_field_pos,',')+1; //strchr returns position of the ",", so add 1 to get to first pos of next field
      field_index++;
    }

#ifdef GPS_debug
    chprintf((BaseSequentialStream*)&SD6, "\n\r saved RMC sentence: %s \n\r",local_GPRMC);
#endif
    result = E_OK;
  }
  else if( strncmp(&tmp_GPS_buffer[3],type_gga,3) == 0 ) //$GPGGA test
  {
#ifdef GPS_debug
    chprintf((BaseSequentialStream*)&SD6, "\n\rIdentified GGA sentence \n\r");
#endif
    strncpy(local_GPGGA,tmp_GPS_buffer,tmp_buffer_index);
    chMtxLock( &gpsMutex );
    strncpy(global_GPGGA,tmp_GPS_buffer,tmp_buffer_index);
    chMtxUnlock( &gpsMutex );
    chEvtBroadcastFlags(&evtGPS, GPS_NEW_GGA);


#ifdef GPS_debug
    chprintf((BaseSequentialStream*)&SD6, "\n\r saved GGA sentence: %s \n\r",local_GPGGA);
#endif
    result = E_OK;
  }
  return result;

}
//char gps_parse_RMC(void);
//char gps_parse_GGA(void);

/*
 * Verify checksum for a complete NMEA sentence. Assumes the first char in the buffer is the '$' and the last is the checksum.
 */
char gps_test_checksum()
{
  uint8_t i = 0;
  uint8_t checksum_val = 0;
  char checksum = 0;
  //eg NMEA string: "$GPRMC*AB\n" should only calculate the checksum for "GPRMC and comparte to "AB"
  for(i=1;i<tmp_buffer_index-4;i++ )
  {
     checksum = checksum^tmp_GPS_buffer[i];
  }
  checksum_val = ( hex_to_int( (char)tmp_GPS_buffer[i+1] ) << 4 ) + hex_to_int( (char)tmp_GPS_buffer[i+2] );
#ifdef GPS_debug
#if GPS_debug >= 2
  chprintf((BaseSequentialStream*)&SD6, "\n\rcalculated GPS checksum: 0x%x, reference checksum: 0x%x", checksum,checksum_val);
#endif
#endif
  if( checksum_val == checksum )
  {
    return E_OK;
  }
  else
  {
    return E_CHKSUM_ERROR;
  }


}

//function : hex_to_int
//this function will return number corresponding
//0,1,2..,9,A,B,C,D,E,F

uint8_t hex_to_int(char ch)
{
    int num=0;
    if(ch>='0' && ch<='9')
    {
        num=ch-0x30;
    }
    else
    {
        switch(ch)
        {
            case 'A': case 'a': num=10; break;
            case 'B': case 'b': num=11; break;
            case 'C': case 'c': num=12; break;
            case 'D': case 'd': num=13; break;
            case 'E': case 'e': num=14; break;
            case 'F': case 'f': num=15; break;
            default: num=0;
        }
    }
    return num;
}

