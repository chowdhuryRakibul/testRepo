/*
    ChibiCopter - https://github.com/grantmd/ChibiCopter
    A quadcopter platform running under ChibiOS/RT.

    Talks to a GPS over serial
*/

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#ifndef _GPS_H_
#define _GPS_H_

//NMEA sentences for gnss
typedef enum GPS_SENTENCE {
  GPS_SENTENCE_UNKNOWN    =   0x00,
  GPS_SENTENCE_GNGGA    =   0x01, // Time, position, and fix/satellite info
  GPS_SENTENCE_GNGLL    =   0x02, // Time, position, some speed/heading
  GPS_SENTENCE_GNGSA    =   0x03, // Time, position
  GPS_SENTENCE_GNRMC    =   0x04, // GPS DOP and active satellites
  GPS_SENTENCE_GNVTG    =   0x05, // Satellites in view
  GPS_SENTENCE_GNZDA    =   0x06 // Track made good and Ground speed
} GPS_SENTENCE;

#define RMC_TIME_POS 1
#define RMC_STAT_POS 2
#define RMC_LAT_POS 3
#define RMC_NS_POS 4
#define RMC_LON_POS 5
#define RMC_EW_POS 6
#define RMC_DATE_POS 9




//GPS parsed data
typedef struct {
  char fix_status;
  char lat[12];
  char lat_NS; //N or S
  char lon[12];
  char lon_EW; //E or W
  char elev[6];
  char utc_time[9];
  char utc_date[9];
  char utc_time_hh[3];
  char utc_time_mm[3];
  char utc_time_ss[3];
  char utc_date_yy[3];
  char utc_date_mm[3];
  char utc_date_dd[3];
  bool gps_fix;

} gps_data_struct;

#define _GPGGA_TERM   "GGA"
#define _GPRMC_TERM   "RMC"
#define _GPGLL_TERM   "GLL"
#define _GPGSA_TERM   "GSA"
#define _GPGSV_TERM   "GSV"
#define _GPVTG_TERM   "VTG"

#define MAX_NMEA_LENGTH 100
#define TMP_GPS_BUFF_LENGTH MAX_NMEA_LENGTH
char global_GPRMC[MAX_NMEA_LENGTH];
char global_GPGGA[MAX_NMEA_LENGTH];
char tmp_GPS_buffer[TMP_GPS_BUFF_LENGTH]; //buffer for un-parsed GPS data (enough for 2 sentences
event_source_t evtGPS; //GPS event source to signal new sentences to other threads
float global_LAT,global_LON; //global latitude and longitude
char global_GPS_STATUS; //global var for gps lock state
gps_data_struct gps_data; //parsed data in a struct

//event flags to distinguish which type of event it is
typedef enum GPS_EVT_FLAG {
  GPS_NEW_RMC = 0x01,
  GPS_NEW_GGA = 0x02

} GPS_EVT_FLAG;



// Public functions
char GPSParseChar(char);
char gps_test_checksum(void);
uint8_t hex_to_int(char ch);
char gps_parse_sentence(void);
char gps_parse_RMC(void);
char gps_parse_GGA(void);


void gps_init(void);



#endif /* _GPS_H_ */
