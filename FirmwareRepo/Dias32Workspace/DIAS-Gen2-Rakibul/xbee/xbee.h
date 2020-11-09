/*
* xbee.h
*
* written by Matthew Friesen 2019
*/

#ifndef XBEE_H
#define XBEE_H

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

//#define DEBUG_XBEE
//#define XBEE_debug

#define FRAME_START_BYTE 0x7E  //xbee digimesh start packet indicator
#define FRAME_MIN_PACKET_LENGTH 5
#define FRAME_LENGTH_OVERHEAD 12
//TX frame offsets
#define FRAME_LENGTH_OFFSET_H 1
#define FRAME_LENGTH_OFFSET_L 2
#define FRAME_DATA_OFFSET 3
#define FRAME_TYPE_OFFSET 3
//#define FRAME_ID_OFFSET 4
#define FRAME_SRC_ADDR_OFFSET_H 4 //4 byte address
#define FRAME_SRC_ADDR_OFFSET_L 8 //4 byte address
//RX frame offsets
#define FRAME_RX_DATA_OFFSET 15
#define FRAME_RX_BCAST_OFFSET 14
#define FRAME_BCAST_MASK 0x02
//TX frame offset
#define FRAME_TX_ADDR_OFFSET_H 5 //4 byte address
#define FRAME_TX_ADDR_OFFSET_L 9 //4 byte address
#define FRAME_TX_ID_OFFSET 4 //frame ID
#define FRAME_TX_RES1_OFFSET 13 //reserved
#define FRAME_TX_RES2_OFFSET 14
#define FRAME_TX_BCAST_OFFSET 15
#define FRAME_TX_OPPS_OFFSET 16
#define FRAME_TX_DATA_OFFSET 17

#define TX_BCAST_RADIUS 0x00
#define TX_OPPS 0xC1 //digimesh, no ACK

#define TX_FRAME 0x10  //transmit frame type
#define RX_FRAME 0x90  //receive frame type
#define AT_RESP 0x88    //AT response frame
#define AT_REQ 0x08     //AT request frame
#define TX_STATUS 0x8B  //transmit status frame
#define REMOTE_AT_REQ 0x17  //remote AT command request
#define TX_STAT 0x8B    //transmit statue
//#define TX_OPPS 0xFFFE00C1  //default transmit options ( const. 16 bit src address, no ack, digimesh )
#define TX_PACKET_OVERHEAD 14 //tx packet length is data+14
 SerialDriver* xbee_stream;
 BaseSequentialStream* debug_stream;

#define XBEE_COM_BUF_SIZE 256 //256 byte input buffer for buffering xbee packet
#define XBEE_CMD_BUF_SIZE 256   //256 byte buffer to hold an xbee packet
#define XBEE_PKT_TIMEOUT_MS 100 //max delay between packet chars

#define DEFAULT_FRAME_ID 1 //



//Xbee parsed data
typedef struct {
//  char start_byte;
  uint16_t packet_length;  //number of bytes between length and checksum
  char type;        //Frame type
  char address_H[4];   //high address 32 bit
  char address_L[4];   //low address 32 bit
//  uint16_t reserved;
  char receive_options;
  uint16_t data_length; //number of bytes in the data array
  char data[80];

} xbee_packet;

#define NUM_PACKET_BUFFERS 5
typedef struct {
  /* xbee received packet mailboxes */
  mailbox_t rx_filled_packets;
  mailbox_t rx_free_packets;
  msg_t rx_free_packet_queue[NUM_PACKET_BUFFERS];
  msg_t rx_filled_packet_queue[NUM_PACKET_BUFFERS];

  mailbox_t tx_filled_packets;
  mailbox_t tx_free_packets;
  msg_t tx_free_packet_queue[NUM_PACKET_BUFFERS];
  msg_t tx_filled_packet_queue[NUM_PACKET_BUFFERS];

  xbee_packet rx_packet_buffers[NUM_PACKET_BUFFERS]; //buffer up to 5 xbee packets
  xbee_packet  tx_packet_buffers[NUM_PACKET_BUFFERS]; //buffer up to 5 xbee packets

}xbee_mailbox_t;


xbee_mailbox_t* xbee_init(SerialDriver* xbee_stream_val,SerialDriver* debug_stream_val);
uint8_t xbee_process_packet(void);
void xbee_com_parse(void);
void xbee_send_packet(void);


#endif /* XBEE_H */
