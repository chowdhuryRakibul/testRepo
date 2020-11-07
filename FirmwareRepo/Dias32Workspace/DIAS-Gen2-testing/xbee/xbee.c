/*
* xbee.c
*
* written by Matthew Friesen 2019
*
* This is a driver for interfacing with xbee PRO modules in api mode using
* the digimesh firmware.
*/
#include "xbee.h"
#include <string.h>
uint16_t buf_start_pos = 0;
uint16_t buf_end_pos = 0; //index pointers to head and tail of input buffer
uint8_t xbee_cmd_buffer[XBEE_CMD_BUF_SIZE];
uint8_t xbee_com_buffer[XBEE_COM_BUF_SIZE];
uint8_t xbee_tx_buffer[XBEE_COM_BUF_SIZE];

xbee_packet new_packet;
uint16_t packet_length;
static xbee_mailbox_t xbee_mailbox;

uint32_t lastXB_RX_mS = 0; //timestamp of last Xbee received byte
uint32_t lastXB_CMD_mS = 0; //timestamp of last Xbee received cmd

/*
 * initialize mailbox etc
 */
xbee_mailbox_t* xbee_init(SerialDriver* xbee_stream_val,SerialDriver* debug_stream_val )
{
  //setup serial streams
  xbee_stream = xbee_stream_val;
  debug_stream = debug_stream_val;


  /* Creating rx the mailboxes.*/
  chMBObjectInit(&xbee_mailbox.rx_filled_packets, xbee_mailbox.rx_filled_packet_queue, NUM_PACKET_BUFFERS);
  chMBObjectInit(&xbee_mailbox.rx_free_packets, xbee_mailbox.rx_free_packet_queue,NUM_PACKET_BUFFERS);

  /* Pre-filling the free buffers pool with the available buffers*/
  uint32_t i;
  for (i = 0; i < NUM_PACKET_BUFFERS; i++)
  {
    (void)chMBPostTimeout(&xbee_mailbox.rx_free_packets, (msg_t)&xbee_mailbox.rx_packet_buffers[i],TIME_IMMEDIATE);
  }

  /* Creating tx the mailboxes.*/
  chMBObjectInit(&xbee_mailbox.tx_filled_packets, xbee_mailbox.tx_filled_packet_queue, NUM_PACKET_BUFFERS);
  chMBObjectInit(&xbee_mailbox.tx_free_packets, xbee_mailbox.tx_free_packet_queue,NUM_PACKET_BUFFERS);

  /* Pre-filling the free buffers pool with the available buffers*/
  for (i = 0; i < NUM_PACKET_BUFFERS; i++)
  {
    (void)chMBPostTimeout(&xbee_mailbox.tx_free_packets, (msg_t)&xbee_mailbox.tx_packet_buffers[i],TIME_IMMEDIATE);
  }
  return &xbee_mailbox;

}

/*
 * Receives incomming bytes from xbee, detects a valid packet and verifies
 * the checksum. Valid packets are forwarded to packet parser
 */
void xbee_com_parse()
{
  //update interface freshness timer
  //append new character to input buffer, then test for valid packet
//  uint8_t error = false;
  msg_t readByte;
  uint32_t cur_mS;
  char newByte;

  //read a byte from xbee radio,
  readByte = sdGetTimeout(xbee_stream, TIME_IMMEDIATE);

  /* Checking if a timeout has occurred. */
  while(readByte != MSG_TIMEOUT)
  {
    cur_mS = TIME_I2MS( chVTGetSystemTime() );
    //test for packet reset timeout since last char
    if( ( ( cur_mS - lastXB_RX_mS ) > XBEE_PKT_TIMEOUT_MS ) && ( buf_end_pos != 0 ) )
    {

//#ifdef XBEE_debug
      chprintf(debug_stream, "\n\rcorrupted xbee packet\n\r",(char)readByte);
//#endif
      buf_end_pos = 0;    //reset buffer
    }
    lastXB_RX_mS = cur_mS;
    /* Not a timeout. Echoing the character to improve user experience. */
#ifdef XBEE_debug
    chprintf(debug_stream, "0x%02X ",(char)readByte);
#endif

    newByte = (char)readByte;

    /*
     * test for unexpected full buffer or start of new buffer
     */
    if( ( newByte == FRAME_START_BYTE ) || ( buf_end_pos == XBEE_COM_BUF_SIZE ) )
    {
      if( buf_end_pos != 0 )
      {
        //send a warning/debug message to show a corrupted packet
//#ifdef XBEE_debug
        chprintf(debug_stream, "\n\rcorrupted xbee packet\n\r",(char)readByte);
//#endif
      }
      buf_end_pos = 0; //reset buffer
    }
    xbee_com_buffer[buf_end_pos++] = newByte;

    /*
     * if we have enough data, we can start looking at the buffer to check
     * for a valid packet
     */
    if( ( xbee_com_buffer[0] == FRAME_START_BYTE ) && ( buf_end_pos >= FRAME_MIN_PACKET_LENGTH ) )
    {
      /*
       * if the current buffer size is equal to standard packet + data payload,
       * then we should have a valid packet so test the checksum
       */
      new_packet.packet_length = ( xbee_com_buffer[FRAME_LENGTH_OFFSET_H] << 8 ) + xbee_com_buffer[ FRAME_LENGTH_OFFSET_L ];


#ifdef XBEE_debug
      chprintf(debug_stream, "\n\rXbee Packet: length: %i\n\r", new_packet.packet_length );
#endif

      if( buf_end_pos == ( new_packet.packet_length + FRAME_DATA_OFFSET + 1 ) )
      {
        // test the checksum
        uint8_t i;
        uint8_t xbee_checksum = 0;
        for(i=FRAME_DATA_OFFSET;i < ( buf_end_pos - 1 );i++)
        {
          xbee_checksum += xbee_com_buffer[i];
  //#ifdef XBEE_debug
          chprintf( debug_stream, "0x%02X ", xbee_com_buffer[i]);
  //#endif

        }
        xbee_checksum = 0xFF - xbee_checksum;
  #ifdef XBEE_debug
        chprintf( debug_stream, "\n\rXbee packet checksum: 0x%02X, 0x%02X", xbee_checksum, xbee_com_buffer[i]);
  #endif
        if( xbee_checksum == xbee_com_buffer[i])
        {
#ifdef XBEE_debug
          chprintf( debug_stream, "\n\rchecksum match, valid packet\n\r");
          //chprintf( debug_stream, "\n\rsource address: 0x%08X, 0x%08X\n\r", (uint32_t)*(xbee_com_buffer+FRAME_SRC_ADDR_OFFSET_H),(uint32_t)*(&xbee_com_buffer[FRAME_SRC_ADDR_OFFSET_H]));
#endif
          //copy relevent data to the packet struct
          new_packet.type = xbee_com_buffer[FRAME_TYPE_OFFSET];   //packet type
          xbee_process_packet();
          buf_end_pos = 0; //reset buffer
        }

      }

    }
    //try reading another byte from xbee
    readByte = sdGetTimeout(xbee_stream, chTimeUS2I( 15 )); //wait up to 15uS for next char.
  }

}

/*
 * sends a packet to xbee
 */
void xbee_send_packet()
{
  xbee_packet *tx_packet;
//  char checksum = 0;

  //check for new packets to transmit
  while(chMBFetchTimeout(&xbee_mailbox.tx_filled_packets, (msg_t *)&tx_packet, TIME_IMMEDIATE) == MSG_OK)
  {
//#ifdef XBEE_debug
    chprintf( debug_stream, "\n\rXbee send data[] = %s\n\r",tx_packet->data);
          //chprintf( debug_stream, "\n\rsource address: 0x%08X, 0x%08X\n\r", (uint32_t)*(xbee_com_buffer+FRAME_SRC_ADDR_OFFSET_H),(uint32_t)*(&xbee_com_buffer[FRAME_SRC_ADDR_OFFSET_H]));
//#endif
    if( tx_packet->type == RX_FRAME )
    {
      //this is a response to a cmd, set packet type to TX
      tx_packet->type = TX_FRAME;
    }
    tx_packet->packet_length = tx_packet->data_length + TX_PACKET_OVERHEAD;
    xbee_tx_buffer[0] = FRAME_START_BYTE;
    xbee_tx_buffer[FRAME_LENGTH_OFFSET_H] = tx_packet->packet_length >> 8;
    xbee_tx_buffer[FRAME_LENGTH_OFFSET_L] = (uint8_t)tx_packet->packet_length;
    xbee_tx_buffer[ FRAME_TYPE_OFFSET ] = tx_packet->type;
    xbee_tx_buffer[ FRAME_TX_ID_OFFSET ] = DEFAULT_FRAME_ID;
    memcpy( &xbee_tx_buffer[ FRAME_TX_ADDR_OFFSET_H ], tx_packet->address_H, 4);
    memcpy( &xbee_tx_buffer[ FRAME_TX_ADDR_OFFSET_L ], tx_packet->address_L, 4);
    xbee_tx_buffer[ FRAME_TX_RES1_OFFSET ] = 0xFF; //reserved values
    xbee_tx_buffer[ FRAME_TX_RES2_OFFSET ] = 0xFE;
    xbee_tx_buffer[ FRAME_TX_BCAST_OFFSET ] = TX_BCAST_RADIUS; //default
    xbee_tx_buffer[ FRAME_TX_OPPS_OFFSET ] = TX_OPPS; //digimesh, no ACK
    memcpy( &xbee_tx_buffer[ FRAME_TX_DATA_OFFSET ], tx_packet->data, tx_packet->data_length );

    //calculate checksum
    uint8_t i,checksum;
    checksum = 0;
    for(i = FRAME_TYPE_OFFSET; i < FRAME_TYPE_OFFSET + tx_packet->packet_length; i++)
    {
      checksum += xbee_tx_buffer[i];
    }
    checksum = 0xFF-checksum;
    xbee_tx_buffer[i] = checksum;


    msg_t result = sdWrite(xbee_stream,xbee_tx_buffer, tx_packet->packet_length+4);
    if( result != MSG_OK )
    {
#ifdef XBEE_debug
          chprintf( debug_stream, "\n\rxbee send failed\n\r");
          //chprintf( debug_stream, "\n\rsource address: 0x%08X, 0x%08X\n\r", (uint32_t)*(xbee_com_buffer+FRAME_SRC_ADDR_OFFSET_H),(uint32_t)*(&xbee_com_buffer[FRAME_SRC_ADDR_OFFSET_H]));
#endif
    }
    (void)chMBPostTimeout(&xbee_mailbox.tx_free_packets, (msg_t)tx_packet,TIME_IMMEDIATE);
  }

}

/*
 * Processes valid xbee packets.
 * most packet will be ignored, command packets will be sent on
 */
uint8_t xbee_process_packet()
{
  // act based on the packet type

  switch ( new_packet.type  )
  {
    // RX packet type. Assume it is a command.
    case RX_FRAME:

      lastXB_CMD_mS = TIME_I2MS( chVTGetSystemTime() );
      //fill out packet struct
      new_packet.data_length = new_packet.packet_length - FRAME_LENGTH_OVERHEAD;
      memcpy(new_packet.data, &xbee_com_buffer[FRAME_RX_DATA_OFFSET],new_packet.data_length );
      new_packet.data[new_packet.data_length] = 0;//null terminate to make it a string
      memcpy(new_packet.address_H, &xbee_com_buffer[FRAME_SRC_ADDR_OFFSET_H], 4);
      memcpy(new_packet.address_L, &xbee_com_buffer[FRAME_SRC_ADDR_OFFSET_L], 4);
      void *pbuf;

      if (chMBFetchTimeout(&xbee_mailbox.rx_free_packets, (msg_t *)&pbuf, TIME_IMMEDIATE) == MSG_OK)
      {
        memcpy(pbuf,&new_packet,sizeof(new_packet)); //copy new packet contents to message buffer
        (void)chMBPostTimeout(&xbee_mailbox.rx_filled_packets, (msg_t)pbuf,TIME_IMMEDIATE);
      }
      /*
      uint8_t j;
      for( j=0; j<new_packet.data_length; j++ )
      {
        new_packet.data[j] = xbee_com_buffer[j+FRAME_RX_DATA_OFFSET]
      }*/



      break;
    case AT_RESP:
      break;
    case TX_STATUS:
//#ifdef XBEE_debug
      chprintf( debug_stream, "\n\rxbee TX status packet received\n\r");
//#endif
      break;
  }
  return 0;

}
