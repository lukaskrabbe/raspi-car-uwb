/*! ----------------------------------------------------------------------------
*  @file    ss_init_main.c
*  @brief   Single-sided two-way ranging (SS TWR) initiator example code
*
*           This is a simple code example which acts as the initiator in a SS TWR distance measurement exchange. This application sends a "poll"
*           frame (recording the TX time-stamp of the poll), after which it waits for a "response" message from the "DS TWR responder" example
*           code (companion to this application) to complete the exchange. The response message contains the remote responder's time-stamps of poll
*           RX, and response TX. With this data and the local time-stamps, (of poll TX and response RX), this example application works out a value
*           for the time-of-flight over-the-air and, thus, the estimated distance between the two devices, which it writes to the LCD.
*
*
*           Notes at the end of this file, expand on the inline comments.
* 
* @attention
*
* Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
* @author Decawave
*/
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"

#include "min.h"


#define APP_NAME "SS TWR INIT v1.3"

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 100

/* Frames used in the ranging process. See NOTE 1,2 below. */
/* IDs Here! */
/* Poller IDs should be set in the range of 100-200! */
/* Responders IDs should be set in the range of 200-300! */
/* Poller Addresses should be set in the range of 1000-1100! */
/* Responders Addresses should be set in the range of 2000-2100! */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0x00, 0x01, 0x00, 0x20, 0x00, 0x10, 0xE0, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0x00, 0x02, 0x00, 0x10, 0x00, 0x20, 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 1 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_ID_IDX 3
#define RESP_MSG_DESTINATION_ADDRESS_IDX 5
#define RESP_MSG_SOURCE_ADDRESS_IDX 7
#define RESP_MSG_TS_LEN 4
#define RESP_MSG_ID_LEN 2
#define RESP_MSG_ADDRESS_LEN 2
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
* Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
* 1 uus = 512 / 499.2 s and 1 s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Number of destination anchors to be polled. */
#define DESTINATION_NB 8

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;
static double range_bias;

/* Declaration of static functions. */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);
static void resp_msg_get_id(uint8 *id_field, uint16 *id);
static void resp_msg_get_address(uint8 *address_field, uint16 *address);
static void update_tx_rx_msg_id(uint8 *msg, uint8 *frame_seq_nb);


/*Transactions Counters */
static volatile int tx_count = 0 ; // Successful transmit counter
static volatile int rx_count = 0 ; // Successful receive counter 



//-----------------min_protocol----------------------------
struct min_context min_ctx;

static uint8 min_id = 0x81U;

struct frame {
  uint16 id;
  uint16 destination;
  uint16 source;
  double distance;
  double distance_with_range_bias;
};

struct frame frame_to_send;
/*! ------------------------------------------------------------------------------------------------------------------
* @fn main()
*
* @brief Application entry point.
*
* @param  none
*
* @return none
*/
int ss_init_run(void)
{


  /* Loop forever initiating ranging exchanges. */

  /* Modify each polling/responder message for a different anchor based on frame number */
  /* Update least significant byte of polling message destination address */
  update_tx_rx_msg_id(&tx_poll_msg[5], &frame_seq_nb);
  /* Update least significant byte of responder message PAN ID */
  update_tx_rx_msg_id(&rx_resp_msg[3], &frame_seq_nb);
  /* Update least significant byte of responder message source address */
  update_tx_rx_msg_id(&rx_resp_msg[7], &frame_seq_nb);

  /* Write frame data to DW1000 and prepare transmission. See NOTE 3 below. */
  tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
  dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
  * set by dwt_setrxaftertxdelay() has elapsed. */
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
  tx_count++;
  // printf("Transmission # : %d for anchor #: %d\r\n",tx_count, frame_seq_nb%DESTINATION_NB);


  /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 4 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  {};

    #if 0  // include if required to help debug timeouts.
    int temp = 0;		
    if(status_reg & SYS_STATUS_RXFCG )
    temp =1;
    else if(status_reg & SYS_STATUS_ALL_RX_TO )
    temp =2;
    if(status_reg & SYS_STATUS_ALL_RX_ERR )
    temp =3;
    #endif

  /* Increment frame sequence number after transmission of the poll message (modulo 256). */
  frame_seq_nb++;

  if (status_reg & SYS_STATUS_RXFCG)
  {		
    uint32 frame_len;

    /* Clear good RX frame event in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;

    if (frame_len <= RX_BUF_LEN)
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    /* Check that the frame is the expected response from the companion "SS TWR responder" example.
    * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
    rx_buffer[ALL_MSG_SN_IDX] = 0;

    if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
    {	
   

      rx_count++;
      uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
      uint16 id, destination_address, source_address;
      int32 rtd_init, rtd_resp;
      float clockOffsetRatio ;

      /* Retrieve poll transmission and response reception timestamps. See NOTE 5 below. */
      poll_tx_ts = dwt_readtxtimestamplo32();
      resp_rx_ts = dwt_readrxtimestamplo32();

      /* Read carrier integrator value and calculate clock offset ratio. See NOTE 7 below. */
      clockOffsetRatio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6) ;

      /* Get id embedded in response message. */
      resp_msg_get_id(&rx_buffer[RESP_MSG_ID_IDX], &id);
      
      frame_to_send.id = id;

      /* Get addresses embedded in response message. */
      resp_msg_get_address(&rx_buffer[RESP_MSG_DESTINATION_ADDRESS_IDX], &destination_address);
      resp_msg_get_address(&rx_buffer[RESP_MSG_SOURCE_ADDRESS_IDX], &source_address);

      frame_to_send.destination = destination_address;
      frame_to_send.source = source_address;

      /* Get timestamps embedded in response message. */
      
      resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
      resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

      /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
      rtd_init = resp_rx_ts - poll_tx_ts;
      rtd_resp = resp_tx_ts - poll_rx_ts;

      tof = ((rtd_init - rtd_resp * (1.0f - clockOffsetRatio)) / 2.0f) * DWT_TIME_UNITS; // Specifying 1.0f and 2.0f are floats to clear warning 
      distance =  tof * SPEED_OF_LIGHT;

      frame_to_send.distance = distance;
    
      range_bias = dwt_getrangebias(5, (float)(distance), DWT_PRF_64M);
      frame_to_send.distance_with_range_bias = distance-range_bias;

      min_send_frame(&min_ctx, min_id++, (uint8_t*)(&frame_to_send), 24U);
    }
  }
  else
  {
    /* Clear RX error/timeout events in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

    /* Reset RX to properly reinitialise LDE operation. */
    dwt_rxreset();
  }

  /* Execute a delay between ranging exchanges. */
  //     deca_sleep(RNG_DELAY_MS);

  //	return(1);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn resp_msg_get_ts()
*
* @brief Read a given timestamp value from the response message. In the timestamp fields of the response message, the
*        least significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to get
*         ts  timestamp value
*
* @return none
*/
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts)
{
  int i;
  *ts = 0;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
    *ts += ts_field[i] << (i * 8);
  }
}



/*! ------------------------------------------------------------------------------------------------------------------
* @fn resp_msg_get_id()
*
* @brief Read a given id value from the response message. In the id fields of the response message, the
*        least significant byte is at the lower address.
*
* @param  id_field  pointer on the first byte of the id field to get
*         id  identifier value
*
* @return none
*/
static void resp_msg_get_id(uint8 *id_field, uint16 *id)
{
  int i;
  *id = 0;
  for (i = 0; i < RESP_MSG_ID_LEN; i++)
  {
    *id += id_field[i] << (i * 8);
  }
}


/*! ------------------------------------------------------------------------------------------------------------------
* @fn resp_msg_get_address()
*
* @brief Read a given address value from the response message. In the address fields of the response message, the
*        least significant byte is at the lower address.
*
* @param  address_field  pointer on the first byte of the address field to get
*         address  address value
*
* @return none
*/
static void resp_msg_get_address(uint8 *address_field, uint16 *address)
{
  int i;
  *address = 0;
  for (i = 0; i < RESP_MSG_ADDRESS_LEN; i++)
  {
    *address += address_field[i] << (i * 8);
  }
}


/*! ------------------------------------------------------------------------------------------------------------------
* @fn update_tx_rx_msg_id()
*
* @brief Update a given poll or responder message to add new destination id. In the address fields of the response 
*        message, the least significant byte is at the lower address.
*
* @param  msg  pointer on the first byte of the message to get
*         frame_seq_nb pointer on the first byte of frame sequence number used for destination anchor calculation
*
* @return none
*/
static void update_tx_rx_msg_id(uint8 *msg, uint8 *frame_seq_nb)
{
  *msg = (*frame_seq_nb)%DESTINATION_NB; 
}


/**@brief SS TWR Initiator task entry function.
*
* @param[in] pvParameter   Pointer that will be used as the parameter for the task.
*/
void ss_initiator_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);

  //dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  dwt_setleds(DWT_LEDS_ENABLE);

  /* Init the MIN Protocol */
  min_init_context(&min_ctx, 0);

  while (true)
  {

    ss_init_run();


    char buf[32];
    uint16_t buf_len = 32;
    min_poll(&min_ctx, (uint8_t *)buf, (uint8_t)buf_len);

    /* Delay a task for a given number of ticks */
    vTaskDelay(RNG_DELAY_MS);
    /* Tasks must be implemented to never return... */
  }
}



/*! ------------------------------------------------------------------------------------------------------------------
 * Callbackes called by the Min Protocol:
 *
 */

void min_tx_start(uint8_t port)
{
  //printf("Call: min_tx_start\r\n");
}


void min_tx_finished(uint8_t port)
{
  //printf("Call: min_tx_finished\r\n");
  printf("\n");
}



// -  min_tx_byte()
//    The programmer's drivers must send a byte on the given port. The implementation of the serial port drivers
//    is in the domain of the programmer: they might be interrupt-based, polled, etc.
void min_tx_byte(uint8_t port, uint8_t byte)
{
  // printf("Call: min_tx_byte\r\n");
  printf("%02X ", byte);
}



// -  min_tx_space()
//    The programmer's serial drivers must return the number of bytes of space available in the sending buffer.
//    This helps cut down on the number of lost frames (and hence improve throughput) if a doomed attempt to transmit a
//    frame can be avoided.
// Maybe 255 Bytes ?
uint16_t min_tx_space(uint8_t port)
{
  //printf("Call: min_tx_space\r\n");
  return 512U;
}



// -  min_time_ms()
//    This is called to obtain current time in milliseconds. This is used by the MIN transport protocol to drive
//    timeouts and retransmits.
uint32_t min_time_ms(void)
{
  //printf("Call: min_time_ms\r\n");
  return dwt_readsystimestamphi32();
}


// -  min_application_handler()
//    This is the callback that provides a MIN frame received on a given port to the application. The programmer
//    should then deal with the frame as part of the application.
void min_application_handler(uint8_t min_id, uint8_t *min_payload, uint8_t len_payload, uint8_t port)
{
  //printf("Call: min_application_handler\r\n");
  //bool result = min_queue_frame(&min_ctx, min_id, min_payload, len_payload);

  //if(!result) {
  //  printf("min_application_handler failed\r\n");
  //}
}





/*****************************************************************************************************************************************************
* NOTES:
*
* 1. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
*    following:
*     - a poll message sent by the initiator to trigger the ranging exchange.
*     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
*       time-of-flight (distance) estimate.
*    The first 10 bytes of those frame are common and are composed of the following fields:
*     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
*     - byte 2: sequence number, incremented for each new frame.
*     - byte 3/4: PAN ID (0xDECA).
*     - byte 5/6: destination address, see NOTE 2 below.
*     - byte 7/8: source address, see NOTE 2 below.
*     - byte 9: function code (specific values to indicate which message it is in the ranging process).
*    The remaining bytes are specific to each message as follows:
*    Poll message:
*     - no more data
*    Response message:
*     - byte 10 -> 13: poll message reception timestamp.
*     - byte 14 -> 17: response message transmission timestamp.
*    All messages end with a 2-byte checksum automatically set by DW1000.
* 2. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
*    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
*    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
* 3. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
*    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
*    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
* 4. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
*    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
*    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
*    bytes.
* 5. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
*    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
*    subtraction.
* 6. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
*     DW1000 API Guide for more details on the DW1000 driver functions.
* 7. The use of the carrier integrator value to correct the TOF calculation, was added Feb 2017 for v1.3 of this example.  This significantly
*     improves the result of the SS-TWR where the remote responder unit's clock is a number of PPM offset from the local inmitiator unit's clock.
*     As stated in NOTE 2 a fixed offset in range will be seen unless the antenna delsy is calibratred and set correctly.
*
****************************************************************************************************************************************************/
