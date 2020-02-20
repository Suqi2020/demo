#ifndef __ATCMD_H__
#define __ATCMD_H__
#include "stdint.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm8l15x_rtc.h"




#define LORAWAN_APP_DATA_MAX_SIZE                           60


extern  uint32_t g_freq_rx;

extern  uint32_t g_freq_tx;


extern uint8_t g_sf_tx;//‘› ±…Ë÷√Œ™5
extern uint8_t g_dt_rx;
/*!
 * User application data
 */
extern uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];
extern uint8_t AppDataSize;
#define LENTH    255
//#define TX_BUF_SIZE     255


extern  u8 gu8Buffer[LENTH];
extern  u8 gu8RecFlag;
extern u16 Len;
extern void scgw_uart_send(char * string, u16 length);
typedef int( * TestCaseHandler )( int argc, char *argv[] );
typedef struct TestCaseSt_ {
    char name[32];
    TestCaseHandler fn;
}TestCaseSt;
enum{
  RADIO_CHANNEL_NONE = 0,
  RADIO_CHANNEL_RX,
  RADIO_CHANNEL_TX,
  RADIO_CHANNEL_RX_TX,
  RADIO_CHANNEL_MAX
};

extern int ATCmdFun();





#endif