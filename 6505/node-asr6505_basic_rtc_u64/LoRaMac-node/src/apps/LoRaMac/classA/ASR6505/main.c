/*!
 * \file      main.c
 *
 * \brief     LoRaMac classA /C device implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */


/*
GPIO_ResetBits(GPIOE, GPIO_Pin_6);
DelayMs(1);
GPIO_SetBits(GPIOE, GPIO_Pin_6);

//send
GPIO_ResetBits(GPIOE, GPIO_Pin_7);
DelayMs(1);
GPIO_SetBits(GPIOE, GPIO_Pin_7);
//OnRxWindow1TimerEvent
GPIO_ResetBits(GPIOC, GPIO_Pin_0);
DelayMs(1);
GPIO_SetBits(GPIOC, GPIO_Pin_0);
//OnRxWindow2TimerEvent
GPIO_ResetBits(GPIOC, GPIO_Pin_1);
DelayMs(1);
GPIO_SetBits(GPIOC, GPIO_Pin_1);

//OnRadioRxTimeout
GPIO_ResetBits(GPIOC, GPIO_Pin_2);
DelayMs(1);
GPIO_SetBits(GPIOC, GPIO_Pin_2);
*/

/*! \file classA/ASR6505/main.c */
#include <stdio.h>
#include "stm8l15x.h"
#include "stm8l15x_gpio.h"
#include "board.h"
#include "utilities.h"
#include "board.h"
#include "gpio.h"
#include "LoRaMac.h"
#include "Commissioning.h"
#include "uart-board.h"
#include "atcmd.h"
#include "LoRaMacClassB.h"
#include "rtc-board.h"
#include "stm8l15x_flash.h"
DeviceClass_t WORKMODE ;  //CLASS_C
#ifndef ACTIVE_REGION

#warning "No active region defined, LORAMAC_REGION_CN470 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_CN470

#endif
static uint8_t g_beacon_retry_times = 0;
 LWanDevConfig_t *g_lwan_dev_config_p = NULL;
#define MAX_BEACON_RETRY_TIMES 2


/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    true

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            2
 #if 0
 uint8_t DevEui[8] = LORAWAN_DEVICE_EUI;
 uint8_t AppEui[8] = LORAWAN_APPLICATION_EUI;
 uint8_t AppKey[16] = LORAWAN_APPLICATION_KEY;
#endif
uint8_t DevEui[8] = {0};
uint8_t AppEui[8] =  {0};
uint8_t AppKey[16] =  {0};

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif
volatile uint8_t gu8Join_bz=0;

static uint8_t gGatewayID[3] ={0};

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
 uint8_t AppDataSize = 4;
/*!
 * User application data buffer size
 */
//#define LORAWAN_APP_DATA_MAX_SIZE                           60

/*!
 * User application data
 */
uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
//static uint32_t TxDutyCycleTime = APP_TX_DUTYCYCLE;

/*!
 * Timer to handle the application data transmission duty cycle
 */
 TimerEvent_t TxNextPacketTimer;

/*!
 * Indicates if a new packet can be sent
 */
 volatile bool NextTx = true;

/*!
 * Device states
 */
 eDeviceState  DeviceState;
/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    AppDataSize = 4;
    AppData[0] = 0x00;
    AppData[1] = 0x01;
    AppData[2] = 0x02;
    AppData[3] = 0x03;
}

/*

static LoRaMacClassBParams_t LoRaMacClassBParams;
uint8_t LoRaMacClassBPingSlotChannelReq( uint8_t datarate, uint32_t frequency )
{
#ifdef LORAMAC_CLASSB_ENABLED
    uint8_t status = 0x03;
    VerifyParams_t verify;
    bool isCustomFreq = false;

    if( frequency != 0 )
    {
        isCustomFreq = true;
        if( Radio.CheckRfFrequency( frequency ) == false )
        {
            status &= 0xFE; // Channel frequency KO
        }
    }

    verify.DatarateParams.Datarate = datarate;
    verify.DatarateParams.DownlinkDwellTime = LoRaMacClassBParams.LoRaMacParams->DownlinkDwellTime;

    if( RegionVerify( *LoRaMacClassBParams.LoRaMacRegion, &verify, PHY_RX_DR ) == false )
    {
        status &= 0xFD; // Datarate range KO
    }

    if( status == 0x03 )
    {
        if( isCustomFreq == true )
        {
            PingSlotCtx.Ctrl.CustomFreq = 1;
            PingSlotCtx.Frequency = frequency;
        }
        else
        {
            PingSlotCtx.Ctrl.CustomFreq = 0;
            PingSlotCtx.Frequency = 0;
        }
        PingSlotCtx.Datarate = datarate;
    }

    return status;
#else
    return 0;
#endif // LORAMAC_CLASSB_ENABLED
}
*/
/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );
	//
	//RTC_AlarmCmd(ENABLE);//suqi

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = false;//true
			return;
        }
        else
        {
            // Network not joined yet. Try to join again
            MlmeReq_t mlmeReq;
            mlmeReq.Type = MLME_JOIN;
            mlmeReq.Req.Join.DevEui = DevEui;
            mlmeReq.Req.Join.AppEui = AppEui;
            mlmeReq.Req.Join.AppKey = AppKey;
            mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;

            if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
            {
                DeviceState = DEVICE_STATE_SLEEP;
				return;
            }
            else
            {
                DeviceState = DEVICE_STATE_CYCLE;
				return;
            }
        }
    }
    DeviceState = DEVICE_STATE_CYCLE;
    return;
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }
    }
    NextTx = true; 
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    printf( "receive data: rssi = %d, snr = %d, datarate = %d\r\n", mcpsIndication->Rssi, (int)mcpsIndication->Snr,
                 (int)mcpsIndication->RxDatarate);
    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    if( mcpsIndication->FramePending == true )
    {
        // The server signals that it has pending data to be sent.
        // We schedule an uplink as soon as possible to flush the server.
      // printf("MlmeIndication\r\n");
        OnTxNextPacketTimerEvent( );
     
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot
    }
    if( mcpsIndication->RxData == true )
    {
    }
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
const static uint8_t JoinOK_str[]="AT+JOIN=OK\r\n";
const static uint8_t JoinERR_str[]="AT+JOIN=ERR\r\n";
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    MibRequestConfirm_t mibReq;
    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                printf("joined\r\n");
                scgw_uart_send((char*)JoinOK_str,sizeof(JoinOK_str));
                // Status is OK, node has joined the network
                
                if(WORKMODE == CLASS_B) {
                    DeviceState = DEVICE_STATE_REQ_DEVICE_TIME;
                }else{
                  DeviceState = DEVICE_STATE_CYCLE;
                }
                gu8Join_bz=2;//入网成功
				//DeviceState = DEVICE_STATE_JOINED;//SUQI
				//TimerStop( &TxDelayedTimer );  //SUQI
            }
            else
            {
              
              printf("join failed\r\n");
              scgw_uart_send((char*)JoinERR_str,sizeof(JoinERR_str));
#if 0
              MlmeReq_t mlmeReq;
                
                scgw_uart_send((char*)JoinERR_str,sizeof(JoinERR_str));
                // Join was not successful. Try to join again
                mlmeReq.Type = MLME_JOIN;
                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;
                mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;

                if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
                {
                    DeviceState = DEVICE_STATE_SLEEP;
                }
                else
                {
                    DeviceState = DEVICE_STATE_CYCLE;
                }
#endif
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
            }
            break;
        }
        
        case MLME_DEVICE_TIME:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK ){
                // Switch to the next state immediately
                DeviceState = DEVICE_STATE_BEACON_ACQUISITION;
                NextTx = true;
            } else {
                //No device time Ans
                DeviceState = DEVICE_STATE_SLEEP;
            }
            
            break;
        }
        case MLME_BEACON_ACQUISITION:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK ) {
                //beacon received
                DeviceState = DEVICE_STATE_REQ_PINGSLOT_ACK;
                g_beacon_retry_times = 0;
            } else {
                //beacon lost
                if(g_beacon_retry_times < MAX_BEACON_RETRY_TIMES) {
                    g_beacon_retry_times ++;
                    DeviceState = DEVICE_STATE_REQ_DEVICE_TIME;
                } else {
                    g_beacon_retry_times = 0;
                    DeviceState = DEVICE_STATE_SLEEP;
                }
            }
            break;
        }
        case MLME_PING_SLOT_INFO:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                mibReq.Type = MIB_DEVICE_CLASS;
                mibReq.Param.Class = CLASS_B;
                LoRaMacMibSetRequestConfirm( &mibReq );
                
                mibReq.Type = MIB_PING_SLOT_DATARATE;
                mibReq.Param.PingSlotDatarate = g_lwan_dev_config_p->classb_param.pslot_dr;
                LoRaMacMibSetRequestConfirm( &mibReq );

                DeviceState = DEVICE_STATE_SEND;
                NextTx = true;
            }
            else
            {
                DeviceState = DEVICE_STATE_REQ_PINGSLOT_ACK;
            }
            break;
        }
        default:
            break;
    }
    NextTx = true; 
}

/*!
 * \brief   MLME-Indication event function
 *
 * \param   [IN] mlmeIndication - Pointer to the indication structure.
 */
static void MlmeIndication( MlmeIndication_t *mlmeIndication )
{
   MibRequestConfirm_t mibReq;
    switch( mlmeIndication->MlmeIndication )
    {
        case MLME_SCHEDULE_UPLINK:
        {// The MAC signals that we shall provide an uplink as soon as possible
         // printf("MlmeIndication\r\n");
            OnTxNextPacketTimerEvent( );
            
            break;
        }
         case MLME_BEACON_LOST:
        {
            mibReq.Type = MIB_DEVICE_CLASS;
            mibReq.Param.Class = CLASS_A;
            LoRaMacMibSetRequestConfirm( &mibReq );

            // Switch to class A again
            DeviceState = DEVICE_STATE_REQ_DEVICE_TIME;
            break;
        }
        case MLME_BEACON:
        {
            if( mlmeIndication->Status == LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED )
            {
                if(mlmeIndication->BeaconInfo.GwSpecific.InfoDesc==3){ //NetID+GatewayID
                    uint8_t *info = mlmeIndication->BeaconInfo.GwSpecific.Info;
                    if((gGatewayID[0]|gGatewayID[1]|gGatewayID[2]) 
                    && (memcmp(&info[3],gGatewayID,3)!=0)){//GatewayID not 0 and changed
                        //send an uplink in [0:120] seconds
                        TimerStop(&TxNextPacketTimer);
                        TimerSetValue(&TxNextPacketTimer,randr(0,120000));
                        TimerStart(&TxNextPacketTimer);                       
                    }
                    memcpy(gGatewayID,&info[3],3);
                }
            }
            break;
        }
        default:
            break;
    }
}

static void lwan_dev_params_update( void )
{
    MibRequestConfirm_t mibReq;
    uint16_t channelsMaskTemp[6];
    channelsMaskTemp[0] = 0x1000;
    channelsMaskTemp[1] = 0x0000;
    channelsMaskTemp[2] = 0x0000;
    channelsMaskTemp[3] = 0x0000;
    channelsMaskTemp[4] = 0x0000;
    channelsMaskTemp[5] = 0x0000;

    mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
    mibReq.Param.ChannelsDefaultMask = channelsMaskTemp;
    LoRaMacMibSetRequestConfirm(&mibReq);
    mibReq.Type = MIB_CHANNELS_MASK;
    mibReq.Param.ChannelsMask = channelsMaskTemp;
    LoRaMacMibSetRequestConfirm(&mibReq);
    if(WORKMODE==CLASS_A)
    {
      
    }else if(WORKMODE==CLASS_C)
    {
      mibReq.Type = MIB_DEVICE_CLASS;
      mibReq.Param.Class = CLASS_C;
      LoRaMacMibSetRequestConfirm( &mibReq );
    }
    else if(WORKMODE==CLASS_B)
    {
    }
}


void scgw_uart_send(char * string, uint16_t length)
{
  while(length --)
  {
    scgw_uart_write_char(*string++);
  }
}

/**
 * Main application entry point.
 */

//#define TIMETEST
    RtcCalendar_t test;
  TimerEvent_t TESTTimer2;

 void TESTEvent2( void )
{
    TimerStop( &TESTTimer2 );
    GPIO_ToggleBits(GPIOE, GPIO_Pin_6);
    TimerStart( &TESTTimer2 );
    GPIO_ToggleBits(GPIOE, GPIO_Pin_6);
    TimerTime_t Timeget= RtcGetTimerValue();
    printf("time2 :%lu(s),%d(ms)\r\n",Timeget.sec,Timeget.msec);


}


   TimerEvent_t TESTTimer3;
 
  void TESTEvent3( void )
 {
   TimerStop( &TESTTimer3 );
    TimerStart( &TESTTimer3 );
 //  printf("time* :%lu\r\n",RtcGetTimerValue());
     TimerTime_t Timeget= RtcGetTimerValue();
  printf("time3 :%lu(s),%d(ms)\r\n",Timeget.sec,Timeget.msec);
  // DeviceState = DEVICE_STATE_SLEEP;	//IO口唤醒 at命令操作失败情况下继续休眠
 
 }

#ifdef TIMETEST
TimerEvent_t TESTTimer;

static void TESTEvent( void )
{
static int i=0;
TimerInit( &TESTTimer2, TESTEvent2 );
TimerStop( &TESTTimer );
//GPIO_ToggleBits(GPIOC, GPIO_Pin_1);
printf("test1\r\n");
TimerStart( &TESTTimer );


if(i++>10)
{
  i=0;
  TimerInit( &TESTTimer2, TESTEvent2 );

  TimerSetValue( &TESTTimer2, 1000 );// TxDutyCycleTime
  TimerStart( &TESTTimer2 );
  printf("ok\r\n");
}
  
}





#endif


uint16_t subsec=0;
uint16_t subsecRead;
uint32_t  second1=0,second2=0;
uint16_t subsec1=0,subsec2=0;
RtcCalendar_t Cal_test,Cal_test2;

 //TimerSysTime_t  TimeCurrent,TimeSet;
rtc_time_t tempBeijing1;
volatile unsigned int  i1=0;

u8  testa=0,testb=0;
u8  testc=0,testd=0;


u8 Key_test[16]={0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xa1,0xa2,0xa3,0xa4,0xa5,0xa9,0xbb};
u8 Data1_test[21]={0xc1,0xc2,0xd3,0xe4,0xf5,0x66,0x97,0xb8,0x99,0xa1,0xab,0xad,0xa4,0xaf,0xae,0xbb,0x12,0x45,0x78,0x98,0x77};
u8 Data2_test[21]={0};
u8 Dataget_test[21]={0};

void LoRaMacJoinComputeMic( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t *mic );
//加密
void LoRaMacJoinDecrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint8_t *decBuffer );
//解密
void  aes_test(void )
{

  	printf("原始数据：");
	for(int i=0;i<21;i++)
	printf("%x ",Data1_test[i]);
	LoRaMacJoinComputeMic(Data1_test,sizeof(Data1_test),Key_test,(uint32_t *)Data2_test);
	printf("加密后数据：");
	for(int i=0;i<21;i++)
	printf("%x ",Data2_test[i]);
	printf("\r\n");
	LoRaMacJoinDecrypt(Data2_test,sizeof(Data2_test),Key_test,Dataget_test);
	printf("解密后数据：");
	for(int i=0;i<21;i++)
	printf("%x ",Dataget_test[i]);
	printf("\r\n");
}


int main( void )
{

    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;
    MibRequestConfirm_t mibReq;

    BoardInitMcu( );                                                                                                                                                                               
                                                                                                                    
    BoardInitPeriph( );                                                                                                                          

    DeviceState = DEVICE_STATE_INIT;
    // RtcWriteTick(946656000,(2047-500));  //2000 1 1 0 0 0  OK

     RtcWriteTick(1577808000,(2047-500));  //2020 1 1 0 0 0  err

    //  RtcWriteTick(1262275200,(2047-500));  //2010 1 1 0 0 0  err
    //RtcWriteTick(946699932,(2047-500));  //2000 1 1 12 12 12  ok
    //RtcWriteTick(978278400,(2047-500));  //2001 1 1 0 0 0  ok
  //  RtcWriteTick(978235941,(2047-500));  
    
   /* while(1)
    {
      test = RtcGetCalendar();//获取日历
        printf("read:%d %d %d %d %d %d %ld\r\n",test.CalendarDate.RTC_Year,test.CalendarDate.RTC_Month,test.CalendarDate.RTC_Date,\
    test.CalendarTime.RTC_Hours,test.CalendarTime.RTC_Minutes,test.CalendarTime.RTC_Seconds,(uint32_t)((2048-test.CalendarSubSeconds&0x7ff)*1000/2048));
    delay_ms(1000);
    }
*/
/*
    FLASH_SetProgrammingTime(FLASH_ProgramTime_Standard);
    FLASH_Unlock(FLASH_MemType_Program);//FLASH_MemType_Program  FLASH_MemType_Data
    


    FLASH_EraseByte(FLASH_PROGRAM_START_PHYSICAL_ADDRESS+60*1024);
    FLASH_ProgramByte(FLASH_PROGRAM_START_PHYSICAL_ADDRESS+60*1024, 0x56);
    FLASH_ProgramByte(FLASH_PROGRAM_START_PHYSICAL_ADDRESS+60*1024+1, 0x78);
  //  FLASH_ProgramByte(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS+0x100, 0x12);
  //  FLASH_ProgramByte(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS +0x100+1, 0x3a);
    //  while(!FLASH_IAPSR_EOP);//等待写操作完成,最好加入超时判断

 //  asm("rim");//打开中断

    FLASH_Lock(FLASH_MemType_Program);
    printf("POWERON ClassB  start\r\n");
   // testa=(bool)(*(uint8_t *)(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS+0x100));
  //  testb=(bool)(*(uint8_t *)(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS+0x100+1));
     testa=FLASH_ReadByte(FLASH_PROGRAM_START_PHYSICAL_ADDRESS+60*1024);
      testb=FLASH_ReadByte(FLASH_PROGRAM_START_PHYSICAL_ADDRESS+60*1024+1);
     printf("POWERON ClassB  start\r\n");
     

        */
/*
    for(i1=0;i1<512;i1++)
    {
      FLASH_SetProgrammingTime(FLASH_ProgramTime_Standard);
      FLASH_Unlock(FLASH_MemType_Data);
      //FLASH_ProgramOptionByte(0x4800,0x11); //ROP on
      //FLASH_ProgramOptionByte(0x480B,0x55); //bootloader enable
      //FLASH_ProgramOptionByte(0x480C,0xAA); //bootloader enable
      
      FLASH_ProgramByte(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS+512+i1,i1);
      //FLASH_ProgramByte(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS +512 +2*i1+1, (uint8_t)2*i1+1);
      FLASH_Lock(FLASH_MemType_Data);
      //delay_ms(1000);
     // testc = (uint8_t)(*(uint8_t *)FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS+512+2*i1);
      //testd = (uint8_t)(*(uint8_t *)(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS +512 +2*i1+1));
     
      
    }*/
   /* for(i1=0;i1<512;i1++)
    {
    testd = (uint8_t)(*(uint8_t *)(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS +512 +i1));
    delay_ms(500);
     printf("%x>>>>>>>>>>>\r\n",testd);
    }
    */
    /*
    RtcWriteTick(1579098374,(2047-500));

    test = RtcGetCalendar();//获取日历
    printf("read:%d %d %d %d %d %d %ld\r\n",test.CalendarDate.RTC_Year,test.CalendarDate.RTC_Month,test.CalendarDate.RTC_Date,\
    test.CalendarTime.RTC_Hours,test.CalendarTime.RTC_Minutes,test.CalendarTime.RTC_Seconds,(uint32_t)((2048-test.CalendarSubSeconds&0x7ff)*1000/2048));
    */      
    // aes_test();
    GPIO_ResetBits(GPIOE, GPIO_Pin_6);
    // while(1);

  /*  while(1)
    {
       
      delay_ms(3000);
     // printf("+++\r\n");
    }*/
   // WORKMODE=   CLASS_B ;
   /* if(WORKMODE==CLASS_A)
    {
       printf("POWERON  ClassA  start\r\n");
    }else if(WORKMODE==CLASS_C)
    {
       printf("POWERON  ClassC  start\r\n");
    }
    else if(WORKMODE==CLASS_B)
    {
      printf("POWERON ClassB  start\r\n");
    }*/
    
   //for(int i=0;i<1000;i++)
   // while(1)
    {
      ///test = RtcGetCalendar();//获取日历
     // printf("read: %02dh  %02dm %02ds %03ldms\r\n",test.CalendarTime.RTC_Hours,test.CalendarTime.RTC_Minutes,test.CalendarTime.RTC_Seconds,(uint32_t)((2048-test.CalendarSubSeconds&0x7ff)*1000/2048));
      //delay_ms(1000);
    }
    
    
   // printf("%ld\r\n",502700000);
  //  DeviceState = DEVICE_STATE_CYCLE;
    
 //  gu8Join_bz=1;
    gu8Join_bz=0;//璋冭瘯classb 鑷姩鍏ョ綉 鍚庢湡鍘绘帀 suqi
    //delay_ms(3000);
    TimerInit( &TESTTimer2, TESTEvent2 );
    TimerSetValue( &TESTTimer2, 100 );
    TimerStart( &TESTTimer2 );
//    TimerInit( &TESTTimer3, TESTEvent3 );
//    TimerSetValue( &TESTTimer3, 3000 );
//    TimerStart( &TESTTimer3 );
    GPIO_SetBits(GPIOE, GPIO_Pin_6);
    TimerTime_t Timeget= RtcGetTimerValue();
    printf("time2 :%lu(s),%d(ms)\r\n",Timeget.sec,Timeget.msec);
 TimerLowPowerHandler( );
    
    while(1);

//TimerInit( &TESTTimer3, TESTEvent3 );
   // TimerSetValue( &TESTTimer3, 1000 );
    //TimerStart( &TESTTimer3 );
    for(int i=0;i<10;i++)
    {
      delay_ms(1200);
    }
    //TimerLowPowerHandler( );
     //  while(1);
    
   //RtcWriteTick(86400,(2047-1000));  //16 10000
  //  RtcWriteTick(86400,(2047-0));
    //delay_ms(3000);

   // RtcWriteTick(1579098374,(2047-1000));//9984  偏差值  什么都不设置 
    
   // RtcWriteTick(12345,(2047-1000)); //9920
  //  RtcWriteTick(946697145,(2047-1000));//9936 //86400
  //  RtcWriteTick(86400,(2047-1000));
    printf("rtc\r\n");
    TimerSetValue( &TESTTimer2, 10000 );
    
    TimerStart( &TESTTimer2 );
    GPIO_SetBits(GPIOE, GPIO_Pin_6);
    printf("time2:%lu\r\n",RtcGetTimerValue());
    TimerLowPowerHandler( );
    
    
    while(1);
    /*
#ifdef TIMETEST
    TimerInit( &TESTTimer, TESTEvent );

    TimerSetValue( &TESTTimer, 200 );// TxDutyCycleTime
    TimerStart( &TESTTimer );

    
    TimerInit( &TESTTimer2, TESTEvent2 );

   // TimerSetValue( &TESTTimer2, 1000 );// TxDutyCycleTime
   // TimerStart( &TESTTimer2 );
    while(1);
#endif
   
    
   while(0)
   {
     //subsec1=RTC_GetSubSecond();
    // printf("b3:%d\r\n",subsec1);
	 subsec = RTC_GetSubSecond();
	 printf("b1:%d\r\n",subsec);
         //RTC_SetSubSecond(30);
         i2++;
      delay_ms(10+(i2)%1000);
	 subsecRead = RTC_GetSubSecond();//2048
	 printf("b4:%d\r\n",subsecRead);//
	 printf("b3:%d\r\n",subsecRead&0x7ff);//同b4
         printf("b2:%d\r\n",subsecRead&0x7ff);//同b4
     delay_ms(1000*(1));//58
    // printf("a:%d\r\n",RTC_GetSubSecond());
     
   }
                test = RtcGetCalendar();//获取日历
             printf("read:%d %d %d %d %d %d %ld\r\n",test.CalendarDate.RTC_Year,test.CalendarDate.RTC_Month,test.CalendarDate.RTC_Date,\
             test.CalendarTime.RTC_Hours,test.CalendarTime.RTC_Minutes,test.CalendarTime.RTC_Seconds,(uint32_t)((2048-test.CalendarSubSeconds&0x7ff)*1000/2048));
            

   while(0)
   {
       
    //  printf("b2:%d\r\n",RTC_GetSubSecond()&0x7ff);//RTC_GetSubSecond()  不变
      for(uint32_t i=0;i<2048;i++)
      {
          i2++;
          // delay_ms(100);
          printf("set ms %ld \r\n",(uint32_t)((i)*1000/2048));


          RtcWriteTick(1579098374,(2047-i));

          test = RtcGetCalendar();//获取日历
          GPIO_SetBits(GPIOC, GPIO_Pin_1);
          GPIO_SetBits(GPIOC, GPIO_Pin_0);
          printf("read:%d %d %d %d %d %d %ld\r\n",test.CalendarDate.RTC_Year,test.CalendarDate.RTC_Month,test.CalendarDate.RTC_Date,\
          test.CalendarTime.RTC_Hours,test.CalendarTime.RTC_Minutes,test.CalendarTime.RTC_Seconds,(uint32_t)((2048-test.CalendarSubSeconds&0x7ff)*1000/2048));
      }
      while(1)
      {
             delay_ms(500);
             test = RtcGetCalendar();//获取日历
             printf("read:%d %d %d %d %d %d %ld\r\n",test.CalendarDate.RTC_Year,test.CalendarDate.RTC_Month,test.CalendarDate.RTC_Date,\
             test.CalendarTime.RTC_Hours,test.CalendarTime.RTC_Minutes,test.CalendarTime.RTC_Seconds,(uint32_t)((2048-test.CalendarSubSeconds&0x7ff)*1000/2048));
        
      }
      
      
         //1,转换成功
        //2，写入 读出成功
   }
   
  // delay_ms(2000);
   
//    GPIO_ToggleBits(GPIOC, GPIO_Pin_0);
//    delay_ms(10); GPIO_ToggleBits(GPIOC, GPIO_Pin_0);
//    delay_ms(20); GPIO_ToggleBits(GPIOC, GPIO_Pin_0);
//    delay_ms(30); GPIO_ToggleBits(GPIOC, GPIO_Pin_0);
//    delay_ms(40); GPIO_ToggleBits(GPIOC, GPIO_Pin_0);
//    delay_ms(50); GPIO_ToggleBits(GPIOC, GPIO_Pin_0);
*/
   
   /*while(1)//倒计时1分钟   后期需要加入
   {
     
            if(gu8RecFlag==1)
            {
               gu8RecFlag=0;
             // scgw_uart_send("AT+CCH=1,471900000,9\n",21);
              ATCmdFun();
                    
            }
            if(gu8Join_bz==1)//命令设置完成 return 进行初始化
            {
              DeviceState = DEVICE_STATE_INIT;
              break;
            }
   }*/
    while( 1 )
    {
        switch( DeviceState )
        {
            case DEVICE_STATE_INIT:
            {
                mibReq.Param.Class = WORKMODE; //重新配置
                LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
                LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;//入网确认
                LoRaMacPrimitives.MacMlmeIndication = MlmeIndication;
                LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
				LoRaMacCallbacks.GetTemperatureLevel= NULL;
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, ACTIVE_REGION );

                TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );

                mibReq.Type = MIB_ADR;
                mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_PUBLIC_NETWORK;
                mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
                LoRaMacMibSetRequestConfirm( &mibReq );
                
                lwan_dev_params_update();

                
                

               // RxBeaconSetup( 0,true );
                if(gu8Join_bz==1)//AT鍛戒护涓厤缃繃workmode 寮�濮嬪叆缃?      suqi
                      DeviceState = DEVICE_STATE_JOIN; //开始入网
                else
                      DeviceState = DEVICE_STATE_CYCLE;//还未入网
                break;
                

            }
            case DEVICE_STATE_JOIN:
            {
#if( OVER_THE_AIR_ACTIVATION != 0 )
                MlmeReq_t mlmeReq;

                // Initialize LoRaMac device unique ID
                //BoardGetUniqueId( DevEui );

                mlmeReq.Type = MLME_JOIN;

                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;
                
                
                if(WORKMODE==CLASS_A)
                {
                      mlmeReq.Req.Join.NbTrials = 8;
                }
                else if(WORKMODE==CLASS_C)
                {
                }
                else if(WORKMODE==CLASS_B)
                {
                      mlmeReq.Req.Join.NbTrials = 8;
                }
    
                mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;

                if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
                {
                    DeviceState = DEVICE_STATE_SLEEP;
                }
                else
                {
                    DeviceState = DEVICE_STATE_CYCLE;
                }
#else
                mibReq.Type = MIB_NET_ID;
                mibReq.Param.NetID = LORAWAN_NETWORK_ID;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_DEV_ADDR;
                mibReq.Param.DevAddr = DevAddr;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NWK_SKEY;
                mibReq.Param.NwkSKey = NwkSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_APP_SKEY;
                mibReq.Param.AppSKey = AppSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NETWORK_JOINED;
                mibReq.Param.IsNetworkJoined = true;
                LoRaMacMibSetRequestConfirm( &mibReq );
                
                DeviceState = DEVICE_STATE_SEND;
                DeviceState =DEVICE_STATE_CYCLE;
#endif
                break;
            }
             case DEVICE_STATE_REQ_DEVICE_TIME: {//获取设置时间 beacon
                MlmeReq_t mlmeReq;
                MibRequestConfirm_t mib_req;

                mib_req.Type = MIB_NETWORK_JOINED;
                LoRaMacMibGetRequestConfirm(&mib_req);
                if (mib_req.Param.IsNetworkJoined == true) {
                    if( NextTx == true ) {
                        mlmeReq.Type = MLME_DEVICE_TIME;//发送ready to send MOTE_MAC_DEVICE_TIME_REQ  获取时间
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
                    DeviceState = DEVICE_STATE_SEND_MAC;
                } else {
                    DeviceState = DEVICE_STATE_SLEEP;
                }
                
                break;
            }
            case DEVICE_STATE_BEACON_ACQUISITION: {//发ping包 请求多少秒一个时隙
                MlmeReq_t mlmeReq;

                if( NextTx == true ) {
                    if(g_lwan_dev_config_p->classb_param.beacon_freq)
                        LoRaMacClassBBeaconFreqReq(g_lwan_dev_config_p->classb_param.beacon_freq);
                    if(g_lwan_dev_config_p->classb_param.pslot_freq)
                        LoRaMacClassBPingSlotChannelReq(g_lwan_dev_config_p->classb_param.pslot_dr, g_lwan_dev_config_p->classb_param.pslot_freq);
                    mlmeReq.Type = MLME_BEACON_ACQUISITION;
                    LoRaMacMlmeRequest( &mlmeReq );
                }
                DeviceState = DEVICE_STATE_SLEEP;
                break;
            }
            case DEVICE_STATE_REQ_PINGSLOT_ACK: {
                MlmeReq_t mlmeReq;

                if( NextTx == true ) {
                    mlmeReq.Type = MLME_PING_SLOT_INFO;
                    mlmeReq.Req.PingSlotInfo.PingSlot.Fields.Periodicity = g_lwan_dev_config_p->classb_param.periodicity;
                    mlmeReq.Req.PingSlotInfo.PingSlot.Fields.RFU = 0;
                    LoRaMacMlmeRequest( &mlmeReq );
                }
                DeviceState = DEVICE_STATE_SEND_MAC;
                break;
            }
            case DEVICE_STATE_SEND:
            {
                if( NextTx == true )
                {
                    //PrepareTxFrame( AppPort );
                    NextTx = SendFrame( );
                }
                // Schedule next packet transmission
                //TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( 0, APP_TX_DUTYCYCLE_RND );//不发送数据的话 5.7s唤醒一次
                DeviceState = DEVICE_STATE_CYCLE;
                break;
            }
            case DEVICE_STATE_SEND_MAC: {
                if (NextTx == true) {
                    AppDataSize = 0;
                    NextTx = SendFrame();
                }
                DeviceState = DEVICE_STATE_SLEEP;
                break;
            }
            case DEVICE_STATE_CYCLE:
            {
                DeviceState = DEVICE_STATE_SLEEP;
                // Schedule next packet transmission
                TimerSetValue( &TxNextPacketTimer, 30000 );// TxDutyCycleTime
                TimerStart( &TxNextPacketTimer );
                break;
            }
            case DEVICE_STATE_SLEEP:
            {
                // Wake up through events
                if(WORKMODE==CLASS_A)
                {
                  TimerLowPowerHandler( );
                }
                else if(WORKMODE==CLASS_C)
                {
                  
                }
                else if(WORKMODE==CLASS_B)
                {
                  TimerLowPowerHandler( );
                }
                // Process Radio IRQ
                Radio.IrqProcess( );//RadioIrqProcess
                break;
            }
            case DEVICE_STATE_ATCMD://进入AT指令设置模式  DEVICE_STATE_CYCLE会超时设定时间（0-60s之间）退出
           // TimerSetValue( &TxNextPacketTimer, 10000 );// TxDutyCycleTime
            // printf("..\r\n");
            if(gu8RecFlag==1)
            {
                gu8RecFlag=0;
             // scgw_uart_send("AT+CCH=1,471900000,9\n",21);
                ATCmdFun();
            }
            break;
            default:
            {
                DeviceState = DEVICE_STATE_INIT;
                break;
            }
        }
    }
}
