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
//CLASS A &C合并  注意:放到自制件中需要修改 RFSWITCH 具体参考网关
//CONFIG_UART_DEBUG=0 //默认 
//CONFIG_UART_CMD=1  //默认
//数据发送 接收buff   AppData[LORAWAN_APP_DATA_MAX_SIZE];
//tx_cmd_str[LORAWAN_APP_DATA_MAX_SIZE]
//sleep-low  rtcirq-high
//正式交付模式 TEST需要屏蔽 同时需要修改rfswitch
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
DeviceClass_t WORKMODE ;  //CLASS_C
#ifndef ACTIVE_REGION

#warning "No active region defined, LORAMAC_REGION_CN470 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_CN470

#endif
static uint8_t g_beacon_retry_times = 0;
static LWanDevConfig_t *g_lwan_dev_config_p = NULL;
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
    channelsMaskTemp[0] = 0x00FF;
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

/*

bool LoRaMacClassBBeaconFreqReq( uint32_t frequency )
{
#ifdef LORAMAC_CLASSB_ENABLED
    if( frequency != 0 )
    {
        if( Radio.CheckRfFrequency( frequency ) == true )
        {
            BeaconCtx.Ctrl.CustomFreq = 1;
            BeaconCtx.Frequency = frequency;
            return true;
        }
    }
    else
    {
        BeaconCtx.Ctrl.CustomFreq = 0;
        return true;
    }
    return false;
#else
    return false;
#endif // LORAMAC_CLASSB_ENABLED
}*/
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

int main( void )
{

    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;
    MibRequestConfirm_t mibReq;

    BoardInitMcu( );
    BoardInitPeriph( );

    DeviceState = DEVICE_STATE_INIT;

    WORKMODE=   CLASS_A ;
    if(WORKMODE==CLASS_A)
    {
       printf("POWERON  ClassA  start\r\n");
    }else if(WORKMODE==CLASS_C)
    {
       printf("POWERON  ClassC  start\r\n");
    }
    else if(WORKMODE==CLASS_B)
    {
      printf("POWERON ClassB  start\r\n");

	  
    }
    
  //  DeviceState = DEVICE_STATE_CYCLE;
    
   


   
    
    
   
    
#if 0
while(1)//上电进入配置模式
  {
    
    
          GPIO_ToggleBits(GPIOC, GPIO_Pin_4);//LED
          //delay_ms(2000);
          if(gu8RecFlag==1)
          {
            gu8RecFlag=0;
           // scgw_uart_send("AT+CCH=1,471900000,9\n",21);
             ATCmdFun();
                  
          }
    
  }
#endif
    while( 1 )
    {
        switch( DeviceState )
        {
            case DEVICE_STATE_INIT:
            {
              
                mibReq.Param.Class = WORKMODE; //重新配置
                LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
                LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
                LoRaMacPrimitives.MacMlmeIndication = MlmeIndication;
                LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, ACTIVE_REGION );

                TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );

                mibReq.Type = MIB_ADR;
                mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_PUBLIC_NETWORK;
                mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
                LoRaMacMibSetRequestConfirm( &mibReq );
                
                lwan_dev_params_update();
                if(gu8Join_bz==1)//AT命令中配置过workmode 开始入网  suqi
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
