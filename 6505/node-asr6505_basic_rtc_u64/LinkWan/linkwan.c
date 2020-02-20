/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */
#include "commissioning.h"
#include "utilities.h"
#include "board.h"
#include "LoRaMacCrypto.h"
#include "LoRaMac.h"
#include "radio.h"
#include "lwan_config.h"  
#include "debug.h"
#include "linkwan.h"
#include "LoRaMACClassD.h"
 
#define LINKWAN_TEST_DATA "linkwan test"
#define LINKWAN_TEST_PORT 10

static uint8_t tx_buf[LORAWAN_APP_DATA_BUFF_SIZE];
static lora_AppData_t tx_data = {tx_buf, 1, 10};

static LoRaMacPrimitives_t LoRaMacPrimitives;

static LoRaMacCallback_t LoRaMacCallbacks;

#ifdef CONFIG_LINKWAN_D2D
static bool g_data_send_local = false;

static rx_cb_t  d2d_rx_callback = NULL;

static MulticastParams_t ClassDMCChannels[] = {
{ LORAWAN_MC_ADDR_0, {0},{0},0, NULL  },
//{ LORAWAN_MC_ADDR_1, {0},{0},0, NULL  },
//{ LORAWAN_MC_ADDR_2, {0},{0},0, NULL  },
//{ LORAWAN_MC_ADDR_3, {0},{0},0, NULL  },
};

static ClassDKey_t ClassDMCKeys[] = {
    {LORAWAN_MC_KEY_0},
    //{LORAWAN_MC_KEY_1},
    //{LORAWAN_MC_KEY_2},
    //{LORAWAN_MC_KEY_3}
};
#endif

static volatile bool next_tx = true;
static volatile bool rejoin_flag = true;

static uint8_t g_data_send_nbtrials = 0;
static int8_t g_data_send_msg_type = -1;
static uint8_t g_freqband_num = 0;   

TimerEvent_t TxNextPacketTimer;
volatile DeviceState_t g_lwan_device_state = DEVICE_STATE_INIT;
static DeviceStatus_t g_lwan_device_status = DEVICE_STATUS_IDLE;

static LWanDevConfig_t *g_lwan_dev_config_p = NULL;
static LWanMacConfig_t *g_lwan_mac_config_p = NULL;
static LWanDevKeys_t *g_lwan_dev_keys_p = NULL;

static void start_dutycycle_timer(void);
static bool send_frame(void)
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;
    uint8_t send_msg_type;
	uint8_t state=0;
    if (LoRaMacQueryTxPossible(tx_data.BuffSize, &txInfo) != LORAMAC_STATUS_OK) {
	  //DBG_LINKWAN("send test1\n\r");  
        return true;
    }

    if(g_lwan_mac_config_p->modes.linkcheck_mode == 2) {
        MlmeReq_t mlmeReq;
        mlmeReq.Type = MLME_LINK_CHECK;
        LoRaMacMlmeRequest(&mlmeReq);
		//DBG_LINKWAN("send test2\n\r");  
    }
    
    send_msg_type = g_data_send_msg_type>=0?g_data_send_msg_type:g_lwan_mac_config_p->modes.confirmed_msg;
    if (send_msg_type == LORAWAN_UNCONFIRMED_MSG) {
        MibRequestConfirm_t mibReq;
        mibReq.Type = MIB_CHANNELS_NB_REP;
        mibReq.Param.ChannelNbRep = g_data_send_nbtrials?g_data_send_nbtrials:
                                                    g_lwan_mac_config_p->nbtrials.unconf + 1;
        LoRaMacMibSetRequestConfirm(&mibReq);
    
        mcpsReq.Type = MCPS_UNCONFIRMED;
		#ifdef CONFIG_LINKWAN_D2D
        if(g_data_send_local){
            g_data_send_local = false;
            mcpsReq.Req.Unconfirmed.fPort = LORAWAN_LOCAL_APP_PORT;
        } 
		else
		#endif
        {
            mcpsReq.Req.Unconfirmed.fPort = g_lwan_mac_config_p->port;
        }
        mcpsReq.Req.Unconfirmed.fBuffer = tx_data.Buff;
        mcpsReq.Req.Unconfirmed.fBufferSize = tx_data.BuffSize;
        mcpsReq.Req.Unconfirmed.Datarate = g_lwan_mac_config_p->datarate;
		//DBG_LINKWAN("send test3\n\r");  
    } else {
        mcpsReq.Type = MCPS_CONFIRMED;
        mcpsReq.Req.Confirmed.fPort = g_lwan_mac_config_p->port;
        mcpsReq.Req.Confirmed.fBuffer = tx_data.Buff;
        mcpsReq.Req.Confirmed.fBufferSize = tx_data.BuffSize;
        mcpsReq.Req.Confirmed.NbTrials = g_data_send_nbtrials?g_data_send_nbtrials:
                                                    g_lwan_mac_config_p->nbtrials.conf+1;
        mcpsReq.Req.Confirmed.Datarate = g_lwan_mac_config_p->datarate;
		//DBG_LINKWAN("send test4\n\r");  
    }

    g_data_send_nbtrials = 0;
    g_data_send_msg_type = -1;
    
	state=LoRaMacMcpsRequest(&mcpsReq);
    if (state == LORAMAC_STATUS_OK) {
	  //DBG_LINKWAN("send test5  %d\n\r",state);  
        return false;
    }

    return true;
}

static void prepare_tx_frame(void)
{
    if (g_lwan_mac_config_p->modes.report_mode == TX_ON_TIMER) {
        tx_data.BuffSize = sprintf( (char *) tx_data.Buff, LINKWAN_TEST_DATA);
        tx_data.Port = LINKWAN_TEST_PORT;
    }
}

static uint8_t get_freqband_num(void)
{
    uint8_t i = 0;
    uint8_t num = 0;
    uint16_t mask = g_lwan_dev_config_p->freqband_mask;

    for (i = 0; i < 16; i++) {
        if ((mask & (1 << i)) && i != 1) {
            num++;
        }
    }
    return num;
}
static uint8_t get_next_freqband(void)
{
    uint8_t i = 0; 
    uint8_t freqband[16];
    uint8_t freqnum = 0;
    uint16_t mask = g_lwan_dev_config_p->freqband_mask;

    freqband[freqnum++] = 1; //1A2
    for (i = 0; i < 16; i++) {
        if ((mask & (1 << i)) && i != 1) {
            freqband[freqnum++] = i;
        }
    }
    
    return freqband[randr(0,freqnum-1)];
}

static void reset_join_state(void)
{
    MibRequestConfirm_t mib_req;
    mib_req.Type = MIB_DEVICE_CLASS;
    if (LoRaMacMibGetRequestConfirm(&mib_req) == LORAMAC_STATUS_OK) {
        if (mib_req.Param.Class != CLASS_A) {
    
            mib_req.Type = MIB_DEVICE_CLASS;
            mib_req.Param.Class = CLASS_A;
            LoRaMacMibSetRequestConfirm(&mib_req);
        }
    }     
    g_lwan_device_state = DEVICE_STATE_JOIN;                
}


static void on_tx_next_packet_timer_event(void)
{
    MibRequestConfirm_t mib_req;
    LoRaMacStatus_t status;

    TimerStop(&TxNextPacketTimer);

    mib_req.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm(&mib_req);

    if (status == LORAMAC_STATUS_OK) {
        if (mib_req.Param.IsNetworkJoined == true) {
            g_lwan_device_state = DEVICE_STATE_SEND;//此处默認每隔30秒发送
			DBG_LINKWAN("Send 3...\r");
        } else {
            rejoin_flag = true;
            g_lwan_device_state = DEVICE_STATE_JOIN;
			DBG_LINKWAN("Join 3...\r");
        }
    }
}

static void mcps_confirm(McpsConfirm_t *mcpsConfirm)
{
    if (mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
        PRINTF_AT("\r\nOK+SENT:%02X\r\n", (uint16_t)mcpsConfirm->NbRetries);      
    } else {
        PRINTF_AT("\r\nERR+SENT:%02X  mcps->status=%d\r\n", (uint16_t)mcpsConfirm->NbRetries,mcpsConfirm->Status);          
    }
    next_tx = true;
}


static void mcps_indication(McpsIndication_t *mcpsIndication)
{
    if ( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK ) {
        return;
    }
    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot
    DBG_PRINTF( "receive data: %d  rssi = %d, snr = %u, datarate = %u\r\n",mcpsIndication->BufferSize, mcpsIndication->Rssi, (uint16_t)mcpsIndication->Snr,
                 (uint16_t)mcpsIndication->RxDatarate);
    lwan_dev_status_set(DEVICE_STATUS_SEND_PASS_WITH_DL);
    if (mcpsIndication->RxData == true) {
        switch ( mcpsIndication->Port ) {
            case 224:
                break;
			#ifdef CONFIG_LINKWAN_D2D
            case LORAWAN_LOCAL_APP_PORT:
                if(d2d_rx_callback){
                    d2d_rx_callback(mcpsIndication->Buffer, mcpsIndication->BufferSize);
                    break;
                }
			#endif                
            default: {
                int i;
                for (i = 0; i < mcpsIndication->BufferSize; i++) {
                    PRINTF_RAW("0x%02x ", mcpsIndication->Buffer[i] );
                }
                break;
            }
        }
    }
  
    if(mcpsIndication->UplinkNeeded) {
        g_lwan_device_state = DEVICE_STATE_SEND_MAC;
    }
}

static uint32_t generate_rejoin_delay(void)
{
    uint32_t rejoin_delay = 0;

    while (rejoin_delay < g_lwan_dev_config_p->join_settings.join_interval*1000) {
        rejoin_delay += (rand1() % 250);
    }

    return rejoin_delay;
}

static void mlme_confirm( MlmeConfirm_t *mlmeConfirm )
{
    uint32_t rejoin_delay = 8*1000;

    switch ( mlmeConfirm->MlmeRequest ) {
        case MLME_JOIN: {
            if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
                // Status is OK, node has joined the network
                g_lwan_device_state = DEVICE_STATE_JOINED;
                lwan_dev_status_set(DEVICE_STATUS_JOIN_PASS);
                
                PRINTF_AT("+CJOIN:OK\r\n");               
            } else {
                lwan_dev_status_set(DEVICE_STATUS_JOIN_FAIL);
                                
                // Join was not successful. Try to join again
                reset_join_state();
                if (g_lwan_dev_config_p->join_settings.join_method != JOIN_METHOD_SCAN) {
                    g_lwan_dev_config_p->join_settings.join_method = 
                        (g_lwan_dev_config_p->join_settings.join_method + 1) % JOIN_METHOD_NUM;
                    rejoin_delay = generate_rejoin_delay();
                    if (g_lwan_dev_config_p->join_settings.join_method == JOIN_METHOD_SCAN) {
                        g_freqband_num = get_freqband_num();
                    }
                }

                if (g_lwan_dev_config_p->join_settings.join_method == JOIN_METHOD_SCAN) {
                    if (g_freqband_num == 0) {
                        g_lwan_dev_config_p->join_settings.join_method = JOIN_METHOD_DEF;
                        rejoin_delay = (uint32_t) 60 * 60 * 1000;  // 1 hour
                        
                        PRINTF_AT("+CJOIN:FAIL\r\n");                        
                        DBG_LINKWAN("Wait 1 hour for new round of scan\r\n");
                    } else {
                        g_freqband_num--;
                        rejoin_delay = generate_rejoin_delay();
                    }
                }  
                TimerSetValue(&TxNextPacketTimer, rejoin_delay);
                TimerStart(&TxNextPacketTimer);
                rejoin_flag = false;
            }
            break;
        }
        case MLME_LINK_CHECK: {          
            if ( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK ) {
                // Check DemodMargin
                // Check NbGateways
            } else {
                lwan_dev_status_set(DEVICE_STATUS_NETWORK_ABNORMAL);
            }
            break;
        }
		#ifdef CONFIG_LINKWAN_D2D
        case MLME_PREAMBLE_INFO: {
            if ( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK ) {
                MibRequestConfirm_t mib_req;
                mib_req.Type = MIB_PREAMBLE_TIME;
                mib_req.Param.PreambleTime = g_lwan_dev_config_p->classd_param.preambletime;

                LoRaMacMibSetRequestConfirm(&mib_req);
            } else {
                //No device time Ans
                g_lwan_device_state = DEVICE_STATE_SLEEP;
            }
            break;
        }        
		#endif        
        default:
            break;
    }
    next_tx = true;
}

static void mlme_indication( MlmeIndication_t *mlmeIndication )
{
    switch( mlmeIndication->MlmeIndication )
    {
        case MLME_SCHEDULE_UPLINK:
        {// The MAC signals that we shall provide an uplink as soon as possible
            g_lwan_device_state = DEVICE_STATE_SEND_MAC;
            break;
        }
        default:
            break;
    }
}


static void start_dutycycle_timer(void)
{
    MibRequestConfirm_t mib_req;
    LoRaMacStatus_t status;

    TimerStop(&TxNextPacketTimer);
    mib_req.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm(&mib_req);
    if (status == LORAMAC_STATUS_OK) {
        if (mib_req.Param.IsNetworkJoined == true &&
            g_lwan_mac_config_p->modes.report_mode == TX_ON_TIMER && g_lwan_mac_config_p->report_interval != 0) {
            TimerSetValue(&TxNextPacketTimer, g_lwan_mac_config_p->report_interval*1000);
            TimerStart(&TxNextPacketTimer);
            return;
        }
    }
    if (g_lwan_mac_config_p->report_interval == 0 && g_lwan_mac_config_p->modes.report_mode == TX_ON_TIMER) {
        g_lwan_mac_config_p->modes.report_mode = TX_ON_NONE;
    }
}


MulticastParams_t *get_lora_cur_multicast(void)
{
    MibRequestConfirm_t mib_req;
    LoRaMacStatus_t status;

    mib_req.Type = MIB_MULTICAST_CHANNEL;
    status = LoRaMacMibGetRequestConfirm(&mib_req);
    if (status == LORAMAC_STATUS_OK) {
        return mib_req.Param.MulticastList;
    }
    return NULL;
}

static void print_dev_info(void)
{
    int i = 0;
    if(g_lwan_dev_config_p->modes.join_mode == JOIN_MODE_OTAA){
        DBG_LINKWAN("OTAA\r\n" );
        DBG_LINKWAN("DevEui= %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\r\n",
                    (uint16_t)g_lwan_dev_keys_p->ota.deveui[0], (uint16_t)g_lwan_dev_keys_p->ota.deveui[1], \
                    (uint16_t)g_lwan_dev_keys_p->ota.deveui[2], (uint16_t)g_lwan_dev_keys_p->ota.deveui[3], \
                    (uint16_t)g_lwan_dev_keys_p->ota.deveui[4], (uint16_t)g_lwan_dev_keys_p->ota.deveui[5], \
                    (uint16_t)g_lwan_dev_keys_p->ota.deveui[6], (uint16_t)g_lwan_dev_keys_p->ota.deveui[7]);
        DBG_LINKWAN("AppEui= %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\r\n",
                    (uint16_t)g_lwan_dev_keys_p->ota.appeui[0], (uint16_t)g_lwan_dev_keys_p->ota.appeui[1], \
                    (uint16_t)g_lwan_dev_keys_p->ota.appeui[2], (uint16_t)g_lwan_dev_keys_p->ota.appeui[3], \
                    (uint16_t)g_lwan_dev_keys_p->ota.appeui[4], (uint16_t)g_lwan_dev_keys_p->ota.appeui[5], \
                    (uint16_t)g_lwan_dev_keys_p->ota.appeui[6], (uint16_t)g_lwan_dev_keys_p->ota.appeui[7]);
        DBG_LINKWAN("AppKey= %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\r\n",
                    (uint16_t)g_lwan_dev_keys_p->ota.appkey[0], (uint16_t)g_lwan_dev_keys_p->ota.appkey[1], \
                    (uint16_t)g_lwan_dev_keys_p->ota.appkey[2], (uint16_t)g_lwan_dev_keys_p->ota.appkey[3], \
                    (uint16_t)g_lwan_dev_keys_p->ota.appkey[4], (uint16_t)g_lwan_dev_keys_p->ota.appkey[5], \
                    (uint16_t)g_lwan_dev_keys_p->ota.appkey[6], (uint16_t)g_lwan_dev_keys_p->ota.appkey[7], \
                    (uint16_t)g_lwan_dev_keys_p->ota.appkey[8], (uint16_t)g_lwan_dev_keys_p->ota.appkey[9], \
                    (uint16_t)g_lwan_dev_keys_p->ota.appkey[10], (uint16_t)g_lwan_dev_keys_p->ota.appkey[11], \
                    (uint16_t)g_lwan_dev_keys_p->ota.appkey[12], (uint16_t)g_lwan_dev_keys_p->ota.appkey[13], \
                    (uint16_t)g_lwan_dev_keys_p->ota.appkey[14], (uint16_t)g_lwan_dev_keys_p->ota.appkey[15]);
    } else if(g_lwan_dev_config_p->modes.join_mode == JOIN_MODE_ABP){
        DBG_LINKWAN("ABP\r\n");
        DBG_LINKWAN("DevAddr= %08X\r\n", (unsigned int)g_lwan_dev_keys_p->abp.devaddr);
        DBG_LINKWAN("NwkSKey= ");
        for (i = 0; i < LORA_KEY_LENGTH; i++) {
            PRINTF_RAW("%02X", g_lwan_dev_keys_p->abp.nwkskey[i]);
        };
        PRINTF_RAW("\r\n");
        DBG_LINKWAN("AppSKey= ");
        for (i = 0; i < LORA_KEY_LENGTH; i++) {
            PRINTF_RAW("%02X", g_lwan_dev_keys_p->abp.appskey[i]);
        };
        PRINTF_RAW("\r\n");
    }
    DBG_LINKWAN("class type %c\r\n", (char)('A' + g_lwan_dev_config_p->modes.class_mode));
    DBG_LINKWAN("freq mode %s\r\n", g_lwan_dev_config_p->modes.uldl_mode == ULDL_MODE_INTER ? "inter" : "intra");
    DBG_LINKWAN("scan chn mask 0x%04x\r\n", g_lwan_dev_config_p->freqband_mask);
	 DBG_LINKWAN("scan chn mask 0x%04x\r\n", g_lwan_dev_config_p->freqband_mask);
	  
}

void init_lwan_configs( void ) 
{
    LWanDevKeys_t default_keys = LWAN_DEV_KEYS_DEFAULT;
    LWanDevConfig_t default_dev_config = LWAN_DEV_CONFIG_DEFAULT;
    LWanMacConfig_t default_mac_config = LWAN_MAC_CONFIG_DEFAULT;
    g_lwan_dev_keys_p = lwan_dev_keys_init(&default_keys);
    g_lwan_dev_config_p = lwan_dev_config_init(&default_dev_config);//LWanDevConfig_t  classD此处配置
	
    g_lwan_mac_config_p = lwan_mac_config_init(&default_mac_config);
}


void lora_init(LoRaMainCallback_t *callbacks)
{
    (void)callbacks;
    g_lwan_device_state = DEVICE_STATE_INIT;
}


void lora_fsm( void )
{
    while (1) {
        if (Radio.IrqProcess != NULL) {
            Radio.IrqProcess();//调用 RadioIrqProcess
        }
        switch (g_lwan_device_state) {
            case DEVICE_STATE_INIT: { 
			  DBG_LINKWAN("DEVICE_STATE_INIT\n\r"); 

				
                LoRaMacPrimitives.MacMcpsConfirm = mcps_confirm;
                LoRaMacPrimitives.MacMcpsIndication = mcps_indication;
                LoRaMacPrimitives.MacMlmeConfirm = mlme_confirm;
                LoRaMacPrimitives.MacMlmeIndication = mlme_indication;
                LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;

                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN470A);//初始化回调函数  修改此處region 
			 /* LoRaMacPrimitives =&LoRaMacPrimitives2 ;
	         LoRaMacPrimitives->MacMcpsConfirm = &mcps_confirm;
                LoRaMacPrimitives->MacMcpsIndication = &mcps_indication;
                LoRaMacPrimitives->MacMlmeConfirm = &mlme_confirm;
                LoRaMacPrimitives->MacMlmeIndication = &mlme_indication;
                LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;

                LoRaMacInitialization( LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN470A);//初始化回调函数  
		*/
                init_lwan_configs();//配置key
                print_dev_info();
                led_lignt1s();//TEST
                TimerInit( &TxNextPacketTimer, on_tx_next_packet_timer_event );

                lwan_dev_params_update();
				#ifdef CONFIG_LINKWAN_D2D
                lwan_classd_mcchannel_setup();
				#endif                
                if(g_lwan_dev_config_p->modes.join_mode == JOIN_MODE_ABP){
                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_NET_ID;
                    mibReq.Param.NetID = LORAWAN_NETWORK_ID;
                    LoRaMacMibSetRequestConfirm(&mibReq);
                    mibReq.Type = MIB_DEV_ADDR;
                    mibReq.Param.DevAddr = g_lwan_dev_keys_p->abp.devaddr;
                    LoRaMacMibSetRequestConfirm(&mibReq);    
                    mibReq.Type = MIB_NWK_SKEY;
                    mibReq.Param.NwkSKey = g_lwan_dev_keys_p->abp.nwkskey;
                    LoRaMacMibSetRequestConfirm(&mibReq);
                    mibReq.Type = MIB_APP_SKEY;
                    mibReq.Param.AppSKey = g_lwan_dev_keys_p->abp.appskey;
                    LoRaMacMibSetRequestConfirm(&mibReq);                   
                    mibReq.Type = MIB_FREQ_BAND;
                    mibReq.Param.freqband = get_next_freqband();
                    LoRaMacMibSetRequestConfirm(&mibReq);
                   
                    mibReq.Type = MIB_NETWORK_JOINED;
                    mibReq.Param.IsNetworkJoined = true;
                    LoRaMacMibSetRequestConfirm(&mibReq);
                    
                    mibReq.Type = MIB_DEVICE_CLASS;
                    mibReq.Param.Class = g_lwan_dev_config_p->modes.class_mode;
                    LoRaMacMibSetRequestConfirm(&mibReq);  
                    
                    lwan_mac_params_update();
                    
                    g_lwan_device_state = DEVICE_STATE_SLEEP;   
		        }else if(g_lwan_dev_config_p->modes.join_mode == JOIN_MODE_OTAA) {
                    if(g_lwan_dev_config_p->join_settings.auto_join){
                        g_lwan_device_state = DEVICE_STATE_JOIN;
                    } else {
                        g_lwan_device_state = DEVICE_STATE_SLEEP;
                    }
                }
                lwan_dev_status_set(DEVICE_STATUS_IDLE);
                break;
            }

            case DEVICE_STATE_JOIN: {
			  
			   DBG_LINKWAN("class type2 %c\r\n", (char)('A' + g_lwan_dev_config_p->modes.class_mode));
                if(g_lwan_dev_config_p->modes.join_mode == JOIN_MODE_OTAA){
                    MlmeReq_t mlmeReq;

                    mlmeReq.Type = MLME_JOIN;
                    mlmeReq.Req.Join.DevEui = g_lwan_dev_keys_p->ota.deveui;
                    mlmeReq.Req.Join.AppEui = g_lwan_dev_keys_p->ota.appeui;
                    mlmeReq.Req.Join.AppKey = g_lwan_dev_keys_p->ota.appkey;   
                    mlmeReq.Req.Join.method = g_lwan_dev_config_p->join_settings.join_method;
                    if (g_lwan_dev_config_p->join_settings.join_method == JOIN_METHOD_STORED) {
                        mlmeReq.Req.Join.freqband = g_lwan_dev_config_p->join_settings.stored_freqband;
                        mlmeReq.Req.Join.datarate = g_lwan_dev_config_p->join_settings.stored_datarate;
                        mlmeReq.Req.Join.NbTrials = 3;
                    } else {
                        mlmeReq.Req.Join.NbTrials = g_lwan_dev_config_p->join_settings.join_trials;
                    }

                    if (next_tx == true && rejoin_flag == true) {
                        if (LoRaMacMlmeRequest(&mlmeReq) == LORAMAC_STATUS_OK) {
                            next_tx = false;
                        }                        
                        DBG_LINKWAN("Start to Join, method %u, nb_trials:%u\r\n",
                                    (uint16_t)g_lwan_dev_config_p->join_settings.join_method, (uint16_t)mlmeReq.Req.Join.NbTrials);
                    }
		        }
                g_lwan_device_state = DEVICE_STATE_SLEEP;
                break;
            }
            case DEVICE_STATE_JOINED: {
                MibRequestConfirm_t mib_req;
                JoinSettings_t join_settings;
                DBG_LINKWAN("Joined\n\r");     
				 DBG_LINKWAN("class type3 %c\r\n", (char)('A' + g_lwan_dev_config_p->modes.class_mode));
				led_lignt();// led_lignt1s();//TEST
                //切换classd？

				/*#ifdef CONFIG_LINKWAN_D2D
                g_lwan_dev_config_p->modes.class_mode = CLASS_D;
				#endif      
				*/
                lwan_dev_config_get(DEV_CONFIG_JOIN_SETTINGS, &join_settings);
                mib_req.Type = MIB_FREQ_BAND;
                LoRaMacMibGetRequestConfirm(&mib_req);
                join_settings.stored_freqband = mib_req.Param.freqband;
                mib_req.Type = MIB_CHANNELS_DATARATE;
                LoRaMacMibGetRequestConfirm(&mib_req);
                join_settings.stored_datarate = mib_req.Param.ChannelsDatarate;
                join_settings.join_method = JOIN_METHOD_STORED;
                
                lwan_dev_config_set(DEV_CONFIG_JOIN_SETTINGS, &join_settings);
                
                lwan_mac_params_update();
                if(g_lwan_dev_config_p->modes.class_mode != CLASS_A){
                    
                    mib_req.Type = MIB_DEVICE_CLASS;
                    mib_req.Param.Class = g_lwan_dev_config_p->modes.class_mode;
                    LoRaMacMibSetRequestConfirm(&mib_req);
                    
                    g_lwan_device_state = DEVICE_STATE_SLEEP;
                }else{
                    g_lwan_device_state = DEVICE_STATE_SEND;
                }
				//testbz=1;
				g_lwan_device_state = DEVICE_STATE_SEND;//必须joined之后才能切换send
				//DBG_LINKWAN("Send 2...\r");
                break;
            }
            case DEVICE_STATE_SEND: {
                if (next_tx == true) {
				  DBG_LINKWAN("start send\n\r");  
                    prepare_tx_frame();
                    next_tx = send_frame();
                }
                if (g_lwan_mac_config_p->modes.report_mode == TX_ON_TIMER) {
                    start_dutycycle_timer();//定时发送
                }
                g_lwan_device_state = DEVICE_STATE_SLEEP;
                break;
            }
            case DEVICE_STATE_SEND_MAC: {
                if (next_tx == true) {
                    tx_data.BuffSize = 0;
                    next_tx = send_frame();
                }
                g_lwan_device_state = DEVICE_STATE_SLEEP;
                break;
            }
            case DEVICE_STATE_SLEEP: {
                TimerLowPowerHandler( );  //SLEEP為空           
                break;
            }
            default: {
                g_lwan_device_state = DEVICE_STATE_INIT;
                break;
            }
        }
    }
}

DeviceState_t lwan_dev_state_get( void )
{
    return g_lwan_device_state;
}

void lwan_dev_state_set(DeviceState_t state)
{
    if (g_lwan_device_state == DEVICE_STATE_SLEEP) {
        TimerStop(&TxNextPacketTimer);
    }
    g_lwan_device_state = state;
}

bool lwan_dev_status_set(DeviceStatus_t ds)
{
    g_lwan_device_status = ds;
    return true;
}
DeviceStatus_t lwan_dev_status_get(void)
{
    return g_lwan_device_status;
}

bool lwan_is_dev_busy()
{
    MibRequestConfirm_t mibReq;
    mibReq.Type = MIB_MAC_STATE;
    LoRaMacMibGetRequestConfirm(&mibReq);
    
    if(g_lwan_device_state == DEVICE_STATE_SLEEP 
        && mibReq.Param.LoRaMacState == 0)
        return false;
    
    return true;
}

int lwan_mac_req_send(int type, void *param)
{
    MlmeReq_t mlmeReq;
    int ret = LWAN_SUCCESS;
    
    switch(type) {
        case MAC_REQ_LINKCHECK: {
            mlmeReq.Type = MLME_LINK_CHECK;
            break;
        }
		#ifdef CONFIG_LINKWAN_D2D
        case MAC_REQ_PREAMBLE_INFO:{
            uint16_t preambletime = *(uint16_t *)param;
            mlmeReq.Type = MLME_PREAMBLE_INFO;
            mlmeReq.Req.PreambleInfo.Period.Value = preambletime/100-1;
            preambletime = (preambletime / 100 - 1) * 100;
            lwan_dev_config_set(DEV_CONFIG_CLASSD_PARAM, &preambletime);
            break;
        }
		#endif        
        default: {
            ret = LWAN_ERROR;
            break;
        }
    }
    
    if (LoRaMacMlmeRequest(&mlmeReq) == LORAMAC_STATUS_OK) {
        g_lwan_device_state = DEVICE_STATE_SEND_MAC;
    }
    

    return ret;
}


#ifdef CONFIG_LINKWAN_D2D
int lwan_set_d2d_preamble_time(uint16_t interval)
{
    return lwan_mac_req_send(MAC_REQ_PREAMBLE_INFO, &interval);
}

int lwan_get_d2d_preamble_time(uint16_t * interval)
{
    return lwan_dev_config_get(DEV_CONFIG_CLASSD_PARAM, interval);
}

int lwan_data_local_send(const char * data, uint32_t len)
{
    MibRequestConfirm_t mib_req;
    LoRaMacStatus_t status;

    TimerStop(&TxNextPacketTimer);
    
    mib_req.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm(&mib_req);
    if (status == LORAMAC_STATUS_OK) {
        if (mib_req.Param.IsNetworkJoined == true) {
            g_data_send_msg_type = LORAWAN_UNCONFIRMED_MSG;
            g_data_send_local = true;
            memcpy(tx_data.Buff, data, len);
            tx_data.BuffSize = len;
            g_data_send_nbtrials = 1;
            g_lwan_device_state = DEVICE_STATE_SEND;
			DBG_LINKWAN("Send 4...\r");
            return LWAN_SUCCESS;
        }
    }
    return LWAN_ERROR;
}

int lwan_reg_local_rx_cb(rx_cb_t rx_cb)
{
    d2d_rx_callback = rx_cb;
    return LWAN_SUCCESS;
}
#endif

int lwan_join(uint8_t bJoin, uint8_t bAutoJoin, uint16_t joinInterval, uint16_t joinRetryCnt)
{
    int ret = LWAN_SUCCESS;
    JoinSettings_t join_settings;
    lwan_dev_config_get(DEV_CONFIG_JOIN_SETTINGS, &join_settings);
    join_settings.auto_join = bAutoJoin;
    if(joinInterval>=7 && joinInterval<=255)
        join_settings.join_interval = joinInterval;
    if(joinRetryCnt>=1 && joinRetryCnt<=255)
        join_settings.join_trials = joinRetryCnt;
    lwan_dev_config_set(DEV_CONFIG_JOIN_SETTINGS, &join_settings);
        
    if(bJoin == 0){//stop join
        MibRequestConfirm_t mib_req;
        LoRaMacStatus_t status;
        
        TimerStop(&TxNextPacketTimer);
        
        mib_req.Type = MIB_NETWORK_JOINED;
        mib_req.Param.IsNetworkJoined = false;
        status = LoRaMacMibSetRequestConfirm(&mib_req);

        if (status != LORAMAC_STATUS_OK)
            return LWAN_ERROR;
        g_lwan_device_state = DEVICE_STATE_SLEEP;
        rejoin_flag = bAutoJoin;
    } else if(bJoin == 1){
        LoRaMacStatus_t status;
        MibRequestConfirm_t mib_req;
        mib_req.Type = MIB_NETWORK_JOINED;
        status = LoRaMacMibGetRequestConfirm(&mib_req);
        if (status != LORAMAC_STATUS_OK) 
            return LWAN_ERROR;
        
        if (mib_req.Param.IsNetworkJoined == true) {
            mib_req.Type = MIB_NETWORK_JOINED;
            mib_req.Param.IsNetworkJoined = false;
            status = LoRaMacMibSetRequestConfirm(&mib_req);
            if(status  != LORAMAC_STATUS_OK) {
                return LWAN_ERROR;
            }
            DBG_LINKWAN("Rejoin again...\r");
        }
        
        TimerStop(&TxNextPacketTimer);   
        rejoin_flag = true;
        reset_join_state();
    } else{
        ret = LWAN_ERROR;
    }
    
    return ret;
}

int lwan_data_send(uint8_t confirm, uint8_t Nbtrials, uint8_t *payload, uint8_t len)
{
    MibRequestConfirm_t mib_req;
    LoRaMacStatus_t status;

    TimerStop(&TxNextPacketTimer);

    mib_req.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm(&mib_req);
    if (status == LORAMAC_STATUS_OK) {
        if (mib_req.Param.IsNetworkJoined == true) {
            g_data_send_msg_type = confirm;
            memcpy(tx_data.Buff, payload, len);
            tx_data.BuffSize = len;
            g_data_send_nbtrials = Nbtrials;
            g_lwan_device_state = DEVICE_STATE_SEND;
			DBG_LINKWAN("Send 5...\r");
            return LWAN_SUCCESS;
        }
    }
    return LWAN_ERROR;
}


bool lwan_multicast_add(void *multicastInfo )
{
    MibRequestConfirm_t mibset;
    LoRaMacStatus_t status;
    MulticastParams_t *pmulticastInfo;

    pmulticastInfo = (MulticastParams_t *)multicastInfo;
    if (!pmulticastInfo)
        return false;

    mibset.Type = MIB_MULTICAST_CHANNEL;
    mibset.Param.MulticastList = pmulticastInfo;
    status = LoRaMacMibSetRequestConfirm(&mibset);
    if (status !=  LORAMAC_STATUS_OK) {
        return false;
    }
    return true;
}

bool lwan_multicast_del(uint32_t dev_addr)
{
    MulticastParams_t *multiCastNode = get_lora_cur_multicast();
    if (multiCastNode == NULL) {
        return false;
    }
    
    while (multiCastNode != NULL) {
        if (dev_addr == multiCastNode->Address) {
            MibRequestConfirm_t mibset;
            LoRaMacStatus_t status;
            mibset.Type = MIB_MULTICAST_CHANNEL_DEL;
            mibset.Param.MulticastList = multiCastNode;
            status = LoRaMacMibSetRequestConfirm(&mibset);
            if (status !=  LORAMAC_STATUS_OK) {
                return false;
            } else {
                return true;
            }
        }
        multiCastNode = multiCastNode->Next;

    }
    return false;
}

#ifdef CONFIG_LINKWAN_D2D
void lwan_classd_mcchannel_setup(void)
{
    uint32_t i ;
    for( i = 0;i<sizeof(ClassDMCChannels)/sizeof(MulticastParams_t);i++){
        if(ClassDMCChannels[i].Address){
            LoRaMacD2DMcastComputeSKeys(ClassDMCKeys[i].key,ClassDMCChannels[i].Address,ClassDMCChannels[i].AppSKey,ClassDMCChannels[i].NwkSKey);
            lwan_multicast_add(&ClassDMCChannels[i]);
        }
    }    
}
#endif
