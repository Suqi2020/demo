/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#ifndef LINKWAN_H
#define LINKWAN_H

#include <stdbool.h>
#include <stdio.h>  

#define LORAWAN_APP_DATA_BUFF_SIZE 242
#define LORAWAN_CONFIRMED_MSG 1
#define LORAWAN_UNCONFIRMED_MSG 0
#define JOINREQ_NBTRIALS 3    
    
#ifdef CONFIG_DEBUG_LINKWAN
#include "debug.h"
#define ERR_LINKWAN     ERR_PRINTF
#define WARN_LINKWAN    WARN_PRINTF
#define DBG_LINKWAN     DBG_PRINTF
#define VDBG_LINKWAN    VDBG_PRINTF
#else
#define ERR_LINKWAN(...)
#define WARN_LINKWAN(...)
#define DBG_LINKWAN(...)
#define VDBG_LINKWAN(...)
#define LORA_LOG(...)
#endif

typedef enum eTxEventType {
    TX_ON_NONE,
    TX_ON_TIMER
} TxEventType_t;

typedef struct {
    uint8_t *Buff;
    uint8_t BuffSize;
    uint8_t Port;
} lora_AppData_t;

typedef struct sLoRaMainCallback {
    uint8_t (*BoardGetBatteryLevel)(void);
    void (*BoardGetUniqueId)(uint8_t *id);
    uint32_t (*BoardGetRandomSeed)(void);
    void (*LoraTxData)(lora_AppData_t *AppData);
    void (*LoraRxData)(lora_AppData_t *AppData);
} LoRaMainCallback_t;

typedef enum eDevicState {
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_JOINED,
    DEVICE_STATE_SEND,
    DEVICE_STATE_SEND_MAC,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP,
    DEVICE_STATE_REQ_DEVICE_TIME,
    DEVICE_STATE_BEACON_ACQUISITION,
    DEVICE_STATE_REQ_PINGSLOT_ACK
} DeviceState_t;

typedef enum eDeviceStatus {
    DEVICE_STATUS_IDLE,//0
    DEVICE_STATUS_SENDING,
    DEVICE_STATUS_SEND_FAIL,
    DEVICE_STATUS_SEND_PASS,
    DEVICE_STATUS_JOIN_PASS,
    DEVICE_STATUS_JOIN_FAIL,
    DEVICE_STATUS_NETWORK_ABNORMAL,
    DEVICE_STATUS_SEND_PASS_WITHOUT_DL,
    DEVICE_STATUS_SEND_PASS_WITH_DL
} DeviceStatus_t;

typedef enum eMacReq {
    MAC_REQ_LINKCHECK,//0
    MAC_REQ_DEVICE_TIME,
    MAC_REQ_PSLOT_INFO,
	#ifdef CONFIG_LINKWAN_D2D
    MAC_REQ_PREAMBLE_INFO,
	#endif    
    MAC_REQ_MAC
} MacReq_t;

#ifdef CONFIG_LINKWAN_D2D
typedef void (* rx_cb_t)(uint8_t *data,uint32_t len);
#endif


void lora_init(LoRaMainCallback_t *callbacks);
void lora_fsm( void );

bool lwan_is_dev_busy();
DeviceState_t lwan_dev_state_get( void );
void lwan_dev_state_set(DeviceState_t state);
DeviceStatus_t lwan_dev_status_get(void);
bool lwan_dev_status_set(DeviceStatus_t ds);

int lwan_join(uint8_t bJoin, uint8_t bAutoJoin, uint16_t joinInterval, uint16_t joinRetryCnt);
int lwan_mac_req_send(int type, void *param);
#ifdef CONFIG_LINKWAN_D2D
int lwan_set_d2d_preamble_time(uint16_t interval);
int lwan_get_d2d_preamble_time(uint16_t * interval);
int lwan_data_local_send(const char * data, uint32_t len);
int lwan_reg_local_rx_cb(rx_cb_t rx_cb);
#endif
int lwan_data_send(uint8_t confirm, uint8_t Nbtrials, uint8_t *payload, uint8_t size);
void lwan_classd_mcchannel_setup(void);
#endif /* LINKWAN_H */
