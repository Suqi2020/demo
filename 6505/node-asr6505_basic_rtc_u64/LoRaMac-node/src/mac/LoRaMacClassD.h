#ifndef __LORAMACCLASSD_H__
#define __LORAMACCLASSD_H__    

#ifdef CONFIG_LINKWAN_D2D

#define KEY_SIZE         16    
    
typedef enum eClassDGroupID
{
    CLASSD_GROUP_ID_0 = 0,
    CLASSD_GROUP_ID_1,
    CLASSD_GROUP_ID_2,
    CLASSD_GROUP_ID_3,    
    CLASSD_GROUP_ID_NUM
}ClassDGroupID_t;

typedef struct sClassDKey
{
    uint8_t key[KEY_SIZE];
}ClassDKey_t;


void OnD2DCADTimerEvent( void );
void LoRaMacClassDCADDone(bool channelActivityDetected);
void LoRaMacStartCADTimer(void);
LoRaMacStatus_t LoRaMacClassDSwitchClass( void);

#endif

#endif    
    