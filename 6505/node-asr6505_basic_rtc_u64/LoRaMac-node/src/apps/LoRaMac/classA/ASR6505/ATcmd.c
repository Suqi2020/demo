#include "atcmd.h"
#include "RegionCN470.h"
#include "uart-board.h"
#include "board.h"
#include "Commissioning.h"
#include  <ctype.h>
#include "LoRaMac.h"
#define   LWAN_DEV_CONFIG_DEFAULT {{0, ULDL_MODE_INTRA, WORK_MODE_NORMAL, CLASS_A}, \
                                 {3, DR_2, DR_3, 0, 0}, {0, 8, 8, JOIN_METHOD_DEF, 1, DR_3}, \
                                 0x0001, 0}


extern uint8_t gu8Join_bz;
 /*uint32_t g_freq_rx = CN470_FIRST_RX1_CHANNEL;

 uint32_t g_freq_tx = CN470_TX_CHANNEL;


 uint8_t g_sf_tx = 5;//暂时设置为5
 uint8_t g_dt_rx = 7;*/
static uint16_t g_tx_preamble_len = 0;
//static uint16_t g_tx_len = 0;
//static uint8_t tx_buf[TX_BUF_SIZE];
const static uint8_t Channel_str[]="AT+CCH=OK\r\n";
//const static uint8_t Join_str[]="AT+JOIN=OK\r\n";
const static uint8_t Work_str[]="AT+WORKMODE=OK\r\n";
const static uint8_t Deveui_str[]="AT+DEVICEEUI=OK\r\n";
const static uint8_t Appeui_str[]="AT+APPLEUI=OK\r\n";
const static uint8_t Appkey_str[]="AT+APPLKEY=OK\r\n";
const static uint8_t DTX_str[]="AT+DTX=OK\r\n";
extern LWanDevConfig_t *g_lwan_dev_config_p;

static int node_channel(int argc, char *argv[])
{
    uint8_t channel = strtol(argv[0], NULL, 0);
    
    if(channel < RADIO_CHANNEL_MAX)
    {
       // g_radio_channel = channel;
        
       //Radio.SetPublicNetwork(true);
        switch(channel)
        {
        case RADIO_CHANNEL_TX:
		if(argc==3)
		{
		    scgw_uart_send((char*)Channel_str,sizeof(Channel_str));
			//g_freq_rx = strtol(argv[1], NULL, 0);
			//printf("rx %ld\r\n",g_freq_rx);
			//g_dt_rx = strtol(argv[2], NULL, 0); 
			//printf("%d\r\n",g_dt_rx);
		}

		  break;
        case RADIO_CHANNEL_RX:
        //case RADIO_CHANNEL_RX_TX:
          if(argc == 3)
          {


			//g_freq_tx = strtol(argv[1], NULL, 0);
          //  printf("tx %ld\r\n",g_freq_tx);
            //g_sf_tx = strtol(argv[2], NULL, 0); 
           // printf("%d\r\n",g_sf_tx);
           //临时加入返回OK指令
            //node_lora_receive();      
          }          
          break;
        default:
          break;
        }
    }
    
    return 0;
}

void hex2bin(unsigned char *pbDest, char *pszSrc, uint16_t nLen)
{
	unsigned char h1, h2;
	unsigned char s1, s2;
	for (int i = 0; i < nLen; i++)
	{
		h1 = pszSrc[2 * i];
		h2 = pszSrc[2 * i + 1];
 
		s1 = toupper(h1) - 0x30;
		if (s1 > 9)
			s1 -= 7;
 
		s2 = toupper(h2) - 0x30;
		if (s2 > 9)
			s2 -= 7;
 
		pbDest[i] = s1 * 16 + s2;
	}
}
/*
static int hex2bin(const char *hex, uint8_t *bin, uint16_t bin_length)
{
    uint16_t hex_length = strlen(hex);
    const char *hex_end    = hex + hex_length;
    uint8_t *cur        = bin;
    uint8_t num_chars  = hex_length & 1;
    uint8_t byte       = 0;

    if (hex_length % 2 != 0) {
        return -1;
    }

    if (hex_length / 2 > bin_length) {
        return -1;
    }

    while (hex < hex_end) {
        if ('A' <= *hex && *hex <= 'F') {
            byte |= 10 + (*hex - 'A');
        } else if ('a' <= *hex && *hex <= 'f') {
            byte |= 10 + (*hex - 'a');
        } else if ('0' <= *hex && *hex <= '9') {
            byte |= *hex - '0';
        } else {
            return -1;
        }
        hex++;
        num_chars++;

        if (num_chars >= 2) {
            num_chars = 0;
            *cur++    = byte;
            byte      = 0;
        } else {
            byte <<= 4;
        }
    }
    return cur - bin;
}*/
extern bool NextTx;
static int node_dtx(int argc, char *argv[])
{
    g_tx_preamble_len = strtol(argv[0], NULL, 0)/2; 
     hex2bin(AppData,( char *)argv[1],LORAWAN_APP_DATA_MAX_SIZE);
	 scgw_uart_send((char*)DTX_str,sizeof(DTX_str));
	 
   // g_tx_len = hex2bin((const char *)argv[1],AppData,LORAWAN_APP_DATA_MAX_SIZE);//AppData[LORAWAN_APP_DATA_MAX_SIZE];
    printf("data:");
    for(int i=0;i<g_tx_preamble_len;i++)
    {
      printf("%x ",AppData[i]);
    }
    AppDataSize=g_tx_preamble_len;
    printf("\r\n");
    //TimerInit( &LoraTxTimer, OnTxTimerEvent );
    //TimerSetValue(&LoraTxTimer, LORA_TX_DELAY);
    //OnTxTimerEvent();
    DeviceState = DEVICE_STATE_SEND;//此處重新初始化
    NextTx = true;
    return 0;
}

/*static int node_join(int argc, char *argv[])
{
	scgw_uart_send(Join_str,sizeof(Join_str));
	
}*/
static int node_deviceeui(int argc, char *argv[])
{
   //DevEui[8],
   hex2bin(DevEui,( char *)argv[0],16);
  // g_tx_len=hex2bin((const char *)argv[0],DevEui,16);
   printf("DevEui:");
   for(int i=0;i<8;i++)
     printf("%x ",DevEui[i]);
   printf("\r\n");
   scgw_uart_send((char*)Deveui_str,sizeof(Deveui_str));
   return 0;
   
}
static int node_appleui(int argc, char *argv[])
{
   //AppEui[8],
    hex2bin(AppEui,( char *)argv[0],16);
    //g_tx_len=hex2bin((const char *)argv[0],AppEui,16);
    printf("AppEui:");
    for(int i=0;i<8;i++)
    printf("%x ",AppEui[i]);
    printf("\r\n");
	scgw_uart_send((char*)Appeui_str,sizeof(Appeui_str));
    return 0;

}
static int node_applkey(int argc, char *argv[])
{
   //AppKey[16];
     hex2bin(AppKey,( char *)argv[0],32);
    //g_tx_len=hex2bin((const char *)argv[0],AppKey,32);
    printf("AppKey:");
    for(int i=0;i<16;i++)
    printf("%x ",AppKey[i]);
    printf("\r\n");
	scgw_uart_send((char*)Appkey_str,sizeof(Appkey_str));
    return 0;

}

static LWanDevConfig_t g_lwan_dev_config;
LWanDevConfig_t *lwan_dev_config_init(LWanDevConfig_t *default_config)
{
  
   // if(read_lwan_dev_config(&g_lwan_dev_config) != LWAN_SUCCESS) {
        memcpy(&g_lwan_dev_config, default_config, sizeof(LWanDevConfig_t));
  //  }
    return &g_lwan_dev_config;
}

static int node_workmode(int argc, char *argv[])
{
  u8 mode=0;//Work_mode
  scgw_uart_send((char*)Work_str,sizeof(Work_str));
  mode =strtol(argv[0], NULL, 0);
  if(mode==1){
  	WORKMODE=CLASS_A;
  
  	printf("workmode A\r\n");
  	}
  else if(mode==2){
  	WORKMODE=CLASS_B;    
        LWanDevConfig_t default_dev_config = (LWanDevConfig_t)LWAN_DEV_CONFIG_DEFAULT;
        default_dev_config.classb_param.beacon_freq=502700000;
        default_dev_config.classb_param.beacon_dr  =DR_TEST;
        default_dev_config.classb_param.pslot_freq =502700000;
        default_dev_config.classb_param.pslot_dr   =DR_TEST;
        g_lwan_dev_config_p = lwan_dev_config_init(&default_dev_config);
    
    
  	printf("workmode B\r\n");
        
        

  	}
  else if(mode==3){
  	WORKMODE=CLASS_C;
  	printf("workmode C\r\n");
  	}
   
  DeviceState = DEVICE_STATE_INIT;//此處重新初始化
  gu8Join_bz=1;
  return 0;
   
}
static TestCaseSt gCases[] = {

    { "AT+CCH=", &node_channel },//AT+CCH=1 rx   AT+CCH=2 RX
    { "AT+DTX=", &node_dtx },// 数据通过lora发送
    //{"AT+JOIN=0", &node_join},
    {"AT+DEVICEEUI=",&node_deviceeui},
    {"AT+APPLEUI=",&node_appleui},
    {"AT+APPLKEY=",&node_applkey},
    {"AT+WORKMODE=",&node_workmode}
};



int ATCmdFun(void )
{	
  
    int ret ;
    int argc ;
    char *argv[16];
    char *ptr = NULL;
    char *str = NULL;
    int i=0;
    int case_num = sizeof(gCases)/sizeof(TestCaseSt);
    for (i=0; i<case_num; i++) 
    {
	    int cmd_len = strlen(gCases[i].name);
	    if (!strncmp((const char *)gu8Buffer, gCases[i].name, cmd_len))
	    {
	        if(gCases[i].fn)
	        {
	          ptr = (char *)gu8Buffer + cmd_len;
	          argc = 0;
	          str = strtok((char *)ptr, ",");
	          while(str) {
	              argv[argc++] = str;
	              str = strtok((char *)NULL, ",");
	          }
	          ret = gCases[i].fn(argc, argv);
                  //Len=0;
	        }
	        break;
            }
    }
    Len=0;
    return ret;
}
