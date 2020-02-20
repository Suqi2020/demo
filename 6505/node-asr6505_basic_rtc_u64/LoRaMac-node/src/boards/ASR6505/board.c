/*!
 * \file      board.c
 *
 * \brief     Target board general functions implementation
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
#include <stdio.h> 
#include "stm8l15x.h"
#include "stm8l15x_rtc.h"
#include "stm8l15x_flash.h"
#include "utilities.h"
#include "gpio.h"
#include "spi.h"
#include "board-config.h"
#include "timer.h"
#include "rtc-board.h"
#include "sx126x-board.h"
#include "uart-board.h"
#include "board.h"
//uart
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define EVAL_BUSY_GPIO_PORT              GPIOG
#define EVAL_BUSY_GPIO_PIN               GPIO_Pin_6
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

extern bool WakeUpTimeInitialized;
extern volatile uint32_t McuWakeUpTime;


static void BoardUnusedIoInit( void );
static void SystemClockConfig( void );
static void CalibrateSystemWakeupTime( void );
static void SystemClockReConfig( void );



static TimerEvent_t CalibrateSystemWakeupTimeTimer;
static bool McuInitialized = false;
static bool SystemWakeupTimeCalibrated = false;
//uint32_t clk;//CLK_GetClockFreq()
static void OnCalibrateSystemWakeupTimeTimerEvent( void )
{
    SystemWakeupTimeCalibrated = true;

}

uint8_t GetBoardPowerSource(void)
{
	return BATTERY_POWER;
}

void BoardDisableIrq( void )
{
    disableInterrupts( );
}

void BoardEnableIrq( void )
{
    enableInterrupts( );
}

void BoardInitPeriph( void )
{
    
}

/*与207交互IO口设置为高*/

void BoardInitMcu( void )
{  
    if( McuInitialized == false )
    {
        CFG->GCR |= CFG_GCR_SWD;
		//clk=CLK_GetClockFreq();
        SystemClockConfig( );
        //clk=CLK_GetClockFreq();
        BoardUnusedIoInit( );
		//RadioIqrRst();
        RtcInit( );
    }
    else
    {
        SystemClockReConfig( );
    }
    
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);

    USARTInit();
    SX126xIoInit( );
    if( McuInitialized == false ){
      SX126xDioInit();
    }
    
    SpiInit(&SX126x.Spi, RADIO_SPI_ID, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC/*RADIO_NSS*/);  
    
    if( McuInitialized == false )
    {
        McuInitialized = true;        
        
        //read the last MCU wakeup time
        WakeUpTimeInitialized = (bool)(*(uint8_t *)FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS);
        McuWakeUpTime = (uint32_t)(*(uint8_t *)(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS + sizeof(uint8_t)));
     //   printf("read time %lu\r\n",McuWakeUpTime);
        if(!WakeUpTimeInitialized) {
            if( GetBoardPowerSource( ) == BATTERY_POWER )
            {
    //    printf("write time1: %lu\r\n",McuWakeUpTime);  
                 CalibrateSystemWakeupTime( );   //烧录后第一次上电计算McuWakeUpTime值 应该为2 单步调试时候会使该值变的很大  会导致软件定时器不准  解决方法：debug Project--Download--Erase memory
      // printf("write time2: %lu\r\n",McuWakeUpTime);  
            }
            
            FLASH_SetProgrammingTime(FLASH_ProgramTime_Standard);
            FLASH_Unlock(FLASH_MemType_Data);
            //FLASH_ProgramOptionByte(0x4800,0x11); //ROP on
            //FLASH_ProgramOptionByte(0x480B,0x55); //bootloader enable
            //FLASH_ProgramOptionByte(0x480C,0xAA); //bootloader enable
       // printf("write time3: %lu\r\n",McuWakeUpTime);  
            FLASH_ProgramByte(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS, (uint8_t)WakeUpTimeInitialized);
            FLASH_ProgramByte(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS + sizeof(uint8_t), (uint8_t)McuWakeUpTime);
    //    printf("write time4: %lu\r\n",McuWakeUpTime);  
            FLASH_Lock(FLASH_MemType_Data);
        }        
    }    
}

void BoardResetMcu( void )
{
    BoardDisableIrq( );
}

void BoardDeInitMcu( void )
{
    COMDeInit(COM0);
    COMDeInit(COM1);
    SpiDeInit( &SX126x.Spi );
    SX126xIoDeInit( );
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, DISABLE);
}

uint32_t BoardGetRandomSeed( void )
{
    return 0;
}

#define         ID1                                 ( 0x4926 )
#define         ID2                                 ( 0x492A )
#define         ID3                                 ( 0x492E )

void BoardGetUniqueId( uint8_t *id )
{
    id[7] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 24;
    id[6] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 16;
    id[5] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 8;
    id[4] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) );
    id[3] = ( ( *( uint32_t* )ID2 ) ) >> 24;
    id[2] = ( ( *( uint32_t* )ID2 ) ) >> 16;
    id[1] = ( ( *( uint32_t* )ID2 ) ) >> 8;
    id[0] = ( ( *( uint32_t* )ID2 ) );
}

uint16_t BoardBatteryMeasureVolage( void )
{
    return 0;
}

uint32_t BoardGetBatteryVoltage( void )
{
    return 0;
}

uint8_t BoardGetBatteryLevel( void )
{
    return 0;
}

static void BoardUnusedIoInit( void )
{
    GPIO_Init(GPIOA, GPIO_Pin_All, GPIO_Mode_In_PU_No_IT);
    GPIO_Init(GPIOB, GPIO_Pin_All, GPIO_Mode_In_PU_No_IT);
	
    GPIO_Init(GPIOC, GPIO_Pin_All, GPIO_Mode_In_PU_No_IT);
    GPIO_Init(GPIOD, GPIO_Pin_All, GPIO_Mode_In_PU_No_IT);
    GPIO_Init(GPIOE, GPIO_Pin_All, GPIO_Mode_In_PU_No_IT);
    GPIO_Init(GPIOF, GPIO_Pin_All, GPIO_Mode_In_PU_No_IT);
    GPIO_Init(GPIOG, GPIO_Pin_All, GPIO_Mode_In_PU_No_IT);
    GPIO_Init(GPIOH, GPIO_Pin_All, GPIO_Mode_In_PU_No_IT);
    GPIO_Init(GPIOI, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3, GPIO_Mode_In_PU_No_IT);
	
	//GPIO_Init(GPIOE, GPIO_Pin_6, GPIO_Mode_Out_PP_Low_Fast);
	GPIO_Init(GPIOC, GPIO_Pin_4, GPIO_Mode_Out_PP_High_Fast);
	/****************************以下三句只适用于网关不适用于结点*****************************************************/
	//GPIO_Init(GPIOE, GPIO_Pin_6, GPIO_Mode_Out_PP_High_Fast);//GPIO3
	//GPIO_Init(GPIOE, GPIO_Pin_7, GPIO_Mode_In_PU_IT);//GPIO4 PE7
	//EXTI_SetPinSensitivity(EXTI_Pin_7, EXTI_Trigger_Falling_Low);
    /*****************************使用结点时候需要屏蔽*********************************************/
	GPIO_Init(GPIOE, GPIO_Pin_6, GPIO_Mode_Out_PP_High_Fast);//GPIO3
	GPIO_SetBits(GPIOE, GPIO_Pin_6);
	GPIO_Init(GPIOE, GPIO_Pin_7, GPIO_Mode_In_PU_IT);//GPIO4 PE7
	EXTI_SetPinSensitivity(EXTI_Pin_7, EXTI_Trigger_Falling);
        
        
#ifdef TEST

	/*GPIO_Init(GPIOE, GPIO_Pin_6, GPIO_Mode_Out_PP_High_Fast);//GPIO3
	GPIO_SetBits(GPIOE, GPIO_Pin_6);
	GPIO_Init(GPIOE, GPIO_Pin_7, GPIO_Mode_Out_PP_High_Fast);//GPIO4
	GPIO_SetBits(GPIOE, GPIO_Pin_7);*/
	GPIO_Init(GPIOC, GPIO_Pin_0, GPIO_Mode_Out_PP_High_Fast);//I2C_SDA
	GPIO_ResetBits(GPIOC, GPIO_Pin_0);
	GPIO_Init(GPIOC, GPIO_Pin_1, GPIO_Mode_Out_PP_High_Fast);//I2C_SCL
	GPIO_ResetBits(GPIOC, GPIO_Pin_1);
	//GPIO_Init(GPIOC, GPIO_Pin_2, GPIO_Mode_Out_PP_High_Fast);//ADC1_IN0
	//GPIO_SetBits(GPIOC, GPIO_Pin_2);
#else  
        GPIO_Init(GPIOC, GPIO_Pin_1, GPIO_Mode_Out_PP_High_Fast);//I2C_SCL
	GPIO_SetBits(GPIOC, GPIO_Pin_1);
        //GPIO_ResetBits(GPIOC, GPIO_Pin_1);
        	GPIO_Init(GPIOC, GPIO_Pin_0, GPIO_Mode_Out_PP_High_Fast);//I2C_SDA
	GPIO_SetBits(GPIOC, GPIO_Pin_0);
        
#endif
        
}

volatile unsigned int  i;
void delay_1ms(void)//1ms延时函数
{
    i=850;
    while(--i);
}
void delay_ms(int ms)//ms延时函数
{

 while(--ms)
 {
   delay_1ms();
 }
}
 void Led_fun()
{
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);
  //GPIO_Init(GPIOC, GPIO_Pin_4, GPIO_Mode_Out_PP_High_Fast);
  	GPIO_SetBits(GPIOC, GPIO_Pin_4);
	delay_ms(200);
	GPIO_ResetBits(GPIOC, GPIO_Pin_4);
	delay_ms(200);
	GPIO_ToggleBits(GPIOC, GPIO_Pin_4);
	delay_ms(200);
	GPIO_ToggleBits(GPIOC, GPIO_Pin_4);
	delay_ms(200);//DelayMs(20);
	GPIO_ToggleBits(GPIOC, GPIO_Pin_4);
	delay_ms(200);//DelayMs(20);
	GPIO_ToggleBits(GPIOC, GPIO_Pin_4);
	delay_ms(2000);//DelayMs(100);
}

void led_lignt1s()
{
	GPIO_SetBits(GPIOC, GPIO_Pin_4);
	delay_ms(2000);
	GPIO_ResetBits(GPIOC, GPIO_Pin_4);
	delay_ms(2000);

}


void led_lignt()
{
	GPIO_SetBits(GPIOC, GPIO_Pin_4);
}
void CalibrateSystemWakeupTime( void )
{
    if( SystemWakeupTimeCalibrated == false )
    {
        TimerInit( &CalibrateSystemWakeupTimeTimer, OnCalibrateSystemWakeupTimeTimerEvent );
        TimerSetValue( &CalibrateSystemWakeupTimeTimer, 1000 );
        TimerStart( &CalibrateSystemWakeupTimeTimer );
        while( SystemWakeupTimeCalibrated == false )
        {
            TimerLowPowerHandler( );
        }
    }
}
uint32_t clk=0;
void SystemClockConfig( void )
{
	CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);//CLK_SYSCLKSourceConfig
	
#if 1  
    /* Enable LSE */
    CLK_LSEConfig(CLK_LSE_ON);
	//clk=CLK_GetClockFreq();
    /* Wait for LSE clock to be ready */
    while (CLK_GetFlagStatus(CLK_FLAG_LSERDY) == RESET);
    
    /* Select LSE (32.768 KHz) as RTC clock source */
    CLK_RTCClockConfig(CLK_RTCCLKSource_LSE, CLK_RTCCLKDiv_1);
#else    
    CLK_RTCClockConfig(CLK_RTCCLKSource_LSI, CLK_RTCCLKDiv_1);
#endif    
    
    CLK_PeripheralClockConfig(CLK_Peripheral_RTC, ENABLE);   
}


void SystemClockReConfig( void )
{
    CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);   
}
