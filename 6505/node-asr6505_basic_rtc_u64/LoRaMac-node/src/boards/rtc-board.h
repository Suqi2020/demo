/*!
 * \file      rtc-board.h
 *
 * \brief     Target board RTC timer and low power modes management
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
#ifndef __RTC_BOARD_H__
#define __RTC_BOARD_H__

#include <stdint.h>
#include <stdbool.h>
#include "stm8l15x_rtc.h"

/*!
 * RTC Time base in ms
 */
#define RTC_ALARM_TICK_PER_MS                       0x7FF           //  2047 > number of sub-second ticks per second

/* sub-second number of bits */
#define N_PREDIV_S                11

/* Synchronous prediv  */
#define PREDIV_S                  ( ( 1 << N_PREDIV_S ) - 1 )

/* Asynchronous prediv   */
#define PREDIV_A                  ( 1 << ( 15 - N_PREDIV_S ) ) - 1

/* RTC Time base in us */
#define USEC_NUMBER               1000000
#define MSEC_NUMBER               ( USEC_NUMBER / 1000 )
#define RTC_ALARM_TIME_BASE       ( USEC_NUMBER >> N_PREDIV_S )

#define COMMON_FACTOR             3
#define CONV_NUMER                ( MSEC_NUMBER >> COMMON_FACTOR ) // 1000>>3   //125
#define CONV_DENOM                ( 1 << ( N_PREDIV_S - COMMON_FACTOR ) )//1<<(11-3)    256

#define MIN_ALARM_DELAY                             6
   /*!
 * RTC timer context
 */
typedef struct RtcCalendar_s
{
    RTC_DateTypeDef CalendarDate; //! Reference time in calendar format
    RTC_TimeTypeDef CalendarTime; //! Reference date in calendar format
    uint32_t CalendarSubSeconds;
} RtcCalendar_t;

/*!
 * \brief Timer time variable definition
 */
//#ifndef TimerTime_t
//typedef uint32_t TimerTime_t;
//#endif
uint32_t RtcConvertMsToTick( uint32_t timeoutValue );
/*!
 * \brief Initializes the RTC timer
 *
 * \remark The timer is based on the RTC
 */
void RtcInit( void );

/*!
 * \brief Start the RTC timer
 *
 * \remark The timer is based on the RTC Alarm running at 32.768KHz
 *
 * \param[IN] timeout Duration of the Timer
 */
void RtcSetTimeout( uint32_t timeout );

/*!
 * \brief Adjust the value of the timeout to handle wakeup time from Alarm and GPIO irq
 *
 * \param[IN] timeout Duration of the Timer without compensation for wakeup time
 * \retval new value for the Timeout with compensations
 */
uint32_t RtcGetAdjustedTimeoutValue( uint32_t timeout );

/*!
 * \brief Get the RTC timer value
 *
 * \retval RTC Timer value
 */
TimerTime_t RtcGetTimerValue( void );

/*!
 * \brief Get the RTC timer elapsed time since the last Alarm was set
 *
 * \retval RTC Elapsed time since the last alarm
 */
uint32_t RtcGetElapsedAlarmTime( void );

/*!
 * \brief Compute the timeout time of a future event in time
 *
 * \param[IN] futureEventInTime Value in time
 * \retval time Time between now and the futureEventInTime
 */
TimerTime_t RtcComputeFutureEventTime( TimerTime_t futureEventInTime );

/*!
 * \brief Compute the elapsed time since a fix event in time
 *
 * \param[IN] eventInTime Value in time
 * \retval elapsed Time since the eventInTime
 */
uint32_t RtcComputeElapsedTime( TimerTime_t eventInTime );

/*!
 * \brief This function blocks the MCU from going into low power mode
 *
 * \param [IN] status [true: Enable, false: Disable
 */
void BlockLowPowerDuringTask ( bool status );

/*!
 * \brief Sets the MCU into low power STOP mode
 */
void RtcEnterLowPowerStopMode( void );

/*!
 * \brief Restore the MCU to its normal operation mode
 */
void RtcRecoverMcuStatus( void );

/*!
 * \brief Processes pending timer events
 */
void RtcProcess( void );
TimerTime_t RtcGetTickCurrentTime( void );
 RtcCalendar_t RtcGetCalendar( void );
///void RtcWriteCalendar( RtcCalendar_t CalendarTime,uint16_t MsCounter );
typedef struct rtc_time_struct
{
    uint16_t ui8Year;       // 1970~2038
    uint8_t ui8Month;       // 1~12
    uint8_t ui8DayOfMonth;  // 1~31
    uint8_t ui8Week;
    uint8_t ui8Hour;        // 0~23
    uint8_t ui8Minute;      // 0~59
    uint8_t ui8Second;      // 0~59
   
}rtc_time_t;
    void RtcWriteTick(uint32_t unixTime,uint16_t MsCounter);
    uint32_t RtcConvertTickToMs( uint32_t timeoutValue );
#endif // __RTC_BOARD_H__
