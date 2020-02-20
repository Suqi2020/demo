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

/* sub-second number of bits */
#define N_PREDIV_S                11

/* Synchronous prediv  */
#define PREDIV_S                  ( ( 1 << N_PREDIV_S ) - 1 )  //2047

/* Asynchronous prediv   */
#define PREDIV_A                  ( 1 << ( 15 - N_PREDIV_S ) ) - 1 //15



/*!
 * RTC timer context
 */
typedef struct RtcCalendar_s
{
    RTC_DateTypeDef CalendarDate; //! Reference time in calendar format
    RTC_TimeTypeDef CalendarTime; //! Reference date in calendar format
    uint32_t CalendarSubSeconds;
} RtcCalendar_t;


RtcCalendar_t RtcConvertTimerTimeToCalendarTick( TimerTime_t timeCounter );
/*!
 * \brief Timer time variable definition
 */
//#ifndef TimerTime_t
//typedef uint32_t TimerTime_t;
//#endif

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
TimerTime_t RtcGetAdjustedTimeoutValue( uint32_t timeout );

/*!
 * \brief Get the RTC timer value
 *
 * \retval RTC Timer value
 */
TimerTime_t RtcGetTimerValue( Tick_s_ms_enum tick );

/*!
 * \brief Get the RTC timer elapsed time since the last Alarm was set
 *
 * \retval RTC Elapsed time since the last alarm
 */
TimerTime_t RtcGetElapsedAlarmTime( void );

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
TimerTime_t RtcComputeElapsedTime( TimerTime_t eventInTime );

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

TimerTime_t RtcConvertCalendarTickToTimerTime( RtcCalendar_t *calendar,Tick_s_ms_enum tick );

#endif // __RTC_BOARD_H__
