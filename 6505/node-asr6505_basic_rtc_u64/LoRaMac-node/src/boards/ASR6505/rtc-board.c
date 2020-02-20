/*!
 * \file      rtc-board.c
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
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "stm8l15x.h"
#include "board.h"

#include "stm8l15x_pwr.h"
#include "utilities.h"

#include "timer.h"
#include "gpio.h"
#include "radio.h"
#include "rtc-board.h"


/*!
 * Number of seconds in a minute
 */
static const uint8_t SecondsInMinute = 60;

/*!
 * Number of seconds in an hour
 */
static const uint16_t SecondsInHour = 3600;

/*!
 * Number of seconds in a day
 */
static const uint32_t SecondsInDay = 86400;

/*!
 * Number of hours in a day
 */
static const uint8_t HoursInDay = 24;

/*!
 * Number of seconds in a leap year
 */
static const uint32_t SecondsInLeapYear = 31622400;

/*!
 * Number of seconds in a year
 */
static const uint32_t SecondsInYear = 31536000;

/*!
 * Number of days in each month on a normal year
 */
static  uint8_t const DaysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*!
 * Number of days in each month on a leap year
 */
static  uint8_t const DaysInMonthLeapYear[] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*!
 * Flag used to indicates a the MCU has waken-up from an external IRQ
 */
volatile bool NonScheduledWakeUp = false;


/*!
 * \brief RTC Handler
 */
RTC_InitTypeDef   RTC_InitStr;
RTC_TimeTypeDef   RTC_TimeStr;
RTC_DateTypeDef   RTC_DateStr;
RTC_AlarmTypeDef  RTC_AlarmStr;


/*!
 * Current RTC timer context
 */
RtcCalendar_t RtcCalendarContext;

/*!
 * \brief Flag to indicate if the timestamp until the next event is long enough
 * to set the MCU into low power mode
 */
static bool RtcTimerEventAllowsLowPower = false;

/*!
 * \brief Flag to disable the low power mode even if the timestamp until the
 * next event is long enough to allow low power mode
 */
static bool LowPowerDisableDuringTask = false;

/*!
 * \brief Indicates if the RTC is already Initialized or not
 */
static bool RtcInitialized = false;

/*!
 * \brief Indicates if the RTC Wake Up Time is calibrated or not
 */
bool WakeUpTimeInitialized = false;

/*!
 * \brief Hold the Wake-up time duration in ms
 */
volatile uint32_t McuWakeUpTime = 0;

static uint32_t RtcAlarmSubSeconds = 0;

static volatile bool McuStop = false;

/*!
 * \brief RTC wakeup time computation
 */
static void RtcComputeWakeUpTime( void );

/*!
 * \brief Start the RTC Alarm (timeoutValue is in ms)
 */
static void RtcStartWakeUpAlarm( uint32_t timeoutValue );

/*!
 * \brief Converts a TimerTime_t value into RtcCalendar_t value
 *
 * \param[IN] timeCounter Value to convert to RTC calendar
 * \retval rtcCalendar New RTC calendar value
 */
//
// REMARK: Removed function static attribute in order to suppress
//         "#177-D function was declared but never referenced" warning.
// static RtcCalendar_t RtcConvertTimerTimeToCalendarTick( TimerTime_t timeCounter )
//
RtcCalendar_t RtcConvertTimerTimeToCalendarTick( TimerTime_t timeCounter );

/*!
 * \brief Converts a RtcCalendar_t value into TimerTime_t value
 *
 * \param[IN/OUT] calendar Calendar value to be converted
 *                         [NULL: compute from "now",
 *                          Others: compute from given calendar value]
 * \retval timerTime New TimerTime_t value
 */
static TimerTime_t RtcConvertCalendarTickToTimerTime( RtcCalendar_t *calendar );

//static TimerTime_t RtcConvertMsToTick( TimerTime_t timeoutValue );

 uint32_t RtcConvertTickToMs( uint32_t timeoutValue );

/*!
 * \brief Converts a TimerTime_t value into a value for the RTC Alarm
 *
 * \param[IN] timeCounter Value in ms to convert into a calendar alarm date
 * \param[IN] now Current RTC calendar context
 * \retval rtcCalendar Value for the RTC Alarm
 */
static RtcCalendar_t RtcComputeTimerTimeToAlarmTick( uint32_t timeCounter, RtcCalendar_t now );

/*!
 * \brief Returns the internal RTC Calendar and check for RTC overflow
 *
 * \retval calendar RTC calendar
 */
RtcCalendar_t RtcGetCalendar( void );

static double round(double r)
{
	return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}

void RtcInit( void )
{
    if( RtcInitialized == false )
    {
        RTC_InitStr.RTC_HourFormat = RTC_HourFormat_24;
        RTC_InitStr.RTC_AsynchPrediv = PREDIV_A;
        RTC_InitStr.RTC_SynchPrediv = PREDIV_S;
        RTC_Init(&RTC_InitStr);
  
        // Set Date: Friday 1st of January 2000
        RTC_DateStructInit(&RTC_DateStr);
        RTC_DateStr.RTC_WeekDay = RTC_Weekday_Saturday;
        RTC_DateStr.RTC_Date = 1;
        RTC_DateStr.RTC_Month = RTC_Month_January;
        RTC_DateStr.RTC_Year = 0;
        RTC_SetDate(RTC_Format_BIN, &RTC_DateStr);
        
        RTC_TimeStructInit(&RTC_TimeStr);
        RTC_TimeStr.RTC_Hours   = 00;
        RTC_TimeStr.RTC_Minutes = 00;
        RTC_TimeStr.RTC_Seconds = 00;
        RTC_SetTime(RTC_Format_BIN, &RTC_TimeStr);
  

        // Enable Direct Read of the calendar registers (not through Shadow registers)
        RTC_BypassShadowCmd(ENABLE);

        RtcInitialized = true;
    }  
}






void RtcSetTimeout( uint32_t timeout )
{
    RtcStartWakeUpAlarm( RtcConvertMsToTick( timeout ) );//// ӎ˽ոɫms
}

 uint32_t RtcGetAdjustedTimeoutValue( uint32_t timeout )
{
   // printf("waketime %lu\r\n",McuWakeUpTime);//
        
   // McuWakeUpTime=0;
    if( timeout > McuWakeUpTime )
    {   // we have waken up from a GPIO and we have lost "McuWakeUpTime" that we need to compensate on next event
        if( NonScheduledWakeUp == true )
        {
            NonScheduledWakeUp = false;
            timeout -= McuWakeUpTime;
        }
    }

    if( timeout > McuWakeUpTime )
    {   // we don't go in low power mode for delay below 50ms (needed for LEDs)
        if( timeout < 50 ) // 50 ms
        {
            RtcTimerEventAllowsLowPower = false;
        }
        else
        {
            RtcTimerEventAllowsLowPower = true;
            timeout -= McuWakeUpTime;
        }
    }
    return  timeout;
}

TimerTime_t RtcGetTimerValue( void )
{

    return RtcConvertCalendarTickToTimerTime( NULL );


}

//作用：求两个TimerTime_t数的减法 要求 前者时间大于后者
TimerTime_t RtcCompareSubFun(TimerTime_t large,TimerTime_t small)
{
    TimerTime_t sub ={0};
    if(large.msec<small.msec)//前者ms 小于后者
    {
        sub.msec=1000+large.msec-small.msec;
        sub.sec =large.sec-small.sec-1; //被上边借走1
    }
    else
    {
        sub.msec=large.msec-small.msec;
        sub.sec =large.sec-small.sec;
    }
    return sub;
}

//作用：求两个TimerTime_t数的减法 要求 前者时间大于后者
TimerTime_t RtcComparAddFun(TimerTime_t large,TimerTime_t small)
{
    TimerTime_t add ={0};
    if(large.msec+small.msec>=1000)//和大于1000 要进位
    {
        add.msec=large.msec+small.msec-1000;
        add.sec =large.sec+small.sec+1;
    }
    else
    {
        add.msec=large.msec+small.msec;
        add.sec =large.sec+small.sec;
    }
    return add;
}
//
uint32_t RtcGetElapsedAlarmTime( void )
{
  TimerTime_t retVal = {0};
  TimerTime_t currentTime = {0};
  TimerTime_t contextTime = {0};

    currentTime = RtcConvertCalendarTickToTimerTime( NULL );
    contextTime = RtcConvertCalendarTickToTimerTime( &RtcCalendarContext );

 /*   if( currentTime < contextTime )
    {
        retVal = ( currentTime + ( 0xFFFFFFFF - contextTime ) );//此步为了做丢弃处理
    }
    else
    {
        retVal = ( currentTime - contextTime );
    }*/
    
    if(currentTime.sec>contextTime.sec)
    {
        retVal=RtcCompareSubFun(currentTime,contextTime);
    }
    else if((currentTime.sec==contextTime.sec)&&(currentTime.msec>contextTime.msec))
    {
        retVal=RtcCompareSubFun(currentTime,contextTime);
    }
    else
    {
        return 0xffffffff;
    }
    return( (uint32_t)(retVal.sec*1000)+retVal.msec);//返回ms值
}

TimerTime_t RtcComputeFutureEventTime( TimerTime_t futureEventInTime )
{
     return RtcComparAddFun(RtcGetTimerValue( ),futureEventInTime);
    //return( RtcGetTimerValue( ) + futureEventInTime );
}

uint32_t RtcComputeElapsedTime( TimerTime_t eventInTime )
{
    TimerTime_t elapsedTime = {0};
    TimerTime_t elapsedTime_q={0};
    // Needed at boot, cannot compute with 0 or elapsed time will be equal to current time
    if(( eventInTime.sec == 0 )&&(eventInTime.msec==0))
    {
        return 0;
    }
    // first get the current value of the timer in tick
    elapsedTime = RtcConvertCalendarTickToTimerTime( NULL );
    // convert into ms
   // elapsedTime = RtcConvertTickToMs( elapsedTime );

    // compare "eventInTime" with "elapsedTime" while watching for roll over due to 32-bit
    /*if( elapsedTime < eventInTime ) // // roll over of the counter
    {   // due to convertion tick to ms, roll over value is 0x7D000000 (0x7D000000 * 2.048 = 0xFFFFFFFF)
        return( elapsedTime + ( 0x7D000000 - eventInTime ) );
    }
    else
    {
        return( elapsedTime - eventInTime );
    }*/
    
    
    if(elapsedTime.sec>eventInTime.sec)
    {
        elapsedTime_q=RtcCompareSubFun(elapsedTime,eventInTime);
    }
    else if((elapsedTime.sec==eventInTime.sec)&&(elapsedTime.msec>eventInTime.msec))
    {
        elapsedTime_q=RtcCompareSubFun(elapsedTime,eventInTime);
    }
    else
    {
        return 0xffffffff;
    }
    return( (uint32_t)(elapsedTime_q.sec*1000)+elapsedTime_q.msec);//返回ms值
    
    
    
}

void BlockLowPowerDuringTask ( bool status )
{  
    if( status == true )
    {
        RtcRecoverMcuStatus( );
    }   
    LowPowerDisableDuringTask = status;   
}
//޸ɫACTIVEHALTģʽ RTC۽ё
void RtcEnterLowPowerStopMode( void )
{  
    if (Radio.GetStatus() != RF_IDLE) {
        return;
    }
    
    if( ( LowPowerDisableDuringTask == false )  && ( RtcTimerEventAllowsLowPower == true ) )
    {
#ifdef TEST 
    	GPIO_ResetBits(GPIOE, GPIO_Pin_6);
#endif
        BoardDeInitMcu( );
        PWR_UltraLowPowerCmd(ENABLE);  

        McuStop = true;
        halt();
    }   
}

void RtcRecoverMcuStatus( void )
{
    if(McuStop) {
        BoardInitMcu( );
        McuStop = false;
    }
}

void RtcComputeWakeUpTime( void )
{
    uint32_t start = 0;
    uint32_t stop = 0;
    
    RtcCalendar_t now;
    if( WakeUpTimeInitialized == false )//&& RtcAlarmSubSeconds)
    {
        now = RtcGetCalendar( );

        start = PREDIV_S - RtcAlarmSubSeconds;
        stop = PREDIV_S - now.CalendarSubSeconds;

        McuWakeUpTime = RtcConvertTickToMs( stop - start );
    //   printf("new waktime:%ld\r\n",McuWakeUpTime);

        WakeUpTimeInitialized = true;
    }
}
// timeoutValue is in ms
static void RtcStartWakeUpAlarm( uint32_t timeoutValue )
{
    RtcCalendar_t now;
    RtcCalendar_t alarmTimer;
   // RTC_AlarmTypeDef alarmStructure;

    RTC_AlarmCmd(DISABLE);
    
    if( timeoutValue <= MIN_ALARM_DELAY )
    {
        timeoutValue = MIN_ALARM_DELAY;
    }

    // Load the RTC calendar
    now = RtcGetCalendar( );

    // Save the calendar into RtcCalendarContext to be able to calculate the elapsed time
    RtcCalendarContext = now;

    // timeoutValue is in ms
    alarmTimer = RtcComputeTimerTimeToAlarmTick( timeoutValue, now );
    
    RTC_AlarmStructInit(&RTC_AlarmStr);
    RTC_AlarmStr.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
    RTC_AlarmStr.RTC_AlarmDateWeekDay = alarmTimer.CalendarDate.RTC_Date;
    RTC_AlarmStr.RTC_AlarmTime.RTC_Hours   = alarmTimer.CalendarTime.RTC_Hours;
    RTC_AlarmStr.RTC_AlarmTime.RTC_Minutes = alarmTimer.CalendarTime.RTC_Minutes;
    RTC_AlarmStr.RTC_AlarmTime.RTC_Seconds = alarmTimer.CalendarTime.RTC_Seconds;
    RTC_AlarmStr.RTC_AlarmMask = RTC_AlarmMask_None;//RTC_AlarmMask_All;
    RTC_SetAlarm(RTC_Format_BIN, &RTC_AlarmStr);
    
    RTC_AlarmSubSecondConfig(alarmTimer.CalendarSubSeconds, RTC_AlarmSubSecondMask_None);
    RtcAlarmSubSeconds = alarmTimer.CalendarSubSeconds;
    
    RTC_ITConfig(RTC_IT_ALRA, ENABLE);
    RTC_AlarmCmd(ENABLE);
    
    enableInterrupts();
}


static RtcCalendar_t RtcComputeTimerTimeToAlarmTick( uint32_t timeCounter, RtcCalendar_t now )
{
    RtcCalendar_t calendar = now;

    uint32_t timeoutValue = 0;

    uint16_t milliseconds = 0;
    uint16_t seconds = now.CalendarTime.RTC_Seconds;
    uint16_t minutes = now.CalendarTime.RTC_Minutes;
    uint16_t hours = now.CalendarTime.RTC_Hours;
    uint16_t days = now.CalendarDate.RTC_Date;

    timeoutValue = timeCounter;

    milliseconds = PREDIV_S - now.CalendarSubSeconds;
    milliseconds += ( timeoutValue & PREDIV_S );

    /* convert timeout  to seconds */
    timeoutValue >>= N_PREDIV_S;  /* convert timeout  in seconds */

    // Convert milliseconds to RTC format and add to now
    while( timeoutValue >= SecondsInDay )
    {
        timeoutValue -= SecondsInDay;
        days++;
    }

    // Calculate hours
    while( timeoutValue >= SecondsInHour )
    {
        timeoutValue -= SecondsInHour;
        hours++;
    }

    // Calculate minutes
    while( timeoutValue >= SecondsInMinute )
    {
        timeoutValue -= SecondsInMinute;
        minutes++;
    }

    // Calculate seconds
    seconds += timeoutValue;

    // Correct for modulo
    while( milliseconds >= ( PREDIV_S + 1 ) )
    {
        milliseconds -= ( PREDIV_S + 1 );
        seconds++;
    }

    while( seconds >= SecondsInMinute )
    {
        seconds -= SecondsInMinute;
        minutes++;
    }

    while( minutes >= 60 )
    {
        minutes -= 60;
        hours++;
    }

    while( hours >= HoursInDay )
    {
        hours -= HoursInDay;
        days++;
    }

    if( ( now.CalendarDate.RTC_Year == 0 ) || ( now.CalendarDate.RTC_Year % 4 ) == 0 )
    {
        if( days > DaysInMonthLeapYear[now.CalendarDate.RTC_Month - 1] )
        {
            days = days % DaysInMonthLeapYear[now.CalendarDate.RTC_Month - 1];
            calendar.CalendarDate.RTC_Month++;
        }
    }
    else
    {
        if( days > DaysInMonth[now.CalendarDate.RTC_Month - 1] )
        {
            days = days % DaysInMonth[now.CalendarDate.RTC_Month - 1];
            calendar.CalendarDate.RTC_Month++;
        }
    }

    calendar.CalendarSubSeconds = PREDIV_S - milliseconds;
    calendar.CalendarTime.RTC_Seconds = seconds;
    calendar.CalendarTime.RTC_Minutes = minutes;
    calendar.CalendarTime.RTC_Hours = hours;
    calendar.CalendarDate.RTC_Date = days;

    return calendar;
}

//
// REMARK: Removed function static attribute in order to suppress
//         "#177-D function was declared but never referenced" warning.
// static RtcCalendar_t RtcConvertTimerTimeToCalendarTick( TimerTime_t timeCounter )
//
#if 0
RtcCalendar_t RtcConvertTimerTimeToCalendarTick( TimerTime_t timeCounter )
{
    RtcCalendar_t calendar;

    TimerTime_t timeoutValue = {0};

    uint16_t milliseconds = 0;
    uint16_t seconds = 0;
    uint16_t minutes = 0;
    uint16_t hours = 0;
    uint16_t days = 0;
    uint8_t months = 1; // Start at 1, month 0 does not exist
    uint16_t years = 0;

    memset(&calendar, 0, sizeof(RtcCalendar_t));
    timeoutValue = timeCounter;

    milliseconds += ( timeoutValue & PREDIV_S);

    /* convert timeout  to seconds */
    timeoutValue >>= N_PREDIV_S; // convert timeout  in seconds

    // Convert milliseconds to RTC format and add to now
    while( timeoutValue >= SecondsInDay )
    {
        timeoutValue -= SecondsInDay;
        days++;
    }

    // Calculate hours
    while( timeoutValue >= SecondsInHour )
    {
        timeoutValue -= SecondsInHour;
        hours++;
    }

    // Calculate minutes
    while( timeoutValue >= SecondsInMinute )
    {
        timeoutValue -= SecondsInMinute;
        minutes++;
    }

    // Calculate seconds
    seconds += timeoutValue;

    // Correct for modulo
    while( milliseconds >= ( PREDIV_S + 1 ) )
    {
        milliseconds -= ( PREDIV_S + 1 );
        seconds++;
    }

    while( seconds >= SecondsInMinute )
    {
        seconds -= SecondsInMinute;
        minutes++;
    }

    while( minutes >= 60 )
    {
        minutes -= 60;
        hours++;
    }

    while( hours >= HoursInDay )
    {
        hours -= HoursInDay;
        days++;
    }

    while( days > DaysInMonthLeapYear[months - 1] )
    {
        days -= DaysInMonthLeapYear[months - 1];
        months++;
    }

    calendar.CalendarSubSeconds = PREDIV_S - milliseconds;
    calendar.CalendarTime.RTC_Seconds = seconds;
    calendar.CalendarTime.RTC_Minutes = minutes;
    calendar.CalendarTime.RTC_Hours = hours;
    calendar.CalendarDate.RTC_Date = days;
    calendar.CalendarDate.RTC_Month = (RTC_Month_TypeDef)months;
    calendar.CalendarDate.RTC_Year = years; // on 32-bit, years will never go up

    return calendar;
}
#endif
static TimerTime_t RtcConvertCalendarTickToTimerTime( RtcCalendar_t *calendar )
{
    int16_t i = 0;
    TimerTime_t timeCounter = {0};
    RtcCalendar_t now;
    uint32_t timeCounterTemp = 0;
    // Passing a NULL pointer will compute from "now" else,
    // compute from the given calendar value
    if( calendar == NULL )
    {
        now = RtcGetCalendar();
        //printf("");
       //  printf("read:%03ldms\r\n",(uint32_t)((2048-now.CalendarSubSeconds&0x7ff)*1000/2048));
     //    printf("read:%03ldms\r\n",now.CalendarSubSeconds);    
    }
    else
    {
        now = *calendar;
    }
    // Years (calculation valid up to year 2099)
    for( i = 0; i < now.CalendarDate.RTC_Year ; i++ )
    {
        if( ( i == 0 ) || ( i % 4 ) == 0 )
        {
            timeCounterTemp += ( uint32_t )SecondsInLeapYear;
        }
        else
        {
            timeCounterTemp += ( uint32_t )SecondsInYear;
        }
    }
    // Months (calculation valid up to year 2099)*/
    if( ( now.CalendarDate.RTC_Year == 0 ) || ( now.CalendarDate.RTC_Year % 4 ) == 0 )
    {
        for( i = 0; i < ( now.CalendarDate.RTC_Month - 1 ); i++ )
        {
            timeCounterTemp += ( uint32_t )( DaysInMonthLeapYear[i] * SecondsInDay );
        }
    }
    else
    {
        for( i = 0;  i < ( now.CalendarDate.RTC_Month - 1 ); i++ )
        {
            timeCounterTemp += ( uint32_t )( DaysInMonth[i] * SecondsInDay );
        }
    }

    timeCounterTemp += ( uint32_t )( ( uint32_t )now.CalendarTime.RTC_Seconds +
                     ( ( uint32_t )now.CalendarTime.RTC_Minutes * SecondsInMinute ) +
                     ( ( uint32_t )now.CalendarTime.RTC_Hours * SecondsInHour ) +
                     ( ( uint32_t )( now.CalendarDate.RTC_Date * SecondsInDay ) ) );
   timeCounter.sec = timeCounterTemp;
   timeCounter.msec = (uint16_t)((2048-now.CalendarSubSeconds&0x7ff)*1000/2048);
    //timeCounter = ( timeCounterTemp << N_PREDIV_S ) + ( PREDIV_S - now.CalendarSubSeconds);
    //printf("read1: %lu\r\n",timeCounterTemp);
    //printf("read1: %lu\r\n",timeCounter);
    return ( timeCounter);
}


//获取当前的Tick值 ms级 
TimerTime_t RtcGetTickCurrentTime( void )
{
    int16_t i = 0;
    TimerTime_t timeCounter = {0};
    RtcCalendar_t now;
    uint32_t timeCounterTemp = 0;

    // Passing a NULL pointer will compute from "now" else,
    // compute from the given calendar value

        now = RtcGetCalendar( );



    // Years (calculation valid up to year 2099)
    for( i = 0; i < now.CalendarDate.RTC_Year ; i++ )
    {
        if( ( i == 0 ) || ( i % 4 ) == 0 )
        {
            timeCounterTemp += ( uint32_t )SecondsInLeapYear;
        }
        else
        {
            timeCounterTemp += ( uint32_t )SecondsInYear;
        }
    }

    // Months (calculation valid up to year 2099)*/
    if( ( now.CalendarDate.RTC_Year == 0 ) || ( now.CalendarDate.RTC_Year % 4 ) == 0 )
    {
        for( i = 0; i < ( now.CalendarDate.RTC_Month - 1 ); i++ )
        {
            timeCounterTemp += ( uint32_t )( DaysInMonthLeapYear[i] * SecondsInDay );
        }
    }
    else
    {
        for( i = 0;  i < ( now.CalendarDate.RTC_Month - 1 ); i++ )
        {
            timeCounterTemp += ( uint32_t )( DaysInMonth[i] * SecondsInDay );
        }
    }

    timeCounterTemp += ( uint32_t )( ( uint32_t )now.CalendarTime.RTC_Seconds +
                     ( ( uint32_t )now.CalendarTime.RTC_Minutes * SecondsInMinute ) +
                     ( ( uint32_t )now.CalendarTime.RTC_Hours * SecondsInHour ) +
                     ( ( uint32_t )( now.CalendarDate.RTC_Date * SecondsInDay ) ) );
   timeCounter.sec=timeCounterTemp;
   timeCounter.msec=(uint16_t)((2048-now.CalendarSubSeconds&0x7ff)*1000/2048);
  //  timeCounter. = ( timeCounterTemp << N_PREDIV_S ) + ( PREDIV_S - now.CalendarSubSeconds);

    return ( timeCounter);
}
uint32_t RtcConvertMsToTick( uint32_t timeoutValue )
{
    double retVal = 0;
    retVal = round( (( ( double )timeoutValue * CONV_DENOM ) / CONV_NUMER) );
    return( ( uint32_t )retVal );
}

uint32_t RtcConvertTickToMs( uint32_t timeoutValue )
{
    volatile double retVal = 0.0;
    retVal = round( (( ( double )timeoutValue * CONV_NUMER ) / CONV_DENOM) );
    return( ( uint32_t )retVal );
}

 RtcCalendar_t RtcGetCalendar( void )
{
    uint32_t first_read = 0;
    uint32_t second_read = 0;
    RtcCalendar_t now;


    /* Get the current Time*/
    RTC_GetDate(RTC_Format_BIN, &now.CalendarDate);
    RTC_GetTime(RTC_Format_BIN, &now.CalendarTime);
    first_read = RTC_GetSubSecond();
    RTC_GetDate(RTC_Format_BIN, &now.CalendarDate);
    RTC_GetTime(RTC_Format_BIN, &now.CalendarTime);
    second_read = RTC_GetSubSecond();

    // make sure it is correct due to asynchronous nature of RTC
    while( first_read != second_read )
    {
        first_read = second_read;
        RTC_GetDate(RTC_Format_BIN, &now.CalendarDate);
        RTC_GetTime(RTC_Format_BIN, &now.CalendarTime);
        second_read = RTC_GetSubSecond();
    }
    
    now.CalendarSubSeconds = second_read;
    return( now );
}

/*!
 * \brief RTC IRQ Handler of the RTC Alarm
 */

void RTC_Alarm_IRQHandler( void )
{
  	//printf("W\r\n");
#ifdef TEST 
    //GPIO_SetBits(GPIOE, GPIO_Pin_6);
#endif
    RTC_ITConfig(RTC_IT_ALRA, DISABLE);
    RTC_AlarmCmd(DISABLE);
    //disableInterrupts();
    RtcRecoverMcuStatus( );
    RtcComputeWakeUpTime( );//计算McuWakeUpTime   只计算一次
    BlockLowPowerDuringTask( false );
    TimerIrqHandler( );
	
}

extern TimerEvent_t   TESTTimer2;;
void ExitIO_IRQHandler( void )
{
    
#ifdef TEST 
    //GPIO_SetBits(GPIOE, GPIO_Pin_6);
  /*GPIO_SetBits(GPIOC, GPIO_Pin_2);
  for(int i=0;i<50;i++);
    GPIO_ResetBits(GPIOC, GPIO_Pin_2);
*/

#endif
#if 0
     RTC_AlarmCmd(DISABLE);
    //disableInterrupts();
    RtcRecoverMcuStatus( );//Եʼۯmcu
    RtcComputeWakeUpTime( );
    BlockLowPowerDuringTask( false );
    TimerIrqHandler( );
    TimerStop( &TESTTimer2 );
    TimerSetValue( &TESTTimer2, 10000 );//倒计时步入网的话继续休眠
	 TimerStart( &TESTTimer2 );
	
    DeviceState = DEVICE_STATE_ATCMD;
    
#endif
    //NextTx = true;//true
}
#if 0
void ExitIO_IRQHandler( void )
{
    
#ifdef TEST 
    //GPIO_SetBits(GPIOE, GPIO_Pin_6);
  /*GPIO_SetBits(GPIOC, GPIO_Pin_2);
  for(int i=0;i<50;i++);
    GPIO_ResetBits(GPIOC, GPIO_Pin_2);
*/

#endif

    RTC_AlarmCmd(DISABLE);
    //disableInterrupts();
    RtcRecoverMcuStatus( );
    RtcComputeWakeUpTime( );
    BlockLowPowerDuringTask( false );
    TimerIrqHandler( );
    
    DeviceState = DEVICE_STATE_SEND;
    NextTx = true;//true
}
#endif
void RtcProcess( void )
{
    // Not used on this platform.
}



//////////////////////////////////////////////////////////////////////////////////////
/********************************以下为新增代码**************************************/
//////////////////////////////////////////////////////////////////////////////////////


static u8 isLeapYear(u16 year)
{
    if(year%4==0) //必须能被4整除
    {
        if(year%100==0)
        {
            if(year%400==0)return 1;//如果以00结尾,还要能被400整除
            else return 0;
        } else return 1;
    } else return 0;
}



#define FOURYEARDAY (365+365+365+366)  //4年一个周期内的总天数（1970~2038不存在2100这类年份，故暂不优化）
#define TIMEZONE    8                  //北京时区调整 
const uint16_t dayPerYear[4] = {365, 365, 365, 366};
//转换标准TICK(S)   mstick(0-2047)为日历

static void covUnixTimeStp2Beijing(uint32_t unixTime, rtc_time_t *tempBeijing)
{
    uint32_t totleDaynum=0, totleSecNum=0;
    uint16_t remainDayofYear=0;
    uint8_t *pr, tempYear=0;
    
 
    totleDaynum = unixTime/86400; //总天数(注意加括号)  24*60*60 警告
    totleSecNum = unixTime%86400; //当天剩余的秒速
 
    memset(tempBeijing, 0x00, sizeof(rtc_time_t));
    //1.计算哪一年
    tempBeijing->ui8Year = 70+ (totleDaynum/FOURYEARDAY)*4;//不加1970
    remainDayofYear = totleDaynum%FOURYEARDAY+1;
    while(remainDayofYear >= dayPerYear[tempYear]){
        remainDayofYear -= dayPerYear[tempYear];
        tempBeijing->ui8Year++;
        tempYear++;
    }
    
    //2.计算哪一月的哪一天
    pr = isLeapYear(tempBeijing->ui8Year)?(uint8_t*)DaysInMonthLeapYear:(uint8_t*)DaysInMonth;
    while(remainDayofYear > *(pr+tempBeijing->ui8Month))
    {
		remainDayofYear -= *(pr+tempBeijing->ui8Month);
        tempBeijing->ui8Month++;
    }
    tempBeijing->ui8Month++; //month
    tempBeijing->ui8DayOfMonth = remainDayofYear; //day
  
    //3.计算当天时间
    tempBeijing->ui8Hour = totleSecNum/3600;
    tempBeijing->ui8Minute = (totleSecNum%3600)/60; //error：变量搞错
    tempBeijing->ui8Second = (totleSecNum%3600)%60;
 
    //4.时区调整
    tempBeijing->ui8Hour +=TIMEZONE; 
    if(tempBeijing->ui8Hour>23){
        tempBeijing->ui8Hour -= 24;
        tempBeijing->ui8DayOfMonth++;
    }
}
//输入 日历  和ms的tick  写完要等待写稳定才能读出
static void RtcWriteCalendar( RtcCalendar_t CalendarTime,uint16_t MsCounter1 )//
{
#if 1
    uint16_t MsCounter=MsCounter1&0x7ff;
     // MsCounter+=4;
    uint8_t sec=0;
    RTC_SetSubSecond(MsCounter);
    if(MsCounter<=10)//10个tick  加入延时 确保过了临界点
    {
        sec=1;//自加1S 先设置的ms 后设置的s 设置完ms后 mstick倒计时完  但是s并没有增加 
        delay_ms(10);//延时屏蔽临界点
    }
   // printf("b3:%d\r\n",RTC_GetSubSecond()&0x7ff);
    RTC_InitStr.RTC_HourFormat = RTC_HourFormat_24;
    RTC_InitStr.RTC_AsynchPrediv = PREDIV_A;
    RTC_InitStr.RTC_SynchPrediv = PREDIV_S;
    RTC_Init(&RTC_InitStr);

    // Set Date: Friday 1st of January 2000
    RTC_DateStructInit(&RTC_DateStr);
    //RTC_DateStr.RTC_WeekDay = RTC_Weekday_Saturday;//不设置
    
    RTC_DateStr.RTC_Date  = CalendarTime.CalendarDate.RTC_Date;
    RTC_DateStr.RTC_Month = CalendarTime.CalendarDate.RTC_Month;
    RTC_DateStr.RTC_Year  = CalendarTime.CalendarDate.RTC_Year;
    RTC_SetDate(RTC_Format_BIN, &RTC_DateStr);
    
    RTC_TimeStructInit(&RTC_TimeStr);
    RTC_TimeStr.RTC_Hours   = CalendarTime.CalendarTime.RTC_Hours;
    RTC_TimeStr.RTC_Minutes = CalendarTime.CalendarTime.RTC_Minutes;
    RTC_TimeStr.RTC_Seconds = CalendarTime.CalendarTime.RTC_Seconds+sec;
    RTC_SetTime(RTC_Format_BIN, &RTC_TimeStr);

    // Enable Direct Read of the calendar registers (not through Shadow registers)
    RTC_BypassShadowCmd(ENABLE);
#endif

    //for(int i=0;i<20000;i++);
}

//设置tick  MsCounter 需要ms转tick
void RtcWriteTick(uint32_t unixTime,uint16_t MsCounter)
{
  
  rtc_time_t  tempBeijing_t;
  RtcCalendar_t CalendarTime_t;
  covUnixTimeStp2Beijing( unixTime, &tempBeijing_t);
  CalendarTime_t.CalendarDate.RTC_Year   =tempBeijing_t.ui8Year%100;
  CalendarTime_t.CalendarDate.RTC_Month  =(RTC_Month_TypeDef)tempBeijing_t.ui8Month;
  CalendarTime_t.CalendarDate.RTC_Date   =tempBeijing_t.ui8DayOfMonth;
  CalendarTime_t.CalendarTime.RTC_Hours  =tempBeijing_t.ui8Hour;
  CalendarTime_t.CalendarTime.RTC_Minutes=tempBeijing_t.ui8Minute;
  CalendarTime_t.CalendarTime.RTC_Seconds=tempBeijing_t.ui8Second;
  RtcWriteCalendar(CalendarTime_t, MsCounter );
  
  
}
//读取 RtcCalendar_t RtcGetCalendar( void )
