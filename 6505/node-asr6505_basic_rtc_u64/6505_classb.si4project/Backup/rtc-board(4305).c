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
#include "stm8l15x_pwr.h"
#include "board.h"


#include "utilities.h"

#include "timer.h"
#include "gpio.h"
#include "radio.h"
#include "rtc-board.h"

/*!
 * RTC Time base in ms
 */
#define RTC_ALARM_TICK_PER_MS                       0x7FF           //  2047 > number of sub-second ticks per second



/* RTC Time base in us */
#define USEC_NUMBER               1000000
#define MSEC_NUMBER               ( USEC_NUMBER / 1000 )
#define RTC_ALARM_TIME_BASE       ( USEC_NUMBER >> N_PREDIV_S )

#define COMMON_FACTOR             3
#define CONV_NUMER                ( MSEC_NUMBER >> COMMON_FACTOR )//1000>>3  =125
#define CONV_DENOM                ( 1 << ( N_PREDIV_S - COMMON_FACTOR ) ) //256

#define MIN_ALARM_DELAY                             6
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
static const uint8_t DaysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*!
 * Number of days in each month on a leap year
 */
static const uint8_t DaysInMonthLeapYear[] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

   
const uint16_t dayPerYear[4] = {365, 365, 365, 366};
/*!
 * Flag used to indicates a the MCU has waken-up from an external IRQ
 */
volatile bool NonScheduledWakeUp = false;


/*!
 * \brief RTC Handler
 */
static RTC_InitTypeDef   RTC_InitStr;
static RTC_TimeTypeDef   RTC_TimeStr;
static RTC_DateTypeDef   RTC_DateStr;
static RTC_AlarmTypeDef  RTC_AlarmStr;



    

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


/*!
 * \brief Converts a RtcCalendar_t value into TimerTime_t value
 *
 * \param[IN/OUT] calendar Calendar value to be converted
 *                         [NULL: compute from "now",
 *                          Others: compute from given calendar value]
 * \retval timerTime New TimerTime_t value
 */
//static TimerTime_t RtcConvertCalendarTickToTimerTime( RtcCalendar_t *calendar ,Tick_s_ms_enum tick);

//static TimerTime_t RtcConvertMsToTick( TimerTime_t timeoutValue );

static TimerTime_t RtcConvertTickToMs( TimerTime_t timeoutValue );

/*!
 * \brief Converts a TimerTime_t value into a value for the RTC Alarm
 *
 * \param[IN] timeCounter Value in ms to convert into a calendar alarm date
 * \param[IN] now Current RTC calendar context
 * \retval rtcCalendar Value for the RTC Alarm
 */
static RtcCalendar_t RtcComputeTimerTimeToAlarmTick( TimerTime_t timeCounter, RtcCalendar_t now );

/*!
 * \brief Returns the internal RTC Calendar and check for RTC overflow
 *
 * \retval calendar RTC calendar
 */
static RtcCalendar_t RtcGetCalendar( void );

static double round(double r)
{
	return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}



TimerTime_t RtcConvertMsToTick( TimerTime_t timeoutValue )
{
    double retVal = 0;
    retVal = round( (( ( double )timeoutValue * CONV_DENOM ) / CONV_NUMER) );
    return( ( TimerTime_t )retVal );
}

TimerTime_t RtcConvertTickToMs( TimerTime_t timeoutValue )
{
    double retVal = 0.0;
    retVal = round( (( ( double )timeoutValue * CONV_NUMER ) / CONV_DENOM) );
    return( ( TimerTime_t )retVal );
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
    RtcStartWakeUpAlarm( RtcConvertMsToTick( timeout ) );//// ≤Œ ˝¥¯»Îms
}

 TimerTime_t RtcGetAdjustedTimeoutValue( uint32_t timeout )
{
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

TimerTime_t RtcGetTimerValue( Tick_s_ms_enum tick )
{
    TimerTime_t retVal = 0;
	
    retVal = RtcConvertCalendarTickToTimerTime( NULL,tick );//s  ms tick
    if(tick==TICK_MS)
   	    return( RtcConvertTickToMs( retVal ) );//0-2047 ËΩ¨Êç¢Êàê0-999
    else 
		return retVal;
}

TimerTime_t RtcGetElapsedAlarmTime( void )
{
    TimerTime_t retVal = 0;
    TimerTime_t currentTime = 0;
    TimerTime_t contextTime = 0;

    currentTime = RtcConvertCalendarTickToTimerTime( NULL,TICK_MS );
    contextTime = RtcConvertCalendarTickToTimerTime( &RtcCalendarContext,TICK_MS );

    if( currentTime < contextTime )
    {
        retVal = ( currentTime + ( 0xFFFFFFFF - contextTime ) );
    }
    else
    {
        retVal = ( currentTime - contextTime );
    }
    return( RtcConvertTickToMs( retVal ) );
}

TimerTime_t RtcComputeFutureEventTime( TimerTime_t futureEventInTime )
{
    return( RtcGetTimerValue( TICK_MS) + futureEventInTime );
}

TimerTime_t RtcComputeElapsedTime( TimerTime_t eventInTime )
{
    TimerTime_t elapsedTime = 0;

    // Needed at boot, cannot compute with 0 or elapsed time will be equal to current time
    if( eventInTime == 0 )
    {
        return 0;
    }
    // first get the current value of the timer in tick
    elapsedTime = RtcConvertCalendarTickToTimerTime( NULL,TICK_MS );
    // convert into ms
    elapsedTime = RtcConvertTickToMs( elapsedTime );

    // compare "eventInTime" with "elapsedTime" while watching for roll over due to 32-bit
    if( elapsedTime < eventInTime ) // // roll over of the counter
    {   // due to convertion tick to ms, roll over value is 0x7D000000 (0x7D000000 * 2.048 = 0xFFFFFFFF)
        return( elapsedTime + ( 0x7D000000 - eventInTime ) );
    }
    else
    {
        return( elapsedTime - eventInTime );
    }
}

void BlockLowPowerDuringTask ( bool status )
{  
    if( status == true )
    {
        RtcRecoverMcuStatus( );
    }   
    LowPowerDisableDuringTask = status;   
}
//Ω¯»ÎACTIVEHALTƒ£ Ω RTCªΩ–—
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


static RtcCalendar_t RtcComputeTimerTimeToAlarmTick( TimerTime_t timeCounter, RtcCalendar_t now )
{
    RtcCalendar_t calendar = now;

    TimerTime_t timeoutValue = 0;

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
#if 1
RtcCalendar_t RtcConvertTimerTimeToCalendarTick( TimerTime_t timeCounter )
{
    RtcCalendar_t calendar;

    TimerTime_t timeoutValue = 0;

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

    calendar.CalendarSubSeconds = PREDIV_S - milliseconds;//ËåÉÂõ¥0-2047
    calendar.CalendarTime.RTC_Seconds = seconds;
    calendar.CalendarTime.RTC_Minutes = minutes;
    calendar.CalendarTime.RTC_Hours = hours;
    calendar.CalendarDate.RTC_Date = days;
    calendar.CalendarDate.RTC_Month = (RTC_Month_TypeDef)months;
    calendar.CalendarDate.RTC_Year = years; // on 32-bit, years will never go up

    return calendar;
}
#endif
//tick ◊™ƒÍ‘¬»’
#if 0

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
rtc_time_t tempBeijing;





uint8_t isLeapYear(uint16_t year)
{
	uint8_t res=0;
	
	if(year%4 == 0) // ƒ‹πª±ª4’˚≥˝ 
	{
		if((year%100 == 0) && (year%400 != 0))	//ƒ‹πª±ª100’˚≥˝£¨µ´ «≤ªƒ‹πª±ª400’˚≥˝ 
		{
			res = 0;
		}
		else
		{
			res =1;
		}
	}
	return res;
}

#define FOURYEARDAY (365+365+365+366)  //4ƒÍ“ª∏ˆ÷‹∆⁄ƒ⁄µƒ◊‹ÃÏ ˝£®1970~2038≤ª¥Ê‘⁄2100’‚¿‡ƒÍ∑›£¨π ‘›≤ª”≈ªØ£©
#define TIMEZONE    8 
RtcCalendar_t RtcConvertTimerTimeToCalendarTick( TimerTime_t timeCounter )
{
    RtcCalendar_t calendar;

   uint32_t totleDaynum=0, totleSecNum=0;
    uint16_t remainDayofYear, tempDay=0,milliseconds=0;
    uint8_t *pr, tempYear=0;
    uint32_t unixTime =timeCounter;
 
    
    
     milliseconds += ( unixTime & PREDIV_S);

    /* convert timeout  to seconds */
    //unixTime >>= N_PREDIV_S; // convert timeout  in seconds

    
    
    totleDaynum = unixTime/(SecondsInDay); //◊‹ÃÏ ˝(◊¢“‚º”¿®∫≈)
    totleSecNum = unixTime%(SecondsInDay); //µ±ÃÏ £”‡µƒ√ÎÀŸ
 
    memset(&tempBeijing, 0x00, sizeof(rtc_time_t));
    //1.º∆À„ƒƒ“ªƒÍ
    tempBeijing.ui8Year = 1970 + (totleDaynum/FOURYEARDAY)*4;
    remainDayofYear = totleDaynum%FOURYEARDAY+1;
    while(remainDayofYear >= dayPerYear[tempYear]){
        remainDayofYear -= dayPerYear[tempYear];
        tempBeijing.ui8Year++;
        tempYear++;
    }
    
    //2.º∆À„ƒƒ“ª‘¬µƒƒƒ“ªÃÏ
    pr = isLeapYear(tempBeijing.ui8Year)?(uint8_t *)DaysInMonthLeapYear:(uint8_t *)DaysInMonth;
    while(remainDayofYear > *(pr+tempBeijing.ui8Month))
    {
		remainDayofYear -= *(pr+tempBeijing.ui8Month);
        tempBeijing.ui8Month++;
    }
    tempBeijing.ui8Month++; //month
    tempBeijing.ui8DayOfMonth = remainDayofYear; //day
  
    //3.º∆À„µ±ÃÏ ±º‰
    tempBeijing.ui8Hour = totleSecNum/3600;
    tempBeijing.ui8Minute = (totleSecNum%3600)/60; //error£∫±‰¡ø∏„¥Ì
    tempBeijing.ui8Second = (totleSecNum%3600)%60;
 
    //4. ±«¯µ˜’˚
    tempBeijing.ui8Hour +=TIMEZONE; 
    if(tempBeijing.ui8Hour>23){
        tempBeijing.ui8Hour -= 24;
        tempBeijing.ui8DayOfMonth++;
    }


    calendar.CalendarSubSeconds = PREDIV_S - milliseconds;
    calendar.CalendarTime.RTC_Seconds = tempBeijing.ui8Second;
    calendar.CalendarTime.RTC_Minutes = tempBeijing.ui8Minute;
    calendar.CalendarTime.RTC_Hours = tempBeijing.ui8Hour;
    calendar.CalendarDate.RTC_Date = tempBeijing.ui8DayOfMonth;
    calendar.CalendarDate.RTC_Month = (RTC_Month_TypeDef)tempBeijing.ui8Month;
    calendar.CalendarDate.RTC_Year = 0; // on 32-bit, years will never go up

    return calendar;
}
#endif

//suqi  ms Âíå s ÁöÑtickËé∑Âèñ
 TimerTime_t RtcConvertCalendarTickToTimerTime( RtcCalendar_t *calendar,Tick_s_ms_enum tick )
{
    int16_t i = 0;
    TimerTime_t timeCounter = 0;
    RtcCalendar_t now;
    uint32_t timeCounterTemp = 0;

    // Passing a NULL pointer will compute from "now" else,
    // compute from the given calendar value
    if( calendar == NULL )
    {
        now = RtcGetCalendar( );
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

    timeCounter = ( timeCounterTemp << N_PREDIV_S ) + ( PREDIV_S - now.CalendarSubSeconds);
  if(tick==TICK_S)
    return (timeCounterTemp); //suqi
  else 
    return ( timeCounter );//suqi
}
/*
TimerTime_t RtcConvertMsToTick( TimerTime_t timeoutValue )
{
    double retVal = 0;
    retVal = round( (( ( double )timeoutValue * CONV_DENOM ) / CONV_NUMER) );//Àƒ…·ŒÂ»Î
    return( ( TimerTime_t )retVal );
}

TimerTime_t RtcConvertTickToMs( TimerTime_t timeoutValue )
{
    double retVal = 0.0;
    retVal = round( (( ( double )timeoutValue * CONV_NUMER ) / CONV_DENOM) );
    return( ( TimerTime_t )retVal );
}

*/
//ƒÍ‘¬»’ ±∑÷√Î ∫¡√Î
static RtcCalendar_t RtcGetCalendar( void )
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
    GPIO_SetBits(GPIOE, GPIO_Pin_6);
#endif
    RTC_ITConfig(RTC_IT_ALRA, DISABLE);
    RTC_AlarmCmd(DISABLE);
    //disableInterrupts();
    RtcRecoverMcuStatus( );
    RtcComputeWakeUpTime( );
    BlockLowPowerDuringTask( false );
    TimerIrqHandler( );
	
}

extern TimerEvent_t TxNextPacketTimer;
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
    RtcRecoverMcuStatus( );//≥ı ºªØmcu
    RtcComputeWakeUpTime( );
    BlockLowPowerDuringTask( false );
    TimerIrqHandler( );
    TimerSetValue( &TxNextPacketTimer, 10000 );//ªΩ–—∫Û÷ÿ–¬…Ë÷√—≠ª∑ ±º‰ 
	 TimerStart( &TxNextPacketTimer );
	
    DeviceState = DEVICE_STATE_ATCMD;
    //NextTx = true;//true
}

TimerTime_t RtcConvertMsToTick( TimerTime_t timeoutValue );

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
