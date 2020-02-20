/*!
 * \file      timer.c
 *
 * \brief     Timer objects and scheduling management implementation
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
#include "stm8l15x_rtc.h"
#include "board.h"
#include "rtc-board.h"
#include "timer.h"
//#include "time.h"


/*!
 * This flag is used to loop through the main several times in order to be sure
 * that all pending events have been processed.
 */
volatile uint8_t HasLoopedThroughMain = 0;

/*!
 * Timers list head pointer
 */
static TimerEvent_t *TimerListHead = NULL;

/*!
 * \brief Adds or replace the head timer of the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * \param [IN]  obj Timer object to be become the new head
 * \param [IN]  remainingTime Remaining time of the previous head to be replaced
 */
static void TimerInsertNewHeadTimer( TimerEvent_t *obj, uint32_t remainingTime );

/*!
 * \brief Adds a timer to the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * \param [IN]  obj Timer object to be added to the list
 * \param [IN]  remainingTime Remaining time of the running head after which the object may be added
 */
static void TimerInsertTimer( TimerEvent_t *obj, uint32_t remainingTime );

/*!
 * \brief Sets a timeout with the duration "timestamp"
 *
 * \param [IN] timestamp Delay duration
 */
static void TimerSetTimeout( TimerEvent_t *obj );

/*!
 * \brief Check if the Object to be added is not already in the list
 *
 * \param [IN] timestamp Delay duration
 * \retval true (the object is already in the list) or false
 */
static bool TimerExists( TimerEvent_t *obj );

/*!
 * \brief Read the timer value of the currently running timer
 *
 * \retval value current timer value
 */
TimerTime_t TimerGetValue( void );

void TimerInit( TimerEvent_t *obj, void ( *callback )( void ) )
{
    obj->Timestamp = 0;
    obj->ReloadValue = 0;
    obj->IsRunning = false;
    obj->Callback = callback;
    obj->Next = NULL;
}

void TimerStart( TimerEvent_t *obj )
{
    uint32_t elapsedTime = 0;
    uint32_t remainingTime = 0;

    BoardDisableIrq( );

    if( ( obj == NULL ) || ( TimerExists( obj ) == true ) )
    {
        BoardEnableIrq( );
        return;
    }

    obj->Timestamp = obj->ReloadValue;
    obj->IsRunning = false;

    if( TimerListHead == NULL )
    {
        TimerInsertNewHeadTimer( obj, obj->Timestamp );
    }
    else
    {
        if( TimerListHead->IsRunning == true )
        {
            elapsedTime = TimerGetValue( );
            if( elapsedTime > TimerListHead->Timestamp )
            {
                elapsedTime = TimerListHead->Timestamp; // security but should never occur
            }
            remainingTime = TimerListHead->Timestamp - elapsedTime;
        }
        else
        {
            remainingTime = TimerListHead->Timestamp;
        }

        if( obj->Timestamp < remainingTime )
        {
            TimerInsertNewHeadTimer( obj, remainingTime );
        }
        else
        {
             TimerInsertTimer( obj, remainingTime );
        }
    }
    BoardEnableIrq( );
}

static void TimerInsertTimer( TimerEvent_t *obj, uint32_t remainingTime )
{
    uint32_t aggregatedTimestamp = 0;      // hold the sum of timestamps
    uint32_t aggregatedTimestampNext = 0;  // hold the sum of timestamps up to the next event

    TimerEvent_t* prev = TimerListHead;
    TimerEvent_t* cur = TimerListHead->Next;

    if( cur == NULL )
    { // obj comes just after the head
        obj->Timestamp -= remainingTime;
        prev->Next = obj;
        obj->Next = NULL;
    }
    else
    {
        aggregatedTimestamp = remainingTime;
        aggregatedTimestampNext = remainingTime + cur->Timestamp;

        while( prev != NULL )
        {
            if( aggregatedTimestampNext > obj->Timestamp )
            {
                obj->Timestamp -= aggregatedTimestamp;
                if( cur != NULL )
                {
                    cur->Timestamp -= obj->Timestamp;
                }
                prev->Next = obj;
                obj->Next = cur;
                break;
            }
            else
            {
                prev = cur;
                cur = cur->Next;
                if( cur == NULL )
                { // obj comes at the end of the list
                    aggregatedTimestamp = aggregatedTimestampNext;
                    obj->Timestamp -= aggregatedTimestamp;
                    prev->Next = obj;
                    obj->Next = NULL;
                    break;
                }
                else
                {
                    aggregatedTimestamp = aggregatedTimestampNext;
                    aggregatedTimestampNext = aggregatedTimestampNext + cur->Timestamp;
                }
            }
        }
    }
}

static void TimerInsertNewHeadTimer( TimerEvent_t *obj, uint32_t remainingTime )
{
    TimerEvent_t* cur = TimerListHead;

    if( cur != NULL )
    {
        cur->Timestamp = remainingTime - obj->Timestamp;
        cur->IsRunning = false;
    }

    obj->Next = cur;
    obj->IsRunning = true;
    TimerListHead = obj;
    TimerSetTimeout( TimerListHead );
}

void TimerIrqHandler( void )
{
    uint32_t elapsedTime = 0;

    // Early out when TimerListHead is null to prevent null pointer
    if ( TimerListHead == NULL )
    {
        return;
    }

    elapsedTime = TimerGetValue( );

    if( elapsedTime >= TimerListHead->Timestamp )
    {
        TimerListHead->Timestamp = 0;
    }
    else
    {
        TimerListHead->Timestamp -= elapsedTime;
    }

    TimerListHead->IsRunning = false;

    while( ( TimerListHead != NULL ) && ( TimerListHead->Timestamp == 0 ) )
    {
        TimerEvent_t* elapsedTimer = TimerListHead;
        TimerListHead = TimerListHead->Next;

        if( elapsedTimer->Callback != NULL )
        {
            elapsedTimer->Callback( );
        }
    }

    // start the next TimerListHead if it exists
    if( TimerListHead != NULL )
    {
        if( TimerListHead->IsRunning != true )
        {
            TimerListHead->IsRunning = true;
            TimerSetTimeout( TimerListHead );
        }
    }
}

void TimerStop( TimerEvent_t *obj )
{
    uint32_t elapsedTime;
    uint32_t remainingTime;
    TimerEvent_t* prev;
    TimerEvent_t* cur;
    
    
    BoardDisableIrq( );

    elapsedTime = 0;
    remainingTime = 0;

    prev = TimerListHead;
    cur = TimerListHead;

    // List is empty or the Obj to stop does not exist
    if( ( TimerListHead == NULL ) || ( obj == NULL ) )
    {
        BoardEnableIrq( );
        return;
    }

    if( TimerListHead == obj ) // Stop the Head
    {
        if( TimerListHead->IsRunning == true ) // The head is already running
        {
            elapsedTime = TimerGetValue( );
            if( elapsedTime > obj->Timestamp )
            {
                elapsedTime = obj->Timestamp;
            }

            remainingTime = obj->Timestamp - elapsedTime;

            TimerListHead->IsRunning = false;
            if( TimerListHead->Next != NULL )
            {
                TimerListHead = TimerListHead->Next;
                TimerListHead->Timestamp += remainingTime;
                TimerListHead->IsRunning = true;
                TimerSetTimeout( TimerListHead );
            }
            else
            {
                TimerListHead = NULL;
            }
        }
        else // Stop the head before it is started
        {
            if( TimerListHead->Next != NULL )
            {
                remainingTime = obj->Timestamp;
                TimerListHead = TimerListHead->Next;
                TimerListHead->Timestamp += remainingTime;
            }
            else
            {
                TimerListHead = NULL;
            }
        }
    }
    else // Stop an object within the list
    {
        remainingTime = obj->Timestamp;

        while( cur != NULL )
        {
            if( cur == obj )
            {
                if( cur->Next != NULL )
                {
                    cur = cur->Next;
                    prev->Next = cur;
                    cur->Timestamp += remainingTime;
                }
                else
                {
                    cur = NULL;
                    prev->Next = cur;
                }
                break;
            }
            else
            {
                prev = cur;
                cur = cur->Next;
            }
        }
    }
    BoardEnableIrq( );
}

static bool TimerExists( TimerEvent_t *obj )
{
    TimerEvent_t* cur = TimerListHead;

    while( cur != NULL )
    {
        if( cur == obj )
        {
            return true;
        }
        cur = cur->Next;
    }
    return false;
}

void TimerReset( TimerEvent_t *obj )
{
    TimerStop( obj );
    TimerStart( obj );
}

void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
    TimerStop( obj );
    obj->Timestamp = value;
    obj->ReloadValue = value;
}

TimerTime_t TimerGetValue( void )
{
    return RtcGetElapsedAlarmTime( );
}

TimerTime_t TimerGetCurrentTime( Tick_s_ms_enum tick )
{
    return RtcGetTimerValue( tick );
}

TimerTime_t TimerGetElapsedTime( TimerTime_t savedTime )
{
    return RtcComputeElapsedTime( savedTime );
}

TimerTime_t TimerGetFutureTime( TimerTime_t eventInFuture )
{
    return RtcComputeFutureEventTime( eventInFuture );
}

static void TimerSetTimeout( TimerEvent_t *obj )
{
    HasLoopedThroughMain = 0;
    obj->Timestamp = RtcGetAdjustedTimeoutValue( obj->Timestamp );
    RtcSetTimeout( obj->Timestamp );
}

void TimerLowPowerHandler( void )
{
    if( ( TimerListHead != NULL ) && ( TimerListHead->IsRunning == true ) )
    {
        if( HasLoopedThroughMain < 5 )
        {
            HasLoopedThroughMain++;
        }
        else
        {
            HasLoopedThroughMain = 0;
            if( GetBoardPowerSource( ) == BATTERY_POWER )
            {
                RtcEnterLowPowerStopMode( );
            }
        }
    }
}

void TimerProcess( void )
{
    RtcProcess( );
}



 



#ifndef ADD_NEWCODE

static RTC_InitTypeDef   RTC_InitStr;
static RTC_TimeTypeDef   RTC_TimeStr;
static RTC_DateTypeDef   RTC_DateStr;
//new code  
 TimerTime_t sys_time;
     TimerTime_t timeget;

void TimerSetSysTime(TimerSysTime_t sysTime)
{
    /*stm.tm_year=gRtc.date.year-1900; //年
    stm.tm_mon =gRtc.date.month-1;   //月
    stm.tm_mday=gRtc.date.day;  //è?
    stm.tm_hour=gRtc.time.hour;  //ê±??
    stm.tm_min =gRtc.time.minute;  //·?
    stm.tm_sec =gRtc.second;  //??*/
  ///设置除ms以外的时间戳配置 
    RtcCalendar_t stm;

   
   // sys_time = sysTime.Seconds ;

    

   
    sys_time = ( sysTime.Seconds << N_PREDIV_S ) + ( PREDIV_S - sysTime.SubSeconds); //s  ms 合并转换位tick
    stm = RtcConvertTimerTimeToCalendarTick(sys_time);



	
    RTC_InitStr.RTC_HourFormat = RTC_HourFormat_24;   

    RTC_InitStr.RTC_AsynchPrediv = PREDIV_A;
    RTC_InitStr.RTC_SynchPrediv = PREDIV_S;
    RTC_Init(&RTC_InitStr);

    // Set Date: Friday 1st of January 2000
    RTC_DateStructInit(&RTC_DateStr);
    RTC_DateStr.RTC_WeekDay = stm.CalendarDate.RTC_WeekDay;   
    RTC_DateStr.RTC_Date = stm.CalendarDate.RTC_Date;
    RTC_DateStr.RTC_Month = stm.CalendarDate.RTC_Month;
    RTC_DateStr.RTC_Year = stm.CalendarDate.RTC_Year;
    RTC_SetDate(RTC_Format_BIN, &RTC_DateStr);

    RTC_TimeStructInit(&RTC_TimeStr);
    RTC_TimeStr.RTC_Hours   = stm.CalendarTime.RTC_Hours;
    RTC_TimeStr.RTC_Minutes = stm.CalendarTime.RTC_Minutes;
    RTC_TimeStr.RTC_Seconds = stm.CalendarTime.RTC_Seconds;

    RTC_SetTime(RTC_Format_BIN, &RTC_TimeStr);


    // Enable Direct Read of the calendar registers (not through Shadow registers)
    RTC_BypassShadowCmd(ENABLE);
	RTC_SetSubSecond(stm.CalendarSubSeconds);//?????
//设置ms时间戳配置
    
    
 
       /*timeget= RtcConvertCalendarTickToTimerTime( NULL,Tick_ms );  //读取mstick测试 
          timeget= RtcConvertCalendarTickToTimerTime( NULL,Tick_ms );  //读取测试 
             timeget= RtcConvertCalendarTickToTimerTime( NULL,Tick_ms );  //读取测试 
                timeget= RtcConvertCalendarTickToTimerTime( NULL,Tick_ms );  //读取测试 
                   timeget= RtcConvertCalendarTickToTimerTime( NULL,Tick_ms );  //读取测试    
                   timeget= RtcConvertCalendarTickToTimerTime( NULL,Tick_ms );  //读取测试 */

}

#endif
