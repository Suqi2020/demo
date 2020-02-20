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
#include "stdio.h"
#include "stm8l15x_rtc.h"
#include "board.h"
#include "rtc-board.h"
#include "timer.h"
#include "rtc-board.h"


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
uint32_t TimerGetValue( void );

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
     //   printf("test1 val %lu\r\n",obj->ReloadValue);
        TimerInsertNewHeadTimer( obj, obj->Timestamp );//�պ���ͨ�����ô˴����ó�ʱʱ��
        
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

        if( obj->Timestamp < remainingTime )//����²����ʱ��ֵ С��Ҫִ�е�ͷ����ʱ��ֵ
        {
        //  printf("test2 %lu  %lu\r\n",obj->Timestamp,remainingTime);
            TimerInsertNewHeadTimer( obj, remainingTime );//���ó�ʱʱ��
        }
        else
        {
      //    printf("test3\r\n");
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
        cur->Timestamp = remainingTime - obj->Timestamp;//����ʣ�µ�ֵ��ȥ�����ֵ ���ǲ��붨ʱ��ִ���� Ҫִ����һ����ʱ����ֵ
        cur->IsRunning = false;
    }

    obj->Next = cur;
    obj->IsRunning = true;
    TimerListHead = obj;//��Ҫ�����ֵ�ŵ���ͷ
  // printf("new val :%d",TimerListHead->Timestamp);
    TimerSetTimeout( TimerListHead );//
   // printf("Rtc set timeout1\r\n");
}
//���ã�rtc����alarm��ʱʱ�䵽���� TimerIrqHandler
//ͨ��rtc����alarm��ʱ��������ʱʱ�䵽�Ļص�����
//ͬʱ������һ�γ�ʱʱ��
void TimerIrqHandler( void )
{
    uint32_t elapsedTime = 0;

    // Early out when TimerListHead is null to prevent null pointer
    if ( TimerListHead == NULL )
    {
        return;
    }

    elapsedTime = TimerGetValue( );//��ȡ��������ʱ��

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
           // printf("time irq callback\r\n");//suqi
            elapsedTimer->Callback( );//ִ�лص�����
        }
    }

    // start the next TimerListHead if it exists
    if( TimerListHead != NULL )
    {
        if( TimerListHead->IsRunning != true )
        {
            TimerListHead->IsRunning = true;
            TimerSetTimeout( TimerListHead );//������һ�γ�ʱʱ��
            //printf("Rtc set timeout2\r\n");
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
               // printf("Rtc set timeout3\r\n");
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
   // printf("test0 val %lu\r\n",obj->ReloadValue);
}

uint32_t TimerGetValue( void )
{
    return RtcGetElapsedAlarmTime( );
}

TimerTime_t TimerGetCurrentTime( void )
{
    return RtcGetTimerValue( );
}
//得到值 S或者ms级TICK  u32位  读取ms不能用此函数
/*TimerTime_t TimerGetCurrentTime(  Tick_s_ms_enum tick )
{
    return RtcGetTimerValue( tick );
}*/
//获取ms值 
/*
uint16_t TimerGetCurrentMsTime()
{
  uint16_t Read_Tick=0x7FF-(RTC_GetSubSecond()&0x7FF);
  
  
  return RtcConvertTickToMs(Read_Tick);
}*/
uint32_t TimerGetElapsedTime( TimerTime_t savedTime )
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
   // printf("timeoutset1:%lu \r\n",obj->Timestamp);
   // obj->Timestamp = RtcGetAdjustedTimeoutValue( obj->Timestamp );//���ڶ�ȡ�µ�tick ȥ���Ÿ����_ suqi
    //printf("timeoutset2:%lu \r\n",obj->Timestamp);
    RtcSetTimeout( obj->Timestamp );
   // printf("Rtc set timeout\r\n");
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



 



