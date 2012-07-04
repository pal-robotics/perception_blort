//
// Thread.cpp: implementation file
//
// Copyright (C) Walter E. Capers.  All rights reserved
//
// This source is free to use as you like.  If you make
// any changes please keep me in the loop.  Email your changes
// to walt.capers@comcast.net.
//
// PURPOSE:
//
//  To implement threading as a C++ object
//
// NOTES:
//  This object supports two types of thread models, event driven and
//  interval driven.  Under the event driven model, a thread waits
//  in a paused state until the member function Event is called.  When
//  the Event function is called the thread wakes up and calls OnTask.
//  Under the interval driven model, the thread wakes up every
//  m_dwIdle milli-seconds and calls OnTask.
//
//  You can switch between the two models from within the same object.
//
// COMPILER NOTES:
// On Unix you must use -lpthread a -lrt
// On Windows you must specify threaded under C++ code generation
//
// REVISIONS
// =======================================================
// Date: 10.24.07        
// Name: Walter E. Capers
// Description: File creation
//
// Date: 10.24.07 11:49 am
// Name: Walter E. Capers
// Description: Added SetIdle function to allow the idle time to be altered
//              independent of the SetThreadType member function.
//              Added sleep interval to Stop function.
//
// Date: 10.25.07
// Name: Walter E. Capers
// Description: Added support for other non-windows platforms.
//
//              Added static functions: ThreadIdsEqual and ThreadId.
//
//              Added que for handling multiple events.
//
//              Created the CEventClass and CMutexClass classes to facilitate
//              platform independence.
//
// Date: 10.26.07
// Name: Walter E. Capers
// Description: Made object production ready...
//              Added more comments
//
//              Addressed various issues with threads on UNIX systems.
//                -- there was a defect in the Sleep function
//                -- there was a defect in the main thread function THKERNEL
//                   , when transitioning between thread models the CEvent::Reset
//                   function was not being called when it was necessary resulting
//                   in a lock up.
//              
//				 Transition between thread types also failed on WINDOWS since the Event
//               member function was being called from within SetThreadType.  This
//               resulted in an Event usage error.  To correct the problem m_event.Set
//               is called instead.  Also, eliminated unecessary logic.
//
//               Got rid of OnStart, OnStop, OnDestroy... Could not override with a derived
//				 class, not sure why I will come back to in a later release.
//
//				 Changed default behavior of thread.  If OnTask is not redefined in the derived
//               class the default version now expects a CTask object.  The Class for CTask 
//               is defined in thread.h.  A class must be derived from CTask to use it in
//               the default version of OnTask(LPVOID).
//
//

#include <blort/ThreadObject/Thread.h>


#ifndef WINDOWS
extern "C"
{
 int	usleep(useconds_t useconds);
#ifdef NANO_SECOND_SLEEP
 int 	nanosleep(const struct timespec *rqtp, struct timespec *rmtp);
#endif
}

void Sleep( unsigned int milli )
{
#ifdef NANO_SECOND_SLEEP
	struct timespec interval, remainder;
	milli = milli * 1000000;
	interval.tv_sec= 0;
	interval.tv_nsec=milli;
	nanosleep(&interval,&remainder);
#else
	usleep(milli*1000);
#endif	
}
#endif

#include <iostream>
using namespace std;

/**
 * 
 * _THKERNEL
 * thread callback function used by CreateThread
 *
 *
 **/
#ifdef WINDOWS
DWORD WINAPI
#else
LPVOID
#endif
_THKERNEL( LPVOID lpvData /* CThread Object */ 
		  )
{
	CThread *pThread = (CThread *)lpvData;
	ThreadType_t lastType;
	/*
	 *
	 * initialization
	 *
	 */


    pThread->m_mutex.Lock();
		pThread->m_state = ThreadStateWaiting;
		pThread->m_bRunning = TRUE;
#ifndef WINDOWS
		pThread->m_dwId = CThread::ThreadId();
#endif
	pThread->m_mutex.Unlock();
	
	while( TRUE )
	{
		lastType = pThread->m_type;

		if( lastType == ThreadTypeEventDriven )
		{
			if( ! pThread->m_event.Wait()  ) 
					break;
		}
	
		if( ! pThread->KernelProcess() ) 
				break;


		if( lastType == ThreadTypeEventDriven )
			pThread->m_event.Reset();

		if( pThread->m_type == ThreadTypeIntervalDriven )
			Sleep(pThread->m_dwIdle);

	}


	pThread->m_mutex.Lock();
		pThread->m_state = ThreadStateDown;
		pThread->m_bRunning = FALSE;
	pThread->m_mutex.Unlock();


#ifdef WINDOWS
	return 0;
#else
	return (LPVOID)0;
#endif
}

/**
 *
 * OnTask
 * called when a thread is tasked using the Event
 * member function
 *
 **/
BOOL 
CThread::OnTask( LPVOID lpvData /*data passed from thread*/ 
					   )
{
	CTask *pTask = (CTask *)lpvData;

	pTask->SetTaskStatus(TaskStatusBeingProcessed);

    BOOL bReturn = pTask->Task();

	pTask->SetTaskStatus(TaskStatusCompleted);


	return bReturn; 
} 


/**
 *
 * OnTask
 * overloaded implementation of OnTask that
 * takes no arguments
 *
 **/
BOOL
CThread::OnTask()
{
	printf("\nthread is alive\n");

	return TRUE;
}


BOOL
CThread::Event(CTask *pvTask /* data to be processed by thread */
			   )
{
	ThreadId_t id;
	m_mutex.Lock();
	if( m_type != ThreadTypeEventDriven )
	{
		m_mutex.Unlock();
		m_dwObjectCondition |= ILLEGAL_USE_OF_EVENT;
		m_state = ThreadStateFault;
		return FALSE;
	}

	m_mutex.Unlock();
	GetId(&id);
	pvTask->SetId(&id);
	if( ! Push((LPVOID)pvTask) )
		return FALSE;

	pvTask->SetTaskStatus(TaskStatusWaitingOnQueue);
	m_event.Set();

	return TRUE;
}

/**
 *
 * Event
 * wakes up a thread to process data
 *
 **/
BOOL
CThread::Event(LPVOID lpvData /* data to be processed by thread */
			   )
{
	m_mutex.Lock();
	if( m_type != ThreadTypeEventDriven )
	{
		m_mutex.Unlock();
		m_dwObjectCondition |= ILLEGAL_USE_OF_EVENT;
		m_state = ThreadStateFault;
		return FALSE;
	}

	m_mutex.Unlock();
	if( ! Push(lpvData) )
		return FALSE;

	m_event.Set();

	return TRUE;
}


/**
 *
 * SetPriority
 * sets a threads run priority, see SetThreadPriority
 * Note: only works for Windows family of operating systems
 *
 *
 **/
void
CThread::SetPriority(DWORD dwPriority)
{

#ifdef WINDOWS
	SetThreadPriority(m_thread,dwPriority);
#endif
}

	  
/**
 *
 * KernelProcess
 * routes thread activity
 *
 **/
BOOL
CThread::KernelProcess()
{

	m_mutex.Lock();
	m_state = ThreadStateBusy;
	if( !m_bRunning )
	{
		m_state = ThreadStateShuttingDown;
		m_mutex.Unlock();
		return FALSE;
	}
	m_mutex.Unlock();

	if( !Empty() )
	{
		while( !Empty() )
		{
			Pop();
			if( !OnTask(m_lpvProcessor) )
			{
				m_mutex.Lock();
				m_lpvProcessor = NULL;
				m_state = ThreadStateShuttingDown;
				m_mutex.Unlock();
				return FALSE;
			}
		}
		m_mutex.Lock();
		m_lpvProcessor = NULL;
		m_state = ThreadStateWaiting;
	}
	else {
		if( !OnTask() )
		{
			m_mutex.Lock();
			m_state = ThreadStateShuttingDown;
			m_mutex.Unlock();
			return FALSE;
		}
		m_mutex.Lock();
		m_state = ThreadStateWaiting;
	}

	m_mutex.Unlock();

	return TRUE;
}


/**
 * 
 * GetEventsPending
 * returns the total number of vents waiting
 * in the event que
 * 
 **/
unsigned int
CThread::GetEventsPending()
{
	unsigned int chEventsWaiting;

	m_mutex.Lock();
	  chEventsWaiting = m_quePos;
    m_mutex.Unlock();

	return chEventsWaiting;
}


/**
 *
 * CThread
 * instanciates thread object and
 * starts thread.
 *
 **/
CThread::CThread(void)
:m_bRunning(FALSE)
#ifdef WINDOWS
,m_thread(NULL)
#endif
,m_dwId(0L)
,m_chQue(QUE_SIZE)
,m_quePos(0)
,m_lpvProcessor(NULL)
,m_dwIdle(100)
,m_type(ThreadTypeEventDriven)
,m_stackSize(DEFAULT_STACK_SIZE)
{

	m_dwObjectCondition = NO_ERRORS;

	m_lppvQue = new LPVOID [QUE_SIZE];

	if( !m_lppvQue ) 
	{
		m_dwObjectCondition |= MEMORY_FAULT;
		m_state = ThreadStateFault;
		return;
	}

	if( !m_mutex.m_bCreated )
	{
		perror("mutex creation failed");
		m_dwObjectCondition |= MUTEX_CREATION;
		m_state = ThreadStateFault;
		return;
	}


	if( !m_event.m_bCreated )
	{
		perror("event creation failed");
		m_dwObjectCondition |= EVENT_CREATION;
		m_state = ThreadStateFault;
		return;
	}


	Start();

}

/**
 *
 * Empty
 * returns a value of TRUE if there are no items on the threads que
 * otherwise a value of FALSE is returned.
 *
 **/
BOOL
CThread::Empty()
{
	m_mutex.Lock();
	if( m_quePos <= 0 )
	{
		m_mutex.Unlock();
		return TRUE;
	}
	m_mutex.Unlock();
	return FALSE;
}



/**
 *
 * Push
 * place a data object in the threads que
 *
 **/
BOOL
CThread::Push( LPVOID lpv )
{
	if( !lpv ) return TRUE;

	m_mutex.Lock();

	if( m_quePos+1 >= m_chQue ) {
		m_mutex.Unlock();
		return FALSE;
	}

	m_lppvQue[m_quePos++] = lpv;
	m_mutex.Unlock();
	return TRUE;
}


/**
 *
 * Pop
 * move an object from the input que to the processor
 *
 **/
BOOL
CThread::Pop()
{

	m_mutex.Lock();
        if( (int)m_quePos-1 < 0 )
	{
		m_quePos = 0;
		m_mutex.Unlock();
		return FALSE;
	}
	m_quePos--;
	m_lpvProcessor = m_lppvQue[m_quePos];
	m_mutex.Unlock();
	return TRUE;
}


/**
 *
 * SetThreadType
 * specifies the type of threading that is to be performed.
 *
 * ThreadTypeEventDriven (default): an event must be physically sent
 *									to the thread using the Event member
 *									function.
 *
 * ThreadTypeIntervalDriven       : an event occurs automatically every 
 *                                  dwIdle milli seconds.
 *
 **/
void
CThread::SetThreadType(ThreadType_t typ,
              DWORD dwIdle)
{

	m_mutex.Lock();
	 m_dwIdle = dwIdle;
	
	 if( m_type == typ ) {
		 m_mutex.Unlock();
		 return;
	 }

		m_type = typ;


   m_mutex.Unlock();
   m_event.Set();
}


/**
 *
 * Stop
 * stop thread 
 *
 **/
void
CThread::Stop()
{
		m_mutex.Lock();
			m_bRunning = FALSE;
		m_mutex.Unlock();
		Event();

		Sleep(m_dwIdle);
		while(TRUE)
		{
			m_mutex.Lock();
			if( m_state == ThreadStateDown )
			{
				m_mutex.Unlock();
				return;
			}
			m_mutex.Unlock();
			Sleep(m_dwIdle);
		}
}


/**
 *
 * SetIdle
 * changes the threads idle interval
 *
 **/
void
CThread::SetIdle(DWORD dwIdle)
{
	m_mutex.Lock();
		m_dwIdle = dwIdle;
	m_mutex.Unlock();
}

/**
 *
 * Start
 * start thread
 *
 **/
BOOL
CThread::Start()
{
	m_mutex.Lock();
	if( m_bRunning ) 
	{
		m_mutex.Unlock();
		return TRUE;
	}

	m_mutex.Unlock();


	if( m_dwObjectCondition & THREAD_CREATION )
		m_dwObjectCondition = m_dwObjectCondition ^ THREAD_CREATION;

#ifdef WINDOWS
	if( m_thread ) CloseHandle(m_thread);

	m_thread = CreateThread(NULL,m_stackSize ,_THKERNEL,(LPVOID)this,0,&m_dwId);
	if( !m_thread )
	{
		perror("thread creating failed");
		m_dwObjectCondition |= THREAD_CREATION;
		m_state = ThreadStateFault;
		return FALSE;
	}
#else
	pthread_attr_t attr;
	
	pthread_attr_init(&attr);

#ifdef VMS
	if( m_stackSize == 0 )
		pthread_attr_setstacksize(&attr,PTHREAD_STACK_MIN*10);
#endif
	if( m_stackSize != 0 )
		pthread_attr_setstacksize(&attr,m_stackSize);

	int error = pthread_create(&m_thread,&attr,_THKERNEL,(LPVOID)this);

	if( error != 0 )
	{
		m_dwObjectCondition |= THREAD_CREATION;
		m_state = ThreadStateFault;

#if defined(HPUX) || defined(SUNOS) || defined(LINUX)
		  switch(error)/* show the thread error */
		  {

		  case EINVAL:
			  cerr << "error: attr in an invalid thread attributes object\n";
			  break;
		  case EAGAIN:
			  cerr << "error: the necessary resources to create a thread are not\n";
			  cerr << "available.\n";
			  break;
		  case EPERM:
			  cerr << "error: the caller does not have the privileges to create\n";
			  cerr << "the thread with the specified attr object.\n";
			  break;
#if defined(HPUX)
		  case ENOSYS:

			  cerr << "error: pthread_create not implemented!\n";
			  if( __is_threadlib_linked()==0 )
			  {
				  cerr << "error: threaded library not being used, improper linkage \"-lpthread -lc\"!\n";
			  }
			  break;
#endif
		  default:
			  cerr << "error: an unknown error was encountered attempting to create\n";
			  cerr << "the requested thread.\n";
			  break;
		  }
#else
		cerr << "error: could not create thread, pthread_create failed (" << error << ")!\n";
#endif
		 return FALSE;	
	}
#endif
	return TRUE;
}


/**
 *
 * ThreadState
 * return the current state of the thread
 *
 **/
ThreadState_t 
CThread::ThreadState()
{
	ThreadState_t currentState;
	m_mutex.Lock();
		currentState = m_state;
	m_mutex.Unlock();
	return currentState;
}

/**
 *
 * ~CThread
 * destructor.  Stop should be called prior to destruction to
 * allow for gracefull thread termination.
 *
 **/
CThread::~CThread(void)
{
#ifdef WINDOWS
	if( m_bRunning )
	{
		// TerminateThread(m_thread,1); brutal termination
	  Stop(); // gracefull terminatation
	  WaitForSingleObject(m_thread,INFINITE);

	}
	CloseHandle(m_thread);
#else
	LPVOID lpv;

	if( m_bRunning )
	{
		Stop();
		pthread_join(m_thread,&lpv);
	}
#endif

	delete [] m_lppvQue;
}


/**
 *
 * PingThread
 * used to determine if a thread is running
 *
 **/
BOOL
CThread::PingThread(DWORD dwTimeout /* timeout in milli-seconds */
				 )
{
    DWORD dwTotal = 0;

	while(TRUE)
	{
		if( dwTotal > dwTimeout && dwTimeout > 0 )
			return FALSE;
		m_mutex.Lock();
			if( m_bRunning )
			{
				m_mutex.Unlock();
				return TRUE;
			}
		dwTotal += m_dwIdle;
		m_mutex.Unlock();
		Sleep(m_dwIdle);
	}

	return FALSE;
}




