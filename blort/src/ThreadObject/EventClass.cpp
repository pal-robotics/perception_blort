//
// EventClass.cpp: implementation file
//
// Copyright (C) Walter E. Capers.  All rights reserved
//
// This source is free to use as you like.  If you make
// any changes please keep me in the loop.  Email them to
// walt.capers@comcast.net.
//
// PURPOSE:
//
//  To implement event signals as a C++ object
//
// REVISIONS
// =======================================================
// Date: 10.25.07        
// Name: Walter E. Capers
// Description: File creation
//
// Date:
// Name:
// Description:
//
//
#include <blort/ThreadObject/Thread.h>

CEventClass::CEventClass(void)
:m_bCreated(TRUE)
{
#ifdef WINDOWS
	m_event = CreateEvent(NULL,FALSE,FALSE,NULL);
	if( !m_event )
	{
		m_bCreated = FALSE;
	}
#else
	pthread_mutexattr_t mattr;
	
	pthread_mutexattr_init(&mattr);
	pthread_mutex_init(&m_lock,&mattr);
	pthread_cond_init(&m_ready,NULL);

#endif	
}

CEventClass::~CEventClass(void)
{
#ifdef WINDOWS
	CloseHandle(m_event);
#else
	pthread_cond_destroy(&m_ready);
	pthread_mutex_destroy(&m_lock);
#endif
}


/**
 *
 * Set
 * set an event to signaled
 *
 **/
void
CEventClass::Set()
{
#ifdef WINDOWS
	SetEvent(m_event);
#else
	pthread_mutex_lock(&m_lock);
	pthread_mutex_unlock(&m_lock);
	pthread_cond_signal(&m_ready);
#endif
}

/**
 *
 * Wait
 * wait for an event -- wait for an event object
 * to be set to signaled
 *
 **/
BOOL
CEventClass::Wait()
{
#ifdef WINDOWS
	if( WaitForSingleObject(m_event,INFINITE) != WAIT_OBJECT_0 )
	{
		return FALSE;
	}
	return TRUE;
#else
	pthread_mutex_lock(&m_lock);
	pthread_cond_wait(&m_ready,&m_lock);
	return TRUE;
#endif
}

/**
 *
 * Reset
 * reset an event flag to unsignaled
 *
 **/
void
CEventClass::Reset()
{
#ifndef WINDOWS
	pthread_mutex_unlock(&m_lock);
#endif
}

