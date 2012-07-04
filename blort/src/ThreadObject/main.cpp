//
// main.cpp: implements and tests CThread and CTask classes 
//
// Copyright (C) Walter E. Capers.  All rights reserved
//
// This source is free to use as you like.  If you make
// any changes, keep me in the loop.  Email your changes
// to walt.capers@comcast.net.
//
//

#include <blort/ThreadObject/Thread.h>
#include <iostream>
using namespace std;

/*
 * CIncrementThread
 * Class derived from CThread which redefines OnTask
 *
 */
class CIncrementThread : public CThread
{
public:
	int counter;

	virtual BOOL OnTask( LPVOID lpv )
	{
		ThreadId_t id;

		GetId(&id);
		if( lpv )
		{
			int *pInt = (int *)lpv;

			//dont' use cout here, output could be broken up due to threading
			printf("\tthread(%ld, counter+%d=%d, counter incremented\n",id,*pInt,(counter+=*pInt));
		}
		return TRUE;
	}

	virtual BOOL OnTask()
	{
		ThreadId_t id;

		GetId(&id);
		//don't use cout here, output could be broken up due to threading
                m_mutex.Lock();  // protect the counter variable
		  printf("\tthread(%ld, counter++= %d, counter incremented)\n",id,(++counter));
                m_mutex.Unlock();


		return TRUE;
	}

        int GetValue()
        {
            int counterValue;
            m_mutex.Lock();       // protect the counter variable
              counterValue = counter;
            m_mutex.Unlock();
            return counter;
         }
           
        void Reset()
        {
            m_mutex.Lock();
               counter = 0;
            m_mutex.Unlock();
        }

	CIncrementThread(){counter=0;}
	~CIncrementThread(){}
};

class CTaskIncrementer: public CTask
{
private:
	int counter;
	int incr;
public:
	void SetIncr(int iValue) 
	{
		m_mutex.Lock();
		  incr = iValue;
	    m_mutex.Unlock();
	}

	int GetIncrementValue()
	{
		int incrValue;
		m_mutex.Lock();
		  incrValue=incr;
		m_mutex.Unlock();
		return incrValue;
	}

	int GetValue()
	{
		int counterValue;
		m_mutex.Lock();       // protect the counter variable
		  counterValue = counter;
		m_mutex.Unlock();
		return counter;
	}

	BOOL Task() 
	{
		ThreadId_t id;

	    Thread(&id);

		m_mutex.Lock();
		  printf("\tthread(%ld, counter+%d=%d, counter incremented\n",id,incr,(counter+=incr));
	    m_mutex.Unlock();
		return TRUE;
	}
	CTaskIncrementer(){counter=0;incr=0;}
	~CTaskIncrementer(){}
};



/*
 *
 * MyTaskClass
 * class derived from CTask which redefines Task
 * Task is called by CThreads OnTask member function.
 *
 */
class MyTaskClass: public CTask
{
public:
	BOOL Task()
	{
		cout << "\tperformed a ctask task\n";
		return TRUE;
	}
	MyTaskClass(){}
	~MyTaskClass(){}
};

int 
main(int argc, 
	 char *argv[] )
{
	int increment=10;

	CIncrementThread *pThreadInc;

	cout << "\n****** T H R E A D  T E S T *********\n\n";

	cout << "--INSTANTIATING CINCREMENTTHREAD DERIVED FROM CTHREAD\n";
	pThreadInc = new CIncrementThread;

	if( pThreadInc )
	{
		if( pThreadInc->PingThread(500) )
		{
			cout << "\nSENDING EVENT + 10\n";
			pThreadInc->Event((LPVOID)&increment);
			Sleep(500);
			cout << "\nSENDING EVENT ++\n";
			pThreadInc->Event();
			Sleep(500);
		}
		else {
			cout << "\nTHREAD DID NOT START\n";
			exit(0);
		}
		cout << "\n--CHANGING THREAD MODEL TO INTERVAL BASED\n";
		pThreadInc->SetThreadType(ThreadTypeIntervalDriven,100);
		Sleep(500);
		cout << "\n--CHANGING THREAD MODEL BACK TO EVENT DRIVEN\n";
		pThreadInc->SetThreadType(ThreadTypeEventDriven);
		Sleep(500);
		cout << "\n--SENDING + 10\n";
		pThreadInc->Event((LPVOID)&increment);
		Sleep(500);
		cout << "\n--DESTROYING OBJECT\n";
		delete pThreadInc;
		Sleep(500);
	}
	else
	{
		cout << "FAILED TO ALLOCATE THREAD\n";
		exit(0);
	}

	cout << "\n--INSTANTIATING AN ARRAY OF TEN CINCREMENTTHREADS DERIVED FROM CTHREAD\n";
	pThreadInc = new CIncrementThread [10];
	if( pThreadInc )
	{
		if( pThreadInc[9].PingThread(2000) )
		{
			for(int i=0; i<10 ; i++ )
			{
				pThreadInc[i].Event();
			}
			Sleep(500);
			cout << "\n\n--DESTROYING OBJECT ARRAY\n";
			delete [] pThreadInc;
		}
		else {
			cout << "\nTHREADS DID NOT START\n";
			exit(0);
		}
	}
	else {
		cout << "FAILED TO ALLOCATE THREADS\n";
		exit(0);
	}

	cout << "\n--INSTANTIATING A CINCRMENTTHREAD BY DECLARATION\n";
	{
		CIncrementThread th;

		if( th.PingThread(2000) )
		{
			cout << "\nSENDING EVENT\n";
			th.Event();
			Sleep(500);
		}
		else
		{
			cout << "FAILED TO START THREAD\n";
			exit(0);
		}
		cout << "\nLEAVING SCOPE\n";
		
	}

    cout << "\n--INSTANTIATING A CINCREMENTTHREAD BY DECLARATION\n";
	{
		CIncrementThread MyThread; // object allocated and thread started
		int two = 2;
		while( MyThread.GetValue() < 20 )
		{
			MyThread.Event();  // increment value by one
			Sleep(100);        // pauses the root thread for 100 milli-seconds
		}

		MyThread.Reset();
		while( MyThread.GetValue() < 40 )
		{
			MyThread.Event(&two);
			Sleep(100);
		}

	}

	cout << "--INSTANTIATING THE ORIGIONAL CTHREAD CLASS BY DECLARATION\n";
	{
		MyTaskClass myTask;
		CThread thread;
		thread.Event(&myTask);
		while( myTask.Status() != TaskStatusCompleted )
			Sleep(500);

		thread.Event(&myTask);
		myTask.Wait(1);
	}

	cout << "\n--INSTANTIATING THE ORIGIONAL CTHREAD CLASS BY DECLARATION\n";
	{
		MyTaskClass *pMyTask = new MyTaskClass;

		CThread thread;
		thread.Event(pMyTask);
		if( pMyTask->Wait(1) )
		{
			cout << "\tfreeing pMyTask\n";
			delete pMyTask;
		}
	}

	cout << "\n--INSTANTIATING a CTHREAD AND USING A CTASK OBJECT\n";
	{
		CTaskIncrementer incr;
		CThread thr;

		incr.SetIncr(2);
		while( incr.GetValue() < 40 ) {
			thr.Event(&incr);
			Sleep(100);
		}
	}
	cout << "\ndone\n";

}


