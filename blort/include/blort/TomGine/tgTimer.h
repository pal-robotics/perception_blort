 /**
 * @file tgTimer.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Real-time clock for MS Windows and Linux.
 */

#ifndef TG_TIMER
#define TG_TIMER

#ifdef _WIN32
	#include <windows.h>
#else
	#include <time.h>
	#include <sys/time.h>
#endif

// ***********************************************************************************

namespace TomGine{

/**
* @brief Class tgTimer
*/
class tgTimer
{
private:
#ifdef WIN32
	LONGLONG m_StartTicks;		// QueryPerformance - Ticks at application start
	LONGLONG m_EndTicks;		// QueryPerformance - Ticks when calling Now()
	LONGLONG m_Frequency;		// QueryPerformance - Fequency
	double fNow;
#else	
	struct timespec AppStart, act, old;
#endif
	double m_fAppTime;			// Time since application started
	double m_fTime;				// Time between two Update calls

public:
	tgTimer(void);
	~tgTimer(void);
	
	void	Reset();
	double	Update();
	
	double	GetFrameTime(){ return m_fTime;}
	double	GetApplicationTime(){ return m_fAppTime;}
};

} // namespace TomGine

#endif

