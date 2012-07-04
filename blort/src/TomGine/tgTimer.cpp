
#include <blort/TomGine/tgTimer.h>

using namespace TomGine;

// ***********************************************************************************

tgTimer::tgTimer(void) {
#ifdef WIN32
	QueryPerformanceFrequency((LARGE_INTEGER*) &m_Frequency);
	QueryPerformanceCounter((LARGE_INTEGER*)(&m_StartTicks));
	m_EndTicks = m_StartTicks;
#else
	clock_gettime(CLOCK_REALTIME, &AppStart);
	clock_gettime(CLOCK_REALTIME, &old);
#endif
	m_fAppTime = 0.0;
}

tgTimer::~tgTimer(void) {
}

void tgTimer::Reset() {
#ifdef WIN32
	QueryPerformanceFrequency((LARGE_INTEGER*) &m_Frequency);
	QueryPerformanceCounter((LARGE_INTEGER*)(&m_StartTicks));
	m_EndTicks = m_StartTicks;
#else
	clock_gettime(CLOCK_REALTIME, &AppStart);
	clock_gettime(CLOCK_REALTIME, &old);
#endif
	m_fAppTime = 0.0;
}

double tgTimer::Update() {
#ifdef WIN32
	QueryPerformanceCounter((LARGE_INTEGER*)(&m_EndTicks));
	fNow = (double)(m_EndTicks - m_StartTicks) / m_Frequency;
	m_fTime = fNow - m_fAppTime;
	m_fAppTime = fNow;
#else
	clock_gettime(CLOCK_REALTIME, &act);
	m_fTime = (act.tv_sec - old.tv_sec) + (act.tv_nsec - old.tv_nsec) / 1e9;
	old = act;		
	m_fAppTime += m_fTime;
#endif
	return m_fTime;
}
