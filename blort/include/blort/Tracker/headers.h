#pragma once

#ifndef _HEADERS_H_
#define _HEADERS_H_

#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0500
#endif
#define NOMINMAX
#include <windows.h>
#else
#include <time.h>
#include <sys/time.h>
#endif

#ifdef WIN32
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>
#endif

// OpenCV2.x include has extra directory level
//#include <opencv/highgui.h>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <stdexcept>

#ifdef WIN32
#define snprintf _snprintf
template <typename TYPE> inline TYPE round(TYPE a) {
	return ::floor(a + (TYPE)0.5);
}
#endif

#endif // _HEADERS_H_
