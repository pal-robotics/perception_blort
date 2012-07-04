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

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <algorithm>

#ifdef WIN32
#define snprintf _snprintf
template <typename TYPE> inline TYPE round(TYPE a) {
	return ::floor(a + (TYPE)0.5);
}
#endif

#endif // _HEADERS_H_
