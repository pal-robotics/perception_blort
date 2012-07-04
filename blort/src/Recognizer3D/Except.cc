/**
 * $Id: Except.cc 34111 2012-07-03 14:29:54Z student5 $
 */

#include <stdarg.h>
#include <stdio.h>
#include <blort/Recognizer3D/Except.hh>

namespace P
{

/**
 * Except constructor.
 * @param file     (in) __FILE__ macro
 * @param function (in) __FUNCTION__ macro
 * @param line     (in) __LINE__ macro
 * @param format   (in) printf-style format string
 */
Except::Except(const char *file, const char *function, int line,
               const char *format, ...) throw()
{
  static char what[1024];
  static char msg[1024];
  va_list arg_list;
  va_start(arg_list, format);
  vsnprintf(what, 1024, format, arg_list);
  va_end(arg_list);
  snprintf(msg, 1024, "%s:%s:%d: %s", file, function, line, what);
  _what = msg;
}

}

