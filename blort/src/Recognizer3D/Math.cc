/**
 * $Id: Math.cc 34111 2012-07-03 14:29:54Z student5 $
 */

#include <blort/Recognizer3D/Math.hh>


namespace P 
{


/**
 * Subtract the `struct timeval' values x - y.
 * Return 1 if the difference is negative, otherwise 0.
 */
int timeval_subtract(struct timeval *result, struct timeval *x,
  struct timeval *y)
{
  if(x->tv_usec < y->tv_usec)
  {
    int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
    y->tv_usec -= 1000000 * nsec;
    y->tv_sec += nsec;
  }
  if(x->tv_usec - y->tv_usec > 1000000)
  {
    int nsec = (y->tv_usec - x->tv_usec) / 1000000;
    y->tv_usec += 1000000 * nsec;
    y->tv_sec -= nsec;
  }
  result->tv_sec = x->tv_sec - y->tv_sec;
  result->tv_usec = x->tv_usec - y->tv_usec;
  return x->tv_sec < y->tv_sec;
}

/**
 * Returns timespecs x - y as double.
 */
double timespec_diff(struct timespec *x, struct timespec *y)
{
  /*timespec res = {x->tv_sec - y->tv_sec, x->tv_nsec - y->tv_nsec};
  if(res.tv_nsec >= 1000000000)
  {
    res.tv_nsec -= 1000000000;
    res.tv_sec++;
  }
  if(res.tv_nsec < 0)
  {
    res.tv_nsec += 1000000000;
    res.tv_sec--;
  }
  return (double)res.tv_sec + 1e-9*(double)res.tv_nsec;*/
  /* TODO: make the above clearer version handle nsec overflows of more than one
   * sec, see below*/
  if(x->tv_nsec < y->tv_nsec)
  {
    int nsec = (y->tv_nsec - x->tv_nsec) / 1000000000 + 1;
    y->tv_nsec -= 1000000000 * nsec;
    y->tv_sec += nsec;
  }
  if(x->tv_nsec - y->tv_nsec > 1000000000)
  {
    int nsec = (y->tv_nsec - x->tv_nsec) / 1000000000;
    y->tv_nsec += 1000000000 * nsec;
    y->tv_sec -= nsec;
  }
  return (double)(x->tv_sec - y->tv_sec) +
    (double)(x->tv_nsec - y->tv_nsec)/1000000000.;
}



}

