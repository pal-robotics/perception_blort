
#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdio.h>
#include <stdlib.h>

namespace Tracking{

/* allocate (and free) aligned memory, align must be power of 2 */
void *malloc_align(size_t size, int align);
void free_align(void *ptr);

/* read entierely a file, returning a 0 terminated string */
char *read_text_file(const char* file);

/* offset of a membet in a structure */
#undef OFFSETOF
#define OFFSETOF(TYPE, MEMBER) (((size_t)&((TYPE *)1)->MEMBER) - 1)

/* some templates */
template<typename T>
T Max(T a,T b)
{
    return a>b?a:b;
}

template<typename T>
T Min(T a,T b)
{
    return a<b?a:b;
}

template<typename T>
T Abs(T a)
{
    return a>0?a:-a;
}

template<typename T>
T Square(T a)
{
    return a*a;
}

template<typename T>
T Sign(T a)
{
    return a==0?0:a>0?1:-1;
}

} /* namespace */

#endif //__UTILS_H__
