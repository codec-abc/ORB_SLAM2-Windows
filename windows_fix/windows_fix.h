#ifndef WINDOWS_FIX
#define WINDOWS_FIX
#
#ifdef WIN32
#include <vadefs.h>
void usleep(__int64 usec);
int vasprintf(char **ret, const char *format, va_list args);
#endif

#endif