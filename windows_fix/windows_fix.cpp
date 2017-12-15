#ifdef WIN32
#include "windows_fix.h"
#include <windows.h>
#include <cstdio>
#include <cstdlib>

void usleep(__int64 usec)
{
	HANDLE timer;
	LARGE_INTEGER ft;

	ft.QuadPart = -(10 * usec); // Convert to 100 nanosecond interval, negative value indicates relative time

	timer = CreateWaitableTimer(NULL, TRUE, NULL);
	SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
	WaitForSingleObject(timer, INFINITE);
	CloseHandle(timer);
}

int vasprintf(char **ret, const char *format, va_list args)
{
	va_list copy;
	va_copy(copy, args);

	/* Make sure it is determinate, despite manuals indicating otherwise */
	*ret = NULL;

	int count = vsnprintf(NULL, 0, format, args);
	if (count >= 0)
	{
		char* buffer = (char*) malloc(count + 1);
		if (buffer == NULL)
			count = -1;
		else if ((count = vsnprintf(buffer, count + 1, format, copy)) < 0)
			free(buffer);
		else
			*ret = buffer;
	}
	va_end(copy);  // Each va_start() or va_copy() needs a va_end()

	return count;
}
#endif
