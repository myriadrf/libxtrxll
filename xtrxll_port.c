/*
 * xtrx portability layer file
 * Copyright (c) 2017 Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>
 * For more information, please visit: http://xtrx.io
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */
#include "xtrxll_port.h"

#if defined(__linux) || defined(__APPLE__)
#define  THREAD_SAFE __thread

#elif defined(_WIN32) || defined(_WIN32_WCE)

/* Thread storage is available starting Windows 7, for compatibility reason not using them */
#define  THREAD_SAFE

#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
#define Const64(x)    x##i64
#define DELTA_EPOCH_IN_MICROSECS  11644473600000000Ui64
#else
#define Const64(x)    x##LL
#define DELTA_EPOCH_IN_MICROSECS  11644473600000000ULL
#endif

static struct tm *localtime_r(const time_t *timep, struct tm *result)
{
	if (sizeof(time_t) == 8) {
		if (_localtime64_s(result, (const long long *)timep) == 0) {
			return result;
		}
	} else {
		if (_localtime32_s(result, (const long int *)timep) == 0) {
			return result;
		}
	}
	return NULL;
}

int usleep(unsigned usec)
{
#if 1
	static HANDLE hlib;
	static NTSTATUS (NTAPI *pNtDelayExecution)(IN BOOLEAN, IN PLARGE_INTEGER);
	LARGE_INTEGER timeout;
	NTSTATUS status;

	if (pNtDelayExecution == NULL) {
		 hlib = LoadLibrary("ntdll.dll");
		 pNtDelayExecution = (NTSTATUS (NTAPI *)(IN BOOLEAN, IN PLARGE_INTEGER))GetProcAddress(hlib, "NtDelayExecution");
	}

	timeout.QuadPart = (LONGLONG)-10 * usec;
	status = pNtDelayExecution(FALSE, &timeout);

	return status == 0 ? 0 : -1;
#else
	Sleep(usec / 1000);
	return 0;
#endif
}

unsigned sleep(unsigned seconds)
{
	Sleep(seconds * 1000);
	return 0;
}

static void clock_gettime_realtime(struct timespec *tp)
{
	static THREAD_SAFE unsigned long run_count = 0;
	static THREAD_SAFE unsigned __int64 base_ticks, tick_frequency;
	static THREAD_SAFE unsigned __int64 base_systime_as_filetime;
	unsigned __int64 ticks;
	unsigned __int64 ft;

	if (1) {
		if (run_count++) {
			QueryPerformanceCounter((LARGE_INTEGER*)&ticks);
			ticks -= base_ticks;
			ft = base_systime_as_filetime
				+ Const64(10000000) * (ticks / tick_frequency)
				+(Const64(10000000) * (ticks % tick_frequency)) /
				tick_frequency;
		} else {
			FILETIME ft_val;
			QueryPerformanceFrequency((LARGE_INTEGER*)&tick_frequency);
			QueryPerformanceCounter((LARGE_INTEGER*)&base_ticks);
			GetSystemTimeAsFileTime(&ft_val);
			base_systime_as_filetime = ft_val.dwHighDateTime;
			base_systime_as_filetime <<= 32;
			base_systime_as_filetime |= ft_val.dwLowDateTime;
			ft = base_systime_as_filetime;
		}
	} else {
		FILETIME ft_val;
		GetSystemTimeAsFileTime(&ft_val);
		base_systime_as_filetime = ft_val.dwHighDateTime;
		base_systime_as_filetime <<= 32;
		base_systime_as_filetime |= ft_val.dwLowDateTime;
		ft = base_systime_as_filetime;
	}

	/* seconds since epoch */
	tp->tv_sec = ((long)((ft - DELTA_EPOCH_IN_MICROSECS) / Const64(10000000))) - 1890091648;

	/* microseconds remaining */
	tp->tv_nsec = (long)((ft % Const64(1000000)) * Const64(100));
}

int clock_gettime(clockid_t clk_id, struct timespec *tp)
{
	if (clk_id == CLOCK_REALTIME) {
		clock_gettime_realtime(tp);
		return 0;
	}
	// TODO set errno
	errno = EINVAL;
	return -1;
}

static DWORD win_dl_err;
void *dlopen(const char *file, int mode)
{
	HANDLE h = LoadLibraryA(file);
	win_dl_err = (h == NULL) ? GetLastError() : 0;
	return h;
}

int dlclose(void *handle)
{
	return FreeLibrary((HMODULE)handle) ? 0 : -1;
}

void *dlsym(void *__restrict handle,
			const char *__restrict name)
{
	void *sym = GetProcAddress((HMODULE)handle, name);
	win_dl_err = (sym == NULL) ? GetLastError() : 0;
	return sym;
}

char *dlerror(void)
{
	if (win_dl_err == 0) {
		return NULL;
	}

	// TODO use FormatMessage
	return "UNKNOWN";
}

int pthread_create(pthread_t *__restrict nt,
				   const void *__restrict __attr,
				   void *(*start_routine) (void *),
				   void *__restrict arg)
{
	HANDLE h = CreateThread(NULL, 0,
							(LPTHREAD_START_ROUTINE)start_routine,
							arg,
							0,
							NULL);
	*nt = h;
	return (h == NULL) ? -1 : 0;
}

int pthread_join(pthread_t th, void **thread_return)
{
	DWORD res = WaitForSingleObject((HANDLE)th, INFINITE);
	if (res != WAIT_OBJECT_0)
		return -1;

	if (thread_return) {
		GetExitCodeThread((HANDLE)th, (LPDWORD)thread_return);
	}

	CloseHandle((HANDLE)th);
	return 0;
}


int sem_init(sem_t *sem, int pshared, unsigned int value)
{
	HANDLE h = CreateSemaphore(NULL, value, INT32_MAX, NULL);
	*sem = h;
	return (h == NULL) ? -1 : 0;
}

int sem_destroy(sem_t *sem)
{
	return CloseHandle(*sem) ? 0 : -1;
}

int sem_wait(sem_t *sem)
{
	return (WaitForSingleObject(*sem, INFINITE) == WAIT_OBJECT_0) ? 0 : -1;
}

int sem_trywait(sem_t *sem)
{
	switch (WaitForSingleObject(*sem, 0)) {
	case WAIT_OBJECT_0:
		return 0;
	case WAIT_TIMEOUT:
		errno = EAGAIN;
		return -1;
	default:
		errno = EINVAL;
		return -1;
	}
}

int sem_post(sem_t *sem)
{
	return ReleaseSemaphore(*sem, 1, NULL) ? 0 : -1;
}

int sem_timedwait(sem_t *sem, const struct timespec *abs_timeout)
{
	struct timespec now;
	clock_gettime_realtime(&now);

	int64_t wait_ms = (abs_timeout->tv_sec - now.tv_sec) * 1000 +
			(abs_timeout->tv_nsec - now.tv_nsec) / 1000000;
	if (wait_ms < 0)
		return 0;

	switch (WaitForSingleObject(*sem, (DWORD)wait_ms)) {
	case WAIT_OBJECT_0:
		return 0;
	case WAIT_TIMEOUT:
		errno = ETIMEDOUT;
		return -1;
	default:
		errno = EINVAL;
		return -1;
	}
}

int sem_getvalue(sem_t *sem, int *sval)
{
	return -1;
}
#endif

const struct tm* xtrxll_localtime(time_t now)
{
	const unsigned SECONDS_PER_MINUTE = 60;
	const unsigned MINUTES_PER_HOUR = 60;
	const time_t   SECONDS_PER_DAY = 60 * 60 * 24;

	static THREAD_SAFE time_t day_start;
	static THREAD_SAFE time_t day_end;
	static THREAD_SAFE struct tm day_tm;

	time_t timeofday;

	if (!(day_start <= now && now < day_end)) {
		// initialize new day if changed
		localtime_r(&now, &day_tm);
		day_tm.tm_hour = day_tm.tm_min = day_tm.tm_sec = 0;
		day_start = mktime(&day_tm);
		day_end = day_start + SECONDS_PER_DAY;
	}

	timeofday = now - day_start;
	day_tm.tm_sec = timeofday % SECONDS_PER_MINUTE;
	timeofday /= SECONDS_PER_MINUTE;
	day_tm.tm_min = timeofday % MINUTES_PER_HOUR;
	timeofday /= MINUTES_PER_HOUR;
	day_tm.tm_hour = (int)timeofday;
	return &day_tm;
}

