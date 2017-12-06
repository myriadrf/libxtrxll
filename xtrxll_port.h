/*
 * Internal xtrx portable layer header file
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

#ifndef XTRX_PORT_H
#define XTRX_PORT_H

/** @file xtrx_port.h
 *
 * Portability layer accross differrent platform and OSes
 */

#ifdef _MSC_VER
#if !defined(__cplusplus)
#define inline __inline
#endif
#endif

#if defined(__linux)
/* to use non-standard GNU-extenstions */
#define _GNU_SOURCE
#endif

#include <stdint.h>

//#if defined(_WIN32) || defined(__CYGWIN__) || defined(_WIN32_WCE)
//#define XTRX_API WINAPI
//#else
//#define XTRX_API
//#endif

#include <sys/types.h>

#if defined(__linux) || defined(__APPLE__) || defined(__CYGWIN__)
#include <sys/time.h>
#include <unistd.h>
#include <endian.h>
#endif

#include <time.h>
#include <limits.h>
#include <errno.h>

#if defined(__linux) || defined(__APPLE__)
#include <pthread.h>
#include <semaphore.h>
#elif defined(_WIN32) || defined(_WIN32_WCE) || defined(__CYGWIN__)
#include <windows.h>
/** @ingroup threading
 * Basic emulation of posix threading and mutexes
 */

typedef CRITICAL_SECTION pthread_mutex_t;

static inline int pthread_mutex_init(pthread_mutex_t *__mutex, const void *__mutexattr) {
	InitializeCriticalSection(__mutex);
	return 0;
}

static inline int pthread_mutex_destroy(pthread_mutex_t *__mutex) {
	DeleteCriticalSection(__mutex);
	return 0;
}

static inline int pthread_mutex_lock(pthread_mutex_t *__mutex) {
	EnterCriticalSection(__mutex);
	return 0;
}

static inline int pthread_mutex_unlock(pthread_mutex_t *__mutex) {
	LeaveCriticalSection(__mutex);
	return 0;
}

typedef HANDLE pthread_t;

int pthread_create(pthread_t *__restrict nt,
				   const void *__restrict __attr,
				   void *(*start_routine) (void *),
				   void *__restrict arg);

int pthread_join(pthread_t th, void **thread_return);

typedef HANDLE sem_t;
int sem_init(sem_t *sem, int pshared, unsigned int value);
int sem_destroy(sem_t *sem);

int sem_wait(sem_t *sem);
int sem_post(sem_t *sem);

/** @ingroup timing
 * Basic timing function with high resolution
 */

#ifdef _MSC_VER
struct timespec {
	time_t   tv_sec;
	unsigned tv_nsec;
};

typedef enum {
	CLOCK_REALTIME = 0,
	CLOCK_MONOTONIC_RAW = 0, /**< Use Realtime as the last resort */
} clockid_t;
#else
#define CLOCK_MONOTONIC_RAW CLOCK_REALTIME
#endif

int clock_gettime(clockid_t clk_id, struct timespec *tp);

int usleep(unsigned usec);

unsigned sleep(unsigned seconds);

int sem_timedwait(sem_t *sem, const struct timespec *abs_timeout);

/** @ingroup dynamic
 * Emulation of dl functions
 */

#define RTLD_LOCAL	0
#define RTLD_NOW	2

void *dlopen(const char *file, int mode);

int dlclose(void *handle);

void *dlsym(void *__restrict handle,
			const char *__restrict name);

char *dlerror(void);


/** @ingroup endian
 */
static inline uint32_t __bswap_16 (uint16_t __bsx)
{
	return (__bsx >> 8) | (__bsx << 8);
}

static inline uint32_t __bswap_32 (uint32_t __bsx)
{
	return __builtin_bswap32(__bsx);
}

static inline uint64_t __bswap_64 (uint64_t __bsx)
{
	return __builtin_bswap64(__bsx);
}

#  define htobe16(x) __bswap_16 (x)
#  define htole16(x) (x)
#  define be16toh(x) __bswap_16 (x)
#  define le16toh(x) (x)

#  define htobe32(x) __bswap_32 (x)
#  define htole32(x) (x)
#  define be32toh(x) __bswap_32 (x)
#  define le32toh(x) (x)

#  define htobe64(x) __bswap_64 (x)
#  define htole64(x) (x)
#  define be64toh(x) __bswap_64 (x)
#  define le64toh(x) (x)

/** @ingroup memory
 */
static inline int posix_memalign(void **memptr, size_t alignment, size_t size) {
	*memptr = _aligned_malloc(size, alignment);
	return (*memptr == NULL) ? -1 : 0;
}

// TODO replace this constant
#define ENAVAIL 1000

#else
#error Unknown platform
#endif

/** @ingroup general
 * General portability layes
 */

#if defined(__cplusplus)
extern "C" {
#endif

const struct tm* xtrxll_localtime(time_t now);


#if defined(__cplusplus)
}
#endif

#endif
