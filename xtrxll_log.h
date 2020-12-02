/*
 * Public logging header file
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
#ifndef XTRXLL_LOG_H
#define XTRXLL_LOG_H

#include "xtrxll_port.h"
#include <stdio.h>
#include <stdarg.h>
#include <time.h>

enum xtrxll_loglevel {
	XTRXLL_NONE,
	XTRXLL_ERROR,
	XTRXLL_WARNING,
	XTRXLL_INFO,
	XTRXLL_INFO_LMS,
	XTRXLL_DEBUG,
	XTRXLL_DEBUG_REGS,
	XTRXLL_PARANOIC,
};

/** Global application logging level */
extern enum xtrxll_loglevel s_loglevel;

void xtrxll_log(enum xtrxll_loglevel l,
				const char subsystem[4],
				const char* function,
				const char* file,
				int line,
				const char* fmt, ...)  __attribute__ ((format (printf, 6, 7)));
void xtrxll_vlog(enum xtrxll_loglevel l,
				 const char subsystem[4],
				 const char* function,
				 const char* file,
				 int line,
				 const char* fmt, va_list list);

void xtrxll_log_initialize(FILE* logfile);

void xtrxll_set_loglevel(enum xtrxll_loglevel l);
enum xtrxll_loglevel xtrxll_get_loglevel(void);

typedef void (*logfunc_t)(int severity,
						  const struct tm* stm,
						  int nsec,
						  const char subsystem[4],
						  const char* function,
						  const char* file,
						  int function_lno,
						  const char* fmt,
						  va_list list);

void xtrxll_set_logfunc(logfunc_t function);


#define XTRXLLS_LOG(sys, level,...) \
	do { \
		if (s_loglevel >= (level)) \
			xtrxll_log((level), (sys), __FUNCTION__, __FILE__, __LINE__, __VA_ARGS__); \
	} while (0)

#define XTRXLL_LOG(level, ...) XTRXLLS_LOG("DEF", level, __VA_ARGS__)

#endif
