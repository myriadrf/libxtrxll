/*
 * Public logging source file
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
#include "xtrxll_log.h"
#include <time.h>
#include <stdarg.h>
#include <unistd.h>

enum xtrxll_loglevel s_loglevel = XTRXLL_DEBUG_REGS;

static FILE* s_logfile;
static int s_colorize;

static const char* s_term_name[] =
{
	"\033[0m",
	"\033[0;31mERROR:   ",
	"\033[0;32mWARNING: ",
	"\033[0;33mINFO:    ",
	"\033[0;34mRFIC:    ",
	"\033[0;35mDEBUG:   ",
	"\033[0;36mREGS:    ",
	"\033[0;37mTRACE:   ",
};

static void s_def_logging(int sevirity, const char* message)
{
	(void)sevirity;
	fputs(message, s_logfile);
}

static logfunc_t s_log_function = s_def_logging;

void xtrxll_set_logfunc(logfunc_t function)
{
	s_log_function = (function) ? function : s_def_logging;
}

#define MAX_LOG_LINE 1024
void xtrxll_log(enum xtrxll_loglevel l, const char* func, int line,
				const char* fmt, ...)
{
	(void)func;
	(void)line;

	if (s_loglevel < l)
		return;

	va_list ap;
	struct timespec tp;
	char buf[MAX_LOG_LINE];
	int sz;
	size_t stsz;

	va_start(ap, fmt);

	clock_gettime(CLOCK_REALTIME, &tp);
	stsz = strftime(buf, sizeof(buf), "%H:%M:%S.", xtrxll_localtime(tp.tv_sec));
	sz = snprintf(buf + stsz - 1, sizeof(buf) - stsz, ".%06d %s",
				  (int)(tp.tv_nsec/1000),
				  s_term_name[l] + ((s_colorize > 0) ? 0 : 7));
	if (sz < 0) {
		buf[MAX_LOG_LINE - 1] = 0;
		goto out_truncated;
	}
	stsz += (size_t)sz;
	sz = vsnprintf(buf + stsz - 1, sizeof(buf) - stsz, fmt, ap);
	if (sz < 0) {
		buf[MAX_LOG_LINE - 1] = 0;
		goto out_truncated;
	}
	stsz += (size_t)sz;
	if (s_colorize) {
		sz = snprintf(buf + stsz - 1, sizeof(buf) - stsz, "%s", s_term_name[0]);
		if (sz < 0) {
			buf[MAX_LOG_LINE - 1] = 0;
		}
	}

out_truncated:
	s_log_function(l, buf);
	va_end(ap);
}


void xtrxll_log_initialize(FILE* logfile)
{
	if (logfile == NULL) {
		if (s_logfile == NULL) {
			s_logfile = stderr;
		}
	} else {
		s_logfile = logfile;
	}

	s_colorize = isatty(fileno(s_logfile));
}

void xtrxll_set_loglevel(enum xtrxll_loglevel l)
{
	s_loglevel = l;
}

enum xtrxll_loglevel xtrxll_get_loglevel(void)
{
	return s_loglevel;
}
