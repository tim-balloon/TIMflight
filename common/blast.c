/* blast.c: the BLAST flight code common functions
 *
 * This software is copyright (C) 2004-2010 University of Toronto
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdlib.h>    /* ANSI C std library (exit, realloc, free) */
#include <errno.h>     /* ANSI C library errors (errno) */
#include <pthread.h>   /* POSIX threads (pthread_exit) */
#include <stdio.h>     /* ANSI C std io library (FILE, stderr, stdout) */
#include <string.h>    /* ANSI C strings (strlen, strncat, strdup, &c.) */
#include <syslog.h>    /* BSD system logger (openlog, syslog, closelog) */

#include "blast.h"

/* BUOS (BLAST Unified Output Scheme) */
static int __buos_disable_exit = 0;
static int __buos_allow_mem = 0;
static void (*__buos_real_bputs)(buos_t, const char*) = NULL;

void bputs_stdio(buos_t l, const char* s)
{
  FILE* stream = stderr;

  switch (l) {
    case none:
      return;
    case mem:
      if (l == mem && !__buos_allow_mem)
        return;
    case info:
    case startup:
    case sched:
      stream = stdout;
    case warning:
    case err:
    case tfatal:
    case fatal:
      fputs(s, stream);
      if (s[strlen(s) - 1] != '\n')
        fputs("\n", stream);
  }

  if (!__buos_disable_exit) {
    if (l == fatal)
      exit(1);
    else if (l == tfatal)
      pthread_exit(NULL);
  }
}

void bputs_syslog(buos_t l, const char* s)
{
    int level = LOG_INFO;

    switch (l) {
        case none:
            return;
        case info:
            level = LOG_INFO;
            break;
        case warning:
            level = LOG_WARNING;
            break;
        case err:
            level = LOG_ERR;
            break;
        case tfatal:
            level = LOG_CRIT;
            break;
        case fatal:
            level = LOG_ALERT;
            break;
        case startup:
            level = LOG_NOTICE;
            break;
        case mem:
        case sched:
            level = LOG_DEBUG;
            break;
    }

    if (l != mem || __buos_allow_mem) syslog(level, "%s", s);

    if (!__buos_disable_exit) {
        if (l == fatal)
            exit(1);
        else if (l == tfatal) pthread_exit(NULL);
    }
}

void bputs(buos_t l, const char* s)
{
    if (__buos_real_bputs)
        (*__buos_real_bputs)(l, s);
    else
        bputs_stdio(l, s);
}

void bprintf(buos_t l, const char* fmt, ...)
{
    char message[BUOS_MAX];
    va_list argptr;

    va_start(argptr, fmt);
    vsnprintf(message, BUOS_MAX, fmt, argptr);
    va_end(argptr);

    bputs(l, message);
}

void berror(buos_t l, const char* fmt, ...)
{
    char message[BUOS_MAX];
    va_list argptr;
    int error = errno;

    va_start(argptr, fmt);
    vsnprintf(message, BUOS_MAX, fmt, argptr);
    va_end(argptr);

    /* add a colon */
    strncat(message, ": ", BUOS_MAX - strlen(message));

    message[BUOS_MAX - 1] = '\0';
    /* copy error message into remainder of string -- Note: sterror is reentrant
     * despite what strerror(3) insinuates (and strerror_r is horribly b0rked) */
    strncat(message, strerror(error), BUOS_MAX - strlen(message));
    message[BUOS_MAX - 1] = '\0';

    bputs(l, message);
}

void buos_allow_mem(void)
{
    __buos_allow_mem = 1;
}

void buos_disallow_mem(void)
{
    __buos_allow_mem = 0;
}

void buos_disable_exit(void)
{
    __buos_disable_exit = 1;
}

void buos_enable_exit(void)
{
    __buos_disable_exit = 0;
}

void buos_use_func(void (*puts_func)(buos_t, const char*))
{
    __buos_real_bputs = puts_func;
}

void buos_use_syslog(void)
{
    __buos_real_bputs = bputs_syslog;
}

void buos_use_stdio(void)
{
    __buos_real_bputs = bputs_stdio;
}

/* BLAMM (BLAST Memory Manager) definitions */
void *_balloc(buos_t l, size_t s, const char* f, int w, const char* n)
{
  void *p;

  p = malloc(s);
  blast_mem("malloced %u bytes in %s as %p", (unsigned)s, f, p);

  if (p == NULL)
    berror(l, "unable to malloc %u bytes at %s:%i in %s", (unsigned)s, n, w, f);

  return p;
}

void *_reballoc(buos_t l, void* p, size_t s, const char* f, int w,
    const char* n)
{
  void *q;

  q = realloc(p, s);
  blast_mem("realloced %u bytes from %p in %s as %p", (unsigned)s, p, f, q);

  if (q == NULL)
    berror(l, "unable to realloc %u bytes from %p at %s:%i in %s", (unsigned)s,
        p, n, w, f);

  return q;
}

void _bfree(buos_t l, void* p, const char* f, int w, const char* n)
{
  blast_mem("freeing %p in %s", p, f);
  free(p);
}

char* _bstrdup(buos_t l, const char* s, const char* f, int w, const char* n)
{
  char *q;

  q = strdup(s);
  blast_mem("strduped `%s' in %s as %p", s, f, q);

  if (q == NULL)
    berror(l, "unable to strdup `%s' at %s:%i in %s", s, n, w, f);

  return q;
}

void _basprintf(buos_t l, char **m_dest, const char *m_fmt, const char *m_fn, int m_line, const char *m_file, ...)
{
    va_list argptr;
    int retval;

    va_start(argptr, m_file);
    retval = vasprintf(m_dest, m_fmt, argptr);
    va_end(argptr);

    if (retval < 0) {
        berror(l, "%s:%d (%s): Unable to create string with format '%s'", m_fn, m_line, m_file, m_fmt);
        *m_dest = NULL;
    }
}

char* _bstrndup(buos_t m_level, const char* m_src, size_t m_len, const char* m_fnname, int m_lineno,
        const char* m_filename)
{
    char *dest_ptr;

    size_t len = strnlen(m_src, m_len) + 1;
    dest_ptr = _balloc(m_level, len, m_filename, m_lineno, m_fnname);

    blast_mem("strnduped `%s' in %s as %p", m_src, m_fnname, dest_ptr);

    if (dest_ptr == NULL)
        berror(m_level, "unable to strndup `%s' at %s:%i in %s", m_src, m_filename, m_lineno, m_fnname);
    else
        dest_ptr[len - 1] = '\0';

    return (char*) memcpy(dest_ptr, m_src, len);
}

void *_memdup(buos_t l, const void *m_src, size_t n, const char* m_func, int m_line, const char *m_file)
{
    void *dest;

    dest = _balloc(l, n, m_func, m_line, m_file);
    if (dest == NULL)
        bprintf(l, "Previous balloc error originated from _memdup");
    else
        memcpy(dest, m_src, n);
    return dest;
}

