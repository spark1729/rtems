/**
 * @file
 *
 * @brief Waiting on a Condition
 * @ingroup POSIXAPI
 */

/*
 *  COPYRIGHT (c) 1989-2007.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#if HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems/posix/condimpl.h>

/*
 *  11.4.4 Waiting on a Condition, P1003.1c/Draft 10, p. 105
 */

int pthread_cond_wait(
  pthread_cond_t     *cond,
  pthread_mutex_t    *mutex
)
{
  return _POSIX_Condition_variables_Wait_support(
    cond,
    mutex,
    WATCHDOG_NO_TIMEOUT,
    false
  );
}
