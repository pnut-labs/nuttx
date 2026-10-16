/****************************************************************************
 * boards/xtensa/esp32s3/lilygo-tdeck-max/src/esp32s3_gnss.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * The u-blox MIA-M10Q GNSS receiver talks NMEA on UART1 at 38400 baud.  This
 * is the lower half of NuttX's GNSS upper half, which parses the NMEA into
 * the uORB topics sensor_gnss and sensor_gnss_satellite and passes the raw
 * sentences through /dev/ttyGNSS0.
 *
 * The receiver is only powered while something uses it: the first user
 * switches the GPS rail on, opens the UART and wakes a thread that feeds
 * its data to the upper half.  A few seconds after the last user has gone,
 * the thread parks, the UART is closed and the rail is switched off; the
 * delay spares the receiver a cold start when a program closes and reopens
 * it straight away, as uorb_listener does.  The thread is made once, at
 * start-up, and parked while the receiver is off.
 *
 * The UART is held as a kernel file, not a descriptor, so that a write to
 * /dev/ttyGNSS0 from any task reaches the receiver.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/gnss.h>
#include <nuttx/wqueue.h>

#include "lilygo-tdeck-max.h"

#ifdef CONFIG_SENSORS_GNSS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_LPWORK
#  error "The GNSS glue needs the low priority work queue (SCHED_LPWORK)"
#endif

#ifndef CONFIG_ESP32S3_UART1
#  error "The GNSS receiver is on UART1 (ESP32S3_UART1)"
#endif

/* UART1 is /dev/ttyS0 while UART0 is not used: the console is the USB
 * Serial/JTAG port
 */

#ifdef CONFIG_ESP32S3_UART0
#  define GNSS_UART_PATH     "/dev/ttyS1"
#else
#  define GNSS_UART_PATH     "/dev/ttyS0"
#endif

#define GNSS_THREAD_PRIORITY 110
#define GNSS_THREAD_STACK    3072
#define GNSS_WAIT_MS         500            /* A safety net for the wake-ups */
#define GNSS_INTERVAL_US     1000000        /* The receiver's default 1 Hz */
#define GNSS_OFF_DELAY       SEC2TICK(5)    /* Power stays on after the last user */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tdeck_gnss_s
{
  struct gnss_lowerhalf_s lower;            /* Must be first */
  struct file uart;                         /* The receiver, while powered */
  mutex_t statelock;                        /* Serialises power changes */
  sem_t run;                                /* Unparks the thread */
  sem_t wake;                               /* Data, or a stop */
  sem_t parked;                             /* Posted as the thread parks */
  struct work_s offwork;                    /* Powers off after the delay */
  volatile bool running;                    /* Cleared to stop the thread */
  bool stopping;                            /* Waiting for the thread */
  bool active;                              /* Someone uses the receiver */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     tdeck_gnss_activate(FAR struct gnss_lowerhalf_s *lower,
                                   FAR struct file *filep, bool enable);
static int     tdeck_gnss_set_interval(FAR struct gnss_lowerhalf_s *lower,
                                       FAR struct file *filep,
                                       FAR uint32_t *period_us);
static ssize_t tdeck_gnss_inject(FAR struct gnss_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR const void *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gnss_ops_s g_gnss_ops =
{
  .activate     = tdeck_gnss_activate,
  .set_interval = tdeck_gnss_set_interval,
  .inject_data  = tdeck_gnss_inject,
};

static struct tdeck_gnss_s g_gnss =
{
  .lower =
    {
      .ops = &g_gnss_ops,
    },
  .statelock = NXMUTEX_INITIALIZER,
  .run       = SEM_INITIALIZER(0),
  .wake      = SEM_INITIALIZER(0),
  .parked    = SEM_INITIALIZER(0),
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_gnss_thread
 *
 * Description:
 *   Parked until the receiver is started, then feed its output to the
 *   upper half until told to stop, and park again.  The UART is read
 *   without blocking, whenever its poll says there is data, or every
 *   GNSS_WAIT_MS if it cannot be polled.
 *
 ****************************************************************************/

static int tdeck_gnss_thread(int argc, FAR char *argv[])
{
  FAR struct tdeck_gnss_s *priv = &g_gnss;
  struct pollfd fds;
  char buffer[128];
  ssize_t nread;
  bool polled;
  int ret;

  for (; ; )
    {
      nxsem_wait_uninterruptible(&priv->run);

      memset(&fds, 0, sizeof(fds));
      fds.events = POLLIN;
      fds.arg    = &priv->wake;
      fds.cb     = poll_default_cb;

      ret = file_poll(&priv->uart, &fds, true);
      polled = ret >= 0;
      if (!polled)
        {
          snerr("ERROR: Cannot poll %s (%d): reading every %d ms\n",
                GNSS_UART_PATH, ret, GNSS_WAIT_MS);
        }

      while (priv->running)
        {
          nxsem_tickwait(&priv->wake, MSEC2TICK(GNSS_WAIT_MS));
          fds.revents = 0;

          while (priv->running &&
                 (nread = file_read(&priv->uart, buffer,
                                    sizeof(buffer))) > 0)
            {
              priv->lower.push_data(priv->lower.priv, buffer, nread, true);
            }
        }

      /* Before the UART is closed */

      if (polled)
        {
          file_poll(&priv->uart, &fds, false);
        }

      nxsem_post(&priv->parked);
    }

  return 0;
}

/****************************************************************************
 * Name: tdeck_gnss_start
 *
 * Description:
 *   Unpark the thread, with the rail on and the UART open.  Called with
 *   statelock held.
 *
 ****************************************************************************/

static void tdeck_gnss_start(FAR struct tdeck_gnss_s *priv)
{
  priv->running = true;
  nxsem_post(&priv->run);
}

/****************************************************************************
 * Name: tdeck_gnss_off
 *
 * Description:
 *   Work queue job, some time after the last user has gone: park the
 *   thread, close the UART and switch the receiver off, unless it is in
 *   use again.
 *
 *   The thread is waited for without statelock: it may be pushing to a
 *   topic whose lock a task opening that topic holds while it waits for
 *   statelock in activate.  A user that comes meanwhile finds the receiver
 *   stopping; it is started again here once the thread has parked.
 *
 ****************************************************************************/

static void tdeck_gnss_off(FAR void *arg)
{
  FAR struct tdeck_gnss_s *priv = arg;
  bool stop = false;

  nxmutex_lock(&priv->statelock);
  if (!priv->active && priv->running)
    {
      priv->running  = false;
      priv->stopping = true;
      nxsem_post(&priv->wake);
      stop = true;
    }

  nxmutex_unlock(&priv->statelock);

  if (!stop)
    {
      return;
    }

  nxsem_wait_uninterruptible(&priv->parked);

  nxmutex_lock(&priv->statelock);
  priv->stopping = false;

  if (priv->active)
    {
      tdeck_gnss_start(priv);
    }
  else
    {
      file_close(&priv->uart);
      if (tdeck_xl9555_write(XL9555_GPS_EN, false) < 0)
        {
          snerr("ERROR: Cannot switch the GNSS receiver off\n");
        }
    }

  nxmutex_unlock(&priv->statelock);
}

/****************************************************************************
 * Name: tdeck_gnss_activate
 *
 * Description:
 *   Called by the upper half for the first user and after the last.  It
 *   never waits for the thread: the upper half calls it with its own lock,
 *   and a topic's, held.
 *
 ****************************************************************************/

static int tdeck_gnss_activate(FAR struct gnss_lowerhalf_s *lower,
                               FAR struct file *filep, bool enable)
{
  FAR struct tdeck_gnss_s *priv = (FAR struct tdeck_gnss_s *)lower;
  int ret = OK;

  nxmutex_lock(&priv->statelock);

  priv->active = enable;
  if (!enable)
    {
      work_queue(LPWORK, &priv->offwork, tdeck_gnss_off, priv,
                 GNSS_OFF_DELAY);
    }
  else if (!priv->running && !priv->stopping)
    {
      ret = tdeck_xl9555_write(XL9555_GPS_EN, true);
      if (ret >= 0)
        {
          ret = file_open(&priv->uart, GNSS_UART_PATH,
                          O_RDWR | O_NONBLOCK | O_CLOEXEC);
          if (ret < 0)
            {
              snerr("ERROR: Failed to open %s: %d\n", GNSS_UART_PATH, ret);
            }
          else
            {
              tdeck_gnss_start(priv);
            }

          if (ret < 0)
            {
              tdeck_xl9555_write(XL9555_GPS_EN, false);
            }
        }
    }

  nxmutex_unlock(&priv->statelock);
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: tdeck_gnss_set_interval
 ****************************************************************************/

static int tdeck_gnss_set_interval(FAR struct gnss_lowerhalf_s *lower,
                                   FAR struct file *filep,
                                   FAR uint32_t *period_us)
{
  /* The receiver reports once a second; changing that takes UBX
   * configuration, which can be sent with inject_data (a write to
   * /dev/ttyGNSS0).
   */

  *period_us = GNSS_INTERVAL_US;
  return OK;
}

/****************************************************************************
 * Name: tdeck_gnss_inject
 ****************************************************************************/

static ssize_t tdeck_gnss_inject(FAR struct gnss_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR const void *buffer, size_t buflen)
{
  FAR struct tdeck_gnss_s *priv = (FAR struct tdeck_gnss_s *)lower;
  ssize_t ret = -EAGAIN;

  /* Commands for the receiver (NMEA or UBX) go straight to its UART, open
   * while the writer holds /dev/ttyGNSS0 open.  The UART does not block: a
   * command longer than its buffer, or sent while it drains, may be cut
   * short, and the count says so.
   */

  nxmutex_lock(&priv->statelock);
  if (priv->running || priv->stopping)
    {
      ret = file_write(&priv->uart, buffer, buflen);
    }

  nxmutex_unlock(&priv->statelock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_gnss_initialize
 *
 * Description:
 *   See src/lilygo-tdeck-max.h
 *
 ****************************************************************************/

int tdeck_gnss_initialize(void)
{
  uint32_t nbuffer[SENSOR_GNSS_IDX_GNSS_MAX] =
    {
      1, 1, 1, 1, 1
    };

  int ret;

  /* The upper half first, so that no thread is left behind if registration
   * fails.  Nothing can start the receiver before the thread exists, as
   * bring-up runs before any task; a start would only leave run posted,
   * for the thread to take when it runs.
   */

  ret = gnss_register(&g_gnss.lower, 0, nbuffer, SENSOR_GNSS_IDX_GNSS_MAX);
  if (ret < 0)
    {
      return ret;
    }

  /* The thread, parked until the receiver is started */

  ret = kthread_create("gnss", GNSS_THREAD_PRIORITY, GNSS_THREAD_STACK,
                       tdeck_gnss_thread, NULL);
  if (ret < 0)
    {
      gnss_unregister(&g_gnss.lower, 0);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_SENSORS_GNSS */
