/****************************************************************************
 * boards/xtensa/esp32s3/lilygo-tdeck-max/src/esp32s3_radios.c
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
 * Wi-Fi and Bluetooth LE are not brought up at boot unless the board is
 * configured to (LILYGO_TDECK_MAX_BOOT_WIFI, LILYGO_TDECK_MAX_BOOT_BLE).  A
 * handheld spends most of its life with the radios idle, and an idle radio
 * is not free: initialising them costs a large part of the internal heap,
 * and the network stack starts the Wi-Fi radio itself when it brings the
 * interface up.
 *
 * Each radio is instead offered as a device that behaves like the board's
 * other power rails:
 *
 *   nsh> gpio -o 1 /dev/wifi_en      bring Wi-Fi up, then "ifup wlan0"
 *   nsh> gpio -o 1 /dev/ble_en       bring BLE up, then "ifup bnep0"
 *
 * Bringing a radio up is a one way trip: the drivers register network
 * devices that they have no way to remove again, so there is nothing to
 * tear down.  A Wi-Fi radio that is up can still be stopped and started
 * with ifdown and ifup.
 *
 * Once a radio is up, the chip stays out of light sleep: the Bluetooth
 * controller holds it awake itself, and for Wi-Fi this file does.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <syslog.h>

#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/power/pm.h>
#include <nuttx/semaphore.h>
#include <nuttx/ioexpander/gpio.h>

#ifdef CONFIG_ESPRESSIF_WIFI
#  include "esp32s3_board_wlan.h"
#endif

#ifdef CONFIG_ESPRESSIF_BLE
#  include "esp32s3_ble.h"
#endif

#ifdef CONFIG_ESPRESSIF_WIFI_BT_COEXIST
#  include "esp32s3_wifi_adapter.h"
#endif

#include "lilygo-tdeck-max.h"

#ifdef CONFIG_ESPRESSIF_WIRELESS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Stack for the thread that runs a radio's initialisation, and the least
 * the board's own init thread needs when it does the same at boot.
 *
 * The init thread used to be given 2 KB, copied from an upstream Wi-Fi
 * configuration, and that only worked by a hair.  The radio drivers' setup
 * is deep, and once the call chain got a few frames longer the board
 * panicked at boot: the init thread's thread-local block sits at the very
 * bottom of its stack, so it is the first thing to be trampled, and the
 * damage only shows when the thread exits and NuttX walks the list in it.
 * Syslog goes to a buffer only, so nothing is printed; the board just
 * resets in a loop and looks dead.  Refuse to build that instead.
 */

#define TDECK_RADIO_STACKSIZE 4096

#if (defined(CONFIG_LILYGO_TDECK_MAX_BOOT_WIFI) || \
     defined(CONFIG_LILYGO_TDECK_MAX_BOOT_BLE)) && \
    defined(CONFIG_BOARD_INITTHREAD_STACKSIZE) && \
    CONFIG_BOARD_INITTHREAD_STACKSIZE < TDECK_RADIO_STACKSIZE
#  error "Radios at boot need BOARD_INITTHREAD_STACKSIZE >= 4096, or the board panics in a silent reset loop"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tdeck_radio_s
{
  struct gpio_dev_s gpio;     /* Must be first */
  FAR const char   *name;     /* Device name under /dev */
  CODE int        (*start)(void);
  FAR bool         *enabled;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  tdeck_coex_prepare(void);
static int  tdeck_run_isolated(CODE int (*start)(void));
static int  tdeck_enable(CODE int (*start)(void), FAR bool *enabled,
                         bool isolate);

#ifdef CONFIG_ESPRESSIF_WIFI
static int  tdeck_wifi_start(void);
#endif

#ifdef CONFIG_ESPRESSIF_BLE
static int  tdeck_ble_start(void);
#endif

#ifdef CONFIG_DEV_GPIO
static int  tdeck_radio_read(FAR struct gpio_dev_s *dev,
                             FAR bool *value);
static int  tdeck_radio_write(FAR struct gpio_dev_s *dev, bool value);
static int  tdeck_radio_setpintype(FAR struct gpio_dev_s *dev,
                                   enum gpio_pintype_e pintype);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Serialises bring-up, and protects the flags below */

static mutex_t g_radio_lock = NXMUTEX_INITIALIZER;

#ifdef CONFIG_ESPRESSIF_WIFI
static bool g_wifi_enabled;
PM_WAKELOCK_DECLARE_STATIC(g_wifi_wakelock, "wifi", PM_IDLE_DOMAIN, PM_IDLE);
#endif

#ifdef CONFIG_ESPRESSIF_BLE
static bool g_ble_enabled;
#endif

/* The bring-up that a helper thread is to run, and its outcome.  Only one
 * exists at a time because the caller holds g_radio_lock throughout.
 */

static struct
{
  CODE int (*start)(void);
  sem_t    done;
  int      result;
} g_job;

#ifdef CONFIG_DEV_GPIO
static const struct gpio_operations_s g_radio_ops =
{
  .go_read       = tdeck_radio_read,
  .go_write      = tdeck_radio_write,
  .go_setpintype = tdeck_radio_setpintype,
};

static struct tdeck_radio_s g_radios[] =
{
#ifdef CONFIG_ESPRESSIF_WIFI
  {
    .gpio    =
    {
      .gp_pintype = GPIO_OUTPUT_PIN,
      .gp_ops     = &g_radio_ops,
    },
    .name    = "wifi_en",
    .start   = tdeck_wifi_start,
    .enabled = &g_wifi_enabled,
  },
#endif

#ifdef CONFIG_ESPRESSIF_BLE
  {
    .gpio    =
    {
      .gp_pintype = GPIO_OUTPUT_PIN,
      .gp_ops     = &g_radio_ops,
    },
    .name    = "ble_en",
    .start   = tdeck_ble_start,
    .enabled = &g_ble_enabled,
  },
#endif
};

#define TDECK_NRADIOS (sizeof(g_radios) / sizeof(g_radios[0]))
#endif /* CONFIG_DEV_GPIO */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_coex_prepare
 *
 * Description:
 *   Start the Wi-Fi/BT coexistence arbiter.  It has to be running before
 *   either radio is initialised, and it must only be started once, so
 *   whichever radio is brought up first does it.
 *
 ****************************************************************************/

static int tdeck_coex_prepare(void)
{
#ifdef CONFIG_ESPRESSIF_WIFI_BT_COEXIST
  static bool ready;
  int ret;

  if (!ready)
    {
      ret = esp_wifi_bt_coexist_init();
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to init Wi-Fi/BT coexistence: %d\n",
                 ret);
          return ret;
        }

      ready = true;
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: tdeck_wifi_start
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_WIFI
static int tdeck_wifi_start(void)
{
  static bool tried;
  int ret;

  ret = tdeck_coex_prepare();
  if (ret < 0)
    {
      return ret;
    }

  /* A bring-up that failed part way cannot be repeated: the driver's own
   * state is left behind
   */

  if (tried)
    {
      syslog(LOG_ERR, "ERROR: Wi-Fi failed to come up; reset to retry\n");
      return -EIO;
    }

  tried = true;

  /* Initialises the Wi-Fi driver, with its task and buffers, and registers
   * wlan0.  The radio itself only starts when wlan0 is brought up.
   */

  ret = board_wlan_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize Wi-Fi: %d\n", ret);
      return ret;
    }

  /* The Wi-Fi driver does not keep the chip awake while its radio runs */

  pm_wakelock_stay(&g_wifi_wakelock);

  g_wifi_enabled = true;
  return OK;
}
#endif

/****************************************************************************
 * Name: tdeck_ble_start
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_BLE
static int tdeck_ble_start(void)
{
  static bool tried;
  int ret;

  ret = tdeck_coex_prepare();
  if (ret < 0)
    {
      return ret;
    }

  /* A bring-up that failed part way cannot be repeated: the controller
   * only initialises once
   */

  if (tried)
    {
      syslog(LOG_ERR, "ERROR: BLE failed to come up; reset to retry\n");
      return -EIO;
    }

  tried = true;

  /* Enables the controller and registers the bnep0 network device */

  ret = esp32s3_ble_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize BLE: %d\n", ret);
      return ret;
    }

  g_ble_enabled = true;
  return OK;
}
#endif

/****************************************************************************
 * Name: tdeck_radio_thread
 ****************************************************************************/

static int tdeck_radio_thread(int argc, FAR char *argv[])
{
  g_job.result = g_job.start();
  nxsem_post(&g_job.done);
  return OK;
}

/****************************************************************************
 * Name: tdeck_run_isolated
 *
 * Description:
 *   Run a bring-up on a thread of its own and wait for it, so that it does
 *   not depend on how much stack the caller happens to have.  The caller
 *   holds g_radio_lock.
 *
 ****************************************************************************/

static int tdeck_run_isolated(CODE int (*start)(void))
{
  int pid;

  g_job.start  = start;
  g_job.result = -EIO;
  nxsem_init(&g_job.done, 0, 0);
  nxsem_set_protocol(&g_job.done, SEM_PRIO_NONE);

  pid = kthread_create("radio_init", SCHED_PRIORITY_DEFAULT,
                       TDECK_RADIO_STACKSIZE, tdeck_radio_thread,
                       NULL);
  if (pid < 0)
    {
      nxsem_destroy(&g_job.done);
      return pid;
    }

  nxsem_wait_uninterruptible(&g_job.done);
  nxsem_destroy(&g_job.done);
  return g_job.result;
}

/****************************************************************************
 * Name: tdeck_enable
 *
 * Description:
 *   Bring a radio up if it is not up already.  Asking again once it is up
 *   is harmless; once a bring-up has failed part way, the radio needs a
 *   reset.
 *
 ****************************************************************************/

static int tdeck_enable(CODE int (*start)(void), FAR bool *enabled,
                        bool isolate)
{
  int ret;

  ret = nxmutex_lock(&g_radio_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (*enabled)
    {
      ret = OK;
    }
  else
    {
      ret = isolate ? tdeck_run_isolated(start) : start();
    }

  nxmutex_unlock(&g_radio_lock);
  return ret;
}

#ifdef CONFIG_DEV_GPIO

/****************************************************************************
 * Name: tdeck_radio_read
 ****************************************************************************/

static int tdeck_radio_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct tdeck_radio_s *radio = (FAR struct tdeck_radio_s *)dev;

  *value = *radio->enabled;
  return OK;
}

/****************************************************************************
 * Name: tdeck_radio_write
 ****************************************************************************/

static int tdeck_radio_write(FAR struct gpio_dev_s *dev, bool value)
{
  FAR struct tdeck_radio_s *radio = (FAR struct tdeck_radio_s *)dev;

  if (value)
    {
      return tdeck_enable(radio->start, radio->enabled, true);
    }

  /* There is no way back down, so only report success if that is a no-op */

  return *radio->enabled ? -ENOTSUP : OK;
}

/****************************************************************************
 * Name: tdeck_radio_setpintype
 ****************************************************************************/

static int tdeck_radio_setpintype(FAR struct gpio_dev_s *dev,
                                  enum gpio_pintype_e pintype)
{
  return pintype == GPIO_OUTPUT_PIN ? OK : -EINVAL;
}

#endif /* CONFIG_DEV_GPIO */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_wifi_enable
 *
 * Description:
 *   See src/lilygo-tdeck-max.h
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_WIFI
int tdeck_wifi_enable(void)
{
  return tdeck_enable(tdeck_wifi_start, &g_wifi_enabled, false);
}
#endif

/****************************************************************************
 * Name: tdeck_ble_enable
 *
 * Description:
 *   See src/lilygo-tdeck-max.h
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_BLE
int tdeck_ble_enable(void)
{
  return tdeck_enable(tdeck_ble_start, &g_ble_enabled, false);
}
#endif

/****************************************************************************
 * Name: tdeck_radios_initialize
 *
 * Description:
 *   See src/lilygo-tdeck-max.h
 *
 ****************************************************************************/

int tdeck_radios_initialize(void)
{
#ifdef CONFIG_DEV_GPIO
  int ret;
  int i;

  for (i = 0; i < TDECK_NRADIOS; i++)
    {
      ret = gpio_pin_register_byname(&g_radios[i].gpio, g_radios[i].name);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register /dev/%s: %d\n",
                 g_radios[i].name, ret);
        }
    }
#endif

  /* BLE goes first, as it always has: the order the radios were brought up
   * in when both were unconditional is the one that has been tested.
   */

#ifdef CONFIG_LILYGO_TDECK_MAX_BOOT_BLE
  tdeck_ble_enable();
#endif

#ifdef CONFIG_LILYGO_TDECK_MAX_BOOT_WIFI
  tdeck_wifi_enable();
#endif

  return OK;
}

#endif /* CONFIG_ESPRESSIF_WIRELESS */
