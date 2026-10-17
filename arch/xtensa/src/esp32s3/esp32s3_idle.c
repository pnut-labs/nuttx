/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_idle.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/debug.h>
#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/power/pm.h>
#include <nuttx/spinlock.h>

#include "xtensa.h"
#include "espressif/esp_pm.h"

#ifdef CONFIG_ESP32S3_DFS
#  include "esp_private/pm_impl.h"
#endif

#if defined(CONFIG_PM) && defined(CONFIG_SCHED_TICKLESS)
#  include "esp32s3_tickless.h"
#  ifdef CONFIG_ESPRESSIF_HR_TIMER
#    include "esp_timer.h"
#  endif
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Values for the RTC Alarm to wake up from the PM_STANDBY mode
 * (which corresponds to ESP32-C3 stop mode).  If this alarm expires,
 * the logic in this file will wakeup from PM_STANDBY mode and
 * transition to PM_SLEEP mode (ESP32-C3 standby mode).
 */

#ifdef CONFIG_PM
#ifndef CONFIG_PM_ALARM_SEC
#  define CONFIG_PM_ALARM_SEC 15
#endif

#ifndef CONFIG_PM_ALARM_NSEC
#  define CONFIG_PM_ALARM_NSEC 0
#endif

#ifndef CONFIG_PM_SLEEP_WAKEUP_SEC
#  define CONFIG_PM_SLEEP_WAKEUP_SEC 20
#endif

#ifndef CONFIG_PM_SLEEP_WAKEUP_NSEC
#  define CONFIG_PM_SLEEP_WAKEUP_NSEC 0
#endif

/* The longest PM_STANDBY, in microseconds */

#define ESP32S3_STANDBY_MAX_US \
  ((uint64_t)CONFIG_PM_ALARM_SEC * 1000000 + CONFIG_PM_ALARM_NSEC / 1000)

/* With the tickless scheduler, PM_STANDBY ends this much before the next
 * timer is due, so that the timer is not late when waking up takes a
 * little longer than the HAL expects.
 */

#define ESP32S3_STANDBY_EARLY_US 500

#if defined(CONFIG_SCHED_TICKLESS) && \
    CONFIG_ESP32S3_PM_STANDBY_MIN_US <= ESP32S3_STANDBY_EARLY_US
#  error "ESP32S3_PM_STANDBY_MIN_US must be longer than 500 us"
#endif

#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_PM
static spinlock_t g_esp32s3_idle_lock = SP_UNLOCKED;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_standby_time
 *
 * Description:
 *   How long PM_STANDBY may last, in microseconds: until the scheduler's
 *   next timer or the next esp_timer alarm, whichever comes first, and no
 *   longer than PM_ALARM_SEC and PM_ALARM_NSEC.  The call that decides
 *   the sleep is made with interrupts disabled, so that nothing can become
 *   due between it and the sleep.
 *
 ****************************************************************************/

#if defined(CONFIG_PM) && defined(CONFIG_SCHED_TICKLESS)
static uint64_t up_standby_time(void)
{
  uint64_t sleep_us = ESP32S3_STANDBY_MAX_US;
  uint64_t next;
#ifdef CONFIG_ESPRESSIF_HR_TIMER
  int64_t alarm;
  int64_t now;
#endif

  next = esp32s3_tickless_next();
  if (next < sleep_us)
    {
      sleep_us = next;
    }

#ifdef CONFIG_ESPRESSIF_HR_TIMER
  alarm = esp_timer_get_next_alarm_for_wake_up();
  if (alarm != INT64_MAX)
    {
      now = esp_timer_get_time();
      if (alarm <= now)
        {
          return 0;
        }

      if ((uint64_t)(alarm - now) < sleep_us)
        {
          sleep_us = alarm - now;
        }
    }
#endif

  return sleep_us;
}
#endif

/****************************************************************************
 * Name: up_idlepm
 *
 * Description:
 *   Perform IDLE state power management.
 *
 * Returned Value:
 *   True if the chip slept; false if the caller is to wait for an
 *   interrupt.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static bool up_idlepm(void)
{
  irqstate_t flags;
  static enum pm_state_e oldstate = PM_NORMAL;
  enum pm_state_e newstate;
  bool slept = false;
  int ret;
  int count;

  count = pm_staycount(PM_IDLE_DOMAIN, PM_NORMAL);
  if (oldstate != PM_NORMAL && count == 0)
    {
      pm_stay(PM_IDLE_DOMAIN, PM_NORMAL);

      /* Keep working in normal stage */

      pm_changestate(PM_IDLE_DOMAIN, PM_NORMAL);

      /* Release the stay above: it only forces this one state change. */

      pm_relax(PM_IDLE_DOMAIN, PM_NORMAL);

      /* Without this, oldstate goes stale after the first wakeup and the
       * state machine wedges in PM_NORMAL forever -- see commit message.
       */

      oldstate = PM_NORMAL;
    }

  /* Decide, which power saving level can be obtained */

  newstate = pm_checkstate(PM_IDLE_DOMAIN);

#ifdef CONFIG_SCHED_TICKLESS
  /* With the tickless scheduler PM_STANDBY lasts until the next timer:
   * not worth it when that is close.
   */

  if (newstate == PM_STANDBY &&
      up_standby_time() < CONFIG_ESP32S3_PM_STANDBY_MIN_US)
    {
      newstate = PM_IDLE;
    }
#endif

  /* Check for state changes */

  if (newstate != oldstate)
    {
      flags = spin_lock_irqsave(&g_esp32s3_idle_lock);

      /* Perform board-specific, state-dependent logic here */

      _info("newstate= %d oldstate=%d\n", newstate, oldstate);

      /* Then force the global state change */

      ret = pm_changestate(PM_IDLE_DOMAIN, newstate);
      if (ret < 0)
        {
          /* The new state change failed, revert to the preceding state */

          pm_changestate(PM_IDLE_DOMAIN, oldstate);
        }
      else
        {
          /* Save the new state */

          oldstate = newstate;
        }

      spin_unlock_irqrestore(&g_esp32s3_idle_lock, flags);

      /* A driver refused the new state: wait for an interrupt instead */

      if (ret < 0)
        {
          return false;
        }

      /* MCU-specific power management logic */

      switch (newstate)
        {
          case PM_NORMAL:
            break;

          case PM_IDLE:
            break;

          case PM_STANDBY:
            {
#ifdef CONFIG_SCHED_TICKLESS
              uint64_t standby_us;

              /* Light sleep until the next timer, with interrupts off from
               * the decision on.  Only the wake-up sources end the sleep:
               * an interrupt raised meanwhile stays pending and is
               * serviced after the wake-up.  A thread that ran since
               * pm_checkstate() above may have taken a hold, so check
               * again.
               */

              flags = up_irq_save();

              standby_us = up_standby_time();
              if (pm_checkstate(PM_IDLE_DOMAIN) >= PM_STANDBY &&
                  standby_us >= CONFIG_ESP32S3_PM_STANDBY_MIN_US)
                {
                  esp_pmstandby(standby_us - ESP32S3_STANDBY_EARLY_US);

                  /* The counter was moved on by the time slept: an alarm
                   * it passed without reaching is set again.
                   */

                  esp32s3_tickless_resync();
                  slept = true;
                }

              up_irq_restore(flags);
#else
              /* Enter Force-sleep mode */

              esp_pmstandby(ESP32S3_STANDBY_MAX_US);
              slept = true;
#endif

              /* Without this, /proc/pm/state0 bills sleep time to wake[]
               * -- pm_stats() needs PM_RESTORE to know time was asleep.
               */

              if (slept)
                {
                  pm_changestate(PM_IDLE_DOMAIN, PM_RESTORE);
                }
            }
            break;

          case PM_SLEEP:
            {
              /* Enter Deep-sleep mode */

              esp_pmsleep(CONFIG_PM_SLEEP_WAKEUP_SEC * 1000000 +
                          CONFIG_PM_SLEEP_WAKEUP_NSEC / 1000);
            }

          default:
            break;
        }
    }
  else
    {
#ifdef CONFIG_WATCHDOG
      /* Announce the power management state change to feed watchdog */

      pm_changestate(PM_IDLE_DOMAIN, PM_NORMAL);
#endif
    }

  return slept;
}
#else
#  define up_idlepm() false
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when their is no other
 *   ready-to-run task.  This is processor idle time and will continue until
 *   some interrupt occurs to cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g., this is where
 *   power management operations might be performed.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_idle(void)
{
  /* Report loop explanation here */

#ifdef CONFIG_ESP32S3_SPEED_UP_ISR
  for (; ; )
    {
#endif

#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
      /* If the system is idle and there are no timer interrupts, then
       * process "fake" timer interrupts. Hopefully, something will wake up.
       */

      nxsched_process_timer();
#else

      /* Perform IDLE mode power management first: waiting for an interrupt
       * before it would keep the chip awake until the next one, and with
       * the tickless scheduler that is the next timer.  When it did not
       * sleep, wait for an interrupt in a reduced power mode.
       */

      if (!up_idlepm())
        {
#  ifdef CONFIG_ESP32S3_DFS
          /* Drop to the idle frequency.  The interrupt that ends the wait
           * raises it again.
           */

          esp_pm_impl_idle_hook();
#  endif

#  if XCHAL_HAVE_INTERRUPTS
          __asm__ __volatile__ ("waiti 0");
#  endif
        }
#endif /* CONFIG_SUPPRESS_INTERRUPTS || CONFIG_SUPPRESS_TIMER_INTS */

#ifdef CONFIG_ESP32S3_SPEED_UP_ISR
    }
#endif
}
