/****************************************************************************
 * include/nuttx/sensors/bhi260ap.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_BHI260AP_H
#define __INCLUDE_NUTTX_SENSORS_BHI260AP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/irq.h>

#ifdef CONFIG_SENSORS_BHI260AP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The I2C address with HSDO/SA0 low; 0x29 with it high */

#define BHI260AP_I2C_ADDRESS 0x28

/* The names of the topics of the firmware's sensors that uORB has no type
 * for: /dev/uorb/sensor_<name><devno>, of struct sensor_event
 */

#define BHI260AP_STATIONARY  "stationary"   /* Still for a while: event 1 */
#define BHI260AP_ACTIVITY    "activity"     /* Activities started and
                                             * ended: event the bits below */

/* Activities, and their bits in an activity event */

#define BHI260AP_ACTIVITY_STILL       0
#define BHI260AP_ACTIVITY_WALKING     1
#define BHI260AP_ACTIVITY_RUNNING     2
#define BHI260AP_ACTIVITY_BICYCLE     3
#define BHI260AP_ACTIVITY_VEHICLE     4
#define BHI260AP_ACTIVITY_TILTING     5

#define BHI260AP_ACTIVITY_ENDED(a)    (1 << (a))
#define BHI260AP_ACTIVITY_STARTED(a)  (1 << ((a) + 8))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Board hooks.  The structure must stay valid for as long as the driver is
 * registered.
 */

struct bhi260ap_config_s
{
  uint32_t frequency;              /* I2C frequency */

  /* The firmware image to run from the chip's program RAM, as Bosch
   * distributes it (it starts with the magic bytes 0x2b 0x66).  The chip
   * has no firmware of its own: the image is uploaded when a sensor is
   * first activated, which takes a few seconds.
   */

  FAR const uint8_t *firmware;
  size_t firmware_size;

  /* Attach isr to the host interrupt (HIRQ), which is high while the chip
   * has data for the host (active high and level triggered, the chip's
   * defaults).  The interrupt is left disabled.
   */

  CODE int  (*attach)(FAR const struct bhi260ap_config_s *config,
                      xcpt_t isr, FAR void *arg);

  /* Enable or disable the interrupt */

  CODE void (*enable)(FAR const struct bhi260ap_config_s *config,
                      bool enable);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct i2c_master_s;

/****************************************************************************
 * Name: bhi260ap_register
 *
 * Description:
 *   Register a Bosch BHI260AP smart sensor hub as uORB topics, <devno>
 *   after each name:
 *
 *     sensor_accel, sensor_gyro   its accelerometer in m/s^2 and its
 *                                 gyroscope in rad/s
 *     sensor_tilt_detector        its angle changed: event 1
 *     sensor_step_counter         steps since the firmware started
 *     sensor_glance_gesture       lifted to be looked at   } event 1
 *     sensor_pickup_gesture       picked up                }
 *     sensor_significant_motion   moving: the hub's motion detector (id
 *                                 77, not its significant motion, 55),
 *                                 uORB's SENSOR_TYPE_MOTION_DETECT,
 *                                 which NuttX's table names so
 *     sensor_stationary           still (BHI260AP_STATIONARY)
 *     sensor_activity             activities (BHI260AP_ACTIVITY)
 *
 *   All but the first two are worked out by the firmware, and only some
 *   images have them: one the image lacks fails to activate (open) with
 *   -ENODEV.  The chip holds events for as long as a topic's batch latency
 *   allows (SNIOC_BATCH), up to the ioctl's 2^32 us (about 71 minutes).
 *
 * Input Parameters:
 *   devno  - The instance number of the topics
 *   i2c    - An instance of the I2C interface to use
 *   addr   - The I2C address, normally BHI260AP_I2C_ADDRESS
 *   config - Board hooks, kept by the driver
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure, -ENODEV if no
 *   BHI260 answers.
 *
 ****************************************************************************/

int bhi260ap_register(int devno, FAR struct i2c_master_s *i2c, uint8_t addr,
                      FAR const struct bhi260ap_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_BHI260AP */
#endif /* __INCLUDE_NUTTX_SENSORS_BHI260AP_H */
