/****************************************************************************
 * include/nuttx/input/cst3530.h
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

#ifndef __INCLUDE_NUTTX_INPUT_CST3530_H
#define __INCLUDE_NUTTX_INPUT_CST3530_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/irq.h>

#ifdef CONFIG_INPUT_CST3530

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The controller's I2C address, 7 bits */

#define CST3530_ADDRESS     0x1a

/* Touch keys the controller can report, by id */

#define CST3530_MAX_TKEYS   16

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* How the board's controller is wired, and what its touch keys mean.
 *
 * The touch keys, keys drawn on the glass outside the screen, are optional.
 * With tkeys_path set, they are registered there as a keyboard: the key
 * with id n reports tkeys_codes[n], a special key from enum kbd_keycode_e,
 * as KEYBOARD_SPECPRESS and KEYBOARD_SPECREL, for n below ntkeys.  The
 * board gives the codes, so that it decides what the keys are.
 */

struct cst3530_config_s
{
  uint32_t frequency;     /* I2C frequency */
  uint8_t  address;       /* I2C address, 7 bits */
  uint8_t  flags;         /* TOUCH_FLAG_SWAPXY, _MIRRORX, _MIRRORY */

  /* The touch keys, if any */

  FAR const char     *tkeys_path;
  FAR const uint32_t *tkeys_codes;
  uint8_t             ntkeys;

  /* The interrupt line, pulled low for each report: attach the handler,
   * with the interrupt left disabled; enable or disable it.  The driver
   * masks it until it has read a report, so it may be level triggered.
   */

  CODE int  (*attach)(FAR const struct cst3530_config_s *config,
                      xcpt_t isr, FAR void *arg);
  CODE void (*enable)(FAR const struct cst3530_config_s *config,
                      bool enable);

  /* The reset line: true holds the controller in reset */

  CODE int  (*reset)(FAR const struct cst3530_config_s *config,
                     bool assert);
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
 * Name: cst3530_register
 *
 * Description:
 *   Register a Hynitron CST3530, or a CST66xx that speaks its protocol, as
 *   a touchscreen at devpath (for example "/dev/input0"), and its touch
 *   keys, if the board has any.  The resolution is read from the
 *   controller.
 *
 *   The controller sleeps while neither device is open: the first open
 *   resets it into normal operation, and the last close puts it back into
 *   deep sleep.
 *
 * Input Parameters:
 *   i2c     - The I2C bus the controller is on.
 *   config  - The board's wiring and touch keys; kept, not copied.
 *   devpath - The touchscreen's path.
 *
 * Returned Value:
 *   Zero (OK) on success; -ENODEV if no controller of this family answers;
 *   another negated errno value on failure.
 *
 ****************************************************************************/

int cst3530_register(FAR struct i2c_master_s *i2c,
                     FAR const struct cst3530_config_s *config,
                     FAR const char *devpath);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_INPUT_CST3530 */
#endif /* __INCLUDE_NUTTX_INPUT_CST3530_H */
