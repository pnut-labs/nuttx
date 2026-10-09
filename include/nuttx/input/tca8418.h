/****************************************************************************
 * include/nuttx/input/tca8418.h
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

#ifndef __INCLUDE_NUTTX_INPUT_TCA8418_H
#define __INCLUDE_NUTTX_INPUT_TCA8418_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/irq.h>
#include <nuttx/input/kbd_codec.h>

#ifdef CONFIG_INPUT_TCA8418

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The controller scans up to 8 rows by 10 columns */

#define TCA8418_MAX_ROWS    8
#define TCA8418_MAX_COLS    10

/* A keymap entry: a character (below 0x100), a special key from enum
 * kbd_keycode_e, a modifier, or nothing.
 */

#define TCA8418_NONE        0x0000
#define TCA8418_SHIFT       0x0100
#define TCA8418_ALT         0x0101
#define TCA8418_SPEC(k)     (0x0200 + (k))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* How the board's keyboard is wired and what its keys mean.
 *
 * Shift and Alt each select a layer of the keymap, as printed on the keys:
 * held, for the keys pressed meanwhile; tapped, for the next key; tapped
 * twice, until tapped again.  A key a layer leaves empty, or a layer left
 * NULL, falls back to the base layer.  Modifiers are found in the base
 * layer only.
 *
 * Characters reach the reader as KEYBOARD_PRESS and KEYBOARD_RELEASE, and
 * special keys as KEYBOARD_SPECPRESS and KEYBOARD_SPECREL; a release
 * reports what its press did, whatever the modifiers are by then.  The
 * modifiers themselves are reported as KEYCODE_LSHIFT and KEYCODE_LALT,
 * pressed when the modifier takes effect and released when it ends, held,
 * tapped or locked alike: a reader learns what the keys in between were
 * pressed with, for chords of its own.
 */

struct tca8418_config_s
{
  uint32_t frequency;     /* I2C frequency */
  uint8_t  address;       /* I2C address, 7 bits */
  uint8_t  rows;          /* Rows in use, 1 to TCA8418_MAX_ROWS */
  uint8_t  cols;          /* Columns in use, 1 to TCA8418_MAX_COLS */

  /* Column 0 of the keymap is the matrix's last column: keyboards are
   * often wired so, and the keymap then reads as the keys are laid out.
   */

  bool     colreverse;

  /* The layers, rows * cols entries each, row by row; base is required */

  FAR const uint16_t *base;
  FAR const uint16_t *shift;
  FAR const uint16_t *alt;

  /* The interrupt line, low while events wait: attach the handler, with
   * the interrupt left disabled; enable or disable it.
   */

  CODE int  (*attach)(FAR const struct tca8418_config_s *config, xcpt_t isr,
                      FAR void *arg);
  CODE void (*enable)(FAR const struct tca8418_config_s *config,
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
 * Name: tca8418_register
 *
 * Description:
 *   Set up the controller to scan the board's matrix, and register the
 *   keyboard at devname (for example "/dev/kbd0").
 *
 * Input Parameters:
 *   i2c     - The I2C bus the controller is on.
 *   config  - The board's wiring and keymap; kept, not copied.
 *   devname - The keyboard's path.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int tca8418_register(FAR struct i2c_master_s *i2c,
                     FAR const struct tca8418_config_s *config,
                     FAR const char *devname);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_INPUT_TCA8418 */
#endif /* __INCLUDE_NUTTX_INPUT_TCA8418_H */
