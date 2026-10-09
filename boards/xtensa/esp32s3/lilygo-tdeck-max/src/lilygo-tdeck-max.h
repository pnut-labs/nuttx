/****************************************************************************
 * boards/xtensa/esp32s3/lilygo-tdeck-max/src/lilygo-tdeck-max.h
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

#ifndef __BOARDS_XTENSA_ESP32S3_LILYGO_TDECK_MAX_SRC_LILYGO_TDECK_MAX_H
#define __BOARDS_XTENSA_ESP32S3_LILYGO_TDECK_MAX_SRC_LILYGO_TDECK_MAX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The XL9555's lines: 0 to 7 are P00 to P07, 8 to 15 are P10 to P17 */

#define XL9555_MODEM_PWR      0   /* High powers the A7682E */
#define XL9555_LORA_EN        1   /* High powers the SX1262 */
#define XL9555_GPS_EN         2   /* High powers the MIA-M10Q */
#define XL9555_IMU_1V8_EN     3   /* High powers the BHI260AP's 1.8 V */
#define XL9555_LORA_ANT       4   /* High selects the internal antenna */
#define XL9555_MOTOR_EN       5   /* High powers the DRV2605L */
#define XL9555_AMP_EN         6   /* High enables the speaker amplifier */
#define XL9555_TOUCH_RST      7   /* Low resets the CST3530 */
#define XL9555_MODEM_PWRKEY   8   /* High presses the modem's power key */
#define XL9555_KEY_RST        9   /* Low resets the TCA8418 */
#define XL9555_AUDIO_SEL      10  /* High: the modem's audio; low: codec */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: esp32s3_bringup
 *
 * Description:
 *   Bring up the board's devices.  Called from board_late_initialize()
 *   (CONFIG_BOARD_LATE_INITIALIZE=y).
 *
 ****************************************************************************/

int esp32s3_bringup(void);

/****************************************************************************
 * Name: tdeck_xl9555_initialize
 *
 * Description:
 *   Set every line of the XL9555 I/O expander to its level at start, and
 *   register each as /dev/<name>.
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_PCA9555
struct i2c_master_s;
int tdeck_xl9555_initialize(FAR struct i2c_master_s *i2c);
#endif

/****************************************************************************
 * Name: tdeck_xl9555_write
 *
 * Description:
 *   Set one of the XL9555's lines (XL9555_*), for a driver on the board.
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_PCA9555
int tdeck_xl9555_write(uint8_t pin, bool on);
#endif

/****************************************************************************
 * Name: tdeck_keyboard_initialize
 *
 * Description:
 *   Register the keyboard as /dev/kbd0, once the XL9555 has released its
 *   reset.
 *
 ****************************************************************************/

#ifdef CONFIG_INPUT_TCA8418
struct i2c_master_s;
int tdeck_keyboard_initialize(FAR struct i2c_master_s *i2c);
#endif

/****************************************************************************
 * Name: tdeck_touch_initialize
 *
 * Description:
 *   Register the touch screen as /dev/input0, and the glass keys below it
 *   as /dev/softkeys, once the XL9555 has released the controller's reset.
 *
 ****************************************************************************/

#if defined(CONFIG_INPUT_CST3530) && defined(CONFIG_IOEXPANDER_PCA9555)
struct i2c_master_s;
int tdeck_touch_initialize(FAR struct i2c_master_s *i2c);
#endif

/****************************************************************************
 * Name: tdeck_lora_sleep
 *
 * Description:
 *   Put the SX1262 to sleep, once its rail is on.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI2
int tdeck_lora_sleep(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_XTENSA_ESP32S3_LILYGO_TDECK_MAX_SRC_LILYGO_TDECK_MAX_H */
