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
 * Name: tdeck_battery_initialize
 *
 * Description:
 *   Register the fuel gauge as /dev/batt0 and the charger as /dev/charger0.
 *
 ****************************************************************************/

#if defined(CONFIG_BQ27220) || defined(CONFIG_SY6970)
struct i2c_master_s;
int tdeck_battery_initialize(FAR struct i2c_master_s *i2c);
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

/****************************************************************************
 * Name: tdeck_lora_initialize
 *
 * Description:
 *   Register the SX1262 as /dev/lora0 (NuttX's SX126x driver).  The radio
 *   stays asleep while the device is closed.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_ESP32S3_SPI2) && defined(CONFIG_LPWAN_SX126X)
int tdeck_lora_initialize(void);
#endif

/****************************************************************************
 * Name: tdeck_gnss_initialize
 *
 * Description:
 *   Register the MIA-M10Q GNSS receiver with NuttX's GNSS upper half: the
 *   uORB topics sensor_gnss and sensor_gnss_satellite, and the raw NMEA
 *   stream /dev/ttyGNSS0.  The receiver is powered while any of them is in
 *   use.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_SENSORS_GNSS) && defined(CONFIG_IOEXPANDER_PCA9555)
int tdeck_gnss_initialize(void);
#endif

/****************************************************************************
 * Name: tdeck_audio_initialize
 *
 * Description:
 *   Register the ES8311 codec: /dev/audio/pcm0 for playback, through the
 *   PCM decoder, with the speaker's amplifier on while it is reserved, and
 *   /dev/audio/pcm_in0 for recording from the microphone.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_AUDIO_ES8311) && defined(CONFIG_ESPRESSIF_I2S0) && \
    defined(CONFIG_IOEXPANDER_PCA9555)
int tdeck_audio_initialize(void);
#endif

/****************************************************************************
 * Name: tdeck_haptic_initialize
 *
 * Description:
 *   Register the DRV2605L behind the vibration motor as /dev/input_ff0.
 *   The XL9555 powers it, so this runs after the XL9555 is set up.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_FF_DRV2605) && defined(CONFIG_IOEXPANDER_PCA9555)
int tdeck_haptic_initialize(void);
#endif

/****************************************************************************
 * Name: tdeck_imu_initialize
 *
 * Description:
 *   Register the BHI260AP as the uORB topics sensor_accel0 and
 *   sensor_gyro0.  The XL9555 powers it, so this runs after the XL9555 is
 *   set up.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BHI260AP
int tdeck_imu_initialize(void);
#endif

/****************************************************************************
 * Name: tdeck_radios_initialize
 *
 * Description:
 *   Register /dev/wifi_en and /dev/ble_en, which bring Wi-Fi and Bluetooth
 *   LE up when 1 is written to them, and bring up those that the board is
 *   configured to start at boot (LILYGO_TDECK_MAX_BOOT_WIFI,
 *   LILYGO_TDECK_MAX_BOOT_BLE).
 *
 * Returned Value:
 *   Zero (OK).
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_WIRELESS
int tdeck_radios_initialize(void);
#endif

/****************************************************************************
 * Name: tdeck_wifi_enable, tdeck_ble_enable
 *
 * Description:
 *   Bring Wi-Fi or Bluetooth LE up, if it is not up already.  There is no
 *   way back down.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_WIFI
int tdeck_wifi_enable(void);
#endif

#ifdef CONFIG_ESPRESSIF_BLE
int tdeck_ble_enable(void);
#endif

/****************************************************************************
 * Name: board_spiflash_init
 *
 * Description:
 *   Register the internal flash's system partition and mount its file
 *   system (boards/xtensa/esp32s3/common/src/esp32s3_board_spiflash.c).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPIFLASH
int board_spiflash_init(void);
#endif

/****************************************************************************
 * Name: tdeck_modem_initialize
 *
 * Description:
 *   Set up the modem's ring indicator and DTR, and register them as
 *   /dev/modem_ri and /dev/modem_dtr.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_modem_listen
 *
 * Description:
 *   Listen to the modem's ring indicator while the modem's rail is on, or
 *   no longer: RI then wakes the chip and keeps it awake for a while.
 *
 ****************************************************************************/

#if defined(CONFIG_DEV_GPIO) && defined(CONFIG_IOEXPANDER_PCA9555)
int tdeck_modem_initialize(void);
void tdeck_modem_listen(bool on);
#else
#  define tdeck_modem_listen(on)
#endif

/****************************************************************************
 * Name: tdeck_power_initialize
 *
 * Description:
 *   Register /dev/awake, which keeps the chip out of light sleep while it
 *   is open, and allow light sleep: called once the board is up.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void tdeck_power_initialize(void);
#endif

/****************************************************************************
 * Name: tdeck_power_wake
 *
 * Description:
 *   Make a GPIO wake the chip from light sleep while it is at the level
 *   given (high or low), or no longer.  For the drivers' interrupt enable
 *   hooks: a call from an interrupt handler, a driver masking its
 *   interrupt until its worker has run, leaves the pin waking the chip.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void tdeck_power_wake(int pin, bool high, bool enable);
#else
#  define tdeck_power_wake(pin, high, enable)
#endif

/****************************************************************************
 * Name: tdeck_lights_initialize
 *
 * Description:
 *   Register the front light and the keyboard's backlight as /dev/pwm0,
 *   keeping the chip out of light sleep while one of them is on: the LED
 *   controller stops in light sleep.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_PM) && defined(CONFIG_ESPRESSIF_LEDC)
int tdeck_lights_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_XTENSA_ESP32S3_LILYGO_TDECK_MAX_SRC_LILYGO_TDECK_MAX_H */
