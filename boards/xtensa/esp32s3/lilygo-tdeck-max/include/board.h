/****************************************************************************
 * boards/xtensa/esp32s3/lilygo-tdeck-max/include/board.h
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

#ifndef __BOARDS_XTENSA_ESP32S3_LILYGO_TDECK_MAX_INCLUDE_BOARD_H
#define __BOARDS_XTENSA_ESP32S3_LILYGO_TDECK_MAX_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The T-Deck Max is fitted with a 40 MHz crystal */

#define BOARD_XTAL_FREQUENCY      40000000

#ifdef CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ
#  define BOARD_CLOCK_FREQUENCY   (CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ * 1000000)
#else
#  define BOARD_CLOCK_FREQUENCY   80000000
#endif

/* GPIO pins ****************************************************************/

/* The I2C bus, shared by the keyboard, the touch controller, the I/O
 * expander, the charger, the fuel gauge, the codec's control port, the
 * motor driver and the IMU.
 */

#define BOARD_I2C_SDA             13
#define BOARD_I2C_SCL             14

/* The SPI bus, shared by the e-paper, the microSD card and the SX1262,
 * and their chip selects.
 */

#define BOARD_SPI_SCK             36
#define BOARD_SPI_MOSI            33
#define BOARD_SPI_MISO            47

#define BOARD_EPD_CS              34
#define BOARD_SD_CS               48
#define BOARD_LORA_CS             3

/* The e-paper (UC8253) and its front light */

#define BOARD_EPD_DC              35    /* High selects data */
#define BOARD_EPD_BUSY            37
#define BOARD_EPD_RST             9     /* Active low */
#define BOARD_EPD_FRONTLIGHT      41

/* The LoRa radio (SX1262) */

#define BOARD_LORA_RST            4     /* NRESET, active low */
#define BOARD_LORA_DIO1           5     /* The radio's interrupt */
#define BOARD_LORA_BUSY           6

/* The GNSS receiver (MIA-M10Q), on UART1 */

#define BOARD_GNSS_TX             16    /* To the receiver's RX */
#define BOARD_GNSS_RX             2     /* From the receiver's TX */
#define BOARD_GNSS_PPS            1

/* The modem (A7682E), on UART2 */

#define BOARD_MODEM_TX            10    /* To the modem's RXD */
#define BOARD_MODEM_RX            11    /* From the modem's TXD */
#define BOARD_MODEM_RI            7
#define BOARD_MODEM_DTR           8

/* Inputs, the keyboard's backlight and the IMU's interrupt */

#define BOARD_TOUCH_INT           12
#define BOARD_KEYBOARD_INT        15
#define BOARD_KEYBOARD_BACKLIGHT  42
#define BOARD_IMU_INT             21
#define BOARD_BOOT_BUTTON         0     /* Low when pressed */

/* The audio codec's I2S (ES8311) */

#define BOARD_I2S_MCLK            38
#define BOARD_I2S_BCLK            39
#define BOARD_I2S_WS              18
#define BOARD_I2S_DOUT            40    /* To the codec's DSDIN */
#define BOARD_I2S_DIN             17    /* From the codec's ASDOUT */

#endif /* __BOARDS_XTENSA_ESP32S3_LILYGO_TDECK_MAX_INCLUDE_BOARD_H */
