/****************************************************************************
 * boards/xtensa/esp32s3/lilygo-tdeck-max/bootloader/progress.c
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

/* A swap's progress on the e-paper panel, drawn by MCUboot
 * (CONFIG_ESP32S3_MCUBOOT_PROGRESS): built into the bootloader, never into
 * NuttX, so it has only Espressif's HAL and the chip's ROM.  The panel's
 * bus is driven by hand: there is no room for a frame buffer, so each row
 * is made as it is sent.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#include <bootutil/bootutil_log.h>
#include <bootutil/bootutil_public.h>
#include <mcuboot_config/mcuboot_config.h>
#include <esp_rom_gpio.h>
#include <esp_rom_sys.h>
#include <hal/gpio_ll.h>
#include <soc/gpio_sig_map.h>
#include <soc/gpio_struct.h>

#include "progress_text.h"        /* The lines, drawn beforehand (mktext.py) */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The board's lines (include/board.h, src/lilygo-tdeck-max.h) */

#define PIN_SCK         36
#define PIN_MOSI        33
#define PIN_EPD_CS      34
#define PIN_EPD_DC      35    /* High selects data */
#define PIN_EPD_BUSY    37    /* Low while busy */
#define PIN_EPD_RST     9     /* Low resets */
#define PIN_SD_CS       48
#define PIN_LORA_CS     3
#define PIN_SDA         13
#define PIN_SCL         14

/* The XL9555's port 0: its line 1 powers the LoRa radio */

#define XL9555_ADDRESS  0x20
#define XL9555_OUTPUT0  0x02
#define XL9555_CONFIG0  0x06  /* A set bit is an input */
#define XL9555_LORA_EN  0x02

/* The panel: UC8253, 240 x 320, one bit a pixel, a set bit white */

#define EPD_XRES        240
#define EPD_YRES        320

#define EPD_PSR         0x00
#define EPD_POWER_OFF   0x02
#define EPD_POWER_ON    0x04
#define EPD_DTM1        0x10
#define EPD_DRF         0x12
#define EPD_DTM2        0x13
#define EPD_CDI         0x50
#define EPD_CCSET       0xe0
#define EPD_TSSET       0xe5

#define EPD_POWER_MS    500
#define EPD_DRAW_MS     8000

#define EPD_PTL         0x90  /* Partial window */
#define EPD_PTIN        0x91
#define EPD_PTOUT       0x92

/* The screen, as the device's interface design draws a task that takes a
 * while: a title, a bar of ten blocks, a line below, from the left margin.
 * The blocks fill one at a time, and only the bar is refreshed, partially,
 * in a window aligned to bytes.
 */

#define TEXT_X          12
#define TITLE_Y         124   /* The title's top */
#define CAPTION_Y       172
#define BAR_X           12
#define BAR_Y           156
#define BAR_W           216
#define BAR_H           10
#define BAR_GAP         2     /* Between blocks */
#define BAR_BLOCKS      10
#define WIN_X0          8     /* The bar's window */
#define WIN_X1          232
#define WIN_Y0          BAR_Y
#define WIN_Y1          (BAR_Y + BAR_H)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct text_s *g_title;

static bool g_shown;          /* The screen drawn, at this start */
static bool g_drawing;        /* A partial refresh started, not finished */
static int g_fill;            /* The blocks the panel shows, or is drawing */
static uint32_t g_tenth;      /* The last tenth logged */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Lines */

static void pin_out(int pin, int level)
{
  esp_rom_gpio_pad_select_gpio(pin);
  gpio_ll_set_level(&GPIO, pin, level);
  esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
  gpio_ll_output_enable(&GPIO, pin);
}

static void pin_in(int pin)
{
  esp_rom_gpio_pad_select_gpio(pin);
  gpio_ll_output_disable(&GPIO, pin);
  gpio_ll_input_enable(&GPIO, pin);
  esp_rom_gpio_pad_pullup_only(pin);
}

/* I2C, by hand, slowly: an open-drain line is released (an input, pulled
 * up) or driven low
 */

static void i2c_line(int pin, int level)
{
  if (level)
    {
      gpio_ll_output_disable(&GPIO, pin);
    }
  else
    {
      gpio_ll_set_level(&GPIO, pin, 0);
      gpio_ll_output_enable(&GPIO, pin);
    }

  esp_rom_delay_us(5);
}

static void i2c_start(void)
{
  i2c_line(PIN_SDA, 1);
  i2c_line(PIN_SCL, 1);
  i2c_line(PIN_SDA, 0);
  i2c_line(PIN_SCL, 0);
}

static void i2c_stop(void)
{
  i2c_line(PIN_SDA, 0);
  i2c_line(PIN_SCL, 1);
  i2c_line(PIN_SDA, 1);
}

static bool i2c_send(uint8_t byte)
{
  bool ack;
  int i;

  for (i = 7; i >= 0; i--)
    {
      i2c_line(PIN_SDA, (byte >> i) & 1);
      i2c_line(PIN_SCL, 1);
      i2c_line(PIN_SCL, 0);
    }

  i2c_line(PIN_SDA, 1);
  i2c_line(PIN_SCL, 1);
  ack = gpio_ll_get_level(&GPIO, PIN_SDA) == 0;
  i2c_line(PIN_SCL, 0);
  return ack;
}

static uint8_t i2c_receive(void)
{
  uint8_t byte = 0;
  int i;

  i2c_line(PIN_SDA, 1);
  for (i = 0; i < 8; i++)
    {
      i2c_line(PIN_SCL, 1);
      byte = (byte << 1) | gpio_ll_get_level(&GPIO, PIN_SDA);
      i2c_line(PIN_SCL, 0);
    }

  i2c_line(PIN_SDA, 1);       /* No acknowledge: the last byte */
  i2c_line(PIN_SCL, 1);
  i2c_line(PIN_SCL, 0);
  return byte;
}

static bool xl9555_read(uint8_t reg, uint8_t *value)
{
  bool ok;

  i2c_start();
  ok = i2c_send(XL9555_ADDRESS << 1) && i2c_send(reg);
  if (ok)
    {
      i2c_start();
      ok = i2c_send((XL9555_ADDRESS << 1) | 1);
      if (ok)
        {
          *value = i2c_receive();
        }
    }

  i2c_stop();
  return ok;
}

static bool xl9555_write(uint8_t reg, uint8_t value)
{
  bool ok;

  i2c_start();
  ok = i2c_send(XL9555_ADDRESS << 1) && i2c_send(reg) && i2c_send(value);
  i2c_stop();
  return ok;
}

/* The LoRa radio powered: unpowered, it loads the bus the panel shares.
 * After a reset NuttX left it on; after power-up the XL9555's lines are
 * inputs.  Only its line changes: the modem's stay as they are.
 */

static void lora_power(void)
{
  uint8_t out;
  uint8_t cfg;

  esp_rom_gpio_pad_select_gpio(PIN_SDA);
  esp_rom_gpio_pad_select_gpio(PIN_SCL);
  esp_rom_gpio_connect_out_signal(PIN_SDA, SIG_GPIO_OUT_IDX, false, false);
  esp_rom_gpio_connect_out_signal(PIN_SCL, SIG_GPIO_OUT_IDX, false, false);
  gpio_ll_input_enable(&GPIO, PIN_SDA);
  gpio_ll_input_enable(&GPIO, PIN_SCL);
  esp_rom_gpio_pad_pullup_only(PIN_SDA);
  esp_rom_gpio_pad_pullup_only(PIN_SCL);
  i2c_line(PIN_SDA, 1);
  i2c_line(PIN_SCL, 1);

  if (!xl9555_read(XL9555_OUTPUT0, &out) ||
      !xl9555_read(XL9555_CONFIG0, &cfg))
    {
      BOOT_LOG_WRN("Progress: no XL9555");
      return;
    }

  if ((out & XL9555_LORA_EN) != 0 && (cfg & XL9555_LORA_EN) == 0)
    {
      return;
    }

  if (!xl9555_write(XL9555_OUTPUT0, out | XL9555_LORA_EN) ||
      !xl9555_write(XL9555_CONFIG0, cfg & ~XL9555_LORA_EN))
    {
      BOOT_LOG_WRN("Progress: the LoRa radio not powered");
      return;
    }

  esp_rom_delay_us(10000);
}

/* The panel's bus: SPI mode 0, the most significant bit first */

static void epd_byte(uint8_t byte)
{
  int i;

  for (i = 7; i >= 0; i--)
    {
      gpio_ll_set_level(&GPIO, PIN_MOSI, (byte >> i) & 1);
      gpio_ll_set_level(&GPIO, PIN_SCK, 1);
      gpio_ll_set_level(&GPIO, PIN_SCK, 0);
    }
}

static void epd_cmd(uint8_t cmd)
{
  gpio_ll_set_level(&GPIO, PIN_EPD_DC, 0);
  gpio_ll_set_level(&GPIO, PIN_EPD_CS, 0);
  epd_byte(cmd);
  gpio_ll_set_level(&GPIO, PIN_EPD_CS, 1);
}

static void epd_cmd1(uint8_t cmd, uint8_t arg)
{
  epd_cmd(cmd);
  gpio_ll_set_level(&GPIO, PIN_EPD_DC, 1);
  gpio_ll_set_level(&GPIO, PIN_EPD_CS, 0);
  epd_byte(arg);
  gpio_ll_set_level(&GPIO, PIN_EPD_CS, 1);
}

static void epd_cmd2(uint8_t cmd, uint8_t arg1, uint8_t arg2)
{
  epd_cmd(cmd);
  gpio_ll_set_level(&GPIO, PIN_EPD_DC, 1);
  gpio_ll_set_level(&GPIO, PIN_EPD_CS, 0);
  epd_byte(arg1);
  epd_byte(arg2);
  gpio_ll_set_level(&GPIO, PIN_EPD_CS, 1);
}

static bool epd_busy(void)
{
  return gpio_ll_get_level(&GPIO, PIN_EPD_BUSY) == 0;
}

/* The controller takes a moment to say it is busy after a command.  A
 * wait is up to 8 s: MCUboot's watchdog, should it run, is fed meanwhile.
 */

static bool epd_wait(int ms)
{
  int waited;

  esp_rom_delay_us(1000);
  for (waited = 0; waited < ms; waited++)
    {
      if (!epd_busy())
        {
          return true;
        }

      MCUBOOT_WATCHDOG_FEED();
      esp_rom_delay_us(1000);
    }

  BOOT_LOG_WRN("Progress: the panel stayed busy");
  return false;
}

/* The screen, a pixel at a time: black? */

static bool text_black(const struct text_s *t, int x0, int y0, int x, int y)
{
  if (t == NULL || x < x0 || x >= x0 + t->w || y < y0 || y >= y0 + t->h)
    {
      return false;
    }

  x -= x0;
  y -= y0;
  return (t->bits[y * ((t->w + 7) / 8) + x / 8] >> (7 - x % 8)) & 1;
}

static bool screen_black(int x, int y, int fill)
{
  int block;
  int bx0;
  int bx1;

  if (y >= BAR_Y && y < BAR_Y + BAR_H && x >= BAR_X && x < BAR_X + BAR_W)
    {
      /* Which block, if any: each starts where its share of the width,
       * gaps included, does
       */

      for (block = 0; block < BAR_BLOCKS; block++)
        {
          bx0 = BAR_X + (block * (BAR_W + BAR_GAP) + BAR_BLOCKS / 2) /
                BAR_BLOCKS;
          bx1 = BAR_X + ((block + 1) * (BAR_W + BAR_GAP) + BAR_BLOCKS / 2) /
                BAR_BLOCKS - BAR_GAP;
          if (x >= bx0 && x < bx1)
            {
              return block < fill || x == bx0 || x == bx1 - 1 ||
                     y == BAR_Y || y == BAR_Y + BAR_H - 1;
            }
        }

      return false;
    }

  return text_black(g_title, TEXT_X, TITLE_Y, x, y) ||
         text_black(&g_caption, TEXT_X, CAPTION_Y, x, y);
}

static uint8_t screen_byte(int row, int col, int fill)
{
  uint8_t byte = 0xff;
  int bit;

  for (bit = 0; bit < 8; bit++)
    {
      if (screen_black(col * 8 + bit, row, fill))
        {
          byte &= ~(0x80 >> bit);
        }
    }

  return byte;
}

static void epd_data(int x0, int x1, int y0, int y1, int fill)
{
  int row;
  int col;

  gpio_ll_set_level(&GPIO, PIN_EPD_DC, 1);
  gpio_ll_set_level(&GPIO, PIN_EPD_CS, 0);
  for (row = y0; row < y1; row++)
    {
      for (col = x0 / 8; col < x1 / 8; col++)
        {
          epd_byte(screen_byte(row, col, fill));
        }
    }

  gpio_ll_set_level(&GPIO, PIN_EPD_CS, 1);
}

static void epd_window(void)
{
  static const uint8_t args[7] =
  {
    WIN_X0, WIN_X1 - 1, WIN_Y0 >> 8, WIN_Y0 & 0xff,
    (WIN_Y1 - 1) >> 8, (WIN_Y1 - 1) & 0xff, 0x01
  };

  int i;

  epd_cmd(EPD_PTL);
  gpio_ll_set_level(&GPIO, PIN_EPD_DC, 1);
  gpio_ll_set_level(&GPIO, PIN_EPD_CS, 0);
  for (i = 0; i < 7; i++)
    {
      epd_byte(args[i]);
    }

  gpio_ll_set_level(&GPIO, PIN_EPD_CS, 1);
}

/* After a refresh: the driving voltages off, the image kept; then the
 * soft reset the vendor's code does after every refresh, keeping the
 * controller's RAM
 */

static void epd_after(void)
{
  epd_cmd(EPD_POWER_OFF);
  epd_wait(EPD_POWER_MS);
  epd_cmd2(EPD_PSR, 0x1e, 0x0d);
  esp_rom_delay_us(1000);
  epd_cmd2(EPD_PSR, 0x1f, 0x0d);
}

/* The whole screen, waiting for it: both frames get it */

static void epd_full(int fill)
{
  epd_cmd(EPD_DTM1);
  epd_data(0, EPD_XRES, 0, EPD_YRES, fill);
  epd_cmd(EPD_DTM2);
  epd_data(0, EPD_XRES, 0, EPD_YRES, fill);
  epd_cmd1(EPD_CCSET, 0x02);
  epd_cmd1(EPD_TSSET, 0x5a);
  epd_cmd1(EPD_CDI, 0x97);
  epd_cmd(EPD_POWER_ON);
  if (epd_wait(EPD_POWER_MS))
    {
      epd_cmd(EPD_DRF);
      epd_wait(EPD_DRAW_MS);
    }

  epd_after();
}

/* The fill, from what the panel shows to the new: the previous frame and
 * the new, for its window, then the refresh, not waited for
 */

static void epd_partial_start(int fill)
{
  epd_cmd(EPD_PTIN);
  epd_window();
  epd_cmd(EPD_DTM1);
  epd_data(WIN_X0, WIN_X1, WIN_Y0, WIN_Y1, g_fill);
  epd_cmd(EPD_PTOUT);

  epd_cmd(EPD_PTIN);
  epd_window();
  epd_cmd(EPD_DTM2);
  epd_data(WIN_X0, WIN_X1, WIN_Y0, WIN_Y1, fill);
  epd_cmd(EPD_PTOUT);

  epd_cmd(EPD_PTIN);
  epd_window();
  epd_cmd1(EPD_CCSET, 0x02);
  epd_cmd1(EPD_TSSET, 0x79);
  epd_cmd1(EPD_CDI, 0xd7);
  epd_cmd(EPD_POWER_ON);
  if (epd_wait(EPD_POWER_MS))
    {
      epd_cmd(EPD_DRF);
      esp_rom_delay_us(1000);   /* Then it says it is busy */
    }

  g_fill    = fill;
  g_drawing = true;
}

static void epd_partial_end(void)
{
  epd_cmd(EPD_PTOUT);
  epd_after();
  g_drawing = false;
}

/* Set up the lines and the panel, and draw the screen, waiting */

static void show(int swap_type, int fill)
{
  pin_out(PIN_LORA_CS, 1);
  pin_out(PIN_SD_CS, 1);
  pin_out(PIN_EPD_CS, 1);
  pin_out(PIN_SCK, 0);
  pin_out(PIN_MOSI, 0);
  pin_out(PIN_EPD_DC, 0);
  pin_out(PIN_EPD_RST, 1);
  pin_in(PIN_EPD_BUSY);

  lora_power();

  g_title = swap_type == BOOT_SWAP_TYPE_REVERT ? &g_title_undo :
                                                 &g_title_update;

  gpio_ll_set_level(&GPIO, PIN_EPD_RST, 0);
  esp_rom_delay_us(10000);
  gpio_ll_set_level(&GPIO, PIN_EPD_RST, 1);
  esp_rom_delay_us(10000);
  epd_cmd2(EPD_PSR, 0x1f, 0x0d);

  epd_full(fill);
  g_fill = fill;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Told after each step of a swap; the first time, at its start (or where
 * a resumed one has got to).  A step takes some 60 ms, a partial refresh
 * most of a second: one is started when a block fills, and the swap goes
 * on meanwhile; a block filled while the panel is busy waits for the next
 * step.  At the end the last refresh is waited for, and the panel left
 * powered down, showing a full bar until NuttX clears it.
 */

void boot_swap_progress(int swap_type, uint32_t done, uint32_t total)
{
  int fill;
  uint32_t tenth;

  if (total == 0)
    {
      return;
    }

  fill  = done >= total ? BAR_BLOCKS : (int)(done * BAR_BLOCKS / total);
  tenth = done * 10 / total;

  if (!g_shown)
    {
      g_shown = true;
      g_tenth = tenth;
      BOOT_LOG_INF("Progress: swap type %d, %u of %u", swap_type,
                   (unsigned)done, (unsigned)total);
      show(swap_type, fill);
      BOOT_LOG_INF("Progress: shown");
      return;
    }

  if (tenth != g_tenth)
    {
      g_tenth = tenth;
      BOOT_LOG_INF("Progress: %u of %u", (unsigned)done, (unsigned)total);
    }

  if (g_drawing)
    {
      if (done < total && epd_busy())
        {
          return;
        }

      epd_wait(EPD_DRAW_MS);
      epd_partial_end();
    }

  if (fill != g_fill)
    {
      epd_partial_start(fill);
    }

  if (done >= total && g_drawing)
    {
      epd_wait(EPD_DRAW_MS);
      epd_partial_end();
      BOOT_LOG_INF("Progress: done");
    }
}

