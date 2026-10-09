/****************************************************************************
 * drivers/lcd/uc8253.c
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

/* Driver for the UltraChip UC8253 e-paper controller, as on the GoodDisplay
 * GDEQ031T10 panel (240x320, 1 bit).
 *
 * Drawing goes into a frame buffer in memory; the panel is refreshed when
 * the LCD framebuffer driver calls redraw(), with only the area that
 * changed when that is part of the screen (a partial refresh) or all of it.
 * The controller's RAM cannot be read back, so the driver keeps two
 * frames: the one being drawn, and the one the glass shows, which a
 * partial refresh needs as its "previous" frame.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/debug.h>
#include <nuttx/kthread.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/uc8253.h>
#include <nuttx/mutex.h>
#include <nuttx/sched.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>
#include <nuttx/video/fb.h>

#ifdef CONFIG_LCD_UC8253

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SPI_CMDDATA
#  error "CONFIG_SPI_CMDDATA is needed for the data/command line"
#endif

#ifndef CONFIG_LCD_PACKEDMSFIRST
#  error "CONFIG_LCD_PACKEDMSFIRST is needed: the controller is MSB first"
#endif

/* Geometry: the panel in its own portrait orientation */

#define UC8253_XRES           240
#define UC8253_YRES           320
#define UC8253_ROWSIZE        (UC8253_XRES / 8)              /* 30 bytes */
#define UC8253_FBSIZE         (UC8253_ROWSIZE * UC8253_YRES) /* 9600 */

/* Commands */

#define UC8253_PSR            0x00  /* Panel setting */
#define UC8253_POWER_OFF      0x02
#define UC8253_POWER_ON       0x04
#define UC8253_DEEP_SLEEP     0x07
#define UC8253_DTM1           0x10  /* Data: the previous frame */
#define UC8253_DRF            0x12  /* Display refresh */
#define UC8253_DTM2           0x13  /* Data: the new frame */
#define UC8253_CDI            0x50  /* VCOM and data interval */
#define UC8253_PTL            0x90  /* Partial window */
#define UC8253_PTIN           0x91  /* Partial in */
#define UC8253_PTOUT          0x92  /* Partial out */
#define UC8253_CCSET          0xe0  /* Cascade setting */
#define UC8253_TSSET          0xe5  /* Force a temperature */

/* Panel setting: the waveforms from OTP, scan up, shift right, booster on;
 * the second byte is the controller's default resolution and VCOM.  The
 * soft reset form is the same with the reset bit clear.
 */

#define UC8253_PSR_RUN        0x1f
#define UC8253_PSR_SOFTRESET  0x1e
#define UC8253_PSR_BYTE2      0x0d

/* The data interval differs for a full and a partial refresh */

#define UC8253_CDI_FULL       0x97
#define UC8253_CDI_PARTIAL    0xd7

#define UC8253_SLEEP_CHECK    0xa5  /* Deep sleep is ignored without it */

/* The controller picks a waveform for the temperature it measures.  A
 * forced one selects a faster waveform: 90 degrees makes a full refresh
 * about 1 s instead of about 3 s; 121 degrees is the partial waveform,
 * about 0.7 s.
 */

#define UC8253_TSSET_FAST     0x5a
#define UC8253_TSSET_PARTIAL  0x79
#define UC8253_CCSET_FORCED   0x02
#define UC8253_CCSET_MEASURED 0x00

/* Timeouts, in milliseconds: generous, since a cold panel is slower and
 * the alternative to waiting is a torn image.
 */

#define UC8253_BUSY_POLL      5
#define UC8253_POWER_TIMEOUT  500
#define UC8253_DRAW_TIMEOUT   8000

#define UC8253_RESET_MS       10    /* At least 10 ms around the pulse */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct uc8253_dev_s
{
  struct lcd_dev_s dev;                  /* Must be first */

  FAR struct spi_dev_s *spi;
  FAR const struct uc8253_priv_s *board;

  bool on;                               /* Powered up */
  bool configured;                       /* Reset and set up */
  bool blank;                            /* Controller RAM undefined */
  bool known;                            /* glass is what the panel shows */
  bool dropped;                          /* First flush taken as a clear */
  bool stale;                            /* The last refresh failed */
  bool full;                             /* Next refresh a full one */

  /* panel serialises everything that talks to the controller, for a whole
   * refresh; fblock only protects the frame being drawn and its dirty
   * area, briefly, so that drawing never waits for the panel.  When both
   * are taken, panel is first.
   */

  mutex_t panel;
  mutex_t fblock;

#ifdef CONFIG_LCD_UC8253_ASYNC
  sem_t kick;                            /* Wakes the refresh thread */
#endif

  /* What has been drawn since the last refresh, inclusive; x1 > x2 when
   * nothing.
   */

  fb_coord_t x1;
  fb_coord_t y1;
  fb_coord_t x2;
  fb_coord_t y2;

  uint8_t frame[UC8253_FBSIZE];          /* Being drawn */
  uint8_t glass[UC8253_FBSIZE];          /* On the glass */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int uc8253_putrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                         fb_coord_t col, FAR const uint8_t *buffer,
                         size_t npixels);
static int uc8253_putarea(FAR struct lcd_dev_s *dev, fb_coord_t row_start,
                          fb_coord_t row_end, fb_coord_t col_start,
                          fb_coord_t col_end, FAR const uint8_t *buffer,
                          fb_coord_t stride);
static int uc8253_getrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                         fb_coord_t col, FAR uint8_t *buffer,
                         size_t npixels);
static int uc8253_redraw(FAR struct lcd_dev_s *dev);
static int uc8253_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo);
static int uc8253_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int pno,
                               FAR struct lcd_planeinfo_s *pinfo);
static int uc8253_getpower(FAR struct lcd_dev_s *dev);
static int uc8253_setpower(FAR struct lcd_dev_s *dev, int power);
static int uc8253_getcontrast(FAR struct lcd_dev_s *dev);
static int uc8253_setcontrast(FAR struct lcd_dev_s *dev,
                              unsigned int contrast);
static int uc8253_ioctl(FAR struct lcd_dev_s *dev, int cmd,
                        unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = FB_FMT_Y1,
  .xres    = UC8253_XRES,
  .yres    = UC8253_YRES,
  .nplanes = 1,
};

static struct uc8253_dev_s g_uc8253;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uc8253_bitcpy
 *
 * Description:
 *   Copy nbits from the start of src into dest, starting offset bits into
 *   its first byte.  Both are packed most significant bit first, so an
 *   aligned copy is a memcpy.
 *
 ****************************************************************************/

static void uc8253_bitcpy(FAR uint8_t *dest, int offset,
                          FAR const uint8_t *src, size_t nbits)
{
  uint8_t mask;
  uint8_t val;

  if (offset == 0)
    {
      memcpy(dest, src, nbits >> 3);
      dest  += nbits >> 3;
      src   += nbits >> 3;
      nbits &= 7;

      if (nbits > 0)
        {
          mask   = (uint8_t)(0xff << (8 - nbits));
          *dest  = (*dest & ~mask) | (*src & mask);
        }

      return;
    }

  /* Each source byte spans two bytes of dest */

  while (nbits >= 8)
    {
      val = *src++;

      mask   = 0xff >> offset;
      *dest  = (*dest & ~mask) | (val >> offset);
      dest++;

      mask   = (uint8_t)(0xff << (8 - offset));
      *dest  = (*dest & ~mask) | (uint8_t)(val << (8 - offset));

      nbits -= 8;
    }

  if (nbits > 0)
    {
      val = *src;

      if (nbits + offset <= 8)
        {
          /* Narrowed to a byte before the shift right, so that the mask
           * does not reach the pixels before the run.
           */

          mask   = (uint8_t)(0xff << (8 - nbits));
          mask >>= offset;
          *dest  = (*dest & ~mask) | ((val >> offset) & mask);
        }
      else
        {
          mask   = 0xff >> offset;
          *dest  = (*dest & ~mask) | (val >> offset);
          dest++;

          nbits -= 8 - offset;
          mask   = (uint8_t)(0xff << (8 - nbits));
          *dest  = (*dest & ~mask) | ((uint8_t)(val << (8 - offset)) & mask);
        }
    }
}

/****************************************************************************
 * Name: uc8253_lock, uc8253_unlock
 *
 * Description:
 *   Hold the SPI bus, which the panel may share, for a transaction.
 *
 ****************************************************************************/

static void uc8253_lock(FAR struct uc8253_dev_s *priv)
{
  SPI_LOCK(priv->spi, true);
  SPI_SETMODE(priv->spi, SPIDEV_MODE0);
  SPI_SETBITS(priv->spi, 8);
  SPI_HWFEATURES(priv->spi, 0);
  SPI_SETFREQUENCY(priv->spi, CONFIG_LCD_UC8253_FREQUENCY);
}

static void uc8253_unlock(FAR struct uc8253_dev_s *priv)
{
  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: uc8253_cmd, uc8253_data, uc8253_fill, uc8253_cmd1
 *
 * Description:
 *   Send a command, data, a run of one byte, or a command with one byte.
 *   Each pulses the chip select, as the panel vendor's code does.  Called
 *   with the bus held.
 *
 ****************************************************************************/

static void uc8253_cmd(FAR struct uc8253_dev_s *priv, uint8_t cmd)
{
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);
  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(priv->spi, cmd);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);
}

static void uc8253_data(FAR struct uc8253_dev_s *priv,
                        FAR const uint8_t *data, size_t len)
{
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);
  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), false);
  SPI_SNDBLOCK(priv->spi, data, len);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);
}

static void uc8253_fill(FAR struct uc8253_dev_s *priv, uint8_t value)
{
  uint8_t row[UC8253_ROWSIZE];
  int y;

  memset(row, value, sizeof(row));

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);
  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), false);

  for (y = 0; y < UC8253_YRES; y++)
    {
      SPI_SNDBLOCK(priv->spi, row, sizeof(row));
    }

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);
}

static void uc8253_cmd1(FAR struct uc8253_dev_s *priv, uint8_t cmd,
                        uint8_t arg)
{
  uc8253_cmd(priv, cmd);
  uc8253_data(priv, &arg, 1);
}

/****************************************************************************
 * Name: uc8253_busywait
 *
 * Description:
 *   Wait until the controller is no longer busy.
 *
 * Returned Value:
 *   Zero (OK), or -ETIMEDOUT.
 *
 ****************************************************************************/

static int uc8253_busywait(FAR struct uc8253_dev_s *priv, int timeout)
{
  int waited;

  /* The controller takes a moment to say it is busy after a command */

  nxsched_usleep(1000);

  for (waited = 0; waited < timeout; waited += UC8253_BUSY_POLL)
    {
      if (!priv->board->check_busy())
        {
          return OK;
        }

      nxsched_usleep(UC8253_BUSY_POLL * 1000);
    }

  lcderr("ERROR: the panel stayed busy for more than %d ms\n", timeout);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: uc8253_configure
 *
 * Description:
 *   Reset the controller and set it up.  Its RAM is undefined afterwards.
 *
 ****************************************************************************/

static void uc8253_configure(FAR struct uc8253_dev_s *priv)
{
  static const uint8_t psr[] =
  {
    UC8253_PSR_RUN, UC8253_PSR_BYTE2
  };

  priv->board->set_rst(false);
  nxsched_usleep(UC8253_RESET_MS * 1000);
  priv->board->set_rst(true);
  nxsched_usleep(UC8253_RESET_MS * 1000);
  priv->board->set_rst(false);
  nxsched_usleep(UC8253_RESET_MS * 1000);

  uc8253_lock(priv);
  uc8253_cmd(priv, UC8253_PSR);
  uc8253_data(priv, psr, sizeof(psr));
  uc8253_unlock(priv);

  priv->configured = true;
  priv->blank      = true;
}

/****************************************************************************
 * Name: uc8253_poweroff
 *
 * Description:
 *   Turn the driving voltages off: the image stays, while leaving them on
 *   would let it fade.  Called with the bus held.
 *
 ****************************************************************************/

static void uc8253_poweroff(FAR struct uc8253_dev_s *priv)
{
  uc8253_cmd(priv, UC8253_POWER_OFF);
  uc8253_busywait(priv, UC8253_POWER_TIMEOUT);
}

/****************************************************************************
 * Name: uc8253_softreset
 *
 * Description:
 *   Soft reset the controller, keeping its RAM; the vendor's code does it
 *   after every refresh.  Called with the bus held.
 *
 ****************************************************************************/

static void uc8253_softreset(FAR struct uc8253_dev_s *priv)
{
  static const uint8_t reset[] =
  {
    UC8253_PSR_SOFTRESET, UC8253_PSR_BYTE2
  };

  static const uint8_t run[] =
  {
    UC8253_PSR_RUN, UC8253_PSR_BYTE2
  };

  uc8253_cmd(priv, UC8253_PSR);
  uc8253_data(priv, reset, sizeof(reset));
  up_mdelay(1);
  uc8253_cmd(priv, UC8253_PSR);
  uc8253_data(priv, run, sizeof(run));
}

/****************************************************************************
 * Name: uc8253_window
 *
 * Description:
 *   Select a partial window; x and w are multiples of 8.  Called with the
 *   bus held.
 *
 ****************************************************************************/

static void uc8253_window(FAR struct uc8253_dev_s *priv, fb_coord_t x,
                          fb_coord_t y, fb_coord_t w, fb_coord_t h)
{
  uint16_t xe = x + w - 1;
  uint16_t ye = y + h - 1;
  uint8_t args[7];

  args[0] = (uint8_t)x;
  args[1] = (uint8_t)xe;
  args[2] = (uint8_t)(y >> 8);
  args[3] = (uint8_t)y;
  args[4] = (uint8_t)(ye >> 8);
  args[5] = (uint8_t)ye;
  args[6] = 0x01;

  uc8253_cmd(priv, UC8253_PTL);
  uc8253_data(priv, args, sizeof(args));
}

/****************************************************************************
 * Name: uc8253_sendwindow
 *
 * Description:
 *   Write a window of the glass frame into one of the controller's frames
 *   (DTM1 or DTM2).  Called with the bus held.
 *
 ****************************************************************************/

static void uc8253_sendwindow(FAR struct uc8253_dev_s *priv, uint8_t cmd,
                              fb_coord_t x, fb_coord_t y, fb_coord_t w,
                              fb_coord_t h)
{
  fb_coord_t row;

  uc8253_cmd(priv, UC8253_PTIN);
  uc8253_window(priv, x, y, w, h);
  uc8253_cmd(priv, cmd);

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);
  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), false);

  for (row = y; row < y + h; row++)
    {
      SPI_SNDBLOCK(priv->spi,
                   priv->glass + row * UC8253_ROWSIZE + (x >> 3), w >> 3);
    }

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);
  uc8253_cmd(priv, UC8253_PTOUT);
}

/****************************************************************************
 * Name: uc8253_dirty, uc8253_clean
 *
 * Description:
 *   Grow the area drawn since the last refresh; forget it.  Called with
 *   fblock held.
 *
 ****************************************************************************/

static void uc8253_dirty(FAR struct uc8253_dev_s *priv, fb_coord_t x1,
                         fb_coord_t y1, fb_coord_t x2, fb_coord_t y2)
{
  if (priv->x1 > priv->x2)
    {
      priv->x1 = x1;
      priv->y1 = y1;
      priv->x2 = x2;
      priv->y2 = y2;
      return;
    }

  priv->x1 = x1 < priv->x1 ? x1 : priv->x1;
  priv->y1 = y1 < priv->y1 ? y1 : priv->y1;
  priv->x2 = x2 > priv->x2 ? x2 : priv->x2;
  priv->y2 = y2 > priv->y2 ? y2 : priv->y2;
}

static void uc8253_clean(FAR struct uc8253_dev_s *priv)
{
  priv->x1 = UC8253_XRES;
  priv->y1 = UC8253_YRES;
  priv->x2 = 0;
  priv->y2 = 0;
}

/****************************************************************************
 * Name: uc8253_trim
 *
 * Description:
 *   Shrink a byte aligned area to where the frame differs from the glass:
 *   drawing often repaints what is there, and refreshing that would flash
 *   the panel for nothing.  Called with fblock held.
 *
 * Returned Value:
 *   false if nothing differs.
 *
 ****************************************************************************/

static bool uc8253_trim(FAR struct uc8253_dev_s *priv, FAR fb_coord_t *x,
                        FAR fb_coord_t *y, FAR fb_coord_t *w,
                        FAR fb_coord_t *h)
{
  fb_coord_t first = *x >> 3;
  fb_coord_t last  = (*x + *w) >> 3;
  fb_coord_t b0    = last;
  fb_coord_t b1    = 0;
  fb_coord_t y0    = *y + *h;
  fb_coord_t y1    = 0;
  fb_coord_t row;
  fb_coord_t b;
  size_t offset;

  for (row = *y; row < *y + *h; row++)
    {
      offset = row * UC8253_ROWSIZE;
      for (b = first; b < last; b++)
        {
          if (priv->frame[offset + b] != priv->glass[offset + b])
            {
              b0 = b < b0 ? b : b0;
              b1 = b > b1 ? b : b1;
              y0 = row < y0 ? row : y0;
              y1 = row;
            }
        }
    }

  if (b0 > b1)
    {
      return false;
    }

  *x = b0 << 3;
  *w = (b1 - b0 + 1) << 3;
  *y = y0;
  *h = y1 - y0 + 1;
  return true;
}

/****************************************************************************
 * Name: uc8253_draw
 *
 * Description:
 *   Refresh the panel from the controller's frames, and wait for it.  The
 *   bus is let go while the panel works, most of a second, so that the
 *   devices sharing it are not locked out.  Called with the bus held;
 *   returns with it held.
 *
 ****************************************************************************/

static int uc8253_draw(FAR struct uc8253_dev_s *priv, bool partial)
{
  int ret;

#ifdef CONFIG_LCD_UC8253_PARTIAL
  if (partial)
    {
      /* The partial waveform keeps the rest of the screen still */

      uc8253_cmd1(priv, UC8253_CCSET, UC8253_CCSET_FORCED);
      uc8253_cmd1(priv, UC8253_TSSET, UC8253_TSSET_PARTIAL);
      uc8253_cmd1(priv, UC8253_CDI, UC8253_CDI_PARTIAL);
    }
  else
#endif
    {
#ifdef CONFIG_LCD_UC8253_FASTUPDATE
      uc8253_cmd1(priv, UC8253_CCSET, UC8253_CCSET_FORCED);
      uc8253_cmd1(priv, UC8253_TSSET, UC8253_TSSET_FAST);
#else
      /* A partial refresh forced the temperature: back to the measured
       * one, or the full waveform would be picked for the wrong one.
       */

      uc8253_cmd1(priv, UC8253_CCSET, UC8253_CCSET_MEASURED);
#endif
      uc8253_cmd1(priv, UC8253_CDI, UC8253_CDI_FULL);
    }

  uc8253_cmd(priv, UC8253_POWER_ON);
  ret = uc8253_busywait(priv, UC8253_POWER_TIMEOUT);
  if (ret >= 0)
    {
      uc8253_cmd(priv, UC8253_DRF);

      uc8253_unlock(priv);
      ret = uc8253_busywait(priv, UC8253_DRAW_TIMEOUT);
      uc8253_lock(priv);
    }

#ifdef CONFIG_LCD_UC8253_PARTIAL
  if (partial)
    {
      uc8253_cmd(priv, UC8253_PTOUT);
    }
#endif

  uc8253_poweroff(priv);
  uc8253_softreset(priv);
  return ret;
}

/****************************************************************************
 * Name: uc8253_clear
 *
 * Description:
 *   Drive the whole panel white, writing both of the controller's frames,
 *   so that every later refresh starts from a known "previous" frame.
 *   Called with panel held.
 *
 ****************************************************************************/

static int uc8253_clear(FAR struct uc8253_dev_s *priv)
{
  int ret;

  uc8253_lock(priv);
  uc8253_cmd(priv, UC8253_DTM1);
  uc8253_fill(priv, 0xff);
  uc8253_cmd(priv, UC8253_DTM2);
  uc8253_fill(priv, 0xff);
  ret = uc8253_draw(priv, false);
  uc8253_unlock(priv);

  if (ret >= 0)
    {
      memset(priv->glass, 0xff, UC8253_FBSIZE);
      priv->blank = false;
      priv->known = true;
      priv->stale = false;
    }

  return ret;
}

/****************************************************************************
 * Name: uc8253_reseed
 *
 * Description:
 *   Load both of the controller's frames with what the glass shows, with
 *   no refresh: how the panel comes back from deep sleep, which loses the
 *   controller's RAM but not the image.  Called with panel held.
 *
 ****************************************************************************/

static void uc8253_reseed(FAR struct uc8253_dev_s *priv)
{
  uc8253_lock(priv);
  uc8253_cmd(priv, UC8253_DTM1);
  uc8253_data(priv, priv->glass, UC8253_FBSIZE);
  uc8253_cmd(priv, UC8253_DTM2);
  uc8253_data(priv, priv->glass, UC8253_FBSIZE);
  uc8253_unlock(priv);

  priv->blank = false;
}

/****************************************************************************
 * Name: uc8253_update
 *
 * Description:
 *   Bring the panel up to date with what has been drawn.  A partial
 *   refresh drives each pixel from the change between the controller's
 *   previous and new frames, so both are written for its window: the
 *   previous with what the glass shows, the new with the drawing.  A full
 *   refresh gets the new frame in both, as in the vendor's code.
 *
 *   fblock is held only while the drawing is copied out, so drawing goes
 *   on during a refresh; what is drawn meanwhile goes out with the next.
 *
 ****************************************************************************/

static int uc8253_update(FAR struct uc8253_dev_s *priv)
{
  fb_coord_t x;
  fb_coord_t y;
  fb_coord_t w;
  fb_coord_t h;
  fb_coord_t row;
  bool partial = false;
  bool full;
  int ret = OK;

  nxmutex_lock(&priv->panel);

  /* A powered down panel keeps the drawing for when it is powered again */

  if (!priv->on)
    {
      goto out;
    }

  if (!priv->configured)
    {
      uc8253_configure(priv);
    }

  if (priv->blank)
    {
      if (priv->known)
        {
          uc8253_reseed(priv);
        }
      else
        {
          ret = uc8253_clear(priv);
          if (ret < 0)
            {
              goto out;
            }
        }
    }

  nxmutex_lock(&priv->fblock);

  if (priv->x1 > priv->x2 && !priv->full)
    {
      nxmutex_unlock(&priv->fblock);
      goto out;
    }

  /* Taken now, so that a request for a full refresh made while this one
   * runs is kept for the next; if this one fails, the next is full anyway
   */

  full       = priv->full;
  priv->full = false;

  if (full)
    {
      x = 0;
      y = 0;
      w = UC8253_XRES;
      h = UC8253_YRES;
    }
  else
    {
      /* The controller's RAM is addressed by the byte across */

      x = priv->x1 & ~0x0007;
      y = priv->y1;
      w = (priv->x2 | 0x0007) - x + 1;
      h = priv->y2 - y + 1;

      /* After a failed refresh the glass frame cannot say what changed;
       * the full refresh that follows draws everything anyway.
       */

      if (!priv->stale && !uc8253_trim(priv, &x, &y, &w, &h))
        {
          uc8253_clean(priv);
          nxmutex_unlock(&priv->fblock);
          goto out;
        }
    }

#ifdef CONFIG_LCD_UC8253_PARTIAL
  partial = !priv->stale && !full &&
            (x > 0 || y > 0 || w < UC8253_XRES || h < UC8253_YRES);
#endif

  uc8253_clean(priv);
  uc8253_lock(priv);

#ifdef CONFIG_LCD_UC8253_PARTIAL
  if (partial)
    {
      /* The previous frame first, while the glass frame holds it */

      uc8253_sendwindow(priv, UC8253_DTM1, x, y, w, h);

      for (row = y; row < y + h; row++)
        {
          memcpy(priv->glass + row * UC8253_ROWSIZE + (x >> 3),
                 priv->frame + row * UC8253_ROWSIZE + (x >> 3), w >> 3);
        }

      nxmutex_unlock(&priv->fblock);

      uc8253_sendwindow(priv, UC8253_DTM2, x, y, w, h);
      uc8253_cmd(priv, UC8253_PTIN);
      uc8253_window(priv, x, y, w, h);
    }
  else
#endif
    {
      UNUSED(row);
      memcpy(priv->glass, priv->frame, UC8253_FBSIZE);
      nxmutex_unlock(&priv->fblock);

      uc8253_cmd(priv, UC8253_DTM1);
      uc8253_data(priv, priv->glass, UC8253_FBSIZE);
      uc8253_cmd(priv, UC8253_DTM2);
      uc8253_data(priv, priv->glass, UC8253_FBSIZE);
    }

  ret = uc8253_draw(priv, partial);
  uc8253_unlock(priv);

  if (ret < 0)
    {
      /* Keep the change, so that the next update tries again; what the
       * glass shows is in doubt, so that one is a full refresh.
       */

      priv->stale = true;
      nxmutex_lock(&priv->fblock);
      uc8253_dirty(priv, x, y, x + w - 1, y + h - 1);
      nxmutex_unlock(&priv->fblock);
    }
  else if (!partial)
    {
      priv->stale = false;
    }

out:
  nxmutex_unlock(&priv->panel);
  return ret;
}

#ifdef CONFIG_LCD_UC8253_ASYNC

/****************************************************************************
 * Name: uc8253_kick
 *
 * Description:
 *   Wake the refresh thread; one wake-up pending is enough, since the
 *   refresh takes everything drawn until it copies the drawing out.
 *
 ****************************************************************************/

static void uc8253_kick(FAR struct uc8253_dev_s *priv)
{
  int count;

  nxsem_get_value(&priv->kick, &count);
  if (count < 1)
    {
      nxsem_post(&priv->kick);
    }
}

/****************************************************************************
 * Name: uc8253_thread
 *
 * Description:
 *   Refresh the panel when woken, after a short wait for the rest of a
 *   burst of drawing, so that a caller drawing a little at a time gets one
 *   refresh, not one per call.
 *
 ****************************************************************************/

static int uc8253_thread(int argc, FAR char *argv[])
{
  FAR struct uc8253_dev_s *priv = &g_uc8253;

  for (; ; )
    {
      nxsem_wait_uninterruptible(&priv->kick);

#if CONFIG_LCD_UC8253_ASYNC_DELAY > 0
      nxsched_usleep(CONFIG_LCD_UC8253_ASYNC_DELAY * 1000);
#endif

      uc8253_update(priv);
    }

  return OK;
}

#endif /* CONFIG_LCD_UC8253_ASYNC */

/****************************************************************************
 * Name: uc8253_putrun
 *
 * Description:
 *   Draw part of a row; nothing reaches the panel until redraw().
 *
 ****************************************************************************/

static int uc8253_putrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                         fb_coord_t col, FAR const uint8_t *buffer,
                         size_t npixels)
{
  FAR struct uc8253_dev_s *priv = (FAR struct uc8253_dev_s *)dev;

  if (row >= UC8253_YRES || col >= UC8253_XRES)
    {
      return -EINVAL;
    }

  if (npixels == 0)
    {
      return OK;
    }

  if (npixels > (size_t)(UC8253_XRES - col))
    {
      npixels = UC8253_XRES - col;
    }

  nxmutex_lock(&priv->fblock);
  uc8253_bitcpy(priv->frame + row * UC8253_ROWSIZE + (col >> 3), col & 7,
                buffer, npixels);
  uc8253_dirty(priv, col, row, col + npixels - 1, row);
  nxmutex_unlock(&priv->fblock);
  return OK;
}

/****************************************************************************
 * Name: uc8253_putarea
 *
 * Description:
 *   Draw an area; the LCD framebuffer driver aligns col_start to the byte
 *   for a 1 bit panel, so full rows are one memcpy each.
 *
 ****************************************************************************/

static int uc8253_putarea(FAR struct lcd_dev_s *dev, fb_coord_t row_start,
                          fb_coord_t row_end, fb_coord_t col_start,
                          fb_coord_t col_end, FAR const uint8_t *buffer,
                          fb_coord_t stride)
{
  FAR struct uc8253_dev_s *priv = (FAR struct uc8253_dev_s *)dev;
  fb_coord_t row;

  if (row_end >= UC8253_YRES)
    {
      row_end = UC8253_YRES - 1;
    }

  if (col_end >= UC8253_XRES)
    {
      col_end = UC8253_XRES - 1;
    }

  if (row_start > row_end || col_start > col_end)
    {
      return -EINVAL;
    }

  nxmutex_lock(&priv->fblock);

  for (row = row_start; row <= row_end; row++)
    {
      uc8253_bitcpy(priv->frame + row * UC8253_ROWSIZE + (col_start >> 3),
                    col_start & 7, buffer + (row - row_start) * stride,
                    col_end - col_start + 1);
    }

  uc8253_dirty(priv, col_start, row_start, col_end, row_end);
  nxmutex_unlock(&priv->fblock);
  return OK;
}

/****************************************************************************
 * Name: uc8253_getrun
 *
 * Description:
 *   Read part of a row back from the frame in memory: the controller's RAM
 *   cannot be read over this interface.
 *
 ****************************************************************************/

static int uc8253_getrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                         fb_coord_t col, FAR uint8_t *buffer,
                         size_t npixels)
{
  FAR struct uc8253_dev_s *priv = (FAR struct uc8253_dev_s *)dev;
  FAR const uint8_t *src;
  uint8_t pixel;
  size_t bit;
  size_t i;

  if (row >= UC8253_YRES || col >= UC8253_XRES)
    {
      return -EINVAL;
    }

  if (npixels > (size_t)(UC8253_XRES - col))
    {
      npixels = UC8253_XRES - col;
    }

  nxmutex_lock(&priv->fblock);
  src = priv->frame + row * UC8253_ROWSIZE;

  for (i = 0; i < npixels; i++)
    {
      bit   = col + i;
      pixel = (src[bit >> 3] >> (7 - (bit & 7))) & 1;

      if ((i & 7) == 0)
        {
          buffer[i >> 3] = 0;
        }

      buffer[i >> 3] |= (uint8_t)(pixel << (7 - (i & 7)));
    }

  nxmutex_unlock(&priv->fblock);
  return OK;
}

/****************************************************************************
 * Name: uc8253_redraw
 *
 * Description:
 *   Refresh the panel with what has been drawn.
 *
 ****************************************************************************/

static int uc8253_redraw(FAR struct lcd_dev_s *dev)
{
  FAR struct uc8253_dev_s *priv = (FAR struct uc8253_dev_s *)dev;

  nxmutex_lock(&priv->fblock);

  if (!priv->known && !priv->dropped)
    {
      /* The LCD framebuffer driver flushes its buffer as soon as it
       * registers, freshly zeroed, which here is all black, and before it
       * powers the panel on.  Nobody asked for that: the first redraw
       * after a reset is taken as a clear.  The frame is made white too,
       * so that it matches the glass.
       */

      uc8253_clean(priv);
      memset(priv->frame, 0xff, UC8253_FBSIZE);
      priv->dropped = true;
    }

  nxmutex_unlock(&priv->fblock);

  /* A panel powered down keeps the drawing for the first redraw after it
   * is powered again
   */

  if (!priv->on)
    {
      return OK;
    }

#ifdef CONFIG_LCD_UC8253_ASYNC
  uc8253_kick(priv);
  return OK;
#else
  return uc8253_update(priv);
#endif
}

/****************************************************************************
 * Name: uc8253_getvideoinfo, uc8253_getplaneinfo
 ****************************************************************************/

static int uc8253_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo)
{
  *vinfo = g_videoinfo;
  return OK;
}

static int uc8253_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int pno,
                               FAR struct lcd_planeinfo_s *pinfo)
{
  if (pno != 0)
    {
      return -EINVAL;
    }

  memset(pinfo, 0, sizeof(*pinfo));
  pinfo->putrun  = uc8253_putrun;
  pinfo->putarea = uc8253_putarea;
  pinfo->getrun  = uc8253_getrun;
  pinfo->redraw  = uc8253_redraw;
  pinfo->bpp     = 1;
  pinfo->dev     = dev;
  return OK;
}

/****************************************************************************
 * Name: uc8253_getpower, uc8253_setpower
 *
 * Description:
 *   Power down puts the controller in deep sleep, which loses its RAM; the
 *   glass keeps the image, and the controller is reseeded from the glass
 *   frame when powered up again.
 *
 ****************************************************************************/

static int uc8253_getpower(FAR struct lcd_dev_s *dev)
{
  FAR struct uc8253_dev_s *priv = (FAR struct uc8253_dev_s *)dev;

  return priv->on ? CONFIG_LCD_MAXPOWER : 0;
}

static int uc8253_setpower(FAR struct lcd_dev_s *dev, int power)
{
  FAR struct uc8253_dev_s *priv = (FAR struct uc8253_dev_s *)dev;

  /* Never in the middle of a refresh */

  nxmutex_lock(&priv->panel);

  if (power > 0)
    {
      if (!priv->configured)
        {
          uc8253_configure(priv);
        }

      priv->on = true;
    }
  else if (priv->on)
    {
      /* Deep sleep needs its check code, and the voltages off first */

      uc8253_lock(priv);
      uc8253_poweroff(priv);
      uc8253_cmd1(priv, UC8253_DEEP_SLEEP, UC8253_SLEEP_CHECK);
      uc8253_unlock(priv);

      priv->on         = false;
      priv->configured = false;
    }

  nxmutex_unlock(&priv->panel);
  return OK;
}

/****************************************************************************
 * Name: uc8253_getcontrast, uc8253_setcontrast
 ****************************************************************************/

static int uc8253_getcontrast(FAR struct lcd_dev_s *dev)
{
  return -ENOSYS;
}

static int uc8253_setcontrast(FAR struct lcd_dev_s *dev,
                              unsigned int contrast)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: uc8253_ioctl
 ****************************************************************************/

static int uc8253_ioctl(FAR struct lcd_dev_s *dev, int cmd,
                        unsigned long arg)
{
  FAR struct uc8253_dev_s *priv = (FAR struct uc8253_dev_s *)dev;

  switch (cmd)
    {
      case UC8253_IOC_FULLREFRESH:
        nxmutex_lock(&priv->fblock);
        priv->full = true;
        nxmutex_unlock(&priv->fblock);
        return OK;

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uc8253_initialize
 *
 * Description:
 *   Bind the controller to an SPI bus.  Nothing is sent to the panel until
 *   it is powered on and drawn on.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *uc8253_initialize(FAR struct spi_dev_s *spi,
                                     FAR const struct uc8253_priv_s *board)
{
  FAR struct uc8253_dev_s *priv = &g_uc8253;
#ifdef CONFIG_LCD_UC8253_ASYNC
  int ret;
#endif

  DEBUGASSERT(spi != NULL && board != NULL && board->set_rst != NULL &&
              board->check_busy != NULL);

  /* There is one panel: bound once, the same device after that */

  if (priv->spi != NULL)
    {
      return &priv->dev;
    }

  memset(priv, 0, sizeof(*priv));
  priv->dev.getvideoinfo = uc8253_getvideoinfo;
  priv->dev.getplaneinfo = uc8253_getplaneinfo;
  priv->dev.getpower     = uc8253_getpower;
  priv->dev.setpower     = uc8253_setpower;
  priv->dev.getcontrast  = uc8253_getcontrast;
  priv->dev.setcontrast  = uc8253_setcontrast;
  priv->dev.ioctl        = uc8253_ioctl;
  priv->spi              = spi;
  priv->board            = board;

  nxmutex_init(&priv->panel);
  nxmutex_init(&priv->fblock);
  uc8253_clean(priv);
  memset(priv->frame, 0xff, UC8253_FBSIZE);

#ifdef CONFIG_LCD_UC8253_ASYNC
  nxsem_init(&priv->kick, 0, 0);

  ret = kthread_create("uc8253", CONFIG_LCD_UC8253_ASYNC_PRIORITY,
                       CONFIG_LCD_UC8253_ASYNC_STACKSIZE, uc8253_thread,
                       NULL);
  if (ret < 0)
    {
      lcderr("ERROR: no refresh thread: %d\n", ret);
      priv->spi = NULL;
      return NULL;
    }
#endif

  return &priv->dev;
}

#endif /* CONFIG_LCD_UC8253 */
