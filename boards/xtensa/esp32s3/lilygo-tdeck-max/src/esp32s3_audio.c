/****************************************************************************
 * boards/xtensa/esp32s3/lilygo-tdeck-max/src/esp32s3_audio.c
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
 * The ES8311 codec is on the shared I2C bus at 0x18 and on I2S0: MCLK
 * GPIO38, BCLK GPIO39, WS GPIO18, data to the codec GPIO40, from it GPIO17
 * (schematic page 8; the vendor's pin table and macros have these two the
 * other way round).  Its output drives the speaker through an amplifier
 * switched by the XL9555's amp_en line; the XL9555's audio_sel line
 * connects the codec rather than the modem (low, from boot).  The
 * microphone is an analog electret on MIC1P/MIC1N, biased from ADCVREF.
 *
 * Playback is /dev/audio/pcm0, through NuttX's PCM decoder (which in raw
 * mode, AUDIO_FORMAT_RAW, passes raw PCM through and does not parse WAV
 * headers); recording is /dev/audio/pcm_in0.  Both are the same codec,
 * set up twice: use one at a time.  The amplifier is switched on while the
 * output is reserved by a player.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/pcm.h>
#include <nuttx/audio/es8311.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "espressif/esp_i2s.h"
#include "esp32s3_i2c.h"

#include "lilygo-tdeck-max.h"

#if defined(CONFIG_AUDIO_ES8311) && defined(CONFIG_ESPRESSIF_I2S0) && \
    defined(CONFIG_IOEXPANDER_PCA9555)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
#  define SESSION_ARG   , FAR void **session
#  define SESSION_PASS  , session
#  define RSESSION_ARG  , FAR void *session
#else
#  define SESSION_ARG
#  define SESSION_PASS
#  define RSESSION_ARG
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct es8311_lower_s g_es8311_out =
{
  .frequency = 400000,
  .address   = BOARD_I2C_ADDR_ES8311,
};

static const struct es8311_lower_s g_es8311_in =
{
  .frequency = 400000,
  .address   = BOARD_I2C_ADDR_ES8311,
};

/* The playback device's operations, with reserve, release and shutdown
 * extended to switch the amplifier
 */

static struct audio_ops_s g_out_ops;
static FAR const struct audio_ops_s *g_out_orig;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_amp
 *
 * Description:
 *   Switch the speaker's amplifier, an XL9555 line.
 *
 ****************************************************************************/

static void tdeck_amp(bool on)
{
  int ret = tdeck_xl9555_write(XL9555_AMP_EN, on);

  if (ret < 0)
    {
      auderr("ERROR: Failed to switch the amplifier %s: %d\n",
             on ? "on" : "off", ret);
    }
}

/****************************************************************************
 * Name: tdeck_out_reserve / tdeck_out_release / tdeck_out_shutdown
 *
 * Description:
 *   The amplifier is on while a player has reserved the output.  The last
 *   close shuts the device down without releasing it when the player did
 *   not (it failed to start, or it was killed), so shutting down switches
 *   the amplifier off too.
 *
 ****************************************************************************/

static int tdeck_out_reserve(FAR struct audio_lowerhalf_s *dev
                             SESSION_ARG)
{
  int ret = g_out_orig->reserve(dev SESSION_PASS);

  if (ret >= 0)
    {
      tdeck_amp(true);
    }

  return ret;
}

static int tdeck_out_release(FAR struct audio_lowerhalf_s *dev
                             RSESSION_ARG)
{
  int ret = g_out_orig->release(dev SESSION_PASS);

  tdeck_amp(false);
  return ret;
}

static int tdeck_out_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  int ret = g_out_orig->shutdown(dev);

  tdeck_amp(false);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_audio_initialize
 *
 * Description:
 *   See src/lilygo-tdeck-max.h
 *
 ****************************************************************************/

int tdeck_audio_initialize(void)
{
  FAR struct audio_lowerhalf_s *codec;
  FAR struct audio_lowerhalf_s *pcm;
  FAR struct i2c_master_s *i2c;
  FAR struct i2s_dev_s *i2s;
  int ret;

  i2c = esp32s3_i2cbus_initialize(0);
  i2s = esp_i2sbus_initialize(ESP32S3_I2S0);
  if (i2c == NULL || i2s == NULL)
    {
      return -ENODEV;
    }

  /* Playback: codec, PCM decoder, amplifier while reserved */

  codec = es8311_initialize(i2c, i2s, &g_es8311_out);
  pcm   = codec != NULL ? pcm_decode_initialize(codec) : NULL;
  if (pcm == NULL)
    {
      auderr("ERROR: Failed to set up the ES8311 output\n");
      return -ENODEV;
    }

  g_out_orig         = pcm->ops;
  g_out_ops          = *pcm->ops;
  g_out_ops.reserve  = tdeck_out_reserve;
  g_out_ops.release  = tdeck_out_release;
  g_out_ops.shutdown = tdeck_out_shutdown;
  pcm->ops           = &g_out_ops;

  ret = audio_register("pcm0", pcm);
  if (ret < 0)
    {
      auderr("ERROR: Failed to register pcm0: %d\n", ret);
      return ret;
    }

  /* Recording: the codec's microphone input */

  codec = es8311_initialize(i2c, i2s, &g_es8311_in);
  if (codec == NULL)
    {
      auderr("ERROR: Failed to set up the ES8311 input\n");
      return -ENODEV;
    }

  ret = audio_register("pcm_in0", codec);
  if (ret < 0)
    {
      auderr("ERROR: Failed to register pcm_in0: %d\n", ret);
    }

  return ret;
}

#endif /* CONFIG_AUDIO_ES8311 && CONFIG_ESPRESSIF_I2S0 && PCA9555 */
