/****************************************************************************
 * boards/xtensa/esp32s3/lilygo-tdeck-max/src/esp32s3_xl9555.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/param.h>
#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/pca9555.h>
#include <nuttx/mutex.h>
#include <arch/board/board.h>

#include "espressif/esp_gpio.h"
#include "lilygo-tdeck-max.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define XL9555_ADDRESS    0x20
#define XL9555_FREQUENCY  400000

/* The configuration register of port 0 (P00-P07, so the modem's rail): a
 * bit set makes the line an input.  At power-up every line is an input.
 */

#define XL9555_REG_CONFIG0  0x06

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* One line of the XL9555, registered as /dev/<name> */

struct tdeck_line_s
{
#ifdef CONFIG_DEV_GPIO
  struct gpio_dev_s gpio;           /* Must be first */
#endif
  uint8_t pin;                      /* XL9555_* */
  bool boot;                        /* The level set at start */
  FAR const char *name;

  /* For a power rail whose part has lines on the ESP32-S3: called after
   * the rail comes on, and before it goes off.
   */

  CODE void (*powered)(bool on);
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void tdeck_lora_powered(bool on);

#ifdef CONFIG_DEV_GPIO
static int tdeck_line_set(FAR struct tdeck_line_s *line, bool on);
static int tdeck_line_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int tdeck_line_write(FAR struct gpio_dev_s *dev, bool value);
static int tdeck_line_setpintype(FAR struct gpio_dev_s *dev,
                                 enum gpio_pintype_e pintype);
static int tdeck_line_setdebounce(FAR struct gpio_dev_s *dev,
                                  unsigned long duration);
static int tdeck_line_setmask(FAR struct gpio_dev_s *dev, bool enable);
static int tdeck_line_describe(FAR struct gpio_dev_s *dev,
                               FAR char *extra, size_t len);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Every line, with the level it gets at start, in the order they are set.
 * The XL9555 keeps its outputs across a reset of the ESP32-S3, so each line
 * is set at every start; the modem's rail is the exception (see
 * tdeck_xl9555_initialize()).  At power-up the lines are inputs, pulled up,
 * so the modem's power key reads as pressed until it is set: it comes
 * first.
 *
 * The radios' and the amplifier's rails are off, except the LoRa radio's:
 * with its rail off, the radio is fed about 15 mA through its SPI chip
 * select, which must idle high for the bus it shares, while powered it
 * idles in standby.  The IMU's and the motor driver's rails are on, and
 * the touch and keyboard controllers are out of reset, so that every part
 * on the I2C bus answers.
 */

static struct tdeck_line_s g_lines[] =
{
  { .pin = XL9555_MODEM_PWRKEY, .boot = false, .name = "modem_pwrkey" },
  { .pin = XL9555_MODEM_PWR,    .boot = false, .name = "modem_pwr"    },
  { .pin = XL9555_LORA_EN,      .boot = true,  .name = "lora_en",
    .powered = tdeck_lora_powered                                       },
  { .pin = XL9555_GPS_EN,       .boot = false, .name = "gps_en"       },
  { .pin = XL9555_IMU_1V8_EN,   .boot = true,  .name = "imu_en"       },
  { .pin = XL9555_LORA_ANT,     .boot = true,  .name = "lora_ant"     },
  { .pin = XL9555_MOTOR_EN,     .boot = true,  .name = "motor_en"     },
  { .pin = XL9555_AMP_EN,       .boot = false, .name = "amp_en"       },
  { .pin = XL9555_TOUCH_RST,    .boot = true,  .name = "touch_rst"    },
  { .pin = XL9555_KEY_RST,      .boot = true,  .name = "key_rst"      },
  { .pin = XL9555_AUDIO_SEL,    .boot = false, .name = "audio_sel"    },
};

/* CONFIG_PCA9555_SHADOW_MODE must stay off: its shadow registers start at
 * zero, so the first change would drive every line of a port low, the
 * modem's rail included.
 */

static struct pca9555_config_s g_xl9555_config =
{
  .address   = XL9555_ADDRESS,
  .frequency = XL9555_FREQUENCY,
};

static FAR struct ioexpander_dev_s *g_xl9555;

#ifdef CONFIG_DEV_GPIO
/* Held while a line and the lines of its part change together */

static mutex_t g_lock = NXMUTEX_INITIALIZER;
#endif

#ifdef CONFIG_DEV_GPIO
static const struct gpio_operations_s g_line_ops =
{
  .go_read        = tdeck_line_read,
  .go_write       = tdeck_line_write,
  .go_setpintype  = tdeck_line_setpintype,
  .go_setdebounce = tdeck_line_setdebounce,
  .go_setmask     = tdeck_line_setmask,
  .go_describe    = tdeck_line_describe,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_lora_powered
 *
 * Description:
 *   The SX1262's reset follows its rail: held while the rail is off, so
 *   that the line does not feed the unpowered chip, and released once the
 *   rail is on.
 *
 ****************************************************************************/

static void tdeck_lora_powered(bool on)
{
  esp_gpiowrite(BOARD_LORA_RST, on);
}

#ifdef CONFIG_DEV_GPIO
/****************************************************************************
 * Name: tdeck_line_set
 *
 * Description:
 *   Set a line, and look after the lines of the part its rail powers.
 *
 ****************************************************************************/

static int tdeck_line_set(FAR struct tdeck_line_s *line, bool on)
{
  int ret;

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!on && line->powered != NULL)
    {
      line->powered(false);
    }

  ret = IOEXP_WRITEPIN(g_xl9555, line->pin, on);
  if (ret >= 0 && on && line->powered != NULL)
    {
      line->powered(true);
    }

  nxmutex_unlock(&g_lock);
  return ret < 0 ? ret : OK;
}
#endif

/****************************************************************************
 * Name: tdeck_modem_driven
 *
 * Description:
 *   Whether the modem's rail is driven on: its line an output, and high.
 *   Its level alone does not tell, since at power-up the line is an input,
 *   pulled up.
 *
 ****************************************************************************/

static int tdeck_modem_driven(FAR struct i2c_master_s *i2c,
                              FAR bool *driven)
{
  struct i2c_config_s config;
  uint8_t reg = XL9555_REG_CONFIG0;
  uint8_t dirs;
  bool level;
  int ret;

  config.frequency = XL9555_FREQUENCY;
  config.address   = XL9555_ADDRESS;
  config.addrlen   = 7;

  ret = i2c_writeread(i2c, &config, &reg, 1, &dirs, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = IOEXP_READPIN(g_xl9555, XL9555_MODEM_PWR, &level);
  if (ret < 0)
    {
      return ret;
    }

  *driven = (dirs & (1 << XL9555_MODEM_PWR)) == 0 && level;
  return OK;
}

#ifdef CONFIG_DEV_GPIO

/****************************************************************************
 * Name: tdeck_line_read
 ****************************************************************************/

static int tdeck_line_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct tdeck_line_s *line = (FAR struct tdeck_line_s *)dev;

  return IOEXP_READPIN(g_xl9555, line->pin, value);
}

/****************************************************************************
 * Name: tdeck_line_write
 ****************************************************************************/

static int tdeck_line_write(FAR struct gpio_dev_s *dev, bool value)
{
  return tdeck_line_set((FAR struct tdeck_line_s *)dev, value);
}

/****************************************************************************
 * Name: tdeck_line_setpintype
 *
 * Description:
 *   The lines are outputs, and stay so.
 *
 ****************************************************************************/

static int tdeck_line_setpintype(FAR struct gpio_dev_s *dev,
                                 enum gpio_pintype_e pintype)
{
  return pintype == GPIO_OUTPUT_PIN ? OK : -EINVAL;
}

/****************************************************************************
 * Name: tdeck_line_setdebounce
 ****************************************************************************/

static int tdeck_line_setdebounce(FAR struct gpio_dev_s *dev,
                                  unsigned long duration)
{
  return -ENOTSUP;
}

/****************************************************************************
 * Name: tdeck_line_setmask
 ****************************************************************************/

static int tdeck_line_setmask(FAR struct gpio_dev_s *dev, bool enable)
{
  return -ENOTSUP;
}

/****************************************************************************
 * Name: tdeck_line_describe
 ****************************************************************************/

static int tdeck_line_describe(FAR struct gpio_dev_s *dev,
                               FAR char *extra, size_t len)
{
  FAR struct tdeck_line_s *line = (FAR struct tdeck_line_s *)dev;

  snprintf(extra, len, "xl9555:P%d%d", line->pin / 8, line->pin % 8);
  return OK;
}

#endif /* CONFIG_DEV_GPIO */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_xl9555_initialize
 *
 * Description:
 *   Set every line of the XL9555 to its level at start, and register each
 *   as /dev/<name>.
 *
 *   The modem's rail is left on if it is driven on: the ESP32-S3 has been
 *   reset with the modem running, and cutting its power while it runs can
 *   damage its flash.  Whoever looks after the modem shuts it down in
 *   order.
 *
 * Input Parameters:
 *   i2c - The I2C bus the XL9555 is on.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int tdeck_xl9555_initialize(FAR struct i2c_master_s *i2c)
{
  FAR struct tdeck_line_s *line;
  bool driven;
  size_t i;
  int ret;

  g_xl9555 = pca9555_initialize(i2c, &g_xl9555_config);
  if (g_xl9555 == NULL)
    {
      return -ENODEV;
    }

  for (i = 0; i < nitems(g_lines); i++)
    {
      line = &g_lines[i];

      if (line->pin == XL9555_MODEM_PWR)
        {
          ret = tdeck_modem_driven(i2c, &driven);
          if (ret < 0)
            {
              return ret;
            }

          if (driven)
            {
              syslog(LOG_WARNING, "The modem is powered; left on\n");
              line->boot = true;
            }
        }

      /* The level first, then the direction, so that the line does not
       * glitch when it becomes an output; then the lines of the part it
       * powers, once the rail is driven.
       */

      ret = IOEXP_WRITEPIN(g_xl9555, line->pin, line->boot);
      if (ret >= 0)
        {
          ret = IOEXP_SETDIRECTION(g_xl9555, line->pin,
                                   IOEXPANDER_DIRECTION_OUT);
        }

      if (ret < 0)
        {
          return ret;
        }

      if (line->powered != NULL)
        {
          line->powered(line->boot);
        }

#ifdef CONFIG_DEV_GPIO
      line->gpio.gp_pintype = GPIO_OUTPUT_PIN;
      line->gpio.gp_ops     = &g_line_ops;

      /* A line that cannot be registered still has its level */

      ret = gpio_pin_register_byname(&line->gpio, line->name);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register /dev/%s: %d\n",
                 line->name, ret);
        }
#endif
    }

  return OK;
}
