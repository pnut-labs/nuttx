/****************************************************************************
 * boards/xtensa/esp32s3/lilygo-tdeck-max/src/esp32s3_bringup.c
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

#include <syslog.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_ESP32S3_I2C
#  include "esp32s3_i2c.h"
#endif

#include "esp32s3_reset_reasons.h"

#include "lilygo-tdeck-max.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_bringup
 *
 * Description:
 *   Bring up the board's devices.  Called from board_late_initialize().
 *
 * Returned Value:
 *   Zero (OK); a device that fails is logged, and the rest still start.
 *
 ****************************************************************************/

int esp32s3_bringup(void)
{
#ifdef CONFIG_ESP32S3_I2C0
  FAR struct i2c_master_s *i2c;
#endif
  int ret = OK;

  /* Mark each start: a RAM log in .noinit keeps the starts before it, and
   * holds random bytes after a power-up until something is written.
   */

  syslog(LOG_INFO, "Started (reset reason %d)\n",
         (int)esp32s3_reset_reasons(0));

#ifdef CONFIG_FS_PROCFS
  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_TMPFS
  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at %s: %d\n",
             CONFIG_LIBC_TMPDIR, ret);
    }
#endif

#ifdef CONFIG_ESP32S3_I2C0
  /* The I2C bus, and on it the XL9555, which powers most of the board */

  i2c = esp32s3_i2cbus_initialize(0);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize I2C0\n");
    }
  else
    {
#  ifdef CONFIG_I2C_DRIVER
      ret = i2c_register(i2c, 0);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register /dev/i2c0: %d\n", ret);
        }
#  endif

#  ifdef CONFIG_IOEXPANDER_PCA9555
      ret = tdeck_xl9555_initialize(i2c);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to initialize the XL9555: %d\n",
                 ret);
        }
#  endif
    }
#endif

  UNUSED(ret);
  return OK;
}
