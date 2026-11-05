/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_user.c
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

#include <stdint.h>

#include "xtensa.h"
#ifdef CONFIG_ESPRESSIF_SPIFLASH
#include "esp_private/cache_utils.h"
#elif defined(CONFIG_ESP32S3_SPIFLASH)
#include "esp32s3_spiflash.h"
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_user
 *
 * Description:
 *   ESP32-S3-specific user exception handler.
 *
 * Input Parameters:
 *   exccause - Identifies the EXCCAUSE of the user exception.
 *   regs     - The register save are at the time of the interrupt.
 *
 * Returned Value:
 *   Does not return.
 *
 ****************************************************************************/

uint32_t *xtensa_user(int exccause, uint32_t *regs)
{
#ifdef CONFIG_ESPRESSIF_SPIFLASH
  int cpu = this_cpu();

  if (!spi_flash_cache_enabled())
    {
      spi_flash_restore_cache(cpu, 0);
    }
#elif defined(CONFIG_ESP32S3_SPIFLASH)
  /* NuttX's own SPI flash driver: the same, so that the panic's code and
   * the RAM log, both behind the cache, can be reached
   */

  spiflash_panic_resume_cache();
#endif

  /* xtensa_user_panic never returns. */

  xtensa_user_panic(exccause, regs);

  while (1)
    {
    }
}
