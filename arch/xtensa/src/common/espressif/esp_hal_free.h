/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_hal_free.h
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
 * Force-included into the Espressif HAL's component sources (hal.mk).
 *
 * The HAL releases memory it got from heap_caps_malloc() and friends with
 * plain free(), as ESP-IDF's single heap allows.  NuttX's heap_caps_malloc()
 * takes that memory from the kernel heap, so in a build with a separate
 * user heap (CONFIG_MM_KERNEL_HEAP) free() handed it to the wrong heap,
 * which corrupts both: the interrupt allocator did this on every
 * esp_intr_free(), for instance when a UART is closed.  Here free() in the
 * HAL goes back to whichever heap the block came from.
 ****************************************************************************/

#ifndef __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_HAL_FREE_H
#define __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_HAL_FREE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_MM_KERNEL_HEAP) && !defined(__ASSEMBLY__)

#include <stdbool.h>
#include <stdlib.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* From nuttx/kmalloc.h, which would pull in NuttX's own copy of the Xtensa
 * core definitions ahead of the HAL's.
 */

bool kmm_heapmember(FAR void *mem);
void kmm_free(FAR void *mem);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline void esp_hal_free(void *ptr)
{
  if (ptr != NULL && kmm_heapmember(ptr))
    {
      kmm_free(ptr);
    }
  else
    {
      free(ptr);
    }
}

#define free(ptr) esp_hal_free(ptr)

#endif /* CONFIG_MM_KERNEL_HEAP && !__ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_HAL_FREE_H */
