/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_mbedtls_mem.c
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
 * mbedtls's allocator for the Wi-Fi supplicant, in place of the HAL's
 * components/mbedtls/port/esp_mem.c.
 *
 * With none of the CONFIG_MBEDTLS_*_MEM_ALLOC choices set (the sdkconfig of
 * every Espressif chip here), the HAL's version allocates with calloc() but
 * frees with heap_caps_free(), which in NuttX belongs to the kernel heap.
 * In a build with a separate user heap (CONFIG_MM_KERNEL_HEAP) every free
 * was a cross-heap free: heap_caps_free() caught each one and logged an
 * error, dozens for each WPA2 handshake.  Here both ends use the same heap.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void *esp_mbedtls_mem_calloc(size_t n, size_t size)
{
  return calloc(n, size);
}

void esp_mbedtls_mem_free(void *ptr)
{
  free(ptr);
}
