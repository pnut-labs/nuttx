/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_dfs.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_DFS_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_DFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ESP32S3_DFS

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_dfs_initialize
 *
 * Description:
 *   Let the CPU frequency drop to CONFIG_ESP32S3_DFS_MIN_FREQ_MHZ while the
 *   CPU is idle.  The HAL's power management must already be initialized,
 *   which the startup functions do.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure, in which case
 *   the CPU keeps running at its full frequency.
 *
 ****************************************************************************/

int esp32s3_dfs_initialize(void);

#endif /* CONFIG_ESP32S3_DFS */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_DFS_H */
