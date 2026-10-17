/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_tickless.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_TICKLESS_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_TICKLESS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#ifdef CONFIG_SCHED_TICKLESS

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_tickless_next
 *
 * Description:
 *   Return the time until the scheduler's interval timer expires, in
 *   microseconds: 0 if it is due, UINT64_MAX if it is not running.  The
 *   light sleep logic uses it to wake up in time.
 *
 ****************************************************************************/

uint64_t esp32s3_tickless_next(void);

/****************************************************************************
 * Name: esp32s3_tickless_resync
 *
 * Description:
 *   Make sure the interval timer still fires after the counter jumped
 *   forward (a light sleep corrects it by the time slept): an alarm the
 *   counter passed without reaching it is set again, to fire at once.
 *
 ****************************************************************************/

void esp32s3_tickless_resync(void);

#endif /* CONFIG_SCHED_TICKLESS */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_TICKLESS_H */
