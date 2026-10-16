/****************************************************************************
 * boards/xtensa/esp32s3/lilygo-tdeck-max/src/esp32s3_imu_fw.S
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
 * The firmware image the BHI260AP runs from its program RAM: Bosch's
 * standard BHI260AP image, BHI260AP.fw from Bosch Sensortec's BHI2xy
 * SensorAPI, under the BSD-3-Clause license.  The build downloads it (see
 * Make.defs) and checks it; it is not kept in this tree.
 ****************************************************************************/

  .section .rodata.bhi260ap_firmware, "a"
  .balign 4

  .global g_bhi260ap_firmware
  .global g_bhi260ap_firmware_end

g_bhi260ap_firmware:
  .incbin "bhi260ap.fw"
g_bhi260ap_firmware_end:
