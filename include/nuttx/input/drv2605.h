/****************************************************************************
 * include/nuttx/input/drv2605.h
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

#ifndef __INCLUDE_NUTTX_INPUT_DRV2605_H
#define __INCLUDE_NUTTX_INPUT_DRV2605_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_FF_DRV2605

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The DRV2605 answers at this 7-bit I2C address */

#define DRV2605_I2C_ADDRESS    0x5a

/* The DRV2605 is registered as a force feedback device and plays one effect
 * at a time.  The effect types it takes:
 *
 *   FF_CONSTANT  Run the motor at abs(u.constant.level) for replay.length
 *                milliseconds, or until stopped if the length is 0.
 *   FF_RUMBLE    The same, at the larger of the two magnitudes.
 *   FF_PERIODIC  With the FF_CUSTOM waveform only: play effects from the
 *                chip's built-in library, custom_data[0] to
 *                custom_data[custom_len - 1], at most 8 of them, each
 *                between 1 and DRV2605_LIBRARY_EFFECTS.  The TI datasheet
 *                lists them ("Waveform Library Effects List"); 1 is a
 *                strong click, 47 a buzz.
 *
 * replay.delay is honoured.  FF_GAIN scales FF_CONSTANT and FF_RUMBLE; the
 * library effects play at their own strength.  Writing a play count above
 * one plays the effect once.
 */

#define DRV2605_LIBRARY_EFFECTS 123
#define DRV2605_SEQUENCE_LEN    8

/* ioctl: set the int that arg points to 1 while an effect is pending or
 * playing, 0 once the motor is idle.  The force feedback interface has no
 * way to tell when a library effect has finished, and uploaded effects
 * outlive the file that uploaded them, so a program that plays an effect
 * and then erases it needs this to know when it can.
 */

#define DRV2605IOC_BUSY         _FFIOC(0x80)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct i2c_master_s;

/****************************************************************************
 * Name: drv2605_register
 *
 * Description:
 *   Configure a DRV2605 or DRV2605L haptic driver for an ERM (eccentric
 *   rotating mass) motor in open loop, and register it as a force feedback
 *   device.  LRA motors are not supported.
 *
 *   The chip must already be powered and enabled.  It is left in standby
 *   between effects.
 *
 * Input Parameters:
 *   devpath   - The device to register, e.g. "/dev/input_ff0"
 *   i2c       - An instance of the I2C interface to use
 *   addr      - The I2C address, normally DRV2605_I2C_ADDRESS
 *   frequency - The I2C frequency
 *   library   - The built-in effect library, 1 to 5: the ERM libraries A
 *               to E, which differ in drive timing for different motors
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure, -ENODEV if no
 *   DRV2605 answers at the address.
 *
 ****************************************************************************/

int drv2605_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr, uint32_t frequency, uint8_t library);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_FF_DRV2605 */
#endif /* __INCLUDE_NUTTX_INPUT_DRV2605_H */
