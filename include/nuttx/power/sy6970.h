/****************************************************************************
 * include/nuttx/power/sy6970.h
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

#ifndef __INCLUDE_NUTTX_POWER_SY6970_H
#define __INCLUDE_NUTTX_POWER_SY6970_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#if defined(CONFIG_BATTERY_CHARGER) && defined(CONFIG_SY6970)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The SY6970 answers at this 7-bit I2C address */

#define SY6970_I2C_ADDRESS 0x6a

/* Through the battery charger interface:
 *
 *   BATIOC_STATE          enum battery_status_e, from the charge status:
 *                         pre-charge and fast charge are CHARGING,
 *                         termination done is FULL, and not charging is
 *                         IDLE with input power or DISCHARGING without.
 *   BATIOC_HEALTH         enum battery_health_e, from the fault register,
 *                         which reports the faults latched since it was
 *                         last read, and clears them
 *   BATIOC_ONLINE         true while input power is good
 *   BATIOC_VOLTAGE        set the charge voltage, 3840-4608 mV
 *   BATIOC_CURRENT        set the fast charge current, 0-5056 mA
 *   BATIOC_INPUT_CURRENT  set the input current limit, 100-3250 mA; the
 *                         board's ILIM resistor limits it as well
 *   BATIOC_GET_VOLTAGE    the charge voltage currently set, mV
 *   BATIOC_CHIPID         register 0x14's part number, temperature
 *                         profile and revision (0x0c on the T-Deck Max)
 *   BATIOC_OPERATE        a struct batio_operate_msg_s:
 *                         BATIO_OPRTN_SHIPMODE or BATIO_OPRTN_SYSOFF
 *                         disconnects the battery (ship mode); without
 *                         input power this turns the device off until
 *                         power is plugged in.  BATIO_OPRTN_SYSON
 *                         connects it again.
 *
 * Settings are rounded down to the charger's steps.
 */

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
struct battery_charger_dev_s;

/****************************************************************************
 * Name: sy6970_initialize
 *
 * Description:
 *   Initialize the SY6970 battery charger and return an instance of the
 *   lower half interface, to be passed to battery_charger_register().
 *
 *   The charger is not reset, so it keeps charging with whatever it was
 *   set to.  Its I2C watchdog is disabled: once the host has written to it,
 *   an expired watchdog would put every setting back to its default.  Its
 *   input is taken out of high impedance mode, in which it would neither
 *   charge nor power the board.
 *
 * Input Parameters:
 *   i2c       - An instance of the I2C interface to use
 *   addr      - The I2C address of the charger, normally SY6970_I2C_ADDRESS
 *   frequency - The I2C frequency
 *
 * Returned Value:
 *   A pointer to the initialized lower-half driver instance, or NULL on
 *   failure.
 *
 ****************************************************************************/

FAR struct battery_charger_dev_s *
sy6970_initialize(FAR struct i2c_master_s *i2c, uint8_t addr,
                  uint32_t frequency);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_BATTERY_CHARGER && CONFIG_SY6970 */
#endif /* __INCLUDE_NUTTX_POWER_SY6970_H */
