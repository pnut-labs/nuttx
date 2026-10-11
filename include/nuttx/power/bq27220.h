/****************************************************************************
 * include/nuttx/power/bq27220.h
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

#ifndef __INCLUDE_NUTTX_POWER_BQ27220_H
#define __INCLUDE_NUTTX_POWER_BQ27220_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#if defined(CONFIG_BATTERY_GAUGE) && defined(CONFIG_BQ27220)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The BQ27220 answers at this 7-bit I2C address */

#define BQ27220_I2C_ADDRESS 0x55

/* The values returned through the battery gauge interface are plain
 * integers, in the units the battery gauge documentation gives
 * (Documentation/components/drivers/special/power/battery):
 *
 *   BATIOC_STATE        enum battery_status_e
 *   BATIOC_ONLINE       true if the gauge sees a battery
 *   BATIOC_VOLTAGE      millivolts
 *   BATIOC_CAPACITY     state of charge, percent
 *   BATIOC_CURRENT      milliamps, positive while charging
 *   BATIOC_TEMPERATURE  tenths of a degree Celsius
 *   BATIOC_CHIPID       the device number, 0x0220
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
struct battery_gauge_dev_s;

/****************************************************************************
 * Name: bq27220_initialize
 *
 * Description:
 *   Initialize the BQ27220 battery fuel gauge and return an instance of the
 *   lower half interface, to be passed to battery_gauge_register().
 *
 *   The gauge keeps its configuration in RAM, powered by the battery, so it
 *   survives resets of the host but not the battery being disconnected.  If
 *   capacity is not zero, the design capacity and the initial full charge
 *   capacity of the gauge's CEDV profile are checked against it and written
 *   if they differ.  That needs the gauge unsealed and a configuration
 *   update, and takes a couple of seconds; when the values already match,
 *   nothing is written.
 *
 * Input Parameters:
 *   i2c       - An instance of the I2C interface to use
 *   addr      - The I2C address of the gauge, normally BQ27220_I2C_ADDRESS
 *   frequency - The I2C frequency
 *   capacity  - The design capacity of the battery in mAh, or 0 to leave
 *               the gauge's configuration alone
 *
 * Returned Value:
 *   A pointer to the initialized lower-half driver instance, or NULL on
 *   failure, including when no BQ27220 answers at the address.
 *
 ****************************************************************************/

FAR struct battery_gauge_dev_s *
bq27220_initialize(FAR struct i2c_master_s *i2c, uint8_t addr,
                   uint32_t frequency, uint16_t capacity);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_BATTERY_GAUGE && CONFIG_BQ27220 */
#endif /* __INCLUDE_NUTTX_POWER_BQ27220_H */
