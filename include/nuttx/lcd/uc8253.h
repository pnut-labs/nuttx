/****************************************************************************
 * include/nuttx/lcd/uc8253.h
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

#ifndef __INCLUDE_NUTTX_LCD_UC8253_H
#define __INCLUDE_NUTTX_LCD_UC8253_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/lcd/lcd_ioctl.h>

#ifdef CONFIG_LCD_UC8253

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ioctl() commands, passed on by the LCD framebuffer driver
 *
 * UC8253_IOC_FULLREFRESH - Make the next refresh a full one, which clears
 *   the ghosting partial refreshes leave behind; when to ask for it is the
 *   caller's choice.  Argument: none.
 */

#define UC8253_IOC_FULLREFRESH  _LCDIOC(UC8253_NIOCTL_BASE + 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* What the board provides: the controller's reset and busy lines */

struct uc8253_priv_s
{
  /* Drive the reset line: true asserts reset */

  CODE void (*set_rst)(bool on);

  /* Whether the controller is busy */

  CODE bool (*check_busy)(void);
};

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

/****************************************************************************
 * Name: uc8253_initialize
 *
 * Description:
 *   Bind an UC8253 e-paper controller to an SPI bus.  The panel is selected
 *   as SPIDEV_DISPLAY(0), and its data/command line driven through
 *   SPI_CMDDATA.  Nothing is sent to the panel until it is powered on.
 *
 * Input Parameters:
 *   spi  - The SPI bus.
 *   priv - The board's reset and busy lines.
 *
 * Returned Value:
 *   The LCD device, or NULL on failure.
 *
 ****************************************************************************/

struct spi_dev_s;
struct lcd_dev_s;

FAR struct lcd_dev_s *uc8253_initialize(FAR struct spi_dev_s *spi,
                                     FAR const struct uc8253_priv_s *priv);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LCD_UC8253 */
#endif /* __INCLUDE_NUTTX_LCD_UC8253_H */
