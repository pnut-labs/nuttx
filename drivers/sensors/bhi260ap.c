/****************************************************************************
 * drivers/sensors/bhi260ap.c
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
 * Driver for the Bosch BHI260AP smart sensor hub: an accelerometer and a
 * gyroscope with a microcontroller that runs a firmware image uploaded by
 * the host.  The protocol follows Bosch's BHY2 SensorAPI.
 *
 * The host interface is a set of registers.  Four of them are channels:
 *
 *   0x00  commands to the chip: a 16-bit command, a 16-bit length (in
 *         32-bit words for a firmware upload, bytes otherwise), and the
 *         payload padded to a multiple of 4 bytes.  Long commands are
 *         written in pieces, each piece a write to 0x00.
 *   0x01  wake-up FIFO } reading starts with a 16-bit count of the bytes
 *   0x02  non-wake FIFO} in it; the rest is read in pieces from the same
 *                        register
 *   0x03  status: a 16-bit code, a 16-bit length, then the data, used for
 *         parameter reads
 *
 * The FIFOs hold events, each a sensor id followed by its data: 7 bytes in
 * all for a 3-axis sensor, and fixed sizes for the timestamp and meta
 * events that the firmware adds.
 *
 * The chip has no firmware of its own.  This driver uploads the image
 * given by the board when the first sensor is activated, and leaves it
 * running.  The accelerometer and the gyroscope are the firmware's
 * passthrough sensors, with its default ranges of 8 g and 2000 deg/s.
 *
 * Besides them, the firmware works out "virtual sensors" from the
 * readings: steps, gestures, stillness, activity.  Which ones an image has
 * is known only once it runs: every topic is registered, and one the image
 * lacks fails to activate with -ENODEV.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/bhi260ap.h>

#ifdef CONFIG_SENSORS_BHI260AP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_HPWORK
#  error "The BHI260AP driver needs the high priority work queue"
#endif

/* Registers */

#define BHI_REG_CHAN_CMD        0x00
#define BHI_REG_CHAN_FIFO_W     0x01
#define BHI_REG_CHAN_FIFO_NW    0x02
#define BHI_REG_CHAN_STATUS     0x03
#define BHI_REG_RESET_REQ       0x14
#define BHI_REG_PRODUCT_ID      0x1c
#define BHI_REG_KERNEL_VERSION  0x20
#define BHI_REG_BOOT_STATUS     0x25
#define BHI_REG_INT_STATUS      0x2d
#define BHI_REG_ERROR           0x2e

#define BHI_PRODUCT_ID          0x89
#define BHI_FW_MAGIC            0x662b

/* Boot status */

#define BHI_BST_HOST_READY      0x10
#define BHI_BST_FW_VERIFY_DONE  0x20
#define BHI_BST_FW_VERIFY_ERROR 0x40

/* Interrupt status */

#define BHI_IST_HOST            0x01      /* The interrupt is asserted */
#define BHI_IST_FIFO_W          0x06
#define BHI_IST_FIFO_NW         0x18
#define BHI_IST_STATUS          0x20
#define BHI_IST_DEBUG           0x40      /* On the status channel too */
#define BHI_IST_RESET           0x80      /* Reset, or a fault */

/* Commands, and parameters */

#define BHI_CMD_UPLOAD_RAM      0x0002
#define BHI_CMD_BOOT_RAM        0x0003
#define BHI_CMD_CONFIG_SENSOR   0x000d
#define BHI_PARAM_READ          0x1000
#define BHI_PARAM_PRESENT       0x011f    /* A bit per virtual sensor */
#define BHI_PARAM_PRESENT_SIZE  32
#define BHI_PARAM_SENSOR_INFO   0x0300
#define BHI_SENSOR_INFO_SIZE    28
#define BHI_SENSOR_INFO_EVSIZE  20

/* Virtual sensors */

#define BHI_ID_ACC              1         /* Accelerometer passthrough */
#define BHI_ID_GYRO             10        /* Gyroscope passthrough */
#define BHI_ID_TILT             48
#define BHI_ID_STEP_COUNTER     52
#define BHI_ID_GLANCE           59
#define BHI_ID_PICKUP           61
#define BHI_ID_ACTIVITY         63
#define BHI_ID_STATIONARY       75
#define BHI_ID_MOTION           77

/* What a virtual sensor's event holds after its id */

#define BHI_DATA_AXES           0         /* x, y, z: int16 each */
#define BHI_DATA_EVENT          1         /* Nothing: it happened */
#define BHI_DATA_COUNT          2         /* uint32 */
#define BHI_DATA_BITS           3         /* uint16 */

/* System events in the FIFOs, and their sizes including the id */

#define BHI_ID_PADDING          0
#define BHI_ID_SYSTEM           245       /* 245-255: fixed sizes below */

/* Transfers are made in pieces of this much.  Neither the chip nor the
 * ESP32-S3's I2C needs it; it bounds the buffers on the stack.
 */

#define BHI_CHUNK               64

/* Timing */

#define BHI_RESET_TRIES         15        /* x 10 ms */
#define BHI_BOOT_TRIES          100       /* x 50 ms */
#define BHI_STATUS_TRIES        50        /* x 10 ms */

/* Default rates, and units */

#define BHI_DEFAULT_RATE        50.0f     /* Hz, of the readings */
#define BHI_EVENT_RATE          1.0f      /* Hz, of the others */
#define BHI_ACC_SCALE           (9.80665f / 4096.0f)
#define BHI_GYRO_SCALE          (2000.0f / 32768.0f * (float)M_PI / 180.0f)

#define BHI_LATENCY_MAX         0xffffff  /* ms, the command's 24 bits */

#define BHI_NSENSORS            9

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bhi260ap_dev_s;

/* A virtual sensor this driver offers: a uORB type, or a topic of its own
 * (SENSOR_TYPE_CUSTOM) of struct sensor_event, named here
 */

struct bhi260ap_kind_s
{
  uint8_t id;                       /* The firmware's */
  uint8_t type;                     /* uORB's */
  uint8_t data;                     /* BHI_DATA_* */
  uint8_t minsize;                  /* Its event's size at least, the id
                                     * included */
  FAR const char *name;             /* A custom topic's */
};

struct bhi260ap_sensor_s
{
  struct sensor_lowerhalf_s lower;  /* Must be first */
  FAR struct bhi260ap_dev_s *dev;
  FAR const struct bhi260ap_kind_s *kind;
  uint8_t evsize;                   /* Its event size, id included; 0 while
                                     * the firmware lacks it */
  bool enabled;
  float rate;                       /* Hz */
  uint32_t latency;                 /* ms the chip may hold its events */
};

struct bhi260ap_dev_s
{
  struct bhi260ap_sensor_s sensor[BHI_NSENSORS];
  FAR struct i2c_master_s *i2c;
  FAR const struct bhi260ap_config_s *config;
  struct i2c_config_s i2cconfig;
  mutex_t lock;                     /* Serialises access to the chip */
  struct work_s work;               /* Drains the FIFOs */
  bool booted;                      /* The firmware is running */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bhi260ap_activate(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, bool enable);
static int bhi260ap_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR uint32_t *period_us);
static int bhi260ap_batch(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep,
                          FAR uint32_t *latency_us);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The virtual sensors offered, the first two the readings */

static const struct bhi260ap_kind_s g_bhi_kinds[BHI_NSENSORS] =
{
  {
    BHI_ID_ACC, SENSOR_TYPE_ACCELEROMETER, BHI_DATA_AXES, 7, NULL
  },
  {
    BHI_ID_GYRO, SENSOR_TYPE_GYROSCOPE, BHI_DATA_AXES, 7, NULL
  },
  {
    BHI_ID_TILT, SENSOR_TYPE_TILT_DETECTOR, BHI_DATA_EVENT, 1, NULL
  },
  {
    BHI_ID_STEP_COUNTER, SENSOR_TYPE_STEP_COUNTER, BHI_DATA_COUNT, 5, NULL
  },
  {
    BHI_ID_GLANCE, SENSOR_TYPE_GLANCE_GESTURE, BHI_DATA_EVENT, 1, NULL
  },
  {
    BHI_ID_PICKUP, SENSOR_TYPE_PICK_UP_GESTURE, BHI_DATA_EVENT, 1, NULL
  },
  {
    BHI_ID_MOTION, SENSOR_TYPE_MOTION_DETECT, BHI_DATA_EVENT, 1, NULL
  },
  {
    BHI_ID_STATIONARY, SENSOR_TYPE_CUSTOM, BHI_DATA_EVENT, 1,
    BHI260AP_STATIONARY
  },
  {
    BHI_ID_ACTIVITY, SENSOR_TYPE_CUSTOM, BHI_DATA_BITS, 3,
    BHI260AP_ACTIVITY
  },
};

/* The sizes of the system events, including the id */

static const uint8_t g_bhi_system_sizes[11] =
{
  2,  /* 245 small timestamp delta, wake-up */
  3,  /* 246 large timestamp delta, wake-up */
  6,  /* 247 full timestamp, wake-up */
  4,  /* 248 meta event, wake-up */
  0,  /* 249 */
  18, /* 250 debug message */
  2,  /* 251 small timestamp delta */
  3,  /* 252 large timestamp delta */
  6,  /* 253 full timestamp */
  4,  /* 254 meta event */
  1,  /* 255 filler */
};

static const struct sensor_ops_s g_bhi260ap_ops =
{
  .activate     = bhi260ap_activate,
  .set_interval = bhi260ap_set_interval,
  .batch        = bhi260ap_batch,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bhi260ap_read / bhi260ap_write
 ****************************************************************************/

static int bhi260ap_read(FAR struct bhi260ap_dev_s *dev, uint8_t reg,
                         FAR uint8_t *buffer, size_t len)
{
  return i2c_writeread(dev->i2c, &dev->i2cconfig, &reg, 1, buffer, len);
}

static int bhi260ap_write(FAR struct bhi260ap_dev_s *dev, uint8_t reg,
                          FAR const uint8_t *buffer, size_t len)
{
  uint8_t data[1 + BHI_CHUNK];

  DEBUGASSERT(len <= BHI_CHUNK);

  data[0] = reg;
  memcpy(&data[1], buffer, len);
  return i2c_write(dev->i2c, &dev->i2cconfig, data, len + 1);
}

/****************************************************************************
 * Name: bhi260ap_command
 *
 * Description:
 *   Write a command and its payload to the command channel, padded to a
 *   multiple of 4 bytes, in pieces the chip takes.
 *
 ****************************************************************************/

static int bhi260ap_command(FAR struct bhi260ap_dev_s *dev, uint16_t cmd,
                            FAR const uint8_t *payload, size_t len)
{
  uint8_t chunk[BHI_CHUNK];
  size_t padded = (len + 3) & ~3;
  size_t total = 4 + padded;
  size_t pos = 0;
  uint16_t count;
  size_t n;
  size_t i;
  int ret;

  count = cmd == BHI_CMD_UPLOAD_RAM ? padded / 4 : padded;

  while (pos < total)
    {
      n = total - pos < BHI_CHUNK ? total - pos : BHI_CHUNK;

      for (i = 0; i < n; i++)
        {
          size_t at = pos + i;

          if (at < 4)
            {
              uint16_t word = at < 2 ? cmd : count;

              chunk[i] = (at & 1) == 0 ? word & 0xff : word >> 8;
            }
          else
            {
              chunk[i] = at - 4 < len ? payload[at - 4] : 0;
            }
        }

      ret = bhi260ap_write(dev, BHI_REG_CHAN_CMD, chunk, n);
      if (ret < 0)
        {
          return ret;
        }

      pos += n;
    }

  return OK;
}

/****************************************************************************
 * Name: bhi260ap_wait_boot
 *
 * Description:
 *   Wait for the boot status to show the host interface ready and the
 *   firmware verified.
 *
 ****************************************************************************/

static int bhi260ap_wait_boot(FAR struct bhi260ap_dev_s *dev)
{
  uint8_t status;
  int i;

  for (i = 0; i < BHI_BOOT_TRIES; i++)
    {
      nxsched_usleep(50000);
      if (bhi260ap_read(dev, BHI_REG_BOOT_STATUS, &status, 1) >= 0 &&
          (status & BHI_BST_HOST_READY) != 0 &&
          (status & BHI_BST_FW_VERIFY_DONE) != 0)
        {
          return (status & BHI_BST_FW_VERIFY_ERROR) != 0 ? -EIO : OK;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: bhi260ap_get_parameter
 ****************************************************************************/

static int bhi260ap_get_parameter(FAR struct bhi260ap_dev_s *dev,
                                  uint16_t param, FAR uint8_t *buffer,
                                  size_t len)
{
  uint8_t header[4];
  uint8_t status;
  size_t length;
  int ret;
  int i;

  ret = bhi260ap_command(dev, param | BHI_PARAM_READ, NULL, 0);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < BHI_STATUS_TRIES; i++)
    {
      ret = bhi260ap_read(dev, BHI_REG_INT_STATUS, &status, 1);
      if (ret < 0)
        {
          return ret;
        }

      if ((status & BHI_IST_STATUS) != 0)
        {
          break;
        }

      nxsched_usleep(10000);
    }

  if (i == BHI_STATUS_TRIES)
    {
      return -ETIMEDOUT;
    }

  ret = bhi260ap_read(dev, BHI_REG_CHAN_STATUS, header, sizeof(header));
  if (ret < 0)
    {
      return ret;
    }

  length = header[2] | (header[3] << 8);
  if ((header[0] | (header[1] << 8)) != param || length > len)
    {
      return -EIO;
    }

  if (length > 0)
    {
      ret = bhi260ap_read(dev, BHI_REG_CHAN_STATUS, buffer, length);
    }

  return ret < 0 ? ret : (int)length;
}

/****************************************************************************
 * Name: bhi260ap_configure
 *
 * Description:
 *   Set a virtual sensor's rate, 0 to stop it, and how long the chip may
 *   hold its events before it tells the host (0: as they come).
 *
 ****************************************************************************/

static int bhi260ap_configure(FAR struct bhi260ap_dev_s *dev, uint8_t id,
                              float rate, uint32_t latency)
{
  uint8_t payload[8];
  uint32_t bits;

  memcpy(&bits, &rate, sizeof(bits));

  payload[0] = id;
  payload[1] = bits;
  payload[2] = bits >> 8;
  payload[3] = bits >> 16;
  payload[4] = bits >> 24;
  payload[5] = latency;
  payload[6] = latency >> 8;
  payload[7] = latency >> 16;

  return bhi260ap_command(dev, BHI_CMD_CONFIG_SENSOR, payload,
                          sizeof(payload));
}

/****************************************************************************
 * Name: bhi260ap_find
 *
 * Description:
 *   The sensor with this virtual sensor id that the firmware has, or NULL.
 *
 ****************************************************************************/

static FAR struct bhi260ap_sensor_s *
bhi260ap_find(FAR struct bhi260ap_dev_s *dev, uint8_t id)
{
  int i;

  for (i = 0; i < BHI_NSENSORS; i++)
    {
      if (dev->sensor[i].kind->id == id && dev->sensor[i].evsize > 0)
        {
          return &dev->sensor[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: bhi260ap_event
 *
 * Description:
 *   Pass one sensor event on.  Its whole size is in the buffer, the size
 *   bhi260ap_event_size() gave, which is the kind's at least.
 *
 ****************************************************************************/

static void bhi260ap_event(FAR struct bhi260ap_dev_s *dev,
                           FAR const uint8_t *event, uint64_t timestamp)
{
  FAR struct bhi260ap_sensor_s *sensor;
  FAR const uint8_t *data = &event[1];

  /* The system events, which can be as short as one byte at the end of
   * what was read, are not passed on
   */

  sensor = bhi260ap_find(dev, event[0]);
  if (sensor == NULL)
    {
      return;
    }

  switch (sensor->kind->data)
    {
      case BHI_DATA_AXES:
        {
          float x = (int16_t)(data[0] | (data[1] << 8));
          float y = (int16_t)(data[2] | (data[3] << 8));
          float z = (int16_t)(data[4] | (data[5] << 8));

          if (sensor->kind->type == SENSOR_TYPE_ACCELEROMETER)
            {
              struct sensor_accel accel;

              accel.timestamp   = timestamp;
              accel.x           = x * BHI_ACC_SCALE;
              accel.y           = y * BHI_ACC_SCALE;
              accel.z           = z * BHI_ACC_SCALE;
              accel.temperature = NAN;
              sensor->lower.push_event(sensor->lower.priv, &accel,
                                       sizeof(accel));
            }
          else
            {
              struct sensor_gyro gyro;

              gyro.timestamp   = timestamp;
              gyro.x           = x * BHI_GYRO_SCALE;
              gyro.y           = y * BHI_GYRO_SCALE;
              gyro.z           = z * BHI_GYRO_SCALE;
              gyro.temperature = NAN;
              sensor->lower.push_event(sensor->lower.priv, &gyro,
                                       sizeof(gyro));
            }
        }
        break;

      case BHI_DATA_COUNT:
        {
          struct sensor_step_counter steps;

          steps.timestamp = timestamp;
          steps.steps     = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                            ((uint32_t)data[2] << 16) |
                            ((uint32_t)data[3] << 24);
          steps.cadence   = 0;
          sensor->lower.push_event(sensor->lower.priv, &steps,
                                   sizeof(steps));
        }
        break;

      default:
        {
          struct sensor_event ev;

          ev.timestamp = timestamp;
          ev.event     = sensor->kind->data == BHI_DATA_BITS ?
                         (uint32_t)(data[0] | (data[1] << 8)) : 1;
          sensor->lower.push_event(sensor->lower.priv, &ev, sizeof(ev));
        }
        break;
    }
}

/****************************************************************************
 * Name: bhi260ap_event_size
 *
 * Description:
 *   The size of the event with this id, id included, or 0 if unknown.
 *
 ****************************************************************************/

static int bhi260ap_event_size(FAR struct bhi260ap_dev_s *dev, uint8_t id)
{
  int i;

  if (id == BHI_ID_PADDING)
    {
      return 1;
    }

  if (id >= BHI_ID_SYSTEM)
    {
      return g_bhi_system_sizes[id - BHI_ID_SYSTEM];
    }

  for (i = 0; i < BHI_NSENSORS; i++)
    {
      if (dev->sensor[i].kind->id == id && dev->sensor[i].evsize > 0)
        {
          return dev->sensor[i].evsize;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: bhi260ap_drain
 *
 * Description:
 *   Read one FIFO to the end and pass its sensor events on.  Events can
 *   straddle the pieces the FIFO is read in, so whatever is left of one
 *   piece is kept for the next.
 *
 ****************************************************************************/

static int bhi260ap_drain(FAR struct bhi260ap_dev_s *dev, uint8_t reg)
{
  uint8_t buffer[2 * BHI_CHUNK];
  uint64_t timestamp;
  size_t remain;
  size_t have = 0;
  size_t pos;
  size_t n;
  bool skip = false;
  int size;
  int ret;

  ret = bhi260ap_read(dev, reg, buffer, 2);
  if (ret < 0)
    {
      return ret;
    }

  remain = buffer[0] | (buffer[1] << 8);
  timestamp = sensor_get_timestamp();

  while (remain > 0)
    {
      n = remain < BHI_CHUNK ? remain : BHI_CHUNK;
      ret = bhi260ap_read(dev, reg, &buffer[have], n);
      if (ret < 0)
        {
          return ret;
        }

      remain -= n;
      have   += n;

      if (skip)
        {
          have = 0;   /* Read to the end, so that the interrupt clears */
          continue;
        }

      for (pos = 0; pos < have; pos += size)
        {
          size = bhi260ap_event_size(dev, buffer[pos]);
          if (size == 0)
            {
              /* No way to tell where the next event starts: drop the rest
               * of this FIFO's content.
               */

              snerr("ERROR: Unknown event %u in the FIFO\n", buffer[pos]);
              skip = true;
              pos  = have;
              break;
            }

          if (pos + size > have)
            {
              break;  /* Its end comes with the next piece */
            }

          bhi260ap_event(dev, &buffer[pos], timestamp);
        }

      memmove(buffer, &buffer[pos], have - pos);
      have -= pos;
    }

  return OK;
}

/****************************************************************************
 * Name: bhi260ap_worker
 *
 * Description:
 *   Drain the FIFOs until the chip has nothing more for the host, which
 *   releases the (level triggered) interrupt line.
 *
 ****************************************************************************/

static void bhi260ap_worker(FAR void *arg)
{
  FAR struct bhi260ap_dev_s *dev = arg;
  uint8_t header[4];
  uint8_t status = 0;
  uint8_t junk[BHI_CHUNK];
  uint8_t error = 0;
  size_t length;
  bool fault = false;
  int strikes = 0;
  int tries;

  nxmutex_lock(&dev->lock);

  for (tries = 0; tries < 8; tries++)
    {
      /* A read that fails, or the interrupt asserted with nothing this
       * driver knows, may pass: it is a fault only if it happens twice in
       * a row at the end of the loop.  A reset or fault the chip reports is
       * one at once.
       */

      if (bhi260ap_read(dev, BHI_REG_INT_STATUS, &status, 1) < 0)
        {
          strikes++;
          nxsched_usleep(1000);
          continue;
        }

      if ((status & BHI_IST_RESET) != 0)
        {
          fault = true;
          break;
        }

      if ((status & (BHI_IST_FIFO_W | BHI_IST_FIFO_NW | BHI_IST_STATUS |
                     BHI_IST_DEBUG)) == 0)
        {
          if ((status & BHI_IST_HOST) == 0)
            {
              strikes = 0;
              break;  /* Nothing more for the host */
            }

          strikes++;
          nxsched_usleep(1000);
          continue;
        }

      strikes = 0;

      if ((status & BHI_IST_FIFO_W) != 0)
        {
          bhi260ap_drain(dev, BHI_REG_CHAN_FIFO_W);
        }

      if ((status & BHI_IST_FIFO_NW) != 0)
        {
          bhi260ap_drain(dev, BHI_REG_CHAN_FIFO_NW);
        }

      if ((status & (BHI_IST_STATUS | BHI_IST_DEBUG)) != 0 &&
          bhi260ap_read(dev, BHI_REG_CHAN_STATUS, header, 4) >= 0)
        {
          /* Nobody asked: throw it away */

          for (length = header[2] | (header[3] << 8); length > 0; )
            {
              size_t n = length < sizeof(junk) ? length : sizeof(junk);

              if (bhi260ap_read(dev, BHI_REG_CHAN_STATUS, junk, n) < 0)
                {
                  break;
                }

              length -= n;
            }
        }
    }

  if (strikes >= 2)
    {
      fault = true;
    }

  if (fault)
    {
      /* The firmware stopped, or the chip does not answer: leave the
       * interrupt off, which would otherwise stay asserted, until a sensor
       * is activated again and the firmware uploaded anew
       */

      bhi260ap_read(dev, BHI_REG_ERROR, &error, 1);
      snerr("ERROR: The chip stopped (status 0x%02x, error 0x%02x)\n",
            status, error);
      dev->booted = false;
    }
  else
    {
      dev->config->enable(dev->config, true);
    }

  nxmutex_unlock(&dev->lock);
}

/****************************************************************************
 * Name: bhi260ap_interrupt
 ****************************************************************************/

static int bhi260ap_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct bhi260ap_dev_s *dev = arg;

  /* Level triggered: masked until the worker has drained the FIFOs */

  dev->config->enable(dev->config, false);
  work_queue(HPWORK, &dev->work, bhi260ap_worker, dev, 0);
  return OK;
}

/****************************************************************************
 * Name: bhi260ap_boot
 *
 * Description:
 *   Reset the chip, upload the firmware to its program RAM and start it,
 *   then learn the sizes of the events it will send.
 *
 ****************************************************************************/

static int bhi260ap_boot(FAR struct bhi260ap_dev_s *dev)
{
  const uint8_t reset = 0x01;
  uint8_t present[BHI_PARAM_PRESENT_SIZE];
  uint8_t info[BHI_SENSOR_INFO_SIZE];
  uint8_t status;
  uint8_t version[2];
  int ret;
  int i;

  if (dev->config->firmware == NULL || dev->config->firmware_size < 4 ||
      (dev->config->firmware[0] | (dev->config->firmware[1] << 8)) !=
      BHI_FW_MAGIC)
    {
      snerr("ERROR: No BHI260AP firmware image\n");
      return -ENOENT;
    }

  ret = bhi260ap_write(dev, BHI_REG_RESET_REQ, &reset, 1);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < BHI_RESET_TRIES; i++)
    {
      nxsched_usleep(10000);
      if (bhi260ap_read(dev, BHI_REG_BOOT_STATUS, &status, 1) >= 0 &&
          (status & BHI_BST_HOST_READY) != 0)
        {
          break;
        }
    }

  if (i == BHI_RESET_TRIES)
    {
      snerr("ERROR: BHI260AP did not come out of reset\n");
      return -ETIMEDOUT;
    }

  sninfo("Uploading %zu bytes of firmware\n", dev->config->firmware_size);

  ret = bhi260ap_command(dev, BHI_CMD_UPLOAD_RAM, dev->config->firmware,
                         dev->config->firmware_size);
  if (ret >= 0)
    {
      ret = bhi260ap_wait_boot(dev);
    }

  if (ret < 0)
    {
      snerr("ERROR: Firmware upload failed: %d\n", ret);
      return ret;
    }

  ret = bhi260ap_command(dev, BHI_CMD_BOOT_RAM, NULL, 0);
  if (ret >= 0)
    {
      ret = bhi260ap_wait_boot(dev);
    }

  if (ret >= 0)
    {
      ret = bhi260ap_read(dev, BHI_REG_KERNEL_VERSION, version, 2);
    }

  if (ret < 0 || (version[0] | version[1]) == 0)
    {
      snerr("ERROR: Firmware did not start: %d\n", ret);
      return ret < 0 ? ret : -EIO;
    }

  sninfo("Firmware running, kernel version %u\n",
         version[0] | (version[1] << 8));

  /* Throw away what the firmware reported while starting */

  bhi260ap_drain(dev, BHI_REG_CHAN_FIFO_W);
  bhi260ap_drain(dev, BHI_REG_CHAN_FIFO_NW);

  /* Which of the virtual sensors offered this image has, and the sizes of
   * their events
   */

  ret = bhi260ap_get_parameter(dev, BHI_PARAM_PRESENT, present,
                               sizeof(present));
  if (ret != BHI_PARAM_PRESENT_SIZE)
    {
      snerr("ERROR: The firmware's sensors unknown: %d\n", ret);
      return ret < 0 ? ret : -EIO;
    }

  for (i = 0; i < BHI_NSENSORS; i++)
    {
      FAR struct bhi260ap_sensor_s *sensor = &dev->sensor[i];
      uint8_t id = sensor->kind->id;

      sensor->evsize = 0;
      if ((present[id / 8] & (1 << (id % 8))) == 0)
        {
          sninfo("The firmware has no sensor %u\n", id);
          continue;
        }

      ret = bhi260ap_get_parameter(dev, BHI_PARAM_SENSOR_INFO + id, info,
                                   sizeof(info));
      if (ret != BHI_SENSOR_INFO_SIZE || info[0] != id ||
          info[BHI_SENSOR_INFO_EVSIZE] < sensor->kind->minsize ||
          info[BHI_SENSOR_INFO_EVSIZE] > BHI_CHUNK)
        {
          snerr("ERROR: Sensor %u's information: %d\n", id, ret);
          return ret < 0 ? ret : -EIO;
        }

      sensor->evsize = info[BHI_SENSOR_INFO_EVSIZE];
    }

  dev->booted = true;

  /* Sensors that were on when the firmware stopped come back on */

  for (i = 0; i < BHI_NSENSORS; i++)
    {
      FAR struct bhi260ap_sensor_s *sensor = &dev->sensor[i];

      if (sensor->enabled && sensor->evsize > 0)
        {
          bhi260ap_configure(dev, sensor->kind->id, sensor->rate,
                             sensor->latency);
        }
    }

  dev->config->enable(dev->config, true);
  return OK;
}

/****************************************************************************
 * Name: bhi260ap_activate
 ****************************************************************************/

static int bhi260ap_activate(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, bool enable)
{
  FAR struct bhi260ap_sensor_s *sensor =
    (FAR struct bhi260ap_sensor_s *)lower;
  FAR struct bhi260ap_dev_s *dev = sensor->dev;
  int ret = OK;

  nxmutex_lock(&dev->lock);

  if (enable && !dev->booted)
    {
      ret = bhi260ap_boot(dev);
    }

  if (ret >= 0 && dev->booted && sensor->evsize == 0)
    {
      /* Not in this firmware image */

      sensor->enabled = false;
      ret = enable ? -ENODEV : OK;
    }
  else if (ret >= 0 && dev->booted)
    {
      ret = bhi260ap_configure(dev, sensor->kind->id,
                               enable ? sensor->rate : 0, sensor->latency);
      if (ret >= 0)
        {
          sensor->enabled = enable;
        }
    }
  else if (ret >= 0)
    {
      sensor->enabled = false;  /* Off while the firmware is not running */
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: bhi260ap_set_interval
 ****************************************************************************/

static int bhi260ap_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR uint32_t *period_us)
{
  FAR struct bhi260ap_sensor_s *sensor =
    (FAR struct bhi260ap_sensor_s *)lower;
  FAR struct bhi260ap_dev_s *dev = sensor->dev;
  int ret = OK;

  if (*period_us == 0)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);

  sensor->rate = 1000000.0f / *period_us;
  if (sensor->enabled && dev->booted)
    {
      ret = bhi260ap_configure(dev, sensor->kind->id, sensor->rate,
                               sensor->latency);
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: bhi260ap_batch
 *
 * Description:
 *   How long the chip may hold the sensor's events in its FIFO before it
 *   interrupts the host: the host sleeps meanwhile.  Kept in ms, the
 *   command's unit, up to its 24 bits.
 *
 ****************************************************************************/

static int bhi260ap_batch(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep,
                          FAR uint32_t *latency_us)
{
  FAR struct bhi260ap_sensor_s *sensor =
    (FAR struct bhi260ap_sensor_s *)lower;
  FAR struct bhi260ap_dev_s *dev = sensor->dev;
  uint32_t latency = *latency_us / 1000;
  int ret = OK;

  /* The command's field: the ioctl's microseconds never reach it */

  if (latency > BHI_LATENCY_MAX)
    {
      latency = BHI_LATENCY_MAX;
    }

  nxmutex_lock(&dev->lock);

  sensor->latency = latency;
  *latency_us     = latency * 1000;
  if (sensor->enabled && dev->booted)
    {
      ret = bhi260ap_configure(dev, sensor->kind->id, sensor->rate,
                               sensor->latency);
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: bhi260ap_sensor_register, bhi260ap_sensor_unregister
 *
 * Description:
 *   Register a sensor's topic: its uORB type's, or its own.
 *
 ****************************************************************************/

static int bhi260ap_sensor_register(FAR struct bhi260ap_sensor_s *sensor,
                                    int devno)
{
  char path[32];

  if (sensor->kind->type != SENSOR_TYPE_CUSTOM)
    {
      return sensor_register(&sensor->lower, devno);
    }

  snprintf(path, sizeof(path), "/dev/uorb/sensor_%s%d", sensor->kind->name,
           devno);
  return sensor_custom_register(&sensor->lower, path,
                                sizeof(struct sensor_event));
}

static void bhi260ap_sensor_unregister(FAR struct bhi260ap_sensor_s *sensor,
                                       int devno)
{
  char path[32];

  if (sensor->kind->type != SENSOR_TYPE_CUSTOM)
    {
      sensor_unregister(&sensor->lower, devno);
      return;
    }

  snprintf(path, sizeof(path), "/dev/uorb/sensor_%s%d", sensor->kind->name,
           devno);
  sensor_custom_unregister(&sensor->lower, path);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bhi260ap_register
 *
 * Description:
 *   See include/nuttx/sensors/bhi260ap.h
 *
 ****************************************************************************/

int bhi260ap_register(int devno, FAR struct i2c_master_s *i2c, uint8_t addr,
                      FAR const struct bhi260ap_config_s *config)
{
  FAR struct bhi260ap_dev_s *dev;
  uint8_t product = 0;
  int ret;
  int i;

  DEBUGASSERT(i2c != NULL && config != NULL && config->attach != NULL &&
              config->enable != NULL);

  dev = kmm_zalloc(sizeof(struct bhi260ap_dev_s));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->i2c                 = i2c;
  dev->config              = config;
  dev->i2cconfig.frequency = config->frequency;
  dev->i2cconfig.address   = addr;
  dev->i2cconfig.addrlen   = 7;
  nxmutex_init(&dev->lock);

  ret = bhi260ap_read(dev, BHI_REG_PRODUCT_ID, &product, 1);
  if (ret < 0 || product != BHI_PRODUCT_ID)
    {
      snerr("ERROR: No BHI260AP at 0x%02x: %d, id 0x%02x\n", addr, ret,
            product);
      ret = -ENODEV;
      goto errout;
    }

  for (i = 0; i < BHI_NSENSORS; i++)
    {
      FAR struct bhi260ap_sensor_s *sensor = &dev->sensor[i];
      bool axes = g_bhi_kinds[i].data == BHI_DATA_AXES;

      sensor->dev           = dev;
      sensor->kind          = &g_bhi_kinds[i];
      sensor->rate          = axes ? BHI_DEFAULT_RATE : BHI_EVENT_RATE;
      sensor->lower.type    = g_bhi_kinds[i].type;
      sensor->lower.ops     = &g_bhi260ap_ops;
      sensor->lower.nbuffer = axes ? 16 : 4;
    }

  ret = config->attach(config, bhi260ap_interrupt, dev);
  if (ret < 0)
    {
      goto errout;
    }

  for (i = 0; i < BHI_NSENSORS; i++)
    {
      ret = bhi260ap_sensor_register(&dev->sensor[i], devno);
      if (ret < 0)
        {
          snerr("ERROR: sensor_register failed: %d\n", ret);
          while (--i >= 0)
            {
              bhi260ap_sensor_unregister(&dev->sensor[i], devno);
            }

          goto errout;
        }
    }

  return OK;

errout:
  nxmutex_destroy(&dev->lock);
  kmm_free(dev);
  return ret;
}

#endif /* CONFIG_SENSORS_BHI260AP */
