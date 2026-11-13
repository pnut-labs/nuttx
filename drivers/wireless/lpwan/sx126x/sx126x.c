/****************************************************************************
 * drivers/wireless/lpwan/sx126x/sx126x.c
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
 * Included Files
 ****************************************************************************/

#include "sx126x.h"

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/config.h>
#include <nuttx/signal.h>

#include <nuttx/debug.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <poll.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wireless/ioctl.h>
#include <sched.h>
#include <stdint.h>
#include <string.h>
#include <sys/endian.h>
#include <unistd.h>
#include <nuttx/wireless/lpwan/sx126x.h>
#include <nuttx/wqueue.h>

/****************************************************************************
 * Private prototypes for file operations
 ****************************************************************************/

static int sx126x_open(FAR struct file *filep);

static int sx126x_close(FAR struct file *filep);

static ssize_t sx126x_read(FAR struct file *filep,
                           FAR char *buffer,
                           size_t buflen);

static ssize_t sx126x_write(FAR struct file *filep,
                            FAR const char *buf,
                            size_t buflen);

static int sx126x_ioctl(FAR struct file *filep,
                        int cmd,
                        unsigned long arg);

static int sx126x_poll(FAR struct file *filep,
                       FAR struct pollfd *fds,
                       bool setup);

/****************************************************************************
 * Private data types
 ****************************************************************************/

struct sx126x_dev_s
{
  struct spi_dev_s *spi;
  const struct sx126x_lower_s *lower;
  uint8_t times_opened;
  mutex_t lock;                           /* Only let one user in at a time */
  sem_t rx_sem;
  sem_t tx_sem;
  uint16_t irqbits;

  /* Hardware settings */

  bool invert_iq;

  /* Common settings */

  uint8_t payload_len;
  enum sx126x_packet_type_e packet_type;  /* This will decide what modulation to use */
  uint32_t frequency_hz;
  uint8_t power;
  uint16_t preambles;

  /* LoRa settings */

  enum sx126x_lora_sf_e lora_sf;
  enum sx126x_lora_bw_e lora_bw;
  enum sx126x_lora_cr_e lora_cr;
  bool lora_crc;
  bool lora_fixed_header;
  bool low_datarate_optimization;
  uint8_t syncword[SX126X_REG_SYNCWORD_LEN];  /* GFSK */
  uint16_t lora_syncword;
  uint32_t rx_timeout_ms;                 /* read() waits forever if 0 */
  uint8_t rx_payload_len;                 /* A fixed header's, as set */

  /* Listening in the background (SX126XIOC_LISTENSET): the packets heard
   * and not read, under qlock, which is taken after the SPI lock, never
   * before.  receiving is set when a packet's header is heard, cleared
   * (and rxend_sem posted) when the packet ends.
   */

  struct sx126x_listen_s listen;
  volatile bool receiving;
  bool asleep;                            /* Closed: the worker leaves it */
  sem_t rxend_sem;
  mutex_t qlock;
  struct sx126x_read_header_s queue[CONFIG_LPWAN_SX126X_RXQUEUE];
  uint8_t qhead;
  uint8_t qcount;
  FAR struct pollfd *fds[CONFIG_LPWAN_SX126X_NPOLLWAITERS];

  /* Interrupt handling */

  uint16_t irq_mask;
  struct work_s irq0_work;
};

enum sx126x_cmd_status
{
  SX126X_STATUS_RESERVED, SX126X_STATUS_RFU, SX126X_STATUS_DATA_AVAILABLE,
  SX126X_STATUS_TIMEOUT, SX126X_STATUS_ERROR, SX126X_STATUS_EXECUTE_FAIL,
  SX126X_STATUS_TX_DONE
};

enum sx126x_chip_mode
{
  SX126X_MODE_UNUSED, SX126X_MODE_RFU, SX126X_MODE_STBY_RC,
  SX126X_MODE_STBY_XOSC, SX126X_MODE_FS, SX126X_MODE_RX, SX126X_MODE_TX
};

struct sx126x_status_s
{
  enum sx126x_cmd_status cmd;
  enum sx126x_chip_mode mode;
};

static const struct file_operations sx126x_ops =
{
  sx126x_open,
  sx126x_close,
  sx126x_read,
  sx126x_write,
  NULL,
  sx126x_ioctl,
  NULL,
  NULL,
  sx126x_poll
};

/****************************************************************************
 * Globals
 ****************************************************************************/

FAR struct sx126x_dev_s g_sx126x_devices[SX126X_MAX_DEVICES];

/****************************************************************************
 * Private prototypes
 ****************************************************************************/

/* SPI and control **********************************************************/

static void sx126x_command(FAR struct sx126x_dev_s *dev,
                           uint8_t cmd,
                           FAR const uint8_t *params,
                           size_t paramslen,
                           FAR uint8_t *returns);

static void sx126x_reset(FAR struct sx126x_dev_s *dev);

static void sx126x_get_status(FAR struct sx126x_dev_s *dev,
                              FAR struct sx126x_status_s *status);

static void sx126x_spi_lock(FAR struct sx126x_dev_s *dev);

static void sx126x_spi_unlock(FAR struct sx126x_dev_s *dev);

static int sx126x_wait_busy(FAR struct sx126x_dev_s *dev);

/* Operational modes functions **********************************************/

static void sx126x_set_sleep(FAR struct sx126x_dev_s *dev);

static void sx126x_set_standby(FAR struct sx126x_dev_s *dev,
                               enum sx126x_standby_mode_e mode);

static void sx126x_calibrate(FAR struct sx126x_dev_s *dev, uint8_t blocks);

static void sx126x_calibrate_image(FAR struct sx126x_dev_s *dev,
                                   uint32_t frequency_hz);

static void sx126x_set_tx(FAR struct sx126x_dev_s *dev,
                          uint32_t timeout);

static void sx126x_set_rx(FAR struct sx126x_dev_s *dev, uint32_t timeout);

static void sx126x_set_cad(struct sx126x_dev_s *dev);

static void sx126x_set_tx_continuous_wave(FAR struct sx126x_dev_s *dev);

static void sx126x_set_regulator_mode(FAR struct sx126x_dev_s *dev,
                                      enum sx126x_regulator_mode_e mode);

static void sx126x_set_pa_config(FAR struct sx126x_dev_s *dev,
                                 enum sx126x_device_e model,
                                 uint8_t hpmax,
                                 uint8_t padutycycle);

static void sx126x_set_tx_infinite_preamble(FAR struct sx126x_dev_s *dev);

/* DIO and IRQ control functions ********************************************/

static void sx126x_set_dio_irq_params(FAR struct sx126x_dev_s *dev,
                                      uint16_t irq_mask,
                                      uint16_t dio1_mask,
                                      uint16_t dio2_mask,
                                      uint16_t dio3_mask);

static void sx126x_set_dio2_as_rf_switch(FAR struct sx126x_dev_s *dev,
                                         bool enable);

static void sx126x_set_dio3_as_tcxo(FAR struct sx126x_dev_s *dev,
                                    enum sx126x_tcxo_voltage_e voltage,
                                    uint32_t delay);

static void sx126x_get_irq_status(FAR struct sx126x_dev_s *dev,
                                  FAR uint16_t *irqstatus);

static void sx126x_clear_irq_status(FAR struct sx126x_dev_s *dev,
                                    uint16_t clearbits);

/* RF Modulation and Packet-Related Functions *******************************/

static void sx126x_set_packet_params_lora(FAR struct sx126x_dev_s *dev,
                                          FAR struct
                                          sx126x_packetparams_lora_s *
                                          pktparams);

static void sx126x_set_modulation_params_lora(FAR struct sx126x_dev_s *dev,
                                              FAR struct
                                              sx126x_modparams_lora_s *
                                              modparams);

static void sx126x_set_buffer_base_address(FAR struct sx126x_dev_s *dev,
                                           uint8_t tx,
                                           uint8_t rx);

static void sx126x_set_tx_params(FAR struct sx126x_dev_s *dev, uint8_t power,
                                 enum sx126x_ramp_time_e ramp_time);

static void sx126x_set_packet_type(FAR struct sx126x_dev_s *dev,
                                   enum sx126x_packet_type_e type);

static void sx126x_set_rf_frequency(FAR struct sx126x_dev_s *dev,
                                    uint32_t frequency_hz);

/* Communication status information *****************************************/

static void sx126x_get_rssi_inst(FAR struct sx126x_dev_s *dev,
                                 FAR int32_t *dbm);

static void sx126x_get_rx_buffer_status(FAR struct sx126x_dev_s *dev,
                                        uint8_t *status,
                                        uint8_t *payload_len,
                                        uint8_t *rx_buff_offset);

static void sx126x_get_packet_status_lora(FAR struct sx126x_dev_s *dev,
                                          FAR int16_t *rssi_dbm,
                                          FAR int32_t *snr);

/* Registers and buffer *****************************************************/

static void sx126x_write_register(FAR struct sx126x_dev_s *dev,
                                  uint16_t address,
                                  uint8_t *data,
                                  size_t data_length);

static void sx126x_read_register(FAR struct sx126x_dev_s *dev,
                                 uint16_t address,
                                 uint8_t *data,
                                 size_t data_length);

static void sx126x_update_register(FAR struct sx126x_dev_s *dev,
                                   uint16_t address, uint8_t mask,
                                   bool set);

static void sx126x_write_buffer(FAR struct sx126x_dev_s *dev,
                                uint8_t offset,
                                FAR const uint8_t *payload,
                                uint8_t len);

static void sx126x_read_buffer(FAR struct sx126x_dev_s *dev,
                               uint8_t offset,
                               FAR uint8_t *payload,
                               uint8_t len);

/* Register settings ********************************************************/

static void sx126x_set_syncword(FAR struct sx126x_dev_s *dev,
                                uint8_t *syncword,
                                uint8_t syncword_length);

static void sx126x_set_lora_syncword(FAR struct sx126x_dev_s *dev,
                                     uint16_t syncword);

/* Driver specific **********************************************************/

static int sx126x_init(FAR struct sx126x_dev_s *dev);

static int sx126x_deinit(FAR struct sx126x_dev_s *dev);

static int sx126x_setup_radio(FAR struct sx126x_dev_s *dev);

static void sx126x_set_defaults(FAR struct sx126x_dev_s *dev);

/* Listening ****************************************************************/

static void sx126x_wake(FAR struct sx126x_dev_s *dev);

static int sx126x_listen_arm(FAR struct sx126x_dev_s *dev);

static void sx126x_listen_take(FAR struct sx126x_dev_s *dev,
                               uint16_t irqbits);

static void sx126x_listen_drain(FAR struct sx126x_dev_s *dev);

static void sx126x_listen_pause(FAR struct sx126x_dev_s *dev);

static void sx126x_listen_flush(FAR struct sx126x_dev_s *dev);

static ssize_t sx126x_read_queued(FAR struct sx126x_dev_s *dev,
                                  FAR struct file *filep,
                                  FAR struct sx126x_read_header_s *header);

/* Interrupt handlers *******************************************************/

static int sx126x_irq0handler(int irq, FAR void *context, FAR void *arg);

static inline int sx126x_attachirq0(FAR struct sx126x_dev_s *dev, xcpt_t isr,
  FAR void *arg);

static void sx126x_isr0_process(FAR void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* File operations **********************************************************/

static int sx126x_open(FAR struct file *filep)
{
  int ret = 0;

  /* Get device */

  struct sx126x_dev_s *dev;

  dev = filep->f_inode->i_private;
  wlinfo("Opening SX126x %d", dev->lower->dev_number);

  /* Lock dev */

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Only one can open this dev at a time */

  if (dev->times_opened > 0)
    {
      ret = -EBUSY;
      goto exit_err;
    }

  /* Initialize */

  ret = sx126x_init(dev);
  if (ret != 0)
    {
      goto exit_err;
    }

  /* Success */

  dev->times_opened++;
  ret = OK;

exit_err:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int sx126x_close(FAR struct file *filep)
{
  int ret = 0;

  /* Get device */

  struct sx126x_dev_s *dev;

  dev = filep->f_inode->i_private;
  wlinfo("Closing SX126x %d", dev->lower->dev_number);

  /* Lock */

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      goto exit_err;
    }

  /* De-init: the chip sleeps, and the packets kept are dropped */

  ret = sx126x_deinit(dev);
  sx126x_listen_flush(dev);
  if (ret != 0)
    {
      goto exit_err;
    }

  /* Success */

  if (dev->times_opened > 0)
    {
      dev->times_opened--; /* Do not let this wrap around. */
      ret = OK;
    }

exit_err:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static ssize_t sx126x_read(FAR struct file *filep,
                           FAR char *buf,
                           size_t buflen)
{
  FAR struct sx126x_dev_s *dev = filep->f_inode->i_private;
  FAR struct sx126x_read_header_s *header =
    (FAR struct sx126x_read_header_s *)buf;
  uint8_t status;
  uint8_t offset;
  int ret;

  if (buf == NULL || buflen < sizeof(struct sx126x_read_header_s))
    {
      return -EINVAL;
    }

  /* Listening, a packet heard: it waits for none behind a send, so it
   * takes no lock but the queue's
   */

  if (dev->listen.mode != SX126X_LISTEN_OFF)
    {
      return sx126x_read_queued(dev, filep, header);
    }

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Listening may have begun meanwhile; once the lock is held, the mode
   * cannot change under the one-shot receive below
   */

  if (dev->listen.mode != SX126X_LISTEN_OFF)
    {
      nxmutex_unlock(&dev->lock);
      return sx126x_read_queued(dev, filep, header);
    }

  /* Receive continuously until a packet arrives.  The driver keeps the
   * timeout itself, so that it can be of any length.  With an explicit
   * header the payload length only sets the largest packet accepted.
   */

  sx126x_spi_lock(dev);

  dev->payload_len = dev->lora_fixed_header ? dev->rx_payload_len :
                                              SX126X_RX_PAYLOAD_SIZE;

  dev->irq_mask = SX126X_IRQ_RXDONE_MASK | SX126X_IRQ_CRCERR_MASK |
                  SX126X_IRQ_HEADERERR_MASK;
  ret = sx126x_setup_radio(dev);
  if (ret >= 0)
    {
      /* Forget a completion left over from an earlier, abandoned wait,
       * now that the chip's interrupt status is clear
       */

      while (nxsem_trywait(&dev->rx_sem) == OK);

      sx126x_set_rx(dev, SX126X_SETRX_CONTINUOUS);
    }

  sx126x_spi_unlock(dev);

  if (ret < 0)
    {
      goto out;
    }

  if (dev->rx_timeout_ms > 0)
    {
      /* In whole seconds and the rest, so that a long timeout does not
       * overflow the conversion to ticks
       */

      ret = nxsem_tickwait(&dev->rx_sem,
                           SEC2TICK(dev->rx_timeout_ms / 1000) +
                           MSEC2TICK(dev->rx_timeout_ms % 1000));
    }
  else
    {
      ret = nxsem_wait(&dev->rx_sem);
    }

  sx126x_spi_lock(dev);

  /* Leave receive mode first, so that no new packet overwrites this one */

  sx126x_set_standby(dev, SX126X_STDBY_RC);

  if (ret >= 0)
    {
      sx126x_get_rx_buffer_status(dev, &status, &header->payload_length,
                                  &offset);
      sx126x_read_buffer(dev, offset, header->payload,
                         header->payload_length);
      sx126x_get_packet_status_lora(dev, &header->rssi_db, &header->snr);
      header->crc_error = (dev->irqbits & SX126X_IRQ_CRCERR_MASK) != 0;
      ret = sizeof(struct sx126x_read_header_s);
    }

  sx126x_spi_unlock(dev);

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static ssize_t sx126x_write(FAR struct file *filep,
                            FAR const char *buf,
                            size_t buflen)
{
  FAR struct sx126x_dev_s *dev = filep->f_inode->i_private;
  bool listening;
  int ret;

  if (buf == NULL || buflen < 1 || buflen > SX126X_RX_PAYLOAD_SIZE)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Listening: between packets heard, then listen again */

  listening = dev->listen.mode != SX126X_LISTEN_OFF;
  if (listening)
    {
      sx126x_listen_pause(dev);
    }

  while (nxsem_trywait(&dev->tx_sem) == OK);

  sx126x_spi_lock(dev);

  dev->payload_len = buflen;
  dev->irq_mask = SX126X_IRQ_TXDONE_MASK;
  ret = sx126x_setup_radio(dev);
  if (ret >= 0)
    {
      sx126x_write_buffer(dev, 0, (FAR const uint8_t *)buf, buflen);
      sx126x_set_tx(dev, SX126X_SETTX_NO_TIMEOUT);
    }

  sx126x_spi_unlock(dev);

  if (ret >= 0)
    {
      /* The chip returns to standby by itself once the packet is out */

      wlinfo("TXing\n");
      ret = nxsem_tickwait_uninterruptible(&dev->tx_sem,
                                           MSEC2TICK(SX126X_TX_TIMEOUT_MS));
      if (ret < 0)
        {
          wlerr("ERROR: No TX done from the SX126x: %d\n", ret);
          sx126x_spi_lock(dev);
          sx126x_set_standby(dev, SX126X_STDBY_RC);
          sx126x_spi_unlock(dev);
        }
      else
        {
          ret = buflen;
        }
    }

  if (listening)
    {
      sx126x_spi_lock(dev);
      if (sx126x_listen_arm(dev) < 0)
        {
          wlerr("ERROR: The SX126x does not listen again\n");
        }

      sx126x_spi_unlock(dev);
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

static int sx126x_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct sx126x_dev_s *dev = filep->f_inode->i_private;
  int ret;

  wlinfo("IOCTL cmd %d SX126x dev_number %d\n", cmd,
         dev->lower->dev_number);

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      /* Set radio freq. Takes uint32_t *frequency in Hz */

      case WLIOC_SETRADIOFREQ:
        {
          FAR uint32_t *freq_ptr = (FAR uint32_t *)((uintptr_t)arg);

          DEBUGASSERT(freq_ptr != NULL);

          if (dev->lower->check_frequency(*freq_ptr) != 0)
            {
              ret = -EINVAL;
              break;
            }

          dev->frequency_hz = *freq_ptr;
          break;
        }

      /* Get radio freq. Sets uint32_t *frequency in Hz */

      case WLIOC_GETRADIOFREQ:
        {
          FAR uint32_t *freq_ptr = (FAR uint32_t *)((uintptr_t)arg);

          DEBUGASSERT(freq_ptr != NULL);

          *freq_ptr = dev->frequency_hz;
          break;
        }

      /* Set TX power. arg: Pointer to int8_t power value */

      case WLIOC_SETTXPOWER:
        {
          FAR int8_t *ptr = (FAR int8_t *)((uintptr_t)arg);

          DEBUGASSERT(ptr != NULL);

          dev->power = *ptr;
          break;
        }

      /* Get current TX power. arg: Pointer to int8_t power value */

      case WLIOC_GETTXPOWER:
        {
          FAR int8_t *ptr = (FAR int8_t *)((uintptr_t)arg);

          DEBUGASSERT(ptr != NULL);

          *ptr = dev->power;
          break;
        }

      /* TODO: Integration with new common IOCTL API */

      /* Driver specific IOCTL */

      /* Lora config */

      case SX126XIOC_LORACONFIGSET:
        {
          FAR struct sx126x_lora_config_s *ptr =
            (FAR struct sx126x_lora_config_s *)((uintptr_t)arg);

          DEBUGASSERT(ptr != NULL);

          /* Modulation params */

          dev->lora_sf = ptr->modulation.spreading_factor;
          dev->lora_bw = ptr->modulation.bandwidth;
          dev->lora_cr = ptr->modulation.coding_rate;
          dev->low_datarate_optimization =
            ptr->modulation.low_datarate_optimization;

          /* Packet params */

          dev->lora_crc = ptr->packet.crc_enable;
          dev->lora_fixed_header =
            ptr->packet.fixed_length_header;
          dev->payload_len = ptr->packet.payload_length;
          dev->rx_payload_len = ptr->packet.payload_length;
          dev->invert_iq = ptr->packet.invert_iq;
          dev->preambles = ptr->packet.preambles;

          break;
        }

      /* Receive timeout. arg: uint32_t *milliseconds, 0 = forever */

      case SX126XIOC_RXTIMEOUTSET:
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);

          DEBUGASSERT(ptr != NULL);

          dev->rx_timeout_ms = *ptr;
          break;
        }

      /* LoRa sync word. arg: uint16_t *syncword */

      case SX126XIOC_SYNCWORDSET:
        {
          FAR uint16_t *ptr = (FAR uint16_t *)((uintptr_t)arg);

          DEBUGASSERT(ptr != NULL);

          dev->lora_syncword = *ptr;
          break;
        }

      /* Listen in the background, or stop.  arg: struct sx126x_listen_s * */

      case SX126XIOC_LISTENSET:
        {
          FAR struct sx126x_listen_s *ptr =
            (FAR struct sx126x_listen_s *)((uintptr_t)arg);

          DEBUGASSERT(ptr != NULL);

          if (ptr->mode > SX126X_LISTEN_DUTY_CYCLE ||
              (ptr->mode == SX126X_LISTEN_DUTY_CYCLE &&
               (SX126X_DUTY_STEPS(ptr->rx_us) == 0 ||
                SX126X_DUTY_STEPS(ptr->rx_us) > SX126X_DUTY_STEPS_MAX ||
                SX126X_DUTY_STEPS(ptr->sleep_us) == 0 ||
                SX126X_DUTY_STEPS(ptr->sleep_us) > SX126X_DUTY_STEPS_MAX)))
            {
              ret = -EINVAL;
              break;
            }

          /* A packet the chip holds is kept before it is set up anew */

          sx126x_spi_lock(dev);
          if (dev->listen.mode != SX126X_LISTEN_OFF)
            {
              sx126x_listen_drain(dev);
            }

          dev->listen = *ptr;
          if (ptr->mode != SX126X_LISTEN_OFF)
            {
              ret = sx126x_listen_arm(dev);
              if (ret < 0)
                {
                  dev->listen.mode = SX126X_LISTEN_OFF;
                }
            }

          if (dev->listen.mode == SX126X_LISTEN_OFF)
            {
              sx126x_wake(dev);
              sx126x_set_standby(dev, SX126X_STDBY_RC);
            }

          sx126x_spi_unlock(dev);

          if (dev->listen.mode == SX126X_LISTEN_OFF)
            {
              sx126x_listen_flush(dev);
            }

          break;
        }

      default:
        ret = -ENOTTY;
        break;
    }

  /* A configuration changed while listening is used at once */

  if (ret >= 0 && dev->listen.mode != SX126X_LISTEN_OFF &&
      (cmd == WLIOC_SETRADIOFREQ || cmd == SX126XIOC_LORACONFIGSET ||
       cmd == SX126XIOC_SYNCWORDSET))
    {
      sx126x_spi_lock(dev);
      sx126x_listen_drain(dev);
      ret = sx126x_listen_arm(dev);
      sx126x_spi_unlock(dev);
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: sx126x_poll
 *
 * Description:
 *   POLLIN while listening with a packet kept, and when listening stops
 *   (a read then fails with -ECANCELED); POLLOUT always, though a send
 *   waits for the one before it and, listening, for a packet being heard.
 *
 ****************************************************************************/

static int sx126x_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup)
{
  FAR struct sx126x_dev_s *dev = filep->f_inode->i_private;
  FAR struct pollfd **slot;
  pollevent_t eventset;
  int ret;
  int i;

  ret = nxmutex_lock(&dev->qlock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      for (i = 0; i < CONFIG_LPWAN_SX126X_NPOLLWAITERS; i++)
        {
          if (dev->fds[i] == NULL)
            {
              dev->fds[i] = fds;
              fds->priv   = &dev->fds[i];
              break;
            }
        }

      if (i >= CONFIG_LPWAN_SX126X_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
        }
      else
        {
          eventset = POLLOUT;
          if (dev->qcount > 0)
            {
              eventset |= POLLIN;
            }

          poll_notify(&fds, 1, eventset);
        }
    }
  else if (fds->priv != NULL)
    {
      slot  = (FAR struct pollfd **)fds->priv;
      *slot = NULL;
      fds->priv = NULL;
    }

  nxmutex_unlock(&dev->qlock);
  return ret;
}

uint32_t sx126x_convert_freq_in_hz_to_pll_step(uint32_t freq_in_hz)
{
  uint32_t  steps_int;
  uint32_t  steps_frac;

  steps_int     = freq_in_hz / SX126X_PLL_STEP_SCALED;
  steps_frac    = freq_in_hz - (steps_int * SX126X_PLL_STEP_SCALED);

  return (steps_int <<
    SX126X_PLL_STEP_SHIFT_AMOUNT) + (((steps_frac <<
    SX126X_PLL_STEP_SHIFT_AMOUNT) + (SX126X_PLL_STEP_SCALED >>
                                     1)) / SX126X_PLL_STEP_SCALED);
}

/* Operational modes functions **********************************************/

static void sx126x_set_sleep(FAR struct sx126x_dev_s *dev)
{
  /* Cold start: only the wake-up logic stays on, about 160 nA.  The
   * configuration is lost, and is set up again before every operation.
   */

  uint8_t config = SX126X_SETSLEEP_CONF_START_COLD |
                   SX126X_SETSLEEP_CONF_RTC_DISABLE;

  sx126x_command(dev, SX126X_SETSLEEP, &config, SX126X_SETSLEEP_PARAMS,
                 NULL);
}

static void sx126x_set_standby(FAR struct sx126x_dev_s *dev,
                               enum sx126x_standby_mode_e mode)
{
  uint8_t param = mode;

  sx126x_command(dev, SX126X_SETSTANDBY, &param,
                 SX126X_SETSTANDBY_PARAMS, NULL);
}

static void sx126x_calibrate(FAR struct sx126x_dev_s *dev, uint8_t blocks)
{
  sx126x_command(dev, SX126X_CALIBRATE, &blocks, SX126X_CALIBRATE_PARAMS,
                 NULL);
}

static void sx126x_calibrate_image(FAR struct sx126x_dev_s *dev,
                                   uint32_t frequency_hz)
{
  uint8_t params[SX126X_CALIBRATEIMAGE_PARAMS];

  /* The receiver's image rejection is calibrated for one band at a time,
   * 902-928 MHz after a reset (datasheet 9.2.1, table 9-2).
   */

  if (frequency_hz > 900000000)
    {
      params[0] = 0xe1;
      params[1] = 0xe9;
    }
  else if (frequency_hz > 850000000)
    {
      params[0] = 0xd7;
      params[1] = 0xdb;
    }
  else if (frequency_hz > 770000000)
    {
      params[0] = 0xc1;
      params[1] = 0xc5;
    }
  else if (frequency_hz > 460000000)
    {
      params[0] = 0x75;
      params[1] = 0x81;
    }
  else
    {
      params[0] = 0x6b;
      params[1] = 0x6f;
    }

  sx126x_command(dev, SX126X_CALIBRATEIMAGE, params,
                 SX126X_CALIBRATEIMAGE_PARAMS, NULL);
}

static void sx126x_set_tx(FAR struct sx126x_dev_s *dev, uint32_t timeout)
{
  /* Convert timeout to BE24 */

  timeout = htobe32(timeout << 8);

  sx126x_command(dev, SX126X_SETTX, (uint8_t *)&timeout, SX126X_SETTX_PARAMS,
                 NULL);
}

static void sx126x_set_rx(FAR struct sx126x_dev_s *dev, uint32_t timeout)
{
  /* Convert timeout to BE24 */

  timeout = htobe32(timeout << 8);

  sx126x_command(dev, SX126X_SETRX, (uint8_t *)&timeout, SX126X_SETRX_PARAMS,
                 NULL);
}

static void sx126x_stop_timer_on_preamble(FAR struct sx126x_dev_s *dev,
                                          bool enable)
{
  sx126x_command(dev, SX126X_STOPTIMERONPREAMBLE, (uint8_t *)&enable,
                 SX126X_STOPTIMERONPREAMBLE_PARAMS, NULL);
}

static void sx126x_set_rx_duty_cycle(FAR struct sx126x_dev_s *dev,
                                     uint32_t rx_period,
                                     uint32_t sleep_period)
{
  uint8_t params[SX126X_SETRXDUTYCYCLE_PARAMS];

  rx_period     = htobe32(rx_period << 8);
  sleep_period  = htobe32(sleep_period << 8);

  memcpy(params + SX126X_SETRXDUTYCYCLE_RXPERIOD_PARAM,
         (uint8_t *)&rx_period,
         SX126X_SETRXDUTYCYCLE_RXPERIOD_PARAMS);
  memcpy(params + SX126X_SETRXDUTYCYCLE_SLEEPPERIOD_PARAM,
         (uint8_t *)&sleep_period,
         SX126X_SETRXDUTYCYCLE_SLEEPPERIOD_PARAMS);

  sx126x_command(dev, SX126X_SETRXDUTYCYCLE, params,
                 SX126X_SETRXDUTYCYCLE_PARAMS, NULL);
}

static void sx126x_set_cad(FAR struct sx126x_dev_s *dev)
{
  sx126x_command(dev, SX126X_SETCAD, NULL, 0, NULL);
}

static void sx126x_set_tx_continuous_wave(FAR struct sx126x_dev_s *dev)
{
  sx126x_command(dev, SX126X_SETTXCONTINUOUSWAVE, NULL, 0, NULL);
}

static void sx126x_set_regulator_mode(FAR struct sx126x_dev_s *dev,
                                      enum sx126x_regulator_mode_e mode)
{
  sx126x_command(dev, SX126X_SETREGULATORMODE, (uint8_t *)&mode,
                 SX126X_SETREGULATORMODE_PARAMS, NULL);
}

/* Caution! Exceeding the limits listed in DS_SX1261/2 V2.1
 * 13.1.14.1 may cause irreversible damage to the device
 */

static void sx126x_set_pa_config(FAR struct sx126x_dev_s *dev,
                                 enum sx126x_device_e model, uint8_t hpmax,
                                 uint8_t padutycycle)
{
  uint8_t params[SX126X_SETPACONFIG_PARMS];

  memset(params, 0, SX126X_SETPACONFIG_PARMS);

  params[SX126X_SETPACONFIG_PADUTYCYCLE_PARAM]  = padutycycle;
  params[SX126X_SETPACONFIG_HPMAX_PARAM]        = hpmax;
  params[SX126X_SETPACONFIG_DEVICESEL_PARAM]    = model;
  params[SX126X_SETPACONFIG_PALUT_PARAM]        = 0x01;

  sx126x_command(dev, SX126X_SETPACONFIG, params, SX126X_SETPACONFIG_PARMS,
                 NULL);
}

static void sx126x_set_tx_infinite_preamble(FAR struct sx126x_dev_s *dev)
{
  sx126x_command(dev, SX126X_SETTXINFINITEPREAMBLE, NULL, 0, NULL);
}

static void sx126x_set_rx_tx_fallback_mode(FAR struct sx126x_dev_s *dev,
                                           enum sx126x_fallback_mode_e
                                           fallback)
{
  sx126x_command(dev, SX126X_SETRXTXFALLBACKMODE, (uint8_t *)&fallback,
                 SX126X_SETRXTXFALLBACKMODE_PARAMS, NULL);
}

/* DIO and IRQ control functions */

static void sx126x_set_dio_irq_params(FAR struct sx126x_dev_s *dev,
                                      uint16_t irq_mask, uint16_t dio1_mask,
                                      uint16_t dio2_mask, uint16_t dio3_mask)
{
  irq_mask  = htobe16(irq_mask);
  dio1_mask = htobe16(dio1_mask);
  dio2_mask = htobe16(dio2_mask);
  dio3_mask = htobe16(dio3_mask);

  uint8_t params[SX126X_SETDIOIRQPARAMS_PARAMS];

  memcpy(params + SX126X_SETDIOIRQPARAMS_IRQMASK_PARAM, &irq_mask,
         SX126X_SETDIOIRQPARAMS_IRQMASK_PARAMS);

  memcpy(params + SX126X_SETDIOIRQPARAMS_DIO1MASK_PARAM, &dio1_mask,
         SX126X_SETDIOIRQPARAMS_DIO1MASK_PARAMS);

  memcpy(params + SX126X_SETDIOIRQPARAMS_DIO2MASK_PARAM, &dio2_mask,
         SX126X_SETDIOIRQPARAMS_DIO2MASK_PARAMS);

  memcpy(params + SX126X_SETDIOIRQPARAMS_DIO3MASK_PARAM, &dio3_mask,
         SX126X_SETDIOIRQPARAMS_DIO3MASK_PARAMS);

  sx126x_command(dev, SX126X_SETDIOIRQPARAMS, params,
                 SX126X_SETDIOIRQPARAMS_PARAMS, NULL);
}

static void sx126x_set_dio2_as_rf_switch(FAR struct sx126x_dev_s *dev,
                                         bool enable)
{
  sx126x_command(dev, SX126X_SETDIO2RFSWCTRL, (uint8_t *)&enable,
                 SX126X_SETDIO2RFSWCTRL_PARAMS, NULL);
}

static void sx126x_set_dio3_as_tcxo(FAR struct sx126x_dev_s *dev,
                                    enum sx126x_tcxo_voltage_e voltage,
                                    uint32_t delay)
{
  uint8_t params[SX126X_SETDIO3TCXOCTRL_PARAMS];

  params[SX126X_SETDIO3TCXOCTRL_TCXO_V_PARAM] = voltage;

  /* Convert delay to 24 bit and convert to BE */

  delay = htobe32(delay << 8);
  memcpy(params + SX126X_SETDIO3TCXOCTRL_DELAY_PARAM, &delay,
         SX126X_SETDIO3TCXOCTRL_DELAY_PARAMS);

  sx126x_command(dev, SX126X_SETDIO3TCXOCTRL, params,
                 SX126X_SETDIO3TCXOCTRL_PARAMS, NULL);
}

static void sx126x_get_irq_status(FAR struct sx126x_dev_s *dev,
                                  FAR uint16_t *irqstatus)
{
  uint8_t returns[SX126X_GETIRQSTATUS_RETURNS];

  sx126x_command(dev, SX126X_GETIRQSTATUS,
                 NULL, SX126X_GETIRQSTATUS_RETURNS,
                 returns);

  uint16_t bits;

  memcpy(&bits, returns +
         SX126X_GETIRQSTATUS_IRQSTATUS_RETURN,
         SX126X_GETIRQSTATUS_IRQSTATUS_RETURNS);

  *irqstatus = be16toh(bits);
}

static void sx126x_clear_irq_status(FAR struct sx126x_dev_s *dev,
                                    uint16_t clearbits)
{
  uint8_t params[SX126X_CLEARIRQSTATUS_PARAMS];

  clearbits = htobe16(clearbits);
  memcpy(params + SX126X_CLEARIRQSTATUS_CLEAR_PARAM,
         &clearbits,
         SX126X_CLEARIRQSTATUS_CLEAR_PARAMS);

  sx126x_command(dev, SX126X_CLEARIRQSTATUS, params,
                 SX126X_CLEARIRQSTATUS_PARAMS,
                 NULL);
}

/* RF Modulation and Packet-Related Functions *******************************/

static void sx126x_set_packet_params_lora(FAR struct sx126x_dev_s *dev,
                                          FAR struct
                                          sx126x_packetparams_lora_s *
                                          pktparams)
{
  uint8_t params[SX126X_SETPACKETPARMS_PARAMS];

  memset(params, 0, SX126X_SETPACKETPARMS_PARAMS);

  uint16_t preambles = htobe16(pktparams->preambles);

  memcpy(params + SX126X_PKTPARAM1_LORA_PREAMBLELEN_PARAM, &preambles,
         SX126X_PKTPARAM1_LORA_PREAMBLELEN_PARAMS);

  params[SX126X_PKTPARAM3_LORA_HEADERTYPE_PARAM] =
    pktparams->fixed_length_header;
  params[SX126X_PKTPARAM4_LORA_PAYLOADLEN_PARAM] =
    pktparams->payload_length;
  params[SX126X_PKTPARAM5_LORA_CRCTYPE_PARAM]   = pktparams->crc_enable;
  params[SX126X_PKTPARAM6_LORA_INVERTIQ_PARAM]  = pktparams->invert_iq;

  sx126x_command(dev, SX126X_SETPACKETPARMS, params,
                 SX126X_SETPACKETPARMS_PARAMS, NULL);
}

static void sx126x_set_modulation_params_lora(FAR struct sx126x_dev_s *dev,
                                              FAR struct
                                              sx126x_modparams_lora_s *
                                              modparams)
{
  uint8_t params[SX126X_SETMODULATIONPARAMS_PARAMS];

  memset(params, 0, SX126X_SETMODULATIONPARAMS_PARAMS);

  params[SX126X_MODPARAM1_LORA_SF_PARAM] =
    modparams->spreading_factor;
  params[SX126X_MODPARAM2_LORA_BW_PARAM] =
    modparams->bandwidth;
  params[SX126X_MODPARAM3_LORA_CR_PARAM] =
    modparams->coding_rate;
  params[SX126X_MODPARAM4_LORA_LOWDATRATE_OPTI_PARAM] =
    modparams->low_datarate_optimization;

  sx126x_command(dev, SX126X_SETMODULATIONPARAMS, params,
                 SX126X_SETMODULATIONPARAMS_PARAMS, NULL);
}

static void sx126x_set_buffer_base_address(FAR struct sx126x_dev_s *dev,
                                           uint8_t tx, uint8_t rx)
{
  uint8_t params[SX126X_SETBUFFERBASEADDRESS_PARAMS];

  memset(params, 0, SX126X_SETBUFFERBASEADDRESS_PARAMS);

  params[SX126X_SETBUFFERBASEADDRESS_TX_PARAM]  = tx;
  params[SX126X_SETBUFFERBASEADDRESS_RX_PARAM]  = rx;

  sx126x_command(dev, SX126X_SETBUFFERBASEADDRESS, params,
                 SX126X_SETBUFFERBASEADDRESS_PARAMS, NULL);
}

static void sx126x_set_tx_params(FAR struct sx126x_dev_s *dev, uint8_t power,
                                 enum sx126x_ramp_time_e ramp_time)
{
  uint8_t params[SX126X_SETTXPARMS_PARAMS];

  memset(params, 0, SX126X_SETTXPARMS_PARAMS);

  params[SX126X_SETTXPARMS_RAMPTIME_PARAM]  = ramp_time;
  params[SX126X_SETTXPARMS_POWER_PARAM]     = power;

  sx126x_command(dev, SX126X_SETTXPARMS, params, SX126X_SETTXPARMS_PARAMS,
                 NULL);
}

static void sx126x_set_packet_type(FAR struct sx126x_dev_s *dev,
                                   enum sx126x_packet_type_e type)
{
  sx126x_command(dev, SX126X_SETPACKETTYPE, (uint8_t *)&type,
                 SX126X_SETPACKETTYPE_PARAMS, NULL);
}

static void sx126x_set_rf_frequency(FAR struct sx126x_dev_s *dev,
                                    uint32_t frequency_hz)
{
  uint32_t corrected_freq =
    sx126x_convert_freq_in_hz_to_pll_step(frequency_hz);

  corrected_freq = htobe32(corrected_freq);

  sx126x_command(dev, SX126X_SETRFFREQUENCY, (uint8_t *)&corrected_freq,
                 SX126X_SETRFFREQUENCY_PARAMS, NULL);
}

static void sx126x_set_lora_symb_num_timout(FAR struct sx126x_dev_s *dev,
                                            uint8_t symbnum)
{
  sx126x_command(dev, SX126X_SETLORASYMBNUMTIMEOUT, &symbnum,
                 SX126X_SETLORASYMBNUMTIMEOUT_PARAMS, NULL);
}

/* Communication status information *****************************************/

static void sx126x_get_status(FAR struct sx126x_dev_s *dev,
                              FAR struct sx126x_status_s *status)
{
  /* NOP param to shift out result */

  uint8_t   parms[1] = {
    0x00
  };

  uint8_t   rets[1];

  /* Get, mask and shift result into readable mode numbers */

  sx126x_command(dev, SX126X_CMD_GETSTATUS, parms, sizeof(parms), rets);
  status->mode = (rets[0] & SX126X_STATUS_CHIPMODE_MASK) >>
                 SX126X_STATUS_CHIPMODE_SHIFT;
  status->cmd = (rets[0] & SX126X_STATUS_CMD_MASK) >>
                SX126X_STATUS_CMD_SHIFT;
}

static void sx126x_get_rssi_inst(FAR struct sx126x_dev_s *dev,
                                 FAR int32_t *dbm)
{
  uint8_t rets[SX126X_GETRSSIINST_RETURNS];

  sx126x_command(dev, SX126X_GETRSSIINST, NULL, SX126X_GETRSSIINST_RETURNS,
                 rets);

  /* Calculate dBm from returns */

  int32_t rssi = rets[SX126X_GETRSSIINST_RSSI_RETURN];

  (*dbm) = -rssi / 2.0;
}

static void sx126x_get_rx_buffer_status(FAR struct sx126x_dev_s *dev,
  uint8_t *status,
  uint8_t *payload_len,
  uint8_t *rx_buff_offset)
{
  uint8_t returns[SX126X_GETRXBUFFERSTATUS_RETURNS];

  sx126x_command(dev, SX126X_GETRXBUFFERSTATUS,
  NULL,
  SX126X_GETRXBUFFERSTATUS_RETURNS,
  returns);

  *status = returns[SX126X_GETRXBUFFERSTATUS_STATUS_RETURN];
  *payload_len = returns[SX126X_GETRXBUFFERSTATUS_PAYLOAD_LEN_RETURN];
  *rx_buff_offset = returns[SX126X_GETRXBUFFERSTATUS_RX_START_PTR_RETURN];
}

static void sx126x_get_packet_status_lora(FAR struct sx126x_dev_s *dev,
                                          FAR int16_t *rssi_dbm,
                                          FAR int32_t *snr)
{
  uint8_t returns[SX126X_GETPACKETSTATUS_RETURNS];

  sx126x_command(dev, SX126X_GETPACKETSTATUS, NULL,
                 SX126X_GETPACKETSTATUS_RETURNS, returns);

  *rssi_dbm = -(int16_t)returns[SX126X_GETPACKETSTATUS_LORA_RSSI_RETURN] / 2;
  *snr      = (int8_t)returns[SX126X_GETPACKETSTATUS_LORA_SNR_RETURN];
}

/* Lower hardware control ***************************************************/

static void sx126x_reset(FAR struct sx126x_dev_s *dev)
{
  dev->lower->reset();
}

/* SPI Communication ********************************************************/

static void sx126x_select(FAR struct sx126x_dev_s *dev)
{
  SPI_SELECT(dev->spi, SPIDEV_LPWAN(dev->lower->dev_number), true);
}

static void sx126x_deselect(FAR struct sx126x_dev_s *dev)
{
  SPI_SELECT(dev->spi, SPIDEV_LPWAN(dev->lower->dev_number), false);
}

static void sx126x_spi_lock(FAR struct sx126x_dev_s *dev)
{
  struct spi_dev_s *spi = dev->spi;

  SPI_LOCK(spi, true);
  SPI_SETBITS(spi, 8);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETFREQUENCY(spi, SX126X_SPI_SPEED);
}

static void sx126x_spi_unlock(FAR struct sx126x_dev_s *dev)
{
  SPI_LOCK(dev->spi, false);
}

static int sx126x_wait_busy(FAR struct sx126x_dev_s *dev)
{
  clock_t start;
  int i;

  if (dev->lower->busy == NULL)
    {
      return OK;
    }

  /* Most commands keep the chip busy for microseconds; a reset, a
   * calibration or the wake-up from sleep for a few milliseconds.  Spin
   * through that: sleeping costs at least a whole system tick.
   */

  for (i = 0; i < 1000; i++)
    {
      if (!dev->lower->busy())
        {
          return OK;
        }

      up_udelay(10);
    }

  start = clock_systime_ticks();
  while (dev->lower->busy())
    {
      if (clock_systime_ticks() - start >
          MSEC2TICK(SX126X_BUSY_TIMEOUT_MS))
        {
          wlerr("ERROR: SX126x %d stays busy\n", dev->lower->dev_number);
          return -ETIMEDOUT;
        }

      nxsched_usleep(1000);
    }

  return OK;
}

static void sx126x_command(FAR struct sx126x_dev_s *dev, uint8_t cmd,
                           const FAR uint8_t *params, size_t paramslen,
                           FAR uint8_t *returns)
{
  sx126x_wait_busy(dev);
  sx126x_select(dev);

  /* First send the command. This does not return anything.
   * "RFU" according the manual
   */

  SPI_SEND(dev->spi, cmd);

  /* Send all the params and record the returning bytes */

  for (size_t i = 0; i < paramslen; i++)
    {
      uint8_t param = SX126X_NOP;

      if (params != NULL)
        {
          param = params[i];
        }

      uint8_t ret = SPI_SEND(dev->spi, param);

      if (returns != NULL)
        {
          returns[i] = ret;
        }
    }

  sx126x_deselect(dev);
}

/* Registers and buffer *****************************************************/

static void sx126x_write_register(FAR struct sx126x_dev_s *dev,
                                  uint16_t address,
                                  uint8_t *data,
                                  size_t data_length)
{
  sx126x_wait_busy(dev);
  sx126x_select(dev);

  /* Send the opcode and address */

  SPI_SEND(dev->spi, SX126X_WRITEREGISTER);
  SPI_SEND(dev->spi, (uint8_t)(address >> 8));
  SPI_SEND(dev->spi, (uint8_t)address);

  /* Send data */

  for (size_t i = 0; i < data_length; i++)
    {
      SPI_SEND(dev->spi, data[i]);
    }

  sx126x_deselect(dev);
}

static void sx126x_read_register(FAR struct sx126x_dev_s *dev,
                                 uint16_t address,
                                 uint8_t *data,
                                 size_t data_length)
{
  sx126x_wait_busy(dev);
  sx126x_select(dev);

  /* Send the opcode and address, then a NOP while the status comes back */

  SPI_SEND(dev->spi, SX126X_READREGISTER);
  SPI_SEND(dev->spi, (uint8_t)(address >> 8));
  SPI_SEND(dev->spi, (uint8_t)address);
  SPI_SEND(dev->spi, SX126X_NOP);

  /* Read data */

  for (size_t i = 0; i < data_length; i++)
    {
      data[i] = SPI_SEND(dev->spi, SX126X_NOP);
    }

  sx126x_deselect(dev);
}

static void sx126x_update_register(FAR struct sx126x_dev_s *dev,
                                   uint16_t address, uint8_t mask,
                                   bool set)
{
  uint8_t value;

  sx126x_read_register(dev, address, &value, 1);
  value = set ? (value | mask) : (value & ~mask);
  sx126x_write_register(dev, address, &value, 1);
}

static void sx126x_write_buffer(FAR struct sx126x_dev_s *dev,
                                uint8_t offset,
                                FAR const uint8_t *payload,
                                uint8_t len)
{
  sx126x_wait_busy(dev);
  sx126x_select(dev);

  /* Command */

  SPI_SEND(dev->spi, SX126X_WRITEBUFFER);

  /* Offset */

  SPI_SEND(dev->spi, offset);

  /* Data */

  for (size_t i = 0; i < len; i++)
    {
      SPI_SEND(dev->spi, payload[i]);
    }

  sx126x_deselect(dev);
}

static void sx126x_read_buffer(FAR struct sx126x_dev_s *dev,
                               uint8_t offset,
                               FAR uint8_t *payload,
                               uint8_t len)
{
  sx126x_wait_busy(dev);
  sx126x_select(dev);

  /* Command */

  SPI_SEND(dev->spi, SX126X_READBUFFER);

  /* Offset */

  SPI_SEND(dev->spi, offset);

  /* NOP */

  SPI_SEND(dev->spi, SX126X_NOP);

  /* Data */

  for (size_t i = 0; i < len; i++)
    {
      payload[i] = SPI_SEND(dev->spi, SX126X_NOP);
    }

  sx126x_deselect(dev);
}

/* Register settings ********************************************************/

static void sx126x_set_syncword(FAR struct sx126x_dev_s *dev,
                                uint8_t *syncword,
                                uint8_t syncword_length)
{
  if (syncword_length > SX126X_REG_SYNCWORD_LEN)
    {
      syncword_length = SX126X_REG_SYNCWORD_LEN;
      wlerr("Syncword length was limited to the maximum 8 bytes");
    }

  sx126x_write_register(dev, SX126X_REG_SYNCWORD, syncword, syncword_length);
}

static void sx126x_set_lora_syncword(FAR struct sx126x_dev_s *dev,
                                     uint16_t syncword)
{
  uint8_t data[2];

  data[0] = syncword >> 8;
  data[1] = syncword & 0xff;
  sx126x_write_register(dev, SX126X_REG_LORA_SYNCWORD, data, sizeof(data));
}

/* Driver specific **********************************************************/

static int sx126x_init(FAR struct sx126x_dev_s *dev)
{
  int ret;

  sx126x_reset(dev);

  sx126x_spi_lock(dev);
  ret = sx126x_wait_busy(dev);
  dev->asleep = false;
  sx126x_spi_unlock(dev);

  sx126x_set_defaults(dev);
  return ret;
}

static int sx126x_deinit(FAR struct sx126x_dev_s *dev)
{
  /* Nothing to do until the next open: sleep.  Listening, the chip is
   * woken from a duty cycle's sleep and stopped first, its interrupts
   * cleared; a worker still queued for one leaves the chip asleep.
   */

  sx126x_spi_lock(dev);
  sx126x_wake(dev);
  sx126x_set_standby(dev, SX126X_STDBY_RC);
  sx126x_clear_irq_status(dev, 0xffff);
  dev->listen.mode = SX126X_LISTEN_OFF;
  dev->receiving   = false;
  sx126x_set_sleep(dev);
  dev->asleep = true;
  sx126x_spi_unlock(dev);
  return 0;
}

static void sx126x_set_defaults(FAR struct sx126x_dev_s *dev)
{
  /* Hardware defaults */

  dev->invert_iq = SX126X_DEFAULT_INVERT_IQ;

  /* Common defaults */

  dev->packet_type    = SX126X_DEFAULT_PACKET_TYPE;
  dev->frequency_hz   = SX126X_DEFAULT_FREQ;
  dev->power          = SX126X_DEFAULT_POWER;
  dev->preambles      = SX126X_DEFAULT_LORA_PREAMBLES;

  /* LoRa defaults */

  dev->lora_sf                      = SX126X_DEFAULT_LORA_SF;
  dev->lora_bw                      = SX126X_DEFAULT_LORA_BW;
  dev->lora_cr                      = SX126X_DEFAULT_LORA_CR;
  dev->lora_fixed_header            = SX126X_DEFAULT_LORA_FIXED_HEADER;
  dev->lora_crc                     = SX126X_DEFAULT_LORA_CRC_EN;
  dev->low_datarate_optimization    = SX126X_DEFAULT_LORA_LDO;

  uint8_t newsyncword[] = SX126X_DEFAULT_SYNCWORD;

  memcpy(dev->syncword, newsyncword, sizeof(dev->syncword));
  dev->lora_syncword                = SX126X_DEFAULT_LORA_SYNCWORD;
  dev->rx_timeout_ms                = 0;
  dev->rx_payload_len               = SX126X_RX_PAYLOAD_SIZE;
  dev->listen.mode                  = SX126X_LISTEN_OFF;

  /* GFSK defaults */
}

static int sx126x_setup_radio(FAR struct sx126x_dev_s *dev)
{
  enum sx126x_device_e model;
  uint8_t hp;
  uint8_t dc;

  if (dev->lower->check_frequency(dev->frequency_hz) != 0)
    {
      wlerr("Board does not support %" PRIu32 "Hz\n", dev->frequency_hz);
      return -EINVAL;
    }

  /* All of this is set in STDBY_RC.  The chip may have been reset or
   * asleep, so everything is set every time.
   */

  sx126x_set_standby(dev, SX126X_STDBY_RC);

  /* A TCXO has to be enabled before anything runs from the crystal, and
   * the calibration at power-up ran without it: calibrate again, then
   * calibrate the image rejection for the band in use.
   */

  if (dev->lower->dio3_delay > 0)
    {
      sx126x_set_dio3_as_tcxo(dev, dev->lower->dio3_voltage,
                              dev->lower->dio3_delay);
    }

  sx126x_calibrate(dev, SX126X_CALIBRATE_ALL);
  sx126x_calibrate_image(dev, dev->frequency_hz);

  /* Set regulator */

  sx126x_set_regulator_mode(dev, dev->lower->regulator_mode);

  /* DIO 2 */

  sx126x_set_dio2_as_rf_switch(dev, dev->lower->use_dio2_as_rf_sw);

  /* Set packet type */

  sx126x_set_packet_type(dev, dev->packet_type);

  /* Set RF frequency */

  sx126x_set_rf_frequency(dev, dev->frequency_hz);

  /* Set PA settings from lower, and let the PA survive a mismatched
   * antenna (datasheet 15.2)
   */

  dev->lower->get_pa_values(&model, &hp, &dc);
  sx126x_set_pa_config(dev, model, hp, dc);
  sx126x_update_register(dev, SX126X_REG_TX_CLAMP, SX126X_TX_CLAMP_CONFIG,
                         true);

  /* Set TX params */

  dev->lower->limit_tx_power(&dev->power); /* Limited by board */
  sx126x_set_tx_params(dev, dev->power, dev->lower->tx_ramp_time);

  /* Set base */

  sx126x_set_buffer_base_address(dev, 0, 0);

  /* Set params depending on packet type */

  switch (dev->packet_type)
    {
      case (SX126X_PACKETTYPE_LORA):
        {
          /* Mod params */

          struct sx126x_modparams_lora_s modparams = {
            .spreading_factor           = dev->lora_sf,
            .bandwidth                  = dev->lora_bw,
            .coding_rate                = dev->lora_cr,
            .low_datarate_optimization  = dev->low_datarate_optimization
          };

          sx126x_set_modulation_params_lora(dev, &modparams);

          /* Modulation quality at 500 kHz (datasheet 15.1) */

          sx126x_update_register(dev, SX126X_REG_TX_MODULATION,
                                 SX126X_TX_MODULATION_BW500,
                                 dev->lora_bw != SX126X_LORA_BW_500);

          /* Packet params */

          struct sx126x_packetparams_lora_s pktparams = {
            .crc_enable          = dev->lora_crc,
            .fixed_length_header = dev->lora_fixed_header,
            .payload_length      = dev->payload_len,
            .invert_iq           = dev->invert_iq,
            .preambles           = dev->preambles
          };

          sx126x_set_packet_params_lora(dev, &pktparams);

          /* Inverted IQ needs the IQ polarity bit fixed (datasheet 15.4) */

          sx126x_update_register(dev, SX126X_REG_IQ_POLARITY,
                                 SX126X_IQ_POLARITY_STANDARD,
                                 !dev->invert_iq);

          sx126x_set_lora_syncword(dev, dev->lora_syncword);
          break;
        }

      default:

        /* Sync word */

        sx126x_set_syncword(dev, dev->syncword, sizeof(dev->syncword));
        break;
    }

  /* IRQ MASK */

  sx126x_set_dio_irq_params(dev, dev->irq_mask,
    dev->lower->masks.dio1_mask,
    dev->lower->masks.dio2_mask,
    dev->lower->masks.dio3_mask);

  sx126x_clear_irq_status(dev, 0xffff);
  return 0;
}

/* Listening ****************************************************************/

/****************************************************************************
 * Name: sx126x_wake
 *
 * Description:
 *   Wake the chip, if a duty cycle has it asleep: NSS going low wakes it
 *   (datasheet 13.1.1), and it takes commands once BUSY is low again.
 *   Harmless while it is awake.  Called with the SPI locked.
 *
 ****************************************************************************/

static void sx126x_wake(FAR struct sx126x_dev_s *dev)
{
  if (dev->listen.mode == SX126X_LISTEN_DUTY_CYCLE)
    {
      sx126x_select(dev);
      sx126x_deselect(dev);
    }
}

/****************************************************************************
 * Name: sx126x_listen_arm
 *
 * Description:
 *   Set the chip up, waking it first, and have it listen as dev->listen
 *   says.  Called with the SPI locked.
 *
 ****************************************************************************/

static int sx126x_listen_arm(FAR struct sx126x_dev_s *dev)
{
  int ret;

  dev->payload_len = dev->lora_fixed_header ? dev->rx_payload_len :
                                              SX126X_RX_PAYLOAD_SIZE;
  dev->irq_mask    = SX126X_IRQ_RXDONE_MASK | SX126X_IRQ_CRCERR_MASK |
                     SX126X_IRQ_HEADERERR_MASK |
                     SX126X_IRQ_HEADERVALID_MASK | SX126X_IRQ_TIMEOUT_MASK;
  dev->receiving   = false;

  sx126x_wake(dev);
  ret = sx126x_setup_radio(dev);
  if (ret < 0)
    {
      return ret;
    }

  if (dev->listen.mode == SX126X_LISTEN_DUTY_CYCLE)
    {
      sx126x_set_rx_duty_cycle(dev, SX126X_DUTY_STEPS(dev->listen.rx_us),
                               SX126X_DUTY_STEPS(dev->listen.sleep_us));
    }
  else
    {
      sx126x_set_rx(dev, SX126X_SETRX_CONTINUOUS);
    }

  return OK;
}

/****************************************************************************
 * Name: sx126x_listen_take
 *
 * Description:
 *   Read the packet the chip received into the queue, dropping the oldest
 *   kept if it is full, and tell readers.  Called with the SPI locked.
 *
 ****************************************************************************/

static void sx126x_listen_take(FAR struct sx126x_dev_s *dev,
                               uint16_t irqbits)
{
  FAR struct sx126x_read_header_s *pkt;
  uint8_t status;
  uint8_t offset;
  int value;

  nxmutex_lock(&dev->qlock);

  if (dev->qcount == CONFIG_LPWAN_SX126X_RXQUEUE)
    {
      wlwarn("WARNING: SX126x packets not read: the oldest dropped\n");
      dev->qhead = (dev->qhead + 1) % CONFIG_LPWAN_SX126X_RXQUEUE;
      dev->qcount--;
    }

  pkt = &dev->queue[(dev->qhead + dev->qcount) %
                    CONFIG_LPWAN_SX126X_RXQUEUE];
  sx126x_get_rx_buffer_status(dev, &status, &pkt->payload_length, &offset);
  sx126x_read_buffer(dev, offset, pkt->payload, pkt->payload_length);
  sx126x_get_packet_status_lora(dev, &pkt->rssi_db, &pkt->snr);
  pkt->crc_error = (irqbits & SX126X_IRQ_CRCERR_MASK) != 0;
  dev->qcount++;

  poll_notify(dev->fds, CONFIG_LPWAN_SX126X_NPOLLWAITERS, POLLIN);
  nxmutex_unlock(&dev->qlock);

  /* One post wakes a waiting reader; more would only pile up */

  if (nxsem_get_value(&dev->rx_sem, &value) >= 0 && value <= 0)
    {
      nxsem_post(&dev->rx_sem);
    }
}

/****************************************************************************
 * Name: sx126x_listen_pause
 *
 * Description:
 *   Before a send while listening: let a packet being heard end, then
 *   drain the chip.  Called with dev->lock held.
 *
 ****************************************************************************/

static void sx126x_listen_pause(FAR struct sx126x_dev_s *dev)
{
  while (nxsem_trywait(&dev->rxend_sem) == OK);

  if (dev->receiving)
    {
      nxsem_tickwait_uninterruptible(&dev->rxend_sem,
                                     MSEC2TICK(SX126X_RX_END_MS));
    }

  sx126x_spi_lock(dev);
  sx126x_listen_drain(dev);
  sx126x_spi_unlock(dev);
}

/****************************************************************************
 * Name: sx126x_listen_drain
 *
 * Description:
 *   Stop receiving, and keep a packet the chip holds, which setting it up
 *   again or a send would lose.  Called with the SPI locked.
 *
 ****************************************************************************/

static void sx126x_listen_drain(FAR struct sx126x_dev_s *dev)
{
  uint16_t irqbits;

  sx126x_wake(dev);
  sx126x_set_standby(dev, SX126X_STDBY_RC);
  sx126x_get_irq_status(dev, &irqbits);
  if (irqbits & SX126X_IRQ_RXDONE_MASK)
    {
      sx126x_listen_take(dev, irqbits);
    }

  sx126x_clear_irq_status(dev, irqbits);
  dev->receiving = false;
}

/****************************************************************************
 * Name: sx126x_listen_flush
 *
 * Description:
 *   Drop the packets kept, once listening stopped, and wake a reader
 *   waiting for one, or polling, to find it stopped.
 *
 ****************************************************************************/

static void sx126x_listen_flush(FAR struct sx126x_dev_s *dev)
{
  nxmutex_lock(&dev->qlock);
  dev->qhead  = 0;
  dev->qcount = 0;
  poll_notify(dev->fds, CONFIG_LPWAN_SX126X_NPOLLWAITERS, POLLIN);
  nxmutex_unlock(&dev->qlock);

  nxsem_post(&dev->rx_sem);
}

/****************************************************************************
 * Name: sx126x_read_queued
 *
 * Description:
 *   read() while listening: the oldest packet kept, or wait for one as
 *   the receive timeout allows; -EAGAIN at once with O_NONBLOCK, and
 *   -ECANCELED if listening stops meanwhile.
 *
 ****************************************************************************/

static ssize_t sx126x_read_queued(FAR struct sx126x_dev_s *dev,
                                  FAR struct file *filep,
                                  FAR struct sx126x_read_header_s *header)
{
  int ret;

  for (; ; )
    {
      ret = nxmutex_lock(&dev->qlock);
      if (ret < 0)
        {
          return ret;
        }

      if (dev->qcount > 0)
        {
          memcpy(header, &dev->queue[dev->qhead], sizeof(*header));
          dev->qhead = (dev->qhead + 1) % CONFIG_LPWAN_SX126X_RXQUEUE;
          dev->qcount--;
          nxmutex_unlock(&dev->qlock);
          return sizeof(*header);
        }

      nxmutex_unlock(&dev->qlock);

      if (dev->listen.mode == SX126X_LISTEN_OFF)
        {
          return -ECANCELED;
        }

      if (filep->f_oflags & O_NONBLOCK)
        {
          return -EAGAIN;
        }

      if (dev->rx_timeout_ms > 0)
        {
          ret = nxsem_tickwait(&dev->rx_sem,
                               SEC2TICK(dev->rx_timeout_ms / 1000) +
                               MSEC2TICK(dev->rx_timeout_ms % 1000));
        }
      else
        {
          ret = nxsem_wait(&dev->rx_sem);
        }

      if (ret < 0)
        {
          return ret;
        }
    }
}

/* Interrupt handling *******************************************************/

static int sx126x_irq0handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct sx126x_dev_s *dev = (FAR struct sx126x_dev_s *)arg;

  DEBUGASSERT(dev != NULL);

  /* A level triggered line stays active until the worker has cleared the
   * chip's IRQ status
   */

  if (dev->lower->irq0enable != NULL)
    {
      dev->lower->irq0enable(false);
    }

  work_queue(HPWORK, &dev->irq0_work, sx126x_isr0_process, arg, 0);
  return OK;
}

static inline int sx126x_attachirq0(FAR struct sx126x_dev_s *dev, xcpt_t isr,
  FAR void *arg)
{
  DEBUGASSERT(dev->lower->irq0attach != NULL);

  return dev->lower->irq0attach(isr, arg);
}

static void sx126x_isr0_process(FAR void *arg)
{
  DEBUGASSERT(arg);

  FAR struct sx126x_dev_s *dev = (FAR struct sx126x_dev_s *)arg;
  uint16_t irqbits;

  wlinfo("SX126x ISR0 process triggered\n");

  /* Get and clear IRQ bits.  Listening, a packet received is read at
   * once, before a send can overwrite the chip's buffer; a duty cycle,
   * which ends with a packet, starts again.
   */

  sx126x_spi_lock(dev);

  /* Closed meanwhile: the chip sleeps, and stays asleep */

  if (dev->asleep)
    {
      sx126x_spi_unlock(dev);
      if (dev->lower->irq0enable != NULL)
        {
          dev->lower->irq0enable(true);
        }

      return;
    }

  sx126x_wake(dev);
  sx126x_get_irq_status(dev, &irqbits);
  sx126x_clear_irq_status(dev, irqbits);

  if (dev->listen.mode != SX126X_LISTEN_OFF)
    {
      if (irqbits & SX126X_IRQ_RXDONE_MASK)
        {
          sx126x_listen_take(dev, irqbits);
        }

      /* A packet heard begins, or ends (a duty cycle's listen, stretched
       * by a preamble, may end in a timeout): a send waits for its end
       */

      if (irqbits & SX126X_IRQ_HEADERVALID_MASK)
        {
          dev->receiving = true;
        }

      if ((irqbits & (SX126X_IRQ_RXDONE_MASK | SX126X_IRQ_HEADERERR_MASK |
                      SX126X_IRQ_CRCERR_MASK |
                      SX126X_IRQ_TIMEOUT_MASK)) != 0)
        {
          if (dev->receiving)
            {
              dev->receiving = false;
              nxsem_post(&dev->rxend_sem);
            }

          if (dev->listen.mode == SX126X_LISTEN_DUTY_CYCLE)
            {
              sx126x_set_rx_duty_cycle(dev,
                                SX126X_DUTY_STEPS(dev->listen.rx_us),
                                SX126X_DUTY_STEPS(dev->listen.sleep_us));
            }
        }
    }

  sx126x_spi_unlock(dev);

  if (dev->lower->irq0enable != NULL)
    {
      dev->lower->irq0enable(true);
    }

  wlinfo("IRQ status 0x%X\n", irqbits);

  /* On TX done */

  if (irqbits & SX126X_IRQ_TXDONE_MASK)
    {
      wlinfo("TX done\n");

      /* Release writing threads */

      nxsem_post(&dev->tx_sem);
    }

  /* On RX done.  A header error alone is not a packet: receive on.
   * Listening, the packet is queued already.
   */

  if ((irqbits & SX126X_IRQ_RXDONE_MASK) &&
      dev->listen.mode == SX126X_LISTEN_OFF)
    {
      wlinfo("RX done\n");

      dev->irqbits = irqbits;
      nxsem_post(&dev->rx_sem);
    }

  /* On CAD done */

  if (irqbits & SX126X_IRQ_CADDONE_MASK)
    {
      wlinfo("CAD done\n");
    }

  /* On CAD detect */

  if (irqbits & SX126X_IRQ_CADDETECTED_MASK)
    {
      wlinfo("CAD detect\n");
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void sx126x_register(FAR struct spi_dev_s *spi,
                     FAR const struct sx126x_lower_s *lower,
                     const char *path)
{
  /* Register the dev using an unique dev_number,
   * so multiple radios can be registered at once
   */

  if (lower->dev_number >= SX126X_MAX_DEVICES)
    {
      wlerr("SX126x dev_number %d is greater than \
            allowed amount of SX126x devices",
            lower->dev_number);
      return;
    }

  struct sx126x_dev_s *dev;

  dev = &g_sx126x_devices[lower->dev_number];
  dev->lower   = lower;
  dev->spi     = spi;

  /* Initialize locks and semaphores */

  nxmutex_init(&dev->lock);
  nxmutex_init(&dev->qlock);
  nxsem_init(&dev->rx_sem, 0, 0);
  nxsem_init(&dev->tx_sem, 0, 0);
  nxsem_init(&dev->rxend_sem, 0, 0);
  sx126x_set_defaults(dev);

  /* Asleep until the first open, as the board leaves it */

  dev->asleep = true;
  sx126x_attachirq0(dev, sx126x_irq0handler, dev);

  (void)register_driver(path, &sx126x_ops, 0600,
                        dev);
}
