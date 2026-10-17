.. _lilygo-tdeck-max:

=================
LilyGo T-Deck Max
=================

.. tags:: chip:esp32, chip:esp32s3, arch:xtensa, vendor:lilygo

The `LilyGo T-Deck Max <https://github.com/Xinyuan-LilyGO/T-Deck-MAX>`_ is a
handheld built around an ESP32-S3 (dual Xtensa LX7 at 240 MHz, 16 MB quad
flash, 8 MB quad PSRAM): a 3.1" e-paper screen, a thumb keyboard, touch, an
LTE modem, a LoRa radio, GNSS and audio.  Nearly every part sits on a power
rail switched through an XL9555 I/O expander.

Features
========

======================= ========================================= ==========
Part                    Job                                       Connected
======================= ========================================= ==========
ESP32-S3 (QFN56)        processor, Wi-Fi 4, Bluetooth LE 5        —
16 MB flash, 8 MB PSRAM firmware and storage; working memory      quad SPI
GDEQ031T10 (UC8253)     e-paper, 240 × 320, 1 bit; front light    SPI
CST3530                 touch panel and three touch keys          I2C
TCA8418                 keyboard, 4 × 10 matrix; backlight        I2C
A7682E                  LTE Cat 1 modem                           UART2
SX1262                  LoRa radio, internal or external antenna  SPI
MIA-M10Q                GNSS receiver                             UART1
ES8311                  audio codec; speaker amplifier            I2S, I2C
DRV2605L                vibration motor driver                    I2C
BHI260AP                IMU                                       I2C
SY6970, BQ27220         charger, fuel gauge                       I2C
XL9555                  I/O expander: the power rails             I2C
microSD                 removable storage                         SPI
USB-C                   power; USB Serial/JTAG                    USB
======================= ========================================= ==========

Support
=======

The port is being written part by part.  Supported so far:

* NSH on the USB Serial/JTAG console;
* the 8 MB PSRAM, added to the heap;
* ``reboot``;
* the I2C bus, and the XL9555's lines as GPIO devices (``full``);
* the SPI bus, the e-paper as ``/dev/fb0``, and both lights as
  ``/dev/pwm0`` (``full``);
* the keyboard as ``/dev/kbd0`` (``full``);
* the touch screen as ``/dev/input0``, and the keys on the glass below it
  as ``/dev/softkeys`` (``full``);
* the microSD card as ``/dev/mmcsd0``, with FAT (``full``);
* the fuel gauge as ``/dev/batt0``, the charger as ``/dev/charger0``, and
  ``poweroff`` (``full``);
* the LoRa radio as ``/dev/lora0``, and the ``lora`` command (``full``);
* the GNSS receiver through NuttX's GNSS upper half, and the ``gps``
  command (``full``);
* the modem on UART2, and the ``modem`` command (``full``);
* the audio codec as ``/dev/audio/pcm0`` and ``/dev/audio/pcm_in0``, with
  ``nxplayer`` and ``nxrecorder`` (``full``);
* the vibration motor as ``/dev/input_ff0``, and the ``haptic`` command
  (``full``);
* the IMU's accelerometer and gyroscope as uORB topics, with the
  ``sensortest`` command (``full``);
* 2.4 GHz Wi-Fi and Bluetooth LE, off until asked for, with the ``wapi``,
  ``ping`` and ``bt`` commands (``full``);
* power management: light sleep until the next timer, wake-up pins, and
  ``/dev/awake`` (``full``).

Serial Console
==============

NSH runs on the ESP32-S3's **USB Serial/JTAG** unit, which enumerates on
the computer as ``/dev/ttyACM0`` (Linux, USB ID ``303a:1001``); no
USB-to-UART bridge is involved.  The two UARTs are left for the GNSS
receiver (UART1) and the modem (UART2).

The ESP32-S3's low-level console output (``up_putc()``) does not reach the
USB Serial/JTAG unit, so the system log does not appear on the console.
``full`` keeps it in a RAM log instead, read with ``dmesg``.

Buttons
=======

``RST`` resets the chip.  ``BOOT`` pulls GPIO0 low; to enter the ROM's
download mode by hand, hold ``BOOT``, press and release ``RST``, then release
``BOOT``.  ``esptool`` normally does this by itself over USB Serial/JTAG.

Building and Flashing
=====================

Use Espressif's ``xtensa-esp-elf`` toolchain **14.2**, whose
``xtensa-esp32s3-elf-*`` commands must be on the ``PATH``: GCC 15 defaults
to C23, which removed ``ATOMIC_VAR_INIT``, and the Espressif HAL does not
build with it.  ``esptool`` writes the image (see :doc:`the ESP32-S3 page
</platforms/xtensa/esp32s3/index>`)::

    $ ./tools/configure.sh lilygo-tdeck-max:nsh
    $ make -j$(nproc)
    $ make flash ESPTOOL_PORT=/dev/ttyACM0 ESPTOOL_BINDIR=./

The image is written from address 0 and boots without a second-stage
bootloader; ``nsh`` uses no other part of the flash.  At every start the
ROM prints ``SHA-256 comparison failed`` and ``Attempting to boot anyway``:
such an image carries no digest, and the boot goes on.

.. warning::

   Flashing replaces the factory firmware.  To keep it, read the whole
   flash first::

       $ esptool -p /dev/ttyACM0 read-flash 0 0x1000000 factory-16MB.bin
       $ esptool -p /dev/ttyACM0 write-flash 0 factory-16MB.bin   # to restore

Pin map
=======

``include/board.h`` names every pin.  Directions are from the ESP32-S3's
side.

======================== =============================================
Function                 GPIO
======================== =============================================
I2C SDA / SCL            13 / 14
SPI SCK / MOSI / MISO    36 / 33 / 47
SPI chip selects         e-paper 34, microSD 48, SX1262 3
e-paper                  DC 35, BUSY 37, RST 9, front light 41
SX1262                   NRESET 4, DIO1 5, BUSY 6
GNSS (UART1)             TX 16, RX 2, PPS 1
Modem (UART2)            TX 10, RX 11, RI 7, DTR 8
Interrupts               touch 12, keyboard 15, IMU 21
Keyboard backlight       42
ES8311 (I2S)             MCLK 38, BCLK 39, WS 18, DOUT 40, DIN 17
BOOT button              0
======================== =============================================

The e-paper, the microSD card and the SX1262 share one SPI bus.  Early in
boot (``esp32s3_board_initialize()``) every chip select is set high before
anything on the bus is touched: a chip select held low on the unpowered
SX1262 loads the bus, and the e-paper stops answering.  The e-paper's reset
is released and the SX1262's held, and both lights are switched off.

The SX1262's power rail is on the XL9555 (below).  ``nsh`` does not drive
the expander, so the rail is not switched on, and the radio's high chip
select feeds it about 15 mA.  ``full`` switches the rail on and puts the
radio to sleep (``SetSleep``, cold start: about 160 nA) until a driver
wants it.

The board drives the chip selects itself (``ESP32S3_SPI_UDCS``):
``SPIDEV_DISPLAY(0)`` is the e-paper, ``SPIDEV_MMCSD(0)`` the microSD card
and ``SPIDEV_LPWAN(0)`` the SX1262.  The e-paper's data/command line is
driven through ``SPI_CMDDATA``.

Power rails
===========

Nearly every part is powered through the XL9555 I/O expander (I2C,
address ``0x20``), driven with NuttX's PCA9555 driver.  In ``full`` each
line the board uses is a GPIO device, ``/dev/<name>``, set at every start;
use
``gpio -o 0|1 /dev/<name>``:

================= ======= =================================================
Device            XL9555  Line (level at start)
================= ======= =================================================
``modem_pwr``     P00     the A7682E's supply (off)
``lora_en``       P01     the SX1262's supply (on)
``gps_en``        P02     the MIA-M10Q's supply (off)
``imu_en``        P03     the BHI260AP's 1.8 V supply (on)
``lora_ant``      P04     high: internal antenna, low: external (internal)
``motor_en``      P05     the DRV2605L's supply (on)
``amp_en``        P06     the speaker amplifier (off)
``touch_rst``     P07     the CST3530's reset, active low (released)
``modem_pwrkey``  P10     high presses the modem's power key (released)
``key_rst``       P11     the TCA8418's reset, active low (released)
``audio_sel``     P12     high: the modem's audio, low: the codec's (codec)
================= ======= =================================================

* **The XL9555 keeps its outputs across a reset of the ESP32-S3.**  The
  modem's supply is therefore left on if it is already driven on at start,
  since cutting it while the modem runs can damage the modem's flash; the
  log says so.  At power-up the lines are inputs, pulled up: the modem's
  power key is set first, so that it is not held pressed.
* **The SX1262's reset follows its supply:** held while the supply is
  off, so that it does not feed the unpowered chip, and released once it is
  on.  With the supply on, the radio idles in standby.
* The IMU's and the motor driver's supplies are on, and the touch and
  keyboard controllers out of reset, so that every part on the I2C bus
  answers (``i2c dev 0x08 0x77``).  The BQ27220 fuel gauge (``0x55``) does
  not yet: it stretches the clock longer than the I2C driver waits.

Screen
======

The UC8253 controller is driven by ``drivers/lcd/uc8253.c`` and seen as
``/dev/fb0`` through NuttX's LCD framebuffer driver: 240 × 320, 1 bit
(``FB_FMT_Y1``), a set bit white.

* Drawing goes into the framebuffer; the panel is refreshed on
  ``FBIO_UPDATE``, from a thread of the driver's own, after a short wait
  for the rest of a burst of drawing.
* Only the area that changed is refreshed when that is part of the
  screen, with the partial waveform (about 0.7 s); the whole screen with
  the full one (about 1.0 s, with ``LCD_UC8253_FASTUPDATE``; measured
  2026-09-21).
* A partial refresh leaves a little ghosting behind.  The
  ``UC8253_IOC_FULLREFRESH`` ioctl on ``/dev/fb0`` makes the next refresh
  a full one, which clears it; when to ask for it is the caller's choice.
* Powering the panel down (``FBIOSET_POWER``) puts the controller in deep
  sleep; the glass keeps the image.

Keyboard
========

The keyboard is a BlackBerry Q20's: a 4 × 10 matrix scanned by a TCA8418
(I2C ``0x34``, interrupt on GPIO 15, reset on the XL9555's ``key_rst``),
driven by ``drivers/input/tca8418.c`` and seen as ``/dev/kbd0`` through
NuttX's keyboard upper half.

* Characters arrive as they are printed on the keys: Shift for capitals,
  Alt (left of Z) for the digits and symbols.  A modifier held applies to
  the keys pressed meanwhile; tapped, to the next key; tapped twice, until
  it is tapped again.
* Enter, Backspace and Sym (right of Space) are special keys:
  ``KEYCODE_ENTER``, ``KEYCODE_BACKDEL`` and ``KEYCODE_FIND``.
* Shift and Alt are reported as ``KEYCODE_LSHIFT`` and ``KEYCODE_LALT``,
  pressed when they take effect and released when they end, so that a
  reader can tell what a key was pressed with.
* The matrix's columns are wired in reverse, and the key the vendor calls
  "UP" (the arrow keycaps at both ends of the bottom row) is Shift.
* The keyboard needs the XL9555 (``IOEXPANDER_PCA9555``), which releases
  its reset; with ``INPUT_TCA8418`` the board selects the GPIO interrupts
  it needs (``ESPRESSIF_GPIO_IRQ``).

``kbd`` prints the events::

    nsh> kbd /dev/kbd0 20

Touch screen
============

The e-paper is covered by a CST3530 capacitive touch controller (I2C
``0x1a``, interrupt on GPIO 12, reset on the XL9555's ``touch_rst``),
driven by ``drivers/input/cst3530.c`` and seen as ``/dev/input0`` through
NuttX's touchscreen upper half.

* Touches arrive in the panel's pixels, 240 × 320 in portrait, with no
  swapping or mirroring: (0, 0) is the top left corner.
* A sample holds the touches reported, up to five, each with its id.  A
  touch no longer reported is released in a sample of its own, before the
  touches that remain.  The controller reports every 10 to 20 ms while a
  finger is on the glass, and the pressure of a finger held still wavers:
  a touch that has not moved is reported again only once its pressure has
  changed by ``INPUT_CST3530_PRESSURE_STEP`` (4) since it was last
  reported.
* The three keys on the glass below the screen are the device's soft keys:
  ``/dev/softkeys``, a keyboard device, reports them as the special keys
  ``KEYCODE_F1``, ``KEYCODE_F2`` and ``KEYCODE_F3``, from the left.
* The controller sleeps while neither device is open; the first open
  resets it into normal operation, which takes about 60 ms.
* The touch screen needs the XL9555 (``IOEXPANDER_PCA9555``), which drives
  its reset; with ``INPUT_CST3530`` the board selects the GPIO interrupts
  it needs (``ESPRESSIF_GPIO_IRQ``).

``tc`` prints the samples, and ``kbd`` the soft keys::

    nsh> tc 20
    nsh> kbd /dev/softkeys 6

Lights
======

Both lights are LEDC PWM channels on ``/dev/pwm0`` (timer 0): the first
is the front light (GPIO 41), the second the keyboard's backlight
(GPIO 42).  For example, both at half for five seconds::

    nsh> pwm -f 1000 -c 1 -d 50 -c 2 -d 50 -t 5

* The LEDC driver applies the duties in the order they are given, and
  ignores the channel numbers: ``-c 2 -d 50 -c 1 -d 0`` lights the front
  light.
* ``ESPRESSIF_LEDC_TIMER0_CHANNELS`` must stay 2: the further channels
  default to GPIO 4 to 9, which the board uses for the SX1262, the modem
  and the e-paper.

microSD card
============

The microSD slot is on the shared SPI bus (chip select GPIO 48) and is
always powered.  ``full`` registers the card as ``/dev/mmcsd0`` (NuttX's
SPI MMC/SD driver, through the common ``board_sdmmc_spi_initialize()``)
and has FAT, with long and lower-case names; mounting it is left to the
system::

    nsh> mount -t vfat /dev/mmcsd0 /mnt/sd
    nsh> umount /mnt/sd

* The slot has no card-detect line, so the board reports a card always
  present.  A card put into a slot that was empty at the start is
  identified when ``/dev/mmcsd0`` is next opened; once a card has been
  identified, a card taken out and put back, or swapped, is not identified
  again until a restart, even unmounted, and its reads fail.
* The card runs at 20 MHz (``MMCSD_SPICLOCK``), after 400 kHz while it is
  identified.  Reading 1 MB with ``dd`` takes about 0.76 s (about 1.3 MB/s);
  the screen and the card take turns on the bus.
* ``df -h`` (``/proc/fs/usage``) wraps sizes at 4 GB without
  ``FS_LARGEFILE``: it shows an 8 GB card as 3286M.  The same limit keeps
  files, and offsets into ``/dev/mmcsd0``, under 2 GB.
* FAT only: cards of 64 GB and more, sold with exFAT, need formatting
  with FAT32 first.

Battery
=======

The cell is a 1400 mAh Li-Po (``BOARD_BATTERY_MAH``).  A BQ27220 fuel gauge
(I2C address 0x55) measures it and an SY6970 (0x6A) charges it from USB;
``full`` registers them as ``/dev/batt0`` and ``/dev/charger0``.  Both chips
are powered by the cell, not the board: they keep their settings across the
board's resets, but not across the cell being disconnected, so the board
applies its settings at every start.

* The gauge's values, read with the ``BATIOC_*`` ioctls, are plain
  integers: millivolts, percent, milliamps (positive while charging) and
  tenths of a degree Celsius.  The ``batterydump`` command decodes them as
  fixed point, so it prints wrong values for this gauge.
* The gauge stretches the I2C clock for longer than the ESP32-S3's
  default timeout allows, and its reads fail without
  ``ESP32S3_I2C_SCL_TIMEOUT_US=20000``, which ``full`` sets.
* At start the board checks the gauge's design capacity against
  ``BOARD_BATTERY_MAH`` and writes it, with the initial full charge
  capacity, only when they differ, which normally means the cell was
  disconnected.  The write unseals the gauge with TI's default keys and
  takes about two seconds.
* The charger charges to 4288 mV at 1024 mA (``BOARD_CHARGE_MV``,
  ``BOARD_CHARGE_MA``), the values the vendor's firmware sets.  4288 mV is
  above the usual 4.20 V, and the cell's rating is not published.  The
  driver turns off the charger's I2C watchdog: when it expires, the charger
  puts its settings back to their defaults (4208 mV).
* ``poweroff`` (``BOARDIOC_POWEROFF``) has the charger cut the cell off
  (ship mode), which leaves only the cell's own chips powered.  Plugging
  in USB power should end ship mode, as on the BQ25895; this has not been
  tried on the board yet.  On USB power the board cannot be turned off,
  and ``poweroff`` fails with ``EBUSY``.

LoRa
====

The SX1262 is registered as ``/dev/lora0`` with NuttX's SX126x driver
(``LPWAN_SX126X``), and the ``lora`` command sends and receives packets with
the frequency, spreading factor, bandwidth, coding rate, power, sync word and
preamble given on the command line::

    nsh> lora tx hello
    nsh> lora -t 30 rx

* The module is the 868 MHz variant, with a TCXO powered from DIO3 at 2.4 V
  and its RF switch on DIO2.  The board allows 863-870 MHz and -9 to
  +22 dBm; ``lora`` sends at 14 dBm unless told otherwise.  Mind the band's
  duty cycle limits.
* The radio sleeps from boot (``full`` switches its rail on), is reset when
  ``/dev/lora0`` is opened and put back to sleep when it is closed.  DIO1
  (GPIO5) is its interrupt, level triggered.
* The driver waits for the chip's BUSY line before every command, clears
  the chip's interrupt status in its worker, returns received packets with
  their RSSI and SNR, and takes a receive timeout
  (``SX126XIOC_RXTIMEOUTSET``) and a LoRa sync word
  (``SX126XIOC_SYNCWORDSET``).  Its setup starts the TCXO, calibrates, and
  applies the datasheet's known-limitation fixes.
* A send takes its time on air and a few milliseconds more: 130 ms for
  124 ms on air at SF9 and 125 kHz.  In earlier firmware, the same driver
  exchanged packets both ways with a MeshCore node (a T-Watch S3 at
  869.618 MHz, 62.5 kHz, SF8).

GNSS
====

The u-blox MIA-M10Q talks NMEA on UART1 (``/dev/ttyS0``) at 38400 baud.
``full`` registers it with NuttX's GNSS upper half (``SENSORS_GNSS``), which
parses the sentences into the uORB topics ``sensor_gnss`` and
``sensor_gnss_satellite`` and passes them through ``/dev/ttyGNSS0``::

    nsh> gps -t 60

``gps`` waits for a fix, reports the satellites in view while it waits and
prints the position in lines that fit 40 columns; ``-s`` sets the system
clock from the fix.

* The receiver is powered while any of these is in use: the first user
  switches the ``gps_en`` rail on and starts a thread that feeds the UART
  to the upper half; five seconds after the last user has gone, the thread
  stops and the rail goes off, so that a program that closes and opens it
  again does not cost a cold start.
* Writing to ``/dev/ttyGNSS0`` sends commands (NMEA or UBX) to the
  receiver.
* The build downloads the minmea library, which needs ``unzip``.
* Indoors, from a cold start, it reports satellites in view; a position
  fix has not been tried with a view of the sky yet.

Modem
=====

The SIMCom A7682E (4G LTE Cat 1) is on UART2 at 115200 baud as
``/dev/ttyS1``, and the ``modem`` command drives it::

    nsh> modem on            # supply, PWRKEY, wait for the first answer
    nsh> modem info          # identity, SIM, signal, network
    nsh> modem at +CSQ       # any command, the AT prefix is optional
    nsh> modem sms send <number> hello
    nsh> modem sms list      # unread, or "list all"
    nsh> modem term          # type at it yourself; Ctrl-] leaves
    nsh> modem off

* ``modem on`` switches the ``modem_pwr`` rail on, presses PWRKEY (high on
  the XL9555 pulls the modem's PWRKEY low) for 100 ms, then sends ``AT``
  once a second until the modem answers, about 7 s later; it turns the
  command echo off and verbose errors on.  The supply alone does not start
  the modem.
* ``modem off`` shuts the modem down with ``AT+CPOF``, or with a 3 s
  PWRKEY press if it does not answer, waits for it to stop answering, and
  only then cuts its rail: about 6 s in all.  If it keeps running, the
  rail is left on and the command fails.
* The level shifter between the modem and the ESP32-S3 (RX, TX, RI and
  DTR) is powered by the modem's own 1.8 V output, so while the modem is
  off UART2 is silent.
* The SIM and the microSD card share one holder and can both be fitted.
  The modem has its own microphone; its speaker output and the codec's
  share the speaker, chosen by ``audio_sel``.

.. warning::

   Do not cut ``modem_pwr`` while the modem is running: SIMCom warns that
   it can damage the modem's flash.  Use ``modem off``.

Found with earlier firmware for this board:

* Without IMS, SMS need the modem attached to the circuit-switched domain
  too (``AT+CEMODE=2``, kept across power cycles); with LTE for data only
  and no IMS, sending fails with ``+CMS ERROR: Network timeout``.  The
  modem refuses ``AT+CMGS`` with the UCS2 character set and GSM 7-bit
  coding together, so the command sends the text as IRA.
* Calls work over VoLTE where the operator offers it for the line (the
  modem registers to IMS by itself, ``+CIREG: 1,1``).  Without VoLTE a
  call falls back to GSM, and the modem switched itself off as it left
  LTE, probably a brownout on the GSM transmit bursts.
* Mobile data ran over PPP (NuttX's ``pppd``, on a 3GPP 27.010 channel of
  the modem's link); the operator's path dropped packets over 1480 bytes,
  so the TUN MTU wants 1440.

Audio
=====

The ES8311 codec on I2S0 is registered with the ``es8311`` driver
(``AUDIO_ES8311``) as ``/dev/audio/pcm0`` for playback, through the PCM
decoder, and ``/dev/audio/pcm_in0`` for recording from the microphone, an
analog electret on the codec's MIC1 input, with 30 dB of analog and 36 dB
of digital gain (``ES8311_MIC_GAIN``).  ``full`` puts the decoder in raw
mode (``AUDIO_FORMAT_RAW``), so it passes raw PCM through and does not
parse WAV headers: WAV files do not play.  ``nxplayer`` and
``nxrecorder`` drive them, for example with raw 16-bit mono files at
16 kHz on the microSD card::

    nsh> nxrecorder
    nxrecorder> device /dev/audio/pcm_in0
    nxrecorder> recordraw /mnt/sd/rec.raw 1 16 16000
    nxrecorder> stop
    nxrecorder> q
    nsh> nxplayer
    nxplayer> device /dev/audio/pcm0
    nxplayer> volume 50
    nxplayer> playraw /mnt/sd/rec.raw 1 16 16000
    nxplayer> q

* ``pcm0`` and ``pcm_in0`` are the same codec, set up twice: use one at a
  time, as closing either resets the codec.
* The speaker's amplifier (``amp_en``) is on while the playback device is
  reserved, and goes off when it is closed for the last time, even by a
  player that was killed.  Between streams the codec's analog side is
  powered down.
* A player killed mid-stream leaves ``pcm0`` reserved: it is busy until
  the next restart.
* The vendor's pin table and macros name the two data lines the other way
  round from the schematic: the ESP32-S3 sends on GPIO40 (the codec's
  DSDIN) and receives on GPIO17 (its ASDOUT).
* Writes to the microSD card put low-frequency noise on the microphone, at
  the rate the recorder writes buffers and its harmonics.  ``full`` raises
  the ADC's high-pass filter (``ES8311_ADC_HPF=4``), which takes it down
  by about 30 dB below 20 Hz while costing speech about 3 dB at
  200-500 Hz.  The vibration motor is picked up too.
* Every transfer goes through a buffer in internal RAM, where the DMA
  works, from the kernel heap.  When no such buffer can be had, the
  transfer fails and its audio buffer comes back empty, so a recording
  stops growing, and ``stop`` still works.
* Record to the microSD card, not to ``/tmp``.  In ``full`` the kernel
  heap is internal RAM only (see Memory below), about 230 KB, and
  ``/tmp`` is a tmpfs on it: a recording into ``/tmp`` stops after about
  five seconds, when the heap runs out, and deleting the file gives the
  memory back.
* The ESP32-S3's I2S driver and the ES8311 driver needed fixes for this,
  found with earlier firmware for this board: receive clocks in
  full-duplex master mode, mono slot selection, a receiver that keeps
  running across buffers, DMA buffers in internal RAM, and buffers that
  were lost or never completed.  Playback also needed the interrupt
  sources that the ROM leaves attached (UART0 on CPU interrupt 5)
  detached at start-up: the I2S output's DMA got the same CPU interrupt,
  and UART0's interrupt, which nothing cleared, never stopped.

IMU
===

The BHI260AP smart sensor hub, at I2C address 0x28 and powered by the
XL9555's ``imu_en`` line, is registered with the ``bhi260ap`` driver
(``SENSORS_BHI260AP``) as the uORB topics ``sensor_accel0`` and
``sensor_gyro0``, which ``sensortest`` reads::

    nsh> sensortest -n 5 -i 20000 accel0
    nsh> sensortest -n 5 -i 20000 gyro0

* The chip runs a firmware image from its program RAM.  The driver uploads
  Bosch's standard BHI260AP image when a sensor is first activated (about
  3.5 s over I2C at 400 kHz) and leaves it running.
* The image is not kept in this tree: the build downloads ``BHI260AP.fw``
  from Bosch Sensortec's BHI2xy SensorAPI, under that repository's
  BSD-3-Clause license, at a fixed commit, checks its SHA-256, and includes
  it in the firmware (``esp32s3_imu_fw.S``).  Building with the IMU needs
  the network the first time, and again after ``make distclean``, which
  deletes the image.
* The accelerometer (m/s², 8 g range) and the gyroscope (rad/s, 2000
  degrees/s) are the firmware's passthrough sensors, at 50 Hz by default.
  Their host interrupt on GPIO21 is level triggered.

Vibration motor
===============

The vibration motor is an ERM driven by a DRV2605L at I2C address 0x5a.
The chip's supply is the XL9555's ``motor_en`` line, on from the start.  The
driver is ``drivers/input/drv2605.c`` (``FF_DRV2605``), and ``full``
registers it as the force feedback device ``/dev/input_ff0``, with the
``haptic`` command to play it::

    nsh> haptic                  # 200 ms at full strength
    nsh> haptic buzz 500 40      # 500 ms at 40 %
    nsh> haptic effect 1 7 10    # strong click, soft bump, double click

* The driver runs the motor in open loop with the chip's ERM library 1, as
  the vendor's firmware does, and puts the chip in standby between effects.
  Full strength is the chip's default overdrive clamp, about 3 V, as in the
  vendor's firmware.
* ``FF_CONSTANT`` and ``FF_RUMBLE`` effects run the motor at a level for the
  effect's length; ``FF_PERIODIC`` effects with the ``FF_CUSTOM`` waveform
  play up to eight of the chip's 123 built-in effects, whose numbers go in
  ``custom_data``.  ``FF_GAIN`` scales the level.
* Effects uploaded to a force feedback device outlive the file they were
  uploaded through, and only that file may erase them: a program that exits
  without ``EVIOCRMFF`` leaves its slot in use for good.  ``haptic`` waits
  for its effect to end, with the driver's ``DRV2605IOC_BUSY`` ioctl, and
  erases it.

Wi-Fi and Bluetooth
===================

``full`` has the ESP32-S3's 2.4 GHz Wi-Fi station and Bluetooth LE, with
their coexistence arbiter, but neither radio is brought up at boot:
bringing one up costs a large part of the internal heap, and the network
stack starts the Wi-Fi radio as soon as ``wlan0`` exists.  Each radio is a
device that brings it up when 1 is written to it::

    nsh> gpio -o 1 /dev/wifi_en
    nsh> ifup wlan0
    nsh> wapi scan wlan0
    nsh> wapi psk wlan0 <passphrase> WPA_ALG_CCMP WPA_VER_2
    nsh> wapi essid wlan0 <ssid> WAPI_ESSID_ON
    nsh> renew wlan0

    nsh> gpio -o 1 /dev/ble_en
    nsh> ifup bnep0
    nsh> bt bnep0 scan start
    nsh> bt bnep0 scan get

* Bringing a radio up is a one-way trip: its driver registers a network
  device it has no way to remove.  Once up, Wi-Fi can still be stopped
  and started with ``ifdown`` and ``ifup``.
* ``LILYGO_TDECK_MAX_BOOT_WIFI`` and ``LILYGO_TDECK_MAX_BOOT_BLE`` bring a
  radio up at boot instead; the build refuses them with an init thread
  stack of less than 4 KB, as the radios' set-up overflowed 2 KB in a
  silent reset loop.
* The coexistence arbiter starts the timer service it needs before it
  sets up its timers; until then its first timer could not be created.
* With both radios up, about 118 KB of the kernel heap is left; with
  Wi-Fi only, about 155 KB.
* The ESP32-S3 radio is 2.4 GHz only: 5 GHz access points are invisible
  to it.

Memory
======

Wi-Fi rules out PSRAM in the kernel heap (``ESP32S3_SPIRAM_COMMON_HEAP``),
so ``full`` has a separate user heap (``ESP32S3_SPIRAM_USER_HEAP``, which
selects ``MM_KERNEL_HEAP``): ``free`` shows about 230 KB of internal RAM as
``Kmem`` and the 8 MB of PSRAM as ``Umem``.  Drivers, DMA buffers, the
network stack and tmpfs take from the kernel heap; programs from the user
heap.  The Espressif HAL frees memory from ``heap_caps_malloc()``, which is
in the kernel heap, with plain ``free()``: the build has ``free()`` in the
HAL's components go back to the heap the memory came from.  The user heap
also needs the low priority work queue (``SCHED_LPWORK``), which the
Bluetooth stack selects here: a configuration without Bluetooth has to
set it, or the PSRAM is left out of the heaps.

Power management
================

``full`` has NuttX's power management (``PM``, with the greedy governor)
and the tickless scheduler.  Whenever nothing holds the chip awake, it
light-sleeps (PM_STANDBY) until its next timer, at most ``PM_ALARM_SEC``
(60 s), or until a wake-up pin.

* Every state is held from the start (``PM_GOVERNOR_EXPLICIT_RELAX=-1``),
  and the end of the board's start releases PM_NORMAL and PM_IDLE.
  PM_STANDBY stays held: PM_SLEEP is a deep sleep, which on this board
  would end in a restart.
* What holds the chip awake is listed in ``/proc/pm/wakelock0``:

  ============== ======================================================
  Wakelock       Held while
  ============== ======================================================
  ``usb``        there is USB power: the board has no line that tells,
                 so the charger is asked every 2 s
  ``gps_en``     the GNSS receiver's rail is on (its UART stops in
                 light sleep)
  ``modem_pwr``  the modem's rail is on (likewise)
  ``audio_out``  ``/dev/audio/pcm0`` is reserved (I2S stops in light
                 sleep)
  ``audio_in``   ``/dev/audio/pcm_in0`` is reserved
  ``wifi``       Wi-Fi has been brought up (until a restart)
  ``lights``     a light is on (the LED controller stops in light
                 sleep)
  ``awake``      ``/dev/awake`` is open
  ============== ======================================================

  The Bluetooth controller holds the chip awake itself while it runs
  (under ``system``), and console input does for 6 s (``pm_activity()``).
  ``/proc/pm/state0`` shows the time in each state; light sleep is
  standby's ``SLEEP`` column.
* ``/dev/awake`` holds the chip awake while it is open, for a program
  that needs it so; the hold ends when the program does::

    nsh> sleep 10 < /dev/awake

  ``pmconfig stay idle`` and ``pmconfig relax idle`` hold and release it
  from NSH.
* A pin wakes the chip while its driver has the pin's interrupt enabled:
  the keyboard's, the touch screen's while ``/dev/input0`` is open, the
  LoRa radio's DIO1, and the IMU's.
* Without USB power the USB Serial/JTAG port is disconnected, as if the
  cable had been pulled, and connected again within 2 s of USB power
  coming back.  The USB console could not wake the chip anyway.  Writes
  to the console meanwhile block once its buffers are full, about 128
  bytes, until USB power returns.
* A line driven high into a chip whose rail is off feeds the chip
  through its input protection.  The GNSS UART's transmit line (GPIO 16)
  fed the unpowered receiver about 25 mA: while ``gps_en`` is off, the
  line is taken off the UART and held low, and what is written to
  ``/dev/ttyS0`` meanwhile is lost (``/dev/ttyGNSS0`` writes only while
  the receiver is on).
* During light sleep every pin keeps its configuration, outputs included,
  so inputs must not be left floating.
* The clock gains about 0.85 % of the time slept: the sleep is timed by
  the internal 150 kHz RC oscillator, which runs faster asleep than when
  it was calibrated.  The board has no 32 kHz crystal: its pins, GPIO 15
  and 16, are the keyboard's interrupt and the GNSS receiver's TX.

To measure from the cell with the cable attached, cut the charger's
input (``EN_HIZ``, bit 7 of its register 0), read the gauge's current in
mA (register 0x0c, negative while discharging), and restore it::

    nsh> i2c set -b 0 -a 0x6a -r 0x00 -w 8 0xc8
    nsh> i2c get -b 0 -a 0x55 -r 0x0c -w 16
    nsh> i2c set -b 0 -a 0x6a -r 0x00 -w 8 0x48

Without USB power the console is gone, so run such a test as a script
in the background, with the restore in the same script and its output
redirected to a file (``sh /tmp/test.sh > /tmp/test.log &``): a script
that writes to the console blocks once the console's buffers are full,
and never reaches its restore, which leaves the board on its cell until
it is switched off.  ``pmconfig stay idle`` in it measures the chip
awake.  The gauge takes about 30 s to settle after a change.  Bluetooth
leaves part of the radio powered after it has been up, about 16 mA,
until the board is next powered off or Wi-Fi has been brought up and
down (``ifup wlan0``, ``ifdown wlan0``).

Measured that way with the radios off and nothing running (2026-10-17):
43 mA awake and about 5 mA in light sleep, which is close to what the
gauge resolves.  Along the way, each measured against the run before
it (the baseline varies by a few mA from run to run):

* the GNSS receiver's transmit line held low with its rail off: 25 mA;
* the scheduler's timeslice timer stopped when only the idle task runs:
  4 mA in light sleep, where it woke the chip after each time a task had
  run;
* the start code loading the chip's calibrated core voltages and RTC
  settings (``esp_rtc_init()``): 14 mA awake;
* the audio codec held in reset while no stream runs: about 3 mA in
  light sleep, 4 mA awake.

Configurations
==============

nsh
---

NSH on the USB Serial/JTAG console, with the PSRAM added to the heap,
``/proc`` and ``/tmp`` mounted, and ``reboot``.

The module's PSRAM runs in **quad** mode (``ESP32S3_SPIRAM_MODE_QUAD``, the
default), although the help text of its Kconfig entry describes an octal
one: octal mode would also take GPIO 33 to 37, which the board uses for the
SPI bus and the e-paper.

full
----

Every part the port supports so far: ``nsh``, plus the I2C bus
(``/dev/i2c0``, the ``i2c`` command), the XL9555's lines as GPIO devices
(the ``gpio`` command), the SPI bus, the e-paper (``/dev/fb0``, the ``fb``
test pattern), both lights (``/dev/pwm0``, the ``pwm`` command), the
keyboard (``/dev/kbd0``, the ``kbd`` command), the touch screen
(``/dev/input0``, the ``tc`` command), the soft keys
(``/dev/softkeys``), the microSD card (``/dev/mmcsd0``, with FAT), the
fuel gauge (``/dev/batt0``), the charger (``/dev/charger0``),
``poweroff``, the LoRa radio (``/dev/lora0``, the ``lora`` command), the
GNSS receiver (``/dev/ttyGNSS0`` and its uORB topics, the ``gps``
command), the modem (``/dev/ttyS1``, the ``modem`` command), the audio
codec (``/dev/audio/pcm0``, ``/dev/audio/pcm_in0``, ``nxplayer``,
``nxrecorder``), the vibration motor (``/dev/input_ff0``, the ``haptic``
command), the IMU (the uORB topics ``sensor_accel0`` and ``sensor_gyro0``,
the ``sensortest`` command), Wi-Fi and Bluetooth LE (``/dev/wifi_en``,
``/dev/ble_en``, the ``wapi``, ``ping`` and ``bt`` commands), power
management (light sleep, ``/proc/pm``, ``/dev/awake``, ``pmconfig``), and
the system log in a 4 KB RAM log (``dmesg``).
The RAM log is not cleared by a reset, so the dump of a crash is still
there after the board restarts; each start is marked with its reset
reason (1 is a power-up, 3 a software reset, 21 a reset over USB, as
``esptool`` does).

Programs get 4 KB stacks by default (``DEFAULT_TASK_STACKSIZE``): with
2 KB, ``nxplayer`` and ``uorb_listener`` overflowed theirs.

Debugging
=========

The USB Serial/JTAG unit is also a JTAG probe.  It needs a udev rule that
gives the user access to USB ID ``303a:1001`` (for example ``MODE="0660",
GROUP="plugdev", TAG+="uaccess"``).  Then::

    $ openocd -c 'set ESP_RTOS hwthread' -f board/esp32s3-builtin.cfg \
        -c 'esp32s3.cpu0 configure -event gdb-detach {resume}' \
        -c 'esp32s3.cpu1 configure -event gdb-detach {resume}' \
        -c 'init; reset halt; esp appimage_offset 0x0'
    $ xtensa-esp32s3-elf-gdb -ex 'target extended-remote :3333' \
        -ex 'monitor reset halt' -ex 'thbreak esp32s3_bringup' nuttx

Without the ``gdb-detach`` handlers OpenOCD leaves both cores halted when
the debugger detaches, and the board looks dead until it is reset.  Use
hardware breakpoints (``thbreak``, ``hbreak``): the code runs from flash.
