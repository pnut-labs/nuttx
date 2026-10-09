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
* the keyboard as ``/dev/kbd0`` (``full``).

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
keyboard (``/dev/kbd0``, the ``kbd`` command), and the system log in a
4 KB RAM log (``dmesg``).  The RAM log is not cleared by
a reset, so the dump of a crash is still there after the board restarts;
each start is marked with its reset reason (1 is a power-up, 3 a software
reset, 21 a reset over USB, as ``esptool`` does).

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
