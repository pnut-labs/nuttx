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
* ``reboot``.

Serial Console
==============

NSH runs on the ESP32-S3's **USB Serial/JTAG** unit, which enumerates on
the computer as ``/dev/ttyACM0`` (Linux, USB ID ``303a:1001``); no
USB-to-UART bridge is involved.  The two UARTs are left for the GNSS
receiver (UART1) and the modem (UART2).

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

The SX1262's power rail, on the XL9555, is off until a configuration
drives the expander, and ``nsh`` does not.  Meanwhile its high chip select
feeds the radio, about 15 mA.

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
