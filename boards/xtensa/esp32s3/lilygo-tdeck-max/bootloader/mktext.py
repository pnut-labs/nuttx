#!/usr/bin/env python3
############################################################################
# boards/xtensa/esp32s3/lilygo-tdeck-max/bootloader/mktext.py
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

"""Render the progress screen's lines into progress_text.h.

    mktext.py IBMPlexSans-Regular.ttf IBMPlexSans-Bold.ttf > progress_text.h

MCUboot cannot draw text from a font, so each line is drawn here, once:
FreeType renders it for a monochrome target with its auto-hinter, which
fits the outlines and the advances to whole pixels, as the panel shows
them.  A line is a bitmap, a row at a time, the leftmost pixel the most
significant bit, a set bit black.  The fonts are IBM Plex Sans (SIL Open
Font License 1.1, https://github.com/IBM/plex), as in the device's
interface design; they are not kept here.  Needs freetype-py.  The
header here was made with FreeType 2.13.2 (freetype-py 2.5.1): another
version may hint a few pixels differently.
"""

import sys

import freetype

LINES = [
    ("title_update", "bold", 15, "Installing the update"),
    ("title_undo", "bold", 15, "Undoing the update"),
    ("caption", "regular", 12, "Don't switch off \u00b7 about a minute"),
]

BANNER = "/" + "*" * 76 + "\n * %s\n " + "*" * 76 + "/"

LOAD = (
    freetype.FT_LOAD_RENDER
    | freetype.FT_LOAD_TARGET_MONO
    | freetype.FT_LOAD_FORCE_AUTOHINT
)


def render(path, size, text):
    """Return (width, height, rows) for text, each row a list of 0/1."""
    face = freetype.Face(path)
    face.set_pixel_sizes(0, size)
    ascender = face.size.ascender >> 6
    height = ascender - (face.size.descender >> 6)
    glyphs = []
    pen = 0
    for ch in text:
        face.load_char(ch, LOAD)
        g = face.glyph
        bm = g.bitmap
        bits = [
            [
                (bm.buffer[r * bm.pitch + c // 8] >> (7 - c % 8)) & 1
                for c in range(bm.width)
            ]
            for r in range(bm.rows)
        ]
        glyphs.append((pen + g.bitmap_left, ascender - g.bitmap_top, bits))
        pen += g.advance.x >> 6
    rows = [[0] * pen for _ in range(height)]
    for x0, y0, bits in glyphs:
        for r, row in enumerate(bits):
            for c, bit in enumerate(row):
                if not bit:
                    continue
                if 0 <= y0 + r < height and 0 <= x0 + c < pen:
                    rows[y0 + r][x0 + c] = 1
                else:
                    print("mktext: %r clipped" % text, file=sys.stderr)
    return pen, height, rows


def emit(name, width, height, rows, text):
    stride = (width + 7) // 8
    data = []
    for row in rows:
        for b in range(stride):
            byte = 0
            for bit in range(8):
                x = b * 8 + bit
                if x < width and row[x]:
                    byte |= 0x80 >> bit
            data.append(byte)
    out = ["/* %s */" % text.replace("\u00b7", "-"), ""]
    out.append("static const uint8_t g_%s_bits[%d] =" % (name, len(data)))
    out.append("{")
    for i in range(0, len(data), 12):
        out.append("  " + ", ".join("0x%02x" % v for v in data[i : i + 12]) + ",")
    out.append("};")
    out.append("")
    out.append("static const struct text_s g_%s =" % name)
    out.append("{")
    out.append("  %d, %d, g_%s_bits" % (width, height, name))
    out.append("};")
    out.append("")
    return out


def main():
    fonts = {"regular": sys.argv[1], "bold": sys.argv[2]}
    out = [
        "/" + "*" * 76,
        " * boards/xtensa/esp32s3/lilygo-tdeck-max/bootloader/progress_text.h",
        " *",
        " * SPDX-License-Identifier: Apache-2.0",
        " *",
        " * Made by mktext.py from IBM Plex Sans (SIL Open Font License 1.1):",
        " * do not edit.",
        " *",
        " " + "*" * 76 + "/",
        "",
        "#ifndef __BOARDS_XTENSA_ESP32S3_LILYGO_TDECK_MAX_BOOTLOADER_PROGRESS_TEXT_H",
        "#define __BOARDS_XTENSA_ESP32S3_LILYGO_TDECK_MAX_BOOTLOADER_PROGRESS_TEXT_H",
        "",
        BANNER % "Included Files",
        "",
        "#include <stdint.h>",
        "",
        BANNER % "Public Types",
        "",
        "/* A line of text: a row at a time, the leftmost pixel the most",
        " * significant bit, a set bit black",
        " */",
        "",
        "struct text_s",
        "{",
        "  uint16_t w;",
        "  uint16_t h;",
        "  const uint8_t *bits;",
        "};",
        "",
        BANNER % "Public Data",
        "",
    ]
    for name, weight, size, text in LINES:
        width, height, rows = render(fonts[weight], size, text)
        out += emit(name, width, height, rows, text)
    out.append("#endif /* __BOARDS_XTENSA_..._BOOTLOADER_PROGRESS_TEXT_H */")
    print("\n".join(out))


if __name__ == "__main__":
    main()
