#!/usr/bin/env python3
"""Generate src/assets/umbreon_logo.c from umbreon_logo_source.png (8:1 nearest)."""

from __future__ import annotations

from pathlib import Path

from PIL import Image

REPO = Path(__file__).resolve().parents[1]
SRC = REPO / "src" / "assets" / "umbreon_logo_source.png"
OUT_C = REPO / "src" / "assets" / "umbreon_logo.c"
OUT_H = REPO / "src" / "assets" / "umbreon_logo.h"
PREVIEW = REPO / "tools" / "logo_preview.png"


def pixel_on(rgb: tuple[int, int, int]) -> bool:
    r, g, b = rgb
    if r < 120 or g < 120:
        return False
    if b > min(r, g) - 40:
        return False
    return (r + g) // 2 > 160


def mono_to_xbm(mono: Image.Image) -> tuple[int, int, list[int]]:
    """Pack for u8g2_DrawXBMP (u8g2_DrawHXBMP): LSB = left pixel in each byte."""
    w, h = mono.size
    bpr = (w + 7) // 8
    data: list[int] = []
    for y in range(h):
        for bx in range(bpr):
            byte = 0
            for bit in range(8):
                x = bx * 8 + bit
                if x < w and mono.getpixel((x, y)):
                    byte |= 1 << bit
            data.append(byte)
    return w, h, data


def main() -> None:
    im = Image.open(SRC).convert("RGB")
    if im.size != (1024, 512):
        raise SystemExit(f"expected 1024x512 source, got {im.size}")

    small = im.resize((128, 64), Image.NEAREST)
    mono = Image.new("1", (128, 64), 0)
    sp = small.load()
    mp = mono.load()
    for y in range(64):
        for x in range(128):
            if pixel_on(sp[x, y]):
                mp[x, y] = 1

    rows = [y for y in range(64) if any(mono.getpixel((x, y)) for x in range(128))]
    if not rows:
        raise SystemExit("empty bitmap after threshold")
    mono = mono.crop((0, rows[0], 128, rows[-1] + 1))

    w, h, data = mono_to_xbm(mono)
    lines = []
    for i in range(0, len(data), 16):
        chunk = ", ".join(f"0x{b:02x}" for b in data[i : i + 16])
        lines.append(f"\t{chunk},")

    OUT_H.write_text(
        f"""#pragma once

#include <stdint.h>

#define UMBREON_LOGO_W {w}
#define UMBREON_LOGO_H {h}

extern const uint8_t umbreon_logo_bits[];
""",
        encoding="utf-8",
    )
    OUT_C.write_text(
        f"""/* Umbreon boot logo — 1-bit XBM, 8:1 nearest from umbreon_logo_source.png */

#include "umbreon_logo.h"

const uint8_t umbreon_logo_bits[] = {{
{chr(10).join(lines)}
}};
""",
        encoding="utf-8",
    )
    mono.resize((w * 4, h * 4), Image.NEAREST).save(PREVIEW)
    print(f"Wrote {w}x{h} ({len(data)} bytes) -> {OUT_C.name}")


if __name__ == "__main__":
    main()
