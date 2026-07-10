#!/usr/bin/env python3
"""
sim_dashboard.py — Pixel-accurate 128×64 dashboard simulator with overlap detection.

For each scenario, every draw call is attributed to a named zone.
If two zones write to the same pixel, it is recorded as an overlap.
Overlapping pixels are highlighted red in the output image.

Usage:
  python3 tools/sim_dashboard.py
  python3 tools/sim_dashboard.py --source src/screens

Exit code 0 if no overlaps, 1 if any overlap found.
"""

import argparse
import os
import re
import subprocess
import sys
import tempfile
import textwrap
from collections import defaultdict
from pathlib import Path
from PIL import Image, ImageDraw

# ── BDF font loader ──────────────────────────────────────────────────────────

class Font:
    def __init__(self, name, glyphs, ascent, descent=0):
        self.name = name
        self.glyphs = glyphs
        self.ascent = ascent
        self.descent = descent
        self.height = ascent + descent

    def text_width(self, s):
        return sum(self.glyphs.get(ord(c), {}).get("dwidth", 0) for c in s)


def load_bdf(path):
    """Return Font parsed from a BDF bitmap font."""
    glyphs = {}
    ascent = 7
    descent = 0
    with open(path, encoding="utf-8", errors="replace") as f:
        lines = f.readlines()
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line.startswith("FONT_ASCENT"):
            ascent = int(line.split()[1])
        elif line.startswith("FONT_DESCENT"):
            descent = int(line.split()[1])
        elif line.startswith("ENCODING"):
            cp = int(line.split()[1])
            dwidth = 0
            bbx = (0, 0, 0, 0)
            while i < len(lines) and not lines[i].strip().startswith("BITMAP"):
                cur = lines[i].strip()
                if cur.startswith("DWIDTH"):
                    dwidth = int(cur.split()[1])
                elif cur.startswith("BBX"):
                    parts = cur.split()
                    bbx = tuple(int(p) for p in parts[1:5])
                i += 1
            i += 1
            rows = []
            while i < len(lines) and not lines[i].strip().startswith("ENDCHAR"):
                raw = lines[i].strip()
                rows.append((int(raw, 16), len(raw) * 4))
                i += 1
            width, height, xoff, yoff = bbx
            glyphs[cp] = {
                "rows": rows,
                "width": width,
                "height": height,
                "xoff": xoff,
                "yoff": yoff,
                "dwidth": dwidth or width,
            }
        i += 1
    return Font(Path(path).stem, glyphs, ascent, descent)


def load_fonts(repo_root):
    bdf_dir = Path(repo_root) / "modules" / "u8g2" / "tools" / "font" / "bdf"
    paths = {
        "5x7": bdf_dir / "5x7.bdf",
        "6x10": bdf_dir / "6x10.bdf",
        "9x15": bdf_dir / "9x15.bdf",
    }
    missing = [str(p) for p in paths.values() if not p.exists()]
    if missing:
        raise FileNotFoundError("BDF font not found: " + ", ".join(missing))
    fonts = {name: load_bdf(path) for name, path in paths.items()}
    # u8g2's converted 9x15 font places the common title baseline at y=11.
    # The source BDF ascent is one pixel taller, which creates false display
    # bounds failures for the existing title bars.
    fonts["9x15"].ascent = 11
    return fonts


# ── Framebuffer with zone blame tracking ────────────────────────────────────

SCR_W = 128
SCR_H = 64

class Display:
    def __init__(self, strict_overlaps=True):
        self.buf   = [[0]    * SCR_W for _ in range(SCR_H)]
        self.blame = [[None] * SCR_W for _ in range(SCR_H)]  # zone per pixel
        self.color = 1
        self.font  = None
        self.font_ascent = 6
        self._zone = None
        self.strict_overlaps = strict_overlaps
        # overlap records: {(zone_a, zone_b): [(x, y), ...]}
        self._overlaps = defaultdict(list)
        self._bounds = []

    def clear_buffer(self):
        self.buf   = [[0]    * SCR_W for _ in range(SCR_H)]
        self.blame = [[None] * SCR_W for _ in range(SCR_H)]
        self._overlaps.clear()
        self._bounds.clear()

    # ── Zone management ──────────────────────────────────────────────────────

    def zone(self, name):
        """Context manager: attribute all draws until next zone() call to `name`."""
        self._zone = name
        return self

    # ── Primitive draw ops ───────────────────────────────────────────────────

    def _px(self, x, y):
        if not (0 <= x < SCR_W and 0 <= y < SCR_H):
            return
        if self.color == 0:
            self.buf[y][x] = 0
            self.blame[y][x] = None
            return
        existing = self.blame[y][x]
        changed = self.buf[y][x] != self.color
        if (existing is not None and existing != self._zone and self._zone is not None
                and (self.strict_overlaps or changed)):
            key = tuple(sorted([existing, self._zone]))
            self._overlaps[key].append((x, y))
        self.buf[y][x] = self.color
        if self._zone is not None:
            self.blame[y][x] = self._zone

    def set_draw_color(self, c): self.color = c
    def set_font(self, glyphs, ascent=6):
        self.font = glyphs
        self.font_ascent = glyphs.ascent if isinstance(glyphs, Font) else ascent

    def _record_bounds(self, x, y, w, h):
        if w <= 0 or h <= 0:
            return
        if x < 0 or y < 0 or x + w > SCR_W or y + h > SCR_H:
            self._bounds.append((self._zone, x, y, w, h))

    def draw_pixel(self, x, y):
        self._record_bounds(x, y, 1, 1)
        self._px(x, y)
    def draw_hline(self, x, y, w):
        self._record_bounds(x, y, w, 1)
        [self._px(x+i, y) for i in range(w)]
    def draw_vline(self, x, y, h):
        self._record_bounds(x, y, 1, h)
        [self._px(x, y+i) for i in range(h)]

    def draw_box(self, x, y, w, h):
        self._record_bounds(x, y, w, h)
        for dy in range(h):
            for dx in range(w):
                self._px(x+dx, y+dy)

    def draw_frame(self, x, y, w, h):
        self._record_bounds(x, y, w, h)
        self.draw_hline(x, y, w)
        self.draw_hline(x, y+h-1, w)
        self.draw_vline(x, y, h)
        self.draw_vline(x+w-1, y, h)

    def get_str_width(self, s):
        if isinstance(self.font, Font):
            return self.font.text_width(s)
        return sum(5 for c in s if ord(c) in (self.font or {}))

    def draw_str(self, x, y, s):
        """y is u8g2 baseline; glyph rows drawn from y-ascent upward."""
        top = y - self.font_ascent
        font_h = self.font.height if isinstance(self.font, Font) else self.font_ascent
        self._record_bounds(x, top, self.get_str_width(s), font_h)
        cx = x
        for ch in s:
            if isinstance(self.font, Font):
                glyph = self.font.glyphs.get(ord(ch))
                if glyph is None:
                    cx += 5
                    continue
                rows = glyph["rows"]
                width = glyph["width"]
                dwidth = glyph["dwidth"]
                for ry, (bits, bit_len) in enumerate(rows):
                    for bx in range(width):
                        shift = bit_len - 1 - bx
                        if shift >= 0 and bits & (1 << shift):
                            self._px(cx + glyph["xoff"] + bx, top + ry)
                cx += dwidth
                continue

            rows = (self.font or {}).get(ord(ch))
            if rows is None:
                cx += 5
                continue
            for ry, bits in enumerate(rows):
                for bx in range(5):
                    if bits & (0x10 >> bx):
                        self._px(cx+bx, top+ry)
            cx += 5

    # ── Overlap report ───────────────────────────────────────────────────────

    def overlap_report(self, scenario_name="", whitelist=None):
        whitelist = whitelist or set()
        lines = []
        header = f"=== Overlap check: {scenario_name} ===" if scenario_name else "=== Overlap check ==="
        lines.append(header)

        real, allowed = {}, {}
        for key, pixels in self._overlaps.items():
            (allowed if frozenset(key) in whitelist else real)[key] = pixels

        if not real and not allowed:
            lines.append("  OK — no overlaps")
            return "\n".join(lines), False

        if real:
            total_px = sum(len(v) for v in real.values())
            lines.append(f"  FAIL — {len(real)} overlap pair(s), {total_px} pixel(s) total")
            for (za, zb), pixels in sorted(real.items()):
                xs = [p[0] for p in pixels]
                ys = [p[1] for p in pixels]
                lines.append(
                    f"  [OVERLAP] '{za}' x '{zb}': "
                    f"{len(pixels)} px  x={min(xs)}..{max(xs)}  y={min(ys)}..{max(ys)}"
                )
        else:
            lines.append("  OK — no real overlaps")

        if allowed:
            lines.append(f"  (whitelisted: {len(allowed)} intentional pair(s))")
            for (za, zb), pixels in sorted(allowed.items()):
                xs = [p[0] for p in pixels]
                ys = [p[1] for p in pixels]
                lines.append(
                    f"  [ok]  '{za}' x '{zb}': "
                    f"{len(pixels)} px  x={min(xs)}..{max(xs)}  y={min(ys)}..{max(ys)}"
                )

        return "\n".join(lines), bool(real)

    def bounds_report(self, scenario_name=""):
        header = f"=== Bounds check: {scenario_name} ===" if scenario_name else "=== Bounds check ==="
        lines = [header]
        if not self._bounds:
            lines.append("  OK — all draw calls clipped inside 128×64")
            return "\n".join(lines), False
        lines.append(f"  FAIL — {len(self._bounds)} out-of-bounds draw call(s)")
        for zone, x, y, w, h in self._bounds[:20]:
            lines.append(f"  [BOUNDS] '{zone}': x={x} y={y} w={w} h={h}")
        if len(self._bounds) > 20:
            lines.append(f"  ... {len(self._bounds) - 20} more")
        return "\n".join(lines), True

    def is_blank(self):
        return not any(any(row) for row in self.buf)

    # ── Image export ─────────────────────────────────────────────────────────

    def to_image(self, scale=4, whitelist=None):
        whitelist = whitelist or set()
        img = Image.new("RGB", (SCR_W*scale, SCR_H*scale), (20, 20, 20))
        draw = ImageDraw.Draw(img)

        real_px     = set()
        allowed_px  = set()
        for key, pixels in self._overlaps.items():
            target = allowed_px if frozenset(key) in whitelist else real_px
            target.update(pixels)

        for y in range(SCR_H):
            for x in range(SCR_W):
                if (x, y) in real_px:
                    color = (220, 50, 50)    # red   — real overlap
                elif (x, y) in allowed_px:
                    color = (180, 160, 60)   # yellow — whitelisted/intentional
                elif self.buf[y][x]:
                    color = (200, 230, 200)  # green — normal on
                else:
                    continue
                draw.rectangle(
                    [x*scale, y*scale, (x+1)*scale-1, (y+1)*scale-1],
                    fill=color
                )
        return img


# ── Dashboard drawing ────────────────────────────────────────────────────────

STATUS_Y     = 0
SENSOR_Y     = 10
SENSOR_H     = 22
IMU_Y        = 45   # separator at IMU_Y-2 = 43 = IMU_SCALE_Y
IMU_SCALE_Y  = 43
IMU_MARKER_Y = 44
IMU_LABEL_Y  = 55   # tick degree-label baseline (below marker)
BAR_W        = 8
BAR_GAP      = 4
LEFT_X       = 4
RIGHT_X      = 80
CENTER_X     = 64
IMU_HALF_W   = 58
MAX_SENSOR_RANGE = 800


def bat_pct(v):
    full, empty = 8.4, 6.6
    if v >= full:  return 100
    if v <= empty: return 0
    return int((v - empty) / (full - empty) * 100)


def draw_status_bar(d, font, vbat=7.8, running=False):
    d.zone("status_bar")
    pct = bat_pct(vbat)
    d.set_font(font)
    blocks = (pct + 19) // 20
    buf = "[" + "".join("#" if i < blocks else "." for i in range(5)) + "]"
    d.draw_str(0, 7, buf)
    d.draw_str(38, 7, f"{pct}% {vbat:.1f}V")
    d.draw_str(96, 7, "RUN " if running else "IDLE")


def draw_separator(d, y, name):
    d.zone(name)
    d.draw_hline(0, y, SCR_W)


def draw_sensor_bars(d, font, distances):
    left_idx  = [5, 4, 3]
    left_lbl  = ["HL", "FL", "L "]
    right_idx = [2, 1, 0]
    right_lbl = ["R ", "FR", "HR"]

    for i in range(3):
        x = LEFT_X + i * (BAR_W + BAR_GAP)
        dist = distances[left_idx[i]]
        h = 0 if dist >= MAX_SENSOR_RANGE else dist * SENSOR_H // MAX_SENSOR_RANGE
        d.zone(f"sensor_bar_left_{i}")
        d.draw_frame(x, SENSOR_Y, BAR_W, SENSOR_H)
        if h > 0:
            d.draw_box(x+1, SENSOR_Y+SENSOR_H-h, BAR_W-2, h)
        d.zone(f"sensor_label_left_{i}")
        d.set_font(font)
        d.draw_str(x, SENSOR_Y+SENSOR_H+7, left_lbl[i])

    for i in range(3):
        x = RIGHT_X + i * (BAR_W + BAR_GAP)
        dist = distances[right_idx[i]]
        h = 0 if dist >= MAX_SENSOR_RANGE else dist * SENSOR_H // MAX_SENSOR_RANGE
        d.zone(f"sensor_bar_right_{i}")
        d.draw_frame(x, SENSOR_Y, BAR_W, SENSOR_H)
        if h > 0:
            d.draw_box(x+1, SENSOR_Y+SENSOR_H-h, BAR_W-2, h)
        d.zone(f"sensor_label_right_{i}")
        d.set_font(font)
        d.draw_str(x, SENSOR_Y+SENSOR_H+7, right_lbl[i])


def draw_imu_value_inline(d, font, yaw):
    """Numeric yaw in the center gap of the sensor-label row (x≈51..75 is free)."""
    buf = f"{yaw:.1f}"
    d.zone("imu_value_inline")
    d.set_font(font)
    vw = d.get_str_width(buf)
    d.draw_str((SCR_W - vw) // 2, SENSOR_Y + SENSOR_H + 7, buf)


def draw_imu_scale(d, font, yaw=12.5):
    yaw = max(-90.0, min(90.0, yaw))

    d.zone("imu_baseline")
    d.draw_hline(0, IMU_SCALE_Y, SCR_W)

    # Tick marks + degree labels below
    ticks   = (-90, -45, 0, 45, 90)
    t_lbls  = ("-90", "-45", "0", "45", "90")
    d.set_font(font)
    for t, lbl in zip(ticks, t_lbls):
        tx = CENTER_X + int(t * IMU_HALF_W / 90)
        d.zone("imu_ticks")
        d.draw_vline(tx, IMU_SCALE_Y-3, 6)
        lw = d.get_str_width(lbl)
        lx = tx - lw // 2
        lx = max(0, min(lx, SCR_W - lw))
        d.zone("imu_tick_labels")
        d.draw_str(lx, IMU_LABEL_Y, lbl)

    # 3×3 dot marker
    d.zone("imu_marker")
    mx = CENTER_X + int(yaw * IMU_HALF_W / 90.0)
    d.draw_box(mx-1, IMU_MARKER_Y, 3, 3)


def rssi_to_bars(rssi):
    if rssi >= -60: return 4
    if rssi >= -70: return 3
    if rssi >= -80: return 2
    if rssi >= -90: return 1
    return 0


def draw_wifi_strip(d, font, ready, is_ap, rssi):
    if not ready:
        return
    d.zone("wifi_separator")
    d.draw_hline(0, 56, SCR_W)

    bars = 4 if is_ap else rssi_to_bars(rssi)
    bar_h = [2, 3, 5, 7]
    bar_x = [2, 5, 8, 11]
    d.zone("wifi_bars")
    for i in range(4):
        bh = bar_h[i]
        bx = bar_x[i]
        by = 64 - bh
        if i < bars:
            d.draw_box(bx, by, 2, bh)
        else:
            d.draw_pixel(bx, by)

    d.zone("wifi_label")
    d.set_font(font)
    d.draw_str(16, 63, "AP" if is_ap else f"STA {rssi}")


def render_scenario(font, wifi_ready, wifi_is_ap, wifi_rssi,
                    vbat=7.8, yaw=12.5, distances=None):
    if distances is None:
        distances = [120, 250, 450, 300, 180, 800]

    d = Display()
    d.clear_buffer()

    draw_status_bar(d, font, vbat=vbat)
    draw_separator(d, 8,       "sep_status")
    draw_sensor_bars(d, font, distances)
    draw_imu_value_inline(d, font, yaw)
    draw_separator(d, IMU_Y-2, "sep_imu")
    draw_imu_scale(d, font, yaw=yaw)
    draw_wifi_strip(d, font, wifi_ready, wifi_is_ap, wifi_rssi)

    return d


# ── Main ─────────────────────────────────────────────────────────────────────

# Zone pairs that intentionally overlap (ticks cross the scale/separator baseline).
# Reported separately but do not count as failures.
OVERLAP_WHITELIST = {
    frozenset(["imu_baseline",    "sep_imu"]),       # same line y=40
    frozenset(["imu_baseline",    "imu_ticks"]),     # ticks cross baseline
    frozenset(["imu_ticks",       "sep_imu"]),       # same as above
    frozenset(["imu_marker",      "imu_ticks"]),     # marker may sit on a tick
    frozenset(["imu_tick_labels", "imu_ticks"]),     # label text drawn over tick x
}

SCENARIOS = [
    dict(title="No WiFi",     wifi_ready=False, wifi_is_ap=False, wifi_rssi=0),
    dict(title="STA -65 dBm", wifi_ready=True,  wifi_is_ap=False, wifi_rssi=-65),
    dict(title="AP mode",     wifi_ready=True,  wifi_is_ap=True,  wifi_rssi=0),
]


def discover_draw_functions(screens_dir):
    funcs = {}
    for path in sorted(Path(screens_dir).glob("screen_*.c")):
        text = path.read_text(encoding="utf-8", errors="replace")
        for match in re.finditer(r"void\s+(screen_\w+_draw)\s*\(([^)]*)\)", text):
            funcs[match.group(1)] = match.group(2).strip()
    return funcs


def write_source_harness(tmpdir, repo_root, screens_dir, funcs):
    tmpdir = Path(tmpdir)
    (tmpdir / "zephyr").mkdir()
    (tmpdir / "u8g2.h").write_text(textwrap.dedent(r"""
        #pragma once
        #include <stdint.h>
        #include <stddef.h>

        typedef struct {
            int font_id;
            int draw_color;
        } u8g2_t;

        typedef struct {
            int dummy;
        } u8x8_t;

        extern const uint8_t u8g2_font_5x7_tr[];
        extern const uint8_t u8g2_font_6x10_tr[];
        extern const uint8_t u8g2_font_9x15_tr[];

        void u8g2_SetFont(u8g2_t *u, const uint8_t *font);
        void u8g2_SetDrawColor(u8g2_t *u, uint8_t color);
        int  u8g2_GetStrWidth(u8g2_t *u, const char *s);
        void u8g2_DrawPixel(u8g2_t *u, int x, int y);
        void u8g2_DrawHLine(u8g2_t *u, int x, int y, int w);
        void u8g2_DrawVLine(u8g2_t *u, int x, int y, int h);
        void u8g2_DrawBox(u8g2_t *u, int x, int y, int w, int h);
        void u8g2_DrawFrame(u8g2_t *u, int x, int y, int w, int h);
        void u8g2_DrawStr(u8g2_t *u, int x, int y, const char *s);
        void u8g2_DrawXBMP(u8g2_t *u, int x, int y, int w, int h, const uint8_t *bitmap);
        void u8g2_SendBuffer(u8g2_t *u);
    """).strip() + "\n")

    (tmpdir / "zephyr" / "kernel.h").write_text(textwrap.dedent(r"""
        #pragma once
        #include <stdint.h>
        #define ARG_UNUSED(x) (void)(x)
        static inline int64_t k_uptime_get(void) { return 123000; }
    """).strip() + "\n")

    headers = "\n".join(f'#include "{p.name}"' for p in sorted(Path(screens_dir).glob("screen_*.h")))
    calls = []

    def add_call(name, setup, call):
        escaped = name.replace("\\", "\\\\").replace('"', '\\"')
        calls.append(textwrap.dedent(f"""
            reset_fixture(&st);
            {setup}
            begin_screen("{escaped}");
            {call}
            end_screen();
        """))

    for fn, args in sorted(funcs.items()):
        takes_state = "ui_state" in args
        call = f"{fn}(&st);" if takes_state else f"{fn}();"

        if fn == "screen_dashboard_draw":
            add_call("dashboard_no_wifi", "fake_wifi_ready = false;", call)
            add_call("dashboard_sta", "fake_wifi_ready = true; fake_wifi_ap = false; fake_wifi_rssi = -65;", call)
            add_call("dashboard_ap", "fake_wifi_ready = true; fake_wifi_ap = true;", call)
        elif fn == "screen_wifi_draw":
            add_call("wifi_status", "fake_wifi_ready = true; st.wifi_scroll = 0;", call)
            add_call("wifi_join_qr", "fake_wifi_ready = true; fake_wifi_ap = true; st.wifi_scroll = 1;", call)
            add_call("wifi_web_qr", "fake_wifi_ready = true; fake_wifi_ap = false; st.wifi_scroll = 2;", call)
        elif fn == "screen_settings_list_draw":
            add_call(fn, "st.grp_sel = 0; st.sel = 1; st.scroll = 0;", call)
        elif fn == "screen_edit_draw":
            add_call(fn, "st.param_idx = 0; st.edit_val = 800.0f;", call)
        elif fn == "screen_tests_running_draw":
            add_call(fn, "st.test_idx = 1;", call)
        elif fn == "screen_confirm_draw":
            add_call("confirm_no", 'st.confirm_msg = "Reset Defaults"; st.confirm_yes = false;', call)
            add_call("confirm_yes", 'st.confirm_msg = "Reset Defaults"; st.confirm_yes = true;', call)
        elif fn == "screen_info_draw":
            add_call(fn, "st.info_scroll = 0;", call)
        elif fn == "screen_boot_draw":
            add_call("boot_default", "", 'screen_boot_draw("Starting...");')
        elif fn == "screen_calibrating_draw":
            add_call("cal_countdown", "st.cal_type = 0; st.cal_phase = CAL_COUNTDOWN; st.cal_phase_start_ms = 0;", call)
            add_call("cal_done", "st.cal_type = 0; st.cal_phase = CAL_DONE; snprintf(st.cal_result, sizeof(st.cal_result), \"bias: 0.12\");", call)
        elif takes_state:
            add_call(fn, "st.sel = 1; st.scroll = 0;", call)
        else:
            add_call(fn, "", call)

    harness_c = textwrap.dedent(r"""
        #include <stdbool.h>
        #include <stdint.h>
        #include <stdio.h>
        #include <string.h>
        #include "display_internal.h"
        #include "settings.h"
        #include "battery.h"
        #include "sensors.h"
        #include "imu.h"
        #include "control.h"
        #include "wifi_cmd.h"
        #include "qrcodegen.h"
        #include "u8g2.h"
    """) + headers + textwrap.dedent(r"""

        u8g2_t u8g2 = {5, 1};
        const uint8_t u8g2_font_5x7_tr[] = {5};
        const uint8_t u8g2_font_6x10_tr[] = {6};
        const uint8_t u8g2_font_9x15_tr[] = {9};

        static bool fake_wifi_ready;
        static bool fake_wifi_ap;
        static int fake_wifi_rssi;
        static int fake_distances[6] = {120, 250, 450, 300, 180, 800};

        struct car_settings cfg = {
            .front_obstacle_dist = 800, .side_open_dist = 600,
            .all_close_dist = 400, .close_front_dist = 100,
            .pid_kp = 1.2f, .pid_ki = 0.1f, .pid_kd = 0.05f,
            .min_speed = 1520, .max_speed = 1700, .min_bspeed = 1450,
            .min_point = 55, .max_point = 125, .neutral_point = 90,
            .encoder_holes = 20, .wheel_diam_m = 0.064f,
            .loop_ms = 40, .spd_clear = 1.0f, .spd_blocked = 0.4f,
            .spd_slew = 3.0f, .kick_pct = 10.0f, .kick_ms = 100,
            .corner_kick_us = 25, .coe_clear = 1.0f, .coe_blocked = 0.5f,
            .wrong_dir_deg = 90.0f, .race_cw = true, .stuck_thresh = 200,
            .stall_thresh = 100, .reverse_brake_cmd = 1400,
            .reverse_drive_cmd = 1350, .reverse_brake_ms = 100,
            .reverse_drive_ms = 300, .long_reverse_brake_ms = 200,
            .long_reverse_drive_ms = 500, .long_forward_speed_cap = 0.5f,
            .long_forward_ms = 200, .imu_rotate = false,
            .servo_reverse = false, .calibrated = true,
            .bat_enabled = true, .bat_multiplier = 1.0f, .bat_low = 6.9f,
            .tach_glitch_filter_us = 50,
        };

        const struct param_desc params[PARAM_COUNT] = {
            {"FOD","Front Obs",PT_INT,10,100,50,9000,0,1},
            {"SOD","Side Open",PT_INT,10,100,50,9000,0,1},
            {"ACD","All Close",PT_INT,10,100,50,9000,0,1},
            {"CFD","Close Frt",PT_INT,10,100,50,9000,0,1},
            {"KP","PID Kp",PT_FLOAT,0.1f,1.0f,0,5000,0,1},
            {"KI","PID Ki",PT_FLOAT,0.5f,5.0f,0,10000,0,1},
            {"KD","PID Kd",PT_FLOAT,0.05f,0.5f,0,2000,0,1},
            {"MSP","Min Spd",PT_INT,5,20,1000,2000,0,1},
            {"XSP","Max Spd",PT_INT,5,20,1000,2000,0,1},
            {"BSP","Min Bck",PT_INT,5,20,1000,2000,0,1},
            {"KOP","Kick %",PT_FLOAT,0.5f,2.0f,0,80.0f,0,1},
            {"KOM","Kick ms",PT_INT,10,50,0,5000,0,1},
            {"CKU","Crn Kick",PT_INT,1,5,0,120,0,1},
            {"MNP","Srv Min",PT_INT,1,5,0,180,0,1},
            {"XNP","Srv Max",PT_INT,1,5,0,180,0,1},
            {"NTP","Srv Neu",PT_INT,1,5,0,180,0,1},
            {"ENH","Enc Holes",PT_INT,1,10,1,2000,0,1},
            {"WDM","Wheel mm",PT_FLOAT,1,10,10,1000,0,1000},
            {"TGF","Tach Flt",PT_INT,1,10,1,500,0,1},
            {"LMS","Loop ms",PT_INT,5,10,10,1000,0,1},
            {"SPD1","Spd Clr",PT_FLOAT,0.1f,0.5f,0,5.0f,0,1},
            {"SPD2","Spd Blk",PT_FLOAT,0.1f,0.5f,0,5.0f,0,1},
            {"SLW","Spd Slew",PT_FLOAT,0.05f,0.2f,0,20.0f,0,1},
            {"COE1","Coe Clr",PT_FLOAT,0.05f,0.1f,0,5.0f,0,1},
            {"COE2","Coe Blk",PT_FLOAT,0.05f,0.1f,0,5.0f,0,1},
            {"WDD","Wrong Dir",PT_FLOAT,5.0f,20.0f,1,360,0,1},
            {"RCW","Race CW",PT_BOOL,1,1,0,1,0,1},
            {"STK","Stuck Thr",PT_INT,1,5,0,1000,0,1},
            {"STL","Stall Thr",PT_INT,1,5,0,1000,0,1},
            {"WDT","Det Mode",PT_BOOL,1,1,0,1,0,1},
            {"WMT","Mnv Mode",PT_BOOL,1,1,0,1,0,1},
            {"WST","Sens Thr",PT_FLOAT,1.0f,5.0f,-10000,10000,0,1},
            {"RBC","Rev Brk",PT_INT,5,20,-1000,0,0,1},
            {"RDC","Rev Drv",PT_INT,5,20,-1000,0,0,1},
            {"RBM","Brk ms",PT_INT,10,100,0,5000,0,1},
            {"RDM","Drv ms",PT_INT,10,100,0,5000,0,1},
            {"LBM","LBrk ms",PT_INT,10,100,0,5000,0,1},
            {"LDM","LDrv ms",PT_INT,10,100,0,5000,0,1},
            {"LFS","Long Fwd",PT_FLOAT,0.01f,0.05f,0,2.0f,0,1},
            {"LFM","LFwd ms",PT_INT,10,100,0,5000,0,1},
            {"BSM","Burst Stp",PT_INT,10,50,0,5000,0,1},
            {"BPS","Burst Str",PT_INT,5,20,0,5000,0,1},
            {"BFS","Burst Spd",PT_FLOAT,0.1f,0.5f,0,5.0f,0,1},
            {"BFM","Burst ms",PT_INT,10,100,0,5000,0,1},
            {"IMR","IMU Rot",PT_BOOL,1,1,0,1,0,1},
            {"SVR","Srv Rev",PT_BOOL,1,1,0,1,0,1},
            {"CAL","Calibr.",PT_BOOL,1,1,0,1,0,1},
            {"BEN","Bat En",PT_BOOL,1,1,0,1,0,1},
            {"BML","Bat Mult",PT_FLOAT,0.1f,0.5f,0.1f,20.0f,0,1},
            {"BLV","Bat Low V",PT_FLOAT,0.1f,0.5f,0.1f,20.0f,0,1},
        };
        const struct param_group groups[GROUP_COUNT] = {
            {"Obstacles",0,4}, {"PID",4,3}, {"Speed/ESC",7,6},
            {"Steering",13,3}, {"Tachometer",16,3}, {"Control",19,6},
            {"Navigation",25,7}, {"Maneuver",32,12}, {"Hardware",44,6},
        };
        const struct test_item tests[TEST_COUNT] = {
            {"lidar","Lidar Scan",false}, {"servo","Servo Sweep",false},
            {"taho","Tachometer",false}, {"esc","ESC Test",true},
            {"speed","Speed Hold",true}, {"autotune","PID Autotune",true},
            {"reactive","Reactive",false}, {"cal","ESC Calibr.",true},
        };
        const struct action_item actions[ACTION_COUNT] = {
            {"Start Car",ACT_START,true}, {"Stop Car",ACT_STOP,false},
            {"Save NVS",ACT_SAVE,true}, {"Load NVS",ACT_LOAD,true},
            {"Reset Defaults",ACT_RESET,true},
            {"Gyro Cal",ACT_GYRO_CAL,false}, {"Accel Cal",ACT_ACCEL_CAL,false},
            {"Restart Lidar",ACT_SNS_RECOVER,true},
        };
        const char *main_items[MAIN_REAL] = {"Settings", "Tests", "Actions", "Info", "WiFi"};

        static int font_width(void) {
            if (u8g2.font_id == 9) return 9;
            if (u8g2.font_id == 6) return 6;
            return 5;
        }

        void u8g2_SetFont(u8g2_t *u, const uint8_t *font) {
            if (font == u8g2_font_9x15_tr) u->font_id = 9;
            else if (font == u8g2_font_6x10_tr) u->font_id = 6;
            else u->font_id = 5;
            printf("FONT %dx%d\n", u->font_id, u->font_id == 9 ? 15 : (u->font_id == 6 ? 10 : 7));
        }
        void u8g2_SetDrawColor(u8g2_t *u, uint8_t color) { u->draw_color = color; printf("COLOR %u\n", color); }
        int u8g2_GetStrWidth(u8g2_t *u, const char *s) { (void)u; return (int)strlen(s) * font_width(); }
        void u8g2_DrawPixel(u8g2_t *u, int x, int y) { (void)u; printf("PIXEL %d %d\n", x, y); }
        void u8g2_DrawHLine(u8g2_t *u, int x, int y, int w) { (void)u; printf("HLINE %d %d %d\n", x, y, w); }
        void u8g2_DrawVLine(u8g2_t *u, int x, int y, int h) { (void)u; printf("VLINE %d %d %d\n", x, y, h); }
        void u8g2_DrawBox(u8g2_t *u, int x, int y, int w, int h) { (void)u; printf("BOX %d %d %d %d\n", x, y, w, h); }
        void u8g2_DrawFrame(u8g2_t *u, int x, int y, int w, int h) { (void)u; printf("FRAME %d %d %d %d\n", x, y, w, h); }
        void u8g2_DrawStr(u8g2_t *u, int x, int y, const char *s) {
            (void)u;
            printf("STR %d %d %zu ", x, y, strlen(s));
            fwrite(s, 1, strlen(s), stdout);
            putchar('\n');
        }
        void u8g2_DrawXBMP(u8g2_t *u, int x, int y, int w, int h, const uint8_t *bitmap) {
            /* Match u8g2_DrawHXBMP: LSB of each byte is the leftmost pixel */
            int bw = (w + 7) / 8;
            for (int row = 0; row < h; row++) {
                for (int col = 0; col < w; col++) {
                    int bi = row * bw + (col / 8);
                    int bit = col % 8;
                    if (bitmap[bi] & (1 << bit)) {
                        u8g2_DrawPixel(u, x + col, y + row);
                    }
                }
            }
        }
        void u8g2_SendBuffer(u8g2_t *u) { (void)u; printf("SEND\n"); }

        float battery_get_voltage(void) { return 7.8f; }
        float battery_get_raw_voltage(void) { return 7.8f; }
        float battery_get_min_voltage(void) { return 7.7f; }
        const int *sensors_get_distances(void) { return fake_distances; }
        int sensors_online_count(void) { return 6; }
        bool imu_is_ok(void) { return true; }
        float imu_get_yaw_rate(void) { return 0.0f; }
        float imu_get_heading(void) { return 12.5f; }
        void imu_calibrate(void) {}
        void imu_calibrate_accel(void) {}
        float imu_get_gyro_bias(void) { return 0.0f; }
        void imu_get_accel_bias(float *x, float *y, float *z) { *x = *y = *z = 0.0f; }
        bool control_is_running(void) { return false; }
        bool control_is_monitor(void) { return false; }
        bool control_is_countdown(void) { return false; }
        bool wifi_status_is_ready(void) { return fake_wifi_ready; }
        bool wifi_status_is_ap(void) { return fake_wifi_ap; }
        int wifi_status_get_rssi(void) { return fake_wifi_rssi; }
        const char *wifi_status_get_ssid(void) { return fake_wifi_ap ? "UmbreonAP" : "LabNet"; }
        const char *wifi_status_get_ip(void) { return "192.168.4.1"; }
        const char *wifi_status_get_ap_pass(void) { return "racepass123"; }
        void wifi_cmd_send(const char *str) { (void)str; }
        void wifi_cmd_printf(const char *fmt, ...) { (void)fmt; }
        void wifi_log(const char *fmt, ...) { (void)fmt; }
        bool wifi_log_enabled(void) { return false; }

        float param_get(const struct param_desc *p) {
            if (p->type == PT_BOOL) return 1.0f;
            if (p->lo < 0.0f && p->hi <= 0.0f) return p->hi;
            return p->lo + (p->hi - p->lo) * 0.5f;
        }
        int param_fmt(char *buf, int sz, const struct param_desc *p, float v) {
            if (p->type == PT_BOOL) return snprintf(buf, sz, "%s", v > 0.5f ? "ON" : "OFF");
            if (p->type == PT_FLOAT && p->scale == 1.0f) return snprintf(buf, sz, "%.2f", (double)v);
            return snprintf(buf, sz, "%d", (int)(v + 0.5f));
        }

        bool qrcodegen_encodeText(const char *text, uint8_t tempBuffer[], uint8_t qrcode[],
            enum qrcodegen_Ecc ecl, int minVersion, int maxVersion, enum qrcodegen_Mask mask, bool boostEcl) {
            (void)text; (void)tempBuffer; (void)ecl; (void)minVersion; (void)maxVersion; (void)mask; (void)boostEcl;
            qrcode[0] = 21;
            return true;
        }
        int qrcodegen_getSize(const uint8_t qrcode[]) { (void)qrcode; return 21; }
        bool qrcodegen_getModule(const uint8_t qrcode[], int x, int y) {
            (void)qrcode;
            return x == 0 || y == 0 || x == 20 || y == 20 || ((x * 3 + y * 5) % 7) < 3;
        }

        static void begin_screen(const char *name) {
            u8g2.font_id = 5;
            u8g2.draw_color = 1;
            printf("BEGIN %s\n", name);
            printf("FONT 5x7\n");
            printf("COLOR 1\n");
        }
        static void end_screen(void) {
            printf("END\n");
        }
        static void reset_fixture(struct ui_state *st) {
            memset(st, 0, sizeof(*st));
            st->sel = 1;
            st->scroll = 0;
            st->grp_sel = 0;
            st->param_idx = 0;
            st->edit_val = 800.0f;
            st->test_idx = 0;
            st->action_idx = 0;
            st->confirm_msg = "Reset Defaults";
            fake_wifi_ready = false;
            fake_wifi_ap = false;
            fake_wifi_rssi = -65;
        }

        int main(void) {
            struct ui_state st;
    """) + "\n".join(calls) + textwrap.dedent(r"""
            return 0;
        }
    """)
    harness_path = tmpdir / "screen_harness.c"
    harness_path.write_text(harness_c, encoding="utf-8")
    return harness_path


def run_source_harness(repo_root, screens_dir, funcs):
    with tempfile.TemporaryDirectory(prefix="sim_screens_") as tmp:
        harness = write_source_harness(tmp, repo_root, screens_dir, funcs)
        exe = Path(tmp) / "screen_harness"
        sources = [str(harness)] + [str(p) for p in sorted(Path(screens_dir).glob("screen_*.c"))]
        logo_c = Path(repo_root) / "src" / "assets" / "umbreon_logo.c"
        if logo_c.exists():
            sources.append(str(logo_c))
        cmd = [
            "gcc", "-std=c99", "-Wall", "-Wextra",
            "-I", str(Path(tmp)),
            "-I", str(Path(repo_root) / "src"),
            "-I", str(Path(repo_root) / "src" / "assets"),
            "-I", str(Path(screens_dir)),
            *sources,
            "-lm",
            "-o", str(exe),
        ]
        build = subprocess.run(cmd, cwd=repo_root, text=True, capture_output=True)
        if build.returncode != 0:
            print("ERROR: source screen harness build failed", file=sys.stderr)
            print(build.stderr, file=sys.stderr)
            return None, 1
        run = subprocess.run([str(exe)], cwd=repo_root, text=True, capture_output=True)
        if run.returncode != 0:
            print("ERROR: source screen harness failed", file=sys.stderr)
            print(run.stderr, file=sys.stderr)
            return None, 1
        return run.stdout, 0


def parse_harness_output(output, fonts):
    panels = []
    current = None
    title = None
    op_index = 0

    for line in output.splitlines():
        if not line:
            continue
        if line.startswith("BEGIN "):
            title = line.split(" ", 1)[1]
            current = Display(strict_overlaps=False)
            current.clear_buffer()
            current.set_font(fonts["5x7"])
            current.set_draw_color(1)
            op_index = 0
            continue
        if line == "END":
            if current is not None:
                panels.append((title, current))
            current = None
            title = None
            continue
        if current is None:
            continue

        parts = line.split(" ", 4)
        op = parts[0]
        if op == "FONT":
            current.set_font(fonts.get(parts[1], fonts["5x7"]))
            continue
        if op == "COLOR":
            current.set_draw_color(int(parts[1]))
            continue
        if op == "SEND":
            continue

        op_index += 1
        current.zone(f"{title}:{op_index}:{op.lower()}")
        if op == "PIXEL":
            current.draw_pixel(int(parts[1]), int(parts[2]))
        elif op == "HLINE":
            current.draw_hline(int(parts[1]), int(parts[2]), int(parts[3]))
        elif op == "VLINE":
            current.draw_vline(int(parts[1]), int(parts[2]), int(parts[3]))
        elif op == "BOX":
            current.draw_box(int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4]))
        elif op == "FRAME":
            current.draw_frame(int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4]))
        elif op == "STR":
            _, x, y, n, rest = line.split(" ", 4)
            current.draw_str(int(x), int(y), rest[:int(n)])

    return panels


def save_panel_grid(panels, out_path, scale=4, columns=4):
    if not panels:
        return
    gap = 8
    label_h = 12
    columns = max(1, min(columns, len(panels)))
    rows = (len(panels) + columns - 1) // columns
    cell_w = SCR_W * scale
    cell_h = SCR_H * scale + label_h
    total_w = cell_w * columns + gap * (columns - 1)
    total_h = cell_h * rows + gap * (rows - 1)
    canvas = Image.new("RGB", (total_w, total_h), (10, 10, 10))

    try:
        from PIL import ImageFont as PILFont
        lbl_font = PILFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 10)
    except Exception:
        lbl_font = None

    cdraw = ImageDraw.Draw(canvas)
    for idx, (title, d) in enumerate(panels):
        col = idx % columns
        row = idx // columns
        ox = col * (cell_w + gap)
        oy = row * (cell_h + gap)
        canvas.paste(d.to_image(scale), (ox, oy + label_h))
        kw = dict(fill=(180, 180, 180))
        if lbl_font:
            kw["font"] = lbl_font
        cdraw.text((ox + 2, oy + 1), title, **kw)

    canvas.save(out_path)


def main_source(args, repo_root):
    screens_dir = Path(args.screens_dir or "src/screens")
    if not screens_dir.is_absolute():
        screens_dir = Path(repo_root) / screens_dir
    if not screens_dir.exists():
        print(f"ERROR: screens dir not found: {screens_dir}", file=sys.stderr)
        return 1

    funcs = discover_draw_functions(screens_dir)
    if not funcs:
        print(f"ERROR: no screen_*_draw functions found in {screens_dir}", file=sys.stderr)
        return 1

    try:
        fonts = load_fonts(repo_root)
    except FileNotFoundError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    output, rc = run_source_harness(repo_root, screens_dir, funcs)
    if rc != 0:
        return rc

    panels = parse_harness_output(output, fonts)
    any_fail = False
    print()
    print(f"Source screens: {screens_dir}")
    print(f"Draw functions: {', '.join(sorted(funcs))}")
    print()
    for title, d in panels:
        if d.is_blank():
            print(f"=== Blank check: {title} ===")
            print("  FAIL — rendered framebuffer is blank")
            print()
            any_fail = True
        bounds, bounds_failed = d.bounds_report(title)
        print(bounds)
        print()
        report, overlap_failed = d.overlap_report(title)
        print(report)
        print()
        any_fail = any_fail or bounds_failed or overlap_failed

    out_path = Path(args.output or (Path(repo_root) / "tools" / "sim_screens.png"))
    save_panel_grid(panels, out_path, scale=args.scale, columns=args.columns)
    print(f"Image saved: {out_path}")
    return 1 if any_fail else 0


def main_legacy():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root  = os.path.dirname(script_dir)
    bdf_path   = os.path.join(repo_root, "modules", "u8g2", "tools", "font", "bdf", "5x7.bdf")

    if not os.path.exists(bdf_path):
        print(f"ERROR: BDF font not found: {bdf_path}", file=sys.stderr)
        sys.exit(1)

    font = load_bdf(bdf_path)

    scale   = 4
    gap     = 8
    label_h = 12

    panels   = []
    any_fail = False

    print()
    for s in SCENARIOS:
        d = render_scenario(font, **{k: v for k, v in s.items() if k != "title"})
        report, failed = d.overlap_report(s["title"], whitelist=OVERLAP_WHITELIST)
        print(report)
        print()
        if failed:
            any_fail = True
        panels.append(d)

    # Compose image
    total_w = SCR_W*scale * len(panels) + gap*(len(panels)-1)
    total_h = SCR_H*scale + label_h
    canvas  = Image.new("RGB", (total_w, total_h), (10, 10, 10))

    try:
        from PIL import ImageFont as PILFont
        lbl_font = PILFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 10)
    except Exception:
        lbl_font = None

    cdraw = ImageDraw.Draw(canvas)
    for idx, (d, s) in enumerate(zip(panels, SCENARIOS)):
        ox = idx * (SCR_W*scale + gap)
        canvas.paste(d.to_image(scale, whitelist=OVERLAP_WHITELIST), (ox, label_h))
        kw = dict(fill=(220, 80, 80) if any_fail else (180, 180, 180))
        if lbl_font:
            kw["font"] = lbl_font
        cdraw.text((ox+2, 1), s["title"], **kw)

    out_path = os.path.join(script_dir, "sim_dashboard.png")
    canvas.save(out_path)
    print(f"Image saved: {out_path}  ({total_w}×{total_h} px)")
    if any_fail:
        print("Red pixels = overlapping zones.")

    sys.exit(1 if any_fail else 0)


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(script_dir)
    parser = argparse.ArgumentParser(
        description="128x64 OLED simulator and source-screen display checker"
    )
    parser.add_argument(
        "screens_dir",
        nargs="?",
        help="optional src/screens directory to compile and check",
    )
    parser.add_argument(
        "--source",
        action="store_true",
        help="compile real src/screens/*.c with stubs and check rendered output",
    )
    parser.add_argument("--output", help="PNG output path for source mode")
    parser.add_argument("--scale", type=int, default=4, help="PNG pixel scale")
    parser.add_argument("--columns", type=int, default=4, help="PNG grid columns")
    args = parser.parse_args()

    if args.source or args.screens_dir:
        sys.exit(main_source(args, repo_root))
    main_legacy()


if __name__ == "__main__":
    main()
