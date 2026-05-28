#!/usr/bin/env python3
"""
sim_dashboard.py — Pixel-accurate 128×64 dashboard simulator with overlap detection.

For each scenario, every draw call is attributed to a named zone.
If two zones write to the same pixel, it is recorded as an overlap.
Overlapping pixels are highlighted red in the output image.

Usage: python3 tools/sim_dashboard.py
Exit code 0 if no overlaps, 1 if any overlap found.
"""

import os
import sys
from collections import defaultdict
from PIL import Image, ImageDraw

# ── BDF font loader ──────────────────────────────────────────────────────────

def load_bdf(path):
    """Return dict: codepoint → list-of-7-ints (5 bits wide, MSB left)."""
    glyphs = {}
    with open(path) as f:
        lines = f.readlines()
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line.startswith("ENCODING"):
            cp = int(line.split()[1])
            while not lines[i].strip().startswith("BITMAP"):
                i += 1
            i += 1
            rows = []
            while not lines[i].strip().startswith("ENDCHAR"):
                byte = int(lines[i].strip(), 16)
                rows.append(byte >> 3)  # 5-wide: top 5 bits of 8
                i += 1
            glyphs[cp] = rows
        i += 1
    return glyphs


# ── Framebuffer with zone blame tracking ────────────────────────────────────

SCR_W = 128
SCR_H = 64

class Display:
    def __init__(self):
        self.buf   = [[0]    * SCR_W for _ in range(SCR_H)]
        self.blame = [[None] * SCR_W for _ in range(SCR_H)]  # zone per pixel
        self.color = 1
        self.font  = None
        self.font_ascent = 6
        self._zone = None
        # overlap records: {(zone_a, zone_b): [(x, y), ...]}
        self._overlaps = defaultdict(list)

    def clear_buffer(self):
        self.buf   = [[0]    * SCR_W for _ in range(SCR_H)]
        self.blame = [[None] * SCR_W for _ in range(SCR_H)]
        self._overlaps.clear()

    # ── Zone management ──────────────────────────────────────────────────────

    def zone(self, name):
        """Context manager: attribute all draws until next zone() call to `name`."""
        self._zone = name
        return self

    # ── Primitive draw ops ───────────────────────────────────────────────────

    def _px(self, x, y):
        if not (0 <= x < SCR_W and 0 <= y < SCR_H):
            return
        existing = self.blame[y][x]
        if existing is not None and existing != self._zone and self._zone is not None:
            key = tuple(sorted([existing, self._zone]))
            self._overlaps[key].append((x, y))
        self.buf[y][x] = self.color
        if self._zone is not None:
            self.blame[y][x] = self._zone

    def set_draw_color(self, c): self.color = c
    def set_font(self, glyphs, ascent=6):
        self.font = glyphs
        self.font_ascent = ascent

    def draw_pixel(self, x, y):      self._px(x, y)
    def draw_hline(self, x, y, w):   [self._px(x+i, y) for i in range(w)]
    def draw_vline(self, x, y, h):   [self._px(x, y+i) for i in range(h)]

    def draw_box(self, x, y, w, h):
        for dy in range(h):
            for dx in range(w):
                self._px(x+dx, y+dy)

    def draw_frame(self, x, y, w, h):
        self.draw_hline(x, y, w)
        self.draw_hline(x, y+h-1, w)
        self.draw_vline(x, y, h)
        self.draw_vline(x+w-1, y, h)

    def get_str_width(self, s):
        return sum(5 for c in s if ord(c) in (self.font or {}))

    def draw_str(self, x, y, s):
        """y is u8g2 baseline; glyph rows drawn from y-ascent upward."""
        cx = x
        for ch in s:
            rows = (self.font or {}).get(ord(ch))
            if rows is None:
                cx += 5
                continue
            top = y - self.font_ascent
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


def main():
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


if __name__ == "__main__":
    main()
