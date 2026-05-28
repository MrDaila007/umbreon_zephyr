# WiFi Status Strip on Dashboard — Design Spec

**Date:** 2026-05-27  
**Branch:** feature/zephyr-v4.4

---

## Goal

Add a WiFi signal icon and connection status (mode + RSSI) to the bottom of the dashboard screen. The strip is shown only when WiFi status data has been received from the ESP module.

---

## Context

The ESP module (`umbreon_esp_web`) responds to `#WIFISTATUS\n` with:
```
# Mode:  STA\r\n
# SSID:  <ssid>\r\n
# IP:    <ip>\r\n
# RSSI:  <rssi_dBm>\r\n
# Status: ready\r\n
```
Or in AP mode (no RSSI line):
```
# Mode:  AP\r\n
# SSID:  <ssid>\r\n
# IP:    <ip>\r\n
# Status: ready\r\n
```

The Pico already sends `#WIFISTATUS\n` once at startup (`wifi_cmd.c:776`) but never parses the response. The `wifi_cmd_thread` uses `k_poll` with `K_FOREVER` — changing to a 10s timeout enables periodic polling.

---

## Layout

Dashboard is 128×64 pixels, currently:
```
y=0..8   Status bar (battery, mode)
y=9..33  Sensor bars (6× VL53L0X)
y=34..63 IMU scale + numeric value
```

After this change:
```
y=0..8   Status bar            (unchanged)
y=9..33  Sensor bars           (unchanged)
y=34..55 IMU scale + numeric   (numeric font: 9×15 → 5×7, moved to y=53)
y=56     Separator line
y=57..63 WiFi strip            (new)
```

The IMU numeric value moves from baseline y=62 (9×15 font) to y=53 (5×7 font). The scale, ticks, and marker are unchanged.

---

## WiFi Strip Content (y=57..63)

```
[icon 10px] [mode 3ch] [space] [rssi text]
```

Example renders:
- STA connected: `▁▂▄█ STA -65dBm`
- AP mode:       `████ AP`
- Not ready:     (strip not drawn — no separator, no content)

### Signal icon

4 vertical bars at x=2,5,8,11 with heights 2,3,5,7 px, anchored at y=63 (bottom).  
Bars are filled (DrawBox) up to the level indicated by RSSI; unfilled bars drawn as single-pixel outline (DrawPixel at top).

| RSSI        | Filled bars |
|-------------|-------------|
| ≥ −60 dBm   | 4           |
| ≥ −70 dBm   | 3           |
| ≥ −80 dBm   | 2           |
| ≥ −90 dBm   | 1           |
| AP mode     | 4 (always)  |
| not ready   | strip hidden |

### Text

Font: `u8g2_font_5x7_tr`, baseline y=63.  
- STA: `"STA %d"` where `%d` is RSSI in dBm (e.g. `"STA -65"`)  
- AP:  `"AP"`

---

## Data Flow

```
[ESP UART1] --#WIFISTATUS reply--> [wifi_cmd UART RX handler]
                                        |
                                   parse_wifi_status_line()
                                        |
                                   ws_ready / ws_is_ap / ws_rssi
                                        |
                              wifi_status_is_ready()
                              wifi_status_is_ap()
                              wifi_status_get_rssi()
                                        |
                              screen_dashboard.c → draw_wifi_strip()
```

### Periodic re-poll

In `wifi_cmd_thread`, change `k_poll(..., K_FOREVER)` to `k_poll(..., K_SECONDS(10))`.  
On timeout: send `#WIFISTATUS\n`. This keeps the display up-to-date after reconnects.

---

## Files Changed

| File | Change |
|------|--------|
| `src/wifi_cmd.c` | Add `ws_ready`, `ws_is_ap`, `ws_rssi` statics; add `parse_wifi_status_line()`; call it in `handle_line()` for `# `-prefixed lines; change k_poll timeout to 10s |
| `src/wifi_cmd.h` | Add `wifi_status_is_ready()`, `wifi_status_is_ap()`, `wifi_status_get_rssi()` |
| `src/screens/screen_dashboard.c` | Change IMU numeric font 9×15→5×7 and adjust y; add `draw_wifi_strip()`; include `../wifi_cmd.h`; draw separator + strip in `screen_dashboard_draw()` |

No new files. No changes to other screens or modules.

---

## Error Handling

- If `ws_ready` is false (no status received yet), the WiFi strip is not drawn and no separator line is added — layout looks identical to current.
- `parse_wifi_status_line()` uses `sscanf`/`strncmp` with bounds; malformed lines are silently ignored.
- RSSI=0 from `wifi_manager_get_rssi()` means "not in STA mode" — treated as AP (4 bars).

---

## Out of Scope

- Displaying IP address or SSID on the dashboard (too long for 7px strip).
- Showing client count in AP mode.
- WiFi configuration from the display menu.
