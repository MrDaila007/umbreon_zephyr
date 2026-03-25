# TODO — Umbreon Zephyr

## Not yet ported from Arduino

### Major

- [ ] **OLED menu + rotary encoder** — 10-screen state machine (dashboard, settings, tests, info) on SSD1306 128×64. Arduino: `menu.h` (943 lines). No display driver or encoder handling in Zephyr yet
- [ ] **Dual-core (Core 1)** — Arduino uses Core 1 for OLED/encoder (`core1.h`, `setup1()`/`loop1()`). Zephyr runs everything as threads on a single core, second RP2350 core is idle
- [ ] **TF-Luna 4× LiDAR** — 4 UART lidars via SerialPIO (`luna_car.h`). Zephyr only supports 6× VL53L0X, no flexible sensor selection

### Minor

- [ ] **Compile-time sensor config** — Arduino: `sensor_config.h` allows switching between 4×Luna and 6×VL53L0X via `SENSOR_CONFIG`. Zephyr hardcodes 6× VL53L0X
- [ ] **Competition mode** — Auto-start flag without menu/WiFi for races
- [ ] **I2C bus recovery (bit-bang)** — Manual I2C1 bus recovery via SCL toggling. Zephyr uses `i2c_recover_bus()` (less reliable)
- [ ] **Battery sustained low-voltage cutoff** — Arduino cuts motor after 10 seconds of sustained low voltage. Zephyr only does a one-shot check

---

## Code quality (from CODE_AUDIT.md)

### Open

- [ ] **C-3** — Блокирующие маневры go_back()/go_back_long() до 5 сек. Рефактор в state machine если понадобится прерывание по $STOP
- [ ] **C-4** — `uint16_t lap_dist_cm` переполнение для трасс > 655 м. Заменить на `uint32_t` (меняет NVS layout, нужна миграция)
- [ ] **H-1** — `cfg` struct без синхронизации между потоками. Snapshot в начале work() для consistency
- [ ] **H-2/H-3** — PID clamp/feedforward только для forward. Добавить reverse support если потребуется
- [ ] **H-4** — Тахометр маскирует ускорение (`elapsed = max(elapsed, iv)`). Оценить практически, рассмотреть EMA
- [ ] **L-2** — IMU EMA alpha (0.3) не конфигурируемый через $SET
- [ ] **L-3** — `sensors_online_count()` не обновляется в runtime при отключении датчика
- [ ] **L-4** — strtof()/atoi() не сообщают об ошибках парсинга пользователю

### Fixed (2026-03-25)

- [x] **C-1** — `k_cycle_get_32()` overflow в IMU → `k_uptime_ticks()`
- [x] **C-2** — `k_cycle_get_32()` overflow в тахометре → cycle-domain subtraction
- [x] **M-1** — Heading не нормализуется → `fmodf(heading, 360.0f)`
- [x] **M-2** — `parse_set_pair()` без валидации → CLAMP/MAX на все критичные поля
- [x] **M-3** — `bat_voltage` не volatile → добавлен `volatile`
- [x] **M-4** — Watchdog после длительной init → перемещён раньше + feed-ы
- [x] **M-5** — Hardcoded 8190 → `#define VL53L0X_MAX_RAW`
- [x] **M-6** — `trk_mode` не volatile → добавлен `volatile`
- [x] **L-1** — Hardcoded π → `(float)M_PI`

---

## Testing

- [ ] **Миграция на Zephyr ztest + native_sim** — перевести host-тесты (`tests/test_unit.c`) на ztest framework. Потребуется: overlay для native_sim, отдельный CMakeLists, testcase.yaml. Позволит тестировать с реальными Zephyr API
- [ ] **Тесты track_learn binary search** — `track_learn_recommend_speed()` с синтетическими track_point массивами
- [ ] **Тесты PID math** — извлечь PID логику в отдельную функцию, тестировать gain tuning
- [ ] **Тесты steering policy** — sensor fusion логика из `work()` с параметрическими входами
