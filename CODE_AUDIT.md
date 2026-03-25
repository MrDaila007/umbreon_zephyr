# Полный аудит кода: Umbreon Zephyr

**Дата:** 2026-03-25
**Проект:** umbreon_zephyr (RP2350 / ARM Cortex-M33, Zephyr RTOS)
**Файлов проанализировано:** 14

---

## 1. Резюме

| Серьёзность | Количество | Описание |
|-------------|------------|----------|
| CRITICAL | 4 | Таймерные переполнения (IMU, тахометр), блокирующие маневры, переполнение track_learn |
| HIGH | 4 | Потокобезопасность cfg, PID clamp/feedforward, маскирование ускорения |
| MEDIUM | 6 | Heading unbounded, отсутствие валидации параметров, watchdog gap, volatile |
| LOW | 4 | Hardcoded π, некофигурируемые константы, проверки ошибок |

**Общая оценка:** Кодовая база чистая и хорошо структурированная. Критические баги связаны не с качеством кода, а с нюансами Zephyr API (`k_cycle_get_32()` переполнение) и архитектурными решениями (блокирующие маневры в control loop).

---

## 2. CRITICAL — Требуют исправления

### C-1. ~~`k_cycle_get_32()` переполнение в IMU (каждые ~30 сек)~~ [FIXED]

**Файл:** `src/imu.c:138`

```c
int64_t now = k_cyc_to_us_floor64(k_cycle_get_32());
float dt = (now - prev_us) / 1e6f;
```

**Проблема:** `k_cycle_get_32()` возвращает 32-битный аппаратный счётчик тактов. При частоте RP2350 125-150 МГц он переполняется каждые **~29-34 секунды**. После переполнения `k_cyc_to_us_floor64()` конвертирует обнулённый счётчик → `now` прыгает к ~0 → `dt = (0 - 28_600_000) / 1e6 = -28.6` → условие `dt > 0.0f` (стр. 142) отфильтровывает → **heading не обновляется 1 цикл каждые ~30 секунд**.

**Сценарий:** Машина в повороте, heading замирает на 40 мс → руль дёргается, wrong-direction detection может ложно сработать.

**Исправление:**
```c
// Заменить:
int64_t now = k_cyc_to_us_floor64(k_cycle_get_32());
// На:
int64_t now = k_uptime_ticks();
now = k_ticks_to_us_floor64(now);
```

---

### C-2. ~~`k_cycle_get_32()` переполнение в тахометре (каждые ~30 сек)~~ [FIXED]

**Файл:** `src/tachometer.c:30-33`

```c
static inline uint32_t get_us(void)
{
    return (uint32_t)(k_cyc_to_us_floor64(k_cycle_get_32()));
}
```

**Проблема:** Тот же баг. При переполнении `get_us()` прыгает к ~0. В ISR: `delta = now - last` становится огромным uint32_t → debounce пропускает, `taho_interval` получает мусорное значение. В `taho_get_speed()`: `elapsed` огромный → `elapsed > 500000` → скорость = 0.

**Сценарий:** Каждые ~30 секунд скорость прыгает в 0 на 1 цикл → PID-контроллер даёт резкий корректирующий импульс.

**Дополнительная проблема:** `taho_time_since_last_us()` (стр. 101-106) — та же математика. Используется для определения "колесо остановилось" (> 500 мс) → ложное срабатывание каждые 30 сек.

**Исправление:**
```c
static inline uint32_t get_us(void)
{
    return (uint32_t)(k_ticks_to_us_floor64(k_uptime_ticks()));
}
```
`k_uptime_ticks()` использует 64-битный системный таймер — не переполняется. Cast в uint32_t переполняется через ~71 минуту, но `uint32_t` вычитание корректно обрабатывает wraparound (`now - last` даёт правильную разницу даже при переполнении, т.к. оба uint32_t).

---

### C-3. Блокирующие маневры в control loop (до 4 секунд)

**Файл:** `src/control.c:52-88, 208-226`

**`go_back()` (стр. 52-69):**
```c
static void go_back(void)
{
    car_write_speed(0);
    int64_t deadline = k_uptime_get() + 2000;
    while (taho_get_speed() > 0.1f && k_uptime_get() < deadline) {
        wdt_feed_kick();
        k_msleep(10);       // блокирует до 2000 мс
    }
    car_write_speed(-150);
    k_msleep(200);           // ещё 200 мс
    car_write_speed(0);
    k_msleep(80);            // ещё 80 мс
    car_write_speed(-150);
    k_msleep(700);           // ещё 700 мс
    car_write_speed(0);
}
```

**Общее время блокировки:**
- `go_back()`: до ~3 сек
- `go_back_long()`: до ~5 сек
- Wrong-direction recovery (стр. 208-226): ещё ~1 сек внутренний цикл + вызов go_back_long()

**Последствия во время блокировки:**
- Датчики не опрашиваются (кроме внутреннего цикла wrong-dir)
- Телеметрия не отправляется
- Команды $STOP не обрабатываются (wifi_cmd_thread работает, но control_thread не проверяет car_running)
- IMU heading не обновляется → drift

**Смягчающие факторы:**
- Watchdog кормится внутри go_back() (wdt_feed_kick) — reset не произойдёт
- Это сознательное решение (порт из Arduino), и работает на практике
- Альтернатива (state machine) сильно усложнит код

**Рекомендация:** Задокументировать как known limitation. Если нужна возможность прервать маневр по $STOP — потребуется state machine рефактор.

---

### C-4. track_learn.c:99 — uint16_t переполнение lap_dist_cm

**Файл:** `src/track_learn.c:99`

```c
trk_hdr.lap_dist_cm = (uint16_t)(trk_get_odo_m() * 100);
```

**Проблема:** `uint16_t` max = 65535. Трассы длиннее 655.35 м → переполнение → `lap_dist_cm` = мусор.

**Каскадный эффект (стр. 135-138):**
```c
float lap_m = trk_hdr.lap_dist_cm / 100.0f;  // → 0 или мусор
if (lap_m <= 0.0f) return -1.0f;              // спасает при 0
float query_m = fmodf(trk_odo_m + lookahead_m, lap_m);  // fmodf(x, мусор) → мусор
```

**Реальный риск:** Низкий для текущих трасс (indoor, < 50 м). Но если проект масштабируется на outdoor трассы — race mode сломается.

**Исправление:** Изменить `uint16_t lap_dist_cm` на `uint32_t` в `struct track_header`. Это изменит layout NVS → нужна миграция или bump magic number.

---

## 3. HIGH — Рекомендуется исправить

### H-1. Глобальная `cfg` без синхронизации между потоками

**Файл:** `src/settings.h:63`

```c
extern struct car_settings cfg;
```

**Потоки-писатели:** wifi_cmd_thread (через `parse_set_pair()`)
**Потоки-читатели:** control_thread (PID, thresholds, loop timing), battery_thread (bat_enabled)

**Анализ:**
- ARM Cortex-M33: word-aligned reads/writes атомарны → отдельные `int` и `float` поля безопасны
- **Проблема:** чтение нескольких связанных полей НЕ атомарно. Пример:
  - Пользователь отправляет `$SET:KP=80,KI=60,KD=10`
  - parse_set_pair() обновляет KP, затем KI, затем KD (последовательно, стр. 228-230)
  - control_thread между записью KI и KD читает: `KP=80(new), KI=60(new), KD=6(old)` — inconsistent PID tuning
- `$SET:LMS=0` → cfg.loop_ms = 0 → control loop делит на 0 в car_pid_control (через dt)

**Рекомендация:**
- Минимально: добавить валидацию `cfg.loop_ms >= 10` и `cfg.encoder_holes >= 1` в parse_set_pair()
- Оптимально: snapshot `cfg` в локальную копию в начале work() — дешёво, надёжно

---

### H-2. PID clamp запрещает reverse

**Файл:** `src/car.c:185`

```c
esc_val = CLAMP(esc_val, NEUTRAL_SPEED, cfg.max_speed);
```

**Проблема:** При `target_speed < 0` PID вычисляет отрицательный `output`, `esc_val < 1500`, но clamp зажимает в 1500 → мотор не крутится назад через PID.

**Текущая ситуация:** `car_write_speed_ms()` вызывается только с положительным target (cfg.spd_clear, cfg.spd_blocked). Reverse используется только через `car_write_speed()` (прямое управление, обходит PID). Поэтому баг **сейчас не проявляется**, но API вводит в заблуждение.

**Рекомендация:** Документировать что PID — only forward, или добавить reverse clamp:
```c
if (target_speed < -0.01f) {
    esc_val = CLAMP(esc_val, cfg.min_bspeed, NEUTRAL_SPEED);
} else {
    esc_val = CLAMP(esc_val, NEUTRAL_SPEED, cfg.max_speed);
}
```

---

### H-3. Feedforward только для forward

**Файл:** `src/car.c:181`

```c
float ff = (target_speed > 0.01f) ? (float)(cfg.min_speed - NEUTRAL_SPEED) : 0;
```

**Проблема:** Reverse target не получает feedforward компенсацию мёртвой зоны ESC.

**Реальный риск:** Минимальный — reverse через PID не используется (см. H-2). Но если PID reverse будет реализован, feedforward нужен:
```c
float ff;
if (target_speed > 0.01f)
    ff = (float)(cfg.min_speed - NEUTRAL_SPEED);
else if (target_speed < -0.01f)
    ff = (float)(cfg.min_bspeed - NEUTRAL_SPEED);
else
    ff = 0;
```

---

### H-4. Тахометр маскирует ускорение

**Файл:** `src/tachometer.c:84-86`

```c
uint32_t elapsed = now - last;
if (elapsed < iv) {
    elapsed = iv;  // используем старый (больший) интервал
}
```

**Проблема:** При ускорении колеса (`elapsed < iv`) текущий более короткий интервал заменяется старым, более длинным. Скорость вычисляется как `distance / elapsed`, и с бОльшим elapsed скорость кажется **ниже реальной**.

**Сценарий:** Машина резко разгоняется после маневра. PID видит скорость ниже реальной → добавляет газ → overshoot.

**Логика авторского решения:** Вероятно, для сглаживания шума энкодера. Но лучше использовать EMA или min(elapsed, iv).

**Рекомендация:** Оценить практически — если overshoot не наблюдается, оставить как есть.

---

## 4. MEDIUM — Улучшения

### M-1. ~~heading не нормализуется (imu.c:142-144)~~ [FIXED]

```c
heading += yaw_rate * dt;
```

Heading растёт неограниченно. После 10 кругов = 3600°. `turns` в control.c (стр. 196) тоже растёт, но clamp к ±200° (стр. 203) ограничивает. heading используется только в телеметрии, не в управлении → **низкий риск**, но float теряет точность при больших значениях.

**Fix:** `heading = fmodf(heading, 360.0f)` или нормализация к [-180, 180).

---

### M-2. ~~parse_set_pair() — нет валидации диапазонов (wifi_cmd.c:224-250)~~ [FIXED]

Критические примеры:
- `$SET:LMS=0` → busy-wait (k_msleep(0) = yield, 100% CPU), turns перестаёт интегрироваться
- `$SET:LMS=-1` → отрицательный sleep → undefined behavior
- `$SET:ENH=0` → division by zero в car_pid_control() (защищено проверкой, OK)
- `$SET:MNP=500,XNP=10` → min_point > max_point, стеринг инвертируется

**Fix:** Добавлена CLAMP/MAX валидация для всех критичных полей: LMS>=10, ENH>=1, дистанции, ESC 1000-2000, сервоуглы 0-180.

---

### M-3. ~~bat_voltage не volatile (battery.c)~~ [FIXED]

```c
static float bat_voltage;  // пишется battery_thread, читается control_thread
```

На ARM Cortex-M 4-байтный float read/write атомарный. Но без `volatile` компилятор может кешировать значение в регистре → control_thread видит stale voltage.

**Fix:** `static volatile float bat_voltage;`

---

### M-4. ~~Watchdog не защищает инициализацию (main.c:96-123)~~ [FIXED]

Последовательность до `wdt_init()`:
1. `settings_init()` + `settings_load()` — NVS read (~10 мс)
2. `sensors_init()` — I2C probe 6 датчиков (~500 мс)
3. `imu_calibrate()` — 200 × 5мс = **~1 сек**
4. `k_msleep(3700)` — ESC arming = **3.7 сек**
5. ИЛИ `car_run_calibration()` — до **7 сек**

**Итого:** 5-8 сек без watchdog. Если I2C зависнет — hard lock.

**Fix:** Переместить `wdt_init()` раньше в main(), кормить watchdog в длительных операциях.

---

### M-5. ~~sensors.c:98 — hardcoded magic numbers~~ [FIXED]

```c
if (mm >= 8190 || mm <= 0) {
    distances[i] = MAX_SENSOR_RANGE;
```

8190 — max raw value для VL53L0X. Должно быть `#define VL53L0X_MAX_RAW 8190`.

---

### M-6. ~~track_learn — race condition на trk_mode~~ [FIXED]

`trk_mode` пишется из wifi_cmd_thread (`track_learn_dispatch`), читается из control_thread (`track_learn_tick`, `track_learn_recommend_speed`). `volatile` нет, mutex нет.

На ARM Cortex-M enum read/write атомарный, но компилятор может кешировать. Реальный риск — минимальный (mode transitions редкие), но формально — data race.

**Fix:** `static volatile enum trk_mode_t trk_mode;`

---

## 5. LOW — Мелкие замечания

### L-1. ~~Hardcoded π~~ [FIXED]

**Файлы:** `car.c:164`, `imu.c:88,123`, `tachometer.c:92`, `track_learn.c:66`, `tests.c:224`

Заменено `3.14159265f` → `(float)M_PI` во всех 6 вхождениях (5 файлов).

---

### L-2. IMU EMA alpha не конфигурируемый

**Файл:** `imu.c:20`

```c
#define IMU_EMA_ALPHA  0.3f
```

В отличие от PID gains (конфигурируемые через $SET), IMU фильтр фиксирован. Для разных скоростей может потребоваться разный alpha.

---

### L-3. sensors.c — online_count не обновляется в runtime

**Файл:** `src/sensors.c:56-66`

`online_count` считается при init и никогда не обновляется. Если датчик отключится в runtime — `sensors_online_count()` вернёт старое значение.

---

### L-4. strtof()/atoi() не проверяют ошибки

**Файл:** `src/wifi_cmd.c:228-250`

`strtof("abc", NULL)` → 0.0, `atoi("abc")` → 0. Ошибка не сообщается. Пользователь не знает что параметр не применился.

---

## 6. Архитектурные наблюдения (не баги)

### A-1. go_back() — блокирующий vs state-machine

Текущий подход (блокирующий) портирован из Arduino и работает. State-machine подход:
- (+) Не блокирует control loop
- (+) Позволяет прервать маневр по $STOP
- (-) Значительно усложняет код (5-6 состояний)
- (-) Нужно тестировать transitions

Рекомендация: оставить как есть, если нет проблем на практике.

### A-2. Потокобезопасность на ARM Cortex-M

Многие "race conditions" из классического анализа потокобезопасности **безопасны на ARM Cortex-M33**:
- Word-aligned reads/writes атомарны
- `volatile bool`, `volatile int`, `volatile float` — torn read невозможен
- Проблема только при чтении **группы** связанных полей (cfg.KP + cfg.KI + cfg.KD)

### A-3. Стеки и heap

| Ресурс | Размер | Используется | Запас |
|--------|--------|-------------|-------|
| control stack | 4096 | ~2500 (est.) | ~40% |
| wifi_cmd stack | 2048 | ~1500 (est.) | ~25% |
| main stack | 4096 | ~3000 (est.) | ~25% |
| battery stack | 1024 | ~400 (est.) | ~60% |
| heap | 4096 | ~2000 (est.) | ~50% |
| **RAM total** | 34048 / 520K | | ~6.5% |

Рекомендация: включить `CONFIG_STACK_SENTINEL=y` в prj.conf для runtime-детекции stack overflow.

---

## 7. Приоритет исправления

### ~~Немедленно~~ [DONE]:
1. **C-1** — ~~imu.c: заменить `k_cycle_get_32()` на `k_uptime_ticks()`~~ [FIXED]
2. **C-2** — ~~tachometer.c: заменить `k_cycle_get_32()` на `k_uptime_ticks()`~~ [FIXED]

### ~~Скоро~~ [DONE]:
3. **M-2** — ~~wifi_cmd.c: валидация `LMS >= 10`, `ENH >= 1` в parse_set_pair()~~ [FIXED]
4. **M-3** — ~~battery.c: добавить `volatile`~~ [FIXED]
5. **M-6** — ~~track_learn.c: добавить `volatile` на trk_mode~~ [FIXED]

### ~~При возможности~~ [PARTIALLY DONE]:
6. **H-1** — cfg snapshot в начале work()
7. **M-4** — ~~watchdog раньше в main()~~ [FIXED]
8. **C-4** — track_learn: uint32_t lap_dist_cm (если планируются outdoor трассы)
9. **H-2/H-3** — PID reverse support (если потребуется)
10. **H-4** — тахометр: оценить ускорение vs сглаживание

### ~~Опционально~~ [PARTIALLY DONE]:
11. **L-1** — ~~hardcoded π~~ [FIXED], **M-1** — ~~heading normalization~~ [FIXED], **M-5** — ~~magic 8190~~ [FIXED]
12. **L-2..L-4** — остальные code quality improvements
13. `CONFIG_STACK_SENTINEL=y` в prj.conf
14. `CONFIG_ASSERT=y` для debug builds

---

## 8. Файлы проанализированы

| Файл | Строк | Критичность | Найдено проблем |
|------|-------|-------------|-----------------|
| src/control.c | 365 | Критический путь | C-3 |
| src/car.c | ~220 | Критический путь | H-2, H-3 |
| src/tachometer.c | 114 | ISR + timing | C-2, H-4 |
| src/imu.c | 168 | Heading/gyro | C-1, M-1 |
| src/sensors.c | ~110 | I2C sensors | M-5, L-3 |
| src/wifi_cmd.c | 484 | Command protocol | M-2, L-4 |
| src/settings.c/h | ~270 | NVS + config | H-1 |
| src/battery.c | ~120 | ADC monitoring | M-3 |
| src/main.c | 141 | Init + watchdog | M-4 |
| src/track_learn.c | ~310 | Track learning | C-4, M-6 |
| src/tests.c | ~360 | Test harness | (через wifi_cmd) |
| prj.conf | 60 | Build config | A-3 |
| CMakeLists.txt | 18 | Build system | — |
| boards/*.overlay | ~190 | Device tree | — |
