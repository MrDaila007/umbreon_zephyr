# Аудит логирования: влияние на рантайм Umbreon Zephyr

**Дата:** 2026-03-25
**Проект:** umbreon_zephyr (RP2350, Zephyr RTOS)
**Цель:** Оценить влияние логирования и телеметрии на рантайм автономной машины

---

## 1. Резюме

| Аспект | Статус | Влияние на рантайм |
|--------|--------|--------------------|
| Zephyr LOG_* макросы | OK | Нулевое — только в init и ошибках |
| ISR (tachometer) | OK | Нулевое — только атомарные операции |
| sensors_poll() / imu_update() | OK | Нулевое — без LOG вызовов |
| car_pid_control() | OK | Нулевое — без LOG вызовов |
| **wifi_cmd_printf() в control loop** | **ПРОБЛЕМА** | **~6 мс из 40 мс бюджета (15%)** |

**Главный вывод:** Zephyr-логирование безопасно и не влияет на контрольный цикл. Однако **телеметрия через `wifi_cmd_printf()` блокирует control thread** на ~15% каждого цикла из-за побайтового блокирующего UART TX.

---

## 2. Архитектура потоков

| Поток | Приоритет | Стек | Период | Назначение |
|-------|-----------|------|--------|------------|
| control | 2 (высший) | 4096 | 40 мс | Датчики → PID → руль/газ → телеметрия |
| wifi_cmd | 5 (средний) | 2048 | Event-driven | Парсинг команд, ответы |
| battery | 10 (низший) | 1024 | 500 мс | ADC батареи |
| main | default | 4096 | K_FOREVER | Init → sleep |

**Zephyr logging thread:** Используется deferred mode (дефолт), LOG_* не блокируют вызывающий поток. Бэкенд — UART0 (GP16/GP17, 115200).

**Телеметрия:** Отдельный канал — UART1 (GP4/GP5, 115200), НЕ через Zephyr logging.

---

## 3. Конфигурация логирования

**Файл:** `prj.conf`

```
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3          # INFO
CONFIG_LOG_BACKEND_UART=y           # → UART0
CONFIG_PRINTK=y
CONFIG_UART_CONSOLE=y
CONFIG_UART_INTERRUPT_DRIVEN=y      # Включён, но TX не использует!
```

Ключевой момент: `CONFIG_UART_INTERRUPT_DRIVEN=y` включает поддержку interrupt-driven UART, но `wifi_cmd_send()` использует `uart_poll_out()` (polling), игнорируя эту возможность.

---

## 4. Zephyr LOG_* — полная таблица

### 4.1. Зарегистрированные модули

Все 11 исходных файлов регистрируют LOG-модуль на уровне `LOG_LEVEL_INF`:

| Файл | Модуль |
|------|--------|
| main.c | main |
| car.c | car |
| tachometer.c | tachometer |
| sensors.c | sensors |
| imu.c | imu |
| battery.c | battery |
| settings.c | settings |
| wifi_cmd.c | wifi_cmd |
| control.c | control |
| track_learn.c | track_learn |
| tests.c | tests |

### 4.2. Все LOG_* вызовы по файлам

#### main.c — 3 вызова (+ 4 printk)

| Строка | Уровень | Контекст | Вызов |
|--------|---------|----------|-------|
| 37 | WRN | init | `LOG_WRN("Watchdog not available")` |
| 51 | ERR | init | `LOG_ERR("Watchdog install failed: %d")` |
| 58 | ERR | init | `LOG_ERR("Watchdog setup failed: %d")` |
| 90-93 | — | init | `printk()` — стартовый баннер |

#### car.c — 7 вызовов

| Строка | Уровень | Контекст | Вызов |
|--------|---------|----------|-------|
| 56 | ERR | init | `LOG_ERR("Servo PWM not ready")` |
| 59 | ERR | init | `LOG_ERR("ESC PWM not ready")` |
| 66 | INF | init | `LOG_INF("Car PWM init done")` |
| 197 | INF | calibration | `LOG_INF("ESC calibration: max signal")` |
| 203 | INF | calibration | `LOG_INF("ESC calibration: min signal")` |
| 209 | INF | calibration | `LOG_INF("ESC calibration: neutral")` |
| 220 | INF | calibration | `LOG_INF("ESC calibration complete")` |

#### tachometer.c — 2 вызова

| Строка | Уровень | Контекст | Вызов |
|--------|---------|----------|-------|
| 62 | ERR | init | `LOG_ERR("Tachometer GPIO not ready")` |
| 71 | INF | init | `LOG_INF("Tachometer init (GP13, RISING edge)")` |

**ISR `tach_isr()` (стр. 37-55): 0 LOG вызовов** — только `atomic_inc()`, `atomic_set()`.

#### sensors.c — 5 вызовов

| Строка | Уровень | Контекст | Вызов |
|--------|---------|----------|-------|
| 40 | INF | init | `LOG_INF("I2C1 bus recovery OK")` |
| 42 | DBG | init | `LOG_DBG("I2C1 bus recovery not supported")` |
| 44 | WRN | init | `LOG_WRN("I2C1 bus recovery failed")` |
| 64 | WRN | init (цикл 0-5) | `LOG_WRN("VL53L0X[%d] not ready")` |
| 68 | INF | init | `LOG_INF("VL53L0X: %d/%d online")` |

**`sensors_poll()` (стр. 74-107): 0 LOG вызовов.**

#### imu.c — 5 вызовов

| Строка | Уровень | Контекст | Вызов |
|--------|---------|----------|-------|
| 39 | WRN | init | `LOG_WRN("MPU-6050 not ready")` |
| 49 | WRN | init | `LOG_WRN("MPU-6050 gyro FS set failed")` |
| 54 | INF | init | `LOG_INF("MPU-6050 init OK")` |
| 66 | INF | calibration | `LOG_INF("IMU: calibrating gyro bias")` |
| 99 | INF | calibration | `LOG_INF("IMU: bias = %.3f deg/s")` |

**`imu_update()` (стр. 105-145): 0 LOG вызовов.**

#### battery.c — 3 вызова

| Строка | Уровень | Контекст | Вызов |
|--------|---------|----------|-------|
| 92 | WRN | init | `LOG_WRN("ADC not ready")` |
| 99 | ERR | init | `LOG_ERR("ADC channel setup failed")` |
| 110 | INF | init | `LOG_INF("Battery ADC init")` |

**`battery_thread()` (стр. 48-84): 0 LOG вызовов.**

#### settings.c — 8 вызовов

| Строка | Уровень | Контекст | Вызов |
|--------|---------|----------|-------|
| 198 | ERR | init | `LOG_ERR("Flash area open failed")` |
| 210 | ERR | init | `LOG_ERR("NVS mount failed")` |
| 215 | INF | init | `LOG_INF("NVS ready")` |
| 227 | INF | load | `LOG_INF("No saved settings")` |
| 232 | WRN | load | `LOG_WRN("Settings magic/version mismatch")` |
| 237 | WRN | load | `LOG_WRN("Settings checksum mismatch")` |
| 242 | INF | load | `LOG_INF("Settings loaded from NVS")` |
| 257 | ERR | save (on-demand) | `LOG_ERR("NVS write failed")` |

#### wifi_cmd.c — 3 вызова

| Строка | Уровень | Контекст | Вызов |
|--------|---------|----------|-------|
| 433 | INF | thread start | `LOG_INF("WiFi command thread started")` |
| 469 | ERR | init | `LOG_ERR("UART1 not ready")` |
| 482 | INF | init | `LOG_INF("WiFi CMD init")` |

#### control.c — 2 вызова

| Строка | Уровень | Контекст | Вызов |
|--------|---------|----------|-------|
| 237 | INF | thread start | `LOG_INF("Control thread started")` |
| 307 | INF | init | `LOG_INF("Control thread created")` |

**`work()` (стр. 116-227): 0 LOG вызовов.**
**`control_thread()` loop: 0 LOG вызовов.**

#### track_learn.c — 4 вызова

| Строка | Уровень | Контекст | Вызов |
|--------|---------|----------|-------|
| 186 | ERR | save (on-demand) | `LOG_ERR("Track header NVS write failed")` |
| 193 | ERR | save (on-demand) | `LOG_ERR("Track data NVS write failed")` |
| 197 | INF | save (on-demand) | `LOG_INF("Track saved")` |
| 221 | INF | load (on-demand) | `LOG_INF("Track loaded")` |

#### tests.c — 0 LOG вызовов

Модуль зарегистрирован, но все вывод идёт через `wifi_cmd_printf()`.

### 4.3. Итог по LOG_*

| Уровень | Количество | Контекст | Влияние на рантайм |
|---------|------------|----------|-------------------|
| LOG_ERR | 8 | init / ошибки | Нулевое |
| LOG_INF | 16 | init / on-demand | Нулевое |
| LOG_WRN | 6 | init / ошибки | Нулевое |
| LOG_DBG | 1 | init (фильтруется) | Нулевое |
| **В control loop** | **0** | — | — |
| **В ISR** | **0** | — | — |

**Вердикт: Zephyr LOG_* не влияют на рантайм.** Все вызовы в init-коде или error-обработчиках. В критических путях (control loop, ISR, sensor poll, IMU, PID) — ноль вызовов.

---

## 5. Телеметрия (wifi_cmd_printf) — главная проблема

### 5.1. Механизм передачи

```c
// wifi_cmd.c:68-88
void wifi_cmd_send(const char *str)
{
    k_mutex_lock(&tx_mutex, K_FOREVER);
    for (const char *p = str; *p; p++) {
        uart_poll_out(uart_dev, *p);    // БЛОКИРУЮЩИЙ вызов
    }
    k_mutex_unlock(&tx_mutex);
}

void wifi_cmd_printf(const char *fmt, ...)
{
    char buf[256];
    vsnprintf(buf, sizeof(buf), fmt, args);  // форматирование на стеке
    wifi_cmd_send(buf);                       // блокирующая отправка
}
```

### 5.2. Вызов из control loop

```c
// control.c:92-101
static void send_telemetry(int *s, int steer_val, float spd_target)
{
    wifi_cmd_printf("%lld,%d,%d,%d,%d,%d,%d,%d,%.2f,%.1f,%.1f,%.1f\n",
        k_uptime_get(),
        s[0], s[1], s[2], s[3], s[4], s[5],
        steer_val,
        (double)taho_get_speed(),
        (double)spd_target,
        (double)imu_get_yaw_rate(),
        (double)imu_get_heading());
}
```

Вызывается **каждые 40 мс** из `control_thread` (приоритет 2).

### 5.3. Расчёт блокировки

| Параметр | Значение |
|----------|----------|
| Baud rate | 115200 бит/с |
| Время на байт | ~87 мкс (1 start + 8 data + 1 stop = 10 бит) |
| Типичная телеметрия | ~70 символов |
| **Время блокировки** | **~6.1 мс** |
| Бюджет цикла | 40 мс |
| **Доля бюджета** | **~15%** |
| Частота | 25 Гц |

### 5.4. Дополнительные факторы

- `vsnprintf` с 4 float → дополнительно ~0.5-1 мс CPU (newlib + soft float formatting)
- `k_mutex_lock(&tx_mutex, K_FOREVER)` — control thread (prio 2) ждёт, если wifi_cmd_thread (prio 5) держит мьютекс (приоритетная инверсия невозможна — control выше, но задержка от wifi TX — возможна при ответах на команды)
- `send_idle_telemetry()` — вызывается и в idle-режиме, та же блокировка

### 5.5. Другие вызовы wifi_cmd_printf

| Место | Частота | Влияние |
|-------|---------|---------|
| control.c: send_telemetry | 25 Гц (каждые 40 мс) | **15% бюджета** |
| control.c: send_idle_telemetry | 25 Гц (idle) | **15% бюджета** |
| track_learn.c: trk_send_data | On-demand (до 900 итераций) | Безопасно (не в loop) |
| tests.c: тестовые функции | Тестовый режим | Безопасно |
| wifi_cmd.c: ответы на команды | On-demand | Безопасно |

---

## 6. Анализ: стоит ли делать телеметрию наименее приоритетной?

### Текущая ситуация

Телеметрия (`send_telemetry()`) выполняется **внутри** `control_thread` с приоритетом 2. Это означает:
- Она не может быть вытеснена другими задачами
- Она блокирует PID-обновление на время UART TX
- Снижение приоритета невозможно без выноса в отдельный поток

### Ответ: да, телеметрию нужно отвязать от control loop

Телеметрия — это **наблюдение**, а не **управление**. Потеря или задержка телеметрии на 1-2 цикла не влияет на поведение машины. Задержка PID-обновления на 6 мс — влияет.

---

## 7. Рекомендации

### 7.1. [КРИТИЧНО] Неблокирующий UART TX

**Проблема:** `uart_poll_out()` блокирует control thread.

**Решение:** Использовать interrupt-driven TX (`uart_fifo_fill()`). Конфиг `CONFIG_UART_INTERRUPT_DRIVEN=y` уже включён, но не используется для передачи.

Схема:
```
control_thread                   UART1 IRQ
    │                               │
    ├─ sprintf → ring buffer        │
    │  (неблокирующий, ~1 мс)       │
    │                               │
    └─ продолжает PID               ├─ uart_fifo_fill() из ring buffer
                                    └─ (асинхронно, ~6 мс)
```

**Выигрыш:** ~5-6 мс на каждый цикл (с ~6 мс до ~1 мс).

### 7.2. [РЕКОМЕНДУЕТСЯ] Вынести телеметрию в отдельный поток

Вместо вызова `send_telemetry()` из `control_thread`:
1. Control thread пишет данные в shared-структуру (lock-free или с кратким mutex)
2. Отдельный telemetry thread (приоритет 7-8) читает и отправляет

**Выигрыш:** Control loop полностью свободен от UART-блокировок. Телеметрия может пропускать кадры при перегрузке.

### 7.3. [ОПЦИОНАЛЬНО] Снизить частоту телеметрии

Отправлять телеметрию каждые 2-4 цикла (10-12 Гц вместо 25 Гц). Для визуализации и отладки 10 Гц достаточно.

```c
static int telem_div = 0;
if (++telem_div >= cfg.telem_divider) {  // telem_divider = 2..4
    telem_div = 0;
    send_telemetry(...);
}
```

**Выигрыш:** Пропорциональное снижение нагрузки (при div=2 — с 15% до 7.5%).

### 7.4. [НИЗКИЙ ПРИОРИТЕТ] Оптимизация форматирования

Заменить `vsnprintf` с `%f` на целочисленное форматирование (fixed-point):
```c
// Вместо: "%.2f" для speed
// Использовать: "%d.%02d" с предварительным умножением на 100
```

**Выигрыш:** ~0.3-0.5 мс на вызов (newlib float formatting тяжёлый).

### 7.5. [НЕ ТРЕБУЕТСЯ] Изменения в Zephyr LOG

Текущая конфигурация оптимальна:
- Deferred mode (дефолт) — LOG_* не блокируют
- LOG вызовов в critical path нет
- Уровень INFO — приемлем для отладки
- Для production можно переключить на `CONFIG_LOG_DEFAULT_LEVEL=2` (WRN), но выигрыш минимален

---

## 8. Матрица рисков

| Риск | Вероятность | Последствия | Миграция |
|------|-------------|-------------|----------|
| UART TX блокирует PID на 6 мс | **100%** (происходит сейчас) | Джиттер PID, нестабильность на высоких скоростях | 7.1 или 7.2 |
| Mutex contention между control и wifi_cmd | Низкая | Дополнительная задержка control thread | 7.2 |
| Переполнение 256-байт буфера wifi_cmd_printf | Минимальная (данные ~70 байт) | Усечение телеметрии | — |
| Потеря телеметрии при async TX | N/A (при реализации 7.1/7.2) | Пропуск кадров — безвредно | Ring buffer с overwrite |

---

## 9. Приоритет реализации

1. **7.1** — Async UART TX (наибольший выигрыш, минимальные изменения)
2. **7.3** — Делитель частоты (тривиально, можно сделать сразу)
3. **7.2** — Отдельный поток (максимальная изоляция, больше изменений)
4. **7.4** — Оптимизация форматирования (marginal gains)

---

## 10. Файлы проанализированы

| Файл | LOG_* вызовов | wifi_cmd_printf | В critical path |
|------|---------------|-----------------|-----------------|
| src/main.c | 3 + 4 printk | 2 | Нет |
| src/car.c | 7 | 0 | Нет |
| src/tachometer.c | 2 | 0 | ISR — чистый |
| src/sensors.c | 5 | 0 | sensors_poll — чистый |
| src/imu.c | 5 | 0 | imu_update — чистый |
| src/battery.c | 3 | 0 | battery_thread — чистый |
| src/settings.c | 8 | 0 | Нет |
| src/wifi_cmd.c | 3 | ~80 | TX-функции (блокирующие) |
| src/control.c | 2 | 2 (в loop!) | **send_telemetry — 15%** |
| src/track_learn.c | 4 | ~10 | On-demand |
| src/tests.c | 0 | ~30 | Тестовый режим |
| prj.conf | — | — | Конфигурация |
| boards/*.overlay | — | — | UART0/UART1 раздельно |
