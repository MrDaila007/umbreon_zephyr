# Pinout

## Плата

Umbreon roborace board on Raspberry Pi Pico 2 (RP2350A).

| GPIO | Назначение | Примечание |
|------|------------|------------|
| GP0 | I2C0 SDA | MPU-6050 IMU + SSD1306 OLED |
| GP1 | I2C0 SCL | 400 kHz |
| GP2 | I2C1 SDA | 6x VL53L0X ToF |
| GP3 | I2C1 SCL | 100 kHz |
| GP4 | UART1 TX | ESP8266 WiFi bridge |
| GP5 | UART1 RX | ESP8266 WiFi bridge |
| GP6 | XSHUT sensor 0 | Hard-Right |
| GP7 | XSHUT sensor 1 | Front-Right |
| GP8 | XSHUT sensor 2 | Right |
| GP9 | XSHUT sensor 3 | Left |
| GP10 | PWM5A | Steering servo |
| GP11 | PWM5B | Motor ESC |
| GP12 | Encoder DT | Active low |
| GP13 | Tachometer | Rising edge IRQ |
| GP14 | XSHUT sensor 4 | Front-Left |
| GP15 | XSHUT sensor 5 | Hard-Left |
| GP16 | UART0 TX | Debug console |
| GP17 | UART0 RX | Debug console |
| GP18 | GPIO | Piezo buzzer |
| GP19 | Encoder button | Active low |
| GP22 | Encoder CLK | Active low |
| GP26 | ADC ch0 | Battery voltage |

## Маркировка сенсоров

Порядок индексов совпадает с `src/sensors.h` и devicetree:

| Index | Marking | I2C address | XSHUT |
|------:|---------|------------:|-------|
| 0 | HR | `0x30` | GP6 |
| 1 | FR | `0x31` | GP7 |
| 2 | R | `0x32` | GP8 |
| 3 | L | `0x33` | GP9 |
| 4 | FL | `0x34` | GP14 |
| 5 | HL | `0x35` | GP15 |

## Расположение сенсоров

```text
             REAR

      [0 HR]           [5 HL]
      Hard-Right       Hard-Left

          [1 FR]   [4 FL]
          Front-R  Front-L

          [2 R]    [3 L]
          Right    Left

            FRONT
```

## Остальные узлы

| Узел | Интерфейс | Адрес / назначение |
|------|-----------|-------------------|
| MPU-6050 | I2C0 | `0x68` |
| SSD1306 OLED | I2C0 | `0x3C` |
| ESP8266 | UART1 | GP4/GP5, 115200 |
| Debug console | UART0 | GP16/GP17, 115200 |
| Servo | PWM | GP10 |
| ESC | PWM | GP11 |
| Tachometer | GPIO IRQ | GP13 |
| Buzzer | GPIO | GP18 |
