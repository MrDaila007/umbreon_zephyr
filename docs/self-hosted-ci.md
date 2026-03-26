# Self-Hosted CI Runner on Proxmox VE

Руководство по развёртыванию self-hosted GitHub Actions runner на Proxmox VE
с поддержкой Hardware-in-the-Loop (HIL) тестирования через ST-Link.

## Архитектура

```
Proxmox VE (хост)
  └── LXC контейнер (Ubuntu 24.04)
        ├── GitHub Actions Runner (systemd)
        ├── Zephyr SDK 0.17 + workspace v4.3
        ├── OpenOCD
        └── USB ──► ST-Link ──► SWD ──► Pico 2
```

## Что даёт

- **Быстрее GitHub-hosted** — SDK и workspace закешированы постоянно
- **Не тратит GitHub Actions minutes** — бесплатно
- **HIL-тесты** — прошивка и тестирование на реальном железе из CI

## Требования

- Proxmox VE 8.x
- Доступ в интернет с хоста (GitHub API)
- ST-Link V2 (`0483:3748`) или V3 (`0483:374b`) — для HIL
- Pico 2 подключен к ST-Link по SWD

---

## Часть 1: LXC контейнер

### 1.1 Создать контейнер

В Proxmox GUI или через CLI:

```bash
# Скачать шаблон (если нет)
pveam download local ubuntu-24.04-standard_24.04-2_amd64.tar.zst

# Создать контейнер
pct create 200 local:vztmpl/ubuntu-24.04-standard_24.04-2_amd64.tar.zst \
    --hostname ci-runner \
    --cores 4 \
    --memory 4096 \
    --swap 2048 \
    --rootfs local-lvm:20 \
    --net0 name=eth0,bridge=vmbr0,ip=dhcp \
    --unprivileged 0 \
    --features nesting=1

pct start 200
```

> **Примечание:** `--unprivileged 0` (привилегированный) нужен для USB passthrough.
> Если HIL не планируется — используйте `--unprivileged 1` (безопаснее).

### 1.2 USB Passthrough (ST-Link)

#### На хосте Proxmox: найти устройство

```bash
lsusb | grep -i st-link
# Bus 002 Device 005: ID 0483:3748 STMicroelectronics ST-LINK/V2
```

#### На хосте: udev-правило для стабильного имени

```bash
cat > /etc/udev/rules.d/99-st-link.rules << 'EOF'
# ST-Link/V2
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", MODE="0666", SYMLINK+="stlink"
# ST-Link/V3
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE="0666", SYMLINK+="stlink"
EOF

udevadm control --reload-rules && udevadm trigger
```

#### На хосте: прокинуть USB в контейнер

Добавить в `/etc/pve/lxc/200.conf`:

```
lxc.cgroup2.devices.allow: c 189:* rwm
lxc.mount.entry: /dev/bus/usb dev/bus/usb none bind,optional,create=dir
```

Перезапустить контейнер:

```bash
pct stop 200 && pct start 200
```

Проверить внутри контейнера:

```bash
pct enter 200
lsusb | grep -i st-link
# Должно показать ST-Link
```

---

## Часть 2: Настройка контейнера

### 2.1 Системные пакеты

```bash
pct enter 200

apt update && apt upgrade -y
apt install -y \
    git cmake ninja-build python3 python3-pip python3-venv \
    wget curl ccache device-tree-compiler dfu-util \
    build-essential pkg-config libusb-1.0-0-dev \
    picocom usbutils
```

### 2.2 OpenOCD (для HIL)

```bash
apt install -y openocd
```

Проверить что ST-Link виден:

```bash
openocd -f interface/stlink.cfg -f target/rp2350.cfg -c "init; exit"
```

Если стандартный пакет слишком старый для RP2350, собрать из исходников:

```bash
apt install -y automake autoconf libtool
git clone https://github.com/openocd-org/openocd.git /opt/openocd-src
cd /opt/openocd-src
./bootstrap
./configure --enable-stlink
make -j$(nproc)
make install
```

### 2.3 Zephyr SDK + Workspace

```bash
# SDK
wget -q https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.17.0/zephyr-sdk-0.17.0_linux-x86_64_minimal.tar.xz
tar xf zephyr-sdk-0.17.0_linux-x86_64_minimal.tar.xz -C /opt/
rm zephyr-sdk-0.17.0_linux-x86_64_minimal.tar.xz
cd /opt/zephyr-sdk-0.17.0 && ./setup.sh -t arm-zephyr-eabi -c

# Workspace (от имени пользователя runner, см. ниже)
```

---

## Часть 3: GitHub Actions Runner

### 3.1 Создать пользователя

```bash
useradd -m -s /bin/bash runner
usermod -aG dialout,plugdev runner   # доступ к USB/serial
```

### 3.2 Установить runner

На GitHub: **Settings → Actions → Runners → New self-hosted runner** — скопировать токен.

```bash
su - runner

mkdir actions-runner && cd actions-runner
curl -O -L https://github.com/actions/runner/releases/download/v2.333.0/actions-runner-linux-x64-2.333.0.tar.gz
tar xzf actions-runner-linux-x64-2.333.0.tar.gz
rm actions-runner-linux-x64-2.333.0.tar.gz
```

### 3.3 Зарегистрировать

```bash
./config.sh \
    --url https://github.com/<OWNER>/umbreon_zephyr \
    --token <TOKEN_FROM_GITHUB> \
    --name proxmox-runner \
    --labels self-hosted,linux,x64,embedded \
    --work _work
```

### 3.4 Установить как systemd-сервис

```bash
exit   # вернуться в root
cd /home/runner/actions-runner
./svc.sh install runner
./svc.sh start
./svc.sh status
```

Логи:

```bash
journalctl -u actions.runner.* -f
```

### 3.5 Подготовить Zephyr workspace

На Ubuntu/Debian Python помечен как *externally managed* ([PEP 668](https://peps.python.org/pep-0668/)): `pip install` и `pip install --user` в системный интерпретатор запрещены. Нужен venv (или отдельный инструмент вроде `pipx install west`).

`west init` требует пустой каталог, поэтому сначала ставим `west` во временный venv, инициализируем workspace, затем создаём постоянный `.venv` внутри `~/zephyrproject` для сборок и зависимостей Zephyr.

```bash
su - runner

python3 -m venv ~/.zephyr-west-bootstrap
source ~/.zephyr-west-bootstrap/bin/activate
pip install west
west init -m https://github.com/zephyrproject-rtos/zephyr --mr v4.3.0 ~/zephyrproject
cd ~/zephyrproject
west update --narrow -o=--depth=1
deactivate

python3 -m venv .venv
source .venv/bin/activate
pip install west
pip install -r zephyr/scripts/requirements.txt
```

---

## Часть 4: Workflow для self-hosted runner

- **Активный CI:** [`.github/workflows/build.yml`](../.github/workflows/build.yml) — `runs-on: [self-hosted, linux, embedded]`, сборка из предустановленного `~/zephyrproject` (см. §3.5).
- **Резервная копия облачного варианта** (GitHub-hosted + Docker Zephyr CI): [`docs/ci-backup/build.cloud.yml`](ci-backup/build.cloud.yml). Чтобы снова гонять CI в облаке, можно подменить содержимое `build.yml` этим файлом.

У раннера в `config.sh` / настройках GitHub должны быть те же labels: `self-hosted`, `linux`, `embedded`.

### Опционально: HIL (прошивка + smoke по UART)

Когда понадобится железо, добавьте в `build.yml` отдельный job `needs: build` с OpenOCD и своим `tty` (пример ниже — адаптируйте порт и протокол):

```yaml
  test-hardware:
    needs: build
    runs-on: [self-hosted, linux, embedded]
    steps:
      - uses: actions/checkout@v4

      - name: Flash via ST-Link
        run: |
          openocd -f interface/stlink.cfg -f target/rp2350.cfg \
            -c "adapter speed 5000" \
            -c "program ${HOME}/zephyrproject/build/zephyr/zephyr.elf verify reset exit"

      - name: Smoke test (UART)
        run: |
          timeout 10 bash -c '
            exec 3<>/dev/ttyUSB0
            stty -F /dev/ttyUSB0 115200 raw -echo
            echo -ne "\$PING\n" >&3
            while IFS= read -r -t 5 line <&3; do
              if [[ "$line" == *"PONG"* ]]; then
                echo "PASS: got PONG"
                exit 0
              fi
            done
            echo "FAIL: no PONG received"
            exit 1
          '
```

---

## Часть 5: Подключение ST-Link к Pico 2

### Распиновка SWD

```
ST-Link            Pico 2 (debug pads на плате)
  SWCLK  ─────►   SWCLK
  SWDIO  ─────►   SWDIO
  GND    ─────►   GND
  3.3V   ─────►   (опционально, если Pico не питается от USB)
```

> **Важно:** SWD пады на Pico 2 находятся на нижней стороне платы.
> Можно припаять провода или использовать pogo-pin адаптер.

### Опционально: UART через ST-Link V3

ST-Link V3 имеет встроенный UART bridge. Подключение:

```
ST-Link V3         Pico 2
  TX  ────────►    GP17 (UART0 RX)
  RX  ◄────────    GP16 (UART0 TX)
```

Это позволит и прошивать, и читать debug-консоль через один ST-Link.

---

## Обслуживание

### Обновление runner

```bash
# Runner обновляется автоматически при запуске workflow.
# Для ручного обновления:
su - runner
cd actions-runner
./svc.sh stop
# Скачать новую версию, распаковать поверх
./svc.sh start
```

### Очистка диска

```bash
# Добавить в crontab runner'а:
crontab -e
# Очистка build-артефактов старше 7 дней
0 3 * * * find /home/runner/actions-runner/_work -maxdepth 3 -name build -type d -mtime +7 -exec rm -rf {} +
```

### Обновление Zephyr workspace

```bash
su - runner
cd ~/zephyrproject
source .venv/bin/activate
west update
pip install -r zephyr/scripts/requirements.txt
```

---

## Проверка работоспособности

### Чеклист

- [ ] Контейнер запущен: `pct status 200`
- [ ] Runner online: GitHub → Settings → Actions → Runners (зелёный кружок)
- [ ] ST-Link виден: `lsusb | grep 0483` внутри контейнера
- [ ] OpenOCD работает: `openocd -f interface/stlink.cfg -f target/rp2350.cfg -c "init; exit"`
- [ ] Zephyr собирается: `make build` в директории проекта
- [ ] `west flash` работает с ST-Link
- [ ] UART доступен: `picocom /dev/ttyUSB0 -b 115200`

### Тестовый запуск

Создать пустой коммит для запуска CI:

```bash
git commit --allow-empty -m "test: trigger CI"
git push
```

Проверить в GitHub → Actions что job подхватился self-hosted runner'ом.
