# AGENTS.md

## Cursor Cloud specific instructions

### Overview

This is an embedded C firmware project (Zephyr RTOS v4.3, targeting RP2350/Pico 2). There is no web server, database, or Docker — the build toolchain is CMake + Zephyr `west`. The full build/test/flash workflow is documented in the `README.md` and driven via the root `Makefile`.

### Environment prerequisites

The Zephyr workspace lives at `~/zephyrproject-v4.3` (SDK at `~/zephyr-sdk-0.17.0`). The update script handles Python venv dependency refresh. System packages required beyond the base image: `ninja-build`, `device-tree-compiler`, `ccache`, `dfu-util`, `python3-venv`, `gcc-multilib`, `g++-multilib`.

- `gcc-multilib` / `g++-multilib` are required for `make test-ztest` (`native_sim` board uses `-m32`).
- `~/.local/bin` must be on `PATH` for `west` (installed via `pip3 install --user west`).

### Building

```bash
source ~/zephyrproject-v4.3/.venv/bin/activate
make build        # UART console firmware
make build-usb    # USB CDC-ACM console firmware
```

The venv must be activated before any `make build*` or `make test-ztest` target (the Makefile sources it, but `west` must be on PATH).

### Testing

```bash
make test-host    # GCC host unit tests — no Zephyr/venv needed
make test-ztest   # Zephyr ztest on native_sim — needs venv + multilib
make test         # both of the above
```

### Gotchas

- Flashing (`make flash`) and serial monitoring (`make monitor`) require physical RP2350 hardware — these cannot run in Cloud Agent VMs.
- The `setup_zephyr.sh` script expects `west` on PATH; if running manually, ensure `~/.local/bin` is in PATH first or use the venv.
- `west update --narrow -o=--depth=1` takes several minutes on first run but is idempotent.
