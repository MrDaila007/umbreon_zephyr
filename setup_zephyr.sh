#!/usr/bin/env bash
#
# Setup Zephyr v4.1 workspace with RP2350 flash patches.
# Usage:
#   ./setup_zephyr.sh [ZEPHYR_DIR]          # full setup (default ~/zephyrproject)
#   ./setup_zephyr.sh --patch-only [DIR]    # only apply flash patches
#
set -euo pipefail

ZEPHYR_VERSION="v4.1.0"
ZEPHYR_PATCH_URL="https://github.com/zephyrproject-rtos/zephyr/commit/5d36e85b99a.patch"
HAL_PATCH_URL="https://github.com/zephyrproject-rtos/hal_rpi_pico/commit/5d7744c.patch"

PATCH_ONLY=false
ZEPHYR_DIR="${HOME}/zephyrproject"

# --- Parse arguments ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        --patch-only) PATCH_ONLY=true; shift ;;
        --help|-h)
            echo "Usage: $0 [--patch-only] [ZEPHYR_DIR]"
            echo ""
            echo "Sets up a Zephyr ${ZEPHYR_VERSION} workspace with RP2350 flash patches."
            echo ""
            echo "Options:"
            echo "  --patch-only   Skip workspace init, only apply patches"
            echo "  ZEPHYR_DIR     Workspace path (default: ~/zephyrproject)"
            exit 0
            ;;
        *) ZEPHYR_DIR="$1"; shift ;;
    esac
done

# Resolve ~ in path
ZEPHYR_DIR="${ZEPHYR_DIR/#\~/$HOME}"

info()  { echo -e "\033[1;32m[INFO]\033[0m $*"; }
warn()  { echo -e "\033[1;33m[WARN]\033[0m $*"; }
error() { echo -e "\033[1;31m[ERROR]\033[0m $*"; exit 1; }

# --- Check prerequisites ---
for cmd in git curl python3; do
    command -v "$cmd" >/dev/null 2>&1 || error "'$cmd' is required but not found"
done

# --- Step 1: Initialize workspace ---
if [ "$PATCH_ONLY" = false ]; then
    if [ -d "${ZEPHYR_DIR}/.west" ]; then
        info "Workspace already exists at ${ZEPHYR_DIR}, skipping init"
    else
        info "Initializing Zephyr ${ZEPHYR_VERSION} workspace at ${ZEPHYR_DIR}..."
        west init -m https://github.com/zephyrproject-rtos/zephyr \
            --mr "${ZEPHYR_VERSION}" "${ZEPHYR_DIR}"
        info "Running west update (this may take a while)..."
        cd "${ZEPHYR_DIR}"
        west update --narrow -o=--depth=1
    fi

    # --- Step 2: Python venv ---
    if [ -d "${ZEPHYR_DIR}/.venv" ]; then
        info "Python venv already exists, skipping"
    else
        info "Creating Python virtual environment..."
        python3 -m venv "${ZEPHYR_DIR}/.venv"
    fi

    info "Installing Python dependencies..."
    source "${ZEPHYR_DIR}/.venv/bin/activate"
    pip install --quiet west
    pip install --quiet -r "${ZEPHYR_DIR}/zephyr/scripts/requirements.txt"
else
    info "Patch-only mode — skipping workspace init"
    [ -d "${ZEPHYR_DIR}/.west" ] || error "No workspace found at ${ZEPHYR_DIR}"
fi

# --- Step 3: Apply RP2350 flash patches ---
apply_patch() {
    local repo_dir="$1"
    local patch_url="$2"
    local label="$3"

    cd "${repo_dir}"

    # Try dry-run apply; if it fails the patch is already applied (or conflicts)
    if curl -sfL "${patch_url}" | git apply --check 2>/dev/null; then
        info "Applying ${label} patch..."
        curl -sfL "${patch_url}" | git apply -v
    else
        info "${label} patch already applied, skipping"
    fi
}

apply_patch "${ZEPHYR_DIR}/zephyr" \
    "${ZEPHYR_PATCH_URL}" \
    "Zephyr RP2350 flash controller (5d36e85b99a)"

apply_patch "${ZEPHYR_DIR}/modules/hal/rpi_pico" \
    "${HAL_PATCH_URL}" \
    "HAL flash_write_partial (5d7744c)"

# --- Done ---
echo ""
info "Setup complete!"
info "Workspace: ${ZEPHYR_DIR}"
echo ""
echo "To build:"
echo "  cd ${ZEPHYR_DIR} && source .venv/bin/activate"
echo "  west build -b rpi_pico2/rp2350a/m33 --pristine always $(cd "$(dirname "$0")" && pwd)"
echo ""
echo "To flash (BOOTSEL mode):"
echo "  cp build/zephyr/zephyr.uf2 /media/\${USER}/RP2350/"
