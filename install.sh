#!/usr/bin/env bash
# One-click installer for the GELLO -> IIMT master/slave control panel.
# This script installs software only. It never starts or commands a robot.

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
VENV_DIR="${SCRIPT_DIR}/.venv"
SDK_LIB="${SCRIPT_DIR}/third_party/IIMT-SDK/python/linux/x64/lib/libiimtcl.so"
CONFIG_FILE="${SCRIPT_DIR}/configs/iimt_master_slave.json"
PYTHON_BIN="${PYTHON_BIN:-python3}"
TARGET_USER="${SUDO_USER:-${USER:-}}"
NEEDS_RELOGIN=0

info() {
    printf '\n\033[1;34m[INFO]\033[0m %s\n' "$*"
}

ok() {
    printf '\033[1;32m[ OK ]\033[0m %s\n' "$*"
}

warn() {
    printf '\033[1;33m[WARN]\033[0m %s\n' "$*" >&2
}

fail() {
    printf '\033[1;31m[FAIL]\033[0m %s\n' "$*" >&2
    exit 1
}

on_error() {
    local exit_code=$?
    printf '\n\033[1;31m[FAIL]\033[0m Installation stopped at line %s (exit code %s).\n' \
        "${BASH_LINENO[0]}" "${exit_code}" >&2
    printf 'Fix the error above, then run this script again. It is safe to repeat.\n' >&2
    exit "${exit_code}"
}
trap on_error ERR

if [[ "${EUID}" -eq 0 ]]; then
    fail "Please run ./install.sh as a normal user, not with sudo. The script requests sudo only when needed."
fi

[[ "$(uname -s)" == "Linux" ]] || fail "The bundled IIMT SDK supports Linux only."
[[ "$(uname -m)" == "x86_64" ]] || fail "The bundled IIMT SDK is x86_64; current architecture: $(uname -m)."
[[ -f "${SDK_LIB}" ]] || fail "Bundled IIMT SDK library is missing: ${SDK_LIB}"
[[ -f "${CONFIG_FILE}" ]] || fail "Project configuration is missing: ${CONFIG_FILE}"
command -v sudo >/dev/null 2>&1 || fail "sudo is required to install system packages and serial permissions."

if command -v apt-get >/dev/null 2>&1; then
    info "Installing Ubuntu/Debian system packages"
    sudo -v
    sudo apt-get update
    sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y \
        build-essential \
        ca-certificates \
        cmake \
        git \
        iputils-ping \
        libgcc-s1 \
        libstdc++6 \
        libzmq3-dev \
        ninja-build \
        pkg-config \
        python3 \
        python3-dev \
        python3-pip \
        python3-tk \
        python3-venv \
        udev
else
    fail "Only Ubuntu/Debian (apt-get) is supported by this one-click installer."
fi

command -v "${PYTHON_BIN}" >/dev/null 2>&1 || fail "${PYTHON_BIN} was not found after package installation."

info "Checking Python version"
"${PYTHON_BIN}" - <<'PY'
import sys

if sys.version_info < (3, 10):
    raise SystemExit(
        f"Python 3.10 or newer is required; found {sys.version.split()[0]}"
    )
print(f"Python {sys.version.split()[0]}")
PY

info "Creating the isolated Python environment at ${VENV_DIR}"
if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
    "${PYTHON_BIN}" -m venv "${VENV_DIR}"
fi
VENV_PYTHON="${VENV_DIR}/bin/python"

info "Installing the Python packages required by this IIMT control project"
"${VENV_PYTHON}" -m pip install --upgrade pip setuptools wheel
"${VENV_PYTHON}" -m pip install \
    "numpy>=1.21,<3" \
    "pyzmq==27.2.0" \
    "tyro==1.0.16" \
    "ruckig==0.19.4" \
    "dynamixel-sdk==4.1.0"
"${VENV_PYTHON}" -m pip install --no-deps --editable "${SCRIPT_DIR}"

info "Checking the bundled IIMT SDK dependencies"
if ldd "${SDK_LIB}" | grep -q 'not found'; then
    ldd "${SDK_LIB}" >&2
    fail "The IIMT SDK has unresolved shared-library dependencies."
fi
ok "IIMT SDK shared-library dependencies are available"

info "Checking Python imports and project configuration"
PROJECT_DIR="${SCRIPT_DIR}" "${VENV_PYTHON}" - <<'PY'
import ctypes
import json
import os
from pathlib import Path

import dynamixel_sdk
import numpy
import ruckig
import tkinter
import tyro
import zmq

root = Path(os.environ["PROJECT_DIR"])
config_path = root / "configs/iimt_master_slave.json"
config = json.loads(config_path.read_text(encoding="utf-8"))
followers = config.get("followers")
if not isinstance(followers, list) or len(followers) != 2:
    raise SystemExit("configs/iimt_master_slave.json must contain two follower entries")
for index, follower in enumerate(followers, start=1):
    gains = follower.get("joint_gains")
    if not isinstance(gains, list) or len(gains) != 6:
        raise SystemExit(f"Follower {index} must contain six joint gains")

sdk = root / "third_party/IIMT-SDK/python/linux/x64/lib/libiimtcl.so"
ctypes.CDLL(str(sdk))
print("Python imports, JSON configuration, and IIMT SDK loading passed")
PY

info "Configuring USB serial-port permission"
if [[ -z "${TARGET_USER}" ]]; then
    warn "Could not determine the login user; add your account to the dialout group manually."
elif id -nG "${TARGET_USER}" | tr ' ' '\n' | grep -qx dialout; then
    ok "${TARGET_USER} is already a member of dialout"
else
    sudo usermod -aG dialout "${TARGET_USER}"
    NEEDS_RELOGIN=1
    ok "Added ${TARGET_USER} to dialout"
fi

info "Checking the configured GELLO serial device"
GELLO_PORT="$(PROJECT_DIR="${SCRIPT_DIR}" "${VENV_PYTHON}" - <<'PY'
import json
import os
from pathlib import Path

config = json.loads(
    (Path(os.environ["PROJECT_DIR"]) / "configs/iimt_master_slave.json").read_text(
        encoding="utf-8"
    )
)
print(config.get("gello_port", ""))
PY
)"
if [[ -n "${GELLO_PORT}" && -e "${GELLO_PORT}" ]]; then
    ok "GELLO serial device found: ${GELLO_PORT}"
else
    warn "The configured GELLO serial device is not currently present: ${GELLO_PORT:-<empty>}"
    warn "Connect the USB adapter and verify its /dev/serial/by-id path before operation."
fi

printf '\n\033[1;32mInstallation completed successfully.\033[0m\n'
printf '\nThis installer did not start or command either robot.\n'
printf 'Before the first hardware run:\n'
printf '  1. Connect the GELLO USB adapter and both robots to the network.\n'
printf '  2. Start the panel with:\n'
printf '       cd %q\n' "${SCRIPT_DIR}"
printf '       %q %q\n' "${VENV_PYTHON}" "${SCRIPT_DIR}/master_slave_panel.py"
printf '  3. Enter each follower IP and its six joint gains in the UI, then save.\n'
printf '  4. Confirm the computer network interface can reach each robot IP.\n'
if [[ "${NEEDS_RELOGIN}" -eq 1 ]]; then
    printf '\n\033[1;33mImportant: log out and log back in (or reboot) before using the serial device.\033[0m\n'
fi
