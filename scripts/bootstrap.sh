#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[bootstrap] repo: ${ROOT_DIR}"

if [[ -f /opt/agiros/loong/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/agiros/loong/setup.bash
else
  echo "[bootstrap] AGIROS loong setup not found under /opt/agiros/loong."
  echo "[bootstrap] Python-only checks can still run, but colcon/agirosdep steps will be skipped."
fi

python3 -m pip install --upgrade pip
python3 -m pip install -r "${ROOT_DIR}/requirements.txt" ruff pytest pre-commit

if command -v agirosdep >/dev/null 2>&1; then
  agirosdep update || true
  agirosdep install --from-paths "${ROOT_DIR}" --ignore-src -r -y || true
else
  echo "[bootstrap] agirosdep not found; skipping dependency resolution."
fi

echo "[bootstrap] complete"
