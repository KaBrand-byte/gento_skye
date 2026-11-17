#!/usr/bin/env bash
# User-local installation on the GPU host. Does not touch ROS, SDK, or drivers.
set -euo pipefail
vla_root="${VLA_DEPLOY_ROOT:-$HOME/vla_pi05}"
openpi_commit="${1:?Pass the audited official openpi commit}"
mkdir -p "$vla_root/logs" "$vla_root/tools/bin"
trap 'result=$?; printf "%s\n" "$result" > "$vla_root/logs/bootstrap.exit"' EXIT
export UV_CACHE_DIR="$vla_root/cache/uv"
export UV_PYTHON_INSTALL_DIR="$vla_root/python"
export UV_LINK_MODE=copy
export UV_NO_PROGRESS=1
export UV_HTTP_TIMEOUT=120
export UV_CONCURRENT_DOWNLOADS="${UV_CONCURRENT_DOWNLOADS:-3}"
export GIT_LFS_SKIP_SMUDGE=1
export OPENPI_DATA_HOME="$vla_root/cache/openpi"
export HF_HOME="$vla_root/cache/huggingface"
export XLA_PYTHON_CLIENT_PREALLOCATE=false

printf 'Bootstrap started: %s\n' "$(date -Is)"
if [[ ! -x "$vla_root/tools/bin/uv" ]]; then
    UV_INSTALL_DIR="$vla_root/tools/bin" UV_NO_MODIFY_PATH=1 \
        sh "$vla_root/bootstrap/uv-installer.sh"
fi
vla_uv="$vla_root/tools/bin/uv"
"$vla_uv" --version

if [[ ! -e "$vla_root/openpi" ]]; then
    git clone --recurse-submodules https://github.com/Physical-Intelligence/openpi.git "$vla_root/openpi"
fi
cd "$vla_root/openpi"
if [[ "$(git remote get-url origin)" != https://github.com/Physical-Intelligence/openpi.git ]]; then
    printf 'Unexpected repository origin; stopping.\n' >&2
    exit 1
fi
if [[ -n "$(git status --porcelain)" ]]; then
    printf 'Existing checkout has changes; stopping.\n' >&2
    exit 1
fi
git checkout --detach "$openpi_commit"
git submodule update --init --recursive
git rev-parse HEAD | tee "$vla_root/logs/openpi-commit.txt"
"$vla_uv" sync --frozen --python 3.11
"$vla_uv" run --no-sync python - <<'PY'
import json
import jax
import jax.numpy as jnp
import torch
devices = jax.devices()
assert any(device.platform == 'gpu' for device in devices), devices
assert torch.cuda.is_available(), 'PyTorch cannot see CUDA'
x = jnp.ones((64, 64), dtype=jnp.bfloat16)
y = (x @ x).block_until_ready()
assert float(y[0, 0]) == 64
print(json.dumps({
    'jax': jax.__version__, 'jax_devices': [str(device) for device in devices],
    'torch': torch.__version__, 'torch_cuda_runtime': torch.version.cuda,
    'gpu': torch.cuda.get_device_name(0), 'jax_gpu_matmul_passed': True,
}, indent=2))
PY
printf 'Bootstrap completed: %s\n' "$(date -Is)"
