# Source in a shell on the GPU host; changes only this process environment.
export VLA_DEPLOY_ROOT="${VLA_DEPLOY_ROOT:-$HOME/vla_pi05}"
export UV_CACHE_DIR="$VLA_DEPLOY_ROOT/cache/uv"
export UV_PYTHON_INSTALL_DIR="$VLA_DEPLOY_ROOT/python"
export UV_LINK_MODE=copy
export GIT_LFS_SKIP_SMUDGE=1
export OPENPI_DATA_HOME="$VLA_DEPLOY_ROOT/cache/openpi"
export HF_HOME="$VLA_DEPLOY_ROOT/cache/huggingface"
export XLA_PYTHON_CLIENT_PREALLOCATE=false
