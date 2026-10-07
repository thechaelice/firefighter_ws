#!/usr/bin/env bash
# Build the MLX90641 frame reader (scripts/thermal/mlx90641_frames).
#
# There is no MLX90641 driver on PyPI, so the vendor C library is the supported
# path. It is cloned to $MLX90641_LIB (default /tmp/mlx90641lib) if missing.
#
#   bash scripts/thermal/build.sh
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
vendor="${MLX90641_LIB:-/tmp/mlx90641lib}"

if [[ ! -f "$vendor/functions/MLX90641_API.cpp" ]]; then
    echo "cloning melexis/mlx90641-library -> $vendor"
    git clone --depth 1 https://github.com/melexis/mlx90641-library.git "$vendor"
fi

g++ -O2 -Wall -I"$vendor/headers" \
    "$here/mlx90641_frames.cpp" \
    "$vendor/functions/MLX90641_API.cpp" \
    -o "$here/mlx90641_frames"

echo "built: $here/mlx90641_frames"
