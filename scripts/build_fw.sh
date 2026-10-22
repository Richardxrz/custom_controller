#!/usr/bin/env bash
# 编译两份固件：主端（MASTER，默认）与机器人端（ARM）。
set -euo pipefail
cd "$(dirname "$0")/.."
TC="$(pwd)/cmake/gcc-arm-none-eabi.cmake"

echo "=== 主端固件（MASTER）==="
cmake -S . -B build/master -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE="$TC" -DCC_ROLE_ARM=OFF >/dev/null
cmake --build build/master

echo "=== 机器人端固件（ARM）==="
cmake -S . -B build/arm -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE="$TC" -DCC_ROLE_ARM=ON >/dev/null
cmake --build build/arm

echo "=== 尺寸 ==="
arm-none-eabi-size build/master/stm32_module_lab.elf build/arm/stm32_module_lab.elf
