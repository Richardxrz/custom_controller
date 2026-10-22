#!/usr/bin/env bash
# 总线舵机模块主机侧测试台（不需要板子）。
#   ./tests/host/run_servo.sh
set -u
cd "$(dirname "$0")/../.." || exit 2

BIN=build/host_servo_test
mkdir -p build

CFLAGS=(-std=c11 -Wall -Wextra -Werror -g -O0
        -fsanitize=address,undefined -fno-omit-frame-pointer
        -I Module)

echo "--- 编译 ---"
cc "${CFLAGS[@]}" \
  tests/host/servo_main.c \
  Module/ringbuf.c \
  Module/servo_codec.c \
  Module/bus_servo.c \
  -o "$BIN" || exit 2
echo "编译完成: $BIN"
echo

"$BIN"
