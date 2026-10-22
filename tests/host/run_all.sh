#!/usr/bin/env bash
# 全部主机侧测试（不需要板子）。
#   ./tests/host/run_all.sh
set -u
cd "$(dirname "$0")/../.." || exit 2

mkdir -p build
CFLAGS=(-std=c11 -Wall -Wextra -Werror -g -O0
        -fsanitize=address,undefined -fno-omit-frame-pointer
        -I Module -I tests/host)

fail=0
build_run() {
  local name="$1"
  shift
  echo "--- 编译 $name"
  cc "${CFLAGS[@]}" "$@" -o "build/$name" || exit 2
  echo "--- 运行 $name"
  "./build/$name" || fail=1
  echo
}

build_run host_servo tests/host/servo_main.c Module/ringbuf.c Module/servo_codec.c Module/bus_servo.c
build_run host_rm tests/host/test_rm.c Module/rm_frame.c Module/app_payload.c
build_run host_group tests/host/test_group.c Module/ringbuf.c Module/servo_codec.c Module/bus_servo.c Module/bus_servo_group.c
build_run host_dm tests/host/test_dm.c Module/dm_can.c

echo "=== 总结果: $([ $fail -eq 0 ] && echo 全过 || echo 有失败) ==="
exit $fail
