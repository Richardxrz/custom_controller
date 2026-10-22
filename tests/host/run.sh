#!/usr/bin/env bash
# 主机侧测试台入口。不需要板子，只要本机有 cc（macOS 自带 clang）。
#
#   ./tests/host/run.sh              # 全部场景
#   ./tests/host/run.sh s_normal     # 单个场景
#
# 每个场景单独起一个进程（模块内部是 static 状态，进程隔离才能得到干净结果）。
set -u
cd "$(dirname "$0")/../.." || exit 2

BIN=build/host_tx_test
mkdir -p build

CFLAGS=(-std=c11 -Wall -Wextra -g -O0
        -fsanitize=address,undefined -fno-omit-frame-pointer
        -I tests/host/stub -I Module)

echo "--- 编译 ---"
cc "${CFLAGS[@]}" tests/host/main.c Module/tx_uart_it.c -o "$BIN" || exit 2
echo "编译完成: $BIN"

SCENES=(s_uninit s_normal s_len0 s_null s_over s_clamp128 s_two s_reentry s_resend)
if [ $# -gt 0 ]; then SCENES=("$@"); fi

rc=0
for s in "${SCENES[@]}"; do
  "$BIN" "$s" || rc=1
done
echo
echo "=== 总退出码: $rc （0=全部符合期望） ==="
exit $rc
