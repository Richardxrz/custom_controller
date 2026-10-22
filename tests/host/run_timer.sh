#!/usr/bin/env bash
# Timer 模块验收（主机侧，不需要板子）。三段：
#   1) 头文件即契约：.h 声明的函数能不能链接上（契约清单从 .h 现场生成，不手抄）
#   2) 对外符号清单：实现里对外的函数 vs .h 里声明的
#   3) 行为判据：DWT 替身把时间拨到已知值，逐场景单进程定量判
#
#   ./tests/host/run_timer.sh              # 全部
#   ./tests/host/run_timer.sh t_wrap       # 单个行为场景
set -u
cd "$(dirname "$0")/../.." || exit 2

BIN=build/host_timer_cases
LNK=build/host_timer_contract
mkdir -p build

CFLAGS=(-std=c11 -Wall -Wextra -g -O0 -I tests/host/stub -I Module)
rc=0

echo "=== 1) 头文件即契约：.h 里声明的每个函数都必须有定义 ==="
# 契约 TU 从 .h 现场生成：头文件加了新函数，这一步自动覆盖，不会漏
python3 - build/timer_contract.c <<'PYGEN'
import re, pathlib, sys
out = pathlib.Path(sys.argv[1])
hdr = pathlib.Path('Module/timer.h').read_text()
# 抓 "返回类型 函数名(" 形式的声明
names = [m.group(2) for m in re.finditer(r'^\s*([A-Za-z_]\w*)\s+([A-Za-z_]\w*)\s*\(', hdr, re.M)]
names = [n for n in names if n not in ('void', 'if', 'for', 'while')]
body = '\n'.join('  (void *)%s,' % n for n in names)
out.write_text(
    '/* 由 run_timer.sh 从 Module/timer.h 生成，勿手改 */\n'
    '#include "stm32f4xx.h"\n'
    '#include "timer.h"\n'
    'DWT_Type stub_dwt;\n'
    'CoreDebug_Type stub_coredebug;\n'
    'uint32_t SystemCoreClock = 168000000;\n'
    'static void *const k_contract[] = {\n' + body + '\n};\n'
    'int main(void) { return k_contract[0] == 0; }\n')
print('  契约清单 %d 个：%s' % (len(names), ' '.join(names)))
PYGEN
if cc "${CFLAGS[@]}" build/timer_contract.c Module/timer.c -o "$LNK" 2>/tmp/timer_link.err; then
  echo "  ✅ 头文件声明的函数全部链接成功"
else
  echo "  ❌ 链接失败 —— .h 与实现对不上，原始信息："
  sed 's/^/     /' /tmp/timer_link.err | head -20
  rc=1
fi

echo
echo "=== 2) 对外符号：实现里对外的函数 vs .h 里声明的 ==="
if cc "${CFLAGS[@]}" -c Module/timer.c -o build/timer.o 2>/tmp/timer_cc.err; then
  grep -i "warning" /tmp/timer_cc.err | sed 's/^/  ⚠️  /' || true
  python3 - <<'PY'
import re, subprocess, pathlib, sys
nm = subprocess.run(['nm', '-g', '--defined-only', 'build/timer.o'],
                    capture_output=True, text=True).stdout
defined = set()
for line in nm.splitlines():
    parts = line.split()
    if len(parts) >= 3 and parts[-2] == 'T':      # T = 对外（全局）函数
        defined.add(parts[-1].lstrip('_'))        # Mach-O 符号名带下划线前缀
hdr = pathlib.Path('Module/timer.h').read_text()
declared = set(re.findall(r'\b([A-Za-z_]\w*)\s*\(', hdr)) - {'void', 'if', 'for', 'while'}
missing_impl = sorted(declared - defined)
not_declared = sorted(defined - declared)
ok = True
if missing_impl:
    print('  ❌ .h 声明了、实现里没有：', ' '.join(missing_impl)); ok = False
if not_declared:
    print('  ⚠️  实现里对外、但 .h 没声明：', ' '.join(not_declared),
          '（别人用不到，除非自己 extern 声明）')
if not missing_impl:
    print('  ✅ .h 声明的函数，实现里都有（共 %d 个）' % len(declared))
sys.exit(0 if ok else 1)
PY
else
  echo "  ❌ timer.c 编译失败："; sed 's/^/     /' /tmp/timer_cc.err | head -10; rc=1
fi

echo
echo "=== 3) 行为判据（DWT 替身，时间可控）==="
if cc "${CFLAGS[@]}" tests/host/timer_cases.c Module/timer.c -o "$BIN"; then
  SCENES=(t_init t_units t_wrap t_now t_count_max t_min t_reset t_precision)
  if [ $# -gt 0 ]; then SCENES=("$@"); fi
  for s in "${SCENES[@]}"; do
    "$BIN" "$s" || rc=1
  done
else
  echo "  ❌ 行为判据编译失败"; rc=1
fi

echo
echo "=== 总退出码: $rc （0=全部满足）==="
exit $rc
