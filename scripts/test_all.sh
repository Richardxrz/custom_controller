#!/usr/bin/env bash
# 全部测试：C 主机侧 + Python。
set -euo pipefail
cd "$(dirname "$0")/.."

echo "=== C 主机侧测试 ==="
./tests/host/run_all.sh

echo
echo "=== Python 测试 ==="
cd pc
python3 -m unittest discover -s tests -v
