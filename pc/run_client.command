#!/bin/bash
cd "$(dirname "$0")"
PY="$HOME/miniconda3/bin/python3"
[ -x "$PY" ] || PY=python3
exec "$PY" client_app.py "$@"
