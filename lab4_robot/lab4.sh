#!/usr/bin/env bash
# Lab 4 CE212 - Nhanh Robot: chay toan bo bai ve nha bang 1 lenh.
#   bash lab4.sh                 setup (nap firmware) -> check -> estop -> reverse -> report
#   bash lab4.sh demo            man hinh truc tiep de quay video
#   bash lab4.sh --sim all       chay thu voi phan cung GIA LAP (khong can ESP32)
#   bash lab4.sh <lenh khac>     truyen thang cho tools/lab4.py (xem: bash lab4.sh -h)
set -e
cd "$(dirname "$0")/tools"
if [ ! -d .venv ]; then
  python3 -m venv .venv || { echo "Thieu venv. Chay: sudo apt install python3-venv"; exit 1; }
  .venv/bin/pip install -q --upgrade pip
fi
.venv/bin/pip install -q -r requirements.txt
if [ $# -eq 0 ]; then set -- all; fi
exec .venv/bin/python lab4.py "$@"
