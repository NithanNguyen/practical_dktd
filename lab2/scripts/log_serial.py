#!/usr/bin/env python3
"""Buoi 2 - Ghi du lieu IMU tu ESP32 (firmware imu_logger) ra CSV.

Cach dung (tu thu muc lab2/):
    python scripts/log_serial.py --label static    --duration 60
    python scripts/log_serial.py --label vibration --duration 60

Ket qua:
    data/<label>.csv        du lieu tho + da loc (1 dong / mau, co header)
    data/<label>.meta.json  cau hinh firmware (bias gyro, N, alpha, ...) + thong ke truyen

Script reset ESP32 khi mo cong (giong esptool: RTS=1 -> EN thap) de chac chan nhan
duoc cac dong meta '#', roi cho dong '# cols:' moi bat dau ghi.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import serial  # pyserial
except ImportError:
    sys.exit("Thieu pyserial: pip install -r scripts/requirements.txt")

N_COLS = 16


def parse_meta(line: str, meta: dict) -> None:
    """'# k1=v1,k2=v2' -> meta[k] = float(v) neu duoc, nguoc lai giu chuoi."""
    body = line[1:].strip()
    for part in body.split(","):
        if "=" not in part:
            continue
        k, v = part.split("=", 1)
        k, v = k.strip(), v.strip()
        try:
            meta[k] = int(v, 0) if v.lower().startswith("0x") else float(v)
        except ValueError:
            meta[k] = v


def hard_reset(ser: serial.Serial) -> None:
    # Trinh tu HardReset cua esptool: DTR=0 (IO0 cao, khong vao bootloader), RTS=1 (EN thap)
    ser.dtr = False
    ser.rts = True
    time.sleep(0.1)
    ser.rts = False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=460800, help="phai trung SERIAL_BAUD trong firmware")
    ap.add_argument("--label", required=True, help="ten thi nghiem, vd static / vibration")
    ap.add_argument("--duration", type=float, default=60.0, help="thoi gian ghi (s)")
    ap.add_argument("--outdir", default="data")
    ap.add_argument("--no-reset", action="store_true", help="khong reset ESP32 khi mo cong")
    ap.add_argument("--boot-timeout", type=float, default=15.0, help="cho header toi da (s)")
    args = ap.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    csv_path = outdir / f"{args.label}.csv"
    meta_path = outdir / f"{args.label}.meta.json"

    meta: dict = {}
    stats: list[str] = []
    warnings: list[str] = []

    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        if not args.no_reset:
            hard_reset(ser)
        ser.reset_input_buffer()
        print(f"[log] mo {args.port} @ {args.baud}; dang cho firmware khoi dong...")

        # --- Pha 1: doc dong meta cho toi '# cols:' ---
        cols = None
        t0 = time.monotonic()
        while cols is None:
            if time.monotonic() - t0 > args.boot_timeout:
                print("[log] LOI: khong thay dong '# cols:'. Kiem tra baud, firmware, hoac bo --no-reset.")
                return 1
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if not line.startswith("#"):
                continue  # du lieu tu lan chay truoc / rac khi reset
            print(f"  {line}")
            if line.startswith("# ERROR"):
                return 1
            if line.startswith("# cols:"):
                cols = line.split(":", 1)[1].split(",")
            elif line.startswith("# WARN"):
                warnings.append(line)
            elif "=" in line:
                parse_meta(line, meta)

        if len(cols) != N_COLS:
            print(f"[log] LOI: firmware khai bao {len(cols)} cot, script mong {N_COLS}.")
            return 1

        # --- Pha 2: ghi du lieu ---
        fs = float(meta.get("fs_hz", 100.0))
        n_target = int(round(args.duration * fs))
        n_rows = n_bad = 0
        print(f"[log] ghi {n_target} mau (~{args.duration:.0f} s) vao {csv_path}")
        with open(csv_path, "w", newline="") as f:
            f.write(",".join(cols) + "\n")
            last_print = time.monotonic()
            while n_rows < n_target:
                raw = ser.readline()
                if not raw:
                    print("[log] CANH BAO: 1 s khong co du lieu")
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                if line.startswith("#"):
                    (stats if "STAT" in line else warnings).append(line)
                    continue
                parts = line.split(",")
                try:
                    if len(parts) != N_COLS:
                        raise ValueError
                    [float(p) for p in parts]
                except ValueError:
                    n_bad += 1  # dong hong (vd mat ky tu do baud qua cao)
                    continue
                f.write(line + "\n")
                n_rows += 1
                if time.monotonic() - last_print >= 1.0:
                    last_print = time.monotonic()
                    print(f"\r[log] {n_rows}/{n_target} mau, dong hong={n_bad}", end="", flush=True)
        print(f"\r[log] xong: {n_rows} mau, dong hong={n_bad}            ")

    meta_out = {
        "label": args.label,
        "recorded_at": datetime.now().isoformat(timespec="seconds"),
        "port": args.port,
        "baud": args.baud,
        "n_rows": n_rows,
        "n_malformed_lines": n_bad,
        "firmware": meta,
        "firmware_stat_lines": stats[-3:],
        "firmware_warnings": warnings,
    }
    meta_path.write_text(json.dumps(meta_out, indent=2, ensure_ascii=False))
    print(f"[log] meta -> {meta_path}")
    if warnings:
        print("[log] Firmware co canh bao:", *warnings, sep="\n  ")
    return 0


if __name__ == "__main__":
    sys.exit(main())
