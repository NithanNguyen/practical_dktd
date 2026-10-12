#!/usr/bin/env python3
"""
Buoi 1 - THUC HANH VE NHA (phan Python)

1) Doc du lieu lay mau 100 Hz tu ESP32 qua Serial, luu ra CSV.
2) Ve do thi tin hieu theo thoi gian.
3) Tinh do tre (latency) va jitter cua chu ky lay mau, luu hinh + bang so lieu.

Cach dung:
  # Ghi 30 s du lieu roi phan tich luon
  python3 hw1_logger.py --port /dev/ttyUSB0 --duration 30 --out run1

  # Chi phan tich lai file CSV da co
  python3 hw1_logger.py --analyze run1/data.csv

Dinh dang dong du lieu tu ESP32:  seq,t_isr_us,t_read_us,raw
Dong bat dau bang '#' la dong tieu de / thong bao.

Nguyen tac do: moi moc thoi gian deu lay tren ESP32 (micros()), KHONG dung
dong ho cua PC, vi do tre USB-Serial va bo lap lich cua Linux lon va dao dong
hon nhieu so voi jitter can do.
"""
import argparse
import csv
import os
import sys
import time

import numpy as np

EXPECTED_HEADER = "# seq,t_isr_us,t_read_us,raw"
U32 = 2 ** 32


# --------------------------------------------------------------------------
# 1) Ghi du lieu
# --------------------------------------------------------------------------
def reset_board(ser):
    """Reset ESP32 qua chan EN bang xung RTS, giong 'HardReset' cua esptool
    (esptool v4.7: _setRTS(True) # EN->LOW ... _setRTS(False)).
    DTR giu False de IO0 = HIGH -> chip khoi dong binh thuong, khong vao bootloader."""
    ser.dtr = False
    ser.rts = True          # EN -> LOW (chip bi giu reset)
    time.sleep(0.1)
    ser.rts = False         # EN -> HIGH (chip chay lai tu dau, in lai dong tieu de)


def parse_data_line(line):
    parts = line.split(",")
    if len(parts) != 4:
        return None
    try:
        return [int(p) for p in parts]
    except ValueError:
        return None


def record(port, baud, duration_s, csv_path, header_timeout_s=5.0, do_reset=True):
    import serial  # pyserial

    os.makedirs(os.path.dirname(os.path.abspath(csv_path)), exist_ok=True)

    # Dat DTR/RTS = False TRUOC khi mo cong (pyserial ap dung trang thai nay khi open()).
    # Mac dinh pyserial de ca hai = True; tren mach auto-reset kieu NodeMCU-32S,
    # ca hai cung active thi chip KHONG bi reset -> se khong thay dong tieu de.
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, baud, 1
    ser.dtr = False
    ser.rts = False
    try:
        ser.exclusive = True   # bao loi ngay neu Serial Monitor dang giu cong
    except (AttributeError, ValueError):
        pass
    try:
        ser.open()
    except serial.SerialException as e:
        sys.exit(f"[x] Khong mo duoc {port}: {e}\n"
                 "    -> Dong Serial Monitor cua PlatformIO / chuong trinh khac dang dung cong.")

    with ser:
        if do_reset:
            try:
                reset_board(ser)
                print("[i] Da reset ESP32 qua RTS -> EN.")
            except (OSError, serial.SerialException) as e:
                print(f"[!] Khong reset duoc qua RTS ({e}); hay nhan nut EN tren board.")
        ser.reset_input_buffer()
        print(f"[i] Da mo {port} @ {baud}. Cho dong tieu de tu ESP32 "
              f"(toi da {header_timeout_s:.0f} s) ...")

        # Cho dong tieu de; neu het thoi gian ma van co dong du lieu hop le
        # thi van ghi tiep (firmware dang chay, chi la khong bi reset).
        t0 = time.monotonic()
        pending = []
        while True:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            if line.startswith("#"):
                print("   ", line)
                if line == EXPECTED_HEADER:
                    pending = []
                    break
                continue
            vals = parse_data_line(line) if line else None
            if vals is not None:
                pending.append(vals)
            if time.monotonic() - t0 > header_timeout_s:
                if pending:
                    print(f"[!] Khong thay dong tieu de nhung da nhan {len(pending)} dong "
                          "du lieu hop le -> tiep tuc ghi (board khong duoc reset).")
                    break
                sys.exit("[x] Khong nhan duoc du lieu nao. Kiem tra: dung cong? baudrate? "
                         "firmware da nap? Thu nhan nut EN tren board.")

        n_ok = n_bad = 0
        t_start = time.monotonic()
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["seq", "t_isr_us", "t_read_us", "raw", "pc_time_s"])
            for vals in pending:                 # cac dong da nhan khi cho tieu de
                w.writerow(vals + ["nan"])
                n_ok += 1
            while time.monotonic() - t_start < duration_s:
                try:
                    line = ser.readline().decode("ascii", errors="ignore").strip()
                except (serial.SerialException, OSError) as e:
                    print(f"[!] Mat ket noi serial ({e}) -> dung ghi, giu du lieu da co.")
                    break
                if not line:
                    continue
                if line.startswith("#"):
                    print("   ", line)          # canh bao tu firmware
                    continue
                vals = parse_data_line(line)
                if vals is None:
                    n_bad += 1
                    continue
                # pc_time_s chi de tham khao, KHONG dung de tinh jitter
                w.writerow(vals + [f"{time.monotonic() - t_start:.6f}"])
                n_ok += 1
                if n_ok % 500 == 0:
                    print(f"    {n_ok} mau ...")
    print(f"[i] Da ghi {n_ok} mau vao {csv_path} (bo qua {n_bad} dong loi).")
    return csv_path


# --------------------------------------------------------------------------
# 2) Phan tich
# --------------------------------------------------------------------------
def unwrap_u32(x):
    """micros() cua ESP32 la 32-bit, tran sau ~71.6 phut -> go tran."""
    x = np.asarray(x, dtype=np.int64)
    d = np.diff(x)
    wraps = np.concatenate(([0], np.cumsum(d < -(U32 // 2))))
    return x + wraps * U32


def load_csv(csv_path):
    data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                         encoding="ascii")
    seq = unwrap_u32(data["seq"])
    t_isr = unwrap_u32(data["t_isr_us"])
    t_read = unwrap_u32(data["t_read_us"])
    raw = np.asarray(data["raw"], dtype=float)
    return seq, t_isr, t_read, raw


def int_bins(x, max_bins=80):
    """Du lieu la so nguyen (us): dung bin rong 1 us neu du it, tranh 'rang cua'."""
    lo, hi = int(np.floor(np.min(x))), int(np.ceil(np.max(x)))
    if hi - lo + 1 <= max_bins:
        return np.arange(lo - 0.5, hi + 1.5, 1.0)
    return max_bins


def describe(x):
    x = np.asarray(x, dtype=float)
    return {
        "N": int(x.size),
        "mean": float(np.mean(x)),
        "std": float(np.std(x, ddof=1)) if x.size > 1 else float("nan"),
        "min": float(np.min(x)),
        "max": float(np.max(x)),
        "p99": float(np.percentile(x, 99)),
    }


def analyze(csv_path, period_us_nominal=10000.0, out_dir=None, show=False):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_dir = out_dir or os.path.dirname(os.path.abspath(csv_path))
    seq, t_isr, t_read, raw = load_csv(csv_path)
    if seq.size < 3:
        sys.exit("[x] Qua it mau de phan tich.")

    # --- Mat mau: khe ho trong seq ---
    dseq = np.diff(seq)
    n_lost = int(np.sum(dseq - 1))
    consecutive = dseq == 1            # chi tinh chu ky giua 2 mau lien tiep

    # --- Chu ky thuc te va jitter ---
    Ts = np.diff(t_isr)[consecutive].astype(float)   # us
    err = Ts - period_us_nominal                     # sai lech chu ky
    # TIE (time interval error): lech so voi luoi thoi gian ly tuong
    k = seq - seq[0]
    tie = (t_isr - t_isr[0]) - k * period_us_nominal
    # Loai bo xu huong tuyen tinh (sai lech tan so hang so) de thay jitter thuan
    coef = np.polyfit(k, tie, 1)
    tie_detrended = tie - np.polyval(coef, k)

    # --- Do tre: tu luc vao ISR den khi co ket qua ADC ---
    lat = (t_read - t_isr).astype(float)

    duration_s = (t_isr[-1] - t_isr[0]) / 1e6
    fs_eff = (seq[-1] - seq[0]) / duration_s if duration_s > 0 else float("nan")

    s_T, s_err, s_lat = describe(Ts), describe(np.abs(err)), describe(lat)
    pp_jitter = s_T["max"] - s_T["min"]

    # --- Bang ket qua (Markdown, dan thang vao bao cao) ---
    md = []
    md.append(f"# Ket qua do: {os.path.basename(csv_path)}\n")
    md.append(f"- So mau: {seq.size}, thoi gian ghi: {duration_s:.3f} s")
    md.append(f"- Tan so danh nghia: {1e6 / period_us_nominal:.3f} Hz; "
              f"tan so hieu dung (theo dong ho ESP32): {fs_eff:.4f} Hz")
    md.append(f"- So mau bi mat (khe ho seq): {n_lost}")
    md.append(f"- Do lech tan so tuyen tinh uoc luong: "
              f"{coef[0]:.4f} us/mau ({coef[0] / period_us_nominal * 1e6:.1f} ppm)\n")
    md.append("| Dai luong (us) | N | Trung binh | Do lech chuan | Min | Max | P99 |")
    md.append("|---|---|---|---|---|---|---|")
    for name, s in [("Chu ky T_k", s_T), ("abs(T_k - T_nom)", s_err),
                    ("Do tre ISR->ADC xong", s_lat)]:
        md.append(f"| {name} | {s['N']} | {s['mean']:.2f} | {s['std']:.2f} | "
                  f"{s['min']:.0f} | {s['max']:.0f} | {s['p99']:.0f} |")
    md.append(f"\n- Jitter dinh-dinh cua chu ky (max - min): {pp_jitter:.0f} us")
    md.append(f"- Jitter RMS cua chu ky (do lech chuan T_k): {s_T['std']:.2f} us")
    md.append(f"- TIE sau khi bo xu huong: RMS {np.std(tie_detrended):.2f} us, "
              f"max |TIE| {np.max(np.abs(tie_detrended)):.1f} us")
    md.append("\nLuu y: phep do dung chinh dong ho cua ESP32 (micros()), nen "
              "chi phan anh jitter do phan mem/ngat, khong phan anh sai so "
              "tuyet doi cua thach anh. Muon do doc lap: dung oscilloscope/"
              "logic analyzer tren chan DEBUG_PIN (GPIO25).")
    report = "\n".join(md)
    stats_path = os.path.join(out_dir, "stats.md")
    with open(stats_path, "w") as f:
        f.write(report + "\n")
    print(report)

    # --- Hinh 1: tin hieu theo thoi gian ---
    t_s = (t_isr - t_isr[0]) / 1e6
    fig1, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t_s, raw, lw=0.8)
    ax.set_xlabel("Thoi gian (s) - theo moc t_isr cua ESP32")
    ax.set_ylabel("Gia tri ADC (0-4095)")
    ax.set_title(f"Tin hieu cam bien, fs = {1e6 / period_us_nominal:.0f} Hz")
    ax.grid(alpha=0.3)
    fig1.tight_layout()
    p1 = os.path.join(out_dir, "signal.png")
    fig1.savefig(p1, dpi=150)

    # --- Hinh 2: chu ky, histogram chu ky, histogram do tre ---
    fig2, axs = plt.subplots(3, 1, figsize=(10, 9))
    axs[0].plot(Ts, ".", ms=2)
    axs[0].axhline(period_us_nominal, color="k", lw=0.8, ls="--",
                   label="T danh nghia")
    axs[0].set_xlabel("Chi so chu ky k")
    axs[0].set_ylabel("T_k (us)")
    axs[0].set_title("Chu ky lay mau thuc te T_k = t_isr[k] - t_isr[k-1]")
    axs[0].ticklabel_format(axis="y", useOffset=False, style="plain")
    axs[0].legend(); axs[0].grid(alpha=0.3)

    axs[1].hist(err, bins=int_bins(err))
    axs[1].set_xlabel("T_k - T_nom (us)")
    axs[1].set_ylabel("So lan")
    axs[1].set_title("Phan bo sai lech chu ky (jitter)")
    axs[1].grid(alpha=0.3)

    axs[2].hist(lat, bins=int_bins(lat))
    axs[2].set_xlabel("t_read - t_isr (us)")
    axs[2].set_ylabel("So lan")
    axs[2].set_title("Phan bo do tre tu ngat den khi co ket qua ADC")
    axs[2].grid(alpha=0.3)
    fig2.tight_layout()
    p2 = os.path.join(out_dir, "timing.png")
    fig2.savefig(p2, dpi=150)

    print(f"[i] Da luu: {p1}, {p2}, {stats_path}")
    if show:
        plt.show()


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="VD /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--duration", type=float, default=30.0, help="giay")
    ap.add_argument("--out", default="run1", help="thu muc ket qua")
    ap.add_argument("--analyze", metavar="CSV", help="chi phan tich file CSV")
    ap.add_argument("--period-us", type=float, default=10000.0)
    ap.add_argument("--show", action="store_true", help="mo cua so do thi")
    ap.add_argument("--no-reset", action="store_true",
                    help="khong reset ESP32 qua RTS khi mo cong")
    a = ap.parse_args()

    if a.analyze:
        analyze(a.analyze, a.period_us, show=a.show)
        return
    if not a.port:
        ap.error("can --port (hoac --analyze CSV)")
    csv_path = os.path.join(a.out, "data.csv")
    record(a.port, a.baud, a.duration, csv_path, do_reset=not a.no_reset)
    analyze(csv_path, a.period_us, out_dir=a.out, show=a.show)


if __name__ == "__main__":
    main()