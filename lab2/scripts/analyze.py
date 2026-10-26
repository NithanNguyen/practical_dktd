#!/usr/bin/env python3
"""Buoi 2 - Phan tich offline: so sanh goc tho / Moving Average / Complementary Filter.

Cach dung (tu thu muc lab2/):
    python scripts/analyze.py                         # mac dinh: data/static.csv data/vibration.csv
    python scripts/analyze.py data/*.csv --axis pitch

Viec script lam, cho tung file CSV:
  1. Kiem tra chat luong lay mau: mat mau (khe ho seq), chu ky dt, jitter.
  2. Tinh LAI ca 3 bo loc bang Python tu du lieu tho (cung cong thuc, cung tham so
     doc tu .meta.json) va so sanh voi gia tri ESP32 da tinh -> kiem chung cai dat.
  3. Tinh chi so dinh luong: do lech chuan, bien do dinh-dinh, troi, muc giam nhieu (dB),
     va RMS sai so so voi goc tham chieu (lay tu lan do dung yen).
  4. Ve do thi chong (raw / MA / CF) + do thi rieng cho tich phan gyro (troi).

Ket qua trong results/: *.png, metrics.csv, metrics.md, verification.md
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# Mau: raw = xam trung tinh; cac bo loc = 3 slot dau cua bang mau categorical
# (da kiem dinh phan biet duoc voi mu mau theo moi cap).
COLORS = {"raw": "#9b9a94", "ma": "#eb6834", "cf": "#2a78d6", "gyro": "#1baf7a"}
NAMES = {"raw": "Raw (atan2 gia tốc kế)", "ma": "Moving Average", "cf": "Complementary",
         "gyro": "Chỉ tích phân gyro"}
TEXT, GRID = "#52514e", "#e4e3df"
TOL_DEG = 0.01   # sai khac cho phep ESP32 (float32) vs Python (float64)


# ----------------------------------------------------------------------------- I/O
def load(csv_path: Path) -> tuple[dict, dict]:
    arr = np.genfromtxt(csv_path, delimiter=",", names=True)
    data = {k: np.asarray(arr[k], dtype=float) for k in arr.dtype.names}
    meta_path = csv_path.with_suffix(".meta.json")
    if not meta_path.exists():
        sys.exit(f"Thieu {meta_path} (file nay do log_serial.py tao ra).")
    meta = json.loads(meta_path.read_text())
    return data, meta


# ------------------------------------------------------------------- bo loc offline
# Ban Python cua src/imu_logger/filters.h - cung cong thuc, cung cach khoi tao.
def roll_from_acc(ay, az):
    return np.degrees(np.arctan2(ay, az))


def pitch_from_acc(ax, ay, az):
    return np.degrees(np.arctan2(-ax, np.sqrt(ay * ay + az * az)))


def moving_average(x: np.ndarray, n: int) -> np.ndarray:
    """Trung binh N mau gan nhat; khi chua du N mau thi chia cho so mau dang co."""
    c = np.cumsum(np.insert(x, 0, 0.0))
    idx = np.arange(1, len(x) + 1)
    lo = np.maximum(idx - n, 0)
    return (c[idx] - c[lo]) / (idx - lo)


def complementary(acc_angle, rate_dps, dt, alpha):
    out = np.empty_like(acc_angle)
    out[0] = acc_angle[0]
    for k in range(1, len(acc_angle)):
        out[k] = alpha * (out[k - 1] + rate_dps[k] * dt[k]) + (1 - alpha) * acc_angle[k]
    return out


def gyro_integrate(acc_angle, rate_dps, dt):
    # dt[0] = 0 nen out[0] = acc_angle[0] (khoi tao giong firmware)
    return acc_angle[0] + np.cumsum(rate_dps * dt)


# --------------------------------------------------------------------- phan tich
def timing(d: dict) -> dict:
    seq = d["seq"].astype(np.int64)
    t_us = d["t_us"].astype(np.int64)
    dt_us = np.mod(np.diff(t_us), 2 ** 32)        # micros() 32-bit co the tran
    gaps = np.diff(seq) - 1
    return {
        "n": len(seq),
        "lost": int(gaps[gaps > 0].sum()),
        "dt": np.concatenate([[0.0], dt_us * 1e-6]),
        "dt_mean_ms": float(dt_us.mean() / 1e3),
        "dt_std_us": float(dt_us.std()),
        "dt_max_ms": float(dt_us.max() / 1e3),
    }


def recompute(d: dict, fw: dict, axis: str, dt: np.ndarray) -> dict:
    n = int(fw["ma_n"])
    alpha = float(fw["cf_alpha"])
    if axis == "roll":
        acc = roll_from_acc(d["ay"], d["az"])
        rate = fw["gyro_sign_roll"] * (d["gx"] - fw["gyro_bias_x"])
    else:
        acc = pitch_from_acc(d["ax"], d["ay"], d["az"])
        rate = fw["gyro_sign_pitch"] * (d["gy"] - fw["gyro_bias_y"])
    return {"raw": acc, "ma": moving_average(acc, n), "cf": complementary(acc, rate, dt, alpha),
            "gyro": gyro_integrate(acc, rate, dt), "rate": rate}


def gyro_sign_check(acc: np.ndarray, rate: np.ndarray, dt: np.ndarray, n: int) -> str:
    """Tuong quan giua d(goc gia toc ke)/dt (da lam muot) va omega gyro (da nhan dau).
    Dung dau: r > 0. Can chuyen dong nghieng ro (vd Buoc 5), tinh/rung tai cho thi khong ket luan."""
    if np.std(acc) < 2.0:
        return "chưa đủ chuyển động"
    sm = moving_average(acc, n)
    d_acc = np.gradient(sm, np.cumsum(dt))
    r = float(np.corrcoef(d_acc[n:], rate[n:])[0, 1])
    if r > 0.5:
        return f"khớp (r={r:.2f})"
    if r < -0.5:
        return f"NGƯỢC (r={r:.2f}) → đổi GYRO_SIGN"
    return f"không rõ (r={r:.2f})"


def metrics(sig: np.ndarray, t: np.ndarray, ref: float | None) -> dict:
    span = t[-1] - t[0]
    w = min(10.0, span / 3)                        # cua so dau/cuoi de do troi
    head, tail = sig[t <= t[0] + w], sig[t >= t[-1] - w]
    m = {"mean": sig.mean(), "std": sig.std(ddof=1), "p2p": np.ptp(sig),
         "drift": tail.mean() - head.mean()}
    m["rms_ref"] = float(np.sqrt(np.mean((sig - ref) ** 2))) if ref is not None else np.nan
    return m


# -------------------------------------------------------------------------- ve
def style(ax):
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=TEXT, labelsize=8)
    ax.yaxis.label.set_color(TEXT)
    ax.xaxis.label.set_color(TEXT)


def plot_run(label: str, axis: str, t, sig: dict, outdir: Path, zoom_s: float) -> Path:
    fig, axs = plt.subplots(3, 1, figsize=(10, 9), constrained_layout=True,
                            gridspec_kw={"height_ratios": [3, 2, 2]})
    unit = f"Góc {axis} (°)"

    # (a) toan bo ban ghi: raw / MA / CF
    a = axs[0]
    a.plot(t, sig["raw"], color=COLORS["raw"], lw=0.8, label=NAMES["raw"])
    a.plot(t, sig["ma"], color=COLORS["ma"], lw=1.5, label=NAMES["ma"])
    a.plot(t, sig["cf"], color=COLORS["cf"], lw=1.5, label=NAMES["cf"])
    lo, hi = np.percentile(sig["raw"], [0.5, 99.5])
    pad = max(0.2, 0.15 * (hi - lo))
    a.set_ylim(lo - pad, hi + pad)
    a.set_title(f"[{label}] {axis}: raw so với Moving Average và Complementary (toàn bộ bản ghi)",
                fontsize=10, color=TEXT, loc="left")
    a.set_ylabel(unit)
    a.legend(loc="upper right", fontsize=8, frameon=False, ncol=3)

    # (b) phong to mot doan giua ban ghi
    b = axs[1]
    t_mid = t[0] + (t[-1] - t[0]) / 2
    m = (t >= t_mid) & (t <= t_mid + zoom_s)
    for k in ("raw", "ma", "cf"):
        b.plot(t[m], sig[k][m], color=COLORS[k], lw=0.9 if k == "raw" else 1.6,
               marker="o" if k == "raw" else None, ms=2.5, label=NAMES[k])
    b.set_title(f"Phóng to {zoom_s:.0f} s giữa bản ghi (độ trễ của MA, độ mượt của CF)",
                fontsize=10, color=TEXT, loc="left")
    b.set_ylabel(unit)

    # (c) tich phan gyro thuan tuy: troi theo thoi gian (truc y rieng vi co the troi xa)
    c = axs[2]
    c.plot(t, sig["gyro"], color=COLORS["gyro"], lw=1.4, label=NAMES["gyro"])
    c.plot(t, sig["cf"], color=COLORS["cf"], lw=1.2, label=NAMES["cf"])
    c.set_title("Vì sao cần Complementary: tích phân gyro riêng lẻ bị trôi/lệch",
                fontsize=10, color=TEXT, loc="left")
    c.set_ylabel(unit)
    c.set_xlabel("Thời gian (s)")
    c.legend(loc="best", fontsize=8, frameon=True, facecolor="white", edgecolor="none")

    for ax in axs:
        style(ax)
    out = outdir / f"{label}_{axis}.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def plot_summary(rows: list[dict], axis: str, outdir: Path) -> Path:
    """Moi dieu kien 1 o, truc y rieng (bien do rung lon hon tinh hang chuc lan)."""
    labels = sorted({r["label"] for r in rows if r["axis"] == axis},
                    key=lambda s: (s != "static", s))
    keys = ["raw", "ma", "cf"]
    fig, axs = plt.subplots(1, len(labels), figsize=(4.2 * len(labels), 3.8),
                            constrained_layout=True, squeeze=False)
    for ax, lb in zip(axs[0], labels):
        vals = [next(r["std"] for r in rows if r["label"] == lb and r["axis"] == axis
                     and r["signal"] == k) for k in keys]
        bars = ax.bar(range(len(keys)), vals, width=0.7, color=[COLORS[k] for k in keys])
        for bar, v in zip(bars, vals):
            ax.annotate(f"{v:.3f}°", (bar.get_x() + bar.get_width() / 2, v), ha="center",
                        va="bottom", fontsize=8, color=TEXT, xytext=(0, 2), textcoords="offset points")
        ax.set_xticks(range(len(keys)), ["Raw", "MA", "CF"])
        ax.set_title(f"{lb}", fontsize=10, color=TEXT, loc="left")
        ax.set_ylim(0, max(vals) * 1.18)
        style(ax)
    axs[0][0].set_ylabel(f"Độ lệch chuẩn góc {axis} (°)")
    fig.suptitle(f"Độ ổn định theo điều kiện thí nghiệm ({axis}) — thấp hơn là ổn định hơn",
                 fontsize=10, color=TEXT, x=0.01, ha="left")
    out = outdir / f"summary_std_{axis}.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


# ------------------------------------------------------------------------- main
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="*", default=["data/static.csv", "data/vibration.csv"])
    ap.add_argument("--axis", choices=["roll", "pitch", "both"], default="roll")
    ap.add_argument("--outdir", default="results")
    ap.add_argument("--settle-s", type=float, default=2.0,
                    help="bo qua N giay dau (MA chua du cua so, CF dang hoi tu ~4*tau)")
    ap.add_argument("--zoom-s", type=float, default=3.0)
    ap.add_argument("--ref-label", default="static",
                    help="ban ghi dung yen dung lam goc tham chieu cho RMS sai so")
    args = ap.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    axes = ["roll", "pitch"] if args.axis == "both" else [args.axis]

    runs = []
    for p in map(Path, args.csv):
        if not p.exists():
            sys.exit(f"Khong thay {p}")
        d, meta = load(p)
        runs.append((meta.get("label", p.stem), d, meta))

    rows, verif, figs = [], [], []
    for axis in axes:
        # Goc tham chieu = trung binh goc raw khi dung yen (nhieu gia toc ke co trung binh ~0)
        ref = None
        for label, d, meta in runs:
            if label == args.ref_label:
                tm = timing(d)
                sig = recompute(d, meta["firmware"], axis, tm["dt"])
                t = np.cumsum(tm["dt"])
                ref = float(sig["raw"][t >= args.settle_s].mean())

        for label, d, meta in runs:
            fw = meta["firmware"]
            tm = timing(d)
            t = np.cumsum(tm["dt"])
            sig = recompute(d, fw, axis, tm["dt"])

            # Kiem chung: ESP32 vs Python
            col = {"raw": f"{axis}_acc", "ma": f"{axis}_ma", "cf": f"{axis}_cf", "gyro": f"{axis}_gyro"}
            diffs = {k: float(np.max(np.abs(sig[k] - d[col[k]]))) for k in col}
            sign = gyro_sign_check(sig["raw"], sig["rate"], tm["dt"], int(fw["ma_n"]))
            verif.append((label, axis, diffs, tm, meta, sign))

            keep = t >= args.settle_s
            for k in ("raw", "ma", "cf", "gyro"):
                m = metrics(sig[k][keep], t[keep], ref)
                std_raw = np.std(sig["raw"][keep], ddof=1)
                # Gyro thuan tuy it nhieu nhung troi/lech -> so sanh bang std la sai ban chat
                m["reduction_db"] = {"raw": 0.0, "gyro": np.nan}.get(k, 20 * np.log10(std_raw / m["std"]))
                rows.append({"label": label, "axis": axis, "signal": k, **m})
            figs.append(plot_run(label, axis, t, sig, outdir, args.zoom_s))
        figs.append(plot_summary(rows, axis, outdir))

    # ---- ghi bang chi so ----
    hdr = ["label", "axis", "signal", "mean", "std", "p2p", "drift", "reduction_db", "rms_ref"]
    with open(outdir / "metrics.csv", "w") as f:
        f.write(",".join(hdr) + "\n")
        for r in rows:
            f.write(",".join(str(r[h]) if isinstance(r[h], str) else f"{r[h]:.5f}" for h in hdr) + "\n")

    md = ["| Điều kiện | Trục | Tín hiệu | Trung bình (°) | Độ lệch chuẩn (°) | Đỉnh-đỉnh (°) "
          "| Trôi (°) | Giảm nhiễu (dB) | RMS so với tham chiếu (°) |",
          "|---|---|---|---:|---:|---:|---:|---:|---:|"]
    for r in rows:
        md.append(f"| {r['label']} | {r['axis']} | {NAMES[r['signal']]} | {r['mean']:.3f} | {r['std']:.4f} "
                  f"| {r['p2p']:.3f} | {r['drift']:+.3f} | {'—' if np.isnan(r['reduction_db']) else format(r['reduction_db'], '.1f')} | {r['rms_ref']:.4f} |")
    (outdir / "metrics.md").write_text("\n".join(md) + "\n")

    vm = ["| Điều kiện | Trục | Số mẫu | Mất mẫu | dt TB (ms) | Jitter std (µs) | dt max (ms) "
          "| max(abs(ESP32 − Python)) raw / MA / CF / gyro (°) | Dòng hỏng | Dấu gyro | Kết luận |",
          "|---|---|---:|---:|---:|---:|---:|---|---:|---|---|"]
    ok_all = True
    for label, axis, diffs, tm, meta, sign in verif:
        ok = all(v < TOL_DEG for v in diffs.values()) and tm["lost"] == 0 and "NGƯỢC" not in sign
        ok_all &= ok
        vm.append(f"| {label} | {axis} | {tm['n']} | {tm['lost']} | {tm['dt_mean_ms']:.3f} | "
                  f"{tm['dt_std_us']:.1f} | {tm['dt_max_ms']:.2f} | "
                  + " / ".join(f"{diffs[k]:.2e}" for k in ("raw", "ma", "cf", "gyro"))
                  + f" | {meta.get('n_malformed_lines', 0)} | {sign} | {'ĐẠT' if ok else 'XEM LẠI'} |")
    (outdir / "verification.md").write_text("\n".join(vm) + "\n")

    print("\n".join(vm), "\n")
    print("\n".join(md), "\n")
    if ref is None:
        print(f"(Khong co ban ghi '{args.ref_label}' -> cot RMS so voi tham chieu = nan)")
    print("Do thi:", *map(str, figs), sep="\n  ")
    print("\nKIEM CHUNG:", "DAT - ESP32 va Python cho cung ket qua, khong mat mau" if ok_all
          else "CO MUC CAN XEM LAI (xem results/verification.md)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
