#!/usr/bin/env python3
"""Lab 3 CE212 - tu dong hoa Buoc 4 den Buoc 7 va tao bao cao.

  python3 lab3.py calib                  Buoc 4a  hieu chuan CPR (quay tay 10 vong x 3 lan)
  python3 lab3.py verify                 Buoc 4b  chay dong co 40 s de quay video
  python3 lab3.py verify --video v.mp4   Buoc 4b  dem vong tu video va so sanh voi encoder
  python3 lab3.py ident                  Buoc 5   nhan dang doi tuong (tu dong, ~1 phut)
  python3 lab3.py tune                   Buoc 6   Ziegler-Nichols + thu-sai co he thong (tu dong, ~3 phut)
  python3 lab3.py dist --elec            Buoc 7a  tai dot ngot tuong duong bang dien (tu dong)
  python3 lab3.py dist                   Buoc 7b  tai dot ngot bang tay (theo loi nhac)
  python3 lab3.py report                 tao thu muc report/ (bang + do thi + bao cao .md)
  python3 lab3.py all                    chay lan luot tat ca
  python3 lab3.py analyze FILE.csv       phan tich lai mot file CSV bat ky

Ket qua cua moi buoc duoc luu vao lab3_config.json de buoc sau dung lai.
"""
import argparse
import datetime
import json
import os
import re
import shutil
import sys
import time

import numpy as np

import lab3_analysis as A
import lab3_plots as P
from lab3_link import DeviceError, Link, find_port, save_csv

CFG_PATH = "lab3_config.json"
LOG = "logs"
DEFAULT = {
    "port": "/dev/ttyUSB0", "baud": 115200, "win": 4, "sign": 1, "cpr": None,
    # Nguong chap nhan: lab_3.pdf KHONG quy dinh. Day la de xuat, sua o day neu giang vien yeu cau khac.
    "criteria": {"pot_max": 10.0, "ess_max_pct": 2.0, "band": 0.05, "u_noise_max": 3.0},
    "sp_frac": 0.55,
}


# =============================================================== tien ich
def load_cfg():
    cfg = json.loads(json.dumps(DEFAULT))
    if os.path.exists(CFG_PATH):
        with open(CFG_PATH) as f:
            user = json.load(f)
        crit = dict(cfg["criteria"])
        crit.update(user.get("criteria", {}))
        cfg.update(user)
        cfg["criteria"] = crit
    return cfg


def save_cfg(cfg):
    with open(CFG_PATH, "w") as f:
        json.dump(cfg, f, ensure_ascii=False, indent=2, default=_json_default)


def _json_default(o):
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    raise TypeError(type(o))


def stamp():
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def head(txt):
    print("\n" + "=" * 72 + "\n" + txt + "\n" + "=" * 72)


def need(cfg, key, step):
    if not cfg.get(key):
        raise SystemExit("Thiếu kết quả '{}'. Chạy trước: python3 lab3.py {}".format(key, step))


def open_link(cfg, args):
    port = find_port(getattr(args, "port", None) or cfg.get("port"))
    if port != cfg.get("port"):
        cfg["port"] = port
        save_cfg(cfg)
    print("Kết nối {} ...".format(port))
    return Link(port, cfg["baud"], verbose=getattr(args, "verbose", False))


def ask(msg):
    try:
        return input(msg)
    except EOFError:
        return ""


def fmt(v, spec="{:.1f}", nan="—"):
    return nan if v is None or (isinstance(v, float) and not np.isfinite(v)) else spec.format(v)


def mean_std(vals):
    a = np.array([v for v in vals if v is not None and np.isfinite(v)], dtype=float)
    if len(a) == 0:
        return (float("nan"), float("nan"))
    return (float(a.mean()), float(a.std(ddof=1)) if len(a) > 1 else 0.0)


def ensure_sign(link, cfg):
    """Tu kiem tra chieu: chay vong ho 1 s, neu rpm am thi doi dau."""
    if cfg.get("sign_checked"):
        return
    link.configure(cfg)
    link.send("ol 40")
    link.wait(1.2)
    v = link.mean_rpm(0.5)
    link.send("stop")
    link.wait_stopped()
    if abs(v) < 1.0:
        raise SystemExit("Động cơ không quay hoặc encoder không có tín hiệu (rpm ≈ 0 ở duty 40 %). "
                         "Kiểm tra adapter 12V, dây động cơ và dây encoder.")
    if v < 0:
        cfg["sign"] = -cfg["sign"]
        print("Vận tốc đo ra âm khi quay thuận -> tự đổi dấu encoder (sign = {}).".format(cfg["sign"]))
    cfg["sign_checked"] = True
    save_cfg(cfg)


def chatter(link):
    """Phat hien dao dong bang-bang (u nhay lien tuc giua 0 va 100 %) de dung som cho an toan."""
    a = link.last(60)
    if len(a) < 60:
        return False
    u = a[:, 4]
    s = np.where(u >= 98, 1, np.where(u <= 2, -1, 0))
    s = s[s != 0]
    return len(s) > 1 and np.count_nonzero(np.diff(s)) >= 10


# =============================================================== BUOC 4a: CPR
def cmd_calib(args, cfg):
    head("BƯỚC 4a – Hiệu chuẩn số đếm encoder trên 1 vòng trục ra (CPR)")
    print("Chuẩn bị: RÚT adapter 12V (chỉ giữ cáp USB). Dán cờ băng keo lên trục ra.\n"
          "Mỗi lần đo: quay trục ra bằng tay ĐÚNG {} vòng theo một chiều.".format(args.turns))
    vals, signs = [], []
    with open_link(cfg, args) as link:
        link.configure(cfg)
        for k in range(1, args.trials + 1):
            ask("\n[Lần {}/{}] Đặt cờ ở vạch mốc, nhấn Enter...".format(k, args.trials))
            link.query("zero", "# count=0")
            ask("        Quay ĐÚNG {} vòng rồi dừng ở vạch mốc, nhấn Enter...".format(args.turns))
            line = link.query("cnt", "# count=")
            if line is None:
                raise DeviceError("Không đọc được bộ đếm.")
            c = int(re.search(r"count=(-?\d+)", line).group(1))
            if c == 0:
                raise SystemExit("Bộ đếm = 0: encoder không có tín hiệu. Kiểm tra dây xanh dương (3V3), đen (GND), "
                                 "vàng (IO32), xanh lá (IO33).")
            vals.append(abs(c) / args.turns)
            signs.append(np.sign(c))
            print("        đếm = {}  ->  {:.1f} đếm/vòng".format(c, vals[-1]))
        st = link.query("?", "# mode=")
        err = int(re.search(r"enc_err=(\d+)", st).group(1)) if st else -1
    m, s = mean_std(vals)
    spread = (max(vals) - min(vals)) / m * 100
    cfg["cpr"] = round(m, 2)
    cfg["calib"] = {"turns": args.turns, "values": vals, "mean": m, "std": s, "spread_pct": spread,
                    "enc_err": err, "gear_ratio_if_11ppr": m / 44.0}
    cfg.pop("sign_checked", None)
    save_cfg(cfg)
    print("\nKẾT QUẢ: CPR = {:.1f} ± {:.1f} đếm/vòng (độ lệch max–min {:.2f} %), enc_err = {}".format(m, s, spread, err))
    print("Nếu encoder là 11 xung/vòng/kênh (x4 = 44 đếm/vòng trục động cơ) thì tỉ số truyền ≈ 1:{:.1f}".format(m / 44))
    if spread > 1.0:
        print("CẢNH BÁO: các lần đo lệch > 1 %. Có thể đã quay thiếu/thừa. Chạy lại calib.")
    if err > 0:
        print("CẢNH BÁO: enc_err > 0 – có mất cạnh xung. Kiểm tra dây encoder.")
    print("Đã lưu CPR vào {}. Bước tiếp: python3 lab3.py verify".format(CFG_PATH))


# =============================================================== BUOC 4b: kiem chung
def cmd_verify(args, cfg):
    need(cfg, "cpr", "calib")
    if args.video:
        return verify_video(args, cfg)
    head("BƯỚC 4b – Chạy động cơ ổn định để quay video kiểm chứng")
    duty = args.duty
    if duty is None:
        duty = float(np.interp(0.5 * cfg["vmax"], cfg["static"]["rpm"], cfg["static"]["duty"])) if cfg.get("static") else 50.0
    with open_link(cfg, args) as link:
        ensure_sign(link, cfg)
        link.configure(cfg)
        i0 = link.n()
        link.send("ol {:.1f}".format(duty))
        link.wait(3.0)
        i1 = link.n()
        print("\n>>> BẮT ĐẦU QUAY VIDEO NGAY (giữ điện thoại cố định, thấy rõ cờ băng keo).")
        t0 = time.time()
        while time.time() - t0 < args.dur:
            left = args.dur - (time.time() - t0)
            sys.stdout.write("\r    còn {:4.0f} s ".format(left))
            sys.stdout.flush()
            link.wait(min(1.0, max(left, 0.01)))
        link.send("stop")
        print("\n>>> DỪNG QUAY VIDEO.")
        arr = link.segment(i0)
    i_s = i1 - i0
    rpm = arr[i_s:, 3]
    res = {"duty": duty, "rpm_enc": float(rpm.mean()), "rpm_std": float(rpm.std()), "dur": len(rpm) * A.TS}
    path = os.path.join(LOG, "verify_{}.csv".format(stamp()))
    save_csv(path, arr, {"kind": "verify", "duty": duty, "i_steady": int(i_s), "cpr": cfg["cpr"]})
    res["csv"] = path
    cfg["verify"] = res
    save_cfg(cfg)
    P.verify(arr, i_s, res["rpm_enc"], path.replace(".csv", ".png"))
    print("Encoder: {:.2f} rpm trung bình trong {:.0f} s ổn định (độ lệch chuẩn {:.2f} rpm).".format(
        res["rpm_enc"], res["dur"], res["rpm_std"]))
    print("Chép video vào máy rồi chạy: python3 lab3.py verify --video <file video>")


def verify_video(args, cfg):
    from lab3_video import count_revolutions
    need(cfg, "verify", "verify")
    head("BƯỚC 4b – Đếm vòng từ video và so sánh")
    roi = [int(v) for v in args.roi.split(",")] if args.roi else None
    plot = os.path.join(LOG, "video_count.png")
    v = count_revolutions(args.video, roi, plot)
    enc = cfg["verify"]["rpm_enc"]
    err = (enc - v["rpm"]) / v["rpm"] * 100
    cpr_implied = cfg["cpr"] * enc / v["rpm"]
    cfg["verify"].update({"video": os.path.basename(args.video), "rpm_video": v["rpm"], "n_rev": v["n_rev"],
                          "video_dur": v["duration"], "err_pct": err, "cpr_implied": cpr_implied,
                          "roi": v["roi"], "period_max_ratio": v["period_max_ratio"]})
    save_cfg(cfg)
    arr, meta = A.load_csv(cfg["verify"]["csv"])
    P.verify(arr, meta.get("i_steady", 0), enc, cfg["verify"]["csv"].replace(".csv", ".png"), v["rpm"])
    print("Video : {} vòng trong {:.2f} s -> {:.2f} rpm (ROI = {})".format(v["n_rev"], v["duration"], v["rpm"], v["roi"]))
    print("Encoder: {:.2f} rpm".format(enc))
    print("Sai lệch tương đối = {:+.2f} %".format(err))
    print("CPR suy ra từ video = {:.1f} (đang dùng {:.1f})".format(cpr_implied, cfg["cpr"]))
    if v["period_max_ratio"] > 1.5 or v["period_min_ratio"] < 0.6:
        print("CẢNH BÁO: chu kỳ giữa các lần cờ đi qua không đều -> có thể đếm sót/thừa. Xem {}".format(plot))


# =============================================================== BUOC 5: nhan dang
def cmd_ident(args, cfg):
    need(cfg, "cpr", "calib")
    head("BƯỚC 5 – Nhận dạng đối tượng (tự động)")
    d = os.path.join(LOG, "ident")
    os.makedirs(d, exist_ok=True)
    with open_link(cfg, args) as link:
        ensure_sign(link, cfg)
        link.configure(cfg)
        # 5.1 Duty khoi dong (vung chet do ma sat tinh)
        print("5.1 Tìm duty khởi động: tăng duty 1 %/0,25 s ...")
        i0 = link.n()
        dz = None
        for duty in range(0, 51):
            link.send("ol {}".format(duty))
            link.wait(0.25)
            if link.mean_rpm(0.1) > 2.0:
                dz = duty
                break
        link.send("stop")
        link.wait_stopped()
        if dz is None:
            raise SystemExit("Động cơ không quay đến duty 50 %. Kiểm tra cơ khí/nguồn.")
        save_csv(os.path.join(d, "ramp.csv"), link.segment(i0), {"kind": "ramp", "dz": dz})
        print("    duty khởi động = {} %".format(dz))
        # 5.2 Dac tinh tinh
        print("5.2 Đặc tính tĩnh: duty 10 → 100 %, mỗi mức 1,2 s ...")
        i0 = link.n()
        duties, rpms = [], []
        for duty in range(10, 101, 10):
            link.send("ol {}".format(duty))
            link.wait(1.2)
            duties.append(duty)
            rpms.append(link.mean_rpm(0.6))
            print("    u = {:3d} %  ->  {:6.1f} rpm".format(duty, rpms[-1]))
        save_csv(os.path.join(d, "static.csv"), link.segment(i0), {"kind": "static"})
        vmax = rpms[-1]
        r = float(args.sp) if args.sp else 5.0 * round(cfg["sp_frac"] * vmax / 5.0)
        mono = np.maximum.accumulate(np.array(rpms))
        u_r = float(np.interp(r, mono, duties))
        u_a = max(dz + 5.0, u_r - 10.0)
        u_b = min(100.0, u_a + 20.0)
        # 5.3 Dap ung bac vong ho quanh diem lam viec, lap 3 lan
        print("5.3 Đáp ứng bậc vòng hở u = {:.0f} → {:.0f} %, lặp {} lần ...".format(u_a, u_b, args.repeats))
        runs = []
        for k in range(args.repeats):
            link.send("ol {:.1f}".format(u_a))
            link.wait(2.0)
            i_m = link.n()
            link.send("ol {:.1f}".format(u_b))
            link.wait(2.0)
            arr = link.segment(max(i_m - 60, 0))
            i_step = A.first_index(np.abs(arr[:, 4] - u_b) < 0.05)
            fit = A.fopdt_smith(arr, i_step)
            save_csv(os.path.join(d, "step_{}.csv".format(k + 1)), arr, {"kind": "ol_step", "i_step": i_step,
                                                                          "u_a": u_a, "u_b": u_b})
            runs.append((arr, i_step, fit))
            print("    lần {}: K = {:.4f} rpm/%, T = {:.1f} ms, L = {:.1f} ms, sai số mô hình RMS = {:.1f} %".format(
                k + 1, fit["K"], fit["T"] * 1e3, fit["L"] * 1e3, fit["rmse_pct"]))
        link.send("stop")
    K, T, L = (mean_std([f[2][k] for f in runs]) for k in ("K", "T", "L"))
    k_static = (np.interp(u_b, duties, mono) - np.interp(u_a, duties, mono)) / (u_b - u_a)
    zn = A.zn_rules(K[0], T[0], L[0])
    simc = {"{:g}".format(f): A.simc_pi(K[0], T[0], L[0], f * T[0]) for f in (2, 1, 0.5, 0.25, 0.125)}
    cfg.update({"dz": dz, "vmax": vmax, "r": r, "u_r": u_r,
                "static": {"duty": duties, "rpm": rpms},
                "model": {"K": K[0], "K_std": K[1], "T": T[0], "T_std": T[1], "L": L[0], "L_std": L[1],
                          "u_a": u_a, "u_b": u_b, "K_static": float(k_static),
                          "rmse_pct": [f[2]["rmse_pct"] for f in runs],
                          "runs": [{k: f[2][k] for k in ("K", "T", "L", "rmse_pct")} for f in runs]},
                "zn": zn, "simc": simc})
    save_cfg(cfg)
    P.static_curve(duties, rpms, dz, r, u_r, os.path.join(d, "static.png"))
    P.ident(runs, os.path.join(d, "step_fit.png"))
    print("\nKẾT QUẢ")
    print("  v_max = {:.1f} rpm (u = 100 %), duty khởi động = {} %".format(vmax, dz))
    print("  Vận tốc đặt chọn r = {:.0f} rpm ({:.0f} % v_max), duty tương ứng ≈ {:.1f} %".format(r, r / vmax * 100, u_r))
    print("  Mô hình FOPDT: K = {:.4f} ± {:.4f} rpm/%, T = {:.1f} ± {:.1f} ms, L = {:.1f} ± {:.1f} ms".format(
        K[0], K[1], T[0] * 1e3, T[1] * 1e3, L[0] * 1e3, L[1] * 1e3))
    print("  K từ đặc tính tĩnh quanh điểm làm việc = {:.4f} rpm/% (so sánh để kiểm tra tuyến tính)".format(k_static))
    print("  Ziegler–Nichols:")
    for n_, g in zn.items():
        print("     {:7s} Kp = {:.4g}  Ki = {:.4g}  Kd = {:.4g}".format(n_, *g))
    print("Đồ thị: {}/static.png, {}/step_fit.png. Bước tiếp: python3 lab3.py tune".format(d, d))


# =============================================================== BUOC 6: tune
def step_duration(cfg):
    m = cfg["model"]
    return float(min(10.0, max(4.0, 20.0 * (m["T"] + max(m["L"], A.TS)))))


def passes(m, crit):
    why = []
    if m.get("aborted"):
        why.append("dao động bang-bang, dừng sớm")
    if not np.isfinite(m.get("ts", float("nan"))):
        why.append("không xác lập")
    if np.isfinite(m.get("pot", float("nan"))) and m["pot"] > crit["pot_max"]:
        why.append("POT > {:g} %".format(crit["pot_max"]))
    if abs(m.get("ess_pct", 0)) > crit["ess_max_pct"]:
        why.append("|e_ss| > {:g} %".format(crit["ess_max_pct"]))
    if m.get("u_noise", 0) > crit["u_noise_max"]:
        why.append("u nhiễu > {:g} %".format(crit["u_noise_max"]))
    return (len(why) == 0), why


def run_step(link, cfg, tag, phase, kp, ki, kd=0.0, dmeas=0, candidate=True, folder="tune"):
    r = cfg["r"]
    dur = step_duration(cfg)
    link.configure(cfg, kp, ki, kd, dmeas)
    link.send("sp 0")
    link.wait_stopped()
    link.wait(0.5)
    i0 = link.n()
    link.send("sp {:.2f}".format(r))
    completed = link.wait(dur, check=lambda: chatter(link))
    link.send("sp 0")
    link.wait(0.05)
    arr = link.segment(max(i0 - 50, 0))
    on = np.abs(arr[:, 2] - r) < 0.01
    i_step = A.first_index(on)
    i_end = int(np.where(on)[0][-1])
    arr = arr[:i_end + 1]
    m = A.step_metrics(arr, i_step, r, cfg["criteria"]["band"])
    m["aborted"] = not completed
    ok, why = passes(m, cfg["criteria"])
    path = os.path.join(LOG, folder, "{}.csv".format(tag))
    save_csv(path, arr, {"kind": "step", "tag": tag, "phase": phase, "kp": kp, "ki": ki, "kd": kd,
                         "dmeas": dmeas, "r": r, "i_step": i_step})
    tr = {"tag": tag, "phase": phase, "kp": kp, "ki": ki, "kd": kd, "dmeas": dmeas, "candidate": candidate,
          "csv": path, "pass": ok, "why": why}
    tr.update({k: m[k] for k in ("pot", "tr", "ts", "ess", "ess_pct", "iae", "u_noise", "u_sat", "noise_rpm", "yss")})
    tr["aborted"] = m["aborted"]
    print("  {:<16s} Kp={:<8.4g} Ki={:<8.4g} Kd={:<8.4g} | POT={:>5s}% t_s={:>5s} ms e_ss={:>6s}% σu={:>4s}  {}".format(
        tag, kp, ki, kd, fmt(m["pot"]), fmt(m["ts"] * 1e3, "{:.0f}"), fmt(m["ess_pct"], "{:+.2f}"),
        fmt(m["u_noise"]), ("ĐẠT" if ok else "KHÔNG ĐẠT: " + "; ".join(why)) if candidate else "(đối chứng)"))
    return tr


def score(t):
    return (0 if t["pass"] else 1, t["ts"] if np.isfinite(t["ts"]) else 1e9, t["pot"] if np.isfinite(t["pot"]) else 1e9)


def cmd_tune(args, cfg):
    need(cfg, "model", "ident")
    head("BƯỚC 6 – Tune PID tự động (Ziegler–Nichols → quét có hệ thống → lặp lại)")
    m = cfg["model"]
    K, T, L = m["K"], m["T"], m["L"]
    Leff = max(L, A.TS)
    crit = cfg["criteria"]
    print("Mô hình: K = {:.4f} rpm/%, T = {:.1f} ms, L = {:.1f} ms. r = {:.0f} rpm. Mỗi lần thử {:.0f} s.".format(
        K, T * 1e3, L * 1e3, cfg["r"], step_duration(cfg)))
    print("Tiêu chí ĐẠT: POT ≤ {pot_max:g} %, |e_ss| ≤ {ess_max_pct:g} %, xác lập trong dải ±{b:g} %, "
          "σ(u) ≤ {u_noise_max:g} % duty.".format(b=crit["band"] * 100, **crit))
    if os.path.isdir(os.path.join(LOG, "tune")):
        shutil.rmtree(os.path.join(LOG, "tune"))
    trials = []
    with open_link(cfg, args) as link:
        def run(tag, phase, kp, ki, kd=0.0, dmeas=0, candidate=True):
            t = run_step(link, cfg, tag, phase, kp, ki, kd, dmeas, candidate)
            trials.append(t)
            return t

        def best():
            c = [t for t in trials if t["candidate"]]
            return min(c, key=score)

        print("\nPha A – Ziegler–Nichols (luật đáp ứng bậc, tính từ K, T, L):")
        for i, (name, g) in enumerate(cfg["zn"].items()):
            run("A{}_{}".format(i + 1, name), "A", *g)
        print("\nPha B – Quét τc của luật SIMC (từ chậm đến nhanh, dừng khi bắt đầu không đạt):")
        seen_pass = False
        for i, f in enumerate((2, 1, 0.5, 0.25, 0.125)):
            kp, ki, _ = A.simc_pi(K, T, L, f * T)
            t = run("B{}_tc{:.0f}ms".format(i + 1, f * T * 1e3), "B", kp, ki)
            if t["pass"]:
                seen_pass = True
            elif seen_pass:
                break
        b = best()
        print("\nPha C – Tinh chỉnh quanh bộ tốt nhất ({}): đổi Kp rồi đổi Ki, mỗi lần một tham số:".format(b["tag"]))
        for i, f in enumerate((0.7, 1.4)):
            run("C{}_Kp_x{:g}".format(i + 1, f), "C", b["kp"] * f, b["ki"])
        b = best()
        for i, f in enumerate((0.5, 2.0)):
            run("C{}_Ki_x{:g}".format(i + 3, f), "C", b["kp"], b["ki"] * f)
        b = best()
        print("\nPha D – Vai trò của khâu I và khâu D:")
        run("D1_P_only", "D", b["kp"], 0.0, candidate=False)
        run("D2_PI_plus_D", "D", b["kp"], b["ki"], b["kp"] * 0.5 * Leff, dmeas=1)
        b = best()
        print("\nPha E – Lặp lại bộ hệ số được chọn ({}) 3 lần:".format(b["tag"]))
        final = [run("E{}_final".format(k + 1), "E", b["kp"], b["ki"], b["kd"], b["dmeas"], candidate=False)
                 for k in range(3)]
    stats = {k: mean_std([t[k] for t in final]) for k in ("pot", "tr", "ts", "ess", "ess_pct", "iae", "u_noise")}
    ok_all = all(passes(t, crit)[0] for t in final)
    cfg["tune"] = {"trials": trials, "best": {k: b[k] for k in ("tag", "kp", "ki", "kd", "dmeas")},
                   "final": stats, "final_pass": ok_all}
    save_cfg(cfg)
    make_tune_plots(cfg)
    print("\nKẾT QUẢ: chọn {} -> Kp = {:.4g}, Ki = {:.4g}, Kd = {:.4g}{}".format(
        b["tag"], b["kp"], b["ki"], b["kd"], " (khâu D theo đầu ra đo)" if b["dmeas"] else ""))
    print("  Trung bình 3 lần: POT = {:.1f} ± {:.1f} %, t_s = {:.0f} ± {:.0f} ms, e_ss = {:+.2f} ± {:.2f} %".format(
        stats["pot"][0], stats["pot"][1], stats["ts"][0] * 1e3, stats["ts"][1] * 1e3,
        stats["ess_pct"][0], stats["ess_pct"][1]))
    if not ok_all:
        print("CẢNH BÁO: không phải cả 3 lần lặp đều đạt tiêu chí – xem bảng trong logs/tune/.")
    print("Bảng: logs/tune/trials.csv. Bước tiếp: python3 lab3.py dist --elec")


def make_tune_plots(cfg):
    tn = cfg["tune"]
    r = cfg["r"]
    by = {t["tag"]: t for t in tn["trials"]}

    def item(tag, label, hl=False):
        arr, meta = A.load_csv(by[tag]["csv"])
        return (arr, meta["i_step"], label, hl)

    best = tn["best"]["tag"]
    lab = lambda t: "{} (Kp={:.3g}, Ki={:.3g})".format(t["tag"], t["kp"], t["ki"])
    groups = {"A": "Pha A – Ziegler–Nichols", "B": "Pha B – quét τc (SIMC)", "C": "Pha C – tinh chỉnh Kp, Ki",
              "D": "Pha D – vai trò khâu I và D"}
    for ph, title in groups.items():
        its = [item(t["tag"], lab(t), t["tag"] == best) for t in tn["trials"] if t["phase"] == ph]
        if ph in ("C", "D") and best not in [t["tag"] for t in tn["trials"] if t["phase"] == ph]:
            its.append(item(best, lab(by[best]) + " – chọn", True))
        if its:
            xmax = max(1.0, 15.0 * (cfg["model"]["T"] + max(cfg["model"]["L"], A.TS)))
            P.trials(its, r, os.path.join(LOG, "tune", "phase_{}.png".format(ph)), title, xmax)
    fin = [t for t in tn["trials"] if t["phase"] == "E"]
    runs = []
    for t in fin:
        arr, meta = A.load_csv(t["csv"])
        runs.append((arr, meta["i_step"]))
    b = tn["best"]
    P.final_step(runs, r, cfg["criteria"]["band"] * r, tn["final"], (b["kp"], b["ki"], b["kd"]),
                 os.path.join(LOG, "tune", "final_step.png"))
    cols = ["tag", "phase", "kp", "ki", "kd", "dmeas", "pot", "tr", "ts", "ess_pct", "iae", "u_noise", "pass", "why"]
    with open(os.path.join(LOG, "tune", "trials.csv"), "w") as f:
        f.write(",".join(cols) + "\n")
        for t in tn["trials"]:
            f.write(",".join(str(t[c]) if c != "why" else '"' + "; ".join(t[c]) + '"' for c in cols) + "\n")


# =============================================================== BUOC 7: tai dot ngot
def gains_from(cfg, args):
    if args.kp is not None:
        return args.kp, args.ki or 0.0, args.kd or 0.0, 0
    need(cfg, "tune", "tune")
    b = cfg["tune"]["best"]
    return b["kp"], b["ki"], b["kd"], b["dmeas"]


def start_cl(link, cfg, g):
    link.configure(cfg, g[0], g[1], g[2], g[3])
    link.send("sp 0")
    link.wait_stopped()
    i0 = link.n()
    link.send("sp {:.2f}".format(cfg["r"]))
    return i0


def cmd_dist(args, cfg):
    need(cfg, "model", "ident")
    g = gains_from(cfg, args)
    r = cfg["r"]
    band_abs = cfg["criteria"]["band"] * r
    d = os.path.join(LOG, "dist")
    os.makedirs(d, exist_ok=True)
    if args.elec:
        head("BƯỚC 7a – Tải đột ngột tương đương bằng điện (tự động, lặp lại chính xác)")
        dd = args.d if args.d else float(5 * round(min(25.0, 0.6 * (100.0 - cfg["u_r"])) / 5))
        print("Nhiễu d = {:.0f} % duty: bật tại t = 2,5 s, tắt tại t = 5 s (sau bước vận tốc đặt).".format(dd))
        sets = [("Đã tune", g), ("Chỉ P (Ki = 0)", (g[0], 0.0, 0.0, 0)), ("Ki giảm 4 lần", (g[0], g[1] / 4, g[2], g[3]))]
        runs, rows = [], []
        with open_link(cfg, args) as link:
            for k, (name, gg) in enumerate(sets):
                i0 = start_cl(link, cfg, gg)
                link.wait(2.5)
                link.send("dist {:.1f}".format(dd))
                link.wait(2.5)
                link.send("dist 0")
                link.wait(2.5)
                link.send("sp 0")
                arr = link.segment(i0)
                on = np.abs(arr[:, 2] - r) < 0.01
                arr = arr[A.first_index(on):int(np.where(on)[0][-1]) + 1]
                edges = [i for i in range(1, len(arr)) if arr[i, 5] != arr[i - 1, 5]]
                ev = A.dist_known(arr, edges, r, band_abs)
                path = os.path.join(d, "elec_{}.csv".format(k + 1))
                save_csv(path, arr, {"kind": "dist_elec", "name": name, "kp": gg[0], "ki": gg[1], "kd": gg[2], "d": dd,
                                     "r": r})
                t_on = A.rel_t(arr)[edges[0]] if edges else 0
                t_off = A.rel_t(arr)[edges[1]] if len(edges) > 1 else t_on
                runs.append((arr, "{} (Kp={:.3g}, Ki={:.3g})".format(name, gg[0], gg[1]), [(t_on, t_off)]))
                for e in ev:
                    e.update({"set": name, "kp": gg[0], "ki": gg[1], "kd": gg[2]})
                    rows.append(e)
                    print("  {:<16s} {:<7s}: lệch đỉnh {:+6.2f} rpm ({:+5.1f} %), phục hồi {:>5s} ms, "
                          "e cuối đoạn {:+5.2f} rpm, u cuối {:5.1f} %".format(
                              name, e["loai"], e["dev_peak"], e["dev_peak_pct"], fmt(e["recovery"] * 1e3, "{:.0f}"),
                              e["e_end"], e["u_end"]))
        cfg["dist_elec"] = {"d": dd, "events": rows}
        save_cfg(cfg)
        P.dist(runs, r, band_abs, os.path.join(d, "elec.png"),
               "Tải đột ngột tương đương (d = {:.0f} % duty) – so sánh 3 bộ hệ số".format(dd))
        print("Đồ thị: {}/elec.png. Bước tiếp (bắt buộc theo lab): python3 lab3.py dist".format(d))
        return
    head("BƯỚC 7b – Tải đột ngột bằng tay (yêu cầu của lab_3)")
    print("Cản NHẸ trục (vải/tẩy kẹp giữa hai ngón) theo lời nhắc. KHÔNG giữ cứng trục.")
    all_ev = []
    with open_link(cfg, args) as link:
        for rep in range(args.repeats):
            ask("\nLần {}/{}: sẵn sàng tay ở trục, nhấn Enter để bắt đầu...".format(rep + 1, args.repeats))
            i0 = start_cl(link, cfg, g)
            t0 = time.time()
            plan = [(4.0, "CẢN NGAY!"), (6.0, "THẢ RA!"), (8.0, "CẢN LẦN 2!"), (9.5, "THẢ RA!")]
            for tm, msg in plan:
                for c in (3, 2, 1):
                    while time.time() - t0 < tm - c * 0.5:
                        link.wait(0.02)
                    sys.stdout.write("  {}… ".format(c))
                    sys.stdout.flush()
                while time.time() - t0 < tm:
                    link.wait(0.01)
                print("\a>>> " + msg)
            while time.time() - t0 < 12.0:
                link.wait(0.05)
            link.send("sp 0")
            arr = link.segment(i0)
            on = np.abs(arr[:, 2] - r) < 0.01
            arr = arr[A.first_index(on):int(np.where(on)[0][-1]) + 1]
            ev, thr, sd = A.dist_detect(arr, r, band=cfg["criteria"]["band"])
            t = A.rel_t(arr)
            hold = (t >= 5.0) & (t < 5.9)
            e_hold = float(np.mean(r - arr[hold, 3])) if hold.any() else float("nan")
            path = os.path.join(d, "hand_{}.csv".format(rep + 1))
            save_csv(path, arr, {"kind": "dist_hand", "kp": g[0], "ki": g[1], "kd": g[2], "r": r})
            print("  Ngưỡng phát hiện ±{:.2f} rpm (4σ nhiễu hoặc 5 % r). Sai số trung bình khi đang cản (5–5,9 s): {:+.2f} rpm".format(
                thr, e_hold))
            for e in ev:
                e["rep"] = rep + 1
                all_ev.append(e)
                print("  {:<20s} t = {:5.2f} s: lệch đỉnh {:+6.2f} rpm ({:+5.1f} %), phục hồi {:>5s} ms, u max {:5.1f} %{}".format(
                    e["loai"], e["t_start"], e["dev_peak"], e["dev_peak_pct"], fmt(e["recovery"] * 1e3, "{:.0f}"),
                    e["u_max"], "  ! u bão hoà – cản nhẹ hơn" if e["u_max"] >= 99.5 else ""))
            if not ev:
                print("  Không phát hiện sự kiện – lực cản quá nhẹ, chạy lại.")
            spans = [(e["t_start"], e["t_start"] + (e["recovery"] if np.isfinite(e["recovery"]) else 0.5)) for e in ev]
            P.dist([(arr, "Kp={:.3g}, Ki={:.3g}".format(g[0], g[1]), spans)], r, thr,
                   path.replace(".csv", ".png"), "Tải đột ngột bằng tay, lần {}".format(rep + 1))
    cfg["dist_hand"] = {"events": all_ev, "repeats": args.repeats}
    save_cfg(cfg)
    print("Đồ thị: {}/hand_*.png. Bước tiếp: python3 lab3.py report".format(d))


# =============================================================== phan tich lai
def cmd_analyze(args, cfg):
    arr, meta = A.load_csv(args.csv)
    kind = meta.get("kind", "step")
    print("File {}: {} mẫu, loại '{}'".format(args.csv, len(arr), kind))
    if kind == "ol_step":
        f = A.fopdt_smith(arr, meta["i_step"])
        print("K = {:.4f} rpm/%, T = {:.1f} ms, L = {:.1f} ms, RMS = {:.1f} %".format(f["K"], f["T"] * 1e3, f["L"] * 1e3,
                                                                                   f["rmse_pct"]))
    elif kind == "step":
        m = A.step_metrics(arr, meta["i_step"], meta["r"], args.band)
        for k in ("yss", "pot", "tr", "ts", "ess", "ess_pct", "iae", "u_noise"):
            print("  {:8s} = {}".format(k, fmt(m[k], "{:.4g}")))
    elif kind in ("dist_hand", "dist_elec"):
        r = meta["r"]
        if kind == "dist_elec":
            edges = [i for i in range(1, len(arr)) if arr[i, 5] != arr[i - 1, 5]]
            for e in A.dist_known(arr, edges, r, args.band * r):
                print(e)
        else:
            ev, thr, sd = A.dist_detect(arr, r, band=args.band)
            for e in ev:
                print(e)
    else:
        print("Loại '{}' không có phân tích riêng.".format(kind))


# =============================================================== bao cao
def cmd_report(args, cfg):
    head("Tạo báo cáo")
    out = "report"
    fig = os.path.join(out, "figures")
    os.makedirs(fig, exist_ok=True)

    def cp(src, name):
        if src and os.path.exists(src):
            shutil.copy(src, os.path.join(fig, name))
            return "figures/" + name
        return None

    L_ = []
    w = L_.append
    w("# Lab 3 – Vòng lặp PID rời rạc: kết quả đo\n")
    w("Tạo tự động bởi `lab3.py report` lúc {}. Mọi số liệu lấy từ `lab3_config.json` và thư mục `logs/`.\n".format(
        datetime.datetime.now().strftime("%Y-%m-%d %H:%M")))
    crit = cfg["criteria"]
    if cfg.get("calib"):
        c = cfg["calib"]
        w("## 1. Hiệu chuẩn encoder\n")
        w("Quay tay trục ra {} vòng, lặp {} lần.\n".format(c["turns"], len(c["values"])))
        w("| Lần | Đếm/vòng |\n| --- | --- |")
        for i, v in enumerate(c["values"]):
            w("| {} | {:.1f} |".format(i + 1, v))
        w("| **TB ± độ lệch chuẩn** | **{:.1f} ± {:.1f}** |\n".format(c["mean"], c["std"]))
        w("Độ lệch max–min {:.2f} %; enc_err = {}. CPR dùng cho mọi phép đo: **{:.1f}**.\n".format(
            c["spread_pct"], c["enc_err"], cfg["cpr"]))
    if cfg.get("verify"):
        v = cfg["verify"]
        w("## 2. Kiểm chứng vận tốc\n")
        if v.get("rpm_video"):
            w("| Phương pháp | Vận tốc (rpm) |\n| --- | --- |")
            w("| Encoder (trung bình {:.0f} s) | {:.2f} |".format(v["dur"], v["rpm_enc"]))
            w("| Video ({} vòng / {:.2f} s) | {:.2f} |".format(v["n_rev"], v["video_dur"], v["rpm_video"]))
            w("| **Sai lệch tương đối** | **{:+.2f} %** |\n".format(v["err_pct"]))
        else:
            w("Encoder: {:.2f} rpm. Chưa phân tích video.\n".format(v["rpm_enc"]))
        p = cp(v["csv"].replace(".csv", ".png"), "verify.png")
        if p:
            w("![Kiểm chứng vận tốc]({})\n".format(p))
        p = cp(os.path.join(LOG, "video_count.png"), "video_count.png")
        if p:
            w("![Đếm vòng từ video]({})\n".format(p))
    if cfg.get("model"):
        m = cfg["model"]
        w("## 3. Nhận dạng đối tượng\n")
        w("Duty khởi động {} %, v_max = {:.1f} rpm, vận tốc đặt r = {:.0f} rpm.\n".format(cfg["dz"], cfg["vmax"], cfg["r"]))
        w("| Tham số FOPDT | Giá trị (TB ± độ lệch chuẩn, {} lần) |\n| --- | --- |".format(len(m["runs"])))
        w("| K (rpm/%) | {:.4f} ± {:.4f} |".format(m["K"], m["K_std"]))
        w("| T (ms) | {:.1f} ± {:.1f} |".format(m["T"] * 1e3, m["T_std"] * 1e3))
        w("| L (ms) | {:.1f} ± {:.1f} |\n".format(m["L"] * 1e3, m["L_std"] * 1e3))
        w("K từ đặc tính tĩnh quanh điểm làm việc: {:.4f} rpm/%.\n".format(m["K_static"]))
        for src, name, cap in ((os.path.join(LOG, "ident", "static.png"), "static.png", "Đặc tính tĩnh"),
                               (os.path.join(LOG, "ident", "step_fit.png"), "step_fit.png", "Mô hình FOPDT")):
            p = cp(src, name)
            if p:
                w("![{}]({})\n".format(cap, p))
    if cfg.get("tune"):
        tn = cfg["tune"]
        w("## 4. Tune PID\n")
        w("Tiêu chí ĐẠT (đề xuất, lab không quy định): POT ≤ {:g} %, |e_ss| ≤ {:g} %, t_s theo dải ±{:g} %, σ(u) ≤ {:g} % duty.\n".format(
            crit["pot_max"], crit["ess_max_pct"], crit["band"] * 100, crit["u_noise_max"]))
        w("| Lần thử | Kp | Ki | Kd | POT (%) | t_s (ms) | e_ss (%) | σ(u) (%) | Kết quả |")
        w("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
        for t in tn["trials"]:
            res = ("ĐẠT" if t["pass"] else "không đạt: " + "; ".join(t["why"])) if t["candidate"] else "đối chứng"
            w("| {} | {:.4g} | {:.4g} | {:.4g} | {} | {} | {} | {} | {} |".format(
                t["tag"], t["kp"], t["ki"], t["kd"], fmt(t["pot"]), fmt(t["ts"] * 1e3, "{:.0f}"),
                fmt(t["ess_pct"], "{:+.2f}"), fmt(t["u_noise"]), res))
        b, s = tn["best"], tn["final"]
        w("\n**Bộ hệ số chọn:** Kp = {:.4g}, Ki = {:.4g}, Kd = {:.4g}{}.\n".format(
            b["kp"], b["ki"], b["kd"], " (khâu D tính theo đầu ra đo)" if b["dmeas"] else ""))
        w("| Chỉ tiêu (TB ± độ lệch chuẩn, 3 lần) | Giá trị |\n| --- | --- |")
        w("| Độ vọt lố POT | {:.1f} ± {:.1f} % |".format(*s["pot"]))
        w("| Thời gian lên t_r (10–90 %) | {:.0f} ± {:.0f} ms |".format(s["tr"][0] * 1e3, s["tr"][1] * 1e3))
        w("| Thời gian xác lập t_s (±{:g} %) | {:.0f} ± {:.0f} ms |".format(crit["band"] * 100, s["ts"][0] * 1e3, s["ts"][1] * 1e3))
        w("| Sai số xác lập e_ss | {:+.2f} ± {:.2f} % |".format(*s["ess_pct"]))
        w("| IAE | {:.2f} ± {:.2f} rpm·s |\n".format(*s["iae"]))
        for ph in ("A", "B", "C", "D"):
            p = cp(os.path.join(LOG, "tune", "phase_{}.png".format(ph)), "tune_phase_{}.png".format(ph))
            if p:
                w("![Pha {}]({})\n".format(ph, p))
        p = cp(os.path.join(LOG, "tune", "final_step.png"), "final_step.png")
        if p:
            w("![Đáp ứng bậc của bộ hệ số chọn]({})\n".format(p))
    if cfg.get("dist_hand") or cfg.get("dist_elec"):
        w("## 5. Thay đổi tải đột ngột\n")
        if cfg.get("dist_hand"):
            w("### 5.1 Cản tay (yêu cầu của lab)\n")
            w("| Lần | Sự kiện | t (s) | Lệch đỉnh (rpm) | Lệch đỉnh (%) | Phục hồi (ms) | u max (%) |")
            w("| --- | --- | --- | --- | --- | --- | --- |")
            for e in cfg["dist_hand"]["events"]:
                w("| {} | {} | {:.2f} | {:+.2f} | {:+.1f} | {} | {:.1f} |".format(
                    e["rep"], e["loai"], e["t_start"], e["dev_peak"], e["dev_peak_pct"], fmt(e["recovery"] * 1e3, "{:.0f}"),
                    e["u_max"]))
            w("")
            for k in range(cfg["dist_hand"]["repeats"]):
                p = cp(os.path.join(LOG, "dist", "hand_{}.png".format(k + 1)), "dist_hand_{}.png".format(k + 1))
                if p:
                    w("![Cản tay lần {}]({})\n".format(k + 1, p))
        if cfg.get("dist_elec"):
            de = cfg["dist_elec"]
            w("### 5.2 Tải tương đương bằng điện (d = {:.0f} % duty)\n".format(de["d"]))
            w("| Bộ hệ số | Sự kiện | Lệch đỉnh (rpm) | Phục hồi (ms) | e cuối đoạn (rpm) | u cuối đoạn (%) |")
            w("| --- | --- | --- | --- | --- | --- |")
            for e in de["events"]:
                w("| {} | {} | {:+.2f} | {} | {:+.2f} | {:.1f} |".format(
                    e["set"], e["loai"], e["dev_peak"], fmt(e["recovery"] * 1e3, "{:.0f}"), e["e_end"], e["u_end"]))
            w("")
            p = cp(os.path.join(LOG, "dist", "elec.png"), "dist_elec.png")
            if p:
                w("![Tải tương đương bằng điện]({})\n".format(p))
    w("## 6. Áp dụng PID cho đồ án\n")
    w("(Dán đoạn 150–250 từ ở Bước 8 của tài liệu hướng dẫn vào đây.)\n")
    path = os.path.join(out, "BAO_CAO_LAB3.md")
    with open(path, "w") as f:
        f.write("\n".join(L_))
    shutil.copy(CFG_PATH, os.path.join(out, CFG_PATH)) if os.path.exists(CFG_PATH) else None
    print("Đã tạo {} và {}/ ({} hình).".format(path, fig, len(os.listdir(fig))))


# =============================================================== chay tat ca
def cmd_all(args, cfg):
    steps = [("calib", "Bước 4a – hiệu chuẩn CPR (cần quay tay, rút adapter 12V)"),
             ("verify", "Bước 4b – chạy động cơ 40 s để quay video (cắm adapter 12V)"),
             ("ident", "Bước 5 – nhận dạng (tự động)"),
             ("tune", "Bước 6 – tune (tự động)"),
             ("dist_elec", "Bước 7a – tải tương đương bằng điện (tự động)"),
             ("dist_hand", "Bước 7b – cản tay"),
             ("report", "Tạo báo cáo")]
    for key, txt in steps:
        a = ask("\n{}? [Enter = chạy, s = bỏ qua, q = thoát] ".format(txt)).strip().lower()
        if a == "q":
            return
        if a == "s":
            continue
        cfg = load_cfg()
        ns = argparse.Namespace(**vars(args))
        if key == "calib":
            ns.turns, ns.trials = 10, 3
            cmd_calib(ns, cfg)
        elif key == "verify":
            ns.video, ns.duty, ns.dur, ns.roi = None, None, 40.0, None
            cmd_verify(ns, cfg)
            vid = ask("Chép video vào máy rồi nhập đường dẫn file (Enter để làm sau): ").strip().strip("'\"")
            if vid:
                ns.video = vid
                verify_video(ns, load_cfg())
        elif key == "ident":
            ns.sp, ns.repeats = None, 3
            cmd_ident(ns, cfg)
        elif key == "tune":
            cmd_tune(ns, cfg)
        elif key == "dist_elec":
            ns.elec, ns.d, ns.kp, ns.ki, ns.kd = True, None, None, None, None
            cmd_dist(ns, cfg)
        elif key == "dist_hand":
            ns.elec, ns.d, ns.kp, ns.ki, ns.kd, ns.repeats = False, None, None, None, None, 2
            cmd_dist(ns, cfg)
        else:
            cmd_report(ns, cfg)


def main():
    ap = argparse.ArgumentParser(description="Lab 3 – tự động hoá Bước 4–7", formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--port", help="cổng Serial (mặc định: tự tìm)")
    ap.add_argument("-v", "--verbose", action="store_true", help="in mọi thông báo từ ESP32")
    sub = ap.add_subparsers(dest="cmd")
    s = sub.add_parser("calib")
    s.add_argument("--turns", type=int, default=10)
    s.add_argument("--trials", type=int, default=3)
    s = sub.add_parser("verify")
    s.add_argument("--video")
    s.add_argument("--roi", help="x,y,w,h – bỏ qua để chọn bằng chuột")
    s.add_argument("--duty", type=float)
    s.add_argument("--dur", type=float, default=40.0)
    s = sub.add_parser("ident")
    s.add_argument("--sp", type=float, help="vận tốc đặt (rpm), mặc định 55 % v_max")
    s.add_argument("--repeats", type=int, default=3)
    sub.add_parser("tune")
    s = sub.add_parser("dist")
    s.add_argument("--elec", action="store_true", help="tải tương đương bằng điện (tự động)")
    s.add_argument("--d", type=float, help="biên độ nhiễu tương đương (%% duty)")
    s.add_argument("--repeats", type=int, default=2)
    for k in ("kp", "ki", "kd"):
        s.add_argument("--" + k, type=float)
    sub.add_parser("report")
    sub.add_parser("all")
    s = sub.add_parser("analyze")
    s.add_argument("csv")
    s.add_argument("--band", type=float, default=0.05)
    args = ap.parse_args()
    if not args.cmd:
        ap.print_help()
        print(__doc__)
        return
    os.makedirs(LOG, exist_ok=True)
    cfg = load_cfg()
    fn = {"calib": cmd_calib, "verify": cmd_verify, "ident": cmd_ident, "tune": cmd_tune, "dist": cmd_dist,
          "report": cmd_report, "all": cmd_all, "analyze": cmd_analyze}[args.cmd]
    try:
        fn(args, cfg)
    except DeviceError as e:
        raise SystemExit("LỖI: {}".format(e))
    except KeyboardInterrupt:
        raise SystemExit("\nĐã dừng (Ctrl+C). Động cơ đã được lệnh dừng.")


if __name__ == "__main__":
    main()
