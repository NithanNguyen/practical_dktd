#!/usr/bin/env python3
"""Lab 4 CE212 - Nhanh Robot (thuc hanh ve nha): tu dong hoa tu nap firmware den bao cao.

  python3 lab4.py setup        kiem tra may tinh, lay CPR tu Lab 3, nap firmware (PlatformIO), ket noi
  python3 lab4.py check        kiem tra phan cung: 2 nut, chieu encoder (tu sua), dao chieu, dung khan
  python3 lab4.py estop        do thoi gian dung khan: phanh ngan o 3 muc duty + tha tu do (nhan nut khi duoc nhac)
  python3 lab4.py reverse      do dao chieu bang nut DAO CHIEU (nhan khi duoc nhac)
  python3 lab4.py demo         man hinh truc tiep de quay video demo (dieu khien hoan toan bang nut)
  python3 lab4.py report       tao report/BAO_CAO_LAB4.md + do thi tu du lieu da do
  python3 lab4.py all          setup -> check -> estop -> reverse -> report

Tuy chon chung: --port /dev/ttyUSB0 | --sim (chay voi phan cung GIA LAP, du lieu luu rieng *_sim) | -v
"""
import argparse
import datetime
import glob
import json
import os
import shutil
import subprocess
import sys
import time

import numpy as np

import lab4_analysis as A
from lab4_link import DeviceError, bell, find_port, open_link

HERE = os.path.dirname(os.path.abspath(__file__))
FW_DIR = os.path.normpath(os.path.join(HERE, "..", "firmware"))
LAB3_CFG = os.path.normpath(os.path.join(HERE, "..", "..", "lab3_pid", "tools", "lab3_config.json"))
STATE_NAME = {0: "DỪNG", 1: "ĐANG CHẠY", 2: "ĐANG ĐẢO CHIỀU", 3: "DỪNG KHẨN (đã chốt)"}
DEFAULT = {"port": None, "cpr": 1978.7, "cpr_src": "mặc định (= Lab 3)", "sign": 1,
           "settle_s": 1.5, "estop_duties": [40, 70, 100], "estop_reps": 3,
           "coast_duty": 100, "coast_reps": 3, "rev_duty": 70, "rev_reps": 5}


# =============================================================== tien ich
class Ctx:
    def __init__(self, args):
        self.args = args
        self.sim = bool(getattr(args, "sim", False))
        suf = "_sim" if self.sim else ""
        self.cfg_path = os.path.join(HERE, "lab4_config{}.json".format(suf))
        self.logs = os.path.join(HERE, "logs" + suf)
        self.report = os.path.join(HERE, "report" + suf)
        self.cfg = load_cfg(self.cfg_path)
        self.link = None

    def open(self):
        if self.link is None:
            port = getattr(self.args, "port", None) or self.cfg.get("port")
            self.link = open_link(port=port, sim=self.sim, verbose=getattr(self.args, "verbose", False),
                                  sim_enc_sign=getattr(self.args, "sim_enc_sign", 1))
            self.link.send("cpr {:.3f}".format(self.cfg["cpr"]))
            self.link.send("sign {}".format(int(self.cfg["sign"])))
            self.link.send("mode brake")
        return self.link

    def close(self):
        if self.link is not None:
            self.link.close()
            self.link = None

    def save(self):
        with open(self.cfg_path, "w") as f:
            json.dump(self.cfg, f, ensure_ascii=False, indent=2, default=_json_default)


def _json_default(o):
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.bool_):
        return bool(o)
    raise TypeError(type(o))


def load_cfg(path):
    cfg = json.loads(json.dumps(DEFAULT))
    if os.path.exists(path):
        with open(path) as f:
            cfg.update(json.load(f))
    return cfg


def stamp():
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def head(txt):
    print("\n" + "=" * 72 + "\n" + txt + "\n" + "=" * 72)


def prompt(ctx, text):
    """Nhac nguoi dung thao tac. O che do gia lap, nut se duoc 'nhan' tu dong."""
    bell()
    print("\n  >>> " + text + ("   [giả lập: tự nhấn]" if ctx.sim else ""))


def nan2s(x, fmt="{:.1f}"):
    return "–" if x is None or not np.isfinite(x) else fmt.format(x)


# =============================================================== thao tac chung
def ensure_ready(ctx, quiet=False):
    """Dua firmware ve trang thai DUNG: dung dong co, xoa chot dung khan (doi nha nut neu can)."""
    link = ctx.link
    st = link.query_status()
    if st.get("state") in (1, 2):
        link.send("stop")
        time.sleep(0.2)
        st = link.query_status()
    if not st.get("latched"):
        return
    t_end = time.time() + 120
    told = False
    while time.time() < t_end:
        m = link.mark()
        link.send("reset")
        got = link.wait_comment("#OK RESET", m, 0.5)
        if got:
            return
        errs = [c for c in link.comments[m["com"]:] if c.startswith("#ERR reset")]
        if errs and "NHA" in errs[-1] and not told and not quiet:
            print("  ... Nhả nút DỪNG KHẨN để tiếp tục.")
            told = True
        time.sleep(0.3)
    raise DeviceError("Không reset được dừng khẩn sau 120 s (nút DỪNG KHẨN vẫn đang nhấn?).")


def run_and_settle(ctx, duty, direction=1):
    link = ctx.link
    m = link.mark()
    link.send("run {:.1f} {}".format(duty, direction))
    if not link.wait_comment("#EV RUN", m, 1.5):
        errs = [c for c in link.comments[m["com"]:] if c.startswith("#ERR")]
        raise DeviceError("Không chạy được động cơ: {}".format(errs[-1] if errs else "không phản hồi"))
    time.sleep(ctx.cfg["settle_s"])
    a = link.last_rows(40)
    rpm = a[:, 4]
    return float(np.mean(rpm)), float(np.std(rpm))


# =============================================================== setup
def find_pio():
    p = shutil.which("pio") or shutil.which("platformio")
    if p:
        return p
    cand = os.path.expanduser("~/.platformio/penv/bin/pio")
    return cand if os.path.exists(cand) else None


def cmd_setup(ctx):
    head("BƯỚC 1 – Kiểm tra máy tính, lấy CPR từ Lab 3, nạp firmware")
    import matplotlib
    import serial
    print("  Python {} | numpy {} | matplotlib {} | pyserial {}".format(
        sys.version.split()[0], np.__version__, matplotlib.__version__, serial.__version__))

    if not ctx.sim:
        import grp
        import pwd
        user = pwd.getpwuid(os.getuid()).pw_name
        try:
            members = grp.getgrnam("dialout").gr_mem
            gid = grp.getgrnam("dialout").gr_gid
            in_group = user in members or gid in os.getgroups()
        except KeyError:
            in_group = True
        if not in_group:
            print("  [!] User '{}' chưa thuộc nhóm dialout -> không mở được cổng Serial.\n"
                  "      Chạy: sudo usermod -a -G dialout $USER  rồi ĐĂNG XUẤT và đăng nhập lại.".format(user))

    if os.path.exists(LAB3_CFG):
        with open(LAB3_CFG) as f:
            l3 = json.load(f)
        if l3.get("cpr"):
            ctx.cfg["cpr"] = float(l3["cpr"])
            ctx.cfg["cpr_src"] = "Lab 3 – hiệu chuẩn quay tay (lab3_config.json)"
            calib = l3.get("calib", {})
            if calib.get("std"):
                ctx.cfg["cpr_std"] = calib["std"]
        if "sign" in l3:
            ctx.cfg["sign"] = int(l3["sign"])
        print("  CPR = {:.1f} đếm/vòng trục ra, sign = {} (lấy từ Lab 3)".format(ctx.cfg["cpr"], ctx.cfg["sign"]))
    else:
        print("  [!] Không thấy {}. Dùng CPR mặc định {:.1f}.".format(LAB3_CFG, ctx.cfg["cpr"]))

    if ctx.sim:
        print("  Chế độ --sim: biên dịch firmware cho PC thay vì nạp ESP32.")
    elif getattr(ctx.args, "no_flash", False):
        print("  Bỏ qua nạp firmware (--no-flash).")
    else:
        pio = find_pio()
        if not pio:
            raise SystemExit("  [x] Không tìm thấy PlatformIO (pio). Mở VS Code > PlatformIO > Build/Upload thư mục\n"
                             "      lab4_robot/firmware, rồi chạy lại: python3 lab4.py setup --no-flash")
        cmd = [pio, "run", "-d", FW_DIR, "-t", "upload"]
        port = getattr(ctx.args, "port", None) or ctx.cfg.get("port")
        if port:
            cmd += ["--upload-port", port]
        print("  Nạp firmware: " + " ".join(cmd))
        r = subprocess.run(cmd)
        if r.returncode != 0:
            raise SystemExit("  [x] Nạp thất bại. Đóng Serial Monitor; nếu báo 'Failed to connect' thì giữ nút BOOT "
                             "trên board khi thấy 'Connecting...'.")
        time.sleep(1.5)

    link = ctx.open()
    st = link.query_status()
    if not ctx.sim:
        ctx.cfg["port"] = link.tr.name
    print("  Kết nối OK: {} trên {}. Trạng thái: {}".format(link.fw, link.tr.name,
                                                            STATE_NAME.get(st.get("state"), "?")))
    ctx.cfg["fw"] = link.fw
    ctx.save()


# =============================================================== check
def cmd_check(ctx):
    head("BƯỚC 2 – Kiểm tra phần cứng (nút nhấn, encoder, đảo chiều, dừng khẩn)")
    link = ctx.open()
    res = {}
    ensure_ready(ctx)

    # (a) nut DUNG KHAN khi dong co dang dung -> chot, khong quay
    m = link.mark()
    prompt(ctx, "Nhấn rồi NHẢ nút DỪNG KHẨN (động cơ đang đứng yên, không quay).")
    if ctx.sim:
        link.sim_press("estop")
    ok_e = link.wait_button("ESTOP", "DOWN", m, 60) and link.wait_button("ESTOP", "UP", m, 30)
    r = link.wait_result("ESTOP", m, 5)
    ok_e = bool(ok_e and r is not None and r.get("src") in ("btn", "poll"))
    res["btn_estop"] = ok_e
    print("  [{}] Nút DỪNG KHẨN (IO19): {}".format("OK" if ok_e else "LỖI",
          "ngắt + chốt hoạt động" if ok_e else "không nhận được tín hiệu – kiểm tra dây IO19/GND"))

    # (b) nut DAO CHIEU khi dang chot -> chi in goi y, khong chay
    m = link.mark()
    prompt(ctx, "Nhấn nhanh rồi NHẢ nút ĐẢO CHIỀU (đang chốt dừng khẩn nên động cơ không chạy).")
    if ctx.sim:
        link.sim_press("dir")
    ok_d = link.wait_button("DIR", "DOWN", m, 60) and link.wait_button("DIR", "UP", m, 30)
    res["btn_dir"] = bool(ok_d)
    print("  [{}] Nút ĐẢO CHIỀU (IO18)".format("OK" if ok_d else "LỖI"))

    # (c) giu DAO CHIEU 1 s -> reset
    m = link.mark()
    prompt(ctx, "GIỮ nút ĐẢO CHIỀU khoảng 1,5 s rồi nhả (xoá chốt dừng khẩn).")
    if ctx.sim:
        link.sim_press("dir", hold=1.4)
    ok_r = link.wait_comment("#OK RESET src=btn", m, 60) is not None
    link.wait_button("DIR", "UP", m, 10)
    res["reset_btn"] = ok_r
    print("  [{}] Reset bằng giữ nút ĐẢO CHIỀU 1 s".format("OK" if ok_r else "LỖI"))
    ensure_ready(ctx, quiet=True)

    # (d) chieu encoder
    rpm, sd = run_and_settle(ctx, 40, 1)
    link.send("stop")
    time.sleep(0.6)
    st = link.query_status()
    if abs(rpm) < 2.0:
        res["encoder"] = False
        print("  [LỖI] Động cơ không quay hoặc encoder không đếm (rpm = {:.1f}). Kiểm tra adapter 12 V vào VM, "
              "dây AO1/AO2, dây encoder A/B (IO32/IO33) và nguồn 3V3 encoder.".format(rpm))
    else:
        if rpm < 0:
            ctx.cfg["sign"] = -int(ctx.cfg["sign"])
            link.send("sign {}".format(ctx.cfg["sign"]))
            ctx.cfg["sign_fixed"] = True
            print("  [SỬA] dir = +1 cho rpm âm -> đổi sign = {} (đã lưu).".format(ctx.cfg["sign"]))
            rpm = -rpm
        res["encoder"] = True
        print("  [OK] Encoder: duty 40 % -> {:.1f} rpm (dao động ±{:.1f}), enc_err = {}".format(
            rpm, sd, st.get("enc_err")))
    res["rpm_at_40"] = rpm

    # (e) dao chieu + dung khan bang lenh mem (khong can nhan nut)
    run_and_settle(ctx, 60, 1)
    m = link.mark()
    link.send("rev")
    r = link.wait_result("REV", m, 6)
    ok_rev = bool(r and not r.get("timeout") and r.get("rpm0", 0) * r.get("rpm1", 0) < 0)
    res["reverse"] = ok_rev
    print("  [{}] Đảo chiều: {}".format("OK" if ok_rev else "LỖI", "dừng sau {:.0f} ms, 90 % chiều mới sau {:.0f} ms"
                                       .format(r["t_zero_ms"], r["t90_ms"]) if r else "không có kết quả"))
    time.sleep(1.0)
    m = link.mark()
    link.send("estop")
    r = link.wait_result("ESTOP", m, 8)
    ok_es = bool(r and not r.get("timeout"))
    res["estop_soft"] = ok_es
    print("  [{}] Dừng khẩn (lệnh mềm): {}".format("OK" if ok_es else "LỖI",
          "dừng sau {:.1f} ms".format(r["dt_ms"]) if r else "không có kết quả"))
    link.wait_capture("ESTOP", r["id"] if r else -1, 5)
    ensure_ready(ctx, quiet=True)

    st = link.query_status()
    res["enc_err"] = st.get("enc_err")
    res["all_ok"] = all(res[k] for k in ("btn_estop", "btn_dir", "reset_btn", "encoder", "reverse", "estop_soft"))
    ctx.cfg["check"] = res
    ctx.save()
    print("\n  KẾT LUẬN: {}".format("PHẦN CỨNG SẴN SÀNG" if res["all_ok"] else "CÒN LỖI – sửa theo gợi ý ở trên rồi chạy lại check"))
    return res["all_ok"]


# =============================================================== estop
def cmd_estop(ctx):
    a = ctx.args
    duties = a.duties or ctx.cfg["estop_duties"]
    reps = a.reps or ctx.cfg["estop_reps"]
    coast_reps = ctx.cfg["coast_reps"] if a.coast_reps is None else a.coast_reps
    plan = [("brake", d) for d in duties for _ in range(reps)] + [("coast", ctx.cfg["coast_duty"])] * coast_reps
    head("BƯỚC 3 – Đo thời gian dừng khẩn: {} lần ({} bằng nút)".format(len(plan), "0" if a.soft else len(plan)))
    print("  Kế hoạch: phanh ngắn ở duty {} %, mỗi mức {} lần; thả tự do ở {} %, {} lần.".format(
        ", ".join(str(d) for d in duties), reps, ctx.cfg["coast_duty"], coast_reps))
    link = ctx.open()
    sess = os.path.join(ctx.logs, "estop", stamp())
    trials = []
    for k, (mode, duty) in enumerate(plan, 1):
        ensure_ready(ctx)
        link.send("mode " + mode)
        rpm, sd = run_and_settle(ctx, duty, 1)
        m = link.mark()
        if a.soft:
            link.send("estop")
        else:
            prompt(ctx, "[{}/{}] Động cơ quay {:.0f} % (≈ {:.0f} rpm), chế độ {}. NHẤN nút DỪNG KHẨN.".format(
                k, len(plan), duty, rpm, "phanh ngắn" if mode == "brake" else "thả tự do"))
            link.sim_press("estop")
        r = link.wait_result("ESTOP", m, 90)
        if r is None:
            link.send("stop")
            print("  [x] Không thấy nhấn nút sau 90 s – dừng phiên đo, giữ lại {} lần đã đo.".format(len(trials)))
            break
        cap = link.wait_capture("ESTOP", r["id"], 10)
        if cap is None:
            print("  [!] Không nhận được dữ liệu chụp 1 kHz của lần này, bỏ qua.")
            continue
        tag = "{:02d}_{}_d{}".format(k, mode, int(duty))
        csv = os.path.join(sess, tag + ".csv")
        meta = dict(r)
        meta.pop("_time", None)
        meta.update({"mode": mode, "duty": duty, "cpr": ctx.cfg["cpr"], "sim": ctx.sim})
        A.save_capture(csv, {"t_us": cap["t_us"], "count": cap["count"]}, meta)
        an = A.analyze_estop(cap["t_us"] / 1000.0, cap["count"], ctx.cfg["cpr"], fw=r)
        tr = {"tag": tag, "csv": os.path.relpath(csv, HERE), "mode": mode, "duty": duty, "src": r.get("src")}
        tr.update(an)
        trials.append(tr)
        print("      -> t_dừng = {} ms (firmware {} ms) | t_10% = {} ms | θ = {}° | τ = {} ms".format(
            nan2s(an["t_stop_ms"]), nan2s(r.get("dt_ms")), nan2s(an["t10_ms"]), nan2s(an["theta_deg"]),
            nan2s(an["tau_ms"])))
    ensure_ready(ctx, quiet=True)
    link.send("mode brake")
    if trials:
        os.makedirs(sess, exist_ok=True)
        with open(os.path.join(sess, "trials.json"), "w") as f:
            json.dump(trials, f, ensure_ascii=False, indent=1, default=_json_default)
        print("\n  Đã lưu {} lần đo vào {}".format(len(trials), os.path.relpath(sess, HERE)))
    return trials


# =============================================================== reverse
def cmd_reverse(ctx):
    a = ctx.args
    duty = a.duty or ctx.cfg["rev_duty"]
    reps = a.reps or ctx.cfg["rev_reps"]
    head("BƯỚC 4 – Đo đảo chiều bằng nút ĐẢO CHIỀU: {} lần ở duty {} %".format(reps, duty))
    link = ctx.open()
    sess = os.path.join(ctx.logs, "reverse", stamp())
    ensure_ready(ctx)
    link.send("mode brake")
    rpm, _ = run_and_settle(ctx, duty, 1)
    trials = []
    for k in range(1, reps + 1):
        time.sleep(0.5)
        m = link.mark()
        if a.soft:
            link.send("rev")
        else:
            prompt(ctx, "[{}/{}] Động cơ đang quay ổn định. NHẤN nút ĐẢO CHIỀU.".format(k, reps))
            link.sim_press("dir")
        r = link.wait_result("REV", m, 90)
        if r is None:
            print("  [x] Không thấy nhấn nút sau 90 s – dừng phiên đo.")
            break
        cap = link.wait_capture("REV", r["id"], 10)
        if cap is None:
            print("  [!] Không nhận được dữ liệu chụp, bỏ qua.")
            continue
        tag = "{:02d}_rev_d{}".format(k, int(duty))
        csv = os.path.join(sess, tag + ".csv")
        meta = dict(r)
        meta.pop("_time", None)
        meta.update({"duty": duty, "cpr": ctx.cfg["cpr"], "sim": ctx.sim})
        A.save_capture(csv, {"t_us": cap["t_us"], "count": cap["count"]}, meta)
        an = A.analyze_rev(cap["t_us"] / 1000.0, cap["count"], ctx.cfg["cpr"], fw=r)
        tr = {"tag": tag, "csv": os.path.relpath(csv, HERE), "duty": duty, "src": r.get("src")}
        tr.update(an)
        trials.append(tr)
        print("      -> {:+.0f} -> {:+.0f} rpm | dừng (t_zero) {} ms | 90 % chiều mới (t_90) {} ms".format(
            an["rpm0"], an["rpm1"], nan2s(an["t_zero_ms"]), nan2s(an["t90_ms"])))
        time.sleep(ctx.cfg["settle_s"])
    link.send("stop")
    if trials:
        os.makedirs(sess, exist_ok=True)
        with open(os.path.join(sess, "trials.json"), "w") as f:
            json.dump(trials, f, ensure_ascii=False, indent=1, default=_json_default)
        print("\n  Đã lưu {} lần đo vào {}".format(len(trials), os.path.relpath(sess, HERE)))
    return trials


# =============================================================== demo
def cmd_demo(ctx):
    head("DEMO – quay video: điều khiển HOÀN TOÀN bằng nút (đóng cửa sổ để thoát)")
    print("  ĐẢO CHIỀU: chạy / đảo chiều | DỪNG KHẨN: dừng ngay + chốt | giữ ĐẢO CHIỀU 1 s: xoá chốt")
    link = ctx.open()
    ensure_ready(ctx, quiet=True)
    link.send("mode brake")
    link.send("duty {:.0f}".format(ctx.args.duty or 70))
    os.makedirs(os.path.join(ctx.logs, "demo"), exist_ok=True)
    log_path = os.path.join(ctx.logs, "demo", "demo_{}.txt".format(stamp()))
    m0 = link.mark()
    if ctx.sim and not getattr(ctx.args, "no_auto", False):
        _sim_demo_script(link)
    headless = getattr(ctx.args, "headless", False) or not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    try:
        if headless:
            _demo_text(ctx, link, m0)
        else:
            _demo_plot(ctx, link, m0)
    finally:
        with open(log_path, "w") as f:
            for s in link.comments[m0["com"]:]:
                f.write(s + "\n")
        print("  Nhật ký sự kiện: " + os.path.relpath(log_path, HERE))


def _sim_demo_script(link):
    import threading

    def job():
        seq = [(1.0, "dir", 0.15), (3.0, "dir", 0.15), (3.0, "estop", 0.6), (1.5, "dir", 1.3),
               (1.0, "dir", 0.15), (2.5, "estop", 0.3)]
        for wait, btn, hold in seq:
            time.sleep(wait)
            link.sim_press(btn, hold=hold, delay=0)
            time.sleep(hold)
    threading.Thread(target=job, daemon=True).start()


def _last(link, m0, kind):
    rs = [r for r in link.results[m0["res"]:] if r["kind"] == kind]
    return rs[-1] if rs else None


def _demo_text(ctx, link, m0):
    dur = ctx.args.duration or (14 if ctx.sim else 600)
    t_end = time.time() + dur
    last = ""
    while time.time() < t_end:
        a = link.last_rows(1)
        if len(a):
            st = int(a[0, 1])
            e = _last(link, m0, "ESTOP")
            line = "  {:<20s} chiều {:+d}  duty {:5.1f} %  {:+7.1f} rpm   | dừng khẩn gần nhất: {}".format(
                STATE_NAME.get(st, "?"), int(a[0, 2]), a[0, 3], a[0, 4],
                "{:.1f} ms".format(e["dt_ms"]) if e else "–")
            if line != last:
                print(line)
                last = line
        time.sleep(0.25)


def _demo_plot(ctx, link, m0):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    win = 10.0
    fig, (ax, axt) = plt.subplots(2, 1, figsize=(11, 6.5), gridspec_kw={"height_ratios": [3, 1.2]})
    fig.canvas.manager.set_window_title("Lab 4 – Đảo chiều + Dừng khẩn")
    (ln_rpm,) = ax.plot([], [], lw=1.6, label="tốc độ trục ra (rpm)")
    (ln_duty,) = ax.plot([], [], lw=1.0, ls="--", label="duty × hướng (%)")
    ax.axhline(0, color="k", lw=0.5)
    ax.set_xlabel("thời gian (s)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left")
    axt.axis("off")
    txt_state = axt.text(0.01, 0.75, "", fontsize=22, fontweight="bold", transform=axt.transAxes)
    txt_res = axt.text(0.01, 0.15, "", fontsize=13, transform=axt.transAxes)
    colors = {0: "#555555", 1: "#2ca02c", 2: "#ff7f0e", 3: "#d62728"}

    def update(_):
        a = link.last_rows(int(win / 0.01))
        if len(a) < 2:
            return ln_rpm, ln_duty, txt_state, txt_res
        t = (a[:, 0] - a[-1, 0]) / 1000.0
        ln_rpm.set_data(t, a[:, 4])
        ln_duty.set_data(t, a[:, 3] * a[:, 2] * np.where(a[:, 1] == 1, 1, 0))
        ax.set_xlim(-win, 0)
        lim = max(20.0, float(np.nanmax(np.abs(a[:, 4]))) * 1.15, 105.0)
        ax.set_ylim(-lim, lim)
        st = int(a[-1, 1])
        txt_state.set_text("{}   {:+.0f} rpm".format(STATE_NAME.get(st, "?"), a[-1, 4]))
        txt_state.set_color(colors.get(st, "k"))
        e = _last(link, m0, "ESTOP")
        r = _last(link, m0, "REV")
        s1 = "Dừng khẩn #{}: trục dừng sau {:.1f} ms (từ {:.0f} rpm, quay thêm {:.1f}°)".format(
            e["id"], e["dt_ms"], e["rpm0"], e["deg"]) if e else "Dừng khẩn: chưa có"
        s2 = "Đảo chiều #{}: về 0 sau {:.0f} ms, đạt 90 % chiều mới sau {:.0f} ms".format(
            r["id"], r["t_zero_ms"], r["t90_ms"]) if r else "Đảo chiều: chưa có"
        txt_res.set_text(s1 + "\n" + s2 + ("\n[GIẢ LẬP – không phải phần cứng thật]" if ctx.sim else ""))
        return ln_rpm, ln_duty, txt_state, txt_res

    anim = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
    fig.tight_layout()
    if ctx.args.duration:
        timer = fig.canvas.new_timer(interval=int(ctx.args.duration * 1000))
        timer.add_callback(plt.close, fig)
        timer.start()
    plt.show()
    del anim


# =============================================================== report
def _latest(ctx, kind):
    """Phien do moi nhat; phan tich LAI tu CSV goc (meta trong CSV giu ket qua firmware)."""
    files = sorted(glob.glob(os.path.join(ctx.logs, kind, "*", "trials.json")))
    if not files:
        return None, []
    with open(files[-1]) as f:
        tr = json.load(f)
    for t in tr:
        t["csv"] = os.path.join(HERE, t["csv"])
        meta, tm, c = A.load_capture(t["csv"])
        cpr = meta.get("cpr", ctx.cfg["cpr"])
        t.update(A.analyze_estop(tm, c, cpr, fw=meta) if kind == "estop" else A.analyze_rev(tm, c, cpr, fw=meta))
    return os.path.dirname(files[-1]), tr


def cmd_report(ctx):
    head("BƯỚC 5 – Tạo báo cáo")
    cpr = ctx.cfg["cpr"]
    fig_dir = os.path.join(ctx.report, "figures")
    os.makedirs(fig_dir, exist_ok=True)
    sess_e, te = _latest(ctx, "estop")
    sess_r, trv = _latest(ctx, "reverse")
    if not te and not trv:
        raise SystemExit("  Chưa có dữ liệu. Chạy trước: python3 lab4.py estop (và reverse).")
    sim_note = " [GIẢ LẬP]" if ctx.sim else ""
    L = []
    w = L.append
    w("# Lab 4 – Nhánh Robot: đảo chiều và dừng khẩn cấp động cơ DC" + (" (DỮ LIỆU GIẢ LẬP – KHÔNG NỘP)" if ctx.sim else ""))
    w("")
    w("Tạo tự động bởi `lab4.py report` lúc {}. Số liệu lấy từ `{}`{}.".format(
        datetime.datetime.now().strftime("%Y-%m-%d %H:%M"), os.path.relpath(ctx.logs, HERE),
        ", chạy với phần cứng giả lập (host_sim) nên chỉ dùng để kiểm tra quy trình" if ctx.sim else ""))
    w("")
    w("## 1. Yêu cầu")
    w("")
    w("Theo Lab 4, phần Thực hành về nhà của nhánh Robot yêu cầu *“lập trình đảo chiều + dừng khẩn cấp động cơ theo tín hiệu "
      "nút nhấn, đo thời gian đáp ứng dừng khẩn”* và nộp code cùng video demo.")
    w("")
    w("## 2. Phần cứng và đấu dây")
    w("")
    w("| Khối | Kết nối |")
    w("| --- | --- |")
    w("| ESP32-DevKitC-32D → TB6612FNG | IO25 = PWMA, IO26 = AIN1, IO27 = AIN2, 3V3 = VCC và STBY, GND = GND |")
    w("| TB6612FNG → động cơ JGA25-370 | AO1 = dây đỏ (M+), AO2 = dây trắng (M−) |")
    w("| Encoder | xanh dương = 3V3, đen = GND, vàng (A) = IO32, xanh lá (B) = IO33 |")
    w("| Nguồn động cơ | adapter 12 V → VM và GND của TB6612, GND chung với ESP32 |")
    w("| Nút ĐẢO CHIỀU | IO18 ↔ GND (kéo lên nội) |")
    w("| Nút DỪNG KHẨN | IO19 ↔ GND (kéo lên nội, ngắt cạnh xuống) |")
    w("")
    w("CPR = {:.1f} đếm/vòng trục ra ({}).".format(cpr, ctx.cfg.get("cpr_src", "")))
    chk = ctx.cfg.get("check")
    if chk:
        w("")
        w("Kiểm tra phần cứng (`lab4.py check`): nút DỪNG KHẨN {}, nút ĐẢO CHIỀU {}, reset bằng giữ nút {}, encoder {} "
          "({:.1f} rpm ở duty 40 %, enc_err = {}), đảo chiều {}, dừng khẩn {}.".format(
              *["đạt" if chk.get(k) else "LỖI" for k in ("btn_estop", "btn_dir", "reset_btn", "encoder")],
              chk.get("rpm_at_40", float("nan")), chk.get("enc_err"),
              *["đạt" if chk.get(k) else "LỖI" for k in ("reverse", "estop_soft")]))
    w("")
    w("## 3. Nguyên lý điều khiển")
    w("")
    w("Bảng chân lý của TB6612FNG (datasheet Toshiba) cho phép đổi chế độ chỉ bằng hai chân IN1 và IN2. Khi IN1 = IN2 = H thì "
      "driver vào chế độ **phanh ngắn** (hai đầu ra cùng nối GND) dù PWM ở mức nào. Khi IN1 = IN2 = L và PWM = H thì driver vào chế độ "
      "**stop**, hai đầu ra ở trạng thái trở kháng cao nên động cơ quay tự do. Firmware dùng điều này như sau:")
    w("")
    w("- **Dừng khẩn:** nút nối IO19 gây ngắt cạnh xuống. Hàm ngắt ghi thẳng thanh ghi `GPIO_OUT_W1TS` để đặt IN1 = IN2 = H "
      "(phanh ngắn) trước khi làm bất cứ việc gì khác, sau đó **chốt** trạng thái. Vòng lặp chính không thể ghi đè đầu ra khi đã chốt. "
      "Muốn chạy lại thì phải nhả nút, giữ nút ĐẢO CHIỀU 1 s để xoá chốt, rồi nhấn ĐẢO CHIỀU thêm một lần nữa, "
      "nên việc xoá chốt không tự làm động cơ quay.")
    w("- **Đảo chiều:** không đổi chiều đột ngột khi trục đang quay. Firmware phanh ngắn tới khi trục đứng yên "
      "(không dịch chuyển trong 30 ms), rồi mới đổi IN1/IN2 và tăng duty dần (ramp {} %/s).".format(400))
    w("")
    w("## 4. Phương pháp đo")
    w("")
    w("Mốc t = 0 là lúc vào hàm ngắt của nút DỪNG KHẨN, đo bằng `esp_timer` với độ phân giải 1 µs. Encoder đếm x4 bằng ngắt. "
      "Ngoài ra firmware ghi (thời gian, số đếm) mỗi 1 ms, giữ 100 ms trước sự kiện và gửi lên PC sau khi trục dừng.")
    w("")
    w("- **t_dừng**: lần cuối trục dịch chuyển ≥ 2 đếm so với mốc trước, tức ≈ {:.2f}° trục ra. Ngưỡng 2 đếm giúp bỏ qua rung ±1 đếm "
      "của encoder khi trục đứng yên. Trục được coi là đã dừng khi không dịch chuyển trong 150 ms. Firmware tính giá trị này theo thời gian "
      "thực trong ngắt encoder, còn PC tính lại bằng đúng thuật toán đó trên dữ liệu 1 kHz để đối chiếu.".format(2 / cpr * 360))
    w("- **t_10%**: lần đầu |tốc độ| ≤ 10 % tốc độ lúc nhấn. Tốc độ được tính bằng sai phân trung tâm ±2 ms để không làm nhoè thời điểm, "
      "còn tốc độ lúc nhấn là trung bình trong [−50, −5] ms với cửa sổ ±5 ms.")
    w("- **θ**: góc trục ra quay thêm từ lúc nhấn tới lúc dừng. **τ**: hằng số thời gian khi hãm, ước lượng bằng hồi quy ln|ω| theo t "
      "trong khoảng [0, t_10%].")
    w("- **Không đo được** độ trễ từ lúc tiếp điểm nút đóng tới lúc vào ngắt, vì muốn đo phải có máy hiện sóng. "
      "Vì vậy t_dừng ở đây tính từ lúc ESP32 nhận ngắt.")
    w("")
    if te:
        groups = []
        keys = []
        for t in te:
            k = (t["mode"], t["duty"])
            if k not in keys:
                keys.append(k)
        for mode, duty in keys:
            g = [t for t in te if t["mode"] == mode and t["duty"] == duty]
            r0 = A.mean_std([t["rpm0"] for t in g])
            ts = A.mean_std([t["t_stop_ms"] for t in g])
            t10 = A.mean_std([t["t10_ms"] for t in g])
            th = A.mean_std([t["theta_deg"] for t in g])
            tau = A.mean_std([t["tau_ms"] for t in g])
            groups.append({"mode": mode, "duty": duty, "rpm0_mean": r0[0], "t_mean": ts[0], "t_std": ts[1],
                           "t10_mean": t10[0], "t10_std": t10[1], "th_mean": th[0], "th_std": th[1],
                           "tau_mean": tau[0], "tau_std": tau[1], "n": ts[2]})
        A.plot_estop_curves(te, cpr, os.path.join(fig_dir, "estop_curves.png"), sim_note)
        A.plot_estop_summary(groups, os.path.join(fig_dir, "estop_summary.png"), sim_note)
        w("## 5. Kết quả dừng khẩn")
        w("")
        w("Phiên đo: `{}` ({} lần, nguồn kích: {}).".format(os.path.relpath(sess_e, HERE), len(te),
          ", ".join(sorted(set(str(t.get("src")) for t in te)))))
        w("")
        w("### 5.1. Tổng hợp (trung bình ± độ lệch chuẩn)")
        w("")
        w("| Chế độ | Duty (%) | n | rpm lúc nhấn | t_dừng (ms) | t_10% (ms) | θ (°) | τ (ms) |")
        w("| --- | --- | --- | --- | --- | --- | --- | --- |")
        for gr in groups:
            w("| {} | {} | {} | {} | {} ± {} | {} ± {} | {} ± {} | {} |".format(
                "phanh ngắn" if gr["mode"] == "brake" else "thả tự do", gr["duty"], gr["n"],
                nan2s(abs(gr["rpm0_mean"])), nan2s(gr["t_mean"]), nan2s(gr["t_std"]), nan2s(gr["t10_mean"]),
                nan2s(gr["t10_std"]), nan2s(abs(gr["th_mean"])), nan2s(gr["th_std"]), nan2s(gr["tau_mean"])))
        w("")
        w("![Đường tốc độ khi dừng khẩn](figures/estop_curves.png)")
        w("")
        w("![Tổng hợp thời gian dừng](figures/estop_summary.png)")
        w("")
        gb = [g for g in groups if g["mode"] == "brake"]
        gc = [g for g in groups if g["mode"] == "coast"]
        if gb and gc:
            same = [g for g in gb if g["duty"] == gc[0]["duty"]]
            if same and np.isfinite(same[0]["t_mean"]) and np.isfinite(gc[0]["t_mean"]) and same[0]["t_mean"] > 0:
                w("Ở cùng duty {} %, phanh ngắn dừng trục sau {:.1f} ms còn thả tự do mất {:.1f} ms, tức chậm hơn {:.1f} lần. "
                  "Góc quay thêm tương ứng là {:.1f}° và {:.1f}°. Kết quả này là lý do firmware chọn phanh ngắn làm chế độ dừng khẩn mặc định.".format(
                      gc[0]["duty"], same[0]["t_mean"], gc[0]["t_mean"], gc[0]["t_mean"] / same[0]["t_mean"],
                      abs(same[0]["th_mean"]), abs(gc[0]["th_mean"])))
                w("")
        w("### 5.2. Từng lần đo, đối chiếu firmware và PC")
        w("")
        w("| Lần | Chế độ | Duty | Nguồn | rpm lúc nhấn | t_dừng PC (ms) | t_dừng firmware (ms) | Lệch (ms) | t_10% (ms) | θ (°) |")
        w("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
        diffs = []
        for t in te:
            d = t["t_stop_ms"] - t["fw_dt_ms"] if np.isfinite(t.get("fw_dt_ms", np.nan)) else np.nan
            if np.isfinite(d):
                diffs.append(abs(d))
            w("| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |".format(
                t["tag"], t["mode"], t["duty"], t.get("src"), nan2s(abs(t["rpm0"])), nan2s(t["t_stop_ms"]),
                nan2s(t["fw_dt_ms"]), nan2s(d, "{:+.1f}"), nan2s(t["t10_ms"]), nan2s(abs(t["theta_deg"]))))
        w("")
        if diffs:
            w("Độ lệch lớn nhất giữa hai cách tính t_dừng là {:.1f} ms. Hai cách dùng cùng định nghĩa nhưng PC chỉ có mẫu 1 ms, "
              "nên độ lệch dưới khoảng 1–2 ms là bình thường.".format(max(diffs)))
            w("")
    if trv:
        A.plot_rev_curves(trv, cpr, os.path.join(fig_dir, "reverse_curves.png"), sim_note)
        tz = A.mean_std([t["t_zero_ms"] for t in trv])
        t90 = A.mean_std([t["t90_ms"] for t in trv])
        w("## 6. Kết quả đảo chiều")
        w("")
        w("Phiên đo: `{}` ({} lần ở duty {} %, nguồn kích: {}).".format(os.path.relpath(sess_r, HERE), len(trv),
          trv[0]["duty"], ", ".join(sorted(set(str(t.get("src")) for t in trv)))))
        w("")
        w("t = 0 là cạnh nhấn đầu tiên của nút. Firmware chỉ hành động sau khi nút ổn định 20 ms để chống dội phím, nên t_zero và t_90 "
          "đã bao gồm khoảng 20 ms này. Với dừng khẩn thì không có khoảng trễ này, vì ngắt hành động ngay ở cạnh đầu tiên.")
        w("")
        w("| Lần | rpm trước | rpm cuối bản ghi | t_zero (ms) | t_90 (ms) |")
        w("| --- | --- | --- | --- | --- |")
        for t in trv:
            w("| {} | {:+.1f} | {:+.1f} | {} | {} |".format(t["tag"], t["rpm0"], t["rpm1"], nan2s(t["t_zero_ms"]),
                                                       nan2s(t["t90_ms"])))
        w("| **TB ± ĐLC** | | | **{} ± {}** | **{} ± {}** |".format(nan2s(tz[0]), nan2s(tz[1]), nan2s(t90[0]), nan2s(t90[1])))
        w("")
        w("![Đảo chiều](figures/reverse_curves.png)")
        w("")
    w("## 7. Giới hạn")
    w("")
    w("- Nút nhấn thường hở (NO) nên không an toàn khi đứt dây, vì đứt dây thì nhấn nút cũng không gây dừng. Dừng khẩn công nghiệp dùng "
      "tiếp điểm thường đóng (NC) và cắt nguồn động lực bằng phần cứng. Bài này chỉ minh hoạ nguyên lý bằng phần mềm.")
    w("- Thời gian dừng phụ thuộc tải và quán tính gắn trên trục. Số đo ở đây ứng với trục ra không tải.")
    w("- Chế độ thả tự do được đặt trong ngắt bằng IN1 = IN2 = L. PWM chỉ được đưa về H ở vòng lặp kế tiếp, "
      "vì `ledcWrite` không gọi an toàn trong ngắt.")
    w("")
    path = os.path.join(ctx.report, "BAO_CAO_LAB4.md")
    with open(path, "w") as f:
        f.write("\n".join(L) + "\n")
    print("  Báo cáo: {}\n  Hình: {}".format(os.path.relpath(path, HERE), os.path.relpath(fig_dir, HERE)))
    return path


# =============================================================== all
def cmd_all(ctx):
    cmd_setup(ctx)
    if not cmd_check(ctx):
        raise SystemExit("\nDừng: còn lỗi phần cứng. Sửa theo gợi ý rồi chạy lại: python3 lab4.py all --no-flash")
    cmd_estop(ctx)
    cmd_reverse(ctx)
    path = cmd_report(ctx)
    head("XONG. Việc còn lại: quay video demo")
    print("  1. Dựng điện thoại quay cả động cơ, 2 nút nhấn và màn hình laptop.")
    print("  2. Chạy: bash lab4_robot/lab4.sh demo  rồi làm theo kịch bản trong 'Practical guide - Lab4 Robot.md' (mục 8).")
    print("  3. Nộp: thư mục lab4_robot/ (code) + video + {}{}".format(
        os.path.relpath(path, HERE), "  [GIẢ LẬP – KHÔNG NỘP]" if ctx.sim else ""))


def main():
    ap = argparse.ArgumentParser(description="Lab 4 – Nhánh Robot: tự động hoá", formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--port", default=None, help="cổng Serial, vd /dev/ttyUSB0 (mặc định tự tìm)")
    ap.add_argument("--sim", action="store_true", help="chạy với phần cứng GIẢ LẬP (firmware thật biên dịch cho PC)")
    ap.add_argument("--sim-enc-sign", type=int, default=1, help=argparse.SUPPRESS)
    ap.add_argument("-v", "--verbose", action="store_true")
    sub = ap.add_subparsers(dest="cmd")
    for name in ("setup", "all"):
        s = sub.add_parser(name)
        s.add_argument("--no-flash", action="store_true", help="không nạp firmware (đã nạp rồi)")
        if name == "all":
            s.add_argument("--soft", action="store_true", help="kích bằng lệnh Serial thay cho nút (chỉ để thử)")
            s.add_argument("--duties", type=float, nargs="+")
            s.add_argument("--reps", type=int)
            s.add_argument("--coast-reps", type=int)
            s.add_argument("--duty", type=float)
    sub.add_parser("check")
    s = sub.add_parser("estop")
    s.add_argument("--duties", type=float, nargs="+", help="các mức duty (%%), mặc định 40 70 100")
    s.add_argument("--reps", type=int, help="số lần mỗi mức, mặc định 3")
    s.add_argument("--coast-reps", type=int, help="số lần thả tự do, mặc định 3 (0 = bỏ)")
    s.add_argument("--soft", action="store_true", help="kích bằng lệnh Serial thay cho nút (chỉ để thử)")
    s = sub.add_parser("reverse")
    s.add_argument("--duty", type=float)
    s.add_argument("--reps", type=int)
    s.add_argument("--soft", action="store_true")
    s = sub.add_parser("demo")
    s.add_argument("--duty", type=float, help="duty khi chạy bằng nút, mặc định 70")
    s.add_argument("--duration", type=float, help="tự thoát sau N giây")
    s.add_argument("--headless", action="store_true", help="in chữ thay cho cửa sổ đồ thị")
    s.add_argument("--no-auto", action="store_true", help=argparse.SUPPRESS)
    sub.add_parser("report")
    args = ap.parse_args()
    if not args.cmd:
        ap.print_help()
        return
    for k in ("soft", "duties", "reps", "coast_reps", "duty", "no_flash"):
        if not hasattr(args, k):
            setattr(args, k, None if k not in ("soft", "no_flash") else False)
    ctx = Ctx(args)
    if ctx.sim:
        print("*** CHẾ ĐỘ GIẢ LẬP: firmware thật chạy trên PC với động cơ/encoder/nút GIẢ LẬP. "
              "Dữ liệu lưu ở *_sim, KHÔNG dùng để nộp. ***")
    try:
        {"setup": cmd_setup, "check": cmd_check, "estop": cmd_estop, "reverse": cmd_reverse,
         "demo": cmd_demo, "report": cmd_report, "all": cmd_all}[args.cmd](ctx)
    except DeviceError as e:
        raise SystemExit("\n[x] " + str(e))
    except KeyboardInterrupt:
        print("\n  Đã dừng (Ctrl+C).")
    finally:
        ctx.close()


if __name__ == "__main__":
    main()
