"""Phan tich du lieu chup 1 kHz (thoi gian, so dem) quanh su kien dung khan / dao chieu.

Moc thoi gian t = 0 la thoi diem nhan nut (dung khan: luc vao ISR; dao chieu: canh nhan dau tien).
Dinh nghia cac dai luong (dung chung cho bao cao):
  rpm0      toc do trung binh trong [-50, -5] ms truoc khi nhan (cua so +-5 ms)
  t_stop    lan cuoi truc dich chuyen >= 2 dem so voi moc truoc (DUNG thuat toan moveAnchor cua firmware)
  t_10      lan dau |toc do| <= 10 % |rpm0|  (toc do cua so +-2 ms de khong nhoe thoi diem)
  theta     goc truc ra quay them tu luc nhan toi luc dung = (dem_cuoi - dem_luc_nhan)/cpr*360
  tau       hang so thoi gian khi ham, uoc luong bang hoi quy ln|toc do| theo t trong [2 ms, t_10]
  t_zero    (dao chieu) lan dau |toc do| <= 2 % |rpm0| hoac doi dau (cua so +-2 ms)
  t_90      (dao chieu) lan dau toc do theo chieu moi >= 90 % |rpm0| (cua so +-2 ms)
"""
import json
import os

import numpy as np

STOP_BAND = 2          # dem
PRE_WIN = (-50.0, -5.0)  # ms
HALF_FINE = 2.0        # ms, cua so toc do de do thoi diem


def last_move_ms(t, c, anchor0=None, band=STOP_BAND):
    """Tai hien encISR cua firmware tren du lieu 1 kHz.

    Firmware: moi canh encoder (+-1 dem), neu |dem - moc| >= 2 thi moc = dem va ghi t_move.
    Giua 2 mau 1 ms so dem co the doi nhieu dem, nen moc duoc doi theo buoc 2 dem nhu firmware
    (moc += 2*trunc((dem - moc)/2)) de giu dung 'pha' cua moc. anchor0 = moc firmware luc nhan (anc).
    Ket qua tre hon firmware toi da 1 chu ky mau (1 ms).
    """
    post = t > 0
    tt, cc = t[post], c[post]
    if anchor0 is None or not np.isfinite(anchor0):
        anchor0 = cc[0] if len(cc) else 0.0
    anchor = float(anchor0)
    t_last = 0.0
    for ti, ci in zip(tt, cc):
        d = ci - anchor
        if abs(d) >= band:
            anchor += band * np.trunc(d / band)
            t_last = float(ti)
    return t_last


def save_capture(path, cap, meta):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        f.write("# meta " + json.dumps(meta, ensure_ascii=False) + "\n")
        f.write("t_ms,count\n")
        for t, c in zip(cap["t_us"], cap["count"]):
            f.write("{:.3f},{:.0f}\n".format(t / 1000.0, c))


def load_capture(path):
    meta = {}
    with open(path) as f:
        first = f.readline()
        if first.startswith("# meta "):
            meta = json.loads(first[7:])
    a = np.loadtxt(path, delimiter=",", skiprows=2, ndmin=2)
    return meta, a[:, 0], a[:, 1]


def speed_rpm(t_ms, count, cpr, half_ms=5.0):
    """Toc do (rpm) bang sai phan trung tam tren cua so +-half_ms (dung thoi gian that cua tung mau)."""
    t = np.asarray(t_ms, float)
    c = np.asarray(count, float)
    i_lo = np.searchsorted(t, t - half_ms, side="left")
    i_hi = np.searchsorted(t, t + half_ms, side="right") - 1
    i_lo = np.clip(i_lo, 0, len(t) - 1)
    i_hi = np.clip(i_hi, 0, len(t) - 1)
    dt = t[i_hi] - t[i_lo]
    v = np.full(len(t), np.nan)
    ok = dt > 0.5 * half_ms
    v[ok] = (c[i_hi][ok] - c[i_lo][ok]) / cpr / (dt[ok] / 60000.0)
    return v


def _count_at(t, c, t0):
    if t0 <= t[0]:
        return float(c[0])
    if t0 >= t[-1]:
        return float(c[-1])
    return float(np.interp(t0, t, c))


def _rpm0(t, v, fallback):
    m = (t >= PRE_WIN[0]) & (t <= PRE_WIN[1]) & np.isfinite(v)
    if m.sum() >= 10:
        return float(np.mean(v[m])), True
    return float(fallback), False


def analyze_estop(t, c, cpr, fw=None):
    fw = fw or {}
    t = np.asarray(t, float)
    c = np.asarray(c, float)
    rpm0, pre_ok = _rpm0(t, speed_rpm(t, c, cpr), fw.get("rpm0", np.nan))
    v = speed_rpm(t, c, cpr, HALF_FINE)
    c_press = _count_at(t, c, 0.0)
    tail = c[-min(20, len(c)):]
    c_final = float(np.median(tail))
    post = t >= 0
    t_stop = last_move_ms(t, c, fw.get("anc"))
    a0 = abs(rpm0)
    t10 = t50 = np.nan
    if a0 > 1.0:
        m = post & np.isfinite(v)
        below10 = m & (np.abs(v) <= 0.1 * a0)
        below50 = m & (np.abs(v) <= 0.5 * a0)
        # sau t_stop truc khong con dich chuyen nen toc do da ve ~0: cua so +-2 ms chi lam nhoe -> chan tren
        t10 = min(float(t[below10][0]), t_stop) if below10.any() else t_stop
        t50 = min(float(t[below50][0]), t_stop) if below50.any() else t_stop
    tau = r2 = np.nan
    if np.isfinite(t10) and t10 > 3:
        m = post & (t <= t10) & np.isfinite(v) & (np.abs(v) > 0.1 * a0) & (t >= 2.0)
        if m.sum() >= 5:
            y = np.log(np.abs(v[m]))
            A = np.vstack([t[m], np.ones(m.sum())]).T
            sol, *_ = np.linalg.lstsq(A, y, rcond=None)
            if sol[0] < 0:
                tau = float(-1.0 / sol[0])
                yhat = A @ sol
                ss = float(np.sum((y - y.mean()) ** 2))
                r2 = float(1 - np.sum((y - yhat) ** 2) / ss) if ss > 0 else np.nan
    return {"rpm0": rpm0, "pre_ok": bool(pre_ok and t[0] <= PRE_WIN[0]), "t_stop_ms": t_stop,
            "t10_ms": t10, "t50_ms": t50, "theta_deg": (c_final - c_press) / cpr * 360.0,
            "tau_ms": tau, "tau_r2": r2, "complete": bool(t[0] <= 0.0),
            "fw_dt_ms": fw.get("dt_ms", np.nan), "fw_deg": fw.get("deg", np.nan)}


def analyze_rev(t, c, cpr, fw=None):
    fw = fw or {}
    t = np.asarray(t, float)
    c = np.asarray(c, float)
    v5 = speed_rpm(t, c, cpr)
    rpm0, pre_ok = _rpm0(t, v5, fw.get("rpm0", np.nan))
    v = speed_rpm(t, c, cpr, HALF_FINE)
    s = 1.0 if rpm0 >= 0 else -1.0
    post = (t >= 0) & np.isfinite(v)
    zero = post & (v * s <= 0.02 * abs(rpm0))
    t_zero = float(t[zero][0]) if zero.any() else np.nan
    reach = post & (-v * s >= 0.9 * abs(rpm0))
    t90 = float(t[reach][0]) if reach.any() else np.nan
    m_end = (t >= 0) & np.isfinite(v5) & (t >= t[-1] - 50)
    rpm1 = float(np.mean(v5[m_end])) if m_end.any() else np.nan
    return {"rpm0": rpm0, "pre_ok": bool(pre_ok), "t_zero_ms": t_zero, "t90_ms": t90, "rpm1": rpm1,
            "fw_t_zero_ms": fw.get("t_zero_ms", np.nan), "fw_t90_ms": fw.get("t90_ms", np.nan)}


def mean_std(x):
    x = np.asarray([v for v in x if v is not None and np.isfinite(v)], float)
    if len(x) == 0:
        return np.nan, np.nan, 0
    return float(x.mean()), float(x.std(ddof=1)) if len(x) > 1 else 0.0, len(x)


# =============================================================== do thi
def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def plot_estop_curves(trials, cpr, path, title_suffix=""):
    plt = _plt()
    fig, ax = plt.subplots(figsize=(9, 4.5))
    colors = {"brake": "#1f77b4", "coast": "#d62728"}
    shown = set()
    for tr in trials:
        meta, t, c = load_capture(tr["csv"])
        v = speed_rpm(t, c, cpr)
        s = 1.0 if tr["rpm0"] >= 0 else -1.0
        lab = None
        key = (tr["mode"],)
        if key not in shown:
            lab = "phanh ngắn (brake)" if tr["mode"] == "brake" else "thả tự do (coast)"
            shown.add(key)
        ax.plot(t, v * s, color=colors.get(tr["mode"], "k"), lw=0.9, alpha=0.75, label=lab)
        ax.plot([tr["t_stop_ms"]], [0], "o", color=colors.get(tr["mode"], "k"), ms=4)
    ax.axvline(0, color="k", lw=0.8, ls="--")
    ax.text(2, ax.get_ylim()[1] * 0.92, "nhấn DỪNG KHẨN", fontsize=8)
    ax.set_xlim(-60, max(150, max(tr["t_stop_ms"] for tr in trials) + 60))
    ax.set_xlabel("thời gian kể từ lúc nhấn (ms)")
    ax.set_ylabel("tốc độ trục ra theo chiều đang quay (rpm)")
    ax.set_title("Dừng khẩn: tốc độ theo thời gian (chấm = t_dừng)" + title_suffix)
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_estop_summary(groups, path, title_suffix=""):
    """groups: list of dict(mode, duty, rpm0_mean, t_mean, t_std, th_mean, th_std, n)"""
    plt = _plt()
    fig, axs = plt.subplots(1, 2, figsize=(10, 4))
    for mode, col, mk in (("brake", "#1f77b4", "o"), ("coast", "#d62728", "s")):
        g = [x for x in groups if x["mode"] == mode]
        if not g:
            continue
        lab = "phanh ngắn" if mode == "brake" else "thả tự do"
        x = [abs(e["rpm0_mean"]) for e in g]
        axs[0].errorbar(x, [e["t_mean"] for e in g], yerr=[e["t_std"] for e in g], fmt=mk + "-", color=col,
                        capsize=4, label=lab)
        axs[1].errorbar(x, [abs(e["th_mean"]) for e in g], yerr=[e["th_std"] for e in g], fmt=mk + "-",
                        color=col, capsize=4, label=lab)
    axs[0].set_xlabel("|rpm0| lúc nhấn (rpm)")
    axs[0].set_ylabel("t_dừng (ms), TB ± độ lệch chuẩn")
    axs[1].set_xlabel("|rpm0| lúc nhấn (rpm)")
    axs[1].set_ylabel("góc quay thêm |θ| (độ)")
    for a in axs:
        a.grid(alpha=0.3)
        a.legend()
        a.set_xlim(left=0)
        a.set_ylim(bottom=0)
    fig.suptitle("Thời gian dừng và góc quay thêm theo tốc độ ban đầu" + title_suffix)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_rev_curves(trials, cpr, path, title_suffix=""):
    plt = _plt()
    fig, ax = plt.subplots(figsize=(9, 4.5))
    for i, tr in enumerate(trials):
        meta, t, c = load_capture(tr["csv"])
        v = speed_rpm(t, c, cpr)
        ax.plot(t, v, lw=0.9, alpha=0.8, label="lần {}".format(i + 1))
        if np.isfinite(tr["t_zero_ms"]):
            ax.plot([tr["t_zero_ms"]], [0], "k|", ms=10)
        if np.isfinite(tr["t90_ms"]):
            ax.plot([tr["t90_ms"]], [-0.9 * tr["rpm0"]], "kx", ms=6)
    ax.axvline(0, color="k", lw=0.8, ls="--")
    ax.axhline(0, color="k", lw=0.5)
    ax.set_xlabel("thời gian kể từ lúc nhấn ĐẢO CHIỀU (ms)")
    ax.set_ylabel("tốc độ trục ra (rpm, có dấu)")
    ax.set_title("Đảo chiều: phanh tới 0 rồi tăng tốc theo chiều mới (| = t_zero, x = t_90)" + title_suffix)
    ax.grid(alpha=0.3)
    if len(trials) <= 8:
        ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
