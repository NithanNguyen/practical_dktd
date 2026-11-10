"""Cac ham phan tich thuan (khong dung Serial) cho Lab 3.

Mang du lieu: N x 6, cot = t_ms, mode, sp, rpm, u, d (xem lab3_link.COLS).
"""
import json

import numpy as np

TS = 0.01


def load_csv(path):
    rows, meta = [], {}
    with open(path) as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("t_ms"):
                continue
            if s.startswith("# meta "):
                try:
                    meta = json.loads(s[len("# meta "):])
                except ValueError:
                    pass
                continue
            if s.startswith("#"):
                continue
            p = s.split(",")
            if len(p) not in (5, 6):
                continue
            try:
                v = [float(x) for x in p]
            except ValueError:
                continue
            if len(v) == 5:
                v.append(0.0)
            rows.append(v)
    return np.array(rows, dtype=float).reshape(-1, 6), meta


def rel_t(arr, i_ref=0):
    return (arr[:, 0] - arr[i_ref, 0]) / 1000.0


def first_index(cond, start=0):
    idx = np.where(cond[start:])[0]
    return int(idx[0] + start) if len(idx) else None


def crossing_time(t, y, level, rising=True):
    idx = np.where(y >= level)[0] if rising else np.where(y <= level)[0]
    if len(idx) == 0:
        return float("nan")
    i = idx[0]
    if i == 0 or y[i] == y[i - 1]:
        return float(t[i])
    return float(t[i - 1] + (level - y[i - 1]) * (t[i] - t[i - 1]) / (y[i] - y[i - 1]))


def settle_time(tt, y, ref, band_abs, hold=0.3):
    """Thoi diem som nhat ke tu do |y - ref| <= band_abs lien tuc den het doan va it nhat `hold` giay."""
    out = np.where(np.abs(y - ref) > band_abs)[0]
    if len(out) == 0:
        ts = float(tt[0])
    elif out[-1] + 1 >= len(tt):
        return float("nan")
    else:
        ts = float(tt[out[-1] + 1])
    return ts if tt[-1] - ts >= hold else float("nan")


# --------------------------------------------------------------- dap ung bac vong kin
def step_metrics(arr, i_step, r, band=0.05, umax=100.0):
    t = rel_t(arr, i_step)
    y, u = arr[:, 3], arr[:, 4]
    pre = (t < 0) & (t >= -0.5)
    y0 = float(y[pre].mean()) if pre.any() else 0.0
    k = t >= 0
    tt, yy, uu = t[k], y[k], u[k]
    tail = tt >= tt[-1] - 1.0
    yss = float(yy[tail].mean())
    noise = float(yy[tail].std())
    dy = yss - y0
    m = {"r": r, "y0": y0, "yss": yss, "noise_rpm": noise, "dur": float(tt[-1])}
    if abs(dy) < 1e-6:
        m.update(pot=float("nan"), tr=float("nan"), ts=float("nan"), ess=r - yss,
                 ess_pct=(r - yss) / r * 100, iae=float("nan"), u_noise=float(uu[tail].std()),
                 u_sat=0.0, ymax=float(yy.max()))
        return m
    sgn = 1.0 if dy > 0 else -1.0
    peak = float(np.max(sgn * (yy - yss)))
    m["ymax"] = float(yy.max()) if sgn > 0 else float(yy.min())
    m["pot"] = max(0.0, peak / abs(dy) * 100.0)
    t10 = crossing_time(tt, sgn * yy, sgn * (y0 + 0.1 * dy))
    t90 = crossing_time(tt, sgn * yy, sgn * (y0 + 0.9 * dy))
    m["tr"] = t90 - t10
    m["band_abs"] = band * abs(dy)
    m["ts"] = settle_time(tt, yy, yss, m["band_abs"], hold=0.5)
    m["ess"] = r - yss
    m["ess_pct"] = (r - yss) / r * 100.0
    m["iae"] = float(np.sum(np.abs(r - yy)) * TS)
    m["u_noise"] = float(uu[tail].std())
    m["u_sat"] = float(np.mean(uu >= umax - 0.5) * 100.0)
    return m


# --------------------------------------------------------------- nhan dang FOPDT
def fopdt_smith(arr, i_step):
    """Mo hinh bac nhat co tre G(s) = K e^{-Ls} / (Ts + 1) tu dap ung bac vong ho,
    phuong phap hai diem cua Smith (28.3 % va 63.2 %)."""
    t = rel_t(arr, i_step)
    y, u = arr[:, 3], arr[:, 4]
    pre = (t < 0) & (t >= -0.5)
    y0, u0 = float(y[pre].mean()), float(u[pre].mean())
    k = t >= 0
    tt, yy = t[k], y[k]
    u1 = float(u[k][-1])
    tail = tt >= tt[-1] - 0.5
    yf = float(yy[tail].mean())
    dy, du = yf - y0, u1 - u0
    t28 = crossing_time(tt, yy, y0 + 0.283 * dy)
    t63 = crossing_time(tt, yy, y0 + 0.632 * dy)
    T = 1.5 * (t63 - t28)
    L = max(0.0, t63 - T)
    K = dy / du
    ym = fopdt_curve(tt, y0, dy, T, L)
    rmse = float(np.sqrt(np.mean((yy - ym) ** 2)))
    return {"K": K, "T": T, "L": L, "y0": y0, "yf": yf, "u0": u0, "u1": u1,
            "t28": t28, "t63": t63, "rmse": rmse, "rmse_pct": rmse / abs(dy) * 100}


def fopdt_curve(tt, y0, dy, T, L):
    return y0 + dy * (1 - np.exp(-np.clip(tt - L, 0, None) / max(T, 1e-6)))


def zn_rules(K, T, L):
    """Ziegler-Nichols (phuong phap dap ung bac), quy ve dang lab: Ki = Kp/Ti, Kd = Kp*Td."""
    L = max(L, TS)
    a = T / (K * L)
    return {
        "ZN-P": (1.0 * a, 0.0, 0.0),
        "ZN-PI": (0.9 * a, 0.9 * a / (L / 0.3), 0.0),
        "ZN-PID": (1.2 * a, 1.2 * a / (2.0 * L), 1.2 * a * 0.5 * L),
    }


def simc_pi(K, T, L, tauc):
    """SIMC PI (Skogestad 2003): Kp = T / (K (tau_c + L)), Ti = min(T, 4 (tau_c + L))."""
    L = max(L, TS)
    kp = T / (K * (tauc + L))
    ti = min(T, 4.0 * (tauc + L))
    return kp, kp / ti, 0.0


# --------------------------------------------------------------- nhieu tai
def dist_known(arr, edges, r, band_abs):
    """Phan tich nhieu tai co thoi diem biet truoc (nhieu dien tuong duong).
    edges: chi so cac dong ma cot d thay doi."""
    t = rel_t(arr)
    y, u, d = arr[:, 3], arr[:, 4], arr[:, 5]
    out = []
    for j, i in enumerate(edges):
        i_end = edges[j + 1] if j + 1 < len(edges) else len(arr)
        tt = t[i:i_end] - t[i]
        yy, uu = y[i:i_end], u[i:i_end]
        dev = yy - r
        pk = int(np.argmax(np.abs(dev)))
        rec = settle_time(tt, yy, r, band_abs, hold=0.3)
        last = tt >= tt[-1] - 0.5
        out.append({
            "t_on": float(t[i]), "d_after": float(d[i]), "d_before": float(d[i - 1]) if i > 0 else 0.0,
            "loai": "có tải" if d[i] > (d[i - 1] if i > 0 else 0) else "bỏ tải",
            "dev_peak": float(dev[pk]), "dev_peak_pct": float(dev[pk] / r * 100), "t_peak": float(tt[pk]),
            "recovery": rec, "iae": float(np.sum(np.abs(dev)) * TS),
            "e_end": float(np.mean(r - yy[last])), "u_end": float(np.mean(uu[last])), "u_max": float(uu.max()),
        })
    return out


def dist_detect(arr, r, t_quiet=(2.0, 3.8), band=0.05):
    """Tu phat hien su kien tai (can tay) trong mot ban ghi vong kin o setpoint r."""
    t = rel_t(arr)
    y, u = arr[:, 3], arr[:, 4]
    quiet = (t >= t_quiet[0]) & (t < t_quiet[1])
    sd = float(y[quiet].std()) if quiet.any() else 1.0
    thr = max(4 * sd, band * r)
    dev = y - r
    out = (np.abs(dev) > thr) & (t > t_quiet[0])
    events, i, n = [], 0, len(t)
    hold = int(round(0.3 / TS))
    while i < n:
        if not out[i]:
            i += 1
            continue
        j = i
        while j < n and out[j:j + hold].any():
            j += 1
        seg = slice(i, min(j, n))
        pk = int(np.argmax(np.abs(dev[seg]))) + i
        ended = j < n
        events.append({
            "t_start": float(t[i]), "loai": "giảm tốc (đang cản)" if dev[pk] < 0 else "vọt lên (vừa thả)",
            "dev_peak": float(dev[pk]), "dev_peak_pct": float(dev[pk] / r * 100), "t_peak": float(t[pk]),
            "recovery": float(t[min(j, n - 1)] - t[i]) if ended else float("nan"),
            "u_max": float(u[seg].max()),
        })
        i = j
    return events, thr, sd
