"""Do thi cho Lab 3 (luu PNG, khong mo cua so)."""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from lab3_analysis import fopdt_curve, rel_t  # noqa: E402

plt.rcParams.update({"figure.dpi": 110, "savefig.dpi": 150, "axes.grid": True, "grid.alpha": 0.35,
                     "font.size": 10})
C_MAIN, C_REF, C_U, C_D = "#1f5fbf", "#222222", "#2e8b57", "#c0392b"


def _save(fig, path):
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path


def static_curve(duty, rpm, dz, r, u_r, path):
    fig, ax = plt.subplots(figsize=(7, 4.2))
    ax.plot(duty, rpm, "o-", color=C_MAIN, label="tốc độ xác lập đo được")
    ax.axvspan(0, dz, color="0.85", label="vùng chết (duty < {:.0f} %)".format(dz))
    ax.axhline(r, ls="--", color=C_REF, lw=1, label="vận tốc đặt r = {:.0f} rpm".format(r))
    ax.axvline(u_r, ls=":", color=C_REF, lw=1)
    ax.set_xlabel("duty u (%)")
    ax.set_ylabel("tốc độ trục ra (rpm)")
    ax.set_title("Đặc tính tĩnh của động cơ (vòng hở)")
    ax.set_xlim(0, 102)
    ax.legend(loc="upper left")
    return _save(fig, path)


def ident(runs, path):
    """runs: list (arr, i_step, fit)."""
    fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(8, 5.5), gridspec_kw={"height_ratios": [3, 1]})
    for k, (arr, i_step, fit) in enumerate(runs):
        t = rel_t(arr, i_step)
        lab = "đo, lần {}".format(k + 1)
        a1.plot(t, arr[:, 3], lw=1, alpha=0.75, label=lab)
        if k == 0:
            a2.plot(t, arr[:, 4], color=C_U)
    fit = runs[0][2]
    tm = np.linspace(0, rel_t(runs[0][0], runs[0][1])[-1], 400)
    a1.plot(tm, fopdt_curve(tm, fit["y0"], fit["yf"] - fit["y0"], fit["T"], fit["L"]), "--", color=C_REF, lw=1.6,
            label="mô hình: K={:.3f} rpm/%, T={:.0f} ms, L={:.0f} ms".format(fit["K"], fit["T"] * 1e3, fit["L"] * 1e3))
    a1.set_ylabel("rpm")
    a1.set_title("Đáp ứng bậc vòng hở và mô hình bậc nhất có trễ (FOPDT)")
    a1.legend(loc="lower right", fontsize=9)
    a2.set_ylabel("u (%)")
    a2.set_xlabel("thời gian sau bước nhảy (s)")
    a1.set_xlim(-0.5, None)
    return _save(fig, path)


def trials(items, r, path, title, xmax=3.0):
    """items: list (arr, i_step, label, highlight)."""
    fig, ax = plt.subplots(figsize=(8, 4.8))
    for arr, i_step, label, hl in items:
        t = rel_t(arr, i_step)
        ax.plot(t, arr[:, 3], lw=2.2 if hl else 1.0, alpha=1.0 if hl else 0.7, label=label,
                color=C_MAIN if hl else None, zorder=3 if hl else 2)
    ax.axhline(r, ls="--", color=C_REF, lw=1, label="r = {:.0f} rpm".format(r))
    ax.set_xlim(-0.05 * xmax, min(xmax, max(rel_t(a, i)[-1] for a, i, _, _ in items)))
    ax.set_xlabel("thời gian sau bước nhảy (s)")
    ax.set_ylabel("rpm")
    ax.set_title(title)
    ax.legend(fontsize=8, ncol=2, loc="lower right")
    return _save(fig, path)


def final_step(runs, r, band_abs, stats, gains, path):
    fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(8, 5.5), gridspec_kw={"height_ratios": [3, 1]})
    for k, (arr, i_step) in enumerate(runs):
        t = rel_t(arr, i_step)
        a1.plot(t, arr[:, 3], lw=1.3, label="lần {}".format(k + 1))
        a2.plot(t, arr[:, 4], lw=1)
    a1.axhline(r, ls="--", color=C_REF, lw=1, label="r = {:.0f} rpm".format(r))
    a1.axhspan(r - band_abs, r + band_abs, color="orange", alpha=0.15, label="dải ±5 %")
    if np.isfinite(stats["ts"][0]):
        a1.axvline(stats["ts"][0], color=C_D, ls=":", lw=1, label="t_s trung bình")
    a1.set_title("Kp={:.4g}, Ki={:.4g}, Kd={:.4g} | POT={:.1f}±{:.1f} %, t_s={:.0f}±{:.0f} ms, e_ss={:+.2f}±{:.2f} %".format(
        gains[0], gains[1], gains[2], stats["pot"][0], stats["pot"][1], stats["ts"][0] * 1e3, stats["ts"][1] * 1e3,
        stats["ess_pct"][0], stats["ess_pct"][1]), fontsize=9)
    a1.set_ylabel("rpm")
    a1.legend(loc="lower right", fontsize=9)
    a2.set_ylabel("u (%)")
    a2.set_xlabel("thời gian sau bước nhảy (s)")
    a1.set_xlim(-0.2, None)
    return _save(fig, path)


def dist(runs, r, band_abs, path, title):
    """runs: list (arr, label, spans) - spans: list (t0, t1) to mau."""
    fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6), gridspec_kw={"height_ratios": [3, 1.4]})
    for k, (arr, label, spans) in enumerate(runs):
        t = rel_t(arr)
        a1.plot(t, arr[:, 3], lw=1.8 if k == 0 else 1.0, label=label, zorder=3 - k)
        a2.plot(t, arr[:, 4], lw=1.6 if k == 0 else 0.9, label="u – " + label)
        if k == 0:
            for t0, t1 in spans:
                a1.axvspan(t0, t1, color=C_D, alpha=0.08)
            if np.any(arr[:, 5] > 0):
                a2.plot(t, arr[:, 5], color=C_D, ls="--", lw=1.2, label="nhiễu tải d")
    a1.axhline(r, ls="--", color=C_REF, lw=1, label="r = {:.0f} rpm".format(r))
    a1.axhspan(r - band_abs, r + band_abs, color="orange", alpha=0.15, label="dải ±{:.1f} rpm".format(band_abs))
    a1.set_ylabel("rpm")
    a1.set_title(title)
    a1.legend(loc="lower left", fontsize=8)
    a2.set_ylabel("% duty")
    a2.set_xlabel("t (s)")
    a2.legend(loc="upper left", fontsize=8, ncol=2)
    return _save(fig, path)


def verify(arr, i0, rpm_enc, path, rpm_video=None):
    fig, ax = plt.subplots(figsize=(8, 3.8))
    t = rel_t(arr)
    ax.plot(t, arr[:, 3], lw=0.8, color=C_MAIN, label="encoder (mỗi 10 ms)")
    ax.axhline(rpm_enc, color=C_REF, lw=1.2, label="encoder, trung bình = {:.2f} rpm".format(rpm_enc))
    if rpm_video:
        ax.axhline(rpm_video, color=C_D, ls="--", lw=1.2, label="video = {:.2f} rpm".format(rpm_video))
    ax.axvline(t[i0], color="0.5", ls=":", lw=1)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("rpm")
    ax.set_title("Kiểm chứng vận tốc đo bằng encoder")
    ax.legend(loc="lower right")
    return _save(fig, path)
