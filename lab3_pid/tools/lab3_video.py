"""Dem so vong quay tu video (co bang keo dan tren truc ra) bang OpenCV.

Nguyen ly: lay do sang trung binh trong mot vung nho (ROI) ma co bang keo di qua moi vong.
Tin hieu do sang co mot xung moi vong; dem canh len (co tre tu tri) va lay thoi diem
tu nhan thoi gian cua tung khung hinh => rpm = (so vong) / (thoi gian) * 60.
"""
import numpy as np


def count_revolutions(video, roi=None, plot_path=None):
    try:
        import cv2
    except ImportError:
        raise SystemExit("Thiếu OpenCV. Cài bằng: pip install opencv-python")
    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        raise SystemExit("Không mở được video: {}".format(video))
    ok, frame = cap.read()
    if not ok:
        raise SystemExit("Video rỗng: {}".format(video))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    if roi is None:
        print("Kéo chuột chọn một ô NHỎ mà cờ băng keo đi qua mỗi vòng (không chứa phần khác chuyển động), "
              "rồi nhấn Enter.")
        x, y, w, h = cv2.selectROI("Chon vung co bang keo", frame, False, False)
        cv2.destroyAllWindows()
        if w == 0 or h == 0:
            raise SystemExit("Chưa chọn vùng.")
        roi = (int(x), int(y), int(w), int(h))
    x, y, w, h = roi
    sig, ts = [], []
    idx = 0
    while ok:
        g = cv2.cvtColor(frame[y:y + h, x:x + w], cv2.COLOR_BGR2GRAY)
        sig.append(float(g.mean()))
        tm = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
        ts.append(tm if tm > 0 or idx == 0 else idx / fps)
        idx += 1
        ok, frame = cap.read()
    cap.release()
    s, t = np.array(sig), np.array(ts)
    if np.any(np.diff(t) <= 0):           # nhan thoi gian hong -> dung so khung / fps
        t = np.arange(len(s)) / fps
    lo, hi = np.percentile(s, 5), np.percentile(s, 95)
    if hi - lo < 8:
        raise SystemExit("Độ tương phản trong vùng chọn quá thấp ({:.1f} mức xám). Chọn vùng khác hoặc dùng "
                         "băng keo màu tương phản với nền.".format(hi - lo))
    # Co la trang thai hiem: neu phan lon thoi gian tin hieu gan muc thap thi co sang hon nen.
    bright_flag = np.median(s) < (lo + hi) / 2
    z = (s - lo) / (hi - lo) if bright_flag else (hi - s) / (hi - lo)
    on_th, off_th = 0.6, 0.4
    state, edges = False, []
    for i, v in enumerate(z):
        if not state and v > on_th:
            state = True
            edges.append(t[i])
        elif state and v < off_th:
            state = False
    edges = np.array(edges)
    if len(edges) < 3:
        raise SystemExit("Chỉ phát hiện {} lần cờ đi qua. Kiểm tra vùng chọn.".format(len(edges)))
    periods = np.diff(edges)
    med = float(np.median(periods))
    n_rev = len(edges) - 1
    dur = float(edges[-1] - edges[0])
    res = {"rpm": n_rev / dur * 60.0, "n_rev": n_rev, "duration": dur, "fps": float(fps), "roi": list(roi),
           "period_med": med, "period_max_ratio": float(periods.max() / med),
           "period_min_ratio": float(periods.min() / med)}
    if plot_path:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(9, 3))
        ax.plot(t, z, lw=0.8)
        ax.plot(edges, np.full(len(edges), on_th), "r|", ms=12, label="cờ đi qua ({} lần)".format(len(edges)))
        ax.axhline(on_th, color="r", lw=0.6, ls="--")
        ax.axhline(off_th, color="r", lw=0.6, ls=":")
        ax.set_xlabel("thời gian trong video (s)")
        ax.set_ylabel("độ sáng chuẩn hoá")
        ax.set_title("Đếm vòng từ video: {:.2f} rpm".format(res["rpm"]))
        ax.legend(loc="upper right")
        fig.tight_layout()
        fig.savefig(plot_path, dpi=150)
        plt.close(fig)
    return res
