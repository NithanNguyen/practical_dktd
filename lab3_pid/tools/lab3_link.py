"""Ket noi Serial voi firmware lab3_pid_motor (ESP32).

Firmware gui moi 10 ms mot dong "t_ms,mode,sp,rpm,u,d" va cac dong thong bao bat dau bang '#'.
Lop Link doc lien tuc trong mot luong rieng, luu toan bo du lieu vao bo nho de cac thi nghiem
cat ra doan can phan tich.
"""
import glob
import json
import os
import threading
import time

import numpy as np
import serial

COLS = ("t_ms", "mode", "sp", "rpm", "u", "d")


class DeviceError(Exception):
    pass


def find_port(preferred=None):
    """Tra ve cong Serial: dung cong chi dinh neu ton tai, neu khong thi tu tim."""
    if preferred and os.path.exists(preferred):
        return preferred
    cands = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if len(cands) == 1:
        return cands[0]
    if not cands:
        raise DeviceError("Không tìm thấy cổng ESP32 (/dev/ttyUSB* hoặc /dev/ttyACM*). Kiểm tra cáp USB.")
    raise DeviceError("Có nhiều cổng: {}. Chạy lại với --port <cổng>.".format(", ".join(cands)))


def _open(port, baud):
    # Dat DTR/RTS = False truoc khi mo de mach tu-reset cua ESP32 khong giu chip o trang thai reset.
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, baud, 0.05
    try:
        s.dtr = False
        s.rts = False
        s.open()
        return s
    except (OSError, serial.SerialException):
        try:
            s.close()
        except Exception:
            pass
    try:
        return serial.Serial(port, baud, timeout=0.05)
    except serial.SerialException as e:
        msg = str(e)
        if "ermission" in msg:
            msg += "\n  -> Chạy: sudo usermod -a -G dialout $USER, rồi đăng xuất và đăng nhập lại."
        if "busy" in msg or "Resource" in msg:
            msg += "\n  -> Đóng Serial Monitor của PlatformIO hoặc script khác đang mở cổng."
        raise DeviceError("Không mở được {}: {}".format(port, msg))


class Link:
    def __init__(self, port, baud=115200, verbose=False):
        self.port = port
        self.verbose = verbose
        self.ser = _open(port, baud)
        self.rows = []
        self.comments = []
        self.error = None
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self.th = threading.Thread(target=self._reader, daemon=True)
        self.th.start()
        time.sleep(0.3)
        st = self.query("?", "# kp=", timeout=3.0)
        if st is None:
            raise DeviceError("Không nhận phản hồi từ ESP32 trên {}. Kiểm tra firmware đã nạp, "
                              "baud 115200, và không có chương trình khác mở cổng.".format(port))

    # ------------------------------------------------------------ doc du lieu
    def _reader(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except (serial.SerialException, OSError) as e:
                self.error = "Mất kết nối Serial: {}".format(e)
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode("ascii", "ignore").strip()
                if not s:
                    continue
                if s.startswith("#"):
                    with self.lock:
                        self.comments.append(s)
                    if self.verbose:
                        print("   " + s)
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
                with self.lock:
                    self.rows.append(v)

    # ------------------------------------------------------------ lenh
    def send(self, cmd):
        if self.error:
            raise DeviceError(self.error)
        self.ser.write((cmd + "\n").encode("ascii"))
        self.ser.flush()
        time.sleep(0.03)

    def query(self, cmd, prefix, timeout=1.5):
        with self.lock:
            i = len(self.comments)
        self.send(cmd)
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                new = self.comments[i:]
            for s in new:
                if s.startswith(prefix):
                    return s
            time.sleep(0.01)
        return None

    def configure(self, cfg, kp=0.0, ki=0.0, kd=0.0, dmeas=0):
        cmds = ["stop", "dist 0", "stream 1", "win {}".format(cfg["win"]), "sign {}".format(cfg["sign"]),
                "umax 100", "dmeas {}".format(int(dmeas)),
                "kp {:.6g}".format(kp), "ki {:.6g}".format(ki), "kd {:.6g}".format(kd)]
        if cfg.get("cpr"):
            cmds.insert(5, "cpr {:.3f}".format(cfg["cpr"]))
        for c in cmds:
            self.send(c)
        n0 = self.n()
        self.wait(0.2)
        if self.n() == n0:
            raise DeviceError("Firmware không gửi dữ liệu (đã gõ 'stream 0'?).")

    # ------------------------------------------------------------ du lieu
    def n(self):
        with self.lock:
            return len(self.rows)

    def segment(self, i0, i1=None):
        with self.lock:
            a = np.array(self.rows[i0:i1], dtype=float)
        return a.reshape(-1, 6)

    def last(self, k):
        with self.lock:
            a = np.array(self.rows[-k:], dtype=float)
        return a.reshape(-1, 6)

    def mean_rpm(self, sec):
        a = self.last(max(1, int(round(sec / 0.01))))
        return float(a[:, 3].mean()) if len(a) else 0.0

    def wait(self, sec, check=None):
        """Cho sec giay. Neu check() tra True thi dung som va tra False."""
        t_end = time.time() + sec
        while time.time() < t_end:
            if self.error:
                raise DeviceError(self.error)
            if check is not None and check():
                return False
            time.sleep(0.02)
        return True

    def wait_stopped(self, timeout=4.0, thr=1.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            a = self.last(30)
            if len(a) >= 30 and np.all(np.abs(a[:, 3]) < thr):
                return True
            time.sleep(0.05)
        return False

    # ------------------------------------------------------------ dong
    def close(self):
        try:
            self.ser.write(b"dist 0\nstop\n")
            self.ser.flush()
            time.sleep(0.1)
        except Exception:
            pass
        self._stop.set()
        self.th.join(timeout=1)
        try:
            self.ser.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def save_csv(path, arr, meta):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        f.write("# meta " + json.dumps(meta, ensure_ascii=False) + "\n")
        f.write(",".join(COLS) + "\n")
        for r in arr:
            f.write("{:.0f},{:.0f},{:.2f},{:.2f},{:.2f},{:.1f}\n".format(*r))
