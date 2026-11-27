"""Ket noi voi firmware lab4_robot: qua Serial that, hoac qua ban gia lap tren PC (--sim).

Firmware gui:
  - moi 10 ms mot dong CSV "t_ms,state,dir,duty,rpm,count"
  - "#RES ESTOP k=v ..." / "#RES REV k=v ..." : ket qua do
  - "#EV ...", "#BTN <DIR|ESTOP> <DOWN|UP>", "#ST k=v ...", "#ERR ...", "#OK ..."
  - "#CAP BEGIN kind=.. id=.. tref=.. n=..", cac dong "#C,<t_us_tuong_doi>,<so_dem>", "#CAP END ..."
Lop Link doc lien tuc trong mot luong rieng va xep cac loai dong vao danh sach tuong ung.
"""
import glob
import os
import queue
import random
import shutil
import subprocess
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FW_SRC = os.path.join(HERE, "..", "firmware", "src", "main.cpp")
SIM_DIR = os.path.join(HERE, "host_sim")
SIM_BIN = os.path.join(SIM_DIR, "build", "lab4_host")
FW_TAG = "lab4_robot"


class DeviceError(Exception):
    pass


def kv(line):
    """'#RES ESTOP id=3 mode=brake dt_ms=48.2' -> {'id': 3, 'mode': 'brake', 'dt_ms': 48.2}"""
    out = {}
    for tok in line.split():
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        try:
            out[k] = int(v)
        except ValueError:
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
    return out


def find_port(preferred=None):
    if preferred and os.path.exists(preferred):
        return preferred
    cands = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if len(cands) == 1:
        return cands[0]
    if not cands:
        raise DeviceError("Không tìm thấy cổng ESP32 (/dev/ttyUSB* hoặc /dev/ttyACM*). Kiểm tra cáp micro-USB "
                          "(phải là cáp có dây dữ liệu).")
    raise DeviceError("Có nhiều cổng: {}. Chạy lại với --port <cổng>.".format(", ".join(cands)))


# =============================================================== transport
class SerialTransport:
    def __init__(self, port, baud=115200):
        try:
            import serial
        except ImportError:
            raise DeviceError("Thiếu pyserial. Chạy: pip install -r requirements.txt")
        self.serial = serial
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = port, baud, 0.05
        s.dtr = False      # DTR/RTS = False truoc khi mo: mach tu-reset khong giu ESP32 o trang thai reset
        s.rts = False
        try:
            s.open()
        except (OSError, serial.SerialException) as e:
            msg = str(e)
            if "ermission" in msg:
                msg += ("\n  -> Chạy: sudo usermod -a -G dialout $USER, rồi đăng xuất và đăng nhập lại.")
            if "busy" in msg or "Resource" in msg:
                msg += "\n  -> Đóng Serial Monitor của PlatformIO hoặc script khác đang mở cổng."
            raise DeviceError("Không mở được {}: {}".format(port, msg))
        self.s = s
        self.buf = b""
        self.name = port

    def write(self, text):
        self.s.write(text.encode("ascii"))
        self.s.flush()

    def readline(self):
        while b"\n" not in self.buf:
            try:
                chunk = self.s.read(self.s.in_waiting or 1)
            except (self.serial.SerialException, OSError) as e:
                raise DeviceError("Mất kết nối Serial: {}".format(e))
            if not chunk:
                return None
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return line.decode("ascii", "ignore").strip()

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass


def build_sim(force=False):
    """Bien dich firmware/src/main.cpp + harness gia lap thanh chuong trinh chay tren PC."""
    srcs = [FW_SRC, os.path.join(SIM_DIR, "harness.cpp"), os.path.join(SIM_DIR, "Arduino.h")]
    if not force and os.path.exists(SIM_BIN) and all(os.path.getmtime(SIM_BIN) >= os.path.getmtime(s) for s in srcs):
        return SIM_BIN
    gxx = shutil.which("g++")
    if not gxx:
        raise DeviceError("Chế độ --sim cần g++ (sudo apt install build-essential).")
    os.makedirs(os.path.dirname(SIM_BIN), exist_ok=True)
    cmd = [gxx, "-std=gnu++17", "-O2", "-I", SIM_DIR, "-o", SIM_BIN,
           os.path.join(SIM_DIR, "harness.cpp"), FW_SRC]
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
    if r.returncode != 0:
        raise DeviceError("Biên dịch bản giả lập thất bại:\n" + r.stdout)
    return SIM_BIN


class SimTransport:
    """Chay chinh firmware/src/main.cpp tren PC voi dong co/encoder/nut nhan GIA LAP."""

    def __init__(self, enc_sign=1):
        exe = build_sim()
        args = [exe] + (["--enc-sign", "-1"] if enc_sign < 0 else [])
        self.p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  universal_newlines=True, bufsize=1)
        self.q = queue.Queue()
        self.name = "GIẢ LẬP (host_sim)"
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        for line in self.p.stdout:
            self.q.put(line.strip())
        self.q.put(None)

    def write(self, text):
        try:
            self.p.stdin.write(text)
            self.p.stdin.flush()
        except (BrokenPipeError, OSError):
            raise DeviceError("Bản giả lập đã dừng.")

    def readline(self):
        try:
            line = self.q.get(timeout=0.05)
        except queue.Empty:
            return None
        if line is None:
            raise DeviceError("Bản giả lập đã dừng.")
        return line

    def close(self):
        try:
            self.p.stdin.write("!quit\n")
            self.p.stdin.flush()
            self.p.wait(timeout=2)
        except Exception:
            self.p.kill()


# =============================================================== link
class Link:
    def __init__(self, transport, verbose=False):
        self.tr = transport
        self.sim = isinstance(transport, SimTransport)
        self.verbose = verbose
        self.lock = threading.Lock()
        self.rows = []        # [t_ms, state, dir, duty, rpm, count]
        self.comments = []    # moi dong '#'
        self.results = []     # dict, co khoa 'kind'
        self.events = []      # dict, co khoa 'ev'
        self.buttons = []     # (time.time(), 'DIR'|'ESTOP', 'DOWN'|'UP')
        self.captures = {}    # (kind, id) -> dict(meta, t_us, count)
        self.status = {}
        self.error = None
        self._cap = None
        self._stop = threading.Event()
        self.th = threading.Thread(target=self._reader, daemon=True)
        self.th.start()
        self._handshake()

    # ------------------------------------------------------------ doc
    def _reader(self):
        while not self._stop.is_set():
            try:
                s = self.tr.readline()
            except DeviceError as e:
                self.error = str(e)
                return
            if not s:
                continue
            if s[0] != "#":
                p = s.split(",")
                if len(p) == 6:
                    try:
                        v = [float(x) for x in p]
                    except ValueError:
                        continue
                    with self.lock:
                        self.rows.append(v)
                continue
            self._comment(s)

    def _comment(self, s):
        if s.startswith("#C,"):
            if self._cap is not None:
                try:
                    _, t, c = s.split(",")
                    self._cap["t_us"].append(int(t))
                    self._cap["count"].append(int(c))
                except ValueError:
                    pass
            return
        with self.lock:
            self.comments.append(s)
        if self.verbose or s.startswith("#ERR") or s.startswith("# >>>") or s.startswith("#SIM"):
            print("   [ESP32] " + s[1:].strip())
        head = s.split()[0]
        if head == "#CAP":
            d = kv(s)
            if "BEGIN" in s:
                self._cap = {"meta": d, "t_us": [], "count": []}
            elif "END" in s and self._cap is not None:
                cap = self._cap
                self._cap = None
                cap["t_us"] = np.array(cap["t_us"], dtype=float)
                cap["count"] = np.array(cap["count"], dtype=float)
                with self.lock:
                    self.captures[(cap["meta"].get("kind"), cap["meta"].get("id"))] = cap
        elif head == "#RES":
            d = kv(s)
            d["kind"] = s.split()[1]
            d["_time"] = time.time()
            with self.lock:
                self.results.append(d)
        elif head == "#EV":
            d = kv(s)
            d["ev"] = s.split()[1]
            with self.lock:
                self.events.append(d)
        elif head == "#ST":
            with self.lock:
                self.status = kv(s)
        elif head == "#BTN":
            parts = s.split()
            if len(parts) >= 3:
                with self.lock:
                    self.buttons.append((time.time(), parts[1], parts[2]))

    # ------------------------------------------------------------ lenh
    def send(self, cmd):
        if self.error:
            raise DeviceError(self.error)
        self.tr.write(cmd + "\n")
        time.sleep(0.02)

    def _handshake(self):
        t_end = time.time() + 6.0
        while time.time() < t_end:
            n = len(self.comments)
            self.send("ping")
            t1 = time.time() + 0.6
            while time.time() < t1:
                with self.lock:
                    new = self.comments[n:]
                for s in new:
                    if s.startswith("#PONG"):
                        if FW_TAG not in s:
                            raise DeviceError("ESP32 đang chạy firmware khác ({}). Nạp firmware Lab 4: "
                                              "python3 lab4.py setup".format(s))
                        self.fw = s.split(None, 1)[1]
                        self.send("stream 1")
                        self.send("cap 1")
                        return
                time.sleep(0.02)
        raise DeviceError("ESP32 không trả lời 'ping'. Kiểm tra: đã nạp firmware Lab 4 chưa "
                          "(python3 lab4.py setup), đúng cổng, baud 115200, đã đóng Serial Monitor; "
                          "thử nhấn nút EN trên board.")

    def query_status(self, timeout=1.5):
        with self.lock:
            self.status = {}
        self.send("?")
        t_end = time.time() + timeout
        while time.time() < t_end:
            with self.lock:
                if self.status:
                    return dict(self.status)
            time.sleep(0.01)
        raise DeviceError("Không nhận được trạng thái (#ST).")

    def mark(self):
        """Moc de chi lay ket qua/su kien/nut moi phat sinh sau thoi diem nay."""
        with self.lock:
            return {"res": len(self.results), "ev": len(self.events), "btn": len(self.buttons),
                    "com": len(self.comments), "row": len(self.rows)}

    def wait_result(self, kind, mark, timeout, stop_check=None):
        t_end = time.time() + timeout
        while time.time() < t_end:
            self.check()
            with self.lock:
                for r in self.results[mark["res"]:]:
                    if r["kind"] == kind:
                        return r
            if stop_check is not None and stop_check():
                return None
            time.sleep(0.02)
        return None

    def wait_capture(self, kind, rid, timeout=8.0):
        t_end = time.time() + timeout
        while time.time() < t_end:
            self.check()
            with self.lock:
                cap = self.captures.get((kind, rid))
            if cap is not None:
                return cap
            time.sleep(0.02)
        return None

    def wait_button(self, name, edge, mark, timeout):
        t_end = time.time() + timeout
        while time.time() < t_end:
            self.check()
            with self.lock:
                for _, b, e in self.buttons[mark["btn"]:]:
                    if b == name and e == edge:
                        return True
            time.sleep(0.02)
        return False

    def wait_comment(self, prefix, mark, timeout):
        t_end = time.time() + timeout
        while time.time() < t_end:
            self.check()
            with self.lock:
                for s in self.comments[mark["com"]:]:
                    if s.startswith(prefix):
                        return s
            time.sleep(0.02)
        return None

    def rows_since(self, mark):
        with self.lock:
            a = np.array(self.rows[mark["row"]:], dtype=float)
        return a.reshape(-1, 6)

    def last_rows(self, k):
        with self.lock:
            a = np.array(self.rows[-k:], dtype=float)
        return a.reshape(-1, 6)

    def check(self):
        if self.error:
            raise DeviceError(self.error)

    # ------------------------------------------------------------ nut (chi ban gia lap)
    def sim_press(self, btn, hold=0.15, delay=None):
        """Ban gia lap: 'nhan' nut sau mot khoang tre ngau nhien (thay cho tay nguoi)."""
        if not self.sim:
            return

        def job():
            time.sleep(delay if delay is not None else random.uniform(0.3, 0.9))
            try:
                self.tr.write("!press {}\n".format(btn))
                time.sleep(hold)
                self.tr.write("!release {}\n".format(btn))
            except DeviceError:
                pass
        threading.Thread(target=job, daemon=True).start()

    # ------------------------------------------------------------ dong
    def close(self):
        try:
            self.tr.write("stop\n")
            time.sleep(0.05)
        except Exception:
            pass
        self._stop.set()
        self.th.join(timeout=1)
        self.tr.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def open_link(port=None, sim=False, verbose=False, sim_enc_sign=1):
    if sim:
        return Link(SimTransport(enc_sign=sim_enc_sign), verbose=verbose)
    return Link(SerialTransport(find_port(port)), verbose=verbose)


def bell():
    sys.stdout.write("\a")
    sys.stdout.flush()
