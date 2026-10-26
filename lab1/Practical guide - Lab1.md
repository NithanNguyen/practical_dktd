Dưới đây là hướng dẫn chạy project **hw1_sampling.zip** (bài thực hành về nhà Buổi 1) trên VS Code + PlatformIO, Ubuntu 20.04. Mình hiểu chữ "v" cuối tin nhắn là gõ nhầm. Nếu bạn muốn hỏi project khác (ví dụ đồ án tay máy) thì báo mình.

**Lưu ý:** firmware này mình chưa build được vì sandbox của mình không tải được gói ESP32. Lần build ở bước 5 trên máy bạn là lần biên dịch đầu tiên. Nếu lỗi, gửi mình toàn bộ log.

---

## Bước 0: Gói hệ thống cần có

PlatformIO yêu cầu gói `python3-venv` trên Ubuntu:

```bash
sudo apt update
sudo apt install python3-venv git unzip curl
```

## Bước 1: Cài VS Code (bỏ qua nếu `code --version` đã chạy được)

Cách 1, dùng snap:
```bash
sudo snap install --classic code
```

Cách 2, tải file `.deb` từ code.visualstudio.com rồi chạy:
```bash
sudo apt install ./code_*.deb
```

Trang hướng dẫn của VS Code không ghi phiên bản Ubuntu tối thiểu. Nếu bản mới nhất không chạy được trên 20.04, mình chưa có thông tin chắc chắn về chuyện này. Lúc đó hãy báo mình kèm thông báo lỗi.

## Bước 2: Cài extension PlatformIO IDE

1. Mở VS Code, bấm `Ctrl+Shift+X`, tìm **"PlatformIO IDE"** (ID: `platformio.platformio-ide`), bấm **Install**.
2. Đợi extension tự cài PlatformIO Core. Bạn không cần cài Core riêng.
3. Khi có thông báo, bấm **Reload/Restart**. Xong bước này, góc trái thanh trạng thái (đáy cửa sổ) sẽ có các nút: 🏠 Home, ✓ Build, → Upload, 🗑 Clean, 🔌 Serial Monitor, Terminal.

## Bước 3: Cấp quyền cổng USB-serial (làm một lần)

```bash
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -a -G dialout $USER
```

Sau đó **đăng xuất rồi đăng nhập lại** (hoặc khởi động lại máy), và rút ra cắm lại board. Gõ `id`: kết quả phải có chữ `dialout`.

## Bước 4: Giải nén và mở project

```bash
mkdir -p ~/esp32 && cd ~/esp32
unzip ~/Downloads/hw1_sampling.zip
```

Trong VS Code chọn **File → Open Folder → `~/esp32/hw1_sampling`**.

Cần mở **đúng thư mục chứa `platformio.ini`**. Nếu mở thư mục cha, PlatformIO sẽ không nhận ra project. Cây thư mục sau khi mở:

```
hw1_sampling/
├── platformio.ini      ← board = nodemcu-32s
├── src/main.cpp        ← firmware
├── tools/hw1_logger.py ← script Python
└── bao_cao_mau.md
```

## Bước 5: Build (`Ctrl+Alt+B` hoặc nút ✓)

- **Lần đầu build sẽ mất vài phút** vì PlatformIO tự tải platform `espressif32`, toolchain và framework Arduino.
- Build xong, terminal hiện `[SUCCESS]`.
- Trong log có dòng `framework-arduinoespressif32 @ ...`. Hãy ghi lại phiên bản này để đưa vào báo cáo.

## Bước 6: Nối dây và cắm board

| Cảm biến | Nối |
|---|---|
| Biến trở | 2 chân ngoài → 3V3 và GND, chân giữa → **GPIO34** |
| LDR | 3V3 → LDR → **GPIO34** → điện trở 10 kΩ → GND |

Cắm board qua cáp USB (phải là cáp có dây dữ liệu, không phải cáp chỉ sạc). Kiểm tra cổng bằng một trong hai lệnh:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
pio device list        # chạy trong terminal PlatformIO (nút Terminal ở thanh dưới)
```

Bình thường sẽ thấy `/dev/ttyUSB0`. Nếu máy có nhiều cổng, mở `platformio.ini` và bỏ dấu `;` ở hai dòng `upload_port` / `monitor_port`, rồi sửa đúng tên cổng.

## Bước 7: Nạp firmware (`Ctrl+Alt+U` hoặc nút →)

Nạp thành công thì cuối log có `Hard resetting via RTS pin...` và `[SUCCESS]`.

Nếu log dừng ở `Connecting........`: **giữ nút BOOT** trên board cho đến khi thấy dòng `Writing at ...` thì thả ra.

## Bước 8: Kiểm tra bằng Serial Monitor (`Ctrl+Alt+S` hoặc nút 🔌)

Nhấn nút **EN/RST** trên board để thấy hai dòng tiêu đề. Kết quả mong đợi:

```
# fs_hz=100,period_us=10000,adc_pin=34
# seq,t_isr_us,t_read_us,raw
0,1234567,1234610,2048
1,1244567,1244611,2050
...
```

Cách kiểm tra:
- Vặn biến trở hoặc che LDR: cột `raw` (0–4095) phải thay đổi.
- Cột thứ 2 (`t_isr_us`) tăng khoảng 10 000 sau mỗi dòng.

Sau khi kiểm tra xong, **đóng monitor** (bấm `Ctrl+C` trong terminal hoặc đóng tab terminal). Nếu monitor còn mở, script Python sẽ không mở được cổng.

## Bước 9: Chạy script Python để ghi CSV và vẽ đồ thị

Mở terminal mới trong VS Code (`` Ctrl+` ``):

```bash
cd ~/esp32/hw1_sampling
python3 -m venv .venv
source .venv/bin/activate
pip install pyserial numpy matplotlib

python3 tools/hw1_logger.py --port /dev/ttyUSB0 --duration 30 --out run1
```

Trong 30 giây ghi dữ liệu, hãy vặn biến trở hoặc che LDR để đồ thị có biến thiên. Kết quả nằm trong `run1/`:

| File | Dùng để |
|---|---|
| `data.csv` | dữ liệu thô |
| `signal.png` | đồ thị tín hiệu theo thời gian (để nộp) |
| `timing.png` | chu kỳ, jitter, độ trễ |
| `stats.md` | bảng số liệu, dán vào `bao_cao_mau.md` |

Lần sau chỉ cần chạy `source .venv/bin/activate` trước khi gọi script. Muốn xem lại đồ thị từ CSV đã có:
```bash
python3 tools/hw1_logger.py --analyze run1/data.csv --show
```

---

## Cách khác: chạy toàn bộ bằng dòng lệnh

Trong terminal PlatformIO, đứng ở thư mục project:
```bash
pio run                       # build
pio run -t upload             # nạp
pio device monitor            # xem serial (thoát: Ctrl+C)
```

## Xử lý lỗi thường gặp

| Triệu chứng | Nguyên nhân khả dĩ | Cách xử lý |
|---|---|---|
| Thanh dưới không có nút PlatformIO | extension chưa cài xong, hoặc thiếu `python3-venv` | làm lại bước 0, rồi khởi động lại VS Code |
| `Permission denied: '/dev/ttyUSB0'` | user chưa thuộc nhóm `dialout` | làm lại bước 3, nhớ đăng xuất rồi đăng nhập |
| Không thấy `/dev/ttyUSB*` | cáp chỉ sạc, hoặc cổng USB lỏng | đổi cáp/cổng, chạy `sudo dmesg \| tail` ngay sau khi cắm để xem kernel có nhận board không |
| `Could not open port ... busy` | Serial Monitor hoặc chương trình khác đang giữ cổng | đóng monitor |
| Script báo không thấy dòng tiêu đề | sai cổng, hoặc monitor đang mở | kiểm tra bằng `pio device list` |
| Lỗi build quanh `timerBegin` / `timerAlarm` | lệch phiên bản Arduino core | gửi mình log build đầy đủ |
| VS Code báo lỗi đỏ ở `#include` nhưng build vẫn thành công | IntelliSense chưa cập nhật | bỏ qua, hoặc chạy **PlatformIO: Rebuild IntelliSense Index** |

Nếu kẹt ở bước nào, gửi mình thông báo lỗi nguyên văn hoặc ảnh chụp terminal.

**Nguồn:**
- [PlatformIO IDE for VS Code: cài đặt, toolbar, phím tắt](https://docs.platformio.org/en/latest/integration/ide/vscode.html)
- [PlatformIO: yêu cầu python3-venv trên Linux](https://docs.platformio.org/en/latest/faq/install-python.html)
- [PlatformIO: 99-platformio-udev.rules / nhóm dialout](https://docs.platformio.org/en/latest/core/installation/udev-rules.html)
- [PlatformIO: pio device list](https://docs.platformio.org/en/latest/core/userguide/device/cmd_list.html)
- [PlatformIO: board NodeMCU-32S](https://docs.platformio.org/en/latest/boards/espressif32/nodemcu-32s.html)
- [VS Code on Linux: cài đặt](https://code.visualstudio.com/docs/setup/linux)
