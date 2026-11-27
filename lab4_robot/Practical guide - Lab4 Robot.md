# Lab 4 – Thực hành về nhà, nhánh Robot: đảo chiều và dừng khẩn cấp động cơ DC

Theo `Lab 4.pdf`, đề bài yêu cầu *“lập trình đảo chiều + dừng khẩn cấp động cơ theo tín hiệu nút nhấn, đo thời gian đáp ứng dừng khẩn”*. Sản phẩm nộp gồm **code** và **video demo**. Bài về nhà chiếm 4 điểm, báo cáo hoặc video chiếm 2 điểm, và phần giải thích an toàn điện chiếm 1 điểm.

Thư mục này đã làm sẵn mọi phần có thể tự động hoá. Phần bạn tự làm chỉ còn 3 việc: **cắm thêm 2 nút nhấn**, **bấm nút khi script nhắc** và **quay video**.

```
lab4_robot/
├── lab4.sh                    chạy toàn bộ bằng 1 lệnh
├── firmware/                  PlatformIO, board esp32dev (ESP32-DevKitC-32D)
│   └── src/main.cpp
└── tools/
    ├── lab4.py                setup / check / estop / reverse / demo / report / all
    ├── lab4_link.py           giao tiếp Serial (hoặc bản giả lập)
    ├── lab4_analysis.py       phân tích dữ liệu 1 kHz và vẽ đồ thị
    ├── host_sim/              biên dịch CHÍNH firmware này cho PC để chạy thử khi chưa có phần cứng
    ├── logs/                  dữ liệu đo (tạo khi chạy)
    └── report/                BAO_CAO_LAB4.md và hình (tạo khi chạy)
```

---

## 0. Làm nhanh (khi đã quen)

```bash
cd ~/practical_dktd && git pull
bash lab4_robot/lab4.sh            # khoảng 8–10 phút, bấm nút khi được nhắc
bash lab4_robot/lab4.sh demo       # mở màn hình demo rồi quay video theo mục 8
```

---

## 1. Linh kiện

Bài này **giữ nguyên mạch Lab 3** và chỉ cắm thêm 2 nút nhấn.

| Linh kiện | SL | Ghi chú |
|---|---|---|
| ESP32-DevKitC-32D | 1 | PlatformIO dùng `board = esp32dev`, giống repo `eezybot_mk2_robot` |
| Driver TB6612FNG | 1 | Đã đấu ở Lab 3 |
| Động cơ JGA25-370 có encoder | 1 | Đã đấu ở Lab 3. CPR = 1978,7 đếm/vòng trục ra, hiệu chuẩn ở Lab 3 |
| Adapter 12 V | 1 | Nguồn động cơ, đã dùng ở Lab 3 |
| **Nút nhấn** | **2** | Một nút ĐẢO CHIỀU và một nút DỪNG KHẨN, nên dùng 2 nút khác màu, ví dụ nút dừng khẩn màu đỏ |
| Breadboard, dây jumper, cáp micro-USB có dây dữ liệu | – | |

Lab 3 đo được tốc độ trục ra ở 12 V khoảng **52 rpm ở duty 40 %**, **94 rpm ở 70 %** và **136 rpm ở 100 %** (`lab3_pid/tools/lab3_config.json`, mục `static`). Đây là các giá trị bạn nên thấy lại khi đo Lab 4.

---

## 2. Đấu dây

Phần Lab 3 giữ nguyên, và bạn chỉ thêm 2 dòng cuối của bảng.

| Từ | Đến | Ghi chú |
|---|---|---|
| ESP32 IO25 | TB6612 PWMA | |
| ESP32 IO26 | TB6612 AIN1 | |
| ESP32 IO27 | TB6612 AIN2 | |
| ESP32 3V3 | TB6612 VCC **và** STBY | STBY nối 3V3 nên driver luôn bật |
| ESP32 GND | TB6612 GND | **GND chung** với adapter |
| Adapter 12 V (+) / (−) | TB6612 VM / GND | **Không** nối 12 V vào chân 5V/3V3 của ESP32 |
| TB6612 AO1 / AO2 | Động cơ dây đỏ (M+) / dây trắng (M−) | |
| Encoder xanh dương / đen | ESP32 3V3 / GND | |
| Encoder vàng (A) / xanh lá (B) | ESP32 IO32 / IO33 | |
| **Nút ĐẢO CHIỀU** | **IO18 ↔ GND** | Không cần điện trở vì firmware bật kéo lên nội |
| **Nút DỪNG KHẨN** | **IO19 ↔ GND** | Không cần điện trở. Ngắt cạnh xuống |

```
            ESP32-DevKitC-32D                     TB6612FNG              JGA25-370
          ┌───────────────────┐                ┌─────────────┐
          │ IO25 ─────────────┼───────────────►│ PWMA    AO1 ├────── đỏ  (M+)
          │ IO26 ─────────────┼───────────────►│ AIN1    AO2 ├────── trắng (M−)
          │ IO27 ─────────────┼───────────────►│ AIN2        │
          │ 3V3  ─────────────┼──────────┬────►│ VCC     VM  ├◄───── +12 V adapter
          │                   │          └────►│ STBY        │
          │ GND  ─────────────┼───────┬───────►│ GND     GND ├◄───── −  adapter
          │ IO32 ◄────────────┼─ vàng (A)      └─────────────┘
          │ IO33 ◄────────────┼─ xanh lá (B)    encoder: xanh dương = 3V3, đen = GND
          │ IO18 ──[ĐẢO CHIỀU]┼──┤
          │ IO19 ──[DỪNG KHẨN]┼──┴── GND
          └───────────────────┘
```

**Vì sao chọn IO18 và IO19?** Theo tài liệu DevKitC-32D trong Project, hai chân này là GPIO vào/ra thường. Chúng không thuộc nhóm chân strapping (GPIO0, 2, 5, 12, 15), nên mức logic của chúng lúc khởi động không ảnh hưởng tới chế độ boot. Chúng cũng không thuộc nhóm chân chỉ đọc vào (GPIO34, 35, 36, 39).

**Lưu ý với nút 4 chân:** hai chân cùng một cạnh thường đã nối thông với nhau sẵn. Nếu cắm nhầm cặp chân đó thì nút trông như luôn bị nhấn. Bạn không cần đoán, vì bước `check` (mục 5.2) sẽ phát hiện lỗi này. Khi nút DỪNG KHẨN bị đấu như vậy, firmware báo *“Nut DUNG KHAN dang nhan luc khoi dong”* ngay khi bật.

**Quy tắc nguồn của DevKitC:** Espressif ghi rõ board chỉ được cấp nguồn từ **một và chỉ một** trong ba ngõ (USB, chân 5V, chân 3V3). Vì vậy trong bài này ESP32 lấy nguồn từ USB, còn adapter 12 V chỉ nối vào VM của TB6612 và nối chung GND.

---

## 3. Cài đặt một lần

```bash
cd ~/practical_dktd && git pull                  # lấy thư mục lab4_robot
groups | grep -q dialout || sudo usermod -a -G dialout $USER   # sau lệnh này phải ĐĂNG XUẤT rồi đăng nhập lại
```

- `lab4.sh` tự tạo `tools/.venv` và tự cài `pyserial`, `numpy`, `matplotlib`. Nếu báo thiếu venv thì chạy `sudo apt install python3-venv`.
- PlatformIO trong VS Code nằm ở `~/.platformio/penv/bin/pio`, và script tự tìm đường dẫn này để nạp firmware. Gói `espressif32` đã có sẵn từ các lab trước nên lần build đầu sẽ nhanh.
- **Đóng Serial Monitor của PlatformIO** trước khi chạy script, vì mỗi lúc chỉ một chương trình được mở cổng.

---

## 4. (Khuyến nghị) Chạy thử khi chưa đấu dây

```bash
bash lab4_robot/lab4.sh --sim all --reps 1 --coast-reps 1
```

Lệnh này biên dịch **chính file `firmware/src/main.cpp`** để chạy trên PC. Động cơ, encoder và nút nhấn được mô phỏng, và mô hình động cơ lấy K và T từ Lab 3. Nút được “bấm” tự động. Dữ liệu được lưu riêng vào `logs_sim/` và `report_sim/`, đồng thời bị gắn nhãn **GIẢ LẬP**, nên **không dùng để nộp**. Mục đích của bước này là để bạn thấy trước toàn bộ quy trình và các câu nhắc. Lệnh cần `g++`, tức gói `build-essential`.

---

## 5. Chạy thật: `bash lab4_robot/lab4.sh`

Cắm USB, bật adapter 12 V rồi chạy lệnh trên. Script lần lượt chạy 5 bước. Nếu bị gián đoạn, bạn có thể chạy riêng từng bước bằng `bash lab4_robot/lab4.sh <tên bước>`.

### 5.1. `setup`, khoảng 1 phút, bạn không phải làm gì

1. Script kiểm tra Python, các thư viện và nhóm `dialout`.
2. Script lấy **CPR = 1978,7** và **sign** từ `lab3_pid/tools/lab3_config.json`, nên bạn không phải hiệu chuẩn lại encoder.
3. Script nạp firmware bằng `pio run -d firmware -t upload`. Nếu nạp báo `Failed to connect` thì giữ nút **BOOT** trên board khi thấy dòng `Connecting...`.
4. Script kết nối, gửi `ping`, nhận `lab4_robot v1.0` rồi lưu cổng vào `tools/lab4_config.json`.

### 5.2. `check`, khoảng 1 phút, cần bấm nút 3 lần

| Script nhắc | Bạn làm | Nếu đạt thì in |
|---|---|---|
| Nhấn rồi NHẢ nút DỪNG KHẨN | bấm nút đỏ | `[OK] Nút DỪNG KHẨN (IO19)` |
| Nhấn nhanh rồi NHẢ nút ĐẢO CHIỀU | bấm nút kia | `[OK] Nút ĐẢO CHIỀU (IO18)` |
| GIỮ nút ĐẢO CHIỀU khoảng 1,5 s | giữ rồi nhả | `[OK] Reset bằng giữ nút…` |
| *(tự động)* | – | encoder đếm đúng chiều. Nếu ngược chiều thì script **tự đổi `sign`** và lưu lại |
| *(tự động)* | – | đảo chiều và dừng khẩn bằng lệnh mềm |

Kết quả phải là `KẾT LUẬN: PHẦN CỨNG SẴN SÀNG`. Nếu còn lỗi thì tra mục 10.

### 5.3. `estop`, khoảng 4 phút, cần bấm nút DỪNG KHẨN 12 lần

Script cho động cơ quay đến khi ổn định, kêu “bíp” rồi nhắc:

```
  >>> [4/12] Động cơ quay 70 % (≈ 94 rpm), chế độ phanh ngắn. NHẤN nút DỪNG KHẨN.
   [ESP32] >>> DUNG KHAN #4 (brake): truc dung sau …ms, tu … rpm, quay them … do
      -> t_dừng = … ms (firmware … ms) | t_10% = … ms | θ = …° | τ = … ms
```

Bạn **bấm nút lúc nào cũng được**, không cần nhanh. Sau mỗi lần, hãy **nhả nút**. Script tự xoá chốt bằng lệnh `reset`, nhưng firmware chỉ chấp nhận lệnh này khi nút đã nhả. Kế hoạch mặc định gồm:

- **Phanh ngắn** ở duty 40, 70 và 100 %, mỗi mức 3 lần. Mục đích là xem t_dừng phụ thuộc tốc độ ban đầu như thế nào.
- **Thả tự do (coast)** ở duty 100 %, 3 lần. Đây là nhóm đối chứng để chứng minh vì sao dừng khẩn phải dùng phanh ngắn.

Bạn có thể đổi kế hoạch, ví dụ `bash lab4_robot/lab4.sh estop --duties 30 60 90 --reps 5 --coast-reps 2`.

### 5.4. `reverse`, khoảng 1 phút, cần bấm nút ĐẢO CHIỀU 5 lần

Động cơ quay ở 70 %. Mỗi khi script nhắc, bạn bấm nút ĐẢO CHIỀU. Script đo **t_zero** (thời gian trục về 0) và **t_90** (thời gian đạt 90 % tốc độ theo chiều mới).

### 5.5. `report`, vài giây

Script tạo `tools/report/BAO_CAO_LAB4.md` cùng 3 hình trong `tools/report/figures/`:

- `estop_curves.png`: đường tốc độ của mọi lần dừng khẩn, gồm phanh ngắn và thả tự do.
- `estop_summary.png`: t_dừng và góc quay thêm theo tốc độ ban đầu.
- `reverse_curves.png`: các lần đảo chiều.

Báo cáo **tính lại mọi số từ file CSV gốc**, nên bạn có thể chạy lại `report` bất kỳ lúc nào.

---

## 6. Firmware hoạt động thế nào (để giải thích khi bảo vệ)

### 6.1. Ba trạng thái dừng của TB6612

Bảng chân lý trong datasheet Toshiba (mục *H-SW Control Function*) cho thấy driver có các trạng thái sau:

| IN1 | IN2 | PWM | Chế độ |
|---|---|---|---|
| H | L | H | CW (quay thuận) |
| L | H | H | CCW (quay ngược) |
| H | H | H/L | **Short brake**: cả hai đầu ra nối xuống GND |
| L | L | H | **Stop**: đầu ra trở kháng cao, động cơ quay tự do |

Như vậy chỉ cần ghi hai chân IN1 và IN2 là đổi được chế độ, và việc này không phụ thuộc PWM. Firmware tận dụng điểm này để dừng khẩn **ngay trong hàm ngắt**. Hàm ngắt đặt IN1 = IN2 = H bằng đúng một lần ghi vào thanh ghi `GPIO_OUT_W1TS`. Firmware không gọi `ledcWrite` trong ngắt vì hàm đó không an toàn khi gọi từ ngắt.

### 6.2. Vì sao phanh ngắn dừng nhanh hơn thả tự do

Mô hình động cơ DC chuẩn gồm hai phương trình (MECH 3140, Auburn University):

- Điện: L·di/dt + R·i = V − K·ω
- Cơ: J·dω/dt + b·ω = K·i

Khi **phanh ngắn**, hai đầu cuộn dây bị nối tắt nên V = 0. Lúc đó sức phản điện động K·ω tự đẩy một dòng i ≈ −K·ω/R qua cuộn dây. Dòng này sinh mô-men −K·i ngược chiều quay, và động năng tiêu tán thành nhiệt trên R. Khi **thả tự do**, mạch hở nên i = 0, và chỉ còn ma sát (b·ω cùng ma sát hộp số) hãm trục. Kết luận: phanh ngắn có thêm một mô-men hãm tỉ lệ với tốc độ, nên dừng nhanh hơn nhiều. Số đo thật ở mục 5.3 sẽ định lượng chênh lệch này.

### 6.3. Vì sao đảo chiều phải phanh trước

Nếu đổi IN1/IN2 ngay khi trục đang quay ở tốc độ ω thì điện áp nguồn V và sức phản điện động K·ω cộng dồn. Theo phương trình điện ở trên, dòng lúc đó xấp xỉ (V + K·ω)/R. Khi trục đang quay gần tốc độ không tải thì K·ω gần bằng V, nên dòng này gần gấp đôi dòng khởi động V/R. Vì vậy firmware làm theo trình tự **phanh ngắn → chờ trục đứng yên 30 ms → đổi chiều → tăng duty dần (400 %/s)**. Trình tự này cũng giải thích vì sao Lab 3 không cho đảo chiều: mục đích là tránh dòng ngược về adapter.

### 6.4. Chốt và reset

- Sau khi dừng khẩn, firmware **chốt** trạng thái. Vòng lặp chính không thể ghi đè đầu ra, vì mọi lần ghi IN1/IN2 đều kiểm tra cờ `latched` bên trong vùng găng (critical section).
- Muốn xoá chốt, bạn phải **nhả** nút DỪNG KHẨN (ổn định ít nhất 50 ms) rồi **giữ** nút ĐẢO CHIỀU 1 s.
- Xoá chốt **không làm động cơ tự chạy lại**, bạn phải nhấn ĐẢO CHIỀU thêm một lần nữa. Cách làm này đúng với nguyên tắc mà trang hỏi đáp của Pilz nêu: *việc reset dừng khẩn không được tự khởi động lại máy, và muốn khởi động lại phải tác động có chủ ý vào bộ điều khiển dành cho việc đó*. Pilz dẫn nguyên tắc này từ Chỉ thị Máy 2006/42/EC, Phụ lục I, mục 1.2.3.
- Nếu bật nguồn khi nút DỪNG KHẨN đang nhấn thì firmware chốt luôn từ đầu.

### 6.5. Đo thời gian dừng khẩn

- **t = 0** là lúc vào hàm ngắt của nút DỪNG KHẨN, đo bằng `esp_timer` với độ phân giải 1 µs. Nút DỪNG KHẨN **không có** trễ chống dội phím, vì cạnh đầu tiên đã kích phanh và các cạnh dội sau đó bị bỏ qua nhờ cờ chốt. Ngược lại, nút ĐẢO CHIỀU có chống dội 20 ms, nên t_zero và t_90 của phép đảo chiều đã bao gồm 20 ms này.
- **t_dừng** là lần cuối trục dịch chuyển ≥ 2 đếm so với mốc trước, tính ngay trong ngắt encoder. Ngưỡng 2 đếm tương ứng khoảng 0,36° trục ra, và nó giúp bỏ qua rung ±1 đếm khi trục đứng yên. Trục được coi là đã dừng khi không dịch chuyển trong 150 ms.
- Firmware còn ghi (thời gian, số đếm) **mỗi 1 ms** vào bộ đệm vòng. Bộ đệm giữ 100 ms trước sự kiện và tối đa 1,9 s sau sự kiện, rồi gửi lên PC. PC tính lại t_dừng **bằng đúng thuật toán của firmware**, nên đây là phép kiểm tra chéo. Khi chạy giả lập, hai giá trị lệch nhau 0–0,9 ms, đúng với bước lấy mẫu 1 ms. Từ dữ liệu này PC tính thêm t_10 %, góc quay thêm θ và hằng số thời gian hãm τ.
- **Giới hạn:** bài không đo được khoảng trễ từ lúc tiếp điểm nút đóng tới lúc vào ngắt, vì muốn đo phải có máy hiện sóng. Báo cáo ghi rõ điểm này.

---

## 7. Lệnh Serial (khi muốn thử tay bằng `pio device monitor`)

`run [duty] [dir]` · `duty <%>` · `dir <1|-1>` · `rev` · `estop` · `reset` · `stop` · `mode <brake|coast>` · `ramp <%/s>` · `cpr <v>` · `sign <1|-1>` · `stream <0|1>` · `cap <0|1>` · `ping` · `?` · `help`

Firmware gửi một dòng CSV `t_ms,state,dir,duty,rpm,count` mỗi 10 ms, trong đó state 0 = dừng, 1 = chạy, 2 = đang đảo chiều và 3 = dừng khẩn. Các dòng bắt đầu bằng `#` là thông báo. Muốn đọc dễ hơn thì gõ `stream 0`.

---

## 8. Quay video demo, khoảng 60–90 s

Chạy `bash lab4_robot/lab4.sh demo`. Cửa sổ demo hiển thị trạng thái bằng chữ lớn, đồ thị tốc độ 10 s gần nhất và kết quả dừng khẩn hoặc đảo chiều mới nhất. Bạn dựng điện thoại sao cho khung hình thấy cả **động cơ (nên dán một mẩu băng keo lên trục ra để dễ thấy quay), 2 nút nhấn và màn hình laptop**. Từ lúc này bạn chỉ dùng nút.

| # | Thao tác | Video phải thấy |
|---|---|---|
| 1 | Giới thiệu mạch trong 5–10 s | ESP32, TB6612, động cơ, 2 nút, adapter 12 V |
| 2 | Nhấn **ĐẢO CHIỀU** | Động cơ chạy, màn hình hiện `ĐANG CHẠY` |
| 3 | Nhấn **ĐẢO CHIỀU** | Trục phanh rồi quay ngược, đồ thị đổi dấu, màn hình hiện `Đảo chiều #1: về 0 sau … ms` |
| 4 | Nhấn **ĐẢO CHIỀU** lần nữa | Trục quay lại chiều ban đầu |
| 5 | Nhấn **DỪNG KHẨN** | Trục dừng gần như tức thì, màn hình hiện `Dừng khẩn #1: trục dừng sau … ms` |
| 6 | Nhấn **ĐẢO CHIỀU** khi vẫn đang chốt | **Động cơ không chạy**, chứng minh cơ chế chốt |
| 7 | Nhả DỪNG KHẨN rồi **giữ ĐẢO CHIỀU 1 s** | Màn hình chuyển sang `DỪNG` và động cơ **vẫn đứng yên** |
| 8 | Nhấn **ĐẢO CHIỀU** rồi **DỪNG KHẨN** | Chạy lại rồi dừng khẩn lần 2 |
| 9 | Mở `tools/report/figures/estop_summary.png` trong 5–10 s | Kết quả đo: t_dừng theo tốc độ, phanh ngắn so với thả tự do |

Mẹo: nếu điện thoại có chế độ quay chậm thì quay thêm một đoạn ngắn cảnh 5 để thấy rõ trục dừng trong vài chục ms.

---

## 9. Nộp bài

1. **Code**: cả thư mục `lab4_robot/`, gồm `firmware/`, `tools/` và dữ liệu đo `tools/logs/`, `tools/report/`. Lệnh đẩy lên repo:
   ```bash
   cd ~/practical_dktd && git add lab4_robot && git commit -m "Lab 4: nhanh Robot - dao chieu + dung khan" && git push
   ```
2. **Video**: theo kịch bản ở mục 8.
3. **Báo cáo** (nếu nộp kèm, chiếm 2 điểm cùng với video): chép `tools/report/BAO_CAO_LAB4.md` và các hình sang Google Docs như các lab trước, rồi thêm phần an toàn điện ở mục 11.

---

## 10. Xử lý sự cố

| Hiện tượng | Nguyên nhân thường gặp | Cách xử lý |
|---|---|---|
| `Không tìm thấy cổng ESP32` | Cáp chỉ có dây sạc | Đổi cáp, rồi chạy `ls /dev/ttyUSB*` |
| `Permission denied` | User chưa thuộc nhóm `dialout` | Chạy `sudo usermod -a -G dialout $USER` rồi đăng xuất |
| `ESP32 không trả lời 'ping'` | Serial Monitor đang mở, hoặc chưa nạp firmware | Đóng monitor rồi chạy `bash lab4_robot/lab4.sh setup` |
| Board luôn vào chế độ nạp | Lỗi tụ C15 trên DevKitC V4 đời đầu (tài liệu DevKitC trong Project) | Xem ghi chú C15 trong `mk2_connection_diagram.html` |
| `check`: nút LỖI, hoặc báo nút DỪNG KHẨN đang nhấn lúc khởi động | Cắm nhầm cặp chân luôn thông của nút 4 chân, hoặc sai chân | Xoay nút 90°, hoặc dùng 2 chân chéo nhau |
| `check`: `Động cơ không quay hoặc encoder không đếm` | Adapter tắt, hoặc dây AO1/AO2 hay encoder lỏng | Kiểm tra theo bảng ở mục 2 |
| `enc_err` > 0 | Nhiễu trên dây encoder | Đi dây encoder xa dây động cơ và dùng dây ngắn |
| Dừng khẩn tự kích khi không ai bấm | Nhiễu trên dây IO19 dài | Rút ngắn dây và đi xa dây động cơ. Có thể thêm tụ 100 nF giữa IO19 và GND (đề xuất, chưa thử) |
| Nạp báo `Failed to connect` | Mạch tự reset không vào được bootloader | Giữ nút BOOT khi thấy `Connecting...` |

---

## 11. An toàn điện: vì sao không tự đấu AC 220 V (1 điểm)

Lab 4 quy định rằng bài trên lớp chỉ dùng **12 V** để minh hoạ. Khi làm đồ án với tải AC 220 V thì **bắt buộc** dùng Dimmer AC (Triac) hoặc SSR **có cách ly quang**, có vỏ bọc và đấu nối an toàn, và **phải có giảng viên hoặc kỹ thuật viên giám sát** khi đấu điện lưới. Lý do nằm ở ba điểm sau.

1. **Dòng điện rất nhỏ qua người đã nguy hiểm.** Theo tài liệu *Electrical Safety: Safety & Health for Electrical Trades – Student Manual (2002)*, dòng 50–150 mA qua người gây ngừng thở và có thể tử vong. Dòng 1–4,3 A gây rung thất và khả năng tử vong cao. Trong khi đó 15 A là *dòng quá tải thấp nhất mà cầu chì hoặc aptomat thông thường ngắt*. Từ đó suy ra cầu chì hay aptomat trong nhà **không bảo vệ con người** khỏi điện giật, vì chúng chỉ ngắt ở dòng lớn hơn ngưỡng nguy hiểm hàng trăm lần.
2. **Mạch điều khiển 3,3 V/12 V không được thiết kế để cách ly điện lưới.** ESP32 đang nối USB với laptop. Nếu một tiếp điểm 220 V chạm sang mạch điều khiển thì điện lưới có thể lan tới board, laptop và người đang cầm. Đây là suy luận từ cách đấu nối, và nó cũng là lý do Lab 4 yêu cầu linh kiện **cách ly quang** (Triac hoặc SSR) đặt giữa ESP32 và tải AC.
3. **Ở 12 V, lỗi đấu nối chỉ làm hỏng linh kiện. Ở 220 V, lỗi đấu nối có thể gây chết người và cháy nổ.** Vì vậy bài học nguyên lý điều khiển công suất, gồm PWM, cầu H và phanh, được làm ở điện áp thấp. Phần điện lưới chỉ làm khi có người có chuyên môn giám sát.

---

## Tài liệu tham khảo

- `Lab 4.pdf` (Project): đề bài, BOM, tiêu chí chấm, cảnh báo an toàn.
- Toshiba, *TB6612FNG Driver IC for Dual DC motor*, datasheet: https://toshiba.semicon-storage.com/info/docget.jsp?did=10660. Dùng cho bảng chân lý H-SW, dải VM 2,5–13,5 V, VCC 2,7–5,5 V và dòng ra 1,2 A trung bình mỗi kênh.
- `esp32-devkitc-32d-docs.zip` (Project), trích từ tài liệu Espressif: chân J2/J3, chân strapping, quy tắc một nguồn cấp.
- `lab3_pid/tools/lab3_config.json`: CPR = 1978,7 ± 10,4 đếm/vòng, đặc tính tĩnh duty–rpm, mô hình FOPDT (K = 1,38 rpm/%, T ≈ 20 ms), dùng cho bản giả lập.
- Auburn University, MECH 3140, *Modeling Electrical-Mechanical Systems (DC Motors)*: https://eng.auburn.edu/~dmbevly/mech3140/Modeling_DC_Motors.pdf. Dùng cho phương trình điện và phương trình cơ của động cơ DC.
- Pilz FAQ, *When the emergency stop is operated on a machine, does it have to be acknowledged on each individual machine?*: https://www.pilz.com/en-INT/support/faq/standards/articles/180045. Dùng cho nguyên tắc reset dừng khẩn không được tự khởi động lại máy.
- *Electrical Safety: Safety & Health for Electrical Trades – Student Manual* (2002), bản trên eLCOSH: https://www.elcosh.org/document/1624/888/d000543/section2.html. Dùng cho bảng tác động của dòng điện lên cơ thể.
