# Lab 2 – Thực hành về nhà: BNO055 + ESP32, Moving Average & Complementary Filter

Oct 26, 2026 · @Nithan Nguyen

## Tổng quan

Bộ code `lab2/` gửi kèm làm đủ ba yêu cầu của phần "THỰC HÀNH VỀ NHÀ" trong `lab_2.docx`:

1. So sánh định lượng góc nghiêng raw, sau Moving Average (MA) và sau Complementary Filter (CF) trên cùng một tập dữ liệu, có đồ thị chồng lên nhau.
2. Thử ở hai điều kiện: đứng yên và rung nhẹ.
3. Nộp CSV raw + đã lọc, đồ thị và nhận xét.

Cách tiếp cận gồm ba ý chính:

- **Dữ liệu thô từ BNO055.** Module chạy ở chế độ non-fusion **ACCGYRO**, chỉ xuất gia tốc và vận tốc góc thô. Như vậy mọi bộ lọc đều do bạn tự cài, không lẫn với thuật toán fusion của Bosch.
- **ESP32 lọc ngay trên từng mẫu.** ESP32 lấy mẫu 100 Hz bằng timer phần cứng, theo đúng kiến trúc ISR → task → `loop()` của bài 1. Trên mỗi mẫu, nó tính góc raw, MA, CF và tích phân gyro thuần (để đối chứng), rồi gửi một dòng CSV.
- **Python vừa ghi vừa kiểm chứng.** Script Python ghi CSV, rồi **tính lại cả ba bộ lọc offline** và so với kết quả ESP32. Đây là bằng chứng bạn cài đặt đúng, tương ứng các mục 2 điểm và 3 điểm trong tiêu chí chấm. Sau đó script tính chỉ số và vẽ đồ thị.

&#91;embedded content: luồng dữ liệu · ESP32 lọc, Python kiểm chứng\]

Hai ô tô màu là hai nơi cùng cài một công thức lọc. `analyze.py` so hai kết quả này với nhau, nên đây là bước kiểm chứng.

| File | Vai trò |
| --- | --- |
| `platformio.ini` | 2 môi trường: `i2c_scan` (kiểm tra dây) và `imu_logger` (firmware chính, mặc định) |
| `src/i2c_scan/main.cpp` | Quét bus I2C, tìm địa chỉ BNO055 |
| `src/imu_logger/main.cpp` | Đọc BNO055 100 Hz, ước lượng bias gyro, lọc, xuất CSV qua Serial |
| `src/imu_logger/filters.h` | MA, CF, tích phân gyro: C++ thuần, dùng chung cho firmware và kiểm thử |
| `scripts/log_serial.py` | Reset ESP32, đọc dòng meta, ghi `data/<nhãn>.csv` + `.meta.json` |
| `scripts/analyze.py` | Kiểm chứng ESP32 với Python, kiểm tra dấu gyro, tính chỉ số, vẽ đồ thị vào `results/` |
| `scripts/requirements.txt` | pyserial, numpy, matplotlib |

**Phạm vi đã kiểm chứng và chưa kiểm chứng (bạn cần biết trước khi chạy):**

| Hạng mục | Trạng thái | Cách kiểm |
| --- | --- | --- |
| Cú pháp C++ firmware | Đã kiểm | g++ với header giả lập API Arduino/FreeRTOS; kiểm cả chuỗi định dạng `printf` |
| API `Wire` dùng trong code | Đã kiểm | Đối chiếu `Wire.h` của arduino-esp32 2.0.17 và 3.1.1 |
| Địa chỉ thanh ghi, hệ số đơn vị | Đã kiểm | Mã nguồn Adafruit\_BNO055 v1.6.4 |
| Chuỗi Python (ghi → phân tích) | Đã kiểm | Bộ giả lập dùng chính `filters.h` và đúng định dạng firmware, đi qua cổng serial ảo. Sai khác ESP32 với Python ≤ 1,2·10⁻⁴° |
| Biên dịch bằng PlatformIO | **Chưa** | Môi trường của mình bị chặn kết nối tới registry PlatformIO |
| Chạy trên phần cứng thật | **Chưa** | Các bước 3–5 dưới đây chính là bước kiểm tra phần cứng |
| Sơ đồ chân, điện trở kéo lên của GY-BNO055 | **Chưa có tài liệu gốc** | Có nhiều biến thể clone; bước 3 xác định bằng quét I2C |

Nếu bước biên dịch đầu tiên báo lỗi, hãy gửi lại toàn bộ log để mình sửa.

## Bước 0 – Linh kiện và cơ sở lý thuyết

Phần về nhà chỉ cần ESP32, GY-BNO055, cáp USB và 4 dây. DS18B20 và LDR thuộc phần thực hành trên lớp.

| Linh kiện | Số lượng | Ghi chú |
| --- | --- | --- |
| ESP32 NodeMCU-32S | 1 | Đã dùng ở bài 1 |
| Module GY-BNO055 | 1 | Dùng thay MPU6050, nên xác nhận với giảng viên |
| Cáp micro-USB có truyền dữ liệu | 1 | Cáp chỉ sạc sẽ không hiện `/dev/ttyUSB0` |
| Dây jumper cái–cái (hoặc breadboard + dây đực–đực) | 4 | VIN, GND, SDA, SCL |
| Băng keo 2 mặt + một tấm phẳng (bìa cứng, nắp hộp) | 1 | Cố định module để hai lần đo cùng một tư thế |
| Đồng hồ vạn năng | 1 (tùy chọn) | Chỉ cần khi gỡ lỗi I2C |

**Góc nghiêng từ gia tốc kế.** Khi đứng yên, gia tốc kế chỉ đo trọng trường, nên tính được góc bằng `atan2`. Firmware dùng quy ước tay phải: nằm ngang, mặt chip hướng lên thì az ≈ +9,8 m/s².

```latex
\phi_{acc} = \operatorname{atan2}(a_y, a_z), \qquad \theta_{acc} = \operatorname{atan2}\left(-a_x, \sqrt{a_y^2 + a_z^2}\right)
```

Góc này đúng về trung bình nhưng nhiễu. Khi rung, gia tốc chuyển động cộng thêm vào trọng trường, nên góc tính ra sai lớn.

**Tích phân gyro** (θ += ω·dt) cho tín hiệu mượt. Nhưng sai lệch bias của gyro bị tích lũy theo thời gian, làm góc trôi. Đây là lý do cần kết hợp hai cảm biến.

**Moving Average** lấy trung bình N mẫu gần nhất:

```latex
y[n] = \frac{1}{N}\sum_{k=0}^{N-1} x[n-k]
```

- Với nhiễu trắng, MA giảm biên độ nhiễu √N lần (Smith, chương 15). N = 10 cho mức giảm khoảng 3,16 lần, tức khoảng 10 dB.
- Đổi lại, "tâm" của cửa sổ nằm (N−1)/2 mẫu trước thời điểm hiện tại. Ở 100 Hz, N = 10 tương ứng trễ 4,5 mẫu = 45 ms.
- MA vẫn chỉ dùng gia tốc kế, nên rung chậm hơn cửa sổ trung bình vẫn lọt qua.

**Complementary Filter** (Colton, 2007) cho gyro quyết định biến thiên nhanh, còn gia tốc kế kéo góc về đúng giá trị trung bình:

```latex
\theta[n] = \alpha\left(\theta[n-1] + \omega[n]\,\Delta t\right) + (1-\alpha)\,\theta_{acc}[n], \qquad \tau = \frac{\alpha\,\Delta t}{1-\alpha}
```

- Với α = 0,98 và Δt = 0,01 s, hằng số thời gian τ = 0,49 s. Ví dụ trong tài liệu của Colton dùng đúng hai giá trị này.
- Colton cũng chỉ ra rằng bias gyro còn sót lại gây lệch góc cố định xấp xỉ bias × τ. Vì vậy firmware **trừ bias gyro**, ước lượng trong 2 s đầu lúc cảm biến đứng yên.
- CF cần khoảng 4τ ≈ 2 s để hội tụ sau khi khởi động. `analyze.py` bỏ 2 s đầu khi tính chỉ số vì lý do này.

Cả hai tham số có thể đổi trong `main.cpp`: `MA_N` và `CF_ALPHA`. N = 10 và α = 0,98 là lựa chọn của mình, bám theo hai nguồn trên, không phải yêu cầu của đề bài.

## Bước 1 – Cài đặt môi trường (Ubuntu 20.04)

Nếu bài 1 đã chạy được trên máy này, bạn chỉ cần làm bước 3 và 4.

1. **PlatformIO:** mở VS Code, kiểm tra extension PlatformIO IDE đã cài, vì bài 1 dùng nó.
2. **Quyền cổng serial** (theo tài liệu udev rules của PlatformIO; bỏ qua nếu bài 1 đã nạp và đọc Serial được):

   ```bash
   curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger
   sudo usermod -a -G dialout $USER
   sudo usermod -a -G plugdev $USER
   ```

   Sau đó đăng xuất và đăng nhập lại. Chạy `id` và kiểm tra có chữ `dialout`.
3. **Đặt code vào repo:** giải nén `lab2.zip` thành thư mục `practical_dktd/lab2/`, ngang hàng với `lab1/`. Trong VS Code chọn PlatformIO → *Pick a folder* → `lab2`, vì mỗi lab là một project PlatformIO riêng như bài 1.
4. **Môi trường Python** (chạy trong `lab2/`):

   ```bash
   sudo apt install python3-venv   # chỉ cần nếu lệnh dưới báo thiếu venv
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -r scripts/requirements.txt
   ```

   Script chỉ cần Python ≥ 3.7, nên Python 3.8 mặc định của Ubuntu 20.04 dùng được. `.gitignore` đã loại `.venv` và `.pio`.
5. **Kiểm tra cổng:** cắm ESP32, chạy `ls /dev/ttyUSB*`, phải thấy `/dev/ttyUSB0`. Nếu là số khác, sửa `upload_port`/`monitor_port` trong `platformio.ini` và thêm `--port` khi chạy script Python.

## Bước 2 – Nối dây GY-BNO055 với NodeMCU-32S

Chỉ cần 4 dây I2C (Mischianti, phần 2). **Rút USB trước khi nối.**

| Chân GY-BNO055 | Nối tới NodeMCU-32S | Ghi chú |
| --- | --- | --- |
| VIN | 3V3 | Logic ESP32 là 3,3 V; cấp 3V3 để mức I2C không vượt 3,3 V |
| GND | GND |  |
| SDA | GPIO21 | Chân SDA mặc định của `Wire` trên ESP32 |
| SCL | GPIO22 | Chân SCL mặc định |
| ADR / COM3, INT, RST, PS0, PS1 | Để trống | Địa chỉ 0x28 hoặc 0x29 tùy ADR; firmware tự thử cả hai |

- Theo bảng chế độ giao tiếp trong Mischianti, **PS1 = PS0 = 0 là I2C**. Bài viết cũng cho biết muốn chuyển sang UART thì phải tháo PS1, nghĩa là module mặc định chạy I2C.
- Chỉ khi quét I2C (bước 3) không thấy thiết bị nào, mới nối PS0 và PS1 xuống GND.
- Dây SDA/SCL nên ngắn, dưới 20 cm. BNO055 dùng clock stretching, và Adafruit xếp nó vào nhóm chip "khó tính" với ESP32. Dây dài làm lỗi dễ xảy ra hơn.
- Mình **không có tài liệu gốc** cho biết GY-BNO055 có sẵn điện trở kéo lên trên SDA/SCL hay không. Nếu bước 3 hoặc 4 báo lỗi I2C, xem mục Xử lý sự cố.

## Bước 3 – Quét I2C

Mục đích: xác nhận dây đúng và biết địa chỉ thật của module trước khi chạy firmware chính. Chạy trong `lab2/` (hoặc dùng mục *Project Tasks → i2c\_scan* trong PlatformIO):

```bash
pio run -e i2c_scan -t upload
pio device monitor -e i2c_scan        # 115200 baud; Ctrl+C để thoát
```

| Kết quả | Ý nghĩa | Làm tiếp |
| --- | --- | --- |
| `found 0x28  <- BNO055?` hoặc `0x29` | Dây đúng, module trả lời | Sang bước 4 |
| `0 device(s)` | Không có thiết bị nào ACK | Kiểm tra SDA/SCL có bị đảo, VIN/GND; thử nối PS0, PS1 xuống GND; xem mục Xử lý sự cố |
| Thấy địa chỉ khác 0x28/0x29 | Không phải BNO055, hoặc là thiết bị khác trên bus | Kiểm tra lại module |

## Bước 4 – Nạp firmware imu\_logger và đọc thử

Đặt module nằm yên trên bàn. Khi ESP32 khởi động, firmware mất 2 s để ước lượng bias gyro, và module phải không bị chạm trong khoảng này.

```bash
pio run -e imu_logger -t upload
pio device monitor -e imu_logger      # 460800 baud
```

Kết quả mong đợi (giá trị cụ thể trên máy bạn sẽ khác):

```
# lab2 imu_logger booting
# calibrating gyro bias: KEEP STILL for 2 s
# gyro_bias_x=...,gyro_bias_y=...,gyro_bias_z=...,gyro_bias_std_max=...
# bno_addr=0x28,mode=ACCGYRO,fs_hz=100,ma_n=10,cf_alpha=0.9800,...
# cols:seq,t_us,ax,ay,az,gx,gy,gz,roll_acc,roll_ma,roll_cf,roll_gyro,...
0,1234567,0.12,-0.08,9.79,...
```

Kiểm tra nhanh ba điểm:

1. Module nằm ngang, mặt chip hướng lên thì `az` ≈ +9,8. `ax`, `ay` ≈ 0. Úp ngược thì `az` ≈ −9,8.
2. Không có dòng `# WARN sensor moved during bias estimation`. Nếu có, nhấn nút EN để khởi động lại và giữ yên.
3. Dòng `# STAT i2c_fail=0,...` (in mỗi 5 s) phải giữ `i2c_fail=0`.

Tốc độ 460800 baud là bắt buộc. Mỗi dòng dài khoảng 120 byte, ở 100 Hz là khoảng 12 kB/s. Con số này vượt 115200 baud (khoảng 11,5 kB/s), nên bài 1 dùng 115200 được còn bài này thì không.

**Thoát monitor (Ctrl+C) trước khi chạy script Python**, vì một cổng serial chỉ mở được bởi một chương trình.

## Bước 5 – Kiểm tra dấu trục gyro

CF giả định vận tốc góc ω cùng dấu với tốc độ thay đổi của góc gia tốc kế. Nếu ngược dấu, CF vẫn chạy nhưng cho kết quả sai mà không báo lỗi gì, nên cần kiểm tra một lần trước khi đo thật.

1. Đặt module nằm yên, chạy `python scripts/log_serial.py --label signcheck --duration 20`.
2. Khi script in `[log] ghi 2000 mau`, nghiêng module **chậm** quanh trục x khoảng ±30° vài lần. Sau đó làm tương tự quanh trục y.
3. Chạy `python scripts/analyze.py data/signcheck.csv --axis both --ref-label none`.
4. Xem cột **Dấu gyro** trong bảng kiểm chứng (in ra màn hình và lưu ở `results/verification.md`):

| Kết quả | Làm gì |
| --- | --- |
| `khớp (r=0.9x)` ở cả roll và pitch | Giữ nguyên, sang bước 6 |
| `NGƯỢC (r=-0.9x)` | Đổi `GYRO_SIGN_ROLL` hoặc `GYRO_SIGN_PITCH` thành `-1.0f` trong `main.cpp`, nạp lại rồi lặp lại bước này |
| `không rõ` hoặc `chưa đủ chuyển động` | Nghiêng rộng hơn và chậm hơn rồi đo lại |

Theo lý thuyết (quy ước tay phải), dấu `+1` là đúng. Tuy vậy, mình chưa xác nhận được hướng trục in trên board GY-BNO055 của bạn, nên bước này là để kiểm chứng bằng dữ liệu. Đã kiểm thử bằng dữ liệu giả lập: trường hợp đúng dấu cho r = 1,00, trường hợp cố ý đảo dấu cho r = −1,00.

## Bước 6 – Thí nghiệm A (đứng yên) và B (rung nhẹ)

Cả hai lần đo phải **cùng một tư thế**. Nhờ vậy góc trung bình của lần đứng yên dùng được làm tham chiếu để tính RMS sai số cho lần rung.

**Chuẩn bị chung:**

- Dán module bằng băng keo 2 mặt lên tấm phẳng. Để tấm trên bàn, có thể kê nghiêng một chút (vài độ) để góc khác 0.
- Để dây USB chùng, không kéo căng.
- Đo trong `lab2/`, venv đã kích hoạt, monitor đã đóng.

**A – Đứng yên (60 s):**

```bash
python scripts/log_serial.py --label static --duration 60
```

Không chạm bàn trong suốt 60 s. Script tự reset ESP32, chờ hiệu chuẩn bias, rồi ghi 6000 mẫu.

**B – Rung nhẹ (60 s), không xê dịch tấm:**

```bash
python scripts/log_serial.py --label vibration --duration 60
```

Đợi đến khi màn hình in `[log] ghi 6000 mau` mới bắt đầu tạo rung, vì 2 s đầu là hiệu chuẩn bias.

- Cách tạo rung: gõ nhẹ đều tay xuống bàn cạnh tấm, khoảng 2–4 lần mỗi giây, hoặc đặt một điện thoại đang rung lên cùng tấm.
- Rung đủ để thấy tín hiệu, nhưng **tấm không được nghiêng hẳn đi**.
- Ghi lại cách tạo rung vào báo cáo để người khác lặp lại được.

Thời lượng 60 s cho 6000 mẫu mỗi điều kiện, đủ để tính trôi bằng cửa sổ 10 s đầu và 10 s cuối.

Nếu muốn mở rộng (không bắt buộc), thêm thí nghiệm C `--label tilt`: nghiêng chậm module 20–30° rồi giữ. Phóng to trên đồ thị sẽ thấy rõ độ trễ của MA so với CF khi góc thực sự thay đổi.

## Bước 7 – Phân tích, đọc kết quả, viết nhận xét

```bash
python scripts/analyze.py --axis both     # mặc định đọc data/static.csv và data/vibration.csv
```

Script tạo ra các file trong `results/`:

| File | Nội dung | Dùng trong báo cáo |
| --- | --- | --- |
| `static_roll.png`, `vibration_roll.png` (và `_pitch`) | (a) raw, MA và CF chồng lên nhau trên toàn bản ghi; (b) phóng to 3 s; (c) tích phân gyro so với CF | Đồ thị so sánh theo yêu cầu đề bài |
| `summary_std_roll.png` (và `_pitch`) | Độ lệch chuẩn của raw, MA, CF; mỗi điều kiện một ô có trục y riêng | Hình tổng hợp |
| `metrics.md`, `metrics.csv` | Bảng chỉ số định lượng | Bảng chính của báo cáo |
| `verification.md` | Mất mẫu, jitter, sai khác ESP32 với Python, dấu gyro | Bằng chứng cài đặt đúng |

**Đọc `verification.md` trước tiên.** Một dòng đạt khi đủ ba điều kiện:

- Mất mẫu = 0.
- Sai khác ESP32 với Python < 0,01° ở cả bốn tín hiệu (ngưỡng cho chênh lệch float32 với float64 và làm tròn khi in).
- Dấu gyro không ngược.

Khi đạt, bạn có bằng chứng rằng bộ lọc chạy trên ESP32 cho cùng kết quả với cài đặt độc lập bằng Python. Một dòng chưa đạt thì xem lại trước khi viết báo cáo.

**Ý nghĩa các chỉ số** (tính sau khi bỏ 2 s đầu):

| Chỉ số | Định nghĩa | Đọc thế nào |
| --- | --- | --- |
| Độ lệch chuẩn (°) | Độ phân tán quanh trung bình | Đo nhiễu; càng nhỏ càng ổn định |
| Đỉnh–đỉnh (°) | max − min | Dao động xấu nhất |
| Trôi (°) | TB 10 s cuối − TB 10 s đầu | Gần 0 là không trôi; tích phân gyro thuần sẽ lớn |
| Giảm nhiễu (dB) | 20·log₁₀(σ\_raw / σ\_lọc) | 10 dB ≈ giảm 3,16 lần; 20 dB = giảm 10 lần |
| RMS so với tham chiếu (°) | √mean((θ − θ\_ref)²), với θ\_ref là góc raw trung bình lúc đứng yên | Gộp cả nhiễu lẫn lệch; chỉ có nghĩa khi hai lần đo cùng tư thế |

**Gợi ý khung nhận xét.** Kết luận phải rút ra từ số liệu của bạn, không phải từ phần này:

1. **Đứng yên.** So σ của MA và CF với raw. Tỷ số σ\_raw/σ\_MA có gần √10 ≈ 3,16 như lý thuyết về nhiễu trắng không? Nếu khác nhiều, nhiễu có thể không "trắng" (gợi ý: bộ lọc thông thấp tích hợp trong cảm biến).
2. **Rung nhẹ.** σ của raw tăng bao nhiêu lần so với lúc đứng yên? MA hay CF giữ góc gần tham chiếu hơn (cột RMS)? Giải thích bằng cơ chế: MA chỉ trung bình gia tốc kế, nên rung chậm hơn cửa sổ 100 ms vẫn lọt qua. CF dựa vào gyro, vốn không nhạy với gia tốc tịnh tiến.
3. **Trôi.** Dùng ô (c): tích phân gyro thuần trôi bao nhiêu độ trong 60 s? CF có trôi không? Điều này cho thấy vai trò của số hạng (1 − α)·θ\_acc.
4. **Đánh đổi.** MA có độ trễ khoảng 45 ms (N = 10) và cần tính toán ít hơn. CF cần thêm gyro và bias đã hiệu chuẩn, có quá độ khoảng 4τ ≈ 2 s khi khởi động.
5. **Kết luận ứng dụng.** Bộ lọc nào phù hợp hơn cho ứng dụng thực tế có rung hoặc chuyển động (robot, tay máy)? Trong điều kiện nào MA vẫn đủ dùng?

Để tham khảo dạng kết quả: trên **dữ liệu giả lập** dùng khi kiểm thử code, MA với N = 10 giảm σ khoảng 3,35 lần lúc đứng yên, gần với √10. Đây là dữ liệu giả, **không dùng cho báo cáo**.

## Bước 8 – Nộp bài và đối chiếu tiêu chí chấm

- [ ] Code ESP32: `lab2/platformio.ini`, `lab2/src/` (cả `i2c_scan` và `imu_logger`)
- [ ] Script Python: `lab2/scripts/*.py`, `requirements.txt`
- [ ] Dữ liệu thô + đã lọc: `data/static.csv`, `data/vibration.csv` và hai file `.meta.json` đi kèm (cấu hình bộ lọc, bias gyro)
- [ ] Đồ thị: `results/static_*.png`, `results/vibration_*.png`, `results/summary_std_*.png`
- [ ] Bảng `results/metrics.md` và `results/verification.md`
- [ ] Báo cáo nhận xét theo 5 ý ở bước 7, có ghi cách tạo rung và việc thay MPU6050 bằng BNO055

Đối chiếu với tiêu chí chấm trong `lab_2.docx`:

| Tiêu chí | Điểm | Bằng chứng trong bài nộp |
| --- | --- | --- |
| Đọc IMU qua I2C chính xác, ổn định | 2 | `verification.md`: mất mẫu = 0, jitter; dòng `STAT i2c_fail=0`; `az` ≈ 9,8 lúc nằm ngang |
| Cài đặt đúng Moving Average | 2 | Code `filters.h`; sai khác ESP32 với Python < 0,01°; σ giảm gần √N lúc đứng yên |
| Complementary Filter giảm nhiễu rõ rệt, có đồ thị | 3 | Đồ thị (a)–(c), cột giảm nhiễu (dB) và RMS lúc rung |
| Demo 1-Wire và analog, phân biệt 3 kiểu giao tiếp | 1 | Thuộc phần thực hành trên lớp, không nằm trong gói này |
| Báo cáo so sánh các phương pháp lọc | 2 | Bảng `metrics.md` + nhận xét |

Tiêu chí ghi "Đọc MPU6050". Bạn nên hỏi giảng viên xem BNO055 có được chấp nhận thay thế không.

## Xử lý sự cố

| Triệu chứng | Nguyên nhân có thể | Cách xử lý |
| --- | --- | --- |
| `Permission denied: '/dev/ttyUSB0'` | User chưa có quyền với cổng | Làm bước 1 mục 2, rồi đăng xuất và đăng nhập lại |
| Không có `/dev/ttyUSB*` | Cáp chỉ sạc, hoặc cổng USB lỏng | Đổi cáp, chạy `dmesg \| tail` khi cắm để xem hệ thống có nhận không |
| Quét I2C ra `0 device(s)` | Sai dây, đảo SDA/SCL, PS0/PS1 không ở mức 0 | Kiểm tra bảng ở bước 2; nối PS0, PS1 xuống GND |
| `# ERROR BNO055 not found` hoặc `begin() failed` | Như trên, hoặc chip chưa khởi động xong | Nhấn EN; thư viện đã chờ tối đa 850 ms + 1 s cho lần khởi động đầu |
| `STAT i2c_fail` tăng dần | Clock stretching của BNO055, dây dài, điện trở kéo lên yếu | Dây ngắn; giảm `I2C_HZ` xuống 50000; nếu đo không thấy điện trở kéo lên giữa SDA/SCL và VIN, thêm 4,7 kΩ |
| Script báo không thấy `# cols:` | Sai baud, monitor còn mở, hoặc reset không chạy | Đóng monitor; kiểm tra `--baud 460800`; thử nhấn EN ngay sau khi chạy script |
| Có dòng hỏng hoặc mất mẫu > 0 | Đường truyền không giữ được 460800 | Đổi `SERIAL_BAUD`, `monitor_speed` và `--baud` cùng về 230400 (vẫn đủ cho khoảng 12 kB/s) |
| `# WARN sensor moved during bias estimation` | Bị chạm trong 2 s đầu | Khởi động lại và giữ yên |
| CF lệch hẳn khỏi raw khi đứng yên | Bias sai (rung trong lúc hiệu chuẩn) hoặc dấu gyro ngược | Đo lại; chạy bước 5 |
| `pio run` báo lỗi biên dịch | Code chưa được biên dịch bằng PlatformIO thật (xem Tổng quan) | Gửi lại toàn bộ log lỗi để mình sửa |

## Nguồn tham khảo

- `lab_2.docx` (project): yêu cầu "THỰC HÀNH VỀ NHÀ" và tiêu chí chấm. `lab1/` trong repo practical\_dktd: kiến trúc timer + queue được tái sử dụng.
- [Adafruit\_BNO055 – Adafruit\_BNO055.cpp / .h (v1.6.4)](https://github.com/adafruit/Adafruit_BNO055): `begin()`, chế độ ACCGYRO = 0x05, địa chỉ 0x28/0x29, thanh ghi 0x08–0x19, hệ số 1 m/s² = 100 LSB và 1 dps = 16 LSB, ghi chú clock stretching ≥ 500 µs.
- [Mischianti – BNO055: features, configuration and axes remap](https://mischianti.org/bno055-for-esp32-esp8266-and-arduino-features-configuration-and-axes-remap-3/): các chế độ non-fusion (ACCGYRO, AMG) và fusion.
- [Mischianti – BNO055: wiring and advanced Bosch library](https://mischianti.org/bno055-for-esp32-esp8266-and-arduino-wiring-and-advanced-bosch-library-2/): logic 3,3 V, bảng PS1/PS0, nối dây cơ bản 4 chân.
- [Adafruit CircuitPython BNO055 – API reference](https://docs.circuitpython.org/projects/bno055/en/latest/api.html): cấu hình mặc định ở chế độ non-fusion (gia tốc ±4 g, băng thông 62,5 Hz; gyro 2000 dps, 32 Hz).
- [Adafruit Learn – I2C troublesome chips](https://learn.adafruit.com/i2c-addresses/troublesome-chips): BNO055 dùng clock stretching, hoạt động không tốt với ESP32 trên ESP-IDF cũ.
- [arduino-esp32 – Wire.h 2.0.17](https://github.com/espressif/arduino-esp32/blob/2.0.17/libraries/Wire/src/Wire.h) và [3.1.1](https://github.com/espressif/arduino-esp32/blob/3.1.1/libraries/Wire/src/Wire.h): `requestFrom(uint8_t, size_t, bool)`, `setTimeOut`.
- [esptool v4.7.0 – reset.py](https://github.com/espressif/esptool/blob/v4.7.0/esptool/reset.py): trình tự HardReset (RTS → EN thấp) dùng trong `log_serial.py`.
- [PlatformIO – 99-platformio-udev.rules](https://docs.platformio.org/en/latest/core/installation/udev-rules.html): quyền cổng serial trên Linux.
- S. Colton, [The Balance Filter](http://web.mit.edu/scolton/www/filter.pdf), MIT, 2007: công thức CF, τ = αΔt/(1−α), ví dụ α = 0,98 ở 100 Hz, lệch ≈ bias × τ.
- S. W. Smith, [The Scientist and Engineer's Guide to DSP, chương 15 – Moving Average Filters](https://www.dspguide.com/ch15/2.htm): MA giảm nhiễu √N lần.
- [Cirkit Designer – BNO055 + ESP32](https://docs.cirkitdesigner.com/component/ed0fa7f5-8653-4bd3-b8c2-4a1ccaf0e284/bno055): SDA GPIO21, SCL GPIO22, cấp nguồn từ 3V3.

Chưa đối chiếu: datasheet gốc của Bosch (BST-BNO055-DS000) và sơ đồ mạch GY-BNO055. Các thông số trên lấy từ mã nguồn thư viện và tài liệu thứ cấp.
