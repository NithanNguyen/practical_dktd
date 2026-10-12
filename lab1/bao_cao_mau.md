# Báo cáo ngắn – Buổi 1: Lấy mẫu ADC 100 Hz bằng Timer ngắt phần cứng

> Mẫu báo cáo. Các ô `[...]` điền từ kết quả đo thật của bạn (file `stats.md`, `signal.png`, `timing.png` do script Python sinh ra). Không điền số liệu giả định.

## 1. Mục tiêu
Lấy mẫu 1 cảm biến analog ở tần số cố định f_s = 100 Hz (T = 10 ms) bằng Timer ngắt phần cứng của ESP32, không dùng `delay()` trong vòng lặp chính; đo độ trễ và jitter thực tế của chu kỳ lấy mẫu.

## 2. Phần cứng và nối dây
- Board: NodeMCU-32S; môi trường: PlatformIO (VS Code) trên Ubuntu 20.04; Arduino-ESP32 core: `[phiên bản, xem log build]`.
- Cảm biến: `[biến trở / LDR]`, nối vào GPIO34 (ADC1_CH6).
  - Biến trở: 2 chân ngoài → 3V3 và GND, chân giữa → GPIO34.
  - LDR: 3V3 → LDR → GPIO34 → điện trở `[R, VD 10 kΩ]` → GND (cầu phân áp).
- (Tùy chọn) GPIO25: chân debug đảo mức mỗi lần vào ISR, để đo bằng oscilloscope/logic analyzer.

## 3. Phương pháp
**Firmware.** Hardware timer đếm 1 MHz, alarm auto-reload mỗi 10 000 tick. ISR chỉ ghi mốc `t_isr = micros()`, số thứ tự `seq`, rồi đẩy vào queue. Task lấy mẫu (ưu tiên cao hơn `loop()`) gọi `analogRead()` và ghi `t_read`. `loop()` in mỗi mẫu thành 1 dòng CSV `seq,t_isr_us,t_read_us,raw` ở 115200 baud.

**Đại lượng đo** (mọi mốc thời gian lấy trên đồng hồ ESP32, không dùng đồng hồ PC):
| Ký hiệu | Định nghĩa | Ý nghĩa |
|---|---|---|
| T_k | t_isr[k] − t_isr[k−1] | chu kỳ lấy mẫu thực tế |
| Jitter đỉnh-đỉnh | max(T_k) − min(T_k) | dao động lớn nhất của chu kỳ |
| Jitter RMS | độ lệch chuẩn của T_k | dao động điển hình |
| L_k | t_read[k] − t_isr[k] | độ trễ từ lúc ngắt đến khi có giá trị ADC |
| Mất mẫu | Σ(seq[k] − seq[k−1] − 1) | số ngắt không đến được PC |

Thời gian ghi: `[... s]`, số mẫu: `[...]`.

## 4. Kết quả
(Dán nội dung `stats.md` vào đây.)

Hình 1 – Tín hiệu theo thời gian: `signal.png`
Hình 2 – Chu kỳ T_k, phân bố jitter, phân bố độ trễ: `timing.png`

## 5. Nhận xét
- So sánh T trung bình với 10 000 µs: `[...]`.
- Jitter đỉnh-đỉnh / RMS: `[...]` µs, tương ứng `[...]` % chu kỳ.
- Độ trễ L_k trung bình / lớn nhất: `[...]` µs; thành phần chính gồm chuyển ngữ cảnh ISR → task và thời gian chuyển đổi ADC.
- Mất mẫu: `[...]`.
- Giới hạn phép đo: mốc thời gian dùng chính đồng hồ ESP32 nên không phản ánh sai số tuyệt đối của thạch anh; `[nếu có: kết quả đo kiểm chứng bằng oscilloscope trên GPIO25]`.

## 6. Kết luận
`[1–3 câu]`
