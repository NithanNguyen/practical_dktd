# Ket qua do: data.csv

- So mau: 3001, thoi gian ghi: 30.000 s
- Tan so danh nghia: 100.000 Hz; tan so hieu dung (theo dong ho ESP32): 100.0000 Hz
- So mau bi mat (khe ho seq): 0
- Do lech tan so tuyen tinh uoc luong: -0.0000 us/mau (-0.0 ppm)

| Dai luong (us) | N | Trung binh | Do lech chuan | Min | Max | P99 |
|---|---|---|---|---|---|---|
| Chu ky T_k | 3000 | 10000.00 | 0.22 | 9988 | 10000 | 10000 |
| abs(T_k - T_nom) | 3000 | 0.00 | 0.22 | 0 | 12 | 0 |
| Do tre ISR->ADC xong | 3001 | 217.07 | 3.87 | 217 | 429 | 217 |

- Jitter dinh-dinh cua chu ky (max - min): 12 us
- Jitter RMS cua chu ky (do lech chuan T_k): 0.22 us
- TIE sau khi bo xu huong: RMS 0.22 us, max |TIE| 12.0 us

Luu y: phep do dung chinh dong ho cua ESP32 (micros()), nen chi phan anh jitter do phan mem/ngat, khong phan anh sai so tuyet doi cua thach anh. Muon do doc lap: dung oscilloscope/logic analyzer tren chan DEBUG_PIN (GPIO25).
