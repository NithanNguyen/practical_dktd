/*
 * Buoi 2 - Buoc kiem tra phan cung: quet bus I2C.
 * In ra moi dia chi 7-bit co thiet bi tra loi ACK, lap lai moi 3 s.
 * BNO055 phai hien o 0x28 hoac 0x29 (tuy chan ADR/COM3).
 */
#include <Arduino.h>
#include <Wire.h>

constexpr int      PIN_SDA = 21;
constexpr int      PIN_SCL = 22;
constexpr uint32_t I2C_HZ  = 100000;   // 100 kHz: an toan voi clock stretching cua BNO055

void setup() {
  Serial.begin(115200);
  delay(300);
  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  Serial.println("# I2C scanner: SDA=GPIO21, SCL=GPIO22, 100 kHz");
}

void loop() {
  uint8_t found = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("  found 0x%02X%s\n", addr,
                    (addr == 0x28 || addr == 0x29) ? "  <- BNO055?" : "");
      ++found;
    }
  }
  Serial.printf("# scan done: %u device(s)\n\n", found);
  delay(3000);   // chuong trinh kiem tra, dung delay o day la chap nhan duoc
}
