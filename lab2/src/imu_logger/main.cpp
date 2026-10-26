/*
 * Buoi 2 - THUC HANH VE NHA
 * Doc IMU BNO055 (module GY-BNO055) qua I2C o 100 Hz bang Timer ngat phan cung,
 * tinh goc nghieng (roll, pitch) tu gia toc ke, roi loc bang:
 *   (1) Moving Average N mau      tren goc tu gia toc ke
 *   (2) Complementary Filter      ket hop gia toc ke + con quay hoi chuyen
 *   (3) Tich phan gyro thuan tuy  (chi de doi chung: cho thay hien tuong troi)
 * Tat ca tinh tren CUNG mot mau -> so sanh tren cung mot tap du lieu.
 *
 * BNO055 dat o che do NON-FUSION "ACCGYRO": chip chi xuat du lieu tho cua
 * gia toc ke va gyro, KHONG chay thuat toan fusion cua Bosch. Nhu vay moi bo
 * loc trong bai la do chinh minh cai dat.
 *
 * Kien truc (giong Buoi 1):
 *   HW timer 100 Hz -> onTimer() [ISR: chi lay moc thoi gian + seq]
 *                   -> samplerTask [doc I2C, tinh goc, loc]  (uu tien cao)
 *                   -> loop()      [in 1 dong CSV ra Serial] (uu tien thap)
 *
 * Dinh dang du lieu (1 dong / mau); dong bat dau bang '#' la thong tin/meta:
 *   seq,t_us,ax,ay,az,gx,gy,gz,
 *   roll_acc,roll_ma,roll_cf,roll_gyro,pitch_acc,pitch_ma,pitch_cf,pitch_gyro
 *   ax..az : m/s^2 (1 m/s^2 = 100 LSB) ; gx..gz : deg/s (1 dps = 16 LSB), CHUA tru bias
 *   *_acc  : goc tu gia toc ke (raw angle)   *_ma : sau Moving Average
 *   *_cf   : sau Complementary Filter        *_gyro : chi tich phan gyro (da tru bias)
 *
 * Ho tro Arduino-ESP32 core 2.x va 3.x (API Timer khac nhau).
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_BNO055.h>
#include "filters.h"
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#ifndef ARDUINO_ISR_ATTR
#define ARDUINO_ISR_ATTR IRAM_ATTR
#endif

// ============================== Cau hinh ====================================
constexpr int      PIN_SDA       = 21;
constexpr int      PIN_SCL       = 22;
constexpr uint32_t I2C_HZ        = 100000;   // 100 kHz (BNO055 co clock stretching)
constexpr uint16_t I2C_TIMEOUT_MS = 20;

constexpr uint32_t SERIAL_BAUD   = 460800;   // phai trung monitor_speed va --baud cua Python
constexpr uint32_t FS_HZ         = 100;
constexpr uint32_t PERIOD_US     = 1000000UL / FS_HZ;

constexpr int      MA_N          = 10;       // cua so Moving Average (mau)
constexpr float    CF_ALPHA      = 0.98f;    // he so Complementary Filter
constexpr int      BIAS_SAMPLES  = 200;      // 2 s o 100 Hz de uoc luong bias gyro

// Dau cua van toc goc so voi goc tu gia toc ke. Theo quy uoc tay phai,
// roll = atan2(ay, az) tang khi quay duong quanh +x -> gx > 0, tuong tu pitch/gy.
// Neu buoc kiem tra dau (huong dan Buoc 5) cho thay nguoc dau, doi thanh -1.
constexpr float    GYRO_SIGN_ROLL  = +1.0f;
constexpr float    GYRO_SIGN_PITCH = +1.0f;

constexpr UBaseType_t SAMPLER_PRIO = 5;
constexpr BaseType_t  APP_CORE     = 1;
constexpr size_t      ISR_Q_LEN    = 16;
constexpr size_t      TX_Q_LEN     = 64;
constexpr uint32_t    STAT_EVERY_MS = 5000;

// Thanh ghi BNO055 (Adafruit_BNO055.h v1.6.4): ACC_DATA_X_LSB = 0x08 ... GYR_DATA_Z_MSB = 0x19
constexpr uint8_t  REG_DATA_START = 0x08;
constexpr uint8_t  DATA_LEN       = 18;      // acc(6) + mag(6, bo qua) + gyr(6)
// ============================================================================

struct IsrEvent { uint32_t seq; uint32_t t_us; };
struct Sample {
  uint32_t seq, t_us;
  float ax, ay, az, gx, gy, gz;
  float roll_acc, roll_ma, roll_cf, roll_g;
  float pitch_acc, pitch_ma, pitch_cf, pitch_g;
};

static uint8_t           bnoAddr  = 0;
static Adafruit_BNO055*  bno      = nullptr;
static hw_timer_t*       timer    = nullptr;
static QueueHandle_t     isrQueue = nullptr;
static QueueHandle_t     txQueue  = nullptr;
static volatile uint32_t isrSeq   = 0;
static volatile uint32_t isrQueueFull = 0, txQueueFull = 0, i2cFail = 0;
static float biasX = 0, biasY = 0, biasZ = 0;   // deg/s

// --------------------------- Doc BNO055 -------------------------------------
// 1 giao dich I2C: ghi dia chi thanh ghi (repeated start) roi doc 18 byte.
static bool readAccGyro(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  Wire.beginTransmission(bnoAddr);
  Wire.write(REG_DATA_START);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(bnoAddr, (size_t)DATA_LEN, true) != (size_t)DATA_LEN) return false;
  uint8_t b[DATA_LEN];
  for (uint8_t i = 0; i < DATA_LEN; ++i) b[i] = (uint8_t)Wire.read();
  auto s16 = [&](int i) { return (int16_t)((uint16_t)b[i] | ((uint16_t)b[i + 1] << 8)); };
  ax = s16(0)  / 100.0f;  ay = s16(2)  / 100.0f;  az = s16(4)  / 100.0f;   // m/s^2
  gx = s16(12) / 16.0f;   gy = s16(14) / 16.0f;   gz = s16(16) / 16.0f;    // deg/s
  return true;
}

// --------------------------- ISR & task -------------------------------------
void ARDUINO_ISR_ATTR onTimer() {
  IsrEvent ev{isrSeq++, (uint32_t)micros()};
  BaseType_t woken = pdFALSE;
  if (xQueueSendFromISR(isrQueue, &ev, &woken) != pdTRUE) isrQueueFull++;
  if (woken == pdTRUE) portYIELD_FROM_ISR();
}

static void samplerTask(void*) {
  lab2::MovingAverage<MA_N> maRoll, maPitch;
  lab2::Complementary cfRoll(CF_ALPHA), cfPitch(CF_ALPHA);
  lab2::GyroIntegrator giRoll, giPitch;
  uint32_t lastT = 0; bool first = true;
  IsrEvent ev;
  for (;;) {
    if (xQueueReceive(isrQueue, &ev, portMAX_DELAY) != pdTRUE) continue;
    Sample s;
    if (!readAccGyro(s.ax, s.ay, s.az, s.gx, s.gy, s.gz)) { i2cFail++; continue; }
    s.seq = ev.seq;  s.t_us = ev.t_us;
    // dt do tu moc ISR (khong gia dinh dung 10 ms); uint32 tru nhau dung ca khi tran
    float dt = first ? 0.0f : (uint32_t)(ev.t_us - lastT) * 1e-6f;
    lastT = ev.t_us;  first = false;

    s.roll_acc  = lab2::rollFromAcc(s.ay, s.az);
    s.pitch_acc = lab2::pitchFromAcc(s.ax, s.ay, s.az);
    float wRoll  = GYRO_SIGN_ROLL  * (s.gx - biasX);
    float wPitch = GYRO_SIGN_PITCH * (s.gy - biasY);

    s.roll_ma   = maRoll.update(s.roll_acc);
    s.pitch_ma  = maPitch.update(s.pitch_acc);
    s.roll_cf   = cfRoll.update(s.roll_acc, wRoll, dt);
    s.pitch_cf  = cfPitch.update(s.pitch_acc, wPitch, dt);
    s.roll_g    = giRoll.update(s.roll_acc, wRoll, dt);
    s.pitch_g   = giPitch.update(s.pitch_acc, wPitch, dt);

    if (xQueueSend(txQueue, &s, 0) != pdTRUE) txQueueFull++;
  }
}

static void startSamplingTimer() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  timer = timerBegin(1000000);                 // 1 tick = 1 us
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, PERIOD_US, true, 0);
#else
  timer = timerBegin(0, 80, true);             // APB 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, PERIOD_US, true);
  timerAlarmEnable(timer);
#endif
}

[[noreturn]] static void halt(const char* msg) {
  Serial.printf("# ERROR %s\n", msg);
  for (;;) { vTaskDelay(portMAX_DELAY); }
}

static bool probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Uoc luong bias gyro: trung binh BIAS_SAMPLES mau khi cam bien dung yen.
// Chi chay 1 lan trong setup() truoc khi bat timer.
static void estimateGyroBias() {
  double sx = 0, sy = 0, sz = 0, sxx = 0, syy = 0, szz = 0;
  int n = 0, fails = 0;
  while (n < BIAS_SAMPLES) {
    float ax, ay, az, gx, gy, gz;
    if (readAccGyro(ax, ay, az, gx, gy, gz)) {
      sx += gx; sy += gy; sz += gz;
      sxx += gx * gx; syy += gy * gy; szz += gz * gz;
      ++n;
    } else if (++fails > 50) {
      halt("i2c read failed during gyro bias estimation");
    }
    delay(PERIOD_US / 1000);
  }
  biasX = sx / n;  biasY = sy / n;  biasZ = sz / n;
  float sdMax = sqrtf(fmaxf(fmaxf(sxx / n - biasX * biasX, syy / n - biasY * biasY),
                            szz / n - biasZ * biasZ));
  Serial.printf("# gyro_bias_x=%.6f,gyro_bias_y=%.6f,gyro_bias_z=%.6f,gyro_bias_std_max=%.4f\n",
                biasX, biasY, biasZ, sdMax);
  if (sdMax > 0.5f) Serial.println("# WARN sensor moved during bias estimation, restart and keep still");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.println("# lab2 imu_logger booting");

  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);

  if      (probe(BNO055_ADDRESS_A)) bnoAddr = BNO055_ADDRESS_A;   // 0x28
  else if (probe(BNO055_ADDRESS_B)) bnoAddr = BNO055_ADDRESS_B;   // 0x29
  else halt("BNO055 not found at 0x28/0x29 (check wiring, run env i2c_scan)");

  bno = new Adafruit_BNO055(55, bnoAddr, &Wire);
  if (!bno->begin(OPERATION_MODE_ACCGYRO)) halt("BNO055 begin() failed (chip id != 0xA0?)");
  delay(50);

  Serial.println("# calibrating gyro bias: KEEP STILL for 2 s");
  estimateGyroBias();

  isrQueue = xQueueCreate(ISR_Q_LEN, sizeof(IsrEvent));
  txQueue  = xQueueCreate(TX_Q_LEN,  sizeof(Sample));
  if (!isrQueue || !txQueue) halt("xQueueCreate failed");
  xTaskCreatePinnedToCore(samplerTask, "sampler", 4096, nullptr, SAMPLER_PRIO, nullptr, APP_CORE);

  // Dong meta: script Python doc cac dong nay de tinh lai bo loc offline
  Serial.printf("# bno_addr=0x%02X,mode=ACCGYRO,fs_hz=%lu,ma_n=%d,cf_alpha=%.4f,"
                "gyro_sign_roll=%.0f,gyro_sign_pitch=%.0f,i2c_hz=%lu\n",
                bnoAddr, (unsigned long)FS_HZ, MA_N, CF_ALPHA,
                GYRO_SIGN_ROLL, GYRO_SIGN_PITCH, (unsigned long)I2C_HZ);
  Serial.println("# cols:seq,t_us,ax,ay,az,gx,gy,gz,roll_acc,roll_ma,roll_cf,roll_gyro,"
                 "pitch_acc,pitch_ma,pitch_cf,pitch_gyro");

  startSamplingTimer();
}

void loop() {
  static uint32_t lastStat = millis();
  Sample s;
  if (xQueueReceive(txQueue, &s, pdMS_TO_TICKS(1000)) == pdTRUE) {
    Serial.printf("%lu,%lu,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,"
                  "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                  (unsigned long)s.seq, (unsigned long)s.t_us,
                  s.ax, s.ay, s.az, s.gx, s.gy, s.gz,
                  s.roll_acc, s.roll_ma, s.roll_cf, s.roll_g,
                  s.pitch_acc, s.pitch_ma, s.pitch_cf, s.pitch_g);
  } else {
    Serial.println("# WARN no sample in 1 s");
  }
  if (millis() - lastStat >= STAT_EVERY_MS) {
    lastStat = millis();
    Serial.printf("# STAT i2c_fail=%lu,isr_q_full=%lu,tx_q_full=%lu\n",
                  (unsigned long)i2cFail, (unsigned long)isrQueueFull,
                  (unsigned long)txQueueFull);
  }
}
