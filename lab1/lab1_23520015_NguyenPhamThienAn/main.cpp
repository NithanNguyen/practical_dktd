/*
 * Buoi 1 - THUC HANH VE NHA
 * Lay mau 1 cam bien analog (bien tro hoac LDR) o tan so co dinh 100 Hz
 * bang Timer ngat phan cung. Khong dung delay() trong loop().
 *
 * Kien truc (deferred interrupt processing):
 *
 *   HW timer (1 MHz tick, alarm moi 10 000 tick = 10 ms)
 *        |
 *        v
 *   onTimer() [ISR, rat ngan]:  ghi t_isr = micros(), tang seq,
 *        |                      day {seq, t_isr} vao isrQueue
 *        v
 *   samplerTask [uu tien cao]: nhan su kien -> analogRead() -> ghi t_read
 *        |                      -> day mau day du vao txQueue
 *        v
 *   loop() [uu tien thap]:      lay mau tu txQueue -> in 1 dong CSV ra Serial
 *
 * Ly do tach 3 tang:
 *  - ISR chi lam viec toi thieu (lay moc thoi gian) -> moc t_isr phan anh
 *    dung thoi diem ngat, it bi anh huong boi code khac.
 *  - analogRead() di qua driver ADC cua ESP-IDF; khong dat no trong ISR
 *    (lua chon than trong: ISR nen ngan, cong viec nang day sang task).
 *  - Serial.printf() co the bi chan khi bo dem TX day; dat no o loop()
 *    (uu tien thap) nen khong lam tre chu ky lay mau.
 *
 * Dinh dang du lieu gui len PC (moi dong 1 mau):
 *   seq,t_isr_us,t_read_us,raw
 *   - seq      : so thu tu ngat (tang lien tuc; khe ho => mat mau)
 *   - t_isr_us : micros() tai dau ISR   (dung de do jitter chu ky)
 *   - t_read_us: micros() ngay sau analogRead() (dung de do do tre)
 *   - raw      : gia tri ADC 12-bit (0..4095)
 *
 * Ho tro ca Arduino-ESP32 core 2.x (mac dinh cua platform espressif32
 * chinh thuc tren PlatformIO) va core 3.x (API Timer da doi).
 */

#include <Arduino.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#ifndef ARDUINO_ISR_ATTR            // phong truong hop core cu khong dinh nghia
#define ARDUINO_ISR_ATTR IRAM_ATTR
#endif

// ----------------------------- Cau hinh ------------------------------------
constexpr uint8_t  ADC_PIN        = 34;       // ADC1_CH6, chan chi-input
constexpr uint32_t FS_HZ          = 100;      // tan so lay mau yeu cau
constexpr uint32_t PERIOD_US      = 1000000UL / FS_HZ;  // 10 000 us
constexpr uint32_t SERIAL_BAUD    = 115200;

// Chan debug: dao muc moi lan vao ISR -> co the do chu ky bang
// oscilloscope / logic analyzer (doc lap voi dong ho cua ESP32).
// Dat = -1 de tat.
constexpr int      DEBUG_PIN      = 25;

constexpr UBaseType_t SAMPLER_PRIO = 5;       // > loopTask (uu tien 1)
constexpr BaseType_t  APP_CORE     = 1;       // loop() cung chay tren core 1
constexpr size_t      ISR_Q_LEN    = 16;
constexpr size_t      TX_Q_LEN     = 64;      // ~0.64 s dem neu Serial bi nghen
// ---------------------------------------------------------------------------

struct IsrEvent {
  uint32_t seq;
  uint32_t t_isr_us;
};

struct Sample {
  uint32_t seq;
  uint32_t t_isr_us;
  uint32_t t_read_us;
  uint16_t raw;
};

static hw_timer_t*   timer      = nullptr;
static QueueHandle_t isrQueue   = nullptr;
static QueueHandle_t txQueue    = nullptr;
static volatile uint32_t isrSeq = 0;
static volatile bool     dbgLevel = false;

// Dem so lan mat mau o tung tang (chi de chan doan)
static volatile uint32_t isrQueueFull = 0;
static volatile uint32_t txQueueFull  = 0;

// ----------------------------- ISR -----------------------------------------
void ARDUINO_ISR_ATTR onTimer() {
  IsrEvent ev;
  ev.t_isr_us = (uint32_t)micros();   // lay moc thoi gian TRUOC moi viec khac
  ev.seq      = isrSeq++;

  if (DEBUG_PIN >= 0) {
    // Vi du Timer chinh thuc cua Arduino-ESP32 ghi ro digitalWrite dung duoc trong ISR
    dbgLevel = !dbgLevel;           // giu trang thai trong bien, khong doc lai chan OUTPUT
    digitalWrite(DEBUG_PIN, dbgLevel);
  }

  BaseType_t higherPrioWoken = pdFALSE;
  if (xQueueSendFromISR(isrQueue, &ev, &higherPrioWoken) != pdTRUE) {
    isrQueueFull++;
  }
  if (higherPrioWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

// ----------------------------- Sampler task ---------------------------------
static void samplerTask(void*) {
  IsrEvent ev;
  for (;;) {
    if (xQueueReceive(isrQueue, &ev, portMAX_DELAY) == pdTRUE) {
      Sample s;
      s.seq       = ev.seq;
      s.t_isr_us  = ev.t_isr_us;
      s.raw       = (uint16_t)analogRead(ADC_PIN);
      s.t_read_us = (uint32_t)micros();
      if (xQueueSend(txQueue, &s, 0) != pdTRUE) {
        txQueueFull++;        // Serial khong theo kip -> mat mau (seq se co khe ho)
      }
    }
  }
}

// ----------------------------- Timer setup ----------------------------------
static void startSamplingTimer() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  // Core 3.x: timerBegin(frequency_Hz)
  timer = timerBegin(1000000);                       // 1 tick = 1 us
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, PERIOD_US, true, 0);             // auto-reload, vo han
#else
  // Core 2.x: timerBegin(num, divider, countUp); APB = 80 MHz
  timer = timerBegin(0, 80, true);                   // 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, PERIOD_US, true);           // auto-reload
  timerAlarmEnable(timer);
#endif
}

// ----------------------------- Arduino ---------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);

  if (DEBUG_PIN >= 0) {
    pinMode(DEBUG_PIN, OUTPUT);
    digitalWrite(DEBUG_PIN, LOW);
  }

  analogReadResolution(12);                          // 0..4095
  analogSetPinAttenuation(ADC_PIN, ADC_11db);        // dai do rong nhat

  isrQueue = xQueueCreate(ISR_Q_LEN, sizeof(IsrEvent));
  txQueue  = xQueueCreate(TX_Q_LEN,  sizeof(Sample));
  if (isrQueue == nullptr || txQueue == nullptr) {
    Serial.println("# ERROR: xQueueCreate failed");
    for (;;) { vTaskDelay(portMAX_DELAY); }
  }

  xTaskCreatePinnedToCore(samplerTask, "sampler", 4096, nullptr,
                          SAMPLER_PRIO, nullptr, APP_CORE);

  // Dong tieu de: script Python cho dong nay roi moi bat dau ghi du lieu
  Serial.printf("# fs_hz=%lu,period_us=%lu,adc_pin=%u\n",
                (unsigned long)FS_HZ, (unsigned long)PERIOD_US, ADC_PIN);
  Serial.println("# seq,t_isr_us,t_read_us,raw");

  startSamplingTimer();   // setup() chay tren core 1 -> ngat cung duoc cap tren core 1
}

void loop() {
  Sample s;
  // Cho mau tiep theo; KHONG dung delay(). Task bi chan (block) tren queue
  // nen khong ton CPU khi chua co du lieu.
  if (xQueueReceive(txQueue, &s, pdMS_TO_TICKS(1000)) == pdTRUE) {
    Serial.printf("%lu,%lu,%lu,%u\n",
                  (unsigned long)s.seq, (unsigned long)s.t_isr_us,
                  (unsigned long)s.t_read_us, s.raw);
  } else {
    // 1 s khong co mau nao -> timer/ISR co van de
    Serial.printf("# WARN no sample in 1 s (isrQfull=%lu, txQfull=%lu)\n",
                  (unsigned long)isrQueueFull, (unsigned long)txQueueFull);
  }
}
