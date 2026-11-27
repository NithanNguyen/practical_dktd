// Gia lap toi thieu Arduino-ESP32 de bien dich firmware/src/main.cpp tren PC (chi de chay thu).
// KHONG phai phan cung that: dong co, encoder, nut nhan duoc mo phong trong harness.cpp.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

using std::max;
using std::min;

#define IRAM_ATTR
#define DRAM_ATTR
#define LOW 0
#define HIGH 1
#define INPUT 0x01
#define OUTPUT 0x03
#define INPUT_PULLUP 0x05
#define CHANGE 3
#define FALLING 2
#define RISING 1

template <class T, class L, class H>
static inline T constrain(T x, L lo, H hi) { return x < (T)lo ? (T)lo : (x > (T)hi ? (T)hi : x); }

typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(m) ((void)(m))
#define portEXIT_CRITICAL(m) ((void)(m))
#define portENTER_CRITICAL_SAFE(m) ((void)(m))
#define portEXIT_CRITICAL_SAFE(m) ((void)(m))

// ---- ham do harness.cpp cung cap
uint32_t sim_micros();
void sim_service();
void sim_serial_write(const char *s, size_t n);
int sim_serial_available();
int sim_serial_read();
int sim_digital_read(int pin);
void sim_attach_interrupt(int pin, void (*fn)(), int mode);
void sim_ledc_write(int duty);

static inline unsigned long millis() { return sim_micros() / 1000UL; }
static inline unsigned long micros() { return sim_micros(); }
static inline void delay(unsigned long ms) {
  uint32_t t0 = sim_micros();
  while (sim_micros() - t0 < ms * 1000UL) sim_service();
}
static inline void pinMode(int, int) {}
static inline int digitalRead(int pin) { return sim_digital_read(pin); }
static inline int digitalPinToInterrupt(int pin) { return pin; }
static inline void attachInterrupt(int pin, void (*fn)(), int mode) { sim_attach_interrupt(pin, fn, mode); }
static inline void ledcSetup(int, uint32_t, int) {}
static inline void ledcAttachPin(int, int) {}
static inline void ledcWrite(int, uint32_t duty) { sim_ledc_write((int)duty); }

class String {
 public:
  String(const char *s) : s_(s ? s : "") {}
  void toLowerCase() { for (auto &c : s_) c = (char)tolower((unsigned char)c); }
  bool operator==(const char *o) const { return s_ == o; }
  bool operator!=(const char *o) const { return s_ != o; }
 private:
  std::string s_;
};

class HardwareSerial {
 public:
  void setTxBufferSize(size_t) {}
  void begin(unsigned long) {}
  int available() { return sim_serial_available(); }
  int read() { return sim_serial_read(); }
  int printf(const char *fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) sim_serial_write(buf, (size_t)std::min(n, (int)sizeof(buf) - 1));
    return n;
  }
  void println(const char *s) { sim_serial_write(s, strlen(s)); sim_serial_write("\n", 1); }
};
extern HardwareSerial Serial;
