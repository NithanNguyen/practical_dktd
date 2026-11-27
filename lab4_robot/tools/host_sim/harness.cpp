// Harness chay firmware/src/main.cpp tren PC voi phan cung GIA LAP (chi de chay thu quy trinh).
//
// Serial cua firmware <-> stdin/stdout. Dong stdin bat dau bang '!' la lenh cho harness:
//   !press dir|estop     nhan nut (co mo phong doi phim ~2 ms)
//   !release dir|estop   nha nut
//   !quit
// Tham so dong lenh: --enc-sign -1  (mo phong encoder dau nguoc chieu)
//
// Mo hinh dong co (GIA DINH, lay tu Lab 3 cho co bac do lon hop ly, KHONG phai so do Lab 4):
//   chay  : dw/dt = (K*duty*chieu - w)/T,  K = 1.38 rpm/%, T = 20 ms   (FOPDT Lab 3)
//   phanh : dw/dt = -w/T - ma sat Coulomb
//   tha   : dw/dt = -w/0.25 s - ma sat Coulomb
#include <Arduino.h>
#include <esp_timer.h>
#include <soc/gpio_reg.h>

#include <chrono>
#include <deque>
#include <fcntl.h>
#include <string>
#include <unistd.h>
#include <vector>

HardwareSerial Serial;
void setup();
void loop();

namespace {
constexpr int P_AIN1 = 26, P_AIN2 = 27, P_ENC_A = 32, P_ENC_B = 33, P_DIR = 18, P_ESTOP = 19;
constexpr double K_RPM_PER_PCT = 1.38, T_DRIVE = 0.020, T_COAST = 0.25, F_COULOMB = 276.0;  // rpm/s
constexpr double CPR = 1978.7, DT = 20e-6;
constexpr int PWM_MAX = 1023;

auto t0 = std::chrono::steady_clock::now();
double simT = 0.0;                 // thoi gian mo phong (s), bam theo dong ho that
uint32_t outReg = 0;
int pwmDuty = 0;
double w = 0.0;                    // toc do truc ra (rpm)
double pos = 0.0;                  // vi tri (dem)
long posIdx = 0;
int encSignHw = +1;
int level[40];
void (*isrFn[40])() = {nullptr};
int isrMode[40] = {0};
esp_timer_cb_t timerCb = nullptr;
double timerPeriod = 1e-3, timerNext = 1e-3;
std::deque<char> rx;
std::string lineBuf;
struct PinEvent { double t; int pin; int lvl; };
std::vector<PinEvent> events;
bool quit = false;

double realNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

void setPin(int pin, int lvl) {
  if (level[pin] == lvl) return;
  int old = level[pin];
  level[pin] = lvl;
  if (!isrFn[pin]) return;
  int m = isrMode[pin];
  if (m == CHANGE || (m == FALLING && old == 1 && lvl == 0) || (m == RISING && old == 0 && lvl == 1)) isrFn[pin]();
}

void scheduleButton(int pin, bool press) {
  int fin = press ? 0 : 1, oth = 1 - fin;
  double t = simT;
  events.push_back({t + 0.0000, pin, fin});
  events.push_back({t + 0.0003, pin, oth});     // doi phim
  events.push_back({t + 0.0008, pin, fin});
  events.push_back({t + 0.0015, pin, oth});
  events.push_back({t + 0.0020, pin, fin});
}

void handleHarnessCmd(const std::string &s) {
  char a[16] = {0}, b[16] = {0};
  sscanf(s.c_str(), "!%15s %15s", a, b);
  std::string c(a), btn(b);
  int pin = (btn == "dir") ? P_DIR : (btn == "estop" ? P_ESTOP : -1);
  if (c == "quit") { quit = true; return; }
  if ((c == "press" || c == "release") && pin >= 0) { scheduleButton(pin, c == "press"); return; }
  fprintf(stdout, "#SIM lenh harness khong hop le: %s\n", s.c_str());
  fflush(stdout);
}

void pollStdin() {
  char buf[256];
  ssize_t n;
  while ((n = ::read(0, buf, sizeof(buf))) > 0) {
    for (ssize_t i = 0; i < n; i++) {
      char ch = buf[i];
      if (ch == '\n') {
        if (!lineBuf.empty() && lineBuf[0] == '!') handleHarnessCmd(lineBuf);
        else { for (char x : lineBuf) rx.push_back(x); rx.push_back('\n'); }
        lineBuf.clear();
      } else if (ch != '\r') {
        lineBuf.push_back(ch);
      }
    }
  }
  if (n == 0) quit = true;   // stdin dong: ket thuc
}

void physicsStep(double dt) {
  bool in1 = outReg & (1u << P_AIN1), in2 = outReg & (1u << P_AIN2);
  double d = pwmDuty * 100.0 / PWM_MAX;
  double dw;
  if (in1 && !in2)       dw = (K_RPM_PER_PCT * d - w) / T_DRIVE;
  else if (!in1 && in2)  dw = (-K_RPM_PER_PCT * d - w) / T_DRIVE;
  else if (in1 && in2)   dw = -w / T_DRIVE;          // short brake
  else                   dw = -w / T_COAST;          // stop: tha tu do
  bool driving = (in1 != in2) && d > 0.0;
  if (!driving && w != 0.0) {
    double f = (w > 0 ? -F_COULOMB : F_COULOMB);
    double wn = w + (dw + f) * dt;
    w = (wn * w <= 0.0) ? 0.0 : wn;                  // ma sat khong lam doi chieu
  } else {
    w += dw * dt;
  }
  pos += encSignHw * w / 60.0 * CPR * dt;
  long idx = (long)std::floor(pos);
  static const int GRAY[4] = {0, 1, 3, 2};          // trang thai (A<<1)|B theo chieu tang
  while (posIdx != idx) {
    posIdx += (idx > posIdx) ? 1 : -1;
    int s = GRAY[((posIdx % 4) + 4) % 4];
    int a = (s >> 1) & 1, b = s & 1;
    if (level[P_ENC_A] != a) setPin(P_ENC_A, a);
    if (level[P_ENC_B] != b) setPin(P_ENC_B, b);
  }
}
}  // namespace

// ---------------------------------------------------------------- API cho Arduino.h
uint32_t sim_micros() { return (uint32_t)(int64_t)(simT * 1e6); }
int64_t esp_timer_get_time() { return (int64_t)(simT * 1e6); }
int esp_timer_create(const esp_timer_create_args_t *a, esp_timer_handle_t *out) { timerCb = a->callback; *out = nullptr; return 0; }
int esp_timer_start_periodic(esp_timer_handle_t, uint64_t p) { timerPeriod = p * 1e-6; timerNext = simT + timerPeriod; return 0; }
uint32_t sim_reg_read(int reg) {
  if (reg == GPIO_IN1_REG) {
    uint32_t v = 0;
    for (int p = 32; p < 40; p++) if (level[p]) v |= 1u << (p - 32);
    return v;
  }
  return 0;
}
void sim_reg_write(int reg, uint32_t v) {
  if (reg == GPIO_OUT_W1TS_REG) outReg |= v;
  else if (reg == GPIO_OUT_W1TC_REG) outReg &= ~v;
}
void sim_serial_write(const char *s, size_t n) { fwrite(s, 1, n, stdout); fflush(stdout); }
int sim_serial_available() { return (int)rx.size(); }
int sim_serial_read() { if (rx.empty()) return -1; char c = rx.front(); rx.pop_front(); return (unsigned char)c; }
int sim_digital_read(int pin) { return level[pin]; }
void sim_attach_interrupt(int pin, void (*fn)(), int mode) { isrFn[pin] = fn; isrMode[pin] = mode; }
void sim_ledc_write(int duty) { pwmDuty = duty; }

void sim_service() {
  double now = realNow();
  while (simT < now) {
    double dt = std::min(DT, now - simT);
    simT += dt;
    for (size_t i = 0; i < events.size();) {
      if (events[i].t <= simT) { setPin(events[i].pin, events[i].lvl); events.erase(events.begin() + i); }
      else i++;
    }
    physicsStep(dt);
    if (timerCb && simT >= timerNext) { timerNext += timerPeriod; timerCb(nullptr); }
  }
}

int main(int argc, char **argv) {
  for (int i = 1; i + 1 < argc; i++) if (std::string(argv[i]) == "--enc-sign") encSignHw = atoi(argv[i + 1]) < 0 ? -1 : 1;
  fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);
  for (int &l : level) l = 1;                       // keo len: nut nha = HIGH
  level[P_ENC_A] = 0; level[P_ENC_B] = 0;
  fprintf(stdout, "#SIM harness: phan cung GIA LAP, so lieu chi de chay thu quy trinh\n");
  fflush(stdout);
  setup();
  while (!quit) {
    pollStdin();
    sim_service();
    loop();
  }
  return 0;
}
