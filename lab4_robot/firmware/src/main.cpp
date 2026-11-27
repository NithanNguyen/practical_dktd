/*
 * Lab 4 CE212 - Nhanh Robot (thuc hanh ve nha)
 * Dao chieu + dung khan cap (E-STOP) dong co DC theo nut nhan, do thoi gian dap ung dung khan.
 *
 * Phan cung: giu nguyen mach Lab 3, them 2 nut nhan (nut noi chan GPIO voi GND, keo len noi).
 *   ESP32-DevKitC-32D -> TB6612FNG: IO25=PWMA, IO26=AIN1, IO27=AIN2, 3V3=VCC+STBY, GND=GND
 *   TB6612 AO1/AO2    -> dong co (day do M+ / day trang M-)
 *   Encoder: xanh duong=3V3, den=GND, vang(A)=IO32, xanh la(B)=IO33
 *   Adapter 12V       -> TB6612 VM / GND (GND chung voi ESP32)
 *   Nut DAO CHIEU     -> IO18 va GND
 *   Nut DUNG KHAN     -> IO19 va GND
 *
 * Bang chan ly TB6612FNG (datasheet Toshiba, "H-SW Control Function"):
 *   IN1 IN2 PWM STBY -> che do
 *    H   H  H/L  H   -> short brake (phanh ngan: 2 dau ra noi GND)
 *    L   H   H   H   -> CCW            L H L H -> short brake
 *    H   L   H   H   -> CW             H L L H -> short brake
 *    L   L   H   H   -> stop (dau ra tro khang cao: dong co quay tu do)
 *
 * Cach dung nut (khong can may tinh):
 *   - DAO CHIEU khi dang dung  : chay theo chieu hien tai, duty tang dan (ramp).
 *   - DAO CHIEU khi dang chay  : phanh ngan toi khi truc dung, doi chieu, ramp len lai.
 *   - DUNG KHAN                : ngat ISR ghi ngay IN1/IN2 (phanh ngan hoac tha tu do) va CHOT.
 *   - Giu DAO CHIEU >= 1 s      : xoa chot dung khan (chi khi nut dung khan da nha). Dong co
 *                                 KHONG tu chay lai, phai nhan DAO CHIEU them 1 lan nua.
 *
 * Do thoi gian dung khan:
 *   t_press = thoi diem vao ISR cua nut dung khan (esp_timer, 1 us).
 *   Encoder ISR cap nhat t_move moi khi truc dich chuyen >= 2 dem so voi moc truoc
 *   (bo qua rung +-1 dem cua encoder khi truc dung yen).
 *   Truc duoc coi la da dung khi khong co dich chuyen trong STOP_QUIET_MS.
 *   dt = t_move_cuoi - t_press. Goc quay them = so dem / cpr * 360.
 *   Ngoai ra firmware ghi (thoi gian, so dem) moi 1 ms vao bo dem vong va gui ra sau moi su kien
 *   de PC ve duong toc do va tinh t_10% (xem tools/lab4_analysis.py).
 *
 * Serial 115200 baud. Moi 10 ms mot dong CSV: t_ms,state,dir,duty,rpm,count
 *   state: 0 = dung, 1 = chay, 2 = dang dao chieu (pha phanh), 3 = dung khan (da chot)
 * Dong bat dau bang '#' la thong bao/ket qua. Go "help" de xem lenh.
 */
#include <Arduino.h>
#include "esp_timer.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

#define FW_VERSION "lab4_robot v1.0"

// ----------------------------- Chan ket noi -----------------------------
constexpr int PIN_PWMA      = 25;
constexpr int PIN_AIN1      = 26;
constexpr int PIN_AIN2      = 27;
constexpr int PIN_ENC_A     = 32;
constexpr int PIN_ENC_B     = 33;
constexpr int PIN_BTN_DIR   = 18;
constexpr int PIN_BTN_ESTOP = 19;

static_assert(PIN_AIN1 < 32 && PIN_AIN2 < 32, "AIN1/AIN2 phai < GPIO32 de ghi bang GPIO_OUT_W1TS_REG");
static_assert(PIN_ENC_A >= 32 && PIN_ENC_B >= 32, "encoder doc bang GPIO_IN1_REG (GPIO32..39)");
constexpr uint32_t MASK_AIN1 = 1UL << PIN_AIN1;
constexpr uint32_t MASK_AIN2 = 1UL << PIN_AIN2;

// ----------------------------- PWM --------------------------------------
constexpr uint32_t PWM_FREQ = 20000;            // 20 kHz (TB6612 cho phep toi 100 kHz)
constexpr uint8_t  PWM_BITS = 10;
constexpr int      PWM_MAX  = (1 << PWM_BITS) - 1;
constexpr int      PWM_CH   = 0;                // chi dung cho Arduino core 2.x

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static void pwmInit()       { ledcAttach(PIN_PWMA, PWM_FREQ, PWM_BITS); }
static void pwmWrite(int d) { ledcWrite(PIN_PWMA, d); }
#else
static void pwmInit()       { ledcSetup(PWM_CH, PWM_FREQ, PWM_BITS); ledcAttachPin(PIN_PWMA, PWM_CH); }
static void pwmWrite(int d) { ledcWrite(PWM_CH, d); }
#endif
static int pctToPwm(float pct) { return (int)lroundf(constrain(pct, 0.0f, 100.0f) * PWM_MAX / 100.0f); }

// ----------------------------- Hang so thoi gian ------------------------
constexpr uint32_t TS_US            = 10000;    // chu ky dieu khien + gui du lieu 10 ms
constexpr float    TS               = TS_US * 1e-6f;
constexpr int      VEL_WIN          = 4;        // toc do = hieu so dem tren 4 chu ky (40 ms), nhu Lab 3
constexpr uint32_t STOP_QUIET_US    = 150000;   // khong dich chuyen 150 ms -> coi la da dung
constexpr uint32_t ESTOP_TIMEOUT_US = 5000000;  // dung khan qua 5 s chua dung -> bao timeout
constexpr uint32_t REV_QUIET_US     = 30000;    // pha phanh cua dao chieu: dung 30 ms -> doi chieu
constexpr uint32_t REV_BRAKE_MAX_US = 2000000;
constexpr uint32_t REV_TRACK_MAX_US = 3000000;
constexpr uint32_t DEBOUNCE_US      = 20000;
constexpr uint32_t LONG_PRESS_US    = 1000000;
constexpr uint32_t RELEASE_OK_US    = 50000;    // nut dung khan phai nha on dinh 50 ms moi duoc reset

static inline uint32_t IRAM_ATTR nowUs() { return (uint32_t)esp_timer_get_time(); }
static inline bool after(uint32_t a, uint32_t b) { return (int32_t)(a - b) > 0; }   // a sau b (an toan khi tran)

// ----------------------------- Encoder (ngat, giai ma x4) ---------------
volatile int32_t  encCount   = 0;
volatile uint32_t encErrors  = 0;
volatile uint8_t  encState   = 0;
volatile int32_t  moveAnchor = 0;
volatile uint32_t tMove      = 0;   // lan cuoi truc dich chuyen >= 2 dem so voi moc

DRAM_ATTR static const int8_t QDEC[16] = {
   0, +1, -1,  0,
  -1,  0,  0, +1,
  +1,  0,  0, -1,
   0, -1, +1,  0 };

static inline uint8_t IRAM_ATTR readAB() {
  uint32_t v = REG_READ(GPIO_IN1_REG);
  return (uint8_t)((((v >> (PIN_ENC_A - 32)) & 1U) << 1) | ((v >> (PIN_ENC_B - 32)) & 1U));
}

void IRAM_ATTR encISR() {
  uint8_t s = readAB();
  uint8_t p = encState;
  if ((p ^ s) == 3) encErrors++;            // A va B cung doi: mat mot canh
  int32_t c = encCount + QDEC[(p << 2) | s];
  encCount = c;
  encState = s;
  int32_t d = c - moveAnchor;
  if (d >= 2 || d <= -2) { moveAnchor = c; tMove = nowUs(); }
}

// ----------------------------- Bo dem chup 1 kHz ------------------------
// esp_timer goi capCb moi 1 ms: ghi (thoi gian, so dem). Khi co su kien thi giu lai
// CAP_PRE mau truoc va toi da CAP_POST_MAX mau sau, roi gui ra Serial.
constexpr uint32_t CAP_N        = 2048;     // luy thua cua 2
constexpr uint32_t CAP_PRE      = 100;
constexpr uint32_t CAP_POST_MAX = 1900;
static_assert(CAP_PRE + CAP_POST_MAX <= CAP_N, "bo dem chup qua nho");
struct CapSample { uint32_t t; int32_t c; };
static CapSample capBuf[CAP_N];
volatile uint32_t capW      = 0;      // tong so mau da ghi
volatile uint32_t capTrig   = 0;      // chi so mau tai thoi diem su kien
volatile bool     capArmed  = false;
volatile bool     capFrozen = false;
volatile bool     capDumping = false; // dang gui bo dem ra Serial: khong duoc ghi de
volatile bool     capLate    = false; // dung khan xay ra trong luc gui: chup bu sau khi gui xong
bool capEnabled = true;

static void capCb(void *) {
  if (capFrozen) return;
  uint32_t w = capW;
  capBuf[w & (CAP_N - 1)].t = nowUs();
  capBuf[w & (CAP_N - 1)].c = encCount;
  capW = w + 1;
  if (capArmed && (w + 1 - capTrig) >= CAP_POST_MAX) capFrozen = true;
}

// ----------------------------- Trang thai chung -------------------------
enum State : uint8_t { ST_IDLE = 0, ST_RUN = 1, ST_REV = 2, ST_ESTOP = 3 };
enum StopMode : uint8_t { STOP_BRAKE = 0, STOP_COAST = 1 };
enum Src : uint8_t { SRC_BTN = 0, SRC_SOFT = 1, SRC_POLL = 2 };
static const char *srcName(uint8_t s) { return s == SRC_BTN ? "btn" : (s == SRC_SOFT ? "soft" : "poll"); }
static const char *modeName(uint8_t m) { return m == STOP_BRAKE ? "brake" : "coast"; }

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool     latched      = false;   // dung khan da chot: chi ISR/estopTrigger dat, chi tryReset xoa
volatile bool     estopPending = false;   // su kien dung khan chua do xong
volatile uint8_t  estopMode    = STOP_BRAKE;
volatile uint32_t eT = 0;                 // t_press (us)
volatile int32_t  eC = 0;                 // so dem tai t_press
volatile int32_t  eAnchor = 0;            // moc moveAnchor tai t_press (PC dung de tinh lai t_dung y het)
volatile uint8_t  eSrc = SRC_BTN, eMode = STOP_BRAKE;

// Dua cau H ve trang thai dung (goi duoc trong ISR): chi ghi IN1/IN2 bang thanh ghi.
static inline void IRAM_ATTR outStopPins(uint8_t m) {
  if (m == STOP_BRAKE) REG_WRITE(GPIO_OUT_W1TS_REG, MASK_AIN1 | MASK_AIN2);   // H,H: short brake
  else                 REG_WRITE(GPIO_OUT_W1TC_REG, MASK_AIN1 | MASK_AIN2);   // L,L: stop (tha tu do)
}

static void IRAM_ATTR estopTrigger(uint8_t src) {
  uint32_t t = nowUs();
  portENTER_CRITICAL_SAFE(&mux);
  if (!latched) {
    uint8_t m = estopMode;
    outStopPins(m);                         // hanh dong truoc, ghi so lieu sau
    latched = true;
    eT = t; eC = encCount; eAnchor = moveAnchor; eSrc = src; eMode = m;
    if (capDumping) capLate = true;
    else { capTrig = capW; capArmed = true; capFrozen = false; }
    estopPending = true;
  }
  portEXIT_CRITICAL_SAFE(&mux);
}

void IRAM_ATTR estopISR() { estopTrigger(SRC_BTN); }

// Ghi dau ra o trang thai binh thuong. Bi chan neu dung khan da chot (ISR co the chen vao bat ky luc nao).
static bool setOutputs(uint32_t setMask, uint32_t clrMask, int pwm) {
  bool ok;
  portENTER_CRITICAL(&mux);
  ok = !latched;
  if (ok) {
    REG_WRITE(GPIO_OUT_W1TC_REG, clrMask);
    REG_WRITE(GPIO_OUT_W1TS_REG, setMask);
  }
  portEXIT_CRITICAL(&mux);
  if (ok) pwmWrite(pwm);
  return ok;
}
static void outIdle()            { setOutputs(0, MASK_AIN1 | MASK_AIN2, PWM_MAX); }   // L,L,PWM=H: stop
static void outBrake()           { setOutputs(MASK_AIN1 | MASK_AIN2, 0, 0); }         // H,H: short brake
static void outDrive(int d, float pct) {
  if (d > 0) setOutputs(MASK_AIN1, MASK_AIN2, pctToPwm(pct));   // H,L: CW
  else       setOutputs(MASK_AIN2, MASK_AIN1, pctToPwm(pct));   // L,H: CCW
}

// ----------------------------- Tham so (doi duoc qua Serial) ------------
float cprOut   = 1978.7f;   // dem/vong truc ra (x4), hieu chuan o Lab 3 (lab3_config.json)
int   encSign  = +1;        // doi -1 neu dir=+1 ma rpm do duoc am
float target   = 60.0f;     // duty dat (%)
float rampRate = 400.0f;    // toc do tang duty (%/s): 0 -> 100 % trong 0,25 s
bool  streamOn = true;

// ----------------------------- Trang thai -------------------------------
State    st     = ST_IDLE;
int      dir    = +1;
float    duty   = 0.0f;     // duty dang ap (sau ramp)
float    rpm    = 0.0f;
int32_t  hist[VEL_WIN + 1];
uint32_t histT[VEL_WIN + 1];
int      histIdx = 0;
uint32_t tNext = 0;
uint32_t overruns = 0;

uint32_t estopId = 0;
float    eRpm0 = 0.0f, eDuty0 = 0.0f;
bool     eSeen = false;               // loop da nhan su kien dung khan hien tai

uint32_t revId = 0;
uint32_t rT = 0, rTZero = 0;
float    rRpm0 = 0.0f;
uint8_t  rSrc = SRC_BTN;
bool     rTracking = false;           // dang cho dat 90 % toc do theo chieu moi

// Yeu cau gui bo dem chup
bool        dumpReq = false;
const char *dumpKind = "";
uint32_t    dumpId = 0, dumpRef = 0;

// ----------------------------- Nut nhan --------------------------------
struct Button {
  int      pin;
  bool     raw, stable;              // true = dang nhan (muc LOW)
  uint32_t tRawChange, tFirstEdge, tDown;
  bool     longFired, edgeArmed;
};
Button bDir   = {PIN_BTN_DIR,   false, false, 0, 0, 0, false, false};
Button bEstop = {PIN_BTN_ESTOP, false, false, 0, 0, 0, false, false};

// Tra ve +1 vua nhan, -1 vua nha, 0 khong doi.
// tFirstEdge = lan dau doc thay LOW ke tu khi nut o trang thai nha (truoc khi het doi 20 ms),
// dung lam moc thoi gian nhan cho phep do dao chieu.
static int pollButton(Button &b, uint32_t now) {
  bool r = (digitalRead(b.pin) == LOW);
  if (r != b.raw) {
    b.raw = r;
    b.tRawChange = now;
    if (r && !b.stable && !b.edgeArmed) { b.tFirstEdge = now; b.edgeArmed = true; }
  }
  if ((int32_t)(now - b.tRawChange) >= (int32_t)DEBOUNCE_US) {
    if (b.raw != b.stable) {
      b.stable = b.raw;
      if (b.stable) { b.tDown = now; b.longFired = false; return +1; }
      b.edgeArmed = false;
      return -1;
    }
    if (!b.raw && !b.stable) b.edgeArmed = false;   // xung nhieu ngan: bo moc canh
  }
  return 0;
}

static bool estopReleased(uint32_t now) {
  return !bEstop.raw && !bEstop.stable && (int32_t)(now - bEstop.tRawChange) >= (int32_t)RELEASE_OK_US;
}

// ----------------------------- Hanh dong --------------------------------
static void startRun(const char *src) {
  uint32_t now = nowUs();
  if (st != ST_IDLE) { Serial.printf("#ERR run: chi chay duoc khi dang dung (state=%d)\n", (int)st); return; }
  if (latched || !estopReleased(now)) { Serial.println("#ERR run: nut DUNG KHAN dang nhan hoac chua reset"); return; }
  duty = 0.0f;
  rTracking = false;
  st = ST_RUN;
  Serial.printf("#EV RUN src=%s dir=%d target=%.1f\n", src, dir, target);
}

static void triggerReverse(uint8_t src, uint32_t tPress) {
  if (st != ST_RUN) {
    Serial.printf("#ERR rev: chi dao chieu duoc khi dang chay (state=%d)\n", (int)st);
    return;
  }
  revId++;
  rT = tPress; rRpm0 = rpm; rSrc = src; rTracking = false;
  st = ST_REV;
  outBrake();
  if (capEnabled && !estopPending) {
    uint32_t back = (nowUs() - tPress) / 1000U;               // canh nhan som hon luc xac nhan
    if (back > CAP_PRE) back = CAP_PRE;
    portENTER_CRITICAL(&mux);
    capTrig = capW - back; capArmed = true; capFrozen = false;
    portEXIT_CRITICAL(&mux);
  }
  Serial.printf("#EV REV id=%lu src=%s from=%d rpm0=%.2f\n", (unsigned long)revId, srcName(src), dir, rRpm0);
}

static bool tryReset(const char *src) {
  uint32_t now = nowUs();
  if (!latched) { Serial.println("#ERR reset: khong o trang thai dung khan"); return false; }
  if (estopPending) { Serial.println("#ERR reset: dang do, cho truc dung han"); return false; }
  if (!estopReleased(now)) { Serial.println("#ERR reset: hay NHA nut DUNG KHAN truoc"); return false; }
  portENTER_CRITICAL(&mux);
  latched = false;
  portEXIT_CRITICAL(&mux);
  st = ST_IDLE; duty = 0.0f;
  outIdle();
  Serial.printf("#OK RESET src=%s (dong co dung, nhan DAO CHIEU hoac go run de chay)\n", src);
  return true;
}

// ----------------------------- Lenh Serial ------------------------------
static void printStatus() {
  Serial.printf("#ST state=%d dir=%d target=%.1f duty=%.1f rpm=%.2f mode=%s ramp=%.0f cpr=%.1f sign=%d "
                "latched=%d pend=%d btn_dir=%d btn_estop=%d enc_err=%lu overrun=%lu cap=%d\n",
                (int)st, dir, target, duty, rpm, modeName(estopMode), rampRate, cprOut, encSign,
                (int)latched, (int)estopPending, (int)bDir.stable, (int)bEstop.stable,
                (unsigned long)encErrors, (unsigned long)overruns, (int)capEnabled);
}

static void printHelp() {
  Serial.println("# Nut: DAO CHIEU (IO18) chay/dao chieu, giu 1 s de reset; DUNG KHAN (IO19)");
  Serial.println("# Lenh: run [duty] [dir] | duty <%> | dir <1|-1> | rev | estop | reset | stop");
  Serial.println("#       mode <brake|coast> | ramp <%/s> | cpr <v> | sign <1|-1>");
  Serial.println("#       stream <0|1> | cap <0|1> | ping | ? | help");
}

static void handleCommand(char *line) {
  char *cmd = strtok(line, " \t");
  if (!cmd) return;
  char *a1 = strtok(nullptr, " \t");
  char *a2 = strtok(nullptr, " \t");
  String c(cmd); c.toLowerCase();
  float v = a1 ? atof(a1) : 0.0f;

  if (c == "run") {
    if (a1) target = constrain(v, 0.0f, 100.0f);
    if (a2 && st == ST_IDLE) dir = (atof(a2) < 0) ? -1 : +1;
    startRun("soft");
  }
  else if (c == "duty" && a1) { target = constrain(v, 0.0f, 100.0f); }
  else if (c == "dir" && a1) {
    if (st != ST_IDLE) { Serial.println("#ERR dir: chi doi khi dang dung (dang chay thi dung rev)"); return; }
    dir = (v < 0) ? -1 : +1;
  }
  else if (c == "rev")   { triggerReverse(SRC_SOFT, nowUs()); return; }
  else if (c == "estop") { estopTrigger(SRC_SOFT); return; }
  else if (c == "reset") { tryReset("soft"); return; }
  else if (c == "stop") {
    if (st == ST_RUN || st == ST_REV) { st = ST_IDLE; duty = 0.0f; rTracking = false; outIdle(); }
  }
  else if (c == "mode" && a1) {
    String m(a1); m.toLowerCase();
    if (m == "brake" || m == "0") estopMode = STOP_BRAKE;
    else if (m == "coast" || m == "1") estopMode = STOP_COAST;
    else { Serial.println("#ERR mode: brake | coast"); return; }
  }
  else if (c == "ramp" && a1)   { if (v > 0) rampRate = v; }
  else if (c == "cpr" && a1)    { if (v > 0) cprOut = v; }
  else if (c == "sign" && a1)   { encSign = (v < 0) ? -1 : +1; }
  else if (c == "stream" && a1) { streamOn = (v != 0.0f); }
  else if (c == "cap" && a1)    { capEnabled = (v != 0.0f); }
  else if (c == "ping")         { Serial.println("#PONG " FW_VERSION); return; }
  else if (c == "?")            { printStatus(); return; }
  else if (c == "help")         { printHelp(); return; }
  else { Serial.printf("#ERR lenh khong hop le: %s (go help)\n", cmd); return; }
  printStatus();
}

static void pollSerial() {
  static char buf[64];
  static uint8_t n = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') { buf[n] = '\0'; if (n) handleCommand(buf); n = 0; }
    else if (n < sizeof(buf) - 1) buf[n++] = ch;
  }
}

// ----------------------------- Xu ly nut --------------------------------
static void handleButtons(uint32_t now) {
  int e = pollButton(bEstop, now);
  if (e > 0) {
    Serial.println("#BTN ESTOP DOWN");
    if (!latched) estopTrigger(SRC_POLL);   // du phong neu lo canh ngat
  } else if (e < 0) {
    Serial.println("#BTN ESTOP UP");
  }

  int d = pollButton(bDir, now);
  if (d > 0) {
    Serial.println("#BTN DIR DOWN");
    if (st == ST_IDLE)     startRun("btn");
    else if (st == ST_RUN) triggerReverse(SRC_BTN, bDir.tFirstEdge);
    else if (st == ST_ESTOP) Serial.println("# Dang dung khan: nha nut DUNG KHAN roi GIU nut DAO CHIEU 1 s de reset");
  } else if (d < 0) {
    Serial.println("#BTN DIR UP");
  }
  if (bDir.stable && !bDir.longFired && st == ST_ESTOP && (int32_t)(now - bDir.tDown) >= (int32_t)LONG_PRESS_US) {
    bDir.longFired = true;
    tryReset("btn");
  }
}

// ----------------------------- Xu ly dung khan --------------------------
static void handleEstop(uint32_t now) {
  if (latched && !eSeen) {          // lan dau loop thay su kien
    eSeen = true;
    estopId++;
    eRpm0 = rpm; eDuty0 = duty;
    st = ST_ESTOP; duty = 0.0f; rTracking = false;
    pwmWrite(eMode == STOP_COAST ? PWM_MAX : 0);   // coast: dung dung hang "L L H H -> stop" cua bang chan ly
    Serial.printf("#EV ESTOP id=%lu src=%s mode=%s rpm0=%.2f duty0=%.1f\n",
                  (unsigned long)estopId, srcName(eSrc), modeName(eMode), eRpm0, eDuty0);
  }
  if (!latched) eSeen = false;

  if (estopPending && eSeen) {
    uint32_t tp = eT;
    uint32_t tm = tMove;
    uint32_t last = after(tm, tp) ? tm : tp;
    bool quiet = (int32_t)(now - last) >= (int32_t)STOP_QUIET_US;
    bool tout  = (int32_t)(now - tp) >= (int32_t)ESTOP_TIMEOUT_US;
    if (quiet || tout) {
      int32_t counts = encSign * (encCount - eC);
      float dtMs = (last - tp) / 1000.0f;
      float deg = counts / cprOut * 360.0f;
      estopPending = false;
      Serial.printf("#RES ESTOP id=%lu src=%s mode=%s rpm0=%.2f duty0=%.1f dt_ms=%.3f counts=%ld deg=%.2f "
                    "timeout=%d c0=%ld anc=%ld\n",
                    (unsigned long)estopId, srcName(eSrc), modeName(eMode), eRpm0, eDuty0, dtMs,
                    (long)counts, deg, (int)(!quiet), (long)(encSign * eC), (long)(encSign * eAnchor));
      Serial.printf("# >>> DUNG KHAN #%lu (%s): truc dung sau %.1f ms, tu %.1f rpm, quay them %.1f do\n",
                    (unsigned long)estopId, modeName(eMode), dtMs, eRpm0, deg);
      if (capEnabled) { dumpReq = true; dumpKind = "ESTOP"; dumpId = estopId; dumpRef = tp; }
    }
  }
}

// ----------------------------- Gui bo dem chup ---------------------------
static void doDump() {
  dumpReq = false;
  capDumping = true;
  capFrozen = true;
  delay(3);                          // cho capCb (task esp_timer, core 0) ghi xong mau dang do
  uint32_t w = capW, trig = capTrig;
  uint32_t start = (trig >= CAP_PRE) ? trig - CAP_PRE : 0;
  if (w - start > CAP_N) start = w - CAP_N;
  Serial.printf("#CAP BEGIN kind=%s id=%lu tref=%lu n=%lu\n", dumpKind, (unsigned long)dumpId,
                (unsigned long)dumpRef, (unsigned long)(w - start));
  for (uint32_t i = start; i < w; i++) {
    const CapSample &s = capBuf[i & (CAP_N - 1)];
    Serial.printf("#C,%ld,%ld\n", (long)(int32_t)(s.t - dumpRef), (long)(encSign * s.c));
  }
  Serial.printf("#CAP END kind=%s id=%lu\n", dumpKind, (unsigned long)dumpId);
  portENTER_CRITICAL(&mux);
  capDumping = false;
  capArmed = false;
  if (capLate) {                     // dung khan trong luc gui: chup tu bay gio (thieu doan dau)
    capLate = false;
    capTrig = capW; capArmed = true;
  }
  capFrozen = false;
  portEXIT_CRITICAL(&mux);
}

// ----------------------------- Chu ky 10 ms -----------------------------
static void controlStep(uint32_t now) {
  // Toc do = so dem / thoi gian THUC cua cua so (khong gia dinh dung 40 ms: neu mot chu ky
  // bi tre, chia cho 40 ms se cho toc do sai).
  int32_t c = encCount;
  histIdx = (histIdx + 1) % (VEL_WIN + 1);
  hist[histIdx] = c;
  histT[histIdx] = now;
  int oldIdx = (histIdx + 1) % (VEL_WIN + 1);
  uint32_t dtWin = now - histT[oldIdx];
  rpm = (dtWin > 0) ? encSign * (float)(c - hist[oldIdx]) / cprOut / (dtWin * 1e-6f) * 60.0f : 0.0f;

  switch (st) {
    case ST_IDLE:
      duty = 0.0f;
      outIdle();
      break;
    case ST_RUN: {
      float step = rampRate * TS;
      if (duty < target) duty = min(duty + step, target);
      else               duty = max(duty - step, target);
      outDrive(dir, duty);
      if (rTracking) {
        // dat 90 % toc do theo chieu NGUOC voi toc do truoc khi dao (khong phu thuoc sign hieu chuan)
        float opp = (rRpm0 >= 0.0f) ? -rpm : rpm;
        bool reached = opp >= 0.9f * fabsf(rRpm0);
        bool tout = (int32_t)(now - rT) >= (int32_t)REV_TRACK_MAX_US;
        if (reached || tout) {
          rTracking = false;
          float tz = (rTZero - rT) / 1000.0f;
          float t90 = reached ? (now - rT) / 1000.0f : -1.0f;
          Serial.printf("#RES REV id=%lu src=%s rpm0=%.2f t_zero_ms=%.3f t90_ms=%.1f rpm1=%.2f timeout=%d\n",
                        (unsigned long)revId, srcName(rSrc), rRpm0, tz, t90, rpm, (int)(!reached));
          Serial.printf("# >>> DAO CHIEU #%lu: dung sau %.1f ms, dat 90%% toc do chieu moi sau %.0f ms\n",
                        (unsigned long)revId, tz, t90);
          if (capEnabled && !estopPending) { dumpReq = true; dumpKind = "REV"; dumpId = revId; dumpRef = rT; }
        }
      }
      break;
    }
    case ST_REV: {
      outBrake();
      uint32_t tm = tMove;
      uint32_t last = after(tm, rT) ? tm : rT;
      bool stopped = (int32_t)(now - last) >= (int32_t)REV_QUIET_US;
      bool tout = (int32_t)(now - rT) >= (int32_t)REV_BRAKE_MAX_US;
      if (stopped || tout) {
        rTZero = last;
        dir = -dir;
        duty = 0.0f;
        rTracking = true;
        st = ST_RUN;
        Serial.printf("#EV REV_ZERO id=%lu t_zero_ms=%.3f dir=%d\n", (unsigned long)revId, (last - rT) / 1000.0f, dir);
      }
      break;
    }
    case ST_ESTOP:
      break;                         // dau ra da duoc ISR dat; khong ghi lai
  }

  if (streamOn) {
    Serial.printf("%lu,%d,%d,%.1f,%.2f,%ld\n", (unsigned long)millis(), (int)st, dir, duty, rpm, (long)(encSign * c));
  }
}

// ----------------------------- setup / loop -----------------------------
void setup() {
  Serial.setTxBufferSize(4096);      // gui bo dem chup it bi chan hon (goi truoc begin)
  Serial.begin(115200);

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  REG_WRITE(GPIO_OUT_W1TC_REG, MASK_AIN1 | MASK_AIN2);
  pwmInit();
  pwmWrite(PWM_MAX);                 // L,L,H: stop

  pinMode(PIN_BTN_DIR, INPUT_PULLUP);
  pinMode(PIN_BTN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  encState = readAB();
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_ESTOP), estopISR, FALLING);

  for (int i = 0; i <= VEL_WIN; i++) { hist[i] = 0; histT[i] = nowUs() - (VEL_WIN - i) * TS_US; }

  esp_timer_create_args_t ta = {};
  ta.callback = &capCb;
  ta.name = "cap1k";
  esp_timer_handle_t th;
  esp_timer_create(&ta, &th);
  esp_timer_start_periodic(th, 1000);

  delay(200);
  uint32_t now = nowUs();
  bDir.raw = bDir.stable = (digitalRead(PIN_BTN_DIR) == LOW);
  bEstop.raw = bEstop.stable = (digitalRead(PIN_BTN_ESTOP) == LOW);
  bDir.tRawChange = bEstop.tRawChange = now;
  tMove = now;
  Serial.println("#BOOT " FW_VERSION " - go help de xem lenh");
  if (bEstop.stable) {               // khoi dong khi nut dung khan dang nhan: chot luon, khong do
    portENTER_CRITICAL(&mux);
    outStopPins(STOP_BRAKE);
    latched = true;
    portEXIT_CRITICAL(&mux);
    eSeen = true; st = ST_ESTOP;
    Serial.println("# Nut DUNG KHAN dang nhan luc khoi dong: dang chot. Nha nut roi giu DAO CHIEU 1 s.");
  }
  printStatus();
  tNext = nowUs() + TS_US;
}

void loop() {
  pollSerial();
  uint32_t now = nowUs();
  handleButtons(now);
  handleEstop(now);
  now = nowUs();
  if ((int32_t)(now - tNext) >= 0) {
    if ((int32_t)(now - tNext) >= (int32_t)TS_US) { overruns++; tNext = now; }
    tNext += TS_US;
    controlStep(now);
  }
  if (dumpReq) doDump();
}
