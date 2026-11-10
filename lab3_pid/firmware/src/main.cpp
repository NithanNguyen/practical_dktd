/*
 * Lab 3 CE212 - Vong lap PID roi rac cho van toc dong co DC co encoder
 *
 * Phan cung (so do da chot):
 *   ESP32 NodeMCU-32S  ->  TB6612: IO25=PWMA, IO26=AIN1, IO27=AIN2, 3V3=VCC+STBY, GND=GND
 *   TB6612 AO1/AO2     ->  dong co day do (M+) / day trang (M-)
 *   Encoder: xanh duong=3V3, den=GND, vang(A)=IO32, xanh la(B)=IO33
 *   Adapter 12V        ->  TB6612 VM / GND
 *
 * Cong thuc PID (theo lab_3):
 *   u[k] = Kp*e[k] + Ki*Ts*sum(e) + Kd*(e[k]-e[k-1])/Ts ,  Ts = 10 ms
 *
 * Du lieu gui ra Serial (115200 baud), moi 10 ms mot dong CSV:
 *   t_ms,mode,sp,rpm,u,d
 *   mode: 0 = dung, 1 = vong ho (open loop), 2 = vong kin PID
 *   u: lenh cua bo dieu khien (% duty); d: nhieu tai tuong duong (% duty)
 *   Duty thuc su ap vao dong co = u - d (gioi han 0..100).
 *   Ly do: voi dong co DC (bo qua dien cam), mo-men tai T_L tuong duong
 *   mot do sut dien ap R*T_L/Kt o dau vao, nen tru d vao duty mo phong
 *   duoc tai dot ngot mot cach lap lai chinh xac.
 *   Cac dong bat dau bang '#' la thong bao, khong phai du lieu.
 *
 * Lenh nhap qua Serial (ket thuc bang Enter), go "help" de xem danh sach.
 */
#include <Arduino.h>

// ----------------------------- Chan ket noi -----------------------------
constexpr int PIN_PWMA  = 25;
constexpr int PIN_AIN1  = 26;
constexpr int PIN_AIN2  = 27;
constexpr int PIN_ENC_A = 32;
constexpr int PIN_ENC_B = 33;

// ----------------------------- PWM --------------------------------------
constexpr uint32_t PWM_FREQ = 20000;            // 20 kHz (TB6612 cho phep toi 100 kHz)
constexpr uint8_t  PWM_BITS = 10;               // do phan giai 0..1023
constexpr int      PWM_MAX  = (1 << PWM_BITS) - 1;
constexpr int      PWM_CH   = 0;                // chi dung cho Arduino core 2.x

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static void pwmInit()        { ledcAttach(PIN_PWMA, PWM_FREQ, PWM_BITS); }
static void pwmWrite(int d)  { ledcWrite(PIN_PWMA, d); }
#else
static void pwmInit()        { ledcSetup(PWM_CH, PWM_FREQ, PWM_BITS); ledcAttachPin(PIN_PWMA, PWM_CH); }
static void pwmWrite(int d)  { ledcWrite(PWM_CH, d); }
#endif

// ----------------------------- Lay mau ----------------------------------
constexpr uint32_t TS_US   = 10000;             // Ts = 10 ms
constexpr float    TS      = TS_US * 1e-6f;     // giay
constexpr int      WIN_MAX = 10;                // so chu ky toi da cua cua so van toc

// ----------------------------- Encoder (ngat, giai ma x4) ---------------
volatile int32_t  encCount  = 0;   // dem xung tich luy
volatile uint32_t encErrors = 0;   // so lan nhay 2 trang thai (mat canh)
volatile uint8_t  encState  = 0;

// Bang chuyen trang thai: chi so = (trang_thai_cu << 2) | trang_thai_moi, trang_thai = (A<<1)|B
DRAM_ATTR static const int8_t QDEC[16] = {
   0, +1, -1,  0,
  -1,  0,  0, +1,
  +1,  0,  0, -1,
   0, -1, +1,  0 };

void IRAM_ATTR encISR() {
  uint8_t s = (uint8_t)((digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B));
  uint8_t p = encState;
  if ((p ^ s) == 3) encErrors++;           // ca A va B cung doi: da mat mot canh
  encCount += QDEC[(p << 2) | s];
  encState = s;
}

// ----------------------------- Tham so (doi duoc qua Serial) ------------
float Kp = 0.30f, Ki = 2.0f, Kd = 0.0f;   // gia tri khoi dau, CAN TUNE lai
float cprOut   = 1980.0f;   // so dem/vong TRUC RA (x4). 1980 = 44 x 45 chi la uoc doan, PHAI hieu chuan
int   encSign  = +1;        // doi thanh -1 neu toc do do duoc bi am khi dong co quay thuan
int   velWin   = 4;         // cua so tinh van toc: velWin chu ky (4 -> 40 ms)
bool  dOnMeas  = false;     // false: dao ham theo sai so (dung cong thuc lab); true: dao ham theo dau ra do
float uMax     = 100.0f;    // gioi han tren cua lenh (% duty)
constexpr float U_MIN = 0.0f;   // khong cho dao chieu (tranh dong nguoc ve adapter)
bool  streamOn = true;
float distPct  = 0.0f;   // nhieu tai tuong duong (% duty), lenh dist

// ----------------------------- Trang thai -------------------------------
enum Mode : uint8_t { MODE_STOP = 0, MODE_OL = 1, MODE_CL = 2 };
Mode  mode   = MODE_STOP;
float sp     = 0.0f;      // toc do dat (rpm)
float olDuty = 0.0f;      // duty vong ho (%)
float integ  = 0.0f;      // tong sai so sum(e)
float ePrev  = 0.0f;
float rpmPrev = 0.0f;
float rpm    = 0.0f;
float uOut   = 0.0f;
int32_t hist[WIN_MAX + 1];
int   histIdx = 0;
uint32_t tNext = 0;
uint32_t overruns = 0;
int32_t countOffset = 0;

// ----------------------------- Dieu khien dong co -----------------------
static void motorCoast() {               // AIN1=AIN2=L: ngat cau H (stop)
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  pwmWrite(0);
}
static void motorForward(float pct) {   // AIN1=H, AIN2=L: quay thuan, PWM = toc do
  pct = constrain(pct, 0.0f, 100.0f);
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, LOW);
  pwmWrite((int)lroundf(pct * PWM_MAX / 100.0f));
}

static void resetController() { integ = 0.0f; ePrev = 0.0f; rpmPrev = rpm; }

static int32_t readCount() {
  return encCount;   // doc 32-bit can chinh tren Xtensa la nguyen tu
}

// ----------------------------- Lenh Serial ------------------------------
static void printStatus() {
  Serial.printf("# kp=%.4f ki=%.4f kd=%.5f cpr=%.1f sign=%d win=%d dmeas=%d umax=%.1f\n",
                Kp, Ki, Kd, cprOut, encSign, velWin, (int)dOnMeas, uMax);
  Serial.printf("# mode=%d sp=%.2f ol=%.1f dist=%.1f count=%ld enc_err=%lu overrun=%lu\n",
                (int)mode, sp, olDuty, distPct, (long)(readCount() - countOffset),
                (unsigned long)encErrors, (unsigned long)overruns);
}

static void printHelp() {
  Serial.println("# Lenh: kp <v> | ki <v> | kd <v> | sp <rpm> | ol <duty%> | stop");
  Serial.println("#       cpr <dem/vong truc ra> | sign <1|-1> | win <1..10> | dmeas <0|1>");
  Serial.println("#       umax <%> | dist <% duty> | cnt | zero | stream <0|1> | ? (trang thai) | help");
}

static void handleCommand(char *line) {
  char *cmd = strtok(line, " \t");
  if (!cmd) return;
  char *arg = strtok(nullptr, " \t");
  float v = arg ? atof(arg) : 0.0f;
  String c(cmd); c.toLowerCase();

  if      (c == "kp" && arg)    { Kp = v; }
  else if (c == "ki" && arg)    { Ki = v; integ = 0.0f; }
  else if (c == "kd" && arg)    { Kd = v; }
  else if (c == "sp" && arg)    { if (mode != MODE_CL) resetController(); sp = v; mode = MODE_CL; }
  else if (c == "ol" && arg)    { olDuty = constrain(v, 0.0f, 100.0f); mode = MODE_OL; }
  else if (c == "stop")         { mode = MODE_STOP; sp = 0.0f; olDuty = 0.0f; distPct = 0.0f; resetController(); motorCoast(); }
  else if (c == "dist" && arg)  { distPct = constrain(v, 0.0f, 100.0f); }
  else if (c == "cpr" && arg)   { if (v > 0) cprOut = v; }
  else if (c == "sign" && arg)  { encSign = (v < 0) ? -1 : +1; }
  else if (c == "win" && arg)   { velWin = constrain((int)v, 1, WIN_MAX); }
  else if (c == "dmeas" && arg) { dOnMeas = (v != 0.0f); }
  else if (c == "umax" && arg)  { uMax = constrain(v, 0.0f, 100.0f); }
  else if (c == "stream" && arg){ streamOn = (v != 0.0f); }
  else if (c == "cnt")          { Serial.printf("# count=%ld\n", (long)(readCount() - countOffset)); return; }
  else if (c == "zero")         { countOffset = readCount(); Serial.println("# count=0"); return; }
  else if (c == "?")            { printStatus(); return; }
  else if (c == "help")         { printHelp(); return; }
  else { Serial.printf("# Lenh khong hop le: %s (go help)\n", cmd); return; }
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

// ----------------------------- Mot chu ky dieu khien --------------------
static void controlStep() {
  // 1) Do van toc: hieu so dem tren cua so velWin chu ky
  int32_t c = readCount();
  histIdx = (histIdx + 1) % (WIN_MAX + 1);
  hist[histIdx] = c;
  int32_t cOld = hist[(histIdx + (WIN_MAX + 1) - velWin) % (WIN_MAX + 1)];
  rpm = encSign * (float)(c - cOld) / cprOut / (velWin * TS) * 60.0f;

  // 2) Tinh lenh dieu khien
  if (mode == MODE_CL) {
    float e = sp - rpm;
    float integNew = integ + e;
    float dTerm = dOnMeas ? -Kd * (rpm - rpmPrev) / TS
                          :  Kd * (e - ePrev) / TS;
    float uRaw = Kp * e + Ki * TS * integNew + dTerm;
    float u = constrain(uRaw, U_MIN, uMax);
    // Chong bao hoa tich phan (conditional integration): chi cong don khi khong day lenh ra xa them
    bool satHigh = (uRaw > uMax) && (e > 0);
    bool satLow  = (uRaw < U_MIN) && (e < 0);
    if (!satHigh && !satLow) integ = integNew;
    ePrev = e;
    uOut = u;
    motorForward(uOut - distPct);
  } else if (mode == MODE_OL) {
    uOut = olDuty;
    motorForward(uOut - distPct);
  } else {
    uOut = 0.0f;
    motorCoast();
  }
  rpmPrev = rpm;

  // 3) Gui du lieu
  if (streamOn) {
    Serial.printf("%lu,%d,%.2f,%.2f,%.2f,%.1f\n", (unsigned long)millis(), (int)mode, sp, rpm, uOut, distPct);
  }
}

// ----------------------------- setup / loop -----------------------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pwmInit();
  motorCoast();

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  encState = (uint8_t)((digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B));
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encISR, CHANGE);

  for (int i = 0; i <= WIN_MAX; i++) hist[i] = 0;
  delay(200);
  Serial.println("# lab3_pid_motor v1.1 - go help de xem lenh");
  printStatus();
  tNext = micros() + TS_US;
}

void loop() {
  pollSerial();
  uint32_t now = micros();
  if ((int32_t)(now - tNext) >= 0) {
    if ((int32_t)(now - tNext) >= (int32_t)TS_US) { overruns++; tNext = now; }  // bi tre qua 1 chu ky
    tNext += TS_US;
    controlStep();
  }
}
