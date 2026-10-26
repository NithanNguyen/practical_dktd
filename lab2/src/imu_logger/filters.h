// Bo loc dung chung: C++ thuan (khong phu thuoc Arduino) de co the bien dich va
// kiem thu tren may tinh, doi chieu voi ban Python trong scripts/analyze.py.
#pragma once
#include <math.h>

namespace lab2 {

constexpr float RAD2DEG = 57.29577951308232f;

// Goc tu gia toc ke (don vi do). Quy uoc tay phai, cam bien nam ngang -> az ~ +g.
inline float rollFromAcc(float ay, float az)            { return atan2f(ay, az) * RAD2DEG; }
inline float pitchFromAcc(float ax, float ay, float az) {
  return atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
}

// Moving Average N mau, dang tong truot + bo dem vong: O(1) moi mau.
// Khi chua du N mau thi lay trung binh cac mau dang co.
template <int N>
struct MovingAverage {
  static_assert(N > 0, "N must be positive");
  float buf[N] = {0};
  float sum = 0;
  int   idx = 0, count = 0;
  float update(float x) {
    if (count == N) sum -= buf[idx]; else ++count;
    buf[idx] = x;
    sum += x;
    idx = (idx + 1) % N;
    return sum / count;
  }
};

// Complementary Filter (Colton 2007): angle = a*(angle + w*dt) + (1-a)*angle_acc
// Mau dau tien: khoi tao bang goc gia toc ke.
struct Complementary {
  float alpha;
  float angle = 0;
  bool  init = false;
  explicit Complementary(float a) : alpha(a) {}
  float update(float accAngle, float rateDps, float dt) {
    if (!init) { angle = accAngle; init = true; return angle; }
    angle = alpha * (angle + rateDps * dt) + (1.0f - alpha) * accAngle;
    return angle;
  }
};

// Tich phan gyro thuan tuy (doi chung, de thay hien tuong troi).
struct GyroIntegrator {
  float angle = 0;
  bool  init = false;
  float update(float accAngle, float rateDps, float dt) {
    if (!init) { angle = accAngle; init = true; return angle; }
    angle += rateDps * dt;
    return angle;
  }
};

}  // namespace lab2
