// EMG 센서 + 서보 집게, Arduino 단독 동작 (PC 불필요)
// SIG -> A0, 서보 신호선 -> D9
// 전원: 이 보드가 +Vs/GND/-Vs 듀얼 전원 타입이면 9V 배터리 2개 직렬 필요

#include <Servo.h>

const int EMG_PIN = A0;
const int SERVO_PIN = 9;

const int OPEN_ANGLE = 0;
const int CLOSE_ANGLE = 90;

const unsigned long CALIBRATE_RELAX_MS = 3000;
const unsigned long CALIBRATE_FLEX_MS = 3000;

const float SMOOTH_ALPHA = 0.2; // 지수평활 계수 (낮을수록 더 부드러움)
const int CLOSE_THRESHOLD = 45; // 0~100
const int OPEN_THRESHOLD = 25;  // 0~100

Servo claw;
int baseline = 0;
int peak = 0;
bool calibrated = false;
float smoothed = 0;
bool isClosed = false;

int readEMG() {
  return analogRead(EMG_PIN);
}

void calibrate() {
  Serial.println("보정 시작: 팔에 힘을 빼고 가만히 계세요 (3초)");
  unsigned long start = millis();
  long sum = 0;
  int count = 0;
  while (millis() - start < CALIBRATE_RELAX_MS) {
    sum += readEMG();
    count++;
    delay(10);
  }
  baseline = sum / count;
  Serial.print("이완 기준값: ");
  Serial.println(baseline);

  Serial.println("이제 주먹을 꽉 쥐고 3초간 유지하세요");
  start = millis();
  int maxVal = baseline;
  while (millis() - start < CALIBRATE_FLEX_MS) {
    int v = readEMG();
    if (v > maxVal) maxVal = v;
    delay(10);
  }
  peak = maxVal;
  Serial.print("수축 최대값: ");
  Serial.println(peak);

  if (peak - baseline < 10) {
    Serial.println("경고: 신호 변화가 너무 작습니다. 센서/전극/전원 연결을 확인하세요.");
    calibrated = false;
  } else {
    Serial.println("보정 완료. 제어를 시작합니다.");
    calibrated = true;
  }
}

int toPercent(int raw) {
  long range = peak - baseline;
  if (range <= 0) return 0;
  long pct = (long)(raw - baseline) * 100 / range;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (int)pct;
}

void setup() {
  Serial.begin(115200);
  claw.attach(SERVO_PIN);
  claw.write(OPEN_ANGLE);
  delay(500);
  calibrate();
}

void loop() {
  int raw = readEMG();
  smoothed = smoothed + SMOOTH_ALPHA * (raw - smoothed);

  if (!calibrated) {
    Serial.print("raw=");
    Serial.println(raw);
    delay(100);
    return;
  }

  int pct = toPercent((int)smoothed);

  if (!isClosed && pct >= CLOSE_THRESHOLD) {
    isClosed = true;
    claw.write(CLOSE_ANGLE);
  } else if (isClosed && pct <= OPEN_THRESHOLD) {
    isClosed = false;
    claw.write(OPEN_ANGLE);
  }

  Serial.print("raw=");
  Serial.print(raw);
  Serial.print(" pct=");
  Serial.print(pct);
  Serial.print(" state=");
  Serial.println(isClosed ? "CLOSE" : "OPEN");

  delay(50);
}
