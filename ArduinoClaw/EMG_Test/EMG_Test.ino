// EMG 센서 신호 확인용 테스트 스케치
// SIG -> A0, +VS -> 5V, GND -> GND (-VS는 미연결)
// 시리얼 모니터(115200 baud)에서 숫자가 근육 힘줄 때 변하는지 확인

const int EMG_PIN = A0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  int raw = analogRead(EMG_PIN); // 0~1023
  Serial.println(raw);
  delay(100); // 0.1초 간격
}
