/* EMG Claw Lite -- Arduino Uno/Nano + servo.
 * Receive "G,0" through "G,100" followed by '\n' at 115200 baud.
 * Independently open claw if communication stops for 300ms.
 * Servo MUST have its own properly rated external supply, common GND.
 */
#include <Servo.h>
#include <stdlib.h>
#include <string.h>

const byte SERVO_PIN = 9;
const int OPEN_ANGLE = 20;     // Calibrate with the motor unloaded.
const int CLOSED_ANGLE = 100;  // Stop before mechanical hard limits.
const int OPEN_THRESHOLD = 25;
const int CLOSE_THRESHOLD = 45;
const unsigned long SIGNAL_TIMEOUT_MS = 300;
const unsigned long STEP_MS = 15;
const int ANGLE_STEP = 2;

Servo claw;
char line[12];
byte count = 0;
bool discarding = false;
bool gripping = false;
int currentAngle = OPEN_ANGLE;
unsigned long lastValidPacket = 0;
unsigned long lastStep = 0;

void acceptPacket(const char* command) {
  if (strncmp(command, "G,", 2) != 0) return;
  char* endptr = nullptr;
  long level = strtol(command + 2, &endptr, 10);
  if (endptr == command + 2 || *endptr != '\0' || level < 0 || level > 100) return;
  lastValidPacket = millis();
  if (level >= CLOSE_THRESHOLD) gripping = true;
  else if (level <= OPEN_THRESHOLD) gripping = false;
}

void setup() {
  Serial.begin(115200);
  claw.attach(SERVO_PIN);
  claw.write(OPEN_ANGLE);
  lastValidPacket = millis();
}

void loop() {
  while (Serial.available()) {
    const char c = char(Serial.read());
    if (c == '\n') {
      if (!discarding) {
        line[count] = '\0';
        acceptPacket(line);
      }
      count = 0;
      discarding = false;
    } else if (c != '\r' && !discarding) {
      if (count < sizeof(line) - 1) line[count++] = c;
      else discarding = true;
    }
  }

  if (millis() - lastValidPacket > SIGNAL_TIMEOUT_MS) {
    gripping = false;
  }

  const int target = gripping ? CLOSED_ANGLE : OPEN_ANGLE;
  if (millis() - lastStep >= STEP_MS) {
    lastStep = millis();
    if (currentAngle < target) currentAngle = min(currentAngle + ANGLE_STEP, target);
    else if (currentAngle > target) currentAngle = max(currentAngle - ANGLE_STEP, target);
    claw.write(currentAngle);
  }
}
