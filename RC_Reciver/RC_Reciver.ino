/* Receiver for nRF24L01 -> L298N (4 wheels)
   - Expects Telemetry struct packed by the transmitter (j1_x, j1_y, j1_sw, j2_x, j2_y, j2_sw)
   - Uses joystick2 (j2_x, j2_y) to control motion:
       j2_y = forward/back ( -100..100 )
       j2_x = turn left/right ( -100..100 )
   - Differential mixing: left = y + x, right = y - x
   - IMPORTANT: ensure motor IN pins do NOT collide with NRF CE/CSN (D5/D6).
   - Wiring assumptions (edit if you rewire):
       NRF: CE=D8, CSN=D10, MOSI=D11, MISO=D12, SCK=D13
       L298N: IN1=D7, IN2=D6, ENA=D9 (left)
              IN3=D4, IN4=D5, ENB=D3 (right)
*/
/*
  Full sketch: nRF24L01 receiver -> L298N (4 wheels) + 2 servos (A0/A1).
  - Telemetry struct expected from transmitter: j1_x, j1_y, j1_sw, j2_x, j2_y, j2_sw
  - Joystick 1 (j1_x -> servoX on A0, j1_y -> servoY on A1)
    Behavior: when axis > JOY_THRESHOLD, the corresponding servo TARGET increases
              by SERVO_STEP degrees every SERVO_STEP_MS ms.
              when axis < -JOY_THRESHOLD, target decreases similarly.
              when axis within +/- JOY_THRESHOLD, target is NOT changed (held).
  - Joystick 2 (j2_x/j2_y) still drives differential motor mixing as before.
  - JOY2 button (j2_sw) output mirrored on D2.
  - NOTE: servo motors are attached to A0/A1. If servos draw too much current,
          use a separate 5V supply with common ground.
*/

// Libraries
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
// ----------------- RADIO -----------------
RF24 radio(8, 10);
const uint64_t pipe = 0xE8E8F0F0E1LL;

// ----------------- MOTOR PINS -----------------
const uint8_t IN1_PIN = 7;
const uint8_t IN2_PIN = 6;
const uint8_t ENA_PIN = 5;

const uint8_t IN3_PIN = 4;
const uint8_t IN4_PIN = 9;
const uint8_t ENB_PIN = 3;

// ----------------- SAFETY / TUNING -----------------
const unsigned long TIMEOUT_MS = 250;
const uint8_t SMOOTH_FACTOR = 4;
const int MAX_INPUT = 100;
const int PWM_MAX = 255;

// ----- NEW: turn scaling and joystick button output -----
const float TURN_SCALE = 0.8f;
const uint8_t JOY2_BUTTON_OUT_PIN = 2;  // NEW: D2 outputs HIGH/LOW based on j2_sw

// ----------------- Servos -----------------
const uint8_t SERVO_PINX = A0; // servoX (controls X axis)
const uint8_t SERVO_PINY = A1; // servoY (controls Y axis)
Servo servoX;
Servo servoY;

// Servo motion tuning (change these as you like)
const int JOY_THRESHOLD = 12;         // "certain value" threshold (±). Must exceed to change target.
const uint8_t SERVO_STEP = 1;         // degrees to change per step (1 = very slow)
const unsigned long SERVO_STEP_MS = 50; // ms between steps (increase -> slower motion)
 // NOTE: SERVO_STEP + SERVO_STEP_MS determine angular speed.


struct Telemetry {
  int16_t j1_x;
  int16_t j1_y;
  uint8_t j1_sw;
  int16_t j2_x;
  int16_t j2_y;
  uint8_t j2_sw;
};

unsigned long last_rx = 0;
int currentLeftPWM = 0, currentRightPWM = 0;

// servo persistent targets & positions
int targetX = 90;   // stored target for X servo (0..180)
int targetY = 90;   // stored target for Y servo (0..180)
int posX = 90;      // current actual position (angle) written to servoX
int posY = 90;      // current actual position written to servoY
unsigned long lastServoStep = 0;

void setup() {
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);

  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);

  // NEW: joystick 2 button output
  pinMode(JOY2_BUTTON_OUT_PIN, OUTPUT);
  digitalWrite(JOY2_BUTTON_OUT_PIN, LOW); // default LOW

  Serial.begin(9600);
  delay(50);
  Serial.println(F("RX Motor Controller starting"));

  if (!radio.begin()) {
    Serial.println(F("radio.begin failed - check wiring and 3.3V power"));
    while (1) { delay(200); }
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);

  last_rx = millis();
  stopMotors();
  Serial.println(F("RX ready, listening..."));
  posX = targetX = 90;
  posY = targetY = 90;
  servoX.attach(SERVO_PINX);
  servoY.attach(SERVO_PINY);
  servoX.write(posX);
  servoY.write(posY);

}

int mapToPWM(int v) {
  v = constrain(v, -MAX_INPUT, MAX_INPUT);
  return (v * PWM_MAX) / MAX_INPUT;
}

// set a motor side: speed in -255..255
void setMotorSide(uint8_t inA, uint8_t inB, uint8_t enPin, int speed) {
  if (speed > 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    analogWrite(enPin, constrain(speed, 0, PWM_MAX));
  } else if (speed < 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    analogWrite(enPin, constrain(-speed, 0, PWM_MAX));
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    analogWrite(enPin, 0);
  }
}

void stopMotors() {
  setMotorSide(IN1_PIN, IN2_PIN, ENA_PIN, 0);
  setMotorSide(IN3_PIN, IN4_PIN, ENB_PIN, 0);
  currentLeftPWM = currentRightPWM = 0;
}

void loop() {
  // radio + telemetry handling
  if (radio.available()) {
    uint8_t len = radio.getDynamicPayloadSize();
    if (len == 0 || len > sizeof(Telemetry)) {
      radio.flush_rx();
      Serial.print(F("RX: bad payload size "));
      Serial.println(len);
    } else {
      Telemetry t;
      memset(&t, 0, sizeof(t));
      radio.read(&t, len);
      last_rx = millis();

      // ---------------- Driving logic (unchanged) ----------------
      int jx = constrain((int)t.j2_x, -MAX_INPUT, MAX_INPUT);
      int jy = -constrain((int)t.j2_y, -MAX_INPUT, MAX_INPUT);

      float speedFactor = 1.0f - (abs(jy) / (float)MAX_INPUT) * 0.4f;
      if (speedFactor < 0.6f) speedFactor = 0.6f;
      int scaledJx = (int)round(jx * TURN_SCALE * speedFactor);

      int leftVal = constrain(jy + scaledJx, -MAX_INPUT, MAX_INPUT);
      int rightVal = constrain(jy - scaledJx, -MAX_INPUT, MAX_INPUT);

      int targetLeft = mapToPWM(leftVal);
      int targetRight = mapToPWM(rightVal);

      if (SMOOTH_FACTOR <= 1) {
        currentLeftPWM = targetLeft;
        currentRightPWM = targetRight;
      } else {
        currentLeftPWM += (targetLeft - currentLeftPWM) / SMOOTH_FACTOR;
        currentRightPWM += (targetRight - currentRightPWM) / SMOOTH_FACTOR;
      }

      setMotorSide(IN1_PIN, IN2_PIN, ENA_PIN, currentLeftPWM);
      setMotorSide(IN3_PIN, IN4_PIN, ENB_PIN, currentRightPWM);

      // JOY2 button reflect on D2
      digitalWrite(JOY2_BUTTON_OUT_PIN, t.j2_sw ? HIGH : LOW);

      // ---------------- Servo target-maker logic (joystick 1 controls targets) ----------------
      // j1_x controls targetX (servoX), j1_y controls targetY (servoY)
      int j1x = constrain((int)t.j1_x, -MAX_INPUT, MAX_INPUT);
      int j1y = constrain((int)t.j1_y, -MAX_INPUT, MAX_INPUT);

      unsigned long now = millis();

      // Update targetX if joystick X deflection exceeds threshold
      if (j1x > JOY_THRESHOLD) {
        // increase targetX slowly
        if (now - lastServoStep >= SERVO_STEP_MS) {
          lastServoStep = now;
          targetX = constrain(targetX + SERVO_STEP, 0, 180);
        }
      } else if (j1x < -JOY_THRESHOLD) {
        // decrease targetX slowly
        if (now - lastServoStep >= SERVO_STEP_MS) {
          lastServoStep = now;
          targetX = constrain(targetX - SERVO_STEP, 0, 180);
        }
      }
      // If within threshold, do NOT change targetX (held value)

      // Update targetY similarly (independent)
      if (j1y > JOY_THRESHOLD) {
        if (now - lastServoStep >= SERVO_STEP_MS) {
          lastServoStep = now;
          targetY = constrain(targetY + SERVO_STEP, 0, 180);
        }
      } else if (j1y < -JOY_THRESHOLD) {
        if (now - lastServoStep >= SERVO_STEP_MS) {
          lastServoStep = now;
          targetY = constrain(targetY - SERVO_STEP, 0, 180);
        }
      }
      // If within threshold, do NOT change targetY

      // ---------------- Move servos slowly toward stored targets ----------------
      // We'll step posX/posY by at most SERVO_STEP on each SERVO_STEP_MS tick (already gated above).
      // This ensures smooth slow motion.
      // Note: using same lastServoStep for both axes means the step cadence is shared;
      // if you prefer separate rates, create lastServoStepX/lastServoStepY.
      if (posX < targetX) {
        posX = min(posX + SERVO_STEP, targetX);
        servoX.write(posX);
      } else if (posX > targetX) {
        posX = max(posX - SERVO_STEP, targetX);
        servoX.write(posX);
      }

      if (posY < targetY) {
        posY = min(posY + SERVO_STEP, targetY);
        servoY.write(posY);
      } else if (posY > targetY) {
        posY = max(posY - SERVO_STEP, targetY);
        servoY.write(posY);
      }

      // ---------------- Debug print ----------------
      Serial.print(F("J1 x=")); Serial.print(j1x);
      Serial.print(F(" y=")); Serial.print(j1y);
      Serial.print(F(" | tgtX=")); Serial.print(targetX);
      Serial.print(F(" tgtY=")); Serial.print(targetY);
      Serial.print(F(" | posX=")); Serial.print(posX);
      Serial.print(F(" posY=")); Serial.print(posY);
      Serial.print(F(" | motors L=")); Serial.print(currentLeftPWM);
      Serial.print(F(" R=")); Serial.print(currentRightPWM);
      Serial.println();
    }
  } else {
    // radio not available: check timeout
    if (millis() - last_rx > TIMEOUT_MS) {
      stopMotors();
      // keep targets unchanged; servos remain at current pos
    }
  }

  delay(5);
}