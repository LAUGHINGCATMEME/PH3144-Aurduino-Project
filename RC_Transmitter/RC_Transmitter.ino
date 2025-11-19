/* Receiver for nRF24L01 -> L298N (4 wheels) + 2 servos (perpendicular axes)
   - Expects Telemetry struct packed by the transmitter (j1_x, j1_y, j1_sw, j2_x, j2_y, j2_sw)
   - Uses joystick2 (j2_x, j2_y) to control motion:
       j2_y = forward/back ( -100..100 )
       j2_x = turn left/right ( -100..100 )
   - Differential mixing: left = y + x, right = y - x
   - Two servos: one assigned to X-axis, one to Y-axis (perpendicular).
     Only one servo is actively driven at a time (the axis with larger deflection).
   - JOY2 pushbutton (j2_sw) is reflected on D2 (HIGH when pressed).
   - NOTE: left PWM (ENA) moved to D5 to avoid Timer1 conflicts with Servo library.
*/

#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

// ----------------- RADIO -----------------
RF24 radio(8, 10);
const uint64_t pipe = 0xE8E8F0F0E1LL;

// ----------------- MOTOR PINS ----------------- (adjusted ENA -> D5)
const uint8_t IN1_PIN = 7;   // Left motor direction A
const uint8_t IN2_PIN = 6;   // Left motor direction B
const uint8_t ENA_PIN = 5;   // Left PWM (moved from D9 -> D5 to avoid Timer1 conflict)

const uint8_t IN3_PIN = 4;   // Right motor direction A
const uint8_t IN4_PIN = 9;   // Right motor direction B (moved from D5 -> D9)
const uint8_t ENB_PIN = 3;   // Right PWM

// ----------------- SAFETY / TUNING -----------------
const unsigned long TIMEOUT_MS = 250;
const uint8_t SMOOTH_FACTOR = 4;
const int MAX_INPUT = 100;
const int PWM_MAX = 255;

// ----- NEW: turn scaling and joystick button output -----
const float TURN_SCALE = 0.8f;
const uint8_t JOY2_BUTTON_OUT_PIN = 2;  // D2 outputs HIGH/LOW based on j2_sw

// ----- Servos -----
const uint8_t SERVO_X_PIN = A0;   // servo that responds to X axis (use analog pins as digital)
const uint8_t SERVO_Y_PIN = A1;   // servo that responds to Y axis
const int SERVO_DEADZONE = 8;     // joystick deadzone (0..100)
Servo servoX;
Servo servoY;
bool servoX_attached = false;
bool servoY_attached = false;

// Telemetry struct must match the transmitter
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

void setup() {
  // motor pins
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);

  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);

  // joystick 2 button output
  pinMode(JOY2_BUTTON_OUT_PIN, OUTPUT);
  digitalWrite(JOY2_BUTTON_OUT_PIN, LOW); // default LOW

  Serial.begin(9600);
  delay(50);
  Serial.println(F("RX Motor+Servo Controller starting"));

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

  // Ensure servos start detached for lowest current draw
  if (servoX_attached) { servoX.detach(); servoX_attached = false; }
  if (servoY_attached) { servoY.detach(); servoY_attached = false; }

  Serial.println(F("RX ready, listening..."));
}

// map -100..100 to -255..255
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
    // stop/coast
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

// Map -100..100 -> 0..180 servo angle
int axisToServoAngle(int v) {
  v = constrain(v, -MAX_INPUT, MAX_INPUT);
  return map(v, -MAX_INPUT, MAX_INPUT, 0, 180);
}

void attachServoXIfNeeded() {
  if (!servoX_attached) {
    servoX.attach(SERVO_X_PIN);
    servoX_attached = true;
    // short settle delay
    delay(10);
  }
}
void detachServoXIfNeeded() {
  if (servoX_attached) {
    servoX.detach();
    servoX_attached = false;
  }
}
void attachServoYIfNeeded() {
  if (!servoY_attached) {
    servoY.attach(SERVO_Y_PIN);
    servoY_attached = true;
    delay(10);
  }
}
void detachServoYIfNeeded() {
  if (servoY_attached) {
    servoY.detach();
    servoY_attached = false;
  }
}

void loop() {
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

      // Driving controls (joystick 2)
      int jx = constrain((int)t.j2_x, -MAX_INPUT, MAX_INPUT);
      int jy = constrain((int)t.j2_y, -MAX_INPUT, MAX_INPUT);

      // ---- smoother turns ----
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

      // ---- JOY2 button on D2 ----
      if (t.j2_sw) {
        digitalWrite(JOY2_BUTTON_OUT_PIN, HIGH);
      } else {
        digitalWrite(JOY2_BUTTON_OUT_PIN, LOW);
      }

      // ---- Servo axis logic: control only one servo at a time ----
      int abs_jx = abs(jx);
      int abs_jy = abs(jy);

      // If both axes are near zero, detach both to save power.
      if (abs_jx <= SERVO_DEADZONE && abs_jy <= SERVO_DEADZONE) {
        detachServoXIfNeeded();
        detachServoYIfNeeded();
      } else {
        // Choose dominant axis
        if (abs_jx > abs_jy) {
          // Control X servo, detach Y
          detachServoYIfNeeded();
          attachServoXIfNeeded();
          int angleX = axisToServoAngle(jx); // map -100..100 to 0..180
          servoX.write(angleX);
        } else {
          // Control Y servo, detach X
          detachServoXIfNeeded();
          attachServoYIfNeeded();
          int angleY = axisToServoAngle(jy);
          servoY.write(angleY);
        }
      }

      // Debug print
      Serial.print(F("RX j2: X=")); Serial.print(jx);
      Serial.print(F(" Y=")); Serial.print(jy);
      Serial.print(F(" Btn=")); Serial.print(t.j2_sw);
      Serial.print(F("  -> L=")); Serial.print(currentLeftPWM);
      Serial.print(F(" R=")); Serial.print(currentRightPWM);
      Serial.print(F("  ServoX_att=")); Serial.print(servoX_attached);
      Serial.print(F(" ServoY_att=")); Serial.println(servoY_attached);
    }
  } else {
    // check timeout
    if (millis() - last_rx > TIMEOUT_MS) {
      stopMotors();
      // detach servos when radio lost to save power
      detachServoXIfNeeded();
      detachServoYIfNeeded();
    }
  }

  delay(5); // small loop delay
}
