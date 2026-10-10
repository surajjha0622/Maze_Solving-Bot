#include <Servo.h>

const byte PWMA = 5, AIN1 = 9, AIN2 = 10;
const byte PWMB = 6, BIN1 = 11, BIN2 = 12;
const byte STBY = 7, SERVO_PIN = 3, TRIG = A0, ECHO = A1;
const bool INVERT_LEFT = false, INVERT_RIGHT = false;
const int FRONT_ANGLE = 90, LEFT_ANGLE = 170, RIGHT_ANGLE = 10;
const int FORWARD_LEFT_PWM = 95, FORWARD_RIGHT_PWM = 95;
const int TURN_PWM = 110;
const unsigned int LEFT_TURN_MS = 450, RIGHT_TURN_MS = 450;
const unsigned int SERVO_SETTLE_MS = 500;
const float STOP_CM = 10.0;
const float MIN_SIDE_CM = 20.0; // depth threshold only; NOT proof of opening width
Servo head;

void motor(byte pwm, byte in1, byte in2, int speed, bool invert) {
  if (invert) speed = -speed;
  analogWrite(pwm, 0);
  digitalWrite(in1, speed >= 0 ? HIGH : LOW);
  digitalWrite(in2, speed >= 0 ? LOW : HIGH);
  analogWrite(pwm, abs(speed));
}

void drive(int left, int right) {
  motor(PWMA, AIN1, AIN2, left, INVERT_LEFT);
  motor(PWMB, BIN1, BIN2, right, INVERT_RIGHT);
}

void stopCar() {
  // TB6612 short brake: PWM low while STBY high.
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

float distanceCM() {
  // Leave at least 60 ms between ultrasonic triggers, including across calls.
  static unsigned long lastPing = 0;
  unsigned long elapsed = millis() - lastPing;
  if (elapsed < 60) delay(60 - elapsed);
  lastPing = millis();
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long duration = pulseIn(ECHO, HIGH, 25000UL);
  if (duration == 0) return -1;
  float cm = duration / 58.0;
  if (cm < 2 || cm > 400) return -1;
  return cm;
}

float look(int angle) {
  head.write(angle);
  delay(SERVO_SETTLE_MS);
  // Median of three stationary readings rejects a single spurious long echo.
  float a = distanceCM(), b = distanceCM(), c = distanceCM();
  if (a < 0 || b < 0 || c < 0) return -1;
  float temp;
  if (a > b) { temp = a; a = b; b = temp; }
  if (b > c) { temp = b; b = c; c = temp; }
  if (a > b) { temp = a; a = b; b = temp; }
  return b;
}

void chooseAndTurn() {
  stopCar();
  float left = look(LEFT_ANGLE);
  float right = look(RIGHT_ANGLE);
  Serial.print(F("LEFT cm: ")); Serial.print(left);
  Serial.print(F(" | RIGHT cm: ")); Serial.println(right);
  head.write(FRONT_ANGLE);
  delay(SERVO_SETTLE_MS);

  if (left < 0 || right < 0) {
    Serial.println(F("UNKNOWN side (-1): stopped; will recheck"));
    return;
  }
  if (left < MIN_SIDE_CM && right < MIN_SIDE_CM) {
    Serial.println(F("Both sides too short: stopped; will recheck"));
    return;
  }

  // Compare BOTH sides before moving. Equal readings choose left.
  if (left >= right) {
    Serial.println(F("TURN LEFT"));
    drive(-TURN_PWM, TURN_PWM);
    delay(LEFT_TURN_MS);
  } else {
    Serial.println(F("TURN RIGHT"));
    drive(TURN_PWM, -TURN_PWM);
    delay(RIGHT_TURN_MS);
  }
  stopCar();
  delay(150);
  float front = distanceCM();
   
}


void setup() {
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  stopCar();
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);
  Serial.begin(115200);
  head.attach(SERVO_PIN);
  head.write(FRONT_ANGLE);
  delay(1000);
  digitalWrite(STBY, HIGH);
  Serial.println(F("START: automatic. -1 means UNKNOWN distance."));
}

void loop() {
  if (fault) { stopCar(); delay(100); return; }
  float front = distanceCM();
  Serial.print(F("FRONT cm: ")); Serial.println(front);
  if (front <= STOP_CM) {
    chooseAndTurn();
  } else {
    drive(FORWARD_LEFT_PWM, FORWARD_RIGHT_PWM);
  }
}