#include <Servo.h>
// ==================== PINS ====================
const byte PWMA = 5, AIN1 = 9, AIN2 = 10;  // Right motor
const byte PWMB = 6, BIN1 = 11, BIN2 = 12; // Left motor
const byte STBY = 7;
const byte SERVO_PIN = 3;
const byte TRIG = A0, ECHO = A1;
// Reverse a motor's electrical direction if required.
const bool INVERT_LEFT = true;
const bool INVERT_RIGHT = true;
// ==================== SERVO SCAN ====================
Servo head;
const byte FRONT = 90;
// Scan the complete servo range in 30-degree increments.
const byte ANGLES[] = {0, 30, 60, 90, 120, 150, 180};
const byte N_ANGLES = sizeof(ANGLES) / sizeof(ANGLES[0]);
const byte SERVO_STEP_DEG = 3;
const unsigned long SERVO_STEP_MS = 10;
const unsigned long SERVO_SETTLE_MS = 120;
// ==================== MOTOR CALIBRATION ====================
const int LEFT_SPEED = 89;
const int RIGHT_SPEED = 95;
const int TURN_SPEED = 65;
const int TURN_KICK_SPEED = 110;
const unsigned long TURN_KICK_MS = 40;
const float TURN_MS_PER_DEGREE = 4.4;
const float LEFT_TURN_TRIM = 1.00;
const float RIGHT_TURN_TRIM = 1.00;
const byte NUDGE_TURN_DEG = 15;
const byte ESCAPE_TURN_DEG = 90;
// ==================== NAVIGATION ====================
const float STOP_CM = 30.0;
const float FALLBACK_CM = 35.0;
const float SAFE_CM = 40.0;
const float TURN_PENALTY_PER_DEG = 0.25f;
const float UNKNOWN_NEIGHBOR_PENALTY = 20.0f;
const unsigned long FRONT_CHECK_MS = 70;
const unsigned long REVERSE_MS = 200;
const unsigned long TURN_SETTLE_MS = 150;
const unsigned long BLOCKED_MS = 1500;
const unsigned long SAMPLE_GAP_MS = 60;
const unsigned long ECHO_TIMEOUT_US = 12000UL;
const byte SAMPLES_PER_POINT = 3;
const byte MIN_VALID_SAMPLES = 2;
const byte MAX_SCAN_RETRIES = 1;
const byte MAX_ESCAPES = 4;
const byte MAX_NUDGES = 4;
const byte MAX_BACKUPS = 2;
// ==================== STATES ====================
enum State {
  DRIVE,
  SCAN_MOVE,
  SCAN_SETTLE,
  SAMPLING,
  CENTER_SERVO,
  CENTER_SETTLE,
  TURNING,
  TURN_SETTLE,
  BACKUP,
  BLOCKED
};
State state = DRIVE;
unsigned long deadline = 0;
unsigned long nextFrontCheck = 0;
unsigned long lastServoStep = 0;
unsigned long kickEnd = 0;
int servoPosition = FRONT;
int servoTarget = FRONT;
float front = -1;
float scanDistances[N_ANGLES];
float samples[SAMPLES_PER_POINT];
byte scanIndex = 0;
byte sampleCount = 0;
byte validCount = 0;
bool samplingForScan = true;
bool kicking = false;
bool rescanAfterTurn = false;
int turnDir = 0;
int lastTurnDir = 0;
int lastEscapeDir = 1;
byte scanRetries = 0;
byte escapeCount = 0;
byte nudgeCount = 0;
byte backupCount = 0;
// ==================== MOTOR CONTROL ====================
void motor(byte pwm, byte in1, byte in2,
           int speed, bool invert) {
  if (invert) speed = -speed;
  speed = constrain(speed, -255, 255);
  analogWrite(pwm, 0);
  digitalWrite(in1, speed >= 0 ? HIGH : LOW);
  digitalWrite(in2, speed >= 0 ? LOW : HIGH);
  analogWrite(pwm, abs(speed));
}
void drive(int left, int right) {
  motor(PWMA, AIN1, AIN2, right, INVERT_RIGHT);
  motor(PWMB, BIN1, BIN2, left, INVERT_LEFT);
}
void stopCar() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}
void forward() {
  drive(LEFT_SPEED, RIGHT_SPEED);
}
void reverseCar() {
  drive(-LEFT_SPEED, -RIGHT_SPEED);
}
void spin(int dir, int speed) {
  // +1 = left, -1 = right
  drive(-dir * speed, dir * speed);
}
// ==================== SERVO CONTROL ====================
void setServoTarget(int angle) {
  servoTarget = constrain(angle, 0, 180);
}
bool updateServo() {
  unsigned long now = millis();
  if (servoPosition == servoTarget) return true;
  if (now - lastServoStep < SERVO_STEP_MS) return false;
  lastServoStep = now;
  if (servoPosition < servoTarget) {
    servoPosition = min(servoPosition + SERVO_STEP_DEG,
                        servoTarget);
  } else {
    servoPosition = max(servoPosition - SERVO_STEP_DEG,
                        servoTarget);
  }
  head.write(servoPosition);
  return servoPosition == servoTarget;
}
// ==================== ULTRASONIC ====================
float distanceCM() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long duration =
      pulseIn(ECHO, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1;
  float cm = duration / 58.0f;
  if (cm < 2 || cm > 400) return -1;
  return cm;
}
// ==================== SAMPLING ====================
void beginSampling(bool forScan) {
  samplingForScan = forScan;
  sampleCount = 0;
  validCount = 0;
  deadline = millis();
  state = SAMPLING;
}
float sampledDistance() {
  if (validCount < MIN_VALID_SAMPLES) return -1;
  // Three valid readings: median rejects isolated outliers.
  if (validCount == 3) {
    float a = samples[0];
    float b = samples[1];
    float c = samples[2];
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
  }
  // Two valid readings: use the closer one conservatively.
  float a = 9999;
  float b = 9999;
  for (byte i = 0; i < SAMPLES_PER_POINT; i++) {
    if (samples[i] >= 0) {
      if (samples[i] < a) {
        b = a;
        a = samples[i];
      } else if (samples[i] < b) {
        b = samples[i];
      }
    }
  }
  return min(a, b);
}
void finishSampling();
// ==================== SCANNING ====================
void startScan() {
  stopCar();
  for (byte i = 0; i < N_ANGLES; i++) {
    scanDistances[i] = -1;
  }
  scanIndex = 0;
  setServoTarget(ANGLES[scanIndex]);
  state = SCAN_MOVE;
  Serial.println(F("Scanning"));
}
void recordReading(float d) {
  scanDistances[scanIndex] = d;
  scanIndex++;
  if (scanIndex >= N_ANGLES) {
    setServoTarget(FRONT);
    state = CENTER_SERVO;
  } else {
    setServoTarget(ANGLES[scanIndex]);
    state = SCAN_MOVE;
  }
}
// ==================== TURNING ====================
void startTurn(int dir, int degrees) {
  stopCar();
  float trim = (dir > 0)
                   ? LEFT_TURN_TRIM
                   : RIGHT_TURN_TRIM;
  unsigned long duration =
      (unsigned long)(degrees * TURN_MS_PER_DEGREE * trim);
  turnDir = dir;
  lastTurnDir = dir;
  Serial.print(dir > 0 ? F("Left turn: ") : F("Right turn: "));
  Serial.print(degrees);
  Serial.print(F(" deg, "));
  Serial.print(duration);
  Serial.println(F(" ms"));
  if (TURN_KICK_MS > 0 && duration > TURN_KICK_MS) {
    spin(dir, TURN_KICK_SPEED);
    kicking = true;
    kickEnd = millis() + TURN_KICK_MS;
  } else {
    spin(dir, TURN_SPEED);
    kicking = false;
  }
  deadline = millis() + duration;
  state = TURNING;
}
// ==================== DIRECTION SELECTION ====================
int pickBest(float minClear) {
  int best = -1;
  float bestScore = -1000.0f;
  for (byte i = 0; i < N_ANGLES; i++) {
    float d = scanDistances[i];
    int angle = ANGLES[i];
    // Unknown readings and insufficient clearance are rejected.
    if (d < minClear) continue;
    int offset = abs(angle - FRONT);
    float score = min(d, 150.0f)
                  - offset * TURN_PENALTY_PER_DEG;
    if (angle == FRONT) {
      // FRONT is an interior sample; guard indices for safety anyway.
      if (i == 0 || i + 1 >= N_ANGLES) continue;
      float leftD = scanDistances[i - 1];
      float rightD = scanDistances[i + 1];
      if (leftD < 0 || rightD < 0) continue;
      if (leftD <= STOP_CM || rightD <= STOP_CM) continue;
      score += min(leftD, 100.0f) * 0.15f;
      score += min(rightD, 100.0f) * 0.15f;
    } else {
      // Check the neighbor farther from the front direction.
      int outer = (angle > FRONT) ? (int)i + 1 : (int)i - 1;
      if (outer >= 0 && outer < (int)N_ANGLES) {
        float neighbor = scanDistances[outer];
        if (neighbor < 0) {
          score -= UNKNOWN_NEIGHBOR_PENALTY;
        } else if (neighbor <= STOP_CM) {
          continue;
        } else {
          score += min(neighbor, 100.0f) * 0.15f;
        }
      } else {
        // At 0 or 180 degrees, no farther-out sample exists.
        score -= UNKNOWN_NEIGHBOR_PENALTY;
      }
    }
    if (score > bestScore) {
      bestScore = score;
      best = i;
    }
  }
  return best;
}
void escape() {
  if (escapeCount >= MAX_ESCAPES) {
    escapeCount = 0;
    stopCar();
    deadline = millis() + BLOCKED_MS;
    state = BLOCKED;
    Serial.println(F("No safe path; pausing"));
    return;
  }
  escapeCount++;
  if (escapeCount == 1) {
    float leftBest = 0;
    float rightBest = 0;
    for (byte i = 0; i < N_ANGLES; i++) {
      float d = scanDistances[i];
      if (d < 0) continue;
      if (ANGLES[i] > FRONT) {
        leftBest = max(leftBest, d);
      } else if (ANGLES[i] < FRONT) {
        rightBest = max(rightBest, d);
      }
    }
    if (leftBest > rightBest) lastEscapeDir = 1;
    else if (rightBest > leftBest) lastEscapeDir = -1;
    else lastEscapeDir = -lastEscapeDir;
  }
  rescanAfterTurn = true;
  nudgeCount = 0;
  startTurn(lastEscapeDir, ESCAPE_TURN_DEG);
}
void decideDirection() {
  Serial.print(F("Scan:"));
  for (byte i = 0; i < N_ANGLES; i++) {
    Serial.print(' ');
    Serial.print(ANGLES[i]);
    Serial.print('=');
    if (scanDistances[i] < 0) Serial.print(F("UNK"));
    else Serial.print(scanDistances[i], 0);
  }
  Serial.println();
  int best = pickBest(SAFE_CM);
  if (best < 0) {
    best = pickBest(FALLBACK_CM);
  }
  if (best < 0) {
    Serial.println(F("No usable direction"));
    if (scanRetries < MAX_SCAN_RETRIES) {
      scanRetries++;
      startScan();
    } else {
      scanRetries = 0;
      escape();
    }
    return;
  }
  scanRetries = 0;
  nudgeCount = 0;
  rescanAfterTurn = false;
  int chosen = ANGLES[best];
  Serial.print(F("Chosen angle: "));
  Serial.println(chosen);
  if (chosen == FRONT) {
    lastTurnDir = 0;
    beginSampling(false);
    return;
  }
  int dir = (chosen > FRONT) ? 1 : -1;
  int scanOffset = abs(chosen - FRONT);
  // Favor roughly 90-degree turns for side routes, while allowing
  // 30-degree corrections when the selected direction is near-forward.
  int degrees = (scanOffset >= 60) ? 90 : 30;
  startTurn(dir, degrees);
}
// ==================== FRONT VERIFICATION ====================
void handleFrontCheck(float d) {
  front = d;
  Serial.print(F("Front check: "));
  Serial.println(front);
  // Unknown or too close: never drive forward.
  if (front > STOP_CM) {
    nudgeCount = 0;
    escapeCount = 0;
    backupCount = 0;
    forward();
    nextFrontCheck = millis() + FRONT_CHECK_MS;
    state = DRIVE;
    return;
  }
  if (lastTurnDir != 0 && nudgeCount < MAX_NUDGES) {
    nudgeCount++;
    Serial.println(F("Still blocked; nudging"));
    startTurn(lastTurnDir, NUDGE_TURN_DEG);
    return;
  }
  nudgeCount = 0;
  startScan();
}
void finishSampling() {
  float result = sampledDistance();
  if (samplingForScan) {
    recordReading(result);
  } else {
    handleFrontCheck(result);
  }
}
// ==================== OBSTACLE RESPONSE ====================
void haltAndScan(float d) {
  stopCar();
  if (d >= 0 && d < 10.0f && backupCount < MAX_BACKUPS) {
    backupCount++;
    Serial.println(F("Very close; reversing briefly"));
    reverseCar();
    deadline = millis() + REVERSE_MS;
    state = BACKUP;
    return;
  }
  startScan();
}
// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);
  digitalWrite(STBY, LOW);
  stopCar();
  head.attach(SERVO_PIN);
  head.write(FRONT);
  delay(400);
  digitalWrite(STBY, HIGH);
  Serial.println(F("Starting forward movement"));
  // Start moving straight immediately.
  forward();
  // Enable front-obstacle detection immediately.
  nextFrontCheck = millis();
  state = DRIVE;
}
// ==================== MAIN LOOP ====================
void loop() {
  unsigned long now = millis();
  switch (state) {
    case DRIVE:
      if ((long)(now - nextFrontCheck) >= 0) {
        front = distanceCM();
        if (front < 0) {
          delay(SAMPLE_GAP_MS);
          front = distanceCM();
        }
        nextFrontCheck = millis() + FRONT_CHECK_MS;
        if (front < 0) {
          Serial.println(F("Front unknown"));
          haltAndScan(-1);
        } else if (front <= STOP_CM) {
          Serial.print(F("Obstacle at "));
          Serial.println(front);
          haltAndScan(front);
        } else {
          forward();
        }
      }
      break;
    case SCAN_MOVE:
      if (updateServo()) {
        deadline = millis() + SERVO_SETTLE_MS;
        state = SCAN_SETTLE;
      }
      break;
    case SCAN_SETTLE:
      if ((long)(now - deadline) >= 0) {
        beginSampling(true);
      }
      break;
    case SAMPLING:
      if ((long)(now - deadline) >= 0) {
        float d = distanceCM();
        samples[sampleCount] = d;
        if (d >= 0) validCount++;
        sampleCount++;
        if (sampleCount < SAMPLES_PER_POINT) {
          deadline = millis() + SAMPLE_GAP_MS;
        } else {
          finishSampling();
        }
      }
      break;
    case CENTER_SERVO:
      if (updateServo()) {
        deadline = millis() + SERVO_SETTLE_MS;
        state = CENTER_SETTLE;
      }
      break;
    case CENTER_SETTLE:
      if ((long)(now - deadline) >= 0) {
        decideDirection();
      }
      break;
    case TURNING:
      if (kicking && (long)(now - kickEnd) >= 0) {
        spin(turnDir, TURN_SPEED);
        kicking = false;
      }
      if ((long)(now - deadline) >= 0) {
        stopCar();
        kicking = false;
        deadline = millis() + TURN_SETTLE_MS;
        state = TURN_SETTLE;
      }
      break;
    case TURN_SETTLE:
      if ((long)(now - deadline) >= 0) {
        if (rescanAfterTurn) {
          startScan();
        } else {
          beginSampling(false);
        }
      }
      break;
    case BACKUP:
      if ((long)(now - deadline) >= 0) {
        stopCar();
        startScan();
      }
      break;
    case BLOCKED:
      if ((long)(now - deadline) >= 0) {
        scanRetries = 0;
        escapeCount = 0;
        nudgeCount = 0;
        backupCount = 0;
        startScan();
      }
      break;
  }
}
