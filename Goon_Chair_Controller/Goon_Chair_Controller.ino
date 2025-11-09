// -------------------- Motor pins --------------------
const int leftForward   = 11;
const int leftReverse   = 10;
const int rightForward  = 6;
const int rightReverse  = 5;

// -------------------- Constants --------------------
const int MAX_SPEED = 150;
const float Kp = 0.167;
const int DEFAULT_FORWARD = 100;
const int rampStep = 5;

// -------------------- Motor state --------------------
int defaultForward = 0;
int targetLeft = 0;
int targetRight = 0;

// -------------------- Serial Buffer --------------------
const int MAX_SERIAL_LENGTH = 16;
char serialBuffer[MAX_SERIAL_LENGTH];
int bufferIndex = 0;

// -------------------- Chair state --------------------
bool isActive = false;  // only move when UI says track/scan

// -------------------- Setup --------------------
void setup() {
  pinMode(leftForward, OUTPUT);
  pinMode(leftReverse, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightReverse, OUTPUT);

  Serial.begin(115200);
  Serial.println("✅ Arduino ready, default STOP, waiting for UI commands");
  stopMotors("Startup STOP");
}

// -------------------- Motor helper --------------------
void setMotorPWM(int forwardPin, int reversePin, int speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  if (speed >= 0) {
    analogWrite(forwardPin, speed);
    analogWrite(reversePin, 0);
  } else {
    analogWrite(forwardPin, 0);
    analogWrite(reversePin, -speed);
  }
}

// -------------------- Stop helper --------------------
void stopMotors(const char* reason) {
  setMotorPWM(leftForward, leftReverse, 0);
  setMotorPWM(rightForward, rightReverse, 0);
  defaultForward = 0;
  targetLeft = 0;
  targetRight = 0;
  isActive = false;
  Serial.print("🛑 STOP: ");
  Serial.println(reason);
}

// -------------------- Main control --------------------
void updateMotorsPID(int deltaX) {
  if (!isActive) return; // only act if UI activated

  // Ramp forward speed
  if (defaultForward < DEFAULT_FORWARD) defaultForward += rampStep;
  else if (defaultForward > DEFAULT_FORWARD) defaultForward -= rampStep;

  int correction = int(Kp * deltaX);
  targetLeft  = constrain(defaultForward + correction, -MAX_SPEED, MAX_SPEED);
  targetRight = constrain(defaultForward - correction, -MAX_SPEED, MAX_SPEED);

  setMotorPWM(leftForward, leftReverse, targetLeft);
  setMotorPWM(rightForward, rightReverse, targetRight);
}

// -------------------- Serial reading --------------------
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      serialBuffer[bufferIndex] = '\0';
      if (bufferIndex > 0) {
        int deltaX = atoi(serialBuffer);
        bufferIndex = 0;

        if (deltaX == 9999) {
          stopMotors("UI STOP / No command");
        } else if (deltaX == 0) {
          stopMotors("Scan detected person - stop");
        } else {
          isActive = true; // track or scan
          updateMotorsPID(deltaX);
        }
      }
    } else if (bufferIndex < MAX_SERIAL_LENGTH - 1) {
      serialBuffer[bufferIndex++] = c;
    }
  }
}

void loop() {
  readSerial();
}
