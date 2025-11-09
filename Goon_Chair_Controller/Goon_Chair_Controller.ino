#include <Servo.h>

Servo servoOne, servoTwo;

// -------------------- Motor pins --------------------
const int leftForward   = 11;
const int leftReverse   = 10;
const int rightForward  = 6;
const int rightReverse  = 5;

// -------------------- Constants --------------------
const int MAX_SPEED = 150;
const int DEFAULT_FORWARD = 100;
const float Kp = 0.167;
const int rampStep = 5;

// -------------------- Motor state --------------------
int defaultForward = 0;
int targetLeft = 0;
int targetRight = 0;

// -------------------- PID state --------------------
int lastError = 0;

// -------------------- Serial Buffer --------------------
const int MAX_SERIAL_LENGTH = 16;
char serialBuffer[MAX_SERIAL_LENGTH];
int bufferIndex = 0;

// -------------------- Ultrasonic --------------------
const int trigPin = 4;
const int echoPin = 3;
int stableCount = 0;
const int stableThreshold = 5;   // stop after 5 stable readings
const float stopMin = 40.0;
const float stopMax = 55.0;

bool once = true;

// -------------------- Setup --------------------
void setup() {
  pinMode(leftForward, OUTPUT);
  pinMode(leftReverse, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightReverse, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Fixed typo: attach, not attatch
  servoOne.attach(7);  // small jerker
  servoTwo.attach(8);
  servoTwo.write(0);

  Serial.begin(115200);
  Serial.println("✅ Arduino ready: PID + safety stop + ultrasonic check");
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
  Serial.print("🛑 STOP: ");
  Serial.println(reason);
}

// -------------------- Ultrasonic distance --------------------
float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // timeout 30ms
  if (duration == 0) return -1.0; // invalid reading
  return duration * 0.0343 / 2.0; // cm
}

// -------------------- Main control --------------------
void updateMotorsPID(int deltaX, bool stopMotorsFlag) {
  if (stopMotorsFlag) {
    stopMotors("Sensor condition");
    return;
  }

  // Ramp default forward speed toward DEFAULT_FORWARD
  if (defaultForward < DEFAULT_FORWARD) defaultForward += rampStep;
  else if (defaultForward > DEFAULT_FORWARD) defaultForward -= rampStep;

  // Simple proportional control
  int correction = int(Kp * deltaX);
  targetLeft  = constrain(defaultForward + correction, -MAX_SPEED, MAX_SPEED);
  targetRight = constrain(defaultForward - correction, -MAX_SPEED, MAX_SPEED);

  setMotorPWM(leftForward, leftReverse, targetLeft);
  setMotorPWM(rightForward, rightReverse, targetRight);
}

// -------------------- Servo stroker --------------------
void stroker() {
  if (once) {
    servoTwo.write(50); // fixed: use valid angle
    delay(2000);
    once = false; // fixed missing semicolon
  }

  servoOne.write(20);
  delay(300);
  servoOne.write(120);
  delay(300);
}

// -------------------- Serial reading --------------------
void readSerial() {
  static int counter = 0; // moved outside for proper counting

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      serialBuffer[bufferIndex] = '\0';
      if (bufferIndex > 0) {
        int deltaX = atoi(serialBuffer);
        bufferIndex = 0;

        // Condition 1: Python tells us no person / camera issue
        if (deltaX == 9999) {
          stopMotors("No person or camera error");
          return;
        }

        // Condition 2: Ultrasonic safety stop
        float distance = getDistance();
        bool stopUltrasonic = false;

        if (distance == -1 || distance <= 1.0) { // invalid or very close
          stopUltrasonic = true;
          Serial.println("🛑 STOP: Ultrasonic invalid or too close");
        } 
        else if (distance >= stopMin && distance <= stopMax) {
          stableCount++;
          if (stableCount >= stableThreshold) stopUltrasonic = true;
        } else {
          stableCount = 0;
        }

        // Servo action for close detection
        if (distance > 0 && distance <= 1.0) {
          counter++;
          if (counter >= 20) {
            stroker();
          }
        } else {
          counter = 0;
        }

        // Update motors or stop
        updateMotorsPID(deltaX, stopUltrasonic);

      } else {
        bufferIndex = 0;
      }
    } else if (bufferIndex < MAX_SERIAL_LENGTH - 1) {
      serialBuffer[bufferIndex++] = c;
    }
  }
}

// -------------------- Main loop --------------------
void loop() {
  readSerial();
}
