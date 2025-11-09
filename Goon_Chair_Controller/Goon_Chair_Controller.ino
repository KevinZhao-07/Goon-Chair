// -------------------- Motor pins --------------------
const int leftForward   = 11;
const int leftReverse   = 10;
const int rightForward  = 6;
const int rightReverse  = 5;

// -------------------- Constants --------------------
const int MAX_SPEED = 150;
const int DEFAULT_FORWARD = 100;   // target forward speed
const float Kp = 0.167;             // proportional gain for deltaX
const int rampStep = 5;           // step for forward speed ramping

// -------------------- Motor state --------------------
int currentLeft   = 0;
int currentRight  = 0;
int targetLeft    = 0;
int targetRight   = 0;
int defaultForward = 0;           // ramps toward DEFAULT_FORWARD

// -------------------- PID state --------------------
float integral = 0;
int lastError = 0;

// -------------------- Serial Buffer --------------------
const int MAX_SERIAL_LENGTH = 16;
char serialBuffer[MAX_SERIAL_LENGTH];
int bufferIndex = 0;

// -------------------- Ultrasonic --------------------
const int trigPin = 4;
const int echoPin = 3;
int stableCount = 0;            // counts cycles in good range
const int stableThreshold = 5;  // stop after 5 cycles
const int minDistance = 20;     // cm
const int maxDistance = 40;     // cm

// -------------------- Setup --------------------
void setup() {
    pinMode(leftForward, OUTPUT);
    pinMode(leftReverse, OUTPUT);
    pinMode(rightForward, OUTPUT);
    pinMode(rightReverse, OUTPUT);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    analogWrite(leftForward, 0);
    analogWrite(leftReverse, 0);
    analogWrite(rightForward, 0);
    analogWrite(rightReverse, 0);

    Serial.begin(115200);  // must match Python baud
    Serial.println("Arduino ready for deltaX input (PID + ramp).");
}

// -------------------- Motor helper --------------------
void setMotorPWM(int forwardPin, int reversePin, int speed) {
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
    if(speed >= 0) {
        analogWrite(forwardPin, speed);
        analogWrite(reversePin, 0);
    } else {
        analogWrite(forwardPin, 0);
        analogWrite(reversePin, -speed);
    }
}

// -------------------- PID + ramp function --------------------
void updateMotorsPID(int deltaX, bool stopMotors) {
    if(stopMotors) {
        // Stop motors
        setMotorPWM(leftForward, leftReverse, 0);
        setMotorPWM(rightForward, rightReverse, 0);
        Serial.println("Chair stopped: stable distance detected.");
        return;
    }

    // Ramp default forward speed toward DEFAULT_FORWARD
    if(defaultForward < DEFAULT_FORWARD) defaultForward += rampStep;
    else if(defaultForward > DEFAULT_FORWARD) defaultForward -= rampStep;

    // PID for deltaX
    float error = deltaX;
    integral += error;
    float derivative = error - lastError;
    int correction = int(Kp * error); // + Ki * integral + Kd * derivative);
    lastError = error;

    // Apply speeds
    targetLeft  = constrain(defaultForward + correction, -MAX_SPEED, MAX_SPEED);
    targetRight = constrain(defaultForward - correction, -MAX_SPEED, MAX_SPEED);

    setMotorPWM(leftForward, leftReverse, targetLeft);
    setMotorPWM(rightForward, rightReverse, targetRight);

    // Debugging output
    Serial.print("deltaX: "); Serial.print(deltaX);
    Serial.print(" | targetL: "); Serial.print(targetLeft);
    Serial.print(" | targetR: "); Serial.println(targetRight);
}

// -------------------- Serial reading --------------------
void readSerial() {
    while(Serial.available()) {
        char incomingChar = Serial.read();

        if(incomingChar == '\n') {
            serialBuffer[bufferIndex] = '\0';
            if(bufferIndex > 0) {
                int deltaX = atoi(serialBuffer);

                // Read ultrasonic
                long duration;
                float distance;
                digitalWrite(trigPin, LOW);
                delayMicroseconds(2);
                digitalWrite(trigPin, HIGH);
                delayMicroseconds(10);
                digitalWrite(trigPin, LOW);

                duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
                distance = duration * 0.0343 / 2; // cm

                bool stopChair = false;
                if(distance >= minDistance && distance <= maxDistance) {
                    stableCount++;
                    if(stableCount >= stableThreshold) stopChair = true;
                } else {
                    stableCount = 0; // reset if out of range
                }

                updateMotorsPID(deltaX, stopChair);
            }
            bufferIndex = 0;
        } else if(bufferIndex < MAX_SERIAL_LENGTH - 1) {
            serialBuffer[bufferIndex++] = incomingChar;
        }
    }
}

// -------------------- Main loop --------------------
void loop() {
    readSerial();
}
