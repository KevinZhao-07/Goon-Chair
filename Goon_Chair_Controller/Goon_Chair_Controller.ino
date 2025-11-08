// -------------------- Motor pins --------------------
const int leftForward  = 11;
const int leftReverse  = 10;
const int rightForward = 6;
const int rightReverse = 5;

// -------------------- Constants --------------------
const int MAX_SPEED = 150;
const float kP = 0.167;            // proportional gain for X-centering
const int DEFAULT_FORWARD = 67;    // default forward speed
const int DEADZONE = 10;           // pixels

// -------------------- Motor state --------------------
int currentLeft  = 0;
int currentRight = 0;
int targetLeft   = 0;
int targetRight  = 0;

unsigned long lastRampTime = 0;
const unsigned long rampInterval = 2; // ms per step

// -------------------- Setup --------------------
void setup() {
  pinMode(leftForward, OUTPUT);
  pinMode(leftReverse, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightReverse, OUTPUT);

  analogWrite(leftForward, 0);
  analogWrite(leftReverse, 0);
  analogWrite(rightForward, 0);
  analogWrite(rightReverse, 0);

  Serial.begin(9600);  // must match Python baud
  Serial.println("Arduino ready for deltaX input...");
}

// -------------------- Motor update / ramping --------------------
void updateMotors() {
  unsigned long now = millis();
  if(now - lastRampTime >= rampInterval){
    lastRampTime = now;

    if(currentLeft < targetLeft) currentLeft++;
    else if(currentLeft > targetLeft) currentLeft--;

    if(currentRight < targetRight) currentRight++;
    else if(currentRight > targetRight) currentRight--;

    setMotorPWM(leftForward, leftReverse, currentLeft);
    setMotorPWM(rightForward, rightReverse, currentRight);
  }
}

// -------------------- Helper --------------------
void setMotorPWM(int forwardPin, int reversePin, int speed){
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  if(speed >= 0){
    analogWrite(forwardPin, speed);
    analogWrite(reversePin, 0);
  } else {
    analogWrite(forwardPin, 0);
    analogWrite(reversePin, -speed);
  }
}

// -------------------- Track function --------------------
void track(int deltaX) {
  Serial.print("Received deltaX: ");
  Serial.println(deltaX);

  // Deadzone
  if(abs(deltaX) < DEADZONE) {
    targetLeft  = DEFAULT_FORWARD;
    targetRight = DEFAULT_FORWARD;
    return;
  }

  int adjustment = int(kP * deltaX);
  targetLeft  = constrain(DEFAULT_FORWARD + adjustment, -MAX_SPEED, MAX_SPEED);
  targetRight = constrain(DEFAULT_FORWARD - adjustment, -MAX_SPEED, MAX_SPEED);
}

// -------------------- Main loop --------------------
void loop() {
  // Check for serial input
  while(Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim(); // remove whitespace
    if(line.length() > 0){
      int deltaX = line.toInt();
      track(deltaX);
    }
  }
  updateMotors();
}
