// Motor PWM pins
#define pwm1 18
#define pwm2 19
#define pwm3 15
#define pwm4 25

// Motor control pins
#define motor1 14
#define motor2 13
#define motor3 27
#define motor4 12

// Encoder pins
#define encoderY_A 21
#define encoderY_B 23
#define encoderX_A 5
#define encoderX_B 26

volatile long y_ticks = 0;  // Encoder tick count for Y direction
volatile long x_ticks = 0;  // Encoder tick count for X direction

const int maxSpeed = 100;   // Maximum speed (0-255)
const float accelRate = 2;  // Acceleration rate (ticks per ms^2)
const float decelRate = 2;  // Deceleration rate (ticks per ms^2)

int position = 0;
bool movingY = true;
bool movingX = false;
// In form of (y,x)
const long targetPositions[2][2] = {
  { 10000, 5000 },
  { 5000, 5000 },
};
static bool targetReached = false;

void setup() {
  // Set up motor control pins as outputs
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(pwm4, OUTPUT);

  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);

  // Set up encoder pins as inputs
  pinMode(encoderY_A, INPUT_PULLUP);
  pinMode(encoderY_B, INPUT_PULLUP);
  pinMode(encoderX_A, INPUT_PULLUP);
  pinMode(encoderX_B, INPUT_PULLUP);

  // Attach interrupt for Encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderY_A), ai0_2, RISING);   // Encoder Y pin A interrupt
  attachInterrupt(digitalPinToInterrupt(encoderY_B), ai1_2, RISING);   // Encoder Y pin B interrupt
  attachInterrupt(digitalPinToInterrupt(encoderX_A), ai0_2X, RISING);  // Encoder X pin A interrupt
  attachInterrupt(digitalPinToInterrupt(encoderX_B), ai1_2X, RISING);  // Encoder X pin B interrupt

  Serial.begin(115200);
}

void loop() {
 int targetYTicks = targetPositions[position][0];
 int targetXTicks = targetPositions[position][1];

  if (movingY) {
    // Calculate current ticks for Y direction
    Serial.print("Y Ticks: ");
    Serial.println(y_ticks);
    long currentYTicks = y_ticks;
    // Calculate required speed for Y direction
    int speedY = calculateSpeed(currentYTicks, targetYTicks, maxSpeed, accelRate, decelRate);
    Serial.print("Y Speed: ");
    Serial.println(speedY);

    // Set motor speed for Y direction
    setMotorSpeedForward(speedY);

    // Stop if target Y ticks are reached and switch to X direction
    if (currentYTicks >= targetYTicks) {
      setMotorSpeedForward(0);
      movingY = false;
      movingX = true;
      delay(1000);  // Short delay to simulate turning
    }
    x_ticks = 0;
  }

  if (movingX) {
    // Calculate current ticks for X direction
    Serial.print("X Ticks: ");
    Serial.println(x_ticks);
    long currentXTicks = x_ticks;
    // Calculate required speed for X direction
    int speedX = calculateSpeed(currentXTicks, targetXTicks, maxSpeed, accelRate, decelRate);
    Serial.print("X Speed: ");
    Serial.println(speedX);

    // Set motor speed for X direction
    setMotorSpeedRight(speedX);

    // Stop if target X ticks are reached
    if (currentXTicks >= targetXTicks) {
      setMotorSpeedRight(0);
      movingX = false;
      movingY = true;
      position +=1;
    }
    y_ticks = 0;
  }
  delay(100);
  if(position==2){
    Serial.println("Arrived");
    while(1){}
  }
}

// Encoder interrupt service routine (ISR) for Y direction
void ai0_2() {
  // Encoder Y pin A rising edge interrupt
  if (digitalRead(encoderY_B) == LOW) {
    y_ticks++;
  } else {
    y_ticks--;
  }
}

void ai1_2() {
  // Encoder Y pin B rising edge interrupt
  if (digitalRead(encoderY_A) == LOW) {
    y_ticks--;
  } else {
    y_ticks++;
  }
}

// Encoder interrupt service routine (ISR) for X direction
void ai0_2X() {
  // Encoder X pin A rising edge interrupt
  if (digitalRead(encoderX_B) == LOW) {
    x_ticks++;
  } else {
    x_ticks--;
  }
}

void ai1_2X() {
  // Encoder X pin B rising edge interrupt
  if (digitalRead(encoderX_A) == LOW) {
    x_ticks--;
  } else {
    x_ticks++;
  }
}

int calculateSpeed(long currentTicks, long targetTicks, int maxSpeed, float accelRate, float decelRate) {
  long remainingTicks = targetTicks - currentTicks;

  float currentSpeed = sqrt(2 * accelRate * currentTicks);
  if (currentTicks < 240) {
    currentSpeed = 40;
  }
  if (remainingTicks < 0) {
    remainingTicks = 0;
  }
  float requiredDecelSpeed = sqrt(2 * decelRate * remainingTicks);

  if (remainingTicks > targetTicks) {
    currentSpeed = 0;
  }
  if (currentSpeed > maxSpeed) {
    currentSpeed = maxSpeed;
  }

  if (currentSpeed > requiredDecelSpeed) {
    currentSpeed = requiredDecelSpeed;
  }

  return (int)currentSpeed;
}

void setMotorSpeedForward(int speed) {
  analogWrite(pwm1, speed);
  digitalWrite(motor1, LOW);  // Ensure direction pin is set

  analogWrite(pwm2, speed);
  digitalWrite(motor2, LOW);  // Ensure direction pin is set

  analogWrite(pwm3, speed);
  digitalWrite(motor3, LOW);  // Ensure direction pin is set

  analogWrite(pwm4, speed);
  digitalWrite(motor4, LOW);  // Ensure direction pin is set
}

void setMotorSpeedBackward(int speed) {
  analogWrite(pwm1, speed);
  digitalWrite(motor1, HIGH);  // Ensure direction pin is set

  analogWrite(pwm2, speed);
  digitalWrite(motor2, HIGH);  // Ensure direction pin is set

  analogWrite(pwm3, speed);
  digitalWrite(motor3, HIGH);  // Ensure direction pin is set

  analogWrite(pwm4, speed);
  digitalWrite(motor4, HIGH);  // Ensure direction pin is set
}

void setMotorSpeedLeft(int speed) {
  analogWrite(pwm1, speed);
  digitalWrite(motor1, LOW);  // Ensure direction pin is set

  analogWrite(pwm2, speed);
  digitalWrite(motor2, HIGH);  // Ensure direction pin is set

  analogWrite(pwm3, speed);
  digitalWrite(motor3, LOW);  // Ensure direction pin is set

  analogWrite(pwm4, speed);
  digitalWrite(motor4, HIGH);  // Ensure direction pin is set
}

void setMotorSpeedRight(int speed) {
  analogWrite(pwm1, speed);
  digitalWrite(motor1, HIGH);  // Ensure direction pin is set

  analogWrite(pwm2, speed);
  digitalWrite(motor2, LOW);  // Ensure direction pin is set

  analogWrite(pwm3, speed);
  digitalWrite(motor3, HIGH);  // Ensure direction pin is set

  analogWrite(pwm4, speed);
  digitalWrite(motor4, LOW);  // Ensure direction pin is set
}
