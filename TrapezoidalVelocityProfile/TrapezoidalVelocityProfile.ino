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

volatile long y_ticks = 0;  // Encoder tick count

const int maxSpeed = 100;      // Maximum speed (0-255)
const int targetTicks = 20000;  // Target distance in encoder ticks
const float accelRate = 1.5;     // Acceleration rate (ticks per ms^2)
const float decelRate = 0.6;     // Deceleration rate (ticks per ms^2)

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
  pinMode(21, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);

  // Attach interrupt for Encoder pin A (optional)
  attachInterrupt(digitalPinToInterrupt(21), ai0_2, RISING);  // Encoder 1 pin A interrupt
  attachInterrupt(digitalPinToInterrupt(23), ai1_2, RISING);

  Serial.begin(115200);
}

void loop() {
  // Calculate current ticks
  Serial.print("Ticks: ");
  Serial.println(y_ticks);
  long currentTicks = y_ticks;
  // Calculate required speed
  int speed = calculateSpeed(currentTicks, targetTicks, maxSpeed, accelRate, decelRate);
  Serial.print("Speed: ");
  Serial.println(speed);

  // Set motor speed
  setMotorSpeed(speed);

  // Stop if target ticks are reached
  // if (currentTicks >= targetTicks) {
  //   setMotorSpeed(0);
  //   while (true)
  //     ;  // Stop the loop
  // }

  delay(10);  // Loop interval
}

// Encoder interrupt service routine (ISR)
void ai0_2() {
  // Encoder 1 pin A rising edge interrupt
  if (digitalRead(23) == LOW) {
    y_ticks++;
  } else {
    y_ticks--;
  }
}

void ai1_2() {
  // Encoder 1 pin B rising edge interrupt
  if (digitalRead(21) == LOW) {
    y_ticks--;
  } else {
    y_ticks++;
  }
}
int calculateSpeed(long currentTicks, long targetTicks, int maxSpeed, float accelRate, float decelRate) {
  long remainingTicks = targetTicks - currentTicks;

  float currentSpeed = sqrt(2 * accelRate * currentTicks);

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

void setMotorSpeed(int speed) {
  analogWrite(pwm1, speed);
  digitalWrite(motor1, LOW);  // Ensure direction pin is set

  analogWrite(pwm2, speed);
  digitalWrite(motor2, LOW);  // Ensure direction pin is set

  analogWrite(pwm3, speed);
  digitalWrite(motor3, LOW);  // Ensure direction pin is set

  analogWrite(pwm4, speed);
  digitalWrite(motor4, LOW);  // Ensure direction pin is set
}