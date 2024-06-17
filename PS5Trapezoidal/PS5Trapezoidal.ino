#include <ps5Controller.h>

#define pwm1 18
#define pwm2 19
#define pwm3 15
#define pwm4 25

#define motor1 14
#define motor2 13
#define motor3 27
#define motor4 22

// Encoder pins
#define encoderY_A 21
#define encoderY_B 23
#define encoderX_A 5
#define encoderX_B 26

volatile long y_ticks = 0;  // Encoder tick count for Y direction
volatile long x_ticks = 0;  // Encoder tick count for X direction

const int maxSpeed = 100;       // Maximum speed (0-255)
const int targetYTicks = 10000; // Target distance in Y direction encoder ticks
const int targetXTicks = 5000;  // Target distance in X direction encoder ticks
const float accelRate = 2;    // Acceleration rate (ticks per ms^2)
const float decelRate = 2;    // Deceleration rate (ticks per ms^2)

void setup() {
  Serial.begin(115200);
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);

  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(pwm4, OUTPUT);

  pinMode(encoderY_A, INPUT_PULLUP);
  pinMode(encoderY_B, INPUT_PULLUP);
  pinMode(encoderX_A, INPUT_PULLUP);
  pinMode(encoderX_B, INPUT_PULLUP);

  // Attach interrupt for Encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderY_A), ai0_2, RISING);  // Encoder Y pin A interrupt
  attachInterrupt(digitalPinToInterrupt(encoderY_B), ai1_2, RISING);  // Encoder Y pin B interrupt
  attachInterrupt(digitalPinToInterrupt(encoderX_A), ai0_2X, RISING); // Encoder X pin A interrupt
  attachInterrupt(digitalPinToInterrupt(encoderX_B), ai1_2X, RISING); // Encoder X pin B interrupt


  ps5.begin("48:18:8D:61:1A:E9");  //replace with MAC address of your controller
  Serial.println("Ready.");
}

void loop() {
  //  while (ps5.isConnected() == false) { // commented out as ps5 controller seems to connect quicker when microcontroller is doing nothing
  //    Serial.println("PS5 controller not found");
  //    delay(300);
  //  }

  while (ps5.isConnected() == true) {
    if (ps5.Right()) {
      digitalWrite(motor1, LOW);
      digitalWrite(motor2, LOW);
      digitalWrite(motor3, LOW);
      digitalWrite(motor4, LOW);
      Serial.println("MOVING LEFT");
    }
    if (ps5.Down()) {
      digitalWrite(motor1, HIGH);
      digitalWrite(motor2, LOW);
      digitalWrite(motor3, HIGH);
      digitalWrite(motor4, LOW);
      Serial.println("MOVING FORWARD");
    }
    if (ps5.Up()) {
      digitalWrite(motor1, LOW);
      digitalWrite(motor2, HIGH);
      digitalWrite(motor3, LOW);
      digitalWrite(motor4, HIGH);
      Serial.println("MOVING BACKWARD");
    }
    if (ps5.Left()) {
      digitalWrite(motor1, HIGH);
      digitalWrite(motor2, HIGH);
      digitalWrite(motor3, HIGH);
      digitalWrite(motor4, HIGH);
      Serial.println("MOVING RIGHT");
    }
    if (ps5.Square()) {
      static bool movingY = true;  // State variable for moving in Y direction
      static bool movingX = false; // State variable for moving in X direction

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
          delay(1000); // Short delay to simulate turning

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
        }
      }
    }

    // if (ps5.Cross()) Serial.println("Cross Button");
    // if (ps5.Circle()) Serial.println("Circle Button");
    // if (ps5.Triangle()) Serial.println("Triangle Button");

    // if (ps5.UpRight()) Serial.println("Up Right");
    // if (ps5.DownRight()) Serial.println("Down Right");
    // if (ps5.UpLeft()) Serial.println("Up Left");
    // if (ps5.DownLeft()) Serial.println("Down Left");

    if (ps5.L1()) {
      digitalWrite(motor1, HIGH);
      digitalWrite(motor2, HIGH);
      digitalWrite(motor3, LOW);
      digitalWrite(motor4, LOW);
      analogWrite(pwm1, 50);
      analogWrite(pwm2, 50);
      analogWrite(pwm3, 50);
      analogWrite(pwm4, 50);
      Serial.println("ROTATING LEFT");
    }
    if (ps5.R1()) {
      digitalWrite(motor1, LOW);
      digitalWrite(motor2, LOW);
      digitalWrite(motor3, HIGH);
      digitalWrite(motor4, HIGH);
      analogWrite(pwm1, 50);
      analogWrite(pwm2, 50);
      analogWrite(pwm3, 50);
      analogWrite(pwm4, 50);
      Serial.println("ROTATING LEFT");
    }


    // if (ps5.Share()) Serial.println("Share Button");
    // if (ps5.Options()) Serial.println("Options Button");
    // if (ps5.L3()) Serial.println("L3 Button");
    // if (ps5.R3()) Serial.println("R3 Button");

    // if (ps5.PSButton()) Serial.println("PS Button");
    // if (ps5.Touchpad()) Serial.println("Touch Pad Button");

    // if (ps5.L2()) {
    //   Serial.printf("L2 button at %d\n", ps5.L2Value());
    // }
    // if (ps5.R2()) {
    //   Serial.printf("R2 button at %d\n", ps5.R2Value());
    // }

    // if (ps5.LStickX()) {
    //   // Serial.printf("Left Stick x at %d\n", ps5.LStickX());
    //   int lsmotor = map(ps5.LStickX(), 0, 124, 0, 150);
    //   if (lsmotor >= 0) {  //Turn Right
    //     if (lsmotor > 0 && lsmotor < 30) {
    //       lsmotor = 0;
    //       analogWrite(pwm1, lsmotor);
    //       analogWrite(pwm2, lsmotor);
    //       analogWrite(pwm3, lsmotor);
    //       analogWrite(pwm4, lsmotor);
    //       Serial.println("Stopped");
    //     } else if (lsmotor > 31 && lsmotor <= 160) {


    //       analogWrite(pwm1, lsmotor);
    //       analogWrite(pwm2, lsmotor);
    //       analogWrite(pwm3, lsmotor);
    //       analogWrite(pwm4, lsmotor);
    //     }
    //   } else {  //Turn Left
    //     int lsmotorneg = lsmotor * (-1);

    //     if (lsmotorneg > 0 && lsmotorneg < 30) {
    //       lsmotorneg = 0;
    //       analogWrite(pwm1, lsmotorneg);
    //       analogWrite(pwm2, lsmotorneg);
    //       analogWrite(pwm3, lsmotorneg);
    //       analogWrite(pwm4, lsmotorneg);
    //       Serial.println("Stopped");
    //     } else if (lsmotorneg > 31 && lsmotorneg <= 160) {


    //       analogWrite(pwm1, lsmotorneg);
    //       analogWrite(pwm2, lsmotorneg);
    //       analogWrite(pwm3, lsmotorneg);
    //       analogWrite(pwm4, lsmotorneg);
    //     }
    //   }
    //   // Serial.println(lsmotor);
    // }
    // if (ps5.LStickY()) {
    //   // Serial.printf("Left Stick y at %d\n", ps5.LStickY());
    // }
    // if (ps5.RStickX()) {
    //   // Serial.printf("Right Stick x at %d\n", ps5.RStickX());
    // }
    if (ps5.RStickY()) {
      int rsmotor = map(ps5.RStickY(), 0, 124, 0, 150);
      if (rsmotor < 10) {
        rsmotor = 0;
      }
      analogWrite(pwm1, rsmotor);
      analogWrite(pwm2, rsmotor);
      analogWrite(pwm3, rsmotor);
      analogWrite(pwm4, rsmotor);
      Serial.println(rsmotor);
      // Serial.println(rsmotor);
    }
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
  if(currentTicks<240){
    currentSpeed = 30;
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

void setMotorSpeedRight(int speed) {
  analogWrite(pwm1, speed);
  digitalWrite(motor1, LOW);  // Ensure direction pin is set

  analogWrite(pwm2, speed);
  digitalWrite(motor2, HIGH);  // Ensure direction pin is set

  analogWrite(pwm3, speed);
  digitalWrite(motor3, LOW);  // Ensure direction pin is set

  analogWrite(pwm4, speed);
  digitalWrite(motor4, HIGH);  // Ensure direction pin is set
}

void setMotorSpeedLeft(int speed) {
  analogWrite(pwm1, speed);
  digitalWrite(motor1, HIGH);  // Ensure direction pin is set

  analogWrite(pwm2, speed);
  digitalWrite(motor2, LOW);  // Ensure direction pin is set

  analogWrite(pwm3, speed);
  digitalWrite(motor3, HIGH);  // Ensure direction pin is set

  analogWrite(pwm4, speed);
  digitalWrite(motor4, LOW);  // Ensure direction pin is set
}\
