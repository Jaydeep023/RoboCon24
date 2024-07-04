#include <ps5Controller.h>

#define pwm1 18
#define pwm2 19
#define pwm3 15
#define pwm4 25

#define motor1 14
#define motor2 13
#define motor3 27
#define motor4 12

bool Slow_1_mtr = false;
bool Slow_2_mtr = true;

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


  ps5.begin("48:18:8D:61:1A:E9");  //replace with MAC address of your controller
  Serial.println("Ready.");
}

void loop() {
  while (ps5.isConnected() == true) {
    if (ps5.Right()) {
      digitalWrite(motor1, HIGH);
      digitalWrite(motor2, LOW);
      digitalWrite(motor3, HIGH);
      digitalWrite(motor4, LOW);
      Slow_2_mtr = true;
      Slow_1_mtr = false;
    }
    if (ps5.Down()) {
      digitalWrite(motor1, LOW);
      digitalWrite(motor2, LOW);
      digitalWrite(motor3, LOW);
      digitalWrite(motor4, LOW);
      Slow_2_mtr = false;
      Slow_1_mtr = true;
    }
    if (ps5.Up()) {
      digitalWrite(motor1, HIGH);
      digitalWrite(motor2, HIGH);
      digitalWrite(motor3, HIGH);
      digitalWrite(motor4, HIGH);
      Slow_2_mtr = true;
      Slow_1_mtr = false;
    }
    if (ps5.Left()) {
      digitalWrite(motor1, LOW);
      digitalWrite(motor2, HIGH);
      digitalWrite(motor3, LOW);
      digitalWrite(motor4, HIGH);
      Slow_2_mtr = false;
      Slow_1_mtr = true;
    }
    if (ps5.L1()) {
      digitalWrite(motor1, LOW);
      digitalWrite(motor2, HIGH);
      digitalWrite(motor3, HIGH);
      digitalWrite(motor4, LOW);
      analogWrite(pwm1, 50);
      analogWrite(pwm2, 50);
      analogWrite(pwm3, 50);
      analogWrite(pwm4, 50);
      Serial.println("ROTATING LEFT");
    }
    if (ps5.R1()) {
      digitalWrite(motor1, HIGH);
      digitalWrite(motor2, LOW);
      digitalWrite(motor3, LOW);
      digitalWrite(motor4, HIGH);
      analogWrite(pwm1, 50);
      analogWrite(pwm2, 50);
      analogWrite(pwm3, 50);
      analogWrite(pwm4, 50);
      Serial.println("ROTATING RIGHT");
    }
    if (ps5.RStickY()) {
      int rsmotor = map(ps5.RStickY(), 0, 124, 0, 150);
      if (rsmotor < 10) {
        rsmotor = 10;
      }
    }
    if(Slow_1_mtr){
        motor_control_1(rsmotor,-5)
    }
    if(Slow_2_mtr){
        motor_control_2(rsmotor,-5)
    }
  }
}
void motor_control_1(int sp, int change) {
  analogWrite(pwm1, sp + change);
  analogWrite(pwm2, sp);
  analogWrite(pwm3, sp);
  analogWrite(pwm4, sp);
}
void motor_control_2(int sp, int change) {
  analogWrite(pwm1, sp);
  analogWrite(pwm2, sp + change);
  analogWrite(pwm3, sp);
  analogWrite(pwm4, sp);
}
