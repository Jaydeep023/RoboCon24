#include <ps5Controller.h>

#define pwm1 14
#define pwm2 27
#define pwm3 18
#define pwm4 15

#define motor1 13
#define motor2 25
#define motor3 19
#define motor4 5

bool backward = false;
bool forward = false;

bool left = false;
bool right = false;
// bool 
int rsmotor;


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
      left = false;
      right = true;
      backward = false;
      forward = false;
    }
    if (ps5.Down()) {
      digitalWrite(motor1, LOW);
      digitalWrite(motor2, LOW);
      digitalWrite(motor3, LOW);
      digitalWrite(motor4, LOW);
      left = false;
      right = false;
      backward = true;
      forward = false;
    }
    if (ps5.Up()) {
      digitalWrite(motor1, HIGH);
      digitalWrite(motor2, HIGH);
      digitalWrite(motor3, HIGH);
      digitalWrite(motor4, HIGH);
      left = false;
      right = false;
      backward = false;
      forward = true;
    }
    if (ps5.Left()) {
      digitalWrite(motor1, LOW);
      digitalWrite(motor2, HIGH);
      digitalWrite(motor3, LOW);
      digitalWrite(motor4, HIGH);
      left = true;
      right = false;
      backward = false;
      forward = false;
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
      rsmotor = map(ps5.RStickY(), 0, 124, 0, 150);
      if (rsmotor < 20) {
        rsmotor = 0;
      }
    }
    if(right){
        motor_control_1(rsmotor,0);
    }
    if(left){
        motor_control_2(rsmotor,0);
    }
    if(forward){
        motor_control_2(rsmotor,0);
    }
    if(backward){
        motor_control_3(rsmotor,0);
    }
    
  }
}
void motor_control_1(int sp, int change) {
  if(sp<20){
    change = 0;
  }
  analogWrite(pwm1, sp + change);
  analogWrite(pwm2, sp);
  analogWrite(pwm3, sp);
  analogWrite(pwm4, sp);
}
void motor_control_2(int sp, int change) {
  if(sp<20){
    change = 0;
  }
  analogWrite(pwm1, sp);
  analogWrite(pwm2, sp + change);
  analogWrite(pwm3, sp);
  analogWrite(pwm4, sp);
}
void motor_control_3(int sp, int change) {
  if(sp<20){
    change = 0;
  }
  analogWrite(pwm1, sp);
  analogWrite(pwm2, sp);
  analogWrite(pwm3, sp + change);
  analogWrite(pwm4, sp);
}
void motor_control_4(int sp, int change) {
  if(sp<20){
    change = 0;
  }
  analogWrite(pwm1, sp);
  analogWrite(pwm2, sp);
  analogWrite(pwm3, sp);
  analogWrite(pwm4, sp + change);
}
