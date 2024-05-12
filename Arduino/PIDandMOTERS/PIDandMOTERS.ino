#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA_M1 2 // Motor 1 Encoder Pin A
#define ENCB_M1 3 // Motor 1 Encoder Pin B
#define PWM_M1 6  // Motor 1 PWM Pin
#define DIR_M1 36  // Motor 1 Direction Pin

#define ENCA_M2 18 //  Motor 2 Encoder Pin A
#define ENCB_M2 19 // Motor 2 Encoder Pin B
#define PWM_M2 7  // Motor 2 PWM Pin
#define DIR_M2 10 // Motor 2 Direction Pin

#define PWM_M3 8 // Motor 3 PWM Pin
#define DIR_M3 30 // Motor 3 Direction Pin

#define PWM_M4 9 // Motor 4 PWM Pin
#define DIR_M4 33 // Motor 4 Direction Pin

volatile int posi_M1 = 0; // Encoder count for Motor 1
volatile int posi_M2 = 0; // Encoder count for Motor 2

long prevT = 0;    // Previous time
float eprev_M1 = 0, eprev_M2 = 0; // Previous error for each motor
float eintegral_M1 = 0, eintegral_M2 = 0; // Integral error for each motor

void setup() {
  Serial.begin(115200);
  
  // Motor 1 setup
  pinMode(ENCA_M1, INPUT_PULLUP);
  pinMode(ENCB_M1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), ai0_1, RISING); // Encoder 1 pin A interrupt
  attachInterrupt(digitalPinToInterrupt(3), ai1_1, RISING); 
  pinMode(PWM_M1, OUTPUT);
  pinMode(DIR_M1, OUTPUT);

  // Motor 2 setup
  pinMode(ENCA_M2, INPUT);
  pinMode(ENCB_M2, INPUT);
  // attachInterrupt(digitalPinToInterrupt(ENCA_M2), readEncoder_M2, RISING);
  pinMode(PWM_M2, OUTPUT);
  pinMode(DIR_M2, OUTPUT);

  // Motor 3 setup
  pinMode(PWM_M3, OUTPUT);
  pinMode(DIR_M3, OUTPUT);

  // Motor 4 setup
  pinMode(PWM_M4, OUTPUT);
  pinMode(DIR_M4, OUTPUT);

  Serial.println("target pos");
}

void loop() {

  int target = 29000; // Example target calculation, replace with your own logic

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // Time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi_M1;
  }

  // Error
  int e = pos - target;

  // Derivative
  float dedt = (e - eprev_M1) / (deltaT);

  // Integral
  eintegral_M1 = eintegral_M1 + e * deltaT;

  // Control signal
  float u = kp * e + kd * dedt + ki * eintegral_M1;

  // Motor power
  float pwr = fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }

  // Motor direction
  int motorDir = 1;
  if (u < 0) {
    motorDir = -1;
  }
  if (motorDir == 1) {
    digitalWrite(DIR_M1, HIGH);
    digitalWrite(DIR_M2, HIGH);
    digitalWrite(DIR_M3, HIGH);
    digitalWrite(DIR_M4, HIGH);
    
    analogWrite(PWM_M1, pwr);
    analogWrite(PWM_M2, pwr);
    analogWrite(PWM_M3, pwr);
    analogWrite(PWM_M4, pwr);
    
  } else if (motorDir == -1) {
    digitalWrite(DIR_M1, LOW);
    digitalWrite(DIR_M2, LOW);
    digitalWrite(DIR_M3, LOW);
    digitalWrite(DIR_M4, LOW);
    
    analogWrite(PWM_M1, pwr);
    analogWrite(PWM_M2, pwr);
    analogWrite(PWM_M3, pwr);
    analogWrite(PWM_M4, pwr);

  } else {
    analogWrite(PWM_M1, 0);
    analogWrite(PWM_M2, 0);
    analogWrite(PWM_M3, 0);
    analogWrite(PWM_M4, 0);
  }

  // Store previous error
  eprev_M1 = e;

  
  Serial.print(": Target: ");
  Serial.print(target);
  Serial.print(" Pos: ");
  Serial.println(pos);
}

void motorControl(int pwmPin, int dirPin, volatile int& position, float& prevError, float& integralError, int motorNumber) {
  // Set target position
  
}

void ai0_1() {
  // Encoder 1 pin A rising edge interrupt
  if (digitalRead(3) == LOW) {
    posi_M1++;
  } else {
    posi_M1--;
  }
}

void ai1_1() {
  // Encoder 1 pin B rising edge interrupt
  if (digitalRead(2) == LOW) {
    posi_M1--;
  } else {
    posi_M1++;
  }
}