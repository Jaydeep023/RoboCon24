#include <util/atomic.h>  // For the ATOMIC_BLOCK macro

#define ENCA_M1 2  // Motor 1 Encoder Pin A
#define ENCB_M1 3  // Motor 1 Encoder Pin B
#define PWM_M1 6   // Motor 1 PWM Pin
#define DIR_M1 36  // Motor 1 Direction Pin

#define ENCA_M2 18  //  Motor 2 Encoder Pin A
#define ENCB_M2 19  // Motor 2 Encoder Pin B
#define PWM_M2 11   // Motor 2 PWM Pin
#define DIR_M2 12   // Motor 2 Direction Pin

#define PWM_M3 8   // Motor 3 PWM Pin
#define DIR_M3 30  // Motor 3 Direction Pin

#define PWM_M4 9   // Motor 4 PWM Pin
#define DIR_M4 33  // Motor 4 Direction Pin

volatile int posi_M1 = 0;  // Encoder count for Motor 1
int target = 15000;
const int trigPin = 4;
const int echoPin = 38;

long duration;
int distance;  // Interval for stopping the motor (10 seconds)
unsigned long startTime;

bool ultrasonicFlag = true;
bool horizontalEncoderFlag = false;

int t1 = target - 50;
int t2 = target + 50;
int pos = 0;
void setup() {
  pinMode(ENCA_M1, INPUT_PULLUP);
  pinMode(ENCB_M1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(18), ai0_1, RISING);  // Encoder 1 pin A interrupt
  attachInterrupt(digitalPinToInterrupt(19), ai1_1, RISING);
  startTime = millis();
  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);   // Sets the echoPin as an Input
  pinMode(PWM_M1, OUTPUT);
  pinMode(DIR_M1, OUTPUT);

  // Motor 2 setup
  pinMode(PWM_M2, OUTPUT);
  pinMode(DIR_M2, OUTPUT);

  // Motor 3 setup
  pinMode(PWM_M3, OUTPUT);
  pinMode(DIR_M3, OUTPUT);

  // Motor 4 setup
  pinMode(PWM_M4, OUTPUT);
  pinMode(DIR_M4, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - startTime;
  // Clears the trigPin
  if (elapsedTime < 10) {
    if (ultrasonicFlag) {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);

      // Sets the trigPin on HIGH state for 10 microseconds
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = pulseIn(echoPin, HIGH);

      // Calculating the distance
      distance = duration * 0.034 / 2;
      if (distance > 35) {
        digitalWrite(DIR_M1, HIGH);
        digitalWrite(DIR_M2, HIGH);
        digitalWrite(DIR_M3, HIGH);
        digitalWrite(DIR_M4, HIGH);

        analogWrite(PWM_M1, 155);
        analogWrite(PWM_M2, 155);
        analogWrite(PWM_M3, 155);
        analogWrite(PWM_M4, 155);
      } else {
        ultrasonicFlag = false;
        horizontalEncoderFlag = true;
      }
    }
    if (horizontalEncoderFlag) {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        pos = posi_M1;
      }
      int e = target - pos;
      int pwr = e;

      pwr = map(pwr, 0, target, 20, 205);
      if (pwr > 205) {
        pwr = 205;
      }
      if (pos < t1) {
        digitalWrite(DIR_M1, HIGH);
        digitalWrite(DIR_M2, LOW);
        digitalWrite(DIR_M3, HIGH);
        digitalWrite(DIR_M4, LOW);

        analogWrite(PWM_M1, pwr);
        analogWrite(PWM_M2, pwr);
        analogWrite(PWM_M3, pwr);
        analogWrite(PWM_M4, pwr);
      }
      if (pos > t1 && pos < t2) {
        analogWrite(PWM_M1, 0);
        analogWrite(PWM_M2, 0);
        analogWrite(PWM_M3, 0);
        analogWrite(PWM_M4, 0);
      } else {
        horizontalEncoderFlag = false;
      }
    }
  } else {
    analogWrite(PWM_M1, 0);
    analogWrite(PWM_M2, 0);
    analogWrite(PWM_M3, 0);
    analogWrite(PWM_M4, 0);
  }
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