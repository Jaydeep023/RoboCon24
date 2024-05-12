#include <util/atomic.h>  // For the ATOMIC_BLOCK macro

#define ENCA_M1 2  // Motor 1 Encoder Pin A
#define ENCB_M1 3  // Motor 1 Encoder Pin B
#define PWM_M1 6   // Motor 1 PWM Pin
#define DIR_M1 36  // Motor 1 Direction Pin

#define ENCA_M2 18  //  Motor 2 Encoder Pin A
#define ENCB_M2 19  // Motor 2 Encoder Pin B
#define PWM_M2 11    // Motor 2 PWM Pin
#define DIR_M2 12   // Motor 2 Direction Pin

#define PWM_M3 8   // Motor 3 PWM Pin
#define DIR_M3 30  // Motor 3 Direction Pin

#define PWM_M4 9   // Motor 4 PWM Pin
#define DIR_M4 33  // Motor 4 Direction Pin

unsigned long startTime;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Motor 1 setup
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

  startTime = millis();  // Record the start time
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - startTime;

  if (elapsedTime < 2735) { // Run the motors for 2 seconds
    digitalWrite(DIR_M1, HIGH);
    digitalWrite(DIR_M2, HIGH);
    digitalWrite(DIR_M3, HIGH);
    digitalWrite(DIR_M4, HIGH);

    analogWrite(PWM_M1, 255);
    analogWrite(PWM_M2, 255);
    analogWrite(PWM_M3, 255);
    analogWrite(PWM_M4, 255);
  } else if(elapsedTime > 2735 && elapsedTime < 4730){
    digitalWrite(DIR_M1, HIGH);
    digitalWrite(DIR_M2, LOW);
    digitalWrite(DIR_M3, HIGH);
    digitalWrite(DIR_M4, LOW);

    analogWrite(PWM_M1, 255);
    analogWrite(PWM_M2, 255);
    analogWrite(PWM_M3, 255);
    analogWrite(PWM_M4, 255);
  }
    else if(elapsedTime > 4730 && elapsedTime < 6300){
    digitalWrite(DIR_M1, HIGH);
    digitalWrite(DIR_M2, HIGH);
    digitalWrite(DIR_M3, HIGH);
    digitalWrite(DIR_M4, HIGH);

    analogWrite(PWM_M1, 255);
    analogWrite(PWM_M2, 255);
    analogWrite(PWM_M3, 255);
    analogWrite(PWM_M4, 255);
    } else{
    analogWrite(PWM_M1, 0);  // Stop the motors
    analogWrite(PWM_M2, 0);
    analogWrite(PWM_M3, 0);
    analogWrite(PWM_M4, 0);
  }
}

