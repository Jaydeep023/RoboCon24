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


const int trigPin = 4;
const int echoPin = 38;

// Defines variables
long duration;
int distance;  // Interval for stopping the motor (10 seconds)
unsigned long startTime;
void setup() {

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


  Serial.begin(9600);  // Starts the serial communication
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - startTime;


  // Clears the trigPin
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

  // Prints the distance on the Serial Monitor
  // Move forward if distance is greater than 38
  if (elapsedTime < 13000) {
    if (distance > 35) {
      digitalWrite(DIR_M1, HIGH);
      digitalWrite(DIR_M2, HIGH);
      digitalWrite(DIR_M3, HIGH);
      digitalWrite(DIR_M4, HIGH);

      analogWrite(PWM_M1, 100);
      analogWrite(PWM_M2, 100);
      analogWrite(PWM_M3, 100);
      analogWrite(PWM_M4, 100);
    } else {      
      digitalWrite(DIR_M1, HIGH);
      digitalWrite(DIR_M2, LOW);
      digitalWrite(DIR_M3, HIGH);
      digitalWrite(DIR_M4, LOW);

      analogWrite(PWM_M1, 100);
      analogWrite(PWM_M2, 100);
      analogWrite(PWM_M3, 100);
      analogWrite(PWM_M4, 100);
    }
  } else {
    analogWrite(PWM_M1, 0);
    analogWrite(PWM_M2, 0);
    analogWrite(PWM_M3, 0);
    analogWrite(PWM_M4, 0);
  }
}
