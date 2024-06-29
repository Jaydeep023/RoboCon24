#include <Arduino.h>
// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.
// Note sensorValue.status gives calibration accuracy (which improves over time)
#include <Adafruit_BNO08x.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9


// #define FAST_MODE

// For SPI mode, we also need a RESET
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
// Top frequency is reported to be 1000Hz (but freq is somewhat variable)
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
long reportIntervalUs = 2000;
#else
// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;
#endif

bool firstRef = false;
float Reference_Yaw;
float diff_Yaw;

// Motor PWM pins
#define pwm1 18
#define pwm2 19
#define pwm3 15
#define pwm4 25

// Motor control pins
#define motor1 14  //Backward Right
#define motor2 13  //Backward Left
#define motor3 27  //Forward Left
#define motor4 12  //Forward Right

// Encoder pins
#define encoderY_A 23
#define encoderY_B 2
#define encoderX_A 5
#define encoderX_B 26

volatile long y_ticks = 0;  // Encoder tick count for Y direction
volatile long x_ticks = 0;  // Encoder tick count for X direction

const int maxSpeed = 100;        // Maximum speed (0-255)
const int targetYTicks = 10000;  // Target distance in Y direction encoder ticks
const int targetXTicks = 5000;   // Target distance in X direction encoder ticks
const float accelRate = 2;       // Acceleration rate (ticks per ms^2)
const float decelRate = 2;       // Deceleration rate (ticks per ms^2)

float V = 100;

float x_coord = 12000;  //X Coordinare Here
float y_coord = 20000;  //Y Coordinare Here
float theta;
float Vx, Vy;
float Vfl, Vfr, Vrr, Vrl;

bool ClockWise = false;
bool Anti_ClockWise = false;

int ClockWise_Diag_Speed;
int Anti_ClockWise_Diag_Speed;

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void setup(void) {
  Serial.begin(115200);
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
  while (!Serial) delay(10);                                           // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
    //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");


  setReports(reportType, reportIntervalUs);

  Serial.println("Reading events");
  delay(100);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void loop() {

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    static long last = 0;
    long now = micros();
    last = now;
    if (!firstRef) {
      Reference_Yaw = ypr.yaw;
      firstRef = true;
    }
    Serial.print("Reference Yaw: ");
    Serial.println(Reference_Yaw);
    // Serial.print("Current Yaw: ");
    // Serial.println(ypr.yaw);
    diff_Yaw = fabs(Reference_Yaw) - fabs(ypr.yaw);
    if (ypr.yaw > Reference_Yaw + 5) {
      // Serial.print("Distance On Left: ");
      // Serial.print(fabs(diff_Yaw));
      diff_Yaw = fabs(diff_Yaw);
      // Serial.println();
      ClockWise = true;
      Anti_ClockWise = false;

    } else if (ypr.yaw < Reference_Yaw - 5) {
      // Serial.print("Distance On Right: ");
      // Serial.print(fabs(diff_Yaw));
      diff_Yaw = fabs(diff_Yaw);
      // Serial.println();
      ClockWise = false;
      Anti_ClockWise = true;
    }
    else{
      ClockWise = false;
      Anti_ClockWise = false;

    }
  }
  Serial.print("X Ticks: ");
  Serial.println(x_ticks);
  long currentXTicks = x_ticks;
  int speed = calculateSpeed(currentXTicks, targetXTicks, maxSpeed, accelRate, decelRate);
  Vy = speed * sin(diff_Yaw);
  Vx = speed * cos(diff_Yaw);

  // Diagonal TO Pivot Clockwise Motor1 and Motor3
  // Diagonal TO Pivot Anti - Clockwise Motor4 and Motor2

  ClockWise_Diag_Speed = Vy + Vx;
  Anti_ClockWise_Diag_Speed = Vy - Vx;
  ClockWise_Diag_Speed = fabs(ClockWise_Diag_Speed);
  Anti_ClockWise_Diag_Speed = fabs(Anti_ClockWise_Diag_Speed);
  
  Serial.print("Speed: ");
  Serial.println(speed);
  if(ClockWise){
  Serial.print("ClockWise_Diag_Speed : ");
  Serial.print(ClockWise_Diag_Speed);
  Serial.println();
  setMotorSpeedForward_1_3(ClockWise_Diag_Speed);
  }
  if(Anti_ClockWise){
  Serial.print("Anti ClockWise_Diag_Speed : ");
  Serial.print(Anti_ClockWise_Diag_Speed);
  Serial.println();
  setMotorSpeedForward_2_4(Anti_ClockWise_Diag_Speed);
  }
  if(!Anti_ClockWise & !ClockWise){
  Serial.println("GOing Straight");
  setMotorSpeedForward_1_3(speed);
  setMotorSpeedForward_2_4(speed);
  }
  delay(100);
  // setMotorSpeedForward_2_4()
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
void setMotorSpeedForward_1_3(int speed) {
  analogWrite(pwm1, speed);
  digitalWrite(motor1, LOW);  // Ensure direction pin is set

  analogWrite(pwm3, speed);
  digitalWrite(motor3, LOW);  // Ensure direction pin is set

}
void setMotorSpeedForward_2_4(int speed) {
  analogWrite(pwm2, speed);
  digitalWrite(motor2, LOW);  // Ensure direction pin is set

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
}
