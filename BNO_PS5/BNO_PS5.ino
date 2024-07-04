#include <Arduino.h>
// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give
// quaternion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.
// Note sensorValue.status gives calibration accuracy (which improves over time)
#include <Adafruit_BNO08x.h>
#include <ps5Controller.h>

#define pwm1 18
#define pwm2 19
#define pwm3 15
#define pwm4 25

#define motor1 14
#define motor2 13
#define motor3 27
#define motor4 12
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

float initialYaw = 0;
bool initialYawSet = false;
int ref_yaw = 90;
int yaw;

bool consider_bno = true;
bool right = false;
bool left = false;
int speed;

bool flag_motor_1 = false;
bool flag_motor_2 = false;

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void setup(void) {

  Serial.begin(115200);
  ps5.begin("48:18:8D:61:1A:E9");  //replace with MAC address of your controller
  Serial.println("PS5 Ready.");
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);

  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(pwm4, OUTPUT);


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

void normalizeYaw(float* yaw) {
  // Start with an initial yaw of 90 degrees
  *yaw -= initialYaw;
  *yaw = 90 - *yaw;  // Adjust to start at 90 and decrease to 0 on anticlockwise rotation
  if (*yaw > 180) *yaw -= 360;
  if (*yaw < -180) *yaw += 360;
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
        break;
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }

    if (!initialYawSet) {
      initialYaw = ypr.yaw;
      initialYawSet = true;
    }
    normalizeYaw(&ypr.yaw);
    yaw = ypr.yaw;
    if (yaw<92 & yaw> 88) {
      right = false;
      left = false;
    }
    if (yaw > 92) {
      right = true;
      left = false;
    }
    if (yaw < 88) {
      right = false;
      left = true;
    }
  }
  if (ps5.Right()) {
    digitalWrite(motor1, HIGH);
    digitalWrite(motor2, LOW);
    digitalWrite(motor3, HIGH);
    digitalWrite(motor4, LOW);
    bool consider_bno = true;
    flag_motor_2 = true;
    flag_motor_1 = false;
  }
  if (ps5.Down()) {
    digitalWrite(motor1, LOW);
    digitalWrite(motor2, LOW);
    digitalWrite(motor3, LOW);
    digitalWrite(motor4, LOW);
    bool consider_bno = true;
    flag_motor_1 = true;
    flag_motor_2 = false;
  }
  if (ps5.Up()) {
    digitalWrite(motor1, HIGH);
    digitalWrite(motor2, HIGH);
    digitalWrite(motor3, HIGH);
    digitalWrite(motor4, HIGH);
    bool consider_bno = true;
    flag_motor_1 = false;
    flag_motor_2 = true;
  }
  if (ps5.Left()) {
    digitalWrite(motor1, LOW);
    digitalWrite(motor2, HIGH);
    digitalWrite(motor3, LOW);
    digitalWrite(motor4, HIGH);
    bool consider_bno = true;
    flag_motor_1 = true;
    flag_motor_2 = false;
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
    bool consider_bno = false;
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
    bool consider_bno = false;
  }
  if (ps5.RStickY()) {
    speed = map(ps5.RStickY(), 0, 124, 0, 150);
    if (speed < 10) {
      speed = 10;
    }
  }
  if (consider_bno) {
    if (flag_motor_1) {
      if (right) {
        motor_control_1(speed, speed);
      }
      if (left) {
        motor_control_1(speed, speed - 10);
      }
      if (!left & !right) {
        motor_control_1(speed, -5);
      }
    }
    if (flag_motor_2) {
      if (right) {
        motor_control_2(speed, -10);
      }
      if (left) {
        motor_control_2(speed, speed);
      }
      if (!left & !right) {
        motor_control_2(speed, -5);
      }
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
