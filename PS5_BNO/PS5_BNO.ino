#include <ps5Controller.h>
#include <Arduino.h>
#include <Adafruit_BNO08x.h>

#define BNO08X_ADDRESS 0x4A


#define pwm1 18
#define pwm2 19
#define pwm3 15
#define pwm4 25

#define motor1 14
#define motor2 13
#define motor3 27
#define motor4 12

int speed;
int reference_yaw;

bool get_first = false;

bool ClockWise = false;
bool Anti_ClockWise = false;

bool horizontal = false;
bool verticle = false;

int Diagonal_1;
int Diagonal_2;
int diff_Yaw;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x bno08x;

sh2_SensorValue_t sensorValue;

sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void setup() {
  Serial.begin(115200);
  ps5.begin("48:18:8D:61:1A:E9");  //replace with MAC address of your controller
  Serial.println("Ready.");
  Serial.println("Adafruit BNO08x test!");

  if (!bno08x.begin_I2C(BNO08X_ADDRESS)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BNO08x Found!");

  setReports(reportType, reportIntervalUs);

  Serial.println("Reading events");

  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);

  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(pwm4, OUTPUT);
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
    Serial.print("Sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        break;
      case SH2_GYRO_INTEGRATED_RV:
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
  }

  if (!get_first) {
    reference_yaw = ypr.yaw;
    get_first = true;
  }

  diff_Yaw = fabs(reference_yaw) - fabs(ypr.yaw);
  if (ypr.yaw > reference_yaw + 5) {
    diff_Yaw = fabs(diff_Yaw);
    ClockWise = true;
    Anti_ClockWise = false;

  } else if (ypr.yaw < reference_yaw - 5) {
    diff_Yaw = fabs(diff_Yaw);
    ClockWise = false;
    Anti_ClockWise = true;
  } else {
    ClockWise = false;
    Anti_ClockWise = false;
  }

  if (ps5.Right()) {
    digitalWrite(motor1, HIGH);
    digitalWrite(motor2, LOW);
    digitalWrite(motor3, HIGH);
    digitalWrite(motor4, LOW);
    get_first = false;
    horizontal = true;
    verticle = false;
    // Serial.println("MOVING RIGHT");
  }
  if (ps5.Down()) {
    digitalWrite(motor1, LOW);
    digitalWrite(motor2, LOW);
    digitalWrite(motor3, LOW);
    digitalWrite(motor4, LOW);
    get_first = false;
    horizontal = false;
    verticle = true;
    // Serial.println("MOVING BACKWARD");
  }
  if (ps5.Up()) {
    digitalWrite(motor1, HIGH);
    digitalWrite(motor2, HIGH);
    digitalWrite(motor3, HIGH);
    digitalWrite(motor4, HIGH);
    get_first = false;
    horizontal = false;
    verticle = true;
    // Serial.println("MOVING FORWARD");
  }
  if (ps5.Left()) {
    digitalWrite(motor1, LOW);
    digitalWrite(motor2, HIGH);
    digitalWrite(motor3, LOW);
    digitalWrite(motor4, HIGH);
    get_first = false;
    horizontal = true;
    verticle = false;
    // Serial.println("MOVING LEFT");
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
    // Serial.println("ROTATING LEFT");
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
    // Serial.println("ROTATING RIGHT");
  }

  if (ps5.RStickY()) {
    speed = map(ps5.RStickY(), 0, 124, 0, 150);
    if (speed < 10) {
      speed = 0;
    }
  }
  if (ClockWise) {
    Diagonal_1 = speed;
    Diagonal_2 = speed - 25;
  } else if (Anti_ClockWise) {
    Diagonal_2 = speed;
    Diagonal_1 = speed - 25;
  } else {
    Diagonal_1 = speed;
    Diagonal_2 = speed;
  }
  if (Diagonal_1 < 0) {
    Diagonal_1 = 0;
  }
  if (Diagonal_2 < 0) {
    Diagonal_2 = 0;
  }
  if (verticle) {
    if (ClockWise) {
      motors_1_3(Diagonal_2);
      motors_2_4(Diagonal_1);
    } else if (Anti_ClockWise) {
      motors_2_4(Diagonal_1);
      motors_1_3(Diagonal_2);
    } else {
      motors_2_4(Diagonal_1);
      motors_1_3(Diagonal_2);
    }
  }
  if (horizontal) {
    if (ClockWise) {
      motors_1_3(Diagonal_1);
      motors_2_4(Diagonal_2);
    } else if (Anti_ClockWise) {
      motors_2_4(Diagonal_2);
      motors_1_3(Diagonal_1);
    } else {
      motors_2_4(Diagonal_1);
      motors_1_3(Diagonal_2);
    }
  }
  Serial.print(" Diagonal_1 : ");
  Serial.print(Diagonal_1);
  Serial.println();
  Serial.print(" Diagonal_2 : ");
  Serial.print(Diagonal_2);
  Serial.println();
}
void motors_1_3(int dgspeed) {
  analogWrite(pwm1, dgspeed);
  analogWrite(pwm3, dgspeed);
}
void motors_2_4(int dgspeed) {
  analogWrite(pwm2, dgspeed);
  analogWrite(pwm4, dgspeed);
}