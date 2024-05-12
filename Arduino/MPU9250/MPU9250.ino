#include <Wire.h>

const int MPU_addr = 0x68;   // I2C address of the MPU-9250
int16_t GyZ;                  // Raw gyroscope reading along the X-axis
float angle = 0.0;            // Accumulated pitch angle in degrees
float prev_time = 0.0;        // Previous time stamp for integration

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Set to zero (wakes up the MPU-9250)
  Wire.endTransmission(true);
}

void loop() {
  mpu_read();
  delay(100); // Adjust delay as needed to control sampling rate
}

void mpu_read() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x47);  // Starting with register 0x43 (GYRO_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);  // Request a total of 6 registers for gyroscope readings

  GyZ = Wire.read() << 8 | Wire.read(); // Combine high and low bytes for GyX
  // Convert raw GyX to degrees per second (sensitivity: 131 LSB/(°/s))
  float gyroZ_deg_per_sec = GyZ / 131.0;
  
  // Calculate time interval since last integration
  float current_time = millis() / 1000.0; // Convert milliseconds to seconds
  float dt = current_time - prev_time;
  prev_time = current_time;

  // Integrate angular velocity to obtain angle (using the trapezoidal rule)
  angle += gyroZ_deg_per_sec * dt;
  
  angle = fmod(angle, 360.0) ;
  if(angle<0){
    angle = 360;
    }
  Serial.println(angle);
}