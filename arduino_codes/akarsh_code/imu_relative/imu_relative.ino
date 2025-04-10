#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // I2C address

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Initializing BNO055 in IMU mode (no magnetometer)...");

  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring.");
    while (1);
  }

  delay(1000);

  // Set to IMU mode (accel + gyro only)
  bno.setMode(OPERATION_MODE_IMUPLUS);

  bno.setExtCrystalUse(true);
}

void loop() {
  // Get fused orientation (quaternion)
  imu::Quaternion quat = bno.getQuat();

  // Get angular velocity (gyro) in rad/s
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // Get linear acceleration (accel) in m/s^2
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  Serial.print(
    String(quat.w(), 4) + "," +
    String(quat.x(), 4) + "," +
    String(quat.y(), 4) + "," +
    String(quat.z(), 4) + "," +
    String(gyro.x(), 4) + "," +
    String(gyro.y(), 4) + "," +
    String(gyro.z(), 4) + "," +
    String(accel.x(), 4) + "," +
    String(accel.y(), 4) + "," +
    String(accel.z(), 4) + "\r\n"
  );


  delay(50); // 20Hz
}
