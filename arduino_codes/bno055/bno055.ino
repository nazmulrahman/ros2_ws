#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
    Serial.begin(115200);
    if (!bno.begin()) {
        Serial.println("BNO055 not detected");
        while (1);
    }
    delay(1000);
}

void loop() {
    sensors_event_t accel, gyro, mag;
    bno.getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&mag, Adafruit_BNO055::VECTOR_MAGNETOMETER);

    Serial.print(accel.acceleration.x); Serial.print(",");
    Serial.print(accel.acceleration.y); Serial.print(",");
    Serial.print(accel.acceleration.z); Serial.print(",");
    
    Serial.print(gyro.gyro.x); Serial.print(",");
    Serial.print(gyro.gyro.y); Serial.print(",");
    Serial.print(gyro.gyro.z); Serial.print(",");
    
    Serial.print(mag.magnetic.x); Serial.print(",");
    Serial.print(mag.magnetic.y); Serial.print(",");
    Serial.println(mag.magnetic.z);  // End of line

    delay(100);
}
