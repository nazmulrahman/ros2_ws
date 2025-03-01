#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

// Initialize IMU Sensors
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize Accelerometer
    if (!accel.begin()) {
        Serial.println("❌ LSM303 Accelerometer not detected!");
        while (1);
    } else {
        Serial.println("✅ LSM303 Accelerometer detected!");
    }

    // Initialize Magnetometer
    if (!mag.begin()) {
        Serial.println("❌ LSM303 Magnetometer not detected!");
        while (1);
    } else {
        Serial.println("✅ LSM303 Magnetometer detected!");
    }

    Serial.println("✅ IMU Ready!");
}

void loop() {
    sensors_event_t accel_event, mag_event;
    
    // Read sensor data
    accel.getEvent(&accel_event);
    mag.getEvent(&mag_event);

    // Send data as a structured JSON-like format
    Serial.print("{");
    Serial.print("\"ax\":"); Serial.print(accel_event.acceleration.x, 4);
    Serial.print(", \"ay\":"); Serial.print(accel_event.acceleration.y, 4);
    Serial.print(", \"az\":"); Serial.print(accel_event.acceleration.z, 4);
    Serial.print(", \"mx\":"); Serial.print(mag_event.magnetic.x, 2);
    Serial.print(", \"my\":"); Serial.print(mag_event.magnetic.y, 2);
    Serial.print(", \"mz\":"); Serial.print(mag_event.magnetic.z, 2);
    Serial.println("}");

    delay(100);
}
