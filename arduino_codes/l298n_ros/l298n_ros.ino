#include <Arduino.h>

// Motor Driver Pins
#define ENA 9    // PWM for Left Motor
#define IN1 7    // Left Motor Forward
#define IN2 6    // Left Motor Backward
#define ENB 10   // PWM for Right Motor
#define IN3 5    // Right Motor Forward
#define IN4 4    // Right Motor Backward

// PWM Speed Levels
#define SPEED_HIGH 200  // Full speed
#define SPEED_LOW 100   // Reduced speed for turning
#define SPEED_STOP 0    // Motor off

void setup() {
    // Initialize Serial Communication
    Serial.begin(9600);

    // Set motor control pins as outputs
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Stop motors initially
    stopMotors();
}

void loop() {
    if (Serial.available() > 0) {
        int command = Serial.parseInt();  // Read integer command

        // Process the command
        executeCommand(command);
    }
}

void executeCommand(int cmd) {
    switch (cmd) {
        case 1:  moveForward(); break;
        case -1: moveBackward(); break;
        case -2: turnLeft(); break;
        case 2:  turnRight(); break;
        case 4:  forwardLeft(); break;
        case 3:  forwardRight(); break;
        case -3: backwardLeft(); break;
        case -4: backwardRight(); break;
        case 0:  stopMotors(); break;
        default: stopMotors(); break;  // Invalid command: Stop motors
    }
}

// Move Forward (Both Motors Forward)
void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, SPEED_HIGH);
    analogWrite(ENB, SPEED_HIGH);
}

// Move Backward (Both Motors Backward)
void moveBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, SPEED_HIGH);
    analogWrite(ENB, SPEED_HIGH);
}

// Turn Left (Left Backward, Right Forward)
void turnLeft() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, SPEED_HIGH);
    analogWrite(ENB, SPEED_HIGH);
}

// Turn Right (Left Forward, Right Backward)
void turnRight() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, SPEED_HIGH);
    analogWrite(ENB, SPEED_HIGH);
}

// Forward + Left (Left Slow, Right Fast)
void forwardLeft() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, SPEED_LOW);   // Reduce speed for left motor
    analogWrite(ENB, SPEED_HIGH);  // Increase speed for right motor
}

// Forward + Right (Left Fast, Right Slow)
void forwardRight() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, SPEED_HIGH);  // Increase speed for left motor
    analogWrite(ENB, SPEED_LOW);   // Reduce speed for right motor
}

// Backward + Left (Left Slow, Right Fast)
void backwardLeft() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, SPEED_LOW);   // Reduce speed for left motor
    analogWrite(ENB, SPEED_HIGH);  // Increase speed for right motor
}

// Backward + Right (Left Fast, Right Slow)
void backwardRight() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, SPEED_HIGH);  // Increase speed for left motor
    analogWrite(ENB, SPEED_LOW);   // Reduce speed for right motor
}

// Stop Motors
void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, SPEED_STOP);
    analogWrite(ENB, SPEED_STOP);
}
