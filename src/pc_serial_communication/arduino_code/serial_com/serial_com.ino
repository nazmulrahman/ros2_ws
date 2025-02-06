// Motor A (Left) arnab
int IN1 = 8;
int IN2 = 9;
int ENA = 6;  // PWM pin for Motor A

// Motor B (Right)
int IN3 = 10;
int IN4 = 11;
int ENB = 7;  // PWM pin for Motor B

void setup() {
    Serial.begin(9600);  // Start serial communication

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);

    stopMotors();
}

void loop() {
    if (Serial.available()) {
        String data = Serial.readStringUntil('\n');  // Read data from serial
        int commaIndex = data.indexOf(',');  

        if (commaIndex > 0) {
            float linearX = data.substring(0, commaIndex).toFloat();  // Extract and convert linear.x
            float angularZ = data.substring(commaIndex + 1).toFloat(); // Extract and convert angular.z

            if (linearX > 0 && angularZ > 0) {
                moveForwardWithTurn(150, 100); // RMS < LMS
            } 
            else if (linearX > 0 && angularZ < 0) {
                moveForwardWithTurn(100, 150); // RMS > LMS
            } 
            else if (linearX < 0 && angularZ > 0) {
                moveBackwardWithTurn(100, 150); // RMS > LMS
            }
            else if (linearX < 0 && angularZ < 0) {
                moveBackwardWithTurn(150, 100); // RMS < LMS
            }
            else if (linearX > 0 && angularZ == 0) {
                moveForward();
            } 
            else if (linearX < 0 && angularZ == 0) {
                moveBackward();
            } 
            else if (linearX == 0 && angularZ > 0) {
                turnLeft();
            }
            else if (linearX == 0 && angularZ < 0) {
                turnRight();
            }
            else {
                stopMotors();
            }
        }
    }
}

void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 150);  // Adjust speed as needed
    analogWrite(ENB, 150);
}

void moveBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, 150);
    analogWrite(ENB, 150);
}

void moveForwardWithTurn(int lms, int rms) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, lms);
    analogWrite(ENB, rms);
}

void moveBackwardWithTurn(int lms, int rms) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, lms);
    analogWrite(ENB, rms);
}

void turnLeft() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, 150);
    analogWrite(ENB, 150);
}

void turnRight() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 150);
    analogWrite(ENB, 150);
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

