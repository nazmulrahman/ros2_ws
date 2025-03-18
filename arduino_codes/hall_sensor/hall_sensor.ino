// Motor 1 (Left Motor)
#define FWD_REV_1 7  
#define EN_1 6           
#define PWM_1 5              

// Motor 2 (Right Motor)
#define FWD_REV_2 8  
#define EN_2 9           
#define PWM_2 10              

// Hall Sensor Pins
#define HALL_1 2  
#define HALL_2 3  

// Motor & Encoder Parameters
#define PULSES_PER_REV 60  // 3 Hall sensors × 20:1 gear ratio
#define WHEEL_DIAMETER 0.13 // Diameter in meters (130mm = 0.13m)
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * 3.1416) // ≈ 0.408m

const int interval = 1000;  // Measurement interval (1 sec)

// Speed Measurement Variables
volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;
float rpmL = 0.0, rpmR = 0.0;
float velocityL = 0.0, velocityR = 0.0; // Speed in m/s
unsigned long prevMillis = 0;

// Interrupt Service Routines for Hall Sensors
void countPulse1() { pulseCount1++; }
void countPulse2() { pulseCount2++; }

void setup() {
    Serial.begin(9600);
    initializeMotors();
    initializeHallSensors();
}

void loop() {
    measureSpeed();  // Calculate RPM & speed
    readSerialData();
}

// Initialize Motors
void initializeMotors() {
    pinMode(FWD_REV_1, OUTPUT); pinMode(EN_1, OUTPUT); pinMode(PWM_1, OUTPUT);
    pinMode(FWD_REV_2, OUTPUT); pinMode(EN_2, OUTPUT); pinMode(PWM_2, OUTPUT);
    stopMotors();
}

// Initialize Hall Sensors
void initializeHallSensors() {
    pinMode(HALL_1, INPUT); pinMode(HALL_2, INPUT);
    attachInterrupt(digitalPinToInterrupt(HALL_1), countPulse1, RISING);
    attachInterrupt(digitalPinToInterrupt(HALL_2), countPulse2, RISING);
}

// Function to Calculate RPM & Speed
void measureSpeed() {
    if (millis() - prevMillis >= interval) {
        float timeMin = interval / 60000.0; // Convert ms to minutes
        
        rpmL = (pulseCount1 * 60.0) / PULSES_PER_REV;
        rpmR = (pulseCount2 * 60.0) / PULSES_PER_REV;

        velocityL = (WHEEL_CIRCUMFERENCE * rpmL) / 60.0; // m/s
        velocityR = (WHEEL_CIRCUMFERENCE * rpmR) / 60.0; // m/s

        // Send RPM & speed to ROS 2
        Serial.print("L_RPM:"); Serial.print(rpmL); Serial.print("|");
        Serial.print("R_RPM:"); Serial.print(rpmR); Serial.print("|");
        
        Serial.print("L_Speed:"); Serial.print(velocityL); Serial.print(" m/s|");
        Serial.print("R_Speed:"); Serial.println(velocityR); Serial.print(" m/s");

        pulseCount1 = 0;
        pulseCount2 = 0;
        prevMillis = millis();
    }
}

// Read Serial Commands from ROS 2
void readSerialData() {
    if (Serial.available()) {
        String data = Serial.readStringUntil('\n');  
        int command = data.toInt();
        executeCommand(command);
    }
}

// Function to Execute Motor Commands
void executeCommand(int command) {
    int speed = 200;
    int speedL = 0, speedR = 0;
    bool reverseL = false, reverseR = false;

    switch (command) {
        case 1: speedL = -speed; speedR = -speed; break;
        case -1: speedL = speed; speedR = speed; break;
        case 2: speedL = -speed; speedR = speed; break;
        case -2: speedL = speed; speedR = -speed; break;
        case 3: speedL = -(speed / 2); speedR = -speed; break;
        case -3: speedL = speed; speedR = speed / 2; break;
        case 4: speedL = -speed; speedR = -(speed / 2); break;
        case -4: speedL = speed / 2; speedR = speed; break;
        case 0: stopMotors(); return;
        default: stopMotors(); return;
    }

    reverseL = (speedL < 0);
    reverseR = (speedR < 0);
    moveMotors(abs(speedL), abs(speedR), reverseL, reverseR);
}

// Function to Move Motors
void moveMotors(int speedL, int speedR, bool reverseL, bool reverseR) {
    digitalWrite(FWD_REV_1, reverseL);
    digitalWrite(FWD_REV_2, reverseR);
    digitalWrite(EN_1, LOW);
    digitalWrite(EN_2, LOW);
    analogWrite(PWM_1, constrain(speedL, 0, 255));
    analogWrite(PWM_2, constrain(speedR, 0, 255));
}

// Function to Stop Motors
void stopMotors() {
    digitalWrite(EN_1, HIGH);
    digitalWrite(EN_2, HIGH);
    analogWrite(PWM_1, 0);
    analogWrite(PWM_2, 0);
}
