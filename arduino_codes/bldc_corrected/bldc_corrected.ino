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

// Speed Measurement
volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;
const int interval = 1000;  // 1 second
unsigned long prevMillis = 0;

// Interrupt Service Routines for Hall Sensors
void countPulse1() { pulseCount1++; }
void countPulse2() { pulseCount2++; }

void setup() {
    Serial.begin(9600);
    initializeMotors();  // Initialize motor pins
    initializeHallSensors();  // Initialize hall sensors
}

void loop() {
    measureRPM();  // Measure motor speed
    readSerialData();  // Get data from Jetson and control motors
}

// Function to initialize motors
void initializeMotors() {
    pinMode(FWD_REV_1, OUTPUT); pinMode(EN_1, OUTPUT); pinMode(PWM_1, OUTPUT);
    pinMode(FWD_REV_2, OUTPUT); pinMode(EN_2, OUTPUT); pinMode(PWM_2, OUTPUT);
    stopMotors(); // Ensure motors are off at startup
}

// Function to initialize Hall sensors
void initializeHallSensors() {
    pinMode(HALL_1, INPUT); pinMode(HALL_2, INPUT);
    attachInterrupt(digitalPinToInterrupt(HALL_1), countPulse1, RISING);
    attachInterrupt(digitalPinToInterrupt(HALL_2), countPulse2, RISING);
}

// Function to measure RPM
void measureRPM() {
    if (millis() - prevMillis >= interval) {
        prevMillis = millis();
        Serial.print("Left Motor RPM: "); Serial.print(pulseCount1 * 20);
        Serial.print(" | Right Motor RPM: "); Serial.println(pulseCount2 * 20);
        pulseCount1 = 0; pulseCount2 = 0;  // Reset counter
    }
}

// Function to read and execute motor control commands
void readSerialData() {
    if (Serial.available()) {
        String data = Serial.readStringUntil('\n');  // Read Serial input
        int command = data.toInt();  // Convert string to integer
        executeCommand(command);
    }
}

// Function to execute motor commands based on table
void executeCommand(int command) {
    int speed = 200;  // Default speed (adjustable)
    int speedL = 0, speedR = 0;
    bool reverseL = false, reverseR = false;

    switch (command) {
        case 1:  // Move Forward (Both Motors -)
            speedL = -speed; speedR = -speed;
            break;
        case -1: // Move Backward (Both Motors +) [FIXED]
            speedL = speed; 
            speedR = speed; 
            break;
        case 2:  // Turn Right (Left -, Right +)
            speedL = -speed; speedR = speed;
            break;
        case -2: // Turn Left (Left +, Right -)
            speedL = speed; speedR = -speed;
            break;
        case 3:  // Forward + Right (Left < Right, Both -)
            speedL = -(speed / 2); speedR = -speed;
            break;
        case -3: // Backward + Right (Left slower, Right full speed) [FIXED]
            speedL = speed; 
            speedR = speed / 2;
            break;
        case 4:  // Forward + Left (Left > Right, Both -)
            speedL = -speed; speedR = -(speed / 2);
            break;
        case -4: // Backward + Left (Right slower, Left full speed) [FIXED]
            speedL = speed / 2; 
            speedR = speed;
            break;
        case 0:  // Stop
            stopMotors();
            Serial.println("Motors Stopped.");
            return;
        default:
            Serial.println("Unknown Command. Stopping Motors.");
            stopMotors();
            return;
    }

    // Apply direction settings based on sign
    reverseL = (speedL < 0);
    reverseR = (speedR < 0);
    moveMotors(abs(speedL), abs(speedR), reverseL, reverseR);
}

// Function to move motors with direction control
void moveMotors(int speedL, int speedR, bool reverseL, bool reverseR) {
    digitalWrite(FWD_REV_1, reverseL);  // Set Left Motor direction
    digitalWrite(FWD_REV_2, reverseR);  // Set Right Motor direction
    digitalWrite(EN_1, LOW);
    digitalWrite(EN_2, LOW);
    analogWrite(PWM_1, constrain(speedL, 0, 255));  // Control Left Motor Speed
    analogWrite(PWM_2, constrain(speedR, 0, 255));  // Control Right Motor Speed
}

// Function to stop motors
void stopMotors() {
    digitalWrite(EN_1, HIGH);
    digitalWrite(EN_2, HIGH);
    analogWrite(PWM_1, 0);
    analogWrite(PWM_2, 0);
}
