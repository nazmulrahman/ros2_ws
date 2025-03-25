// =======================
// Motor Pin Definitions
// =======================

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

// =======================
// RPM Measurement
// =======================
volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;
const int interval = 1000;  // 1 second
unsigned long prevMillis = 0;

// =======================
// Setup
// =======================
void setup() {
    Serial.begin(9600);
    initializeMotors();       // Initialize motor pins
    initializeHallSensors();  // Initialize hall sensors
    delay(2000);              // Optional delay before test
    testMotorSpin();          // 🧪 Run test once
}

void loop() {
    measureRPM();             // Continuously measure RPM
    // readSerialData();      // Not needed for test
}

// =======================
// Motor Initialization
// =======================
void initializeMotors() {
    pinMode(FWD_REV_1, OUTPUT); pinMode(EN_1, OUTPUT); pinMode(PWM_1, OUTPUT);
    pinMode(FWD_REV_2, OUTPUT); pinMode(EN_2, OUTPUT); pinMode(PWM_2, OUTPUT);
    stopMotors();
}

// =======================
// Hall Sensor Initialization
// =======================
void initializeHallSensors() {
    pinMode(HALL_1, INPUT); 
    pinMode(HALL_2, INPUT);
    attachInterrupt(digitalPinToInterrupt(HALL_1), countPulse1, RISING);
    attachInterrupt(digitalPinToInterrupt(HALL_2), countPulse2, RISING);
}

// =======================
// Interrupt Functions
// =======================
void countPulse1() {
    pulseCount1++;
}

void countPulse2() {
    pulseCount2++;
}

// =======================
// RPM Calculation
// =======================
void measureRPM() {
    if (millis() - prevMillis >= interval) {
        prevMillis = millis();
        Serial.print("Left Motor RPM: "); Serial.print(pulseCount1 * 20);
        Serial.print(" | Right Motor RPM: "); Serial.println(pulseCount2 * 20);
        pulseCount1 = 0; pulseCount2 = 0;
    }
}

// =======================
// Motor Test Function 🧪
// =======================
void testMotorSpin() {
    int rpm = 30;
    int pwmValue = 0.0164*rpm*rpm + 2.2207 * rpm + 9.6945;       // PWM speed (0–255)
    int spinDuration = 10000;  // Time in milliseconds

    Serial.println("Starting test motor spin...");

    // Forward direction
    digitalWrite(FWD_REV_1, LOW);
    digitalWrite(FWD_REV_2, LOW);

    // Enable motors (active LOW)
    digitalWrite(EN_1, LOW);
    digitalWrite(EN_2, LOW);

    // Apply PWM
    analogWrite(PWM_1, pwmValue);
    analogWrite(PWM_2, pwmValue);

    Serial.print("Motors spinning at PWM ");
    Serial.print(pwmValue);
    Serial.print(" for ");
    Serial.print(spinDuration);
    Serial.println(" ms");

    delay(spinDuration);  // Let motors spin

    stopMotors();  // Stop after delay
    Serial.println("Motors stopped after test.");
}

// =======================
// Motor Stop Function
// =======================
void stopMotors() {
    digitalWrite(EN_1, HIGH);
    digitalWrite(EN_2, HIGH);
    analogWrite(PWM_1, 0);
    analogWrite(PWM_2, 0);
}
