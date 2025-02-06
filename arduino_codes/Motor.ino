// Motor Control Pins
#define FORWARD_REVERSE_PIN 7  
#define ENABLE_PIN 6           
#define PWM_PIN 5              

// Hall Sensor Pin
#define HALL_SENSOR 2  

// Speed Measurement
volatile int pulseCount = 0;
unsigned long previousMillis = 0;
const int interval = 1000;  // 1 second

void countPulse() {  
    pulseCount++;
}

void setup() {
    Serial.begin(9600);

    // Setup motor control pins
    pinMode(FORWARD_REVERSE_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(PWM_PIN, OUTPUT);
    
    // Setup Hall sensor pin
    pinMode(HALL_SENSOR, INPUT);
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR), countPulse, RISING);

    // Start with the motor stopped
    digitalWrite(ENABLE_PIN, HIGH);
}

void loop() {
    // Measure RPM every second
    if (millis() - previousMillis >= interval) {
        previousMillis = millis();
        float rpm = (pulseCount * 60) / 3;  // Assuming 3 pulses per revolution
        Serial.print("Motor RPM: ");
        Serial.println(rpm);
        pulseCount = 0;  // Reset counter
    }

    // Change motor direction every 20 seconds
    static unsigned long lastChange = millis();
    static bool movingForward = true;

    if (millis() - lastChange > 20000) {
        lastChange = millis();
        movingForward = !movingForward;

        if (movingForward) {
            Serial.println("Moving Forward...");
            digitalWrite(ENABLE_PIN, LOW);
            digitalWrite(FORWARD_REVERSE_PIN, LOW);
        } else {
            Serial.println("Moving Backward...");
            digitalWrite(ENABLE_PIN, LOW);
            digitalWrite(FORWARD_REVERSE_PIN, HIGH);
        }
    }

    // Set motor speed
    analogWrite(PWM_PIN, 255);  // Adjust speed as needed
}