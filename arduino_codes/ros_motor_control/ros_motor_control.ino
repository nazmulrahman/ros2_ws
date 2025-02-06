// Define pin connections
const int trigPin = 9;   // Pin connected to the ultrasonic sensor's TRIG pin
const int echoPin = 10;  // Pin connected to the ultrasonic sensor's ECHO pin

void setup() {
  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  // Set the trigPin as output and echoPin as input
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // Make sure the trigger pin is LOW for a moment
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Trigger the sensor by making the trigPin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin, which returns the time in microseconds for the sound wave to go and come back
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in centimeters
  // Speed of sound ~343 m/s => ~0.034 cm/µs. We divide by 2 because the signal travels out and back.
  int distance = duration * 0.034 / 2;

  // Print ONLY the distance value to the serial port (one reading per line)
  Serial.println(distance);

  // Short delay before the next reading
  delay(500);
}
