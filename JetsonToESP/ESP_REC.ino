// Define the pin number for the built-in LED
const int ledPin = 13;

void setup() {
  // Initialize the built-in LED pin as an output
  pinMode(ledPin, OUTPUT);

  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming byte from serial port
    int receivedValue = Serial.parseInt();

    // Check if the received value is 200
    if (receivedValue == 200) {
      digitalWrite(ledPin, HIGH);   // Turn the LED on
    } else {
      digitalWrite(ledPin, LOW);    // Turn the LED off
    }
  }
}
