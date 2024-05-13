void setup() {
  Serial.begin(9600);  // Initialize serial communication
}

void loop() {
  if (Serial.available()) {
    char receivedChar = Serial.read();  // Read incoming character
    Serial.print("Received: ");
    Serial.println(receivedChar);  // Print received character
  }
}
