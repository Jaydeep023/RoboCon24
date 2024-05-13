void setup() {
  // Initialize serial communication at 9600 baud
  Serial.begin(9600);
  pinMode(13,OUTPUT);
}

void loop() {
  // Check if there is data available to read
  if (Serial.available() >= 4) {
    // Read the incoming data into a buffer
    byte buffer[4];
    Serial.readBytes(buffer, 4);
    
    // Interpret the received data
    char identifier[3];
    identifier[0] = buffer[0];
    identifier[1] = buffer[1];
    identifier[2] = '\0'; // Null-terminate the string
    
    int value1 = buffer[2];
    int value2 = buffer[3];
    if(value1==200){
      digitalWrite(13,HIGH);
      delay(100);
    }else{
      digitalWrite(13,LOW);
      delay(100);
    }
    // Print the received data
    // Serial.print("Received: Identifier: ");
    // Serial.print(identifier);
    // Serial.print(" Value1: ");
    // Serial.print(value1);
    // Serial.print(" Value2: ");
    // Serial.println(value2);
  }
}
