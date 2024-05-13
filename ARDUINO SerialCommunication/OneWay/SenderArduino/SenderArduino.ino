void setup() {
  Serial.begin(9600);
}

void loop() {
  byte data[] = {'m', '1', 100, 200};
  Serial.write(data,sizeof(data)); // send a byte with the value 45

   //send the string "hello" and return the length of the string.
   delay(1000);
}