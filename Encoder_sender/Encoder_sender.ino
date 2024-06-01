#include <esp_now.h>
#include <WiFi.h>

int counter1 = 0; // Counter for encoder 1
int counter2 = 0; // Counter for encoder 2
volatile unsigned int temp1, temp2; // Temporary variables to store previous counter values

typedef struct struct_message {
  int counter1;
  int counter2;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// MAC Address of the receiver
uint8_t broadcastAddress[] = {0x08, 0xD1, 0xF9, 0xD9, 0x60, 0x28};

void setup() {
  Serial.begin(115200);
  
  pinMode(2, INPUT_PULLUP); // Encoder 1 pin A
  pinMode(3, INPUT_PULLUP); // Encoder 1 pin B
  pinMode(18, INPUT_PULLUP); // Encoder 2 pin A
  pinMode(19, INPUT_PULLUP); // Encoder 2 pin B

  attachInterrupt(digitalPinToInterrupt(2), ai0_1, RISING); // Encoder 1 pin A interrupt
  attachInterrupt(digitalPinToInterrupt(3), ai1_1, RISING); // Encoder 1 pin B interrupt
  attachInterrupt(digitalPinToInterrupt(18), ai0_2, RISING); // Encoder 2 pin A interrupt
  attachInterrupt(digitalPinToInterrupt(19), ai1_2, RISING); // Encoder 2 pin B interrupt
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  // Send the values of both counters if they have changed
  if (counter1 != temp1 || counter2 != temp2) {
    myData.counter1 = counter1;
    myData.counter2 = counter2;

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
    
    Serial.print("Counter 1: ");
    Serial.print(counter1);
    Serial.print("\tCounter 2: ");
    Serial.println(counter2);
    temp1 = counter1;
    temp2 = counter2;
  }
}

void ai0_1() {
  // Encoder 1 pin A rising edge interrupt
  if (digitalRead(3) == LOW) {
    counter1++;
  } else {
    counter1--;
  }
}

void ai1_1() {
  // Encoder 1 pin B rising edge interrupt
  if (digitalRead(2) == LOW) {
    counter1--;
  } else {
    counter1++;
  }
}

void ai0_2() {
  // Encoder 2 pin A rising edge interrupt
  if (digitalRead(19) == LOW) {
    counter2--;
  } else {
    counter2++;
  }
}

void ai1_2() {
  // Encoder 2 pin B rising edge interrupt
  if (digitalRead(18) == LOW) {
    counter2++;
  } else {
    counter2--;
  }
}
