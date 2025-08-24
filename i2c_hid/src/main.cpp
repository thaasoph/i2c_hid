#include <Wire.h>

void receiveEvent(int howMany) {
  while (Wire.available()) {
    byte b = Wire.read();
    // handle incoming data
  }
}

void requestEvent() {
  Wire.write(42); // respond with one byte
}

void setup() {
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  uint8_t address = 0x60;
  address |= digitalRead(2);
  address |= digitalRead(3) << 1;
  address |= digitalRead(4) << 2;
  address |= digitalRead(5) << 3;
  Wire.begin(address);
  
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
  // your normal code, chip sleeps until needed
}
