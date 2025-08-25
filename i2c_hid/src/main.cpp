#define F_CPU 20000000UL
#include <Wire.h>
#include <EZButton.h>

const uint8_t ADDRESS_BASE = 0x60;
const uint8_t ROTARY_BTN_PIN = PIN_PA3;
const uint8_t ROTARY_A = PIN_PA1;
const uint8_t ROTARY_B = PIN_PA2;
const uint8_t BTN_1_PIN = PIN_PC0;
const uint8_t BTN_2_PIN = PIN_PC1;
const uint8_t BTN_3_PIN = PIN_PC2;
const uint8_t BTN_4_PIN = PIN_PC3;

volatile int8_t rotationSteps;
uint8_t buttonPress[5];
uint8_t buttonLongPress[5];

void ReadButtons(bool *states, int num)
{
  states[0] = !digitalRead(ROTARY_BTN_PIN);
  states[1] = !digitalRead(BTN_1_PIN);
  states[2] = !digitalRead(BTN_2_PIN);
  states[3] = !digitalRead(BTN_3_PIN);
  states[4] = !digitalRead(BTN_4_PIN);
}

EZButton _ezb(5, ReadButtons, 1000, 200, 15);

void IRAM_ATTR handleEncoder()
{
  static uint8_t lastState = 0;
  uint8_t state = (digitalRead(ROTARY_A) << 1) | digitalRead(ROTARY_B);

  // Transition table (Gray code)
  if ((lastState == 0b00 && state == 0b01) ||
      (lastState == 0b01 && state == 0b11) ||
      (lastState == 0b11 && state == 0b10) ||
      (lastState == 0b10 && state == 0b00))
  {
    encoderValue++;
  }
  else if ((lastState == 0b00 && state == 0b10) ||
           (lastState == 0b10 && state == 0b11) ||
           (lastState == 0b11 && state == 0b01) ||
           (lastState == 0b01 && state == 0b00))
  {
    encoderValue--;
  }

  lastState = state;
}

int8_t getAndResetRotationSteps()
{
  int8_t val;
  noInterrupts();
  val = rotationSteps;
  rotationSteps = 0;
  interrupts();
  return val;
}

void receiveEvent(int howMany)
{
  while (Wire.available())
  {
    byte b = Wire.read();
    // handle incoming data
  }
}

void requestEvent()
{
  Wire.write(42); // respond with one byte
}

void initRotaryEncoder()
{
  pinMode(ROTARY_A, INPUT_PULLUP);
  pinMode(ROTARY_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ROTARY_A), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_B), handleEncoder, CHANGE);
}

void initButtons()
{
  pinMode(ROTARY_BTN_PIN, INPUT_PULLUP);
  pinMode(BTN_1_PIN, INPUT_PULLUP);
  pinMode(BTN_2_PIN, INPUT_PULLUP);
  pinMode(BTN_3_PIN, INPUT_PULLUP);
  pinMode(BTN_4_PIN, INPUT_PULLUP);
}

void initWire()
{
  pinMode(PIN_PA4, INPUT);
  pinMode(PIN_PA5, INPUT);
  pinMode(PIN_PA6, INPUT);
  pinMode(PIN_PA7, INPUT);
  uint8_t address = ADDRESS_BASE;
  address |= digitalRead(PIN_PA4);
  address |= digitalRead(PIN_PA5) << 1;
  address |= digitalRead(PIN_PA6) << 2;
  address |= digitalRead(PIN_PA7) << 3;
  Wire.begin(address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void setup()
{
  Serial.begin(115200);
  initWire();
  initRotaryEncoder();
  initButtons();
}

void loop()
{
  _ezb.Loop();
}
