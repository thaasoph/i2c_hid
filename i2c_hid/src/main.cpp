// #define F_CPU 20000000UL
#include <EZButton.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

const uint8_t ADDRESS_BASE = 0x60;
uint8_t address = ADDRESS_BASE;

const uint8_t ROTARY_BTN_PIN = PIN_PA3;
const uint8_t ROTARY_A = PIN_PA1;
const uint8_t ROTARY_B = PIN_PA2;
const uint8_t BTN_1_PIN = PIN_PC0;
const uint8_t BTN_2_PIN = PIN_PC1;
const uint8_t BTN_3_PIN = PIN_PC2;
const uint8_t BTN_4_PIN = PIN_PC3;
const uint8_t fractionDebounce = 100;

volatile uint8_t rotationSteps = 0;
uint8_t buttonPress[5] = {0};
uint8_t buttonHold[5] = {0};

uint8_t responseBuffer[11] = {0};
uint8_t bufferIndex = 0;

void ReadButtons(bool *states, int num)
{
  states[0] = !digitalRead(ROTARY_BTN_PIN);
  states[1] = !digitalRead(BTN_1_PIN);
  states[2] = !digitalRead(BTN_2_PIN);
  states[3] = !digitalRead(BTN_3_PIN);
  states[4] = !digitalRead(BTN_4_PIN);
}

void btnPress(int index)
{
  buttonPress[index]++;
}

void btnHold(int index)
{
  buttonHold[index]++;
}

// HoldThreshold: 500ms
// HoldInterval: 300ms
// DebounceTime: 15ms
EZButton _ezb(5, ReadButtons, 500, 300, 15);

void handleEncoder()
{
  static uint8_t lastState = 0;
  static long lastChange = 0;

  if (millis() - lastChange > fractionDebounce)
  {
    if (rotationSteps > -4 && rotationSteps < 4)
    {
      rotationSteps = 0;
    }
  }

  uint8_t state = (digitalRead(ROTARY_A) << 1) | digitalRead(ROTARY_B);

  // Transition table (Gray code)
  if ((lastState == 0b00 && state == 0b01) ||
      (lastState == 0b01 && state == 0b11) ||
      (lastState == 0b11 && state == 0b10) ||
      (lastState == 0b10 && state == 0b00))
  {
    rotationSteps++;
  }
  else if ((lastState == 0b00 && state == 0b10) ||
           (lastState == 0b10 && state == 0b11) ||
           (lastState == 0b11 && state == 0b01) ||
           (lastState == 0b01 && state == 0b00))
  {
    rotationSteps--;
  }

  lastState = state;
  lastChange = millis();
}

int8_t getAndResetRotationSteps()
{
  static long fractionSet;
  int8_t val;
  noInterrupts();
  val = rotationSteps / 4;
  if (val != 0)
  {
    rotationSteps = rotationSteps % 4;
    if (rotationSteps != 0)
    {
      fractionSet = millis();
    }
  }
  // Serial0.printf("left: %d\n", rotationSteps );
  interrupts();
  return val;
}

void createResponseBuffer()
{
  Serial0.println("creating response buffer");
  uint8_t idx = 0;
  responseBuffer[idx] = getAndResetRotationSteps();
  idx++;
  memcpy(&responseBuffer[idx], &buttonPress, sizeof(buttonPress));
  memset(buttonPress, 0, sizeof(buttonPress));
  idx += sizeof(buttonPress);

  memcpy(&responseBuffer[idx], &buttonHold, sizeof(buttonHold));
  memset(buttonHold, 0, sizeof(buttonHold));
  idx += sizeof(buttonHold);
}

void recoverBus()
{
  Serial0.println("performing bus recover");
  TWI0.SCTRLA &= ~TWI_ENABLE_bm; // Disable TWI
  _delay_us(5);
  TWI0.SCTRLA |= TWI_ENABLE_bm; // Re-enable
  bufferIndex = 0;
}

// === TWI0 Interrupt ===
ISR(TWI0_TWIS_vect)
{
  uint8_t status = TWI0.SSTATUS;

  if (status & TWI_BUSERR_bm)
  {
    recoverBus();
    return;
  }

  if (TWI0.SSTATUS & TWI_APIF_bm)
  {
    TWI0.SSTATUS = TWI_APIF_bm; // clear flag
    bufferIndex = 0;
    TWI0.SCTRLB |= TWI_SCMD_RESPONSE_gc;
  }

  if (status & TWI_DIF_bm)
  { // Data interrupt
    if (TWI0.SSTATUS & TWI_DIR_bm)
    { // Master reads

      if (bufferIndex == 0)
      {
        createResponseBuffer();
      }
      if (bufferIndex < sizeof(responseBuffer))
      {
        TWI0.SDATA = responseBuffer[bufferIndex++];
      }
      else
      {
        TWI0.SDATA = 0xFF; // End of buffer, send 0xFF
      }
      TWI0.SCTRLB |= TWI_SCMD_RESPONSE_gc; // ACK next byte
    }
    else
    {                                      // Master writes
      volatile uint8_t data = TWI0.SDATA;  // Read byte (ignore for now)
      TWI0.SCTRLB |= TWI_SCMD_RESPONSE_gc; // ACK
    }
  }
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
  _ezb.Subscribe(0, btnHold, HOLD);
  _ezb.Subscribe(0, btnPress, RELEASED);
  _ezb.Subscribe(1, btnHold, HOLD);
  _ezb.Subscribe(1, btnPress, RELEASED);
  _ezb.Subscribe(2, btnHold, HOLD);
  _ezb.Subscribe(2, btnPress, RELEASED);
  _ezb.Subscribe(3, btnHold, HOLD);
  _ezb.Subscribe(3, btnPress, RELEASED);
  _ezb.Subscribe(4, btnHold, HOLD);
  _ezb.Subscribe(4, btnPress, RELEASED);
}

void initI2C()
{
  pinMode(PIN_PA6, INPUT_PULLUP);
  pinMode(PIN_PA7, INPUT_PULLUP);
  address |= !digitalRead(PIN_PA6);
  address |= !digitalRead(PIN_PA7) << 1;
  Serial0.printf("starting i2c with address: %x", address);
  TWI0.SADDR = address << 1;                 // 7-bit address, LSB ignored
  TWI0.SCTRLA = TWI_ENABLE_bm | TWI_PIEN_bm; // Enable TWI and slave interrupts
  TWI0.SCTRLB = 0;                           // Clear CTRLB
  TWI0.SSTATUS = TWI_BUSERR_bm;              // Clear bus error flag

  bufferIndex = 0;

  sei(); // Enable global interrupts
}

void setup()
{
  Serial0.begin(57600);
  initI2C();
  initRotaryEncoder();
  initButtons();
}

void loop()
{
  _ezb.Loop();
  // Serial0.printf("%d %d %d %d %d %d", rotationSteps, buttonPress[0], buttonPress[1], buttonPress[2], buttonPress[3], buttonPress[4]);
  // Serial0.println();
}
