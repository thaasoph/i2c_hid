#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// === Configuration ===
#define I2C_SLAVE_ADDR 0x42
#define BUFFER_SIZE    8

volatile uint8_t responseBuffer[BUFFER_SIZE];
volatile uint8_t bufferIndex = 0;

// === Forward declarations ===
void i2c_init();
void i2c_recover_bus();
void updateResponseBuffer(); // Optional function to update buffer content


// === Main loop ===
void loop() {
    // Idle loop
    asm volatile("nop");
}

// === Initialize TWI0 as I2C Slave ===
void i2c_init() {
    // PORTMUX.TWIROUTEA = PORTMUX_TWI0_ALT1_gc; // Optional: route pins if needed
    TWI0.SADDR = I2C_SLAVE_ADDR << 1;         // 7-bit address, LSB ignored
    TWI0.SCTRLA = TWI_ENABLE_bm | TWI_PIEN_bm; // Enable TWI and slave interrupts
    TWI0.SCTRLB = 0;                          // Clear CTRLB
    TWI0.SSTATUS = TWI_BUSERR_bm;             // Clear bus error flag

    bufferIndex = 0;

    sei(); // Enable global interrupts
}

// === Bus recovery ===
void i2c_recover_bus() {
    TWI0.SCTRLA &= ~TWI_ENABLE_bm; // Disable TWI
    _delay_us(5);
    TWI0.SCTRLA |= TWI_ENABLE_bm;  // Re-enable
    bufferIndex = 0;
}


// === TWI0 Interrupt ===
ISR(TWI0_TWIS_vect) {
    uint8_t status = TWI0.SSTATUS;

    if (status & TWI_BUSERR_bm) {
        i2c_recover_bus();
        return;
    }

    if (status & TWI_APIF_bm) { // Address/Stop condition detected
        bufferIndex = 0;
        TWI0.SCTRLB |= TWI_SCMD_RESPONSE_gc; // ACK
    }

    if (status & TWI_DIF_bm) { // Data interrupt
        if (TWI0.SSTATUS & TWI_DIR_bm) { // Master reads
            if (bufferIndex < BUFFER_SIZE) {
                TWI0.SDATA = responseBuffer[bufferIndex++];
            } else {
                TWI0.SDATA = 0xFF; // End of buffer, send 0xFF
            }
            TWI0.SCTRLB |= TWI_SCMD_RESPONSE_gc; // ACK next byte
        } else { // Master writes
            volatile uint8_t data = TWI0.SDATA; // Read byte (ignore for now)
            TWI0.SCTRLB |= TWI_SCMD_RESPONSE_gc; // ACK
        }
    }
}



// === Setup ===
void setup() {
    i2c_init();
}