

// === July 2021 Debugging Flags ===
// #define DEBUG_SKIP_PWM_CHECK
// #define DEBUG_SPI_LED_DELAY
#define DEBUG_ROW_OFF_ON_DELAY
// #define DEBUG_ROW_OFF_ON_DELAY_CONTROL
// #define DEBUG_SETUP_PINS_EARLY
// === End Debug Flags =============

#define INVERT_ROW_PINS
// INVERT_ROW_PINS is needed for G4 driver v1

#ifdef SERIAL_DEBUG
#include <Streaming.h>
#endif

// Macros
// ----------------------------------------------------------------------------
#define NOP __asm__ __volatile__ ("nop\n\t")

// Column pins are port D
#ifdef SERIAL_DEBUG
#define SET_COL_PINS(value) ({                           \
    PORTD = (PORTD & 0b00000011) + (value & 0b11111100); \
})
#else
#define SET_COL_PINS(value) ((PORTD=value))
#endif


// Row pins are on ports C and B
#ifdef INVERT_ROW_PINS
#define SET_ROW_PINS(value) ({                              \
    PORTB = (PORTB & 0b11111100) + ((~value) >> 6);         \
    PORTC = (PORTC & 0b11000000) + ((~value) & 0b00111111); \
})
#else
#define SET_ROW_PINS(value) ({                           \
    PORTB = (PORTB & 0b11111100) + (value  >> 6);        \
    PORTC = (PORTC & 0b11000000) + (value & 0b00111111); \
})
#endif


// Constants
// ----------------------------------------------------------------------------
const uint8_t BUF_SIZE = 0xff;
const uint16_t TWI_MAX_COUNT = 0xffff;
const uint8_t PWM_TYPE_MASK = 0x01;
const uint8_t PWM_TYPE_2 = 0;
const uint8_t PWM_TYPE_16 = 1;
const uint8_t DELAY_MASK = 0xfe;
const uint8_t DELAY_SHIFT = 1;
const uint8_t PWM_TYPE_2_MSG_SIZE = 9;
const uint8_t PWM_TYPE_16_MSG_SIZE = 33;

// DEBUG Globals
uint8_t Reset_Source = 0; // OM 2021-07-08
uint8_t Loop_Counter = 0;


// Functions
// ----------------------------------------------------------------------------
void setup()
{
    // DEBUG: Query "Reset Source" Flags
    Reset_Source = MCUSR; // OM 2021-07-08
    MCUSR = 0x00; // OM 2021-07-08

    // Setup SPI communications
    pinMode(MISO,OUTPUT);
    pinMode(SS, INPUT);
    //SPCR |= _BV(SPE);
    SPCR = _BV(SPE);    // Based on Atmel datasheet; OM 2021-06-09

    // Setup pin modes for row & col pins
    resetRowColPins();

    // Turn off interrupts
    noInterrupts();
}

void resetRowColPins() {

#ifdef DEBUG_SETUP_PINS_EARLY
    // Turn "off" all column and row pins
    // Do this *before* setting pins to output mode
    SET_COL_PINS(0x00);
    SET_ROW_PINS(0xff);
#endif

    // Set data direction for row pins
    DDRB |= _BV(0) | _BV(1);
    DDRC |= _BV(0) | _BV(1) | _BV(2) | _BV(3)| _BV(4)| _BV(5);

    // Set data direction for col pins
#ifdef SERIAL_DEBUG
    DDRD = 0xfc;
    Serial.begin(115200);
#else
    DDRD = 0xff;
#endif

    // Turn "off" all column and row pins
    SET_COL_PINS(0x00);
    SET_ROW_PINS(0xff);
}

void loop()
{
    Loop_Counter++; // DEBUG; OM 2021-07-08
    static uint8_t buffer[BUF_SIZE];
    bool dataReady = false;
    uint8_t bufPos = 0;
    uint8_t msgSize;
    uint8_t pwmType;

    // 1) Read incoming spi message
    // ------------------------------------------------------------------------
    // DEBUG: Here is where we can specify the first byte to send back on MISO:
    // SPDR = bufPos; // transmit bufPos; DEBUG: OM 2021-06-09
    // SPDR = Reset_Source; // DEBUG: OM 2021-07-08
#ifdef DEBUG_SKIP_PWM_CHECK
    msgSize = PWM_TYPE_16_MSG_SIZE; // DEBUG: OM 2021-06-09
    pwmType = PWM_TYPE_16;
#endif

    // while (digitalRead(SS) == 1); // Wait for chip-select enable
    while (PINB & _BV(2)); // Wait for chip-select enable (should be faster than above)

    while (true)
    {
        // SPI transfer
        while (!(SPSR & _BV(SPIF))); // wait for transmission complete
        buffer[bufPos] = SPDR;       // Read byte from spi data register
        // DEBUG: Here is where we can specify the byte to send back on MISO:
        // SPDR = buffer[bufPos]; // send back what we recieved; DEBUG: OM 2021-06-09
        SPDR = 0x00;           // send back 0 — easy to see if this stops; DEBUG: OM 2021-07-02
        // SPDR = Reset_Source;    // output the cause of the most recent reset
        // SPDR = Loop_Counter;    // output loop number (to detect reboots)
        bufPos++;
        // SPDR = bufPos; // transmit bufPos; DEBUG: OM 2021-06-09

#ifndef DEBUG_SKIP_PWM_CHECK
        if (bufPos == 1)
        {
            pwmType = buffer[0] & PWM_TYPE_MASK;
            if (pwmType == PWM_TYPE_16)
            {
                msgSize = PWM_TYPE_16_MSG_SIZE;
            }
            else
            {
                msgSize = PWM_TYPE_2_MSG_SIZE;
            }
        }
#endif // DEBUG_SKIP_PWM_CHECK

        if (bufPos >= msgSize)
        {
            dataReady = true;
            break;
        }
    }
    SPDR = 0x00; // DEBUG OM 2021-06-10
    //while (digitalRead(SS) == 0); // Wait for chip-select to be disabled
    while (!(PINB & _BV(2))); // Wait for chip-select disable (should be faster than above)

    // Read SPSR and SPDR a couple times - to clear out any possible mismatch
    for (uint8_t i=0; i<5; i++)
    {
        uint8_t dummy0 = SPSR & _BV(SPIF);
        uint8_t dummy1 = SPDR;
    }
    SPDR = 0x00; // DEBUG OM 2021-06-10

#ifdef DEBUG_SPI_LED_DELAY
    // DEBUG: Add delay between SPI recieve and LED display (OM 2021-07-08)
    delayMicroseconds(400);
# endif // DEBUG_SPI_LED_DELAY

    // 2) Update display
    // ------------------------------------------------------------------------
    if (dataReady)
    {
        uint8_t *bufferPtr = buffer;
        uint8_t row = 0;
        uint8_t pwm = 0;
        uint8_t pwmShift4 = 0;
        uint8_t colValue = 0x00;
        uint8_t dummy = 0;
        uint8_t pwmMaxCount = 0;
        uint8_t delayValue = 0;

        // Get max count for pwm and delayValue
        if (pwmType == PWM_TYPE_16)
        {
            pwmMaxCount = 16;
        }
        else
        {
            pwmMaxCount = 2;
        }
        delayValue = (*bufferPtr & DELAY_MASK);

        // Activate first row
        SET_ROW_PINS(~_BV(0)); // ~_BV(0) => 0b11111110

        while (row < 8)
        {
            // Get column values based on matrix
            colValue = 0x00; // initialize colVal to all 0s

            // check if each column value is > pwmThreshold
            //  (pwmThreshold will increase from 0 to 15 in successive
            //  loop iterations in 16-level mode)
            if (pwmMaxCount == 16)
            {
                // 16 -level grayscale (~1.3kHz)
                pwmShift4 = pwm << 4;

                bufferPtr = buffer + 4*row + 1;
                if ((*bufferPtr & 0x0f) > pwm      ) { colValue |= _BV(0); } else { dummy |= _BV(0); } // col 1
                if ((*bufferPtr & 0xf0) > pwmShift4) { colValue |= _BV(1); } else { dummy |= _BV(1); } // col 2

                bufferPtr++;
                if ((*bufferPtr & 0x0f) > pwm      ) { colValue |= _BV(2); } else { dummy |= _BV(2); } // col 3
                if ((*bufferPtr & 0xf0) > pwmShift4) { colValue |= _BV(3); } else { dummy |= _BV(3); } // col 4

                bufferPtr++;
                if ((*bufferPtr & 0x0f) > pwm      ) { colValue |= _BV(4); } else { dummy |= _BV(4); } // col 5
                if ((*bufferPtr & 0xf0) > pwmShift4) { colValue |= _BV(5); } else { dummy |= _BV(5); } // col 6

                bufferPtr++;
                if ((*bufferPtr & 0x0f) > pwm      ) { colValue |= _BV(6); } else { dummy |= _BV(6); } // col 7
                if ((*bufferPtr & 0xf0) > pwmShift4) { colValue |= _BV(7); } else { dummy |= _BV(7); } // col 8

            }
            else
            {
                // 2-level grayscale (~9.8 kHz)
                bufferPtr = buffer + row + 1;
                if (((*bufferPtr & 0x01) >> 0) > pwm) { colValue |= _BV(0); } else { dummy |= _BV(0); }
                if (((*bufferPtr & 0x02) >> 1) > pwm) { colValue |= _BV(1); } else { dummy |= _BV(1); }
                if (((*bufferPtr & 0x04) >> 2) > pwm) { colValue |= _BV(2); } else { dummy |= _BV(2); }
                if (((*bufferPtr & 0x08) >> 3) > pwm) { colValue |= _BV(3); } else { dummy |= _BV(3); }
                if (((*bufferPtr & 0x10) >> 4) > pwm) { colValue |= _BV(4); } else { dummy |= _BV(4); }
                if (((*bufferPtr & 0x20) >> 5) > pwm) { colValue |= _BV(5); } else { dummy |= _BV(5); }
                if (((*bufferPtr & 0x40) >> 6) > pwm) { colValue |= _BV(6); } else { dummy |= _BV(6); }
                if (((*bufferPtr & 0x80) >> 7) > pwm) { colValue |= _BV(7); } else { dummy |= _BV(7); }
            }
            SET_COL_PINS(colValue);

            // fixed delay at the end of each loop
            if (delayValue > 0)
            {
                for (uint8_t delayCount=0; delayCount < delayValue; delayCount++)
                {
                    NOP;
                }
            }

            // Update pwm count and row count
            pwm++;

            // DEBUG: Flicker row on-off; OM 2021-07-02; THIS CRASHES THE DRIVER
            // SET_ROW_PINS(0xFF); // turn row off
            // SET_ROW_PINS(~_BV(row%8)); // activate current row

            if (pwm >= pwmMaxCount)
            {
                // finished all pwm values: reset pwm and increment row
                SET_COL_PINS(0x00); // turn off all columns
                pwm = 0;
                row++;
                if (row < 8)
                {
#ifdef DEBUG_ROW_OFF_ON_DELAY
                    // DEBUG: Add delay between turning off current row
                    // and turning on next row; OM 2021-07-08
                    SET_ROW_PINS(0xff); // deactivate all rows
                    // delayMicroseconds(1); // might be as long as 3us (?)
                    NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP; // 10x
                    NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP; // 10x
#endif // DEBUG_ROW_OFF_ON_DELAY
                    SET_ROW_PINS(~_BV(row%8)); // activate next row
                }
                else
                {
                    SET_ROW_PINS(0xff); // deactivate all rows
                }
            }


        } // while (row<8)
    }
}
