/*
 * MCU1_master.c
 * Motor Control Project - MASTER (ATmega32)
 *
 * Responsibilities:
 *   - Read user input from 4x4 keypad via MM74C922 16-key encoder (U4)
 *   - Display selected mode and value on 16x2 LCD
 *   - Send mode + value to MCU2 (Slave) via I2C
 *
 *  MM74C922 Interface (U4) 
 *
 *  The MM74C922 handles the full 4×4 keypad matrix internally.
 *  It scans rows Y1–Y4 and columns X1–X4, debounces the keypress,
 *  latches the result, and raises DA (Data Available) HIGH.
 *
 *  MCU1 PORTA connections:
 *    PA0  -  A  (bit 0, LSB)  }  4-bit active-HIGH binary key code
 *    PA1  -  B  (bit 1)       }  valid only while DA is HIGH
 *    PA2  -  C  (bit 2)       }
 *    PA3  -  D  (bit 3, MSB)  }
 *    PA4  -  DA (Data Available) — HIGH = valid key latched
 *    PA5  — unused / spare
 *    PA6  — unused / spare
 *    PA7  — unused / spare
 *
 *  MM74C922 OE pin: tie to GND (OE active-LOW ? outputs always enabled).
 *
 *  MM74C922 truth table (active-HIGH outputs, keys 0–15):
 *
 *   Key  |  D C B A  | Keypad label (standard phone layout)
 *   -----+-----------+------------------------------
 *     0  |  0 0 0 0  |  [1]   row1-col1
 *     1  |  0 0 0 1  |  [2]   row1-col2
 *     2  |  0 0 1 0  |  [3]   row1-col3
 *     3  |  0 0 1 1  |  [A]   row1-col4  - mode: Speed
 *     4  |  0 1 0 0  |  [4]   row2-col1
 *     5  |  0 1 0 1  |  [5]   row2-col2
 *     6  |  0 1 1 0  |  [6]   row2-col3
 *     7  |  0 1 1 1  |  [B]   row2-col4  - mode: Position
 *     8  |  1 0 0 0  |  [7]   row3-col1
 *     9  |  1 0 0 1  |  [8]   row3-col2
 *    10  |  1 0 1 0  |  [9]   row3-col3
 *    11  |  1 0 1 1  |  [C]   row3-col4  - (unused / spare)
 *    12  |  1 1 0 0  |  [*]   row4-col1  - backspace
 *    13  |  1 1 0 1  |  [0]   row4-col2
 *    14  |  1 1 1 0  |  [#]   row4-col3  - confirm/enter
 *    15  |  1 1 1 1  |  [D]   row4-col4  - Emergency Stop
 *
 *  All 16 keys are encoded — no separate function wires needed.
 *
 *   Other connections:
 *   LCD (4-bit)  -> PORTC (PC2=RS, PC3=EN, PC4–PC7=D4–D7)
 *   I2C SDA      -> PC1 (TWI SDA)
 *   I2C SCL      -> PC0 (TWI SCL)
 *   NOTE: On ATmega32, TWI pins are PC0(SCL) and PC1(SDA)
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

/*
 * I2C (TWI) Definitions
*/
#define I2C_SLAVE_ADDR   0x10          // MCU2 slave address (7-bit)
#define I2C_FREQ         100000UL      // 100 kHz

/* TWI bit-rate register value calculation */
#define TWBR_VAL  ((F_CPU / I2C_FREQ - 16) / 2)

/*
 * LCD Definitions  (4-bit mode, PORTC)
 * RS=PC2, EN=PC3, D4=PC4, D5=PC5, D6=PC6, D7=PC7
*/
#define LCD_PORT  PORTC
#define LCD_DDR   DDRC
#define LCD_RS    PC2
#define LCD_EN    PC3
#define LCD_D4    PC4
#define LCD_D5    PC5
#define LCD_D6    PC6
#define LCD_D7    PC7

/*
 * Keypad / MM74C922 Encoder Definitions (PORTA)
 *
 * PA0–PA3 : A,B,C,D  4-bit active-HIGH key code output from MM74C922
 * PA4     : DA        Data Available — HIGH when a valid key is latched
 *
 * All PORTA pins are configured as INPUTS.
 * No pull-ups needed on PA0–PA3 (MM74C922 outputs are push-pull CMOS).
 * Pull-up on PA4 (DA) is optional but harmless — DA is actively driven.
*/
#define KP_DDR      DDRA
#define KP_PORT     PORTA
#define KP_PIN      PINA

#define KP_CODE_MASK  0x0F    /* PA0–PA3: 4-bit key code (active-HIGH) */
#define KP_DA_BIT     PA4     /* Data Available: HIGH = key ready       */

/*
 * MM74C922 key-code to ASCII lookup table (16 entries, index = 4-bit code).
 *
 * Standard 4×4 phone-layout keypad wired to Y1–Y4, X1–X4:
 *   code  0 ? [1]   code  1 ? [2]   code  2 ? [3]   code  3 ? [A]
 *   code  4 ? [4]   code  5 ? [5]   code  6 ? [6]   code  7 ? [B]
 *   code  8 ? [7]   code  9 ? [8]   code 10 ? [9]   code 11 ? [C]
 *   code 12 ? [*]   code 13 ? [0]   code 14 ? [#]   code 15 ? [D]
 *
 * Key roles in this application:
 *   [A] = Select Speed mode / Confirm number entry
 *   [B] = Select Position mode
 *   [D] = Emergency Stop
 *   [*] = Backspace during number entry
 *   [#] = Confirm number entry (alias for [A])
 *   [C] = Unused
 */
unsigned const char kp_code_to_char[16] =
 {
   '7',  '8',  '9',  'A', 
   '4',  '5',  '6',  'B',
   '1',  '2',  '3',  'C',  
   '*',  '0',  '#',  'D'
};

/*
 * Mode constants (must match MCU2)
 */
#define MODE_SPEED     'S'	//Speed
#define MODE_POSITION  'P'	//Position
#define MODE_STOP      'E'  // Emergency stop

/*
 * LCD DRIVER (4-bit)
 */
static void lcd_pulse_enable(void)
{
	LCD_PORT |=  (1 << LCD_EN);
	_delay_us(1);
	LCD_PORT &= ~(1 << LCD_EN);
	_delay_us(50);
}

static void lcd_write_nibble(uint8_t nibble)
{
	/* Place upper nibble on D4-D7 */
	LCD_PORT = (LCD_PORT & 0x0F) | ((nibble & 0x0F) << 4);
	lcd_pulse_enable();
}

static void lcd_send(uint8_t data, uint8_t is_data)
{
	if (is_data)
	LCD_PORT |=  (1 << LCD_RS);   // data
	else
	LCD_PORT &= ~(1 << LCD_RS);   // command

	lcd_write_nibble(data >> 4);      // high nibble first
	lcd_write_nibble(data & 0x0F);    // low nibble

	if ((data == 0x01) || (data == 0x02))
	_delay_ms(2);
}


static void lcd_cmd(uint8_t cmd)  { lcd_send(cmd,  0); }
static void lcd_char(uint8_t ch)  { lcd_send(ch,   1); }

static void lcd_init(void)
{
	LCD_DDR |= (1<<LCD_RS)|(1<<LCD_EN)|(1<<LCD_D4)|(1<<LCD_D5)|(1<<LCD_D6)|(1<<LCD_D7);
	_delay_ms(50);

	/* 4-bit initialization sequence */
	lcd_write_nibble(0x03); _delay_ms(5);
	lcd_write_nibble(0x03); _delay_ms(1);
	lcd_write_nibble(0x03); _delay_ms(1);
	lcd_write_nibble(0x02); _delay_ms(1);

	lcd_cmd(0x28);  // 2-line, 5x8 matrix, 4 bit mode
	lcd_cmd(0x0C);  // display on, cursor off
	lcd_cmd(0x06);  // entry mode: increment
	lcd_cmd(0x01);  // clear
	_delay_ms(2);
}


static void lcd_set_cursor(uint8_t row, uint8_t col)
{
	uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col); //If the row is 0, place the cursor on the first row, else place it on the second row
	lcd_cmd(addr);
}

static void lcd_print(const char *str)
{
	while (*str)
	lcd_char((uint8_t)(*str++));
}

static void lcd_clear(void)
{
	lcd_cmd(0x01);
	_delay_ms(2);
}
// New LCD functions for scrolling
static void lcd_write_char(uint8_t ch)
{
	lcd_send(ch, 1);
}

void lcd_scroll_text(uint8_t row, const char* message, uint8_t width, uint16_t delay_ms)
{
	uint8_t msg_len = strlen(message);

	if (msg_len <= width) {
		lcd_set_cursor(row, 0);
		lcd_print(message);
		return;
	}

	for (uint8_t i = 0; i <= msg_len - width; i++) {
		lcd_set_cursor(row, 0);
		for (uint8_t j = 0; j < width; j++) {
			lcd_write_char(message[i + j]);
		}
		// Loop-based delay to avoid compile-time constant restriction
		for (uint16_t d = 0; d < delay_ms; d++)
		_delay_ms(1);
	}
}

void lcd_scroll_marquee(uint8_t row, const char* message, uint8_t width, uint16_t delay_ms, uint8_t repeats)
{	
	/* Buffer setup and control guard */
	char padded[64]; //This is where the modified (padded) version of the message will be built
	uint8_t msg_len = strlen(message); // Measures the length of the input string excluding the null terminator. This length drives all the padding calculations below.
	if (msg_len >= 64) return; // Overflow GUARD
	strncpy(padded, message, 63); //Copies up to 63 characters of message into padded
	padded[63] = '\0'; //Forces a null terminator at the very last position

	/* Append trailing spaces so text scrolls cleanly off screen */
	uint8_t pad_end = msg_len + width; //Calculates where the spaces should end
	if (pad_end > 63) pad_end = 63; //Another overflow GUARD
	for (uint8_t i = msg_len; i < pad_end; i++) //Fills positions from msg_len up to pad_end with space characters ' ', then plants a new null terminator at pad_end.
		padded[i] = ' ';
		padded[pad_end] = '\0';
		
	/*The scrolling part*/
	uint8_t padded_len = strlen(padded); //Measures the final length of the padded string (message + trailing spaces). This is the total distance the scroll must travel.
	for (uint8_t r = 0; r < repeats; r++) //Simply runs the entire animation repeats times. If repeats = 3, the full marquee plays through 3 times.
	{
		for (uint8_t i = 0; i <= padded_len - width; i++) //i is the sliding window position — the index into padded where the current visible frame starts. It advances by 1 each iteration, moving the text one character to the left.
		 {
			lcd_set_cursor(row, 0); 
			for (uint8_t j = 0; j < width; j++) //For each scroll position i, this loop writes exactly width characters to the LCD, starting at column 0 of the target row.
			{
				lcd_write_char(padded[i + j]); //This is the key expression - i slides the window while j walks it
			}
			for (uint16_t d = 0; d < delay_ms; d++) //Pauses between each scroll step
			_delay_ms(1);
		}
	}
}



/* 
 * I2C (TWI) DRIVER — Master
 */

static void i2c_init(void) /* I2C initialize function */
{
    TWSR = 0x00;               // prescaler = 1
    TWBR = (uint8_t)TWBR_VAL;
    TWCR = (1 << TWEN);        // enable TWI
}

static uint8_t i2c_start(uint8_t addr_rw)  /* I2C start function */ 
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);// Enable TWI, generate start condition and clear interrupt flag 
    while (!(TWCR & (1<<TWINT))); // Wait until TWI finish its current job (start condition) 
    if ((TWSR & 0xF8) != 0x08) return 0;   ///* Check whether start condition transmitted successfully or not?  If not then return 0 to indicate start condition fail

    TWDR = addr_rw; //If yes then write SLA+W in TWI data register 
    TWCR = (1<<TWINT)|(1<<TWEN); // Enable TWI and clear interrupt flag
    while (!(TWCR & (1<<TWINT))); // Wait until TWI finish its current job (Write operation)
    uint8_t status = TWSR & 0xF8; // Read TWI status register with masking lower three bits
    return (status == 0x18) ? 1 : 0;       //  Check whether SLA+W transmitted & ack received or not?  If yes then return 1 to indicate ack received; ready to accept data elseif return 0 to indicate the SLA+W failed
}

static uint8_t i2c_write(uint8_t data) /* I2C write function */ 
{
    TWDR = data; // Copy data in TWI data register
    TWCR = (1<<TWINT)|(1<<TWEN); // Enable TWI and clear interrupt flag
    while (!(TWCR & (1<<TWINT))); // Wait until TWI finish its current job (Write operation)
    return ((TWSR & 0xF8) == 0x28) ? 1 : 0; // Check weather data transmitted & ack received or not?  If yes then return 0 to indicate ack received 
}

static void i2c_stop(void) /* I2C stop function */ 
{
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO); //Enable TWI, generate stop condition and clear interrupt flag
    _delay_us(50);
}

/* Send mode byte + 16-bit value to slave */
static void i2c_send_command(uint8_t mode, uint16_t value)
{
    if (!i2c_start((I2C_SLAVE_ADDR << 1) | 0)) return;  // If i2c_start fails (returns 0/false — e.g. bus busy, no ACK from slave), the function immediately returns, aborting the transaction. This is a basic error guard.
    i2c_write(mode);
    i2c_write((uint8_t)(value >> 8));    // high byte transmission and 8 bit casting
    i2c_write((uint8_t)(value & 0xFF));  // low byte transmission and 8 bit casting
    i2c_stop();
}

/* 
 * KEYPAD DRIVER  — MM74C922 16-Key Encoder (U4)
 *
 * Reading strategy
 * 
 * The MM74C922 handles all scanning, debouncing, and latching
 * internally. The MCU only needs to:
 *
 *  1. Poll DA (PA4): wait until it goes HIGH — this means a key
 *     has been pressed, debounced, and its code is stable on A–D.
 *
 *  2. Read PA0–PA3: the 4-bit active-HIGH key code (0–15).
 *
 *  3. Look up the ASCII character from kp_code_to_char[].
 *
 *  4. Wait for DA to return LOW (key released) before returning,
 *     so the caller never sees the same keypress twice.
 *
 * No software debounce delay is needed — the MM74C922's on-chip
 * debounce capacitor (CKBM) already guarantees a clean DA signal.
 * */

static void keypad_init(void)
{
    KP_DDR  = 0x00;   /* All PORTA pins are inputs                        */
    KP_PORT = 0x00;   /* No pull-ups: MM74C922 outputs are push-pull     */
                      /* DA (PA4) is actively driven HIGH by the IC too  */
}

/*
 * keypad_scan()
 * Non-blocking: returns 0 immediately if no key is available.
 * Returns the ASCII character once DA goes HIGH, then waits for release.
 */
static char keypad_scan(void)
{
    /*  Step 1: check DA — if LOW, no key ready ?? */
    if (!(KP_PIN & (1 << KP_DA_BIT)))
        return 0;

    /*  Step 2: read the 4-bit key code (active-HIGH, PA0–PA3) ??; KP_CODE_MASK isolates the bits from the other pins  */
    uint8_t code = KP_PIN & KP_CODE_MASK;   /* 0x00–0x0F */

    /*  Step 3: map code to ASCII character ?? */
    char key = kp_code_to_char[code];

    /*  Step 4: wait for key release (DA returns LOW) ?? */
    while (KP_PIN & (1 << KP_DA_BIT));

    return key;
}

/* 
 * INPUT HELPERS
 */

/*
 * Read a multi-digit integer from the keypad via MM74C922.
 *
 * Because the MM74C922 encodes all 16 keys, we can use the
 * natural keypad assignments:
 *   [*]  (code 12) = BACKSPACE — delete last digit
 *   [#]  (code 14) = CONFIRM   — submit the entered value
 *   [A]  (code  3) = CONFIRM   — alternate confirm (same row as mode select)
 *   [D]  (code 15) = CANCEL    — returns 0 (caller will re-show mode menu)
 *
 * Displays digits live on LCD row `lcd_row`.
 * Returns the entered value, or 0 on cancel/empty.
 */






//////////////////// Keypad Reading ////////////////
static uint16_t keypad_read_number(uint8_t lcd_row)
{
	char buf[6] = {0}; //String buffer that holds up to 5 digits (plus the null terminator)
	uint8_t idx = 0; //Keeps track of the number of digits entered

	lcd_set_cursor(lcd_row, 0);
	lcd_print("[*]=DEL [#]=OK  ");

	// This section continuously checks if a key has been pressed
	while (1) {
		char key = keypad_scan();
		if (!key) continue; // If no key is pressed, break out of the loop

		//This means, if a key is pressed and there is space left, append the value to the buffer
		if (key >= '0' && key <= '9' && idx < 5) {
			/* Digit — append to buffer */
			buf[idx++] = key;
			buf[idx]   = '\0';
			lcd_set_cursor(lcd_row, 0);
			lcd_print("                ");
			lcd_set_cursor(lcd_row, 0);
			lcd_print(buf);

			//This is basically a delete key
			} else if (key == '*' && idx > 0) {
			/* [*] = backspace */
			buf[--idx] = '\0';
			lcd_set_cursor(lcd_row, 0);
			lcd_print("                ");
			lcd_set_cursor(lcd_row, 0);
			lcd_print(buf);

			//Confirm key - converts the buffer string into an integer
			} else if (key == '#' && idx > 0) {
			/* #= confirm */
			return (uint16_t)atoi(buf);

			//Emergency stop
			} else if (key == 'D') {
			/* [D] = cancel / emergency stop — return 0, main loop handles */
			return 0;
		}
	}
}

/* 
 * MAIN
 */
int main(void)
{
    lcd_init();
    keypad_init();
    i2c_init();

    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Motor Control");
    lcd_set_cursor(1, 0);
    lcd_print("A=Spd B=Pos D=Stop");
    _delay_ms(100);

    while (1) {
        /*  Prompt user for mode  */
        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_print("A=Speed; B=Pos ;");
        lcd_set_cursor(1, 0);
        lcd_print("D = Stop!!");

        char mode_key = 0;
        while (mode_key != 'A' && mode_key != 'B' && mode_key != 'D') // While keys:A, B and D are not pressed, scan the keypad
            mode_key = keypad_scan();

        if (mode_key == 'D') {
            /*  EMERGENCY STOP  */
            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_print("EMERGENCY STOP");
            lcd_set_cursor(1, 0);
            lcd_print("Motor halted!");
            i2c_send_command(MODE_STOP, 0); //Transmit the emergency stop command to the MCU2
            _delay_ms(3000);
            continue;
        }

        if (mode_key == 'A') {
            /*  SPEED MODE  */
            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_print("SPEED MODE");
            lcd_set_cursor(1, 0);
            lcd_scroll_text(1,"RPM (max 5000):",16,300); //The text scrolls after an interval of 300ms
            _delay_ms(200);
			lcd_clear();
			lcd_set_cursor(0, 0);
			lcd_print("Enter RPM:");
		    uint16_t rpm = keypad_read_number(1); // The 1 is used to test the function i.e read the keypad and start with 1

			if (rpm > 5000) rpm = 5000;  // cap the speed at 5000rpm

			lcd_clear();
			lcd_scroll_text(0,"Setting the speed.....",16,50);
			
			
			lcd_clear();
			lcd_set_cursor(0, 0);
			lcd_print("Speed:");
			char tmp[8];
			itoa(rpm, tmp, 10); // Changes the value obtained from the rpm into a string stored in the tmp, 10 means base 10
			lcd_set_cursor(1, 0);
			lcd_print(tmp);
			lcd_set_cursor(1,5);
			lcd_print("RPM");
			

			i2c_send_command(MODE_SPEED, rpm); //Transmit speed to MCU2
			_delay_ms(1000);

        } else if (mode_key == 'B') {
            /*  POSITION MODE  */
            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_print("POSITION MODE");
            lcd_set_cursor(1, 0);
            lcd_print("Revolutions:");
            _delay_ms(800);

            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_print("EnterRevs:");
            uint16_t revs = keypad_read_number(1);

            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_print("Position set:");
            char tmp[8];
            itoa(revs, tmp, 10);
            lcd_set_cursor(1, 0);
            lcd_print(tmp);
            lcd_print(" rev");

            i2c_send_command(MODE_POSITION, revs); //Transmit position to MCU2
            _delay_ms(1500);
        }
    }
}
