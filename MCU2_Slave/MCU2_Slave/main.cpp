/*
 * MCU2_slave.c
 * Motor Control Project - SLAVE (ATmega32)
 *
 * Responsibilities:
 *   - Receive mode + value from MCU1 (Master) via I2C
 *   - Control motor speed using PWM on PB3 (OC0)
 *   - Monitor shaft rotation via incremental encoder (INT0/INT1)
 *   - Display current & target values on 16x2 LCD
 *
 * Connections:
 *   Motor PWM      -> PB3 (OC0, via motor driver e.g. L298N)
 *   Motor DIR      -> PB0, PB1 (L298N IN1/IN2)
 *   Encoder Ch A   -> PD2 (INT0)
 *   Encoder Ch B   -> PD3 (INT1)
 *   LCD (4-bit)    -> PORTC (PC2=RS, PC3=EN, PC4-PC7=D4-D7)
 *   I2C SDA        -> PC1 (TWI SDA)
 *   I2C SCL        -> PC0 (TWI SCL)
 *
 * Encoder spec: 20 pulses/rev (PPR), quadrature ? ×4 = 80 counts/rev
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

/* ???????????????????????????????????????????
 * Encoder & motor parameters
 * ??????????????????????????????????????????? */
#define PULSES_PER_REV   20UL          // encoder lines per revolution
#define COUNTS_PER_REV   (PULSES_PER_REV * 4UL)  // quadrature ×4 = 80

/* Speed PID sample interval: Timer1 fires every 100 ms */
#define SPEED_SAMPLE_MS  100UL
/* Timer1 compare value for 100 ms at F_CPU=8MHz, prescaler=1024 */
#define T1_COMPARE       ((F_CPU / 1024UL) * SPEED_SAMPLE_MS / 1000UL - 1UL)

/* ???????????????????????????????????????????
 * I2C slave address (must match master)
 * ??????????????????????????????????????????? */
#define I2C_MY_ADDR  0x10

/* ???????????????????????????????????????????
 * Mode constants
 * ??????????????????????????????????????????? */
#define MODE_SPEED     'S'
#define MODE_POSITION  'P'
#define MODE_STOP      'E'

/* ???????????????????????????????????????????
 * Motor direction pins (PB0, PB1 ? L298N)
 * ??????????????????????????????????????????? */
#define MOTOR_IN1  PB0
#define MOTOR_IN2  PB1

/* ???????????????????????????????????????????
 * LCD pins (same as master)
 * ??????????????????????????????????????????? */
#define LCD_PORT  PORTC
#define LCD_DDR   DDRC
#define LCD_RS    PC2
#define LCD_EN    PC3
#define LCD_D4    PC4
#define LCD_D5    PC5
#define LCD_D6    PC6
#define LCD_D7    PC7

/* ???????????????????????????????????????????
 * GLOBAL STATE
 * ??????????????????????????????????????????? */

volatile int32_t  g_encoder_count  = 0;    // running quadrature count
volatile uint8_t  g_mode           = MODE_STOP;
volatile uint16_t g_target         = 0;    // RPM or revolutions

/* Speed control */
volatile int32_t  g_prev_count     = 0;    // encoder count at last sample
volatile uint8_t  g_pwm_duty       = 0;    // current OCR0 value (0-255)
volatile uint16_t g_current_rpm    = 0;    // measured RPM

/* Position control */
volatile int32_t  g_target_counts  = 0;    // encoder counts to reach target
volatile uint8_t  g_position_done  = 0;

/* I2C receive buffer */
#define I2C_BUF_LEN 3
volatile uint8_t  g_i2c_buf[I2C_BUF_LEN];
volatile uint8_t  g_i2c_idx        = 0;
volatile uint8_t  g_i2c_received   = 0;

/* ???????????????????????????????????????????
 * LCD DRIVER (identical to master)
 * ??????????????????????????????????????????? */

static void lcd_pulse_enable(void)
{
    LCD_PORT |=  (1 << LCD_EN);
    _delay_us(1);
    LCD_PORT &= ~(1 << LCD_EN);
    _delay_us(50);
}

static void lcd_write_nibble(uint8_t nibble)
{
    LCD_PORT = (LCD_PORT & 0x0F) | ((nibble & 0x0F) << 4);
    lcd_pulse_enable();
}

static void lcd_send(uint8_t data, uint8_t is_data)
{
    if (is_data)
        LCD_PORT |=  (1 << LCD_RS);
    else
        LCD_PORT &= ~(1 << LCD_RS);

    lcd_write_nibble(data >> 4);
    lcd_write_nibble(data & 0x0F);

    if ((data == 0x01) || (data == 0x02))
        _delay_ms(2);
}

static void lcd_cmd(uint8_t cmd)  { lcd_send(cmd, 0); }
static void lcd_char(uint8_t ch)  { lcd_send(ch,  1); }

static void lcd_init(void)
{
    LCD_DDR |= (1<<LCD_RS)|(1<<LCD_EN)|(1<<LCD_D4)|(1<<LCD_D5)|(1<<LCD_D6)|(1<<LCD_D7);
    _delay_ms(50);

    lcd_write_nibble(0x03); _delay_ms(5);
    lcd_write_nibble(0x03); _delay_ms(1);
    lcd_write_nibble(0x03); _delay_ms(1);
    lcd_write_nibble(0x02); _delay_ms(1);

    lcd_cmd(0x28);
    lcd_cmd(0x0C);
    lcd_cmd(0x06);
    lcd_cmd(0x01);
    _delay_ms(2);
}

static void lcd_set_cursor(uint8_t row, uint8_t col)
{
    lcd_cmd((row == 0) ? (0x80 + col) : (0xC0 + col));
}

static void lcd_print(const char *str)
{
    while (*str) lcd_char((uint8_t)(*str++));
}

static void lcd_clear(void)
{
    lcd_cmd(0x01);
    _delay_ms(2);
}

/* Print a right-aligned number in a field of `width` chars */
static void lcd_print_num(uint16_t val, uint8_t width)
{
    char buf[8];
    itoa(val, buf, 10);
    uint8_t len = (uint8_t)strlen(buf);
    for (uint8_t i = len; i < width; i++) lcd_char(' ');
    lcd_print(buf);
}

/* ???????????????????????????????????????????
 * MOTOR CONTROL HELPERS
 * ??????????????????????????????????????????? */

static void motor_init(void)
{
    /* PB3 = OC0 (PWM), PB0/PB1 = direction */
    DDRB |= (1 << PB3) | (1 << MOTOR_IN1) | (1 << MOTOR_IN2);

    /* Timer0: Fast PWM, non-inverting, prescaler=8 ? ~3.9 kHz PWM */
    TCCR0 = (1 << WGM00) | (1 << WGM01) |   // Fast PWM
            (1 << COM01) |                    // non-inverting OC0
            (1 << CS01);                      // clk/8
    OCR0 = 0;                                 // start at 0% duty
}

static void motor_set_pwm(uint8_t duty)
{
    g_pwm_duty = duty;
    OCR0 = duty;
}

static void motor_forward(void)
{
    PORTB |=  (1 << MOTOR_IN1);
    PORTB &= ~(1 << MOTOR_IN2);
}

static void motor_stop(void)
{
    PORTB &= ~((1 << MOTOR_IN1) | (1 << MOTOR_IN2));
    motor_set_pwm(0);
}

/* ???????????????????????????????????????????
 * ENCODER INTERRUPTS (quadrature on INT0/INT1)
 *
 * Quadrature decoding using both edges of both channels:
 *   INT0 = Channel A (PD2)
 *   INT1 = Channel B (PD3)
 * ??????????????????????????????????????????? */

ISR(INT0_vect)   // Channel A edge
{
    uint8_t a = (PIND >> PD2) & 1;
    uint8_t b = (PIND >> PD3) & 1;
    /* CW: A rises while B=0, or A falls while B=1 */
    if (a == b)  g_encoder_count++;
    else         g_encoder_count--;
}

ISR(INT1_vect)   // Channel B edge
{
    uint8_t a = (PIND >> PD2) & 1;
    uint8_t b = (PIND >> PD3) & 1;
    /* CW: B rises while A=1, or B falls while A=0 */
    if (a != b)  g_encoder_count++;
    else         g_encoder_count--;
}

static void encoder_init(void)
{
    DDRD  &= ~((1 << PD2) | (1 << PD3));   // inputs
    PORTD |=  (1 << PD2) | (1 << PD3);     // pull-ups

    /* Trigger INT0 and INT1 on ANY logic change */
    MCUCR |= (1 << ISC00) | (1 << ISC10);
    MCUCR &= ~((1 << ISC01) | (1 << ISC11));
    GICR  |= (1 << INT0) | (1 << INT1);
}

/* ???????????????????????????????????????????
 * TIMER1 — Speed sampling (100 ms CTC)
 * ??????????????????????????????????????????? */

ISR(TIMER1_COMPA_vect)
{
    if (g_mode != MODE_SPEED) return;

    int32_t current = g_encoder_count;
    int32_t delta   = current - g_prev_count;  // counts in 100 ms
    g_prev_count    = current;

    /* RPM = (counts / COUNTS_PER_REV) / (sample_s) * 60 */
    /* = delta * 60 * 10 / COUNTS_PER_REV   (sample_s = 0.1) */
    if (delta < 0) delta = -delta;            // absolute value
    g_current_rpm = (uint16_t)((delta * 600UL) / COUNTS_PER_REV);

    /* Simple bang-bang controller (can be replaced with PID) */
    uint16_t target = g_target;
    if (g_current_rpm < target) {
        if (g_pwm_duty < 255) motor_set_pwm(g_pwm_duty + 5);
    } else if (g_current_rpm > target) {
        if (g_pwm_duty > 0)   motor_set_pwm(g_pwm_duty - 5);
    }
}

static void timer1_init(void)
{
    /* CTC mode, prescaler=1024 */
    TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10);
    OCR1A  = (uint16_t)T1_COMPARE;
    TIMSK |= (1 << OCIE1A);
}

/* ???????????????????????????????????????????
 * I2C (TWI) SLAVE
 *
 * Packet from master: [mode_byte, value_high, value_low]
 * ??????????????????????????????????????????? */

static void i2c_slave_init(void)
{
    TWAR = (I2C_MY_ADDR << 1);             // set slave address
    TWCR = (1<<TWEA)|(1<<TWEN)|(1<<TWIE); // enable, ack, interrupt
}

ISR(TWI_vect)
{
    uint8_t status = TWSR & 0xF8;

    switch (status) {
        case 0x60:   /* SLA+W received, ACK sent */
        case 0x70:   /* general call, ACK sent */
            g_i2c_idx = 0;
            TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWEA);
            break;

        case 0x80:   /* data received, ACK sent */
        case 0x90:
            if (g_i2c_idx < I2C_BUF_LEN) {
                g_i2c_buf[g_i2c_idx++] = TWDR;
            }
            TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWEA);
            break;

        case 0xA0:   /* STOP or repeated START received */
            if (g_i2c_idx == I2C_BUF_LEN)
                g_i2c_received = 1;          // signal main loop
            TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWEA);
            break;

        default:
            TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWEA);
            break;
    }
}

/* ???????????????????????????????????????????
 * COMMAND PROCESSING
 * ??????????????????????????????????????????? */

static void process_command(uint8_t mode, uint16_t value)
{
    switch (mode) {
        case MODE_STOP:
            g_mode = MODE_STOP;
            motor_stop();
            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_print("!! E-STOP !!");
            lcd_set_cursor(1, 0);
            lcd_print("Motor halted");
            break;

        case MODE_SPEED:
            g_mode        = MODE_SPEED;
            g_target      = value;
            g_prev_count  = g_encoder_count;
            g_current_rpm = 0;
            if (value == 0) {
                motor_stop();
            } else {
                motor_forward();
                motor_set_pwm(80);  // initial guess ~30% duty
            }
            break;

        case MODE_POSITION:
            g_mode          = MODE_POSITION;
            g_target        = value;
            g_encoder_count = 0;             // reset position reference
            g_target_counts = (int32_t)value * (int32_t)COUNTS_PER_REV;
            g_position_done = 0;
            if (value == 0) {
                motor_stop();
                g_position_done = 1;
            } else {
                motor_forward();
                motor_set_pwm(150);          // fixed duty for positioning
            }
            break;

        default:
            break;
    }
}

/* ???????????????????????????????????????????
 * LCD STATUS UPDATE
 * ??????????????????????????????????????????? */

static void update_lcd_speed(void)
{
    lcd_set_cursor(0, 0);
    lcd_print("TGT:");
    lcd_print_num(g_target, 4);
    lcd_print(" RPM");

    lcd_set_cursor(1, 0);
    lcd_print("CUR:");
    lcd_print_num(g_current_rpm, 4);
    lcd_print(" RPM");
}

static void update_lcd_position(void)
{
    uint32_t current_counts = (g_encoder_count < 0) ? 0 : (uint32_t)g_encoder_count;
    uint16_t current_revs   = (uint16_t)(current_counts / COUNTS_PER_REV);

    lcd_set_cursor(0, 0);
    lcd_print("TGT:");
    lcd_print_num(g_target, 4);
    lcd_print(" rev");

    lcd_set_cursor(1, 0);
    lcd_print("CUR:");
    lcd_print_num(current_revs, 4);
    lcd_print(g_position_done ? " DONE" : " rev ");
}

/* ???????????????????????????????????????????
 * MAIN
 * ??????????????????????????????????????????? */
int main(void)
{
    lcd_init();
    motor_init();
    encoder_init();
    timer1_init();
    i2c_slave_init();

    sei();   // enable global interrupts

    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("MCU2 Ready");
    lcd_set_cursor(1, 0);
    lcd_print("Awaiting cmd...");

    uint8_t  lcd_refresh_cnt = 0;

    while (1) {
        /* ?? Process received I2C packet ?? */
        if (g_i2c_received) {
            g_i2c_received = 0;
            uint8_t  mode  = g_i2c_buf[0];
            uint16_t value = ((uint16_t)g_i2c_buf[1] << 8) | g_i2c_buf[2];

            process_command(mode, value);

            if (mode == MODE_SPEED || mode == MODE_POSITION)
                lcd_clear();   // fresh screen for status
        }

        /* ?? Position mode: check if target reached ?? */
        if (g_mode == MODE_POSITION && !g_position_done) {
            if (g_encoder_count >= g_target_counts) {
                motor_stop();
                g_position_done = 1;
            }
        }

        /* ?? Refresh LCD every ~200 ms (no blocking delays) ?? */
        if (++lcd_refresh_cnt >= 200) {
            lcd_refresh_cnt = 0;
            if (g_mode == MODE_SPEED)
                update_lcd_speed();
            else if (g_mode == MODE_POSITION)
                update_lcd_position();
        }

        _delay_ms(1);
    }
}

