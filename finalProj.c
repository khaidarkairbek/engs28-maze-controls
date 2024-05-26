#include <avr/io.h>             
#include <avr/interrupt.h>
#include <stdlib.h> // Include the header file that declares the abs() function
#include <util/delay.h>
#include "ADC.h"
#include "USARTE28.h"
#include "ioE28.h"
#include "SevenSeg.h"

#define DIR_X_PIN PB0
#define STEP_X_PIN PB1
#define DIR_Y_PIN PB2
#define STEP_Y_PIN PB3
#define JOYSTICK_X_CHANNEL 1
#define JOYSTICK_Y_CHANNEL 0
#define V_REF 5

#define TIME_LIMIT 90000 // Time limit in milliseconds (60 seconds)

typedef enum {
    START, 
    GAME, 
    IDLE, 
    LOSE
} MAZE_STATE_t; 

void initIdlePinChangeInterrupt(void);
void initStartPinChangeInterrupt(void);
void initGamePinChangeInterrupt(void);
void moveMotor(uint8_t stepPin, uint8_t dirPin, uint8_t dir, uint16_t steps);
void readJoystickAndMoveMotors(void); 
void initTimer1(void); // Function to initialize Timer1
void initTimer0(void); // Function to initialize Timer0
void updateDisplay(uint16_t time_in_seconds); // Function to update the display

/* GLOBAL VARIABLES */
volatile MAZE_STATE_t current_state, next_state = IDLE; 
volatile uint8_t userButtonStateChange;
volatile uint8_t move_enable, timer_count_enable; 
volatile uint8_t startFinishFlag;
volatile uint16_t time_counter = 0; // Variable to count timer overflows

uint16_t display_buffer[HT16K33_NBUF] = {0}; // Display buffer

// Main program
int main(void) {
  i2cInit();
  USART_Init();    
  ADC_init();
  SevenSeg_init(); // initialize Seven Segment display
  initIdlePinChangeInterrupt();
  sei(); 
  
  DDRB |= (1 << STEP_X_PIN) | (1 << DIR_X_PIN); // Configure PD6 and PD7 as outputs for X motor
  DDRB |= (1 << STEP_Y_PIN) | (1 << DIR_Y_PIN); // Configure PB0 and PB1 as outputs for Y motor

  DDRD  &= ~(1 << DDD5) & ~(1 << DDD2);         // Configure pin D5 as input for user button and D2 for joystick button
  PORTD |= (1 << PORTD5) | (1 << PORTD2);           // Turn pullup on for buttons
  DDRB  &= ~(1 << DDB5) & ~(1 << DDB4);         // Configure pin B5 as stop stop sensor and B4 as start sensor
  DDRD |= (1 << DDD3) | (1 << DDD4);         // configure bit 3 as output for red LED and bit 4 as output for green LED

  initTimer1(); // Initialize Timer1
  initTimer0(); // Initialize Timer0

     

  while(1) {
    switch(current_state) {
        case IDLE: 
	    printf("Game in Idle\r\n");
            move_enable = 0; 
            timer_count_enable = 0;
            if (userButtonStateChange == 1){
                next_state = START;
                userButtonStateChange = 0; 
            }
            PORTD &= (1 << PORTD3);
            PORTD |= (1 << PORTD3);     // set bit 3  to turn red LED on
            break; 
        case START:
            printf("Game Start\r\n");
            move_enable = 1; 
            timer_count_enable = 0;
            time_counter = 0; // Reset time counter
            PORTD &= (1 << PORTD4);
            PORTD |= (1 << PORTD4);     // set bit 4  to turn green LED on
            if (startFinishFlag == 1){
                next_state = GAME; 
                startFinishFlag = 0;
            }
            break;
        case GAME:
            printf("Game Playing\r\n"); 
            move_enable = 1; 
            timer_count_enable = 1; 
            PORTD &= ~(1 << PORTD4);  // turn green LED off
            if (time_counter >= TIME_LIMIT / 1000) { // Check if time limit reached
                next_state = LOSE;
            }
            updateDisplay(time_counter); // Update display with current time
            break;
        case LOSE:
            move_enable = 0;
            timer_count_enable = 0;
            PORTD &= ~(1 << PORTD4);  // turn green LED off
            PORTD |= (1 << PORTD3);   // turn red LED on
            break;
    }  

    switch(next_state){
        case IDLE: 
            if (current_state != next_state){
                current_state = next_state; 
                initIdlePinChangeInterrupt();
            }
            break; 
        case START: 
            if (current_state != next_state){
                current_state = next_state; 
                initStartPinChangeInterrupt();
            }
            break; 
        case GAME: 
            if (current_state != next_state){
                current_state = next_state; 
                initGamePinChangeInterrupt();
            }
            break;
        case LOSE:
            if (current_state != next_state){
                current_state = next_state;
                // No need to re-initialize interrupts for LOSE state
            }
            break;
    }
  }
  return 0;                            /* This line is never reached */
}

ISR(PCINT2_vect) {
   userButtonStateChange = 1;               // Set flag to notify main
}

void initIdlePinChangeInterrupt(void) {
    PCICR  |= (1 << PCIE2);         // Enable pin-change interrupt for D-pins
    PCMSK2 |= (1 << PD5);           // Set pin mask for bit 5 of Port D
}

ISR(PCINT0_vect) {
   startFinishFlag = 1;                 // Set flag to notify main
}

void initStartPinChangeInterrupt(void){
    PCICR  &= ~(1 << PCIE2);
    PCICR  |= (1 << PCIE0);         // Enable pin-change interrupt for B-pins
    PCMSK0 |= (1 << PB4);           // Set pin mask for bit 4 of Port B Start sensor
}

void initGamePinChangeInterrupt(void){
    PCICR  |= (1 << PCIE0);         // Enable pin-change interrupt for B-pins
    PCMSK0 &= ~(1 << PB4);
    PCMSK0 |= (1 << PB5);           // Set pin mask for bit 5 of Port B Stop sensor
}

void moveMotor(uint8_t stepPin, uint8_t dirPin, uint8_t dir, uint16_t steps) {
    // Set direction
    if (dir == 1) {
        PORTB |= (1 << dirPin);
    } else {
        PORTB &= ~(1 << dirPin);
    }

    // Step motor
    for (uint16_t i = 0; i < steps; i++) {
        PORTB |= (1 << stepPin);
        _delay_ms(1);
        PORTB &= ~(1 << stepPin);
        _delay_ms(1);
    }
}

void readJoystickAndMoveMotors(void) {
    printf("Moving\r\n");
    ADC_setReference(V_REF); // Set reference 

    ADC_setChannel(JOYSTICK_X_CHANNEL); // Get value for X
    uint16_t joystickX = ADC_getValue();

    ADC_setChannel(JOYSTICK_Y_CHANNEL); // Get Value for Y
    uint16_t joystickY = ADC_getValue();

    uint16_t stepsX = abs(joystickX - 512) / 10; // Adjust scaling as necessary
    uint16_t stepsY = abs(joystickY - 512) / 10;

    // Move depending on X and Y potentiometer values
    if (joystickX > 522) {
        moveMotor(STEP_X_PIN, DIR_X_PIN, 0, stepsX);
    } else if (joystickX < 502) {
        moveMotor(STEP_X_PIN, DIR_X_PIN, 1, stepsX);
    }

    if (joystickY > 522) {
        moveMotor(STEP_Y_PIN, DIR_Y_PIN, 1, stepsY);
    } else if (joystickY < 502) {
        moveMotor(STEP_Y_PIN, DIR_Y_PIN, 0, stepsY);
    }
}

void initTimer1(void) {
    // Set Timer1 to CTC mode
    TCCR1B |= (1 << WGM12);

    // Set the value for OCR1A for 1 second interrupts
    OCR1A = 15624; // Assuming a 16MHz clock and 1024 prescaler

    // Enable Timer1 compare interrupt
    TIMSK1 |= (1 << OCIE1A);

    // Set the prescaler to 1024 and start the timer
    TCCR1B |= (1 << CS12) | (1 << CS10);
}

ISR(TIMER1_COMPA_vect) {
    if (timer_count_enable) {
        time_counter++; // Increment the time counter every second
    }
}

// Initialize Timer0 for joystick reading and motor control
void initTimer0(void) {
    // Set Timer0 to CTC mode
    TCCR0A |= (1 << WGM01);

    // Set the value for OCR0A for 10ms interrupts (100Hz)
    OCR0A = 155; // Assuming a 16MHz clock and 1024 prescaler

    // Enable Timer0 compare interrupt
    TIMSK0 |= (1 << OCIE0A);

    // Set the prescaler to 1024 and start the timer
    TCCR0B |= (1 << CS02) | (1 << CS00);
}

ISR(TIMER0_COMPA_vect) {
    if (move_enable) {
        readJoystickAndMoveMotors();
    }
}

void updateDisplay(uint16_t time_in_seconds) {
    SevenSeg_number(time_in_seconds, display_buffer); // Convert time to display format
    SevenSeg_write(display_buffer); // Write to the display
}

