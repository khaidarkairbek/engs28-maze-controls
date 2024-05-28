/* Maze Game 
 * Stepper Motors controlled by the Joystick, represented by two potentiometers, the countdwon shown on the seven segment display
 * Khaidar Kairbek and Brian Castanon Nunez
 * ENGS 28, Lab 9, 05/28/24 
 * 
 * Description:		Joystick controller and Game Logic
 *
 * Target device:	Arduino UNO / SStepper motors / HT16K33 Backpack Seven Segment Display / Potentiometers abd Motor Drivers
 * Tool version:	AVR-GCC
 * Dependencies:	ADC.c, USARTE28.c, ioE28.c SevenSeg.c 
 *
 * Stepper directions: Port B0 (X-direction), B2 (Y-direction)
 * Start and stop sensor: Port B4 and B5
 * Seven Seg Display: SCL, SDA
 * Joystick Potentiometer Inputs: Analog 0 and 1
 * User and Joystick Pushbuttons with pull-ups on: Port D5 and D2
 * Red and Green LEDs: Port D3 and D4
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
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
#define Vcc 5000
#define TURN_MAX_Y 40
#define TURN_MAX_X 40 
#define TIME_LIMIT 90000

typedef enum {
    START, 
    GAME, 
    IDLE, 
    LOSE, 
    WIN
} MAZE_STATE_t;

typedef enum { 
    NORMAL, 
    REBOUNCE
} MOVE_TYPE;

// Function Prototypes
void initIdlePinChangeInterrupt(void);
void initStartPinChangeInterrupt(void);
void initGamePinChangeInterrupt(void);
void moveMotor(uint8_t stepPin, uint8_t dirPin, uint8_t dir, uint16_t steps);
void readJoystickAndMoveMotorsNormal(void);
void readJoystickAndMoveMotorsRebounce(void);
void initTimer1(void);
void initTimer0(void);
void updateDisplay(uint16_t time_in_seconds);
uint8_t SevenSeg_digitToSegment(uint8_t digit);
void handleStateTransition(MAZE_STATE_t newState);

// Global Variables
volatile MAZE_STATE_t current_state = IDLE;
volatile MAZE_STATE_t next_state = IDLE;
volatile uint8_t userButtonStateChange = 0;
volatile uint8_t joystickButtonStateChange = 0;
volatile uint8_t userButtonFlag = 0;
volatile uint8_t joystickButtonFlag = 0;
volatile uint8_t move_enable = 0;
volatile uint8_t timer_count_enable = 0;
volatile uint8_t startFlag = 0;
volatile uint8_t winFlag = 0;
volatile uint16_t time_counter = TIME_LIMIT / 1000;
volatile uint8_t timer1Flag = 0; 
volatile uint8_t joystickUpdateFlag = 0; 
volatile int16_t positionX = 0;
volatile int16_t positionY = 0;
volatile uint16_t oldJoystickX = 512;
volatile uint16_t oldJoystickY = 512;

MOVE_TYPE joystick_movement_type = NORMAL; 
uint16_t display_buffer[HT16K33_NBUF] = {0};

int main(void) {
    // Initialize peripherals
    i2cInit();
    USART_Init();    
    ADC_init();
    SevenSeg_init();
    initIdlePinChangeInterrupt();
    sei(); // Enable global interrupts

    // Configure pins
    DDRB |= (1 << STEP_X_PIN) | (1 << DIR_X_PIN) | (1 << STEP_Y_PIN) | (1 << DIR_Y_PIN);
    DDRD &= ~(1 << DDD5) & ~(1 << DDD2);
    PORTD |= (1 << PORTD5) | (1 << PORTD2);
    DDRB &= ~(1 << DDB5) & ~(1 << DDB4);
    DDRD |= (1 << DDD3) | (1 << DDD4);

    // Initialize timers
    initTimer1();
    initTimer0();

    while (1) {
        if (current_state != LOSE && current_state != IDLE && current_state != WIN ){
            userButtonStateChange = 0;
            joystickButtonStateChange = 0;
            joystickButtonFlag = 0; 
            joystickButtonFlag = 0;
        }


        if (userButtonStateChange){   //react to user button only on rising edge and in relevant states
            userButtonStateChange = 0;
            if (userButtonFlag== 0) {
                userButtonFlag = 1;
            }else{
                userButtonFlag = 0;
            }
        }

        if (joystickButtonStateChange){   //react to joystick button only on rising edge and in relevant states
            joystickButtonStateChange = 0;
            if (joystickButtonFlag == 0) {
                joystickButtonFlag = 1;
            }else{
                joystickButtonFlag = 0;
            }
        }

        switch (current_state) {
            case IDLE:
                printf("Game in Idle\r\n");
                move_enable = 0;
                timer_count_enable = 0;
                time_counter = TIME_LIMIT / 1000;
                // Check which button was pressed to start the game
                if (userButtonFlag == 1) {  // checks which movement type has been chosen
                    joystick_movement_type = NORMAL; 
                    userButtonFlag = 0; 
                    handleStateTransition(START);
                }else if (joystickButtonFlag == 1){
                    joystick_movement_type = REBOUNCE; 
                    joystickButtonFlag = 0;
                    handleStateTransition(START);
                }
                PORTD &= ~(1 << PORTD4); //turn off green LED
                if (timer1Flag){
                    timer1Flag = 0; 
                    // Update LEDs and display
                    if (PORTD & (1 << PORTD3)){
                        PORTD &= ~(1 << PORTD3); // off if it was on before
                    }else{
                        PORTD |= (1 << PORTD3); // on if it was on before
                    }
                }
                updateDisplay(time_counter);
                break;

            case START:
                printf("Game Start\r\n");
                move_enable = 1;
                timer_count_enable = 1;
                time_counter = TIME_LIMIT / 1000;
                // Update LEDs
                PORTD &= ~(1 << PORTD3);  // turn off red led and turn on green led
                PORTD |= (1 << PORTD4);
                if (startFlag == 1) {
                    handleStateTransition(GAME);
                }
                break;

            case GAME:
                printf("Game Playing\r\n");
                move_enable = 1;
                PORTD &= ~(1 << PORTD4);  //turn off green led
                if (time_counter == 0) {
                    handleStateTransition(LOSE);
                }
                if (winFlag == 1) {
                    handleStateTransition(WIN);
                }
                updateDisplay(time_counter);
                break;

            case LOSE:
                move_enable = 0;
                timer_count_enable = 0;
                PORTD &= ~(1 << PORTD4);//turn off green LED and turn on RED LED
                PORTD |= (1 << PORTD3);
                break;

            case WIN:
                move_enable = 0;
                timer_count_enable = 0;
                PORTD &= ~(1 << PORTD3);//turn off RED LED and turn on green LED
                PORTD |= (1 << PORTD4);
                break;
        }

        current_state = next_state;

        // Handle joystick movement outside of ISR
        if (joystickUpdateFlag) {
            joystickUpdateFlag = 0;
            if (joystick_movement_type == NORMAL){
                readJoystickAndMoveMotorsNormal();
            }else if (joystick_movement_type == REBOUNCE){
                readJoystickAndMoveMotorsRebounce();
            }
        }
    }
    return 0;
}

// ISR for pin change interrupt on port D
ISR(PCINT2_vect) {
    if (PIND & (1 << PD2)) {
        userButtonStateChange = 1;
    }
    if (PIND & (1 << PD5)) {
        joystickButtonStateChange = 1;
    }
}

// ISR for pin change interrupt on port B
ISR(PCINT0_vect) {
    if (!(PINB & (1 << PB4))) {
        startFlag = 1;
    }
    if (PINB & (1 << PB5)) {
        winFlag = 1;
    }
}

// Initialize pin change interrupt for idle state
void initIdlePinChangeInterrupt(void) {
    PCICR &= ~(1 << PCIE0);
    PCICR |= (1 << PCIE2);
    PCMSK2 = (1 << PD5) | (1 << PD2);
}

// Initialize pin change interrupt for start state
void initStartPinChangeInterrupt(void) {
    PCICR &= ~(1 << PCIE2);
    PCICR |= (1 << PCIE0);
    PCMSK0 = (1 << PB4);
}

// Initialize pin change interrupt for game state
void initGamePinChangeInterrupt(void) {
    PCICR &= ~(1 << PCIE2);
    PCICR |= (1 << PCIE0);
    PCMSK0 = (1 << PB5);
}

// Function to move the motor
void moveMotor(uint8_t stepPin, uint8_t dirPin, uint8_t dir, uint16_t steps) {
    if (dir == 1) {
        PORTB |= (1 << dirPin);
    } else {
        PORTB &= ~(1 << dirPin);
    }
    for (uint16_t i = 0; i < steps; i++) {
        PORTB |= (1 << stepPin);
        _delay_ms(1);
        PORTB &= ~(1 << stepPin);
        _delay_ms(1);
    }
}

// Read joystick values and move motors in normal mode
void readJoystickAndMoveMotorsNormal(void) {
    printf("Moving\r\n");
    ADC_setReference(V_REF);

    // Read X-axis joystick value
    ADC_setChannel(JOYSTICK_X_CHANNEL);
    uint16_t newJoystickX = ADC_getValue();
    newJoystickX = (newJoystickX > 472 && newJoystickX < 552) ? 512 : newJoystickX;
    int16_t stepsX = (int16_t)((uint32_t)(TURN_MAX_X * newJoystickX) / 512 - TURN_MAX_X);
    if (stepsX > 0) {
        moveMotor(STEP_X_PIN, DIR_X_PIN, 0, (stepsX * 1) / 20);
    } else if (stepsX < 0) {
        moveMotor(STEP_X_PIN, DIR_X_PIN, 1, (abs(stepsX) * 1)/20);
    }

    // Read Y-axis joystick value
    ADC_setChannel(JOYSTICK_Y_CHANNEL);
    uint16_t newJoystickY = ADC_getValue();
    newJoystickY = (newJoystickY > 482 && newJoystickY < 542) ? 512 : newJoystickY;
    int16_t stepsY = (int16_t)((uint32_t)(TURN_MAX_Y * newJoystickY) / 512 - TURN_MAX_Y);
    if (stepsY > 0) {
        moveMotor(STEP_Y_PIN, DIR_Y_PIN, 1, (stepsY * 1) / 20 );
    } else if (stepsY < 0) {
        moveMotor(STEP_Y_PIN, DIR_Y_PIN, 0, (abs(stepsY) * 1)/20);
    }
}

// Read joystick values and move motors in rebounce mode
void readJoystickAndMoveMotorsRebounce(void) {
    printf("Moving\r\n");
    ADC_setReference(V_REF); // Set reference 

    // Read X-axis joystick value
    ADC_setChannel(JOYSTICK_X_CHANNEL); // Get value for X
    uint16_t newJoystickX = ADC_getValue();

    if (newJoystickX > 482 && newJoystickX < 542){
        newJoystickX = 512;
    }

    int16_t diffJoystickX = newJoystickX - oldJoystickX;
    oldJoystickX = newJoystickX; 
    int16_t stepsX = (int16_t)((uint32_t)(TURN_MAX_X * diffJoystickX) / 512);
    
    if (stepsX > 0) {
        moveMotor(STEP_X_PIN, DIR_X_PIN, 0, stepsX);
    } else if (stepsX < 0) {
        moveMotor(STEP_X_PIN, DIR_X_PIN, 1, (abs(stepsX) * 7)/8);
    }

    // Read Y-axis joystick value
    ADC_setChannel(JOYSTICK_Y_CHANNEL); // Get value for X
    uint16_t newJoystickY = ADC_getValue();

    if (newJoystickY > 482 && newJoystickY < 542){
        newJoystickY = 512;
    }

    int16_t diffJoystickY = newJoystickY - oldJoystickY;
    oldJoystickY = newJoystickY; 
    int16_t stepsY = (int16_t)((uint32_t)(TURN_MAX_Y * diffJoystickY) / 512);
    
    if (stepsY > 0) {
        moveMotor(STEP_Y_PIN, DIR_Y_PIN, 1, stepsY);
    } else if (stepsY < 0) {
        moveMotor(STEP_Y_PIN, DIR_Y_PIN, 0, (abs(stepsY) *7) / 8);
    }
}

// Initialize Timer 1
void initTimer1(void) {
    TCCR1B |= (1 << WGM12);  // Configure timer 1 for CTC mode
    OCR1A = 15624; // Set CTC compare value to 1Hz at 16MHz AVR clock, with a prescaler of 1024
    TIMSK1 |= (1 << OCIE1A); // Enable CTC interrupt
    TCCR1B |= (1 << CS12) | (1 << CS10); // Start timer at 1024 prescale
}

// ISR for Timer 1 compare match
ISR(TIMER1_COMPA_vect) {
    if (timer_count_enable) {
        if (time_counter > 0) {
            time_counter--;
        }
    }
    timer1Flag = 1;
}

// Initialize Timer 0
void initTimer0(void) {
    TCCR0A |= (1 << WGM01); // Configure timer 0 for CTC mode
    OCR0A = 155; // Set CTC compare value to 100Hz at 16MHz AVR clock, with a prescaler of 1024
    TIMSK0 |= (1 << OCIE0A); // Enable CTC interrupt
    TCCR0B |= (1 << CS02) | (1 << CS00); // 1024 prescaler
}

// ISR for Timer 0 compare match
ISR(TIMER0_COMPA_vect) {
    if (move_enable) {
        joystickUpdateFlag = 1;  // Set flag instead of calling the function directly
    }
}

// Update the 7-segment display with the current time
void updateDisplay(uint16_t time_in_seconds) {
    uint16_t minutes = time_in_seconds / 60;
    uint16_t seconds = time_in_seconds % 60;
    for (int i = 0; i < HT16K33_NBUF; i++) {
        display_buffer[i] = 0;
    }
    if (minutes > 9) {
        display_buffer[0] = SevenSeg_digitToSegment(minutes / 10);
    } else {
        display_buffer[0] = 0;
    }
    display_buffer[1] = SevenSeg_digitToSegment(minutes % 10);
    display_buffer[2] = 0x02;
    display_buffer[3] = SevenSeg_digitToSegment(seconds / 10);
    display_buffer[4] = SevenSeg_digitToSegment(seconds % 10);
    SevenSeg_write(display_buffer);
}

// Convert digit to 7-segment display segments
uint8_t SevenSeg_digitToSegment(uint8_t digit) {
    static const uint8_t segmentMap[10] = {
        0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
    };
    if (digit < 10) {
        return segmentMap[digit];
    }
    return 0;
}

// Handle state transition
void handleStateTransition(MAZE_STATE_t newState) {
    if (current_state != newState) {
        userButtonStateChange = 0;
        joystickButtonStateChange = 0;
        joystickButtonFlag = 0; 
        userButtonFlag = 0;
        startFlag = 0;
        winFlag = 0;
        switch (newState) {
            case IDLE:
                initIdlePinChangeInterrupt();
                break;
            case START:
                initStartPinChangeInterrupt();
                break;
            case GAME: 
                initGamePinChangeInterrupt();
                break;
            case LOSE:
            case WIN: 
                // No need to re-initialize interrupts
                break;
        }
        next_state = newState;
    }
}