/* Servo motor controlled by the lighting conditions measured via two photocell on solar panel
 * and depicting the angle of the solar panel on the seven seg display
 * Khaidar Kairbek
 * ENGS 28, Lab 8, 05/21/24 
 * 
 * Description:		Photocells --> ADC --> PWM --> servomotor and SevenSeg
 *
 * Target device:	Arduino UNO / SG92R servomotor / HT16K33 Backpack Seven Segment Display / PDV-P8001 Photocells / Voltage Regulator
 * Tool version:	AVR-GCC
 * Dependencies:	ADC.c, USARTE28.c, ioE28.c SevenSeg.c 
 *
 * Photocells:  	Analog channels 0 (A0), 1 (A1)
 * Seven Seg Display: SCL, SDA
 * Servo PWM input:	PD9
 *
 */

#include <avr/io.h>				
#include <avr/interrupt.h>
#include <stdlib.h> // Include the header file that declares the abs() function
#include "ADC.h"
#include "USARTE28.h"
#include "ioE28.h"
#include "SevenSeg.h"


typedef enum {
    START, 
    GAME, 
    IDLE, 
    //LOSE, 
    //WIN
} MAZE_STATE_t; 
void initIdlePinChangeInterrupt(void);
void initStartPinChangeInterrupt(void);
void initGamePinChangeInterrupt(void);

/* GLOBAL VARIABLES */
volatile MAZE_STATE_t current_state, next_state = IDLE; 
volatile uint8_t userButtonStateChange;
volatile uint8_t move_enable, timer_count_enable; 
volatile uint8_t startFinishFlag;

// Main program
int main(void) {
  i2cInit();
  USART_Init();    
  ADC_init();
  SevenSeg_init(); // initialize Seven Segment display
  initIdlePinChangeInterrupt();
  sei(); 

  DDRD  &= ~(1 << DDD5) & ~(1 << DDD2);			// Configure pin D5 as input for user button and D2 for joystick button
  PORTD |= (1 << PORTD5) | (1 << PORTD2);			// Turn pullup on for buttons
  DDRB  &= ~(1 << DDB5) & ~(1 << DDB4);			// Configure pin B5 as stop stop sensor and B4 as start sensor
  DDRD |= (1 << DDD3) | (1 << DDD4);         // configure bit 3 as output for red LED and bit 4 as output for green LED

  while(1) {
    switch(current_state){
        case IDLE: 
            move_enable = 0; 
            timer_count_enable = 0;
            if (userButtonStateChange == 1){
                next_state = START;
                userButtonStateChange = 0; 
            }
            PORTD &= (1 << PORTD3);
            PORTD |= (1 << PORTD3);		// set bit 3  to turn red LED on
            break; 
        case START:
            move_enable = 1; 
            timer_count_enable = 0;
            PORTD &= (1 << PORTD4);
            PORTD |= (1 << PORTD4);		// set bit 4  to turn green LED on
            if (startFinishFlag == 1){
                next_state = GAME; 
                startFinishFlag = 0;
            }
            break;
        case GAME: 
            move_enable = 1; 
            timer_count_enable = 1; 
            PORTD &= ~(1 << PORTD4);  // turn green LED off
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
    }
  }
  return 0;                            /* This line is never reached */
}

ISR(PCINT2_vect) {
   userButtonStateChange = 1; 				// Set flag to notify main
}

void initIdlePinChangeInterrupt(void) {
    PCICR  |= (1 << PCIE2); 		// Enable pin-change interrupt for D-pins
    PCMSK2 |= (1 << PD5); 			// Set pin mask for bit 5 of Port D
}

ISR(PCINT0_vect) {
   startFinishFlag = 1; 				// Set flag to notify main
}

void initStartPinChangeInterrupt(void){
    PCICR  &= ~(1 << PCIE2);
    PCICR  |= (1 << PCIE0); 		// Enable pin-change interrupt for B-pins
    PCMSK0 |= (1 << PB4); 			// Set pin mask for bit 4 of Port B Start sensor
}

void initGamePinChangeInterrupt(void){
    PCICR  |= (1 << PCIE0); 		// Enable pin-change interrupt for B-pins
    PCMSK0 &= ~(1 << PB4);
    PCMSK0 |= (1 << PB5); 			// Set pin mask for bit 4 of Port B Start sensor
}