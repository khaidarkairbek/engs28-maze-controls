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
    LOSE, 
    WIN
} MAZE_STATE_t; 

volatile MAZE_STATE_t current_state; 

// Main program
int main(void) {
  i2cInit();
  USART_Init();    
  ADC_init();
  SevenSeg_init(); // initialize Seven Segment display
  sei(); 
  
  while(1) {

    switch(current_state){
        case IDLE: 

        case START:
        case GAME: 
        case LOSE:
        case WIN: 

    }  
  }
  return 0;                            /* This line is never reached */
}