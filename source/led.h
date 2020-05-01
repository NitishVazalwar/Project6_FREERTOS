/*
 * led.h
 *
 *  Created on: Apr 4, 2020
 *      Author: nitis
 */

#ifndef LED_H_
#define LED_H_

// Freedom KL25Z LEDs
#define RED_LED_POS (18)		// on port B
#define GREEN_LED_POS (19)	// on port B
#define BLUE_LED_POS (1)		// on port D

// function prototypes
void Init_RGB_LEDs(void);
void Control_RGB_LEDs(unsigned int red_on, unsigned int green_on, unsigned int blue_on);
void Toggle_RGB_LEDs(unsigned int red, unsigned int green, unsigned int blue);
void led_off(void);



#endif /* LED_H_ */
