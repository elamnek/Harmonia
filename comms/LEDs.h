// LEDs.h

#ifndef _LEDS_h
#define _LEDS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void init_leds();

void red_led_on();
void red_led_off();
void orange_led_on();
void orange_led_off();
void yellow_led_on();
void yellow_led_off();

void toggle_orange_led();

#endif

