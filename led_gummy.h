// led_gummy.h
// Helper functions for driving RGB LEDs (PB0, PB1, PB2)
// and reading the light sensor on PC6.

#ifndef LED_GUMMY_H
#define LED_GUMMY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void redLED_On(void);
void redLED_Off(void);

void greenLED_On(void);
void greenLED_Off(void);

void blueLED_On(void);
void blueLED_Off(void);

// turn all three LEDs off
void allLEDs_Off(void);

// Read comparator / sensor output on PC6
// Returns 1 if logic HIGH (light detected), 0 if LOW.


#ifdef __cplusplus
}
#endif

#endif 