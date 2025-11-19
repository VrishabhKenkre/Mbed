// led_gummy.c
// Implementation of RGB LED + sensor helper functions.

#include "led_gummy.h"
#include "hardware_stm_gpio.h"   // your header with PORT* and GPIO_* macros
#include <stdint.h>


void redLED_On(void) { 
    setGPIOB0(); 
    }
void redLED_Off(void) { 
    clearGPIOB0(); 
    }

void greenLED_On(void) { 
    setGPIOB1(); 
    }
void greenLED_Off(void){ 
    clearGPIOB1(); 
    }

void blueLED_On(void)  { 
    setGPIOB2(); 
    }
void blueLED_Off(void) { 
    clearGPIOB2(); 
    }

void allLEDs_Off(void) {
    clearGPIOB0();
    clearGPIOB1();
    clearGPIOB2();
}