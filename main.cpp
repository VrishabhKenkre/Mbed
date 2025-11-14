#include "main.h"
#include "debug_mort.h"
#include "nucleo_led.h"
#include "hardware_stm_gpio.h"
#include "hardware_stm_timer3.h"
#include "hardware_stm_interruptcontroller.h"

int main(void)
{
    initGpioB0AsOutput();     // configure PB0 as output for LED 

    initTimer3ToInterrupt();   // set up Timer 3 + NVIC + interrupts

    while (1) {
        // main loop empty; LED is controlled entirely by ISR
    }
}




// int main (void)
// {
//     // Init pins
//     init_LED1();        // PB0 as output
//     init_PC6_input();   // PC6 as input with PUPDR = 01 (pull-up)

//     // Seed LED to current input level
//     if (read_PC6()) setGPIOB0(); else clearGPIOB0();

//     // Mirror PC6 -> LED every loop (no edge detection; always reflects current state)
//     while (1) {
//         uint32_t s = read_PC6();          // 1 at idle because of pull-up
//         if (s) setGPIOB0(); else clearGPIOB0();
//         delay(2000);                       // tiny delay so changes are visible
//     }
// }
// toggle_LED1(); //uncomment once you have filled in the function
// debugprintHelloWorld();

// int main (void)
// {
//     // Init pins
//     init_LED1();  // PB0 as output
//     while (1) {
//     toggle_LED1(); 
//     delay(2000);
//     }      
// }