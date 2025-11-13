#include "main.h"
#include "debug_mort.h"
#include "nucleo_led.h"
#include "hardware_stm_gpio.h"
#include "hardware_stm_timer3.h"

static void delay(volatile uint32_t ticks)
{
    while (ticks--) { __asm__("nop"); }
}


int main (void)
{
    /* Route PB0 to TIM3_CH3 (AF2) of GPIO output */
    initGpioB0AsAF2_TIM3CH3();   // PB0: MODER=AF, AFRL=AF2  (TIM3_CH3)
    
    /* Start TIM3 CH3 in Output-Compare Toggle mode at ~1 Hz */
    timer3_ch3_init_oc_toggle_1Hz();   // PSC/ARR/CCR3 + CCMR2/CCER/CR1

    while (1) {
        /* nothing to do; hardware is toggling PB0 via TIM3 */
        __asm__("nop");
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
//toggle_LED1(); //uncomment once you have filled in the function
//debugprintHelloWorld();

// int main (void)
// {
//     // Init pins
//     init_LED1();  // PB0 as output
//     while (1) {
//     toggle_LED1(); 
//     delay(2000);
//     }      
// }