#include "hardware_stm_timer3.h"
#include "stm32f4xx_rcc_mort.h"
#include "stm32f4xx_mort2.h"
#include "stm32f446xx.h"   // gives you TIM3 peripheral struct


// /* --- TIM3 register addresses (RM0390 TIM2-5 map) --- */
// #define TIM3_BASE        (TIM3_BASE_MORT)
// #define TIM3_CR1         (TIM3_BASE + 0x00)
// #define TIM3_CR2         (TIM3_BASE + 0x04)
// #define TIM3_SMCR        (TIM3_BASE + 0x08)
// #define TIM3_DIER        (TIM3_BASE + 0x0C)
// #define TIM3_SR          (TIM3_BASE + 0x10)
// #define TIM3_EGR         (TIM3_BASE + 0x14)
// #define TIM3_CCMR1       (TIM3_BASE + 0x18)
// #define TIM3_CCMR2       (TIM3_BASE + 0x1C)
// #define TIM3_CCER        (TIM3_BASE + 0x20)
// #define TIM3_CNT         (TIM3_BASE + 0x24)
// #define TIM3_PSC         (TIM3_BASE + 0x28)
// #define TIM3_ARR         (TIM3_BASE + 0x2C)
// #define TIM3_CCR1        (TIM3_BASE + 0x34)
// #define TIM3_CCR2        (TIM3_BASE + 0x38)
// #define TIM3_CCR3        (TIM3_BASE + 0x3C)
// #define TIM3_CCR4        (TIM3_BASE + 0x40)

// /* Bits I need */
// #define CR1_CEN          (1u << 0)   /* counter enable */
// #define EGR_UG           (1u << 0)   /* update generation */
// #define CCER_CC3E        (1u << 8)   /* ch3 output enable */

// /* CCMR2 (OC3M at bits [6:4], CC3S at [1:0]) */
// #define CCMR2_CC3S_MASK  (0x3u << 0)       /* 00 = output */
// #define CCMR2_OC3M_MASK  (0x7u << 4)
// #define CCMR2_OC3M_TOGGLE (0x3u << 4)      /* 011: toggle on match */

// /* Helper: write 16/32-bit registers as 32-bit accesses */
// #define REG32(addr) (*(volatile uint32_t*)(addr))


void timer3_ch3_init_oc_toggle_1Hz(void)
{
    // 1) Enable APB1 clock for TIM3
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* 2) Compute time base for ~1 Hz toggle events.
       APB1 timer clock used in class = 90 MHz  (mbed config)  -> slide says APB1 timers = 90 MHz. */

    /* Let PSC = 8999 -> f_cnt = 90MHz/(8999+1) = 10 kHz
       Let ARR = 9999 -> event (CNT==CCR3) rate = f_cnt/(ARR+1) = 10k/10k = 1 Hz
       In OC Toggle mode, the pin toggles on each match -> LED flips once per second (~1 Hz). */
    TIM3->PSC  = 8999U;
    TIM3->ARR  = 9999U;

    /* 3) Position the compare for channel 3 somewhere in the period - I am taking half.. (any 0..ARR is fine). */
    TIM3->CCR3 = 5000U;

    // 4) Configure channel 3 as Output Compare, "toggle on match"
    // CC3S[1:0] = 00 (output mode)
    // OC3M[6:4] = 011 (toggle on match)
    TIM3->CCMR2 &= ~((3U << 0) | (7U << 4));   // clear CC3S and OC3M
    TIM3->CCMR2 |=  ((0U << 0) | (3U << 4));   // CC3S=00, OC3M=0b011

    // 5) Enable channel 3 output on the pin
    TIM3->CCER |= (1U << 8);                   // CC3E=1

    /* 6) Force an update so PSC/ARR/CCR latch, then enable the counter */
    TIM3->EGR  = (1U << 0);                    // UG=1
    TIM3->CR1 |= (1U << 0);                    // CEN=1 /* CEN=1 starts the counter */
}