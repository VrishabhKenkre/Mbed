// gummy_timer.c
// Implementation of simple non-blocking timer using SysTick.

#include "gummy_timer.h"
#include "stm32f4xx.h"   // for SystemCoreClock, SysTick, etc.
#include <stdint.h>

/* Global millisecond tick counter, incremented in SysTick_Handler */
static volatile uint32_t g_msTicks = 0;

/* One-shot timer state */
static uint8_t  g_timerActive = 0;
static uint32_t g_timerEndMs  = 0;

/*
 * SysTick interrupt handler
 * - Called automatically every 1 ms after we configure SysTick.
 */
void SysTick_Handler(void)
{
    g_msTicks++;
}

/*
 * GummyTimer_Init
 * - Configure SysTick for a 1 ms time base.
 * - Uses SystemCoreClock from CMSIS startup code.
 */
void GummyTimer_Init(void)
{
    /* Configure SysTick to interrupt every 1 ms */
    SysTick_Config(SystemCoreClock / 1000U);

    g_msTicks      = 0;
    g_timerActive  = 0;
    g_timerEndMs   = 0;
}

/*
 * GummyTimer_GetTimeMs
 * - Returns current time in ms since startup.
 */
uint32_t GummyTimer_GetTimeMs(void)
{
    return g_msTicks;
}

/*
 * GummyTimer_Start
 * - Start one-shot timer of duration delay_ms.
 */
void GummyTimer_Start(uint32_t delay_ms)
{
    g_timerEndMs  = GummyTimer_GetTimeMs() + delay_ms;
    g_timerActive = 1;
}

/*
 * GummyTimer_CheckExpired
 * - Returns 1 once when timer expires, then clears active flag.
 * - Returns 0 if not yet expired or no timer active.
 */
uint8_t GummyTimer_CheckExpired(void)
{
    if (!g_timerActive) {
        return 0;
    }

    /* signed difference handles wrap-around correctly */
    int32_t diff = (int32_t)(GummyTimer_GetTimeMs() - g_timerEndMs);
    if (diff >= 0) {
        g_timerActive = 0;  // auto-stop
        return 1;           // just expired
    }

    return 0;               // still running
}