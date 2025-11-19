// gummy_timer.h
// Simple non-blocking timer for the gummy LED state machine.

#ifndef GUMMY_TIMER_H
#define GUMMY_TIMER_H

#include <stdint.h>

/*
 * GummyTimer_Init
 * - Configure SysTick to generate a 1 ms time base.
 * - called once at startup.
 */
void GummyTimer_Init(void);

/*
 * GummyTimer_Start
 * - Start a one-shot timer that will expire after delay_ms milliseconds.
 * - Non-blocking: it just records the target time.
 */
void GummyTimer_Start(uint32_t delay_ms);

/*
 * GummyTimer_CheckExpired
 * - Checks if the one-shot timer has expired.
 * - Returns:
 *      1  if the timer has just expired (and auto-stops)
 *      0  otherwise.
 */
uint8_t GummyTimer_CheckExpired(void);

/*
 * GummyTimer_GetTimeMs
 * - Returns the current system time in milliseconds since startup.
 */
uint32_t GummyTimer_GetTimeMs(void);

#endif // GUMMY_TIMER_H