// gummy_sm.h
// State machine for scanning RGB LEDs and detecting gummy color.

#ifndef GUMMY_SM_H
#define GUMMY_SM_H

#include <stdint.h>

/* Detected gummy color codes */
typedef enum {
    GUMMY_COLOR_UNKNOWN = 0,
    GUMMY_COLOR_RED,
    GUMMY_COLOR_GREEN,
    GUMMY_COLOR_ORANGE,
    GUMMY_COLOR_WHITE
} gummy_color_t;

/*
 * GummySM_Init
 * - Initializes internal state of the gummy state machine.
 * - Call once at startup (after GPIO + LED + timer init).
 */
void GummySM_Init(void);

/*
 * GummySM_Run
 * - Non-blocking state machine step.
 * - Call this repeatedly in your main while(1) loop.
 * - It will:
 *      * drive LEDs one-by-one
 *      * start timers
 *      * sample the sensor when timers expire
 *      * classify the gummy color at the end of each scan cycle.
 */
void GummySM_Run(void);

/*
 * GummySM_HasNewResult
 * - Returns 1 if a NEW gummy classification is available
 *   since the last time you called GummySM_GetLastColor().
 * - Returns 0 otherwise.
 */
uint8_t GummySM_HasNewResult(void);

/*
 * GummySM_GetLastColor
 * - Returns the most recently classified gummy color.
 * - Calling this also clears the "new result" flag.
 */
gummy_color_t GummySM_GetLastColor(void);

/*
 * (Optional helper)
 * GummySM_ColorToString
 * - Converts a gummy_color_t to a const char* string.
 */
const char* GummySM_ColorToString(gummy_color_t color);

#endif // GUMMY_SM_H
