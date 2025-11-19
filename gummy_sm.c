// gummy_sm.c
// Implementation of RGB LED scanning + gummy color detection state machine.

#include "gummy_sm.h"
#include "hardware_stm_gpio.h"
#include "led_gummy.h"
#include "gummy_timer.h"

extern void debugprint(uint16_t number);
extern void debugprintHelloWorld(void);

/* Internal states for the scanning state machine */
typedef enum {
    GUMMY_STATE_IDLE = 0,
    GUMMY_STATE_EMIT_RED,
    GUMMY_STATE_EMIT_GREEN,
    GUMMY_STATE_EMIT_BLUE,
    GUMMY_STATE_DECIDE
} gummy_state_t;

/* Current state */
static gummy_state_t g_state;

/* Sampled sensor bits for each LED */
static uint8_t g_redBit   = 0;
static uint8_t g_greenBit = 0;
static uint8_t g_blueBit  = 0;

/* Last classified color + "new result" flag */
static gummy_color_t g_lastColor        = GUMMY_COLOR_UNKNOWN;
static uint8_t       g_hasNewResultFlag = 0;

/* Forward: classification logic based on (R,G,B) bits */
static gummy_color_t classifyColor(uint8_t r, uint8_t g, uint8_t b);

/*
 * GummySM_Init
 * - Initializes internal state.
 */
void GummySM_Init(void)
{
    g_state           = GUMMY_STATE_IDLE;
    g_redBit          = 0;
    g_greenBit        = 0;
    g_blueBit         = 0;
    g_lastColor       = GUMMY_COLOR_UNKNOWN;
    g_hasNewResultFlag = 0;

    allLEDs_Off();
    debugprintHelloWorld();
}

/*
 * GummySM_Run
 * - Non-blocking: each call performs at most a small chunk of work.
 * - Uses GummyTimer_Start and GummyTimer_CheckExpired to handle delays.
 */
void GummySM_Run(void)
{
    switch (g_state)
    {
        case GUMMY_STATE_IDLE:
            /* Start a new scan cycle:
             * - turn on RED LED
             * - start small delay (e.g., 5 ms)
             */
            allLEDs_Off();
            redLED_On();
            GummyTimer_Start(5);           // wait 5 ms for sensor to settle
            g_state = GUMMY_STATE_EMIT_RED;
            break;

        case GUMMY_STATE_EMIT_RED:
            /* Wait for timer to expire while RED LED is on.
             * When expired:
             *   - sample sensor bit
             *   - turn off RED
             *   - turn on GREEN
             *   - start next timer
             */
            if (GummyTimer_CheckExpired())
            {
                g_redBit = checkGPIOC6();
                redLED_Off();

                greenLED_On();
                GummyTimer_Start(5);
                g_state = GUMMY_STATE_EMIT_GREEN;
            }
            break;

        case GUMMY_STATE_EMIT_GREEN:
            if (GummyTimer_CheckExpired())
            {
                g_greenBit = checkGPIOC6();
                greenLED_Off();

                blueLED_On();
                GummyTimer_Start(5);
                g_state = GUMMY_STATE_EMIT_BLUE;
            }
            break;

        case GUMMY_STATE_EMIT_BLUE:
            if (GummyTimer_CheckExpired())
            {
                g_blueBit = checkGPIOC6();
                blueLED_Off();

                g_state = GUMMY_STATE_DECIDE;
            }
            break;

        case GUMMY_STATE_DECIDE:
            {
                /* Decide gummy color based on (R,G,B) bits. */
                gummy_color_t color = classifyColor(g_redBit, g_greenBit, g_blueBit);

                g_lastColor        = color;
                g_hasNewResultFlag = 1;   // signal new result available

                /* --- DEBUG OUTPUT --- */

                // Pack bits into a 3-bit number: (r g b) -> [2:0]
                uint16_t rgb_code = ( (g_redBit   & 1u) << 2 ) |
                                    ( (g_greenBit & 1u) << 1 ) |
                                    ( (g_blueBit  & 1u) << 0 );

                // Print raw RGB bit pattern
                debugprint(rgb_code);      // e.g. 0b111 = 7, 0b100 = 4, etc.

                // Print detected color enum as a number
                //   0 = UNKNOWN, 1 = RED, 2 = GREEN, 3 = ORANGE, 4 = WHITE
                debugprint((uint16_t)color);

                g_state = GUMMY_STATE_IDLE;
                break;
            }
    }
}

/*
 * GummySM_HasNewResult
 * - Returns 1 if new classification is available.
 */
uint8_t GummySM_HasNewResult(void)
{
    return g_hasNewResultFlag;
}

/*
 * GummySM_GetLastColor
 * - Returns last classified color and clears "new result" flag.
 */
gummy_color_t GummySM_GetLastColor(void)
{
    g_hasNewResultFlag = 0;
    return g_lastColor;
}

/*
 * GummySM_ColorToString
 * - Converts gummy_color_t to a string for printing.
 */
const char* GummySM_ColorToString(gummy_color_t color)
{
    switch (color)
    {
        case GUMMY_COLOR_RED:     return "RED";
        case GUMMY_COLOR_GREEN:   return "GREEN";
        case GUMMY_COLOR_ORANGE:  return "ORANGE";
        case GUMMY_COLOR_WHITE:   return "WHITE";
        case GUMMY_COLOR_UNKNOWN:
        default:                  return "UNKNOWN";
    }
}

/*
 * classifyColor
 * - Mapping from (R,G,B) sensor bits to gummy color enum.
 *
 * NOTE:
 *   1 = light detected, 0 = no light.
 *   The mapping below is an EXAMPLE. You MUST adjust this
 *   after you measure real readings for your setup.
 *
 * Suggested test:
 *   - Put RED gummy, note (R,G,B)
 *   - Put GREEN gummy, note (R,G,B)
 *   - Put ORANGE gummy, note (R,G,B)
 *   - Put WHITE gummy, note (R,G,B)
 *   Then update the if/else conditions accordingly.
 */
static gummy_color_t classifyColor(uint8_t r, uint8_t g, uint8_t b)
{
    /* Example assumptions (you will likely need to tweak these):
     *
     *   WHITE:  r=1, g=1, b=1  (reflects all)
     *   RED:    r=1, g=0, b=0
     *   GREEN:  r=0, g=1, b=0
     *   ORANGE: r=1, g=1, b=0
     */

    if (r && g && b)
    {
        return GUMMY_COLOR_WHITE;
    }
    else if (r && !g && !b)
    {
        return GUMMY_COLOR_RED;
    }
    else if (!r && g && !b)
    {
        return GUMMY_COLOR_GREEN;
    }
    else if (r && g && !b)
    {
        return GUMMY_COLOR_ORANGE;
    }

    /* Anything else does not match expected patterns */
    return GUMMY_COLOR_UNKNOWN;
}
