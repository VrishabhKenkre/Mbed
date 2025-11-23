#include "gummy_sm.h"
#include "hardware_stm_gpio.h"
#include "led_gummy.h"
#include "gummy_timer.h"
#include "gummy_color.h"      
#include <stdint.h>         

extern void debugprint(uint16_t number);
extern void debugprintHelloWorld(void);
extern uint32_t checkGPIOC6(void);   

/* Internal states for the scanning state machine */
typedef enum {
    GUMMY_STATE_IDLE = 0,
    GUMMY_STATE_EMIT_RED,
    GUMMY_STATE_EMIT_GREEN,
    GUMMY_STATE_EMIT_BLUE,
    GUMMY_STATE_DECIDE,
    GUMMY_STATE_WAIT,
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

/*
 * GummySM_Init
 * - Initializes internal state.
 */
void GummySM_Init(void)
{
    g_state            = GUMMY_STATE_IDLE;
    g_redBit           = 0;
    g_greenBit         = 0;
    g_blueBit          = 0;
    g_lastColor        = GUMMY_COLOR_UNKNOWN;
    g_hasNewResultFlag = 0;

    allLEDs_Off();
    debugprintHelloWorld();   // "I'm aliiiiiivveeee!!!"
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
            // Start a new scan cycle:
            allLEDs_Off();
            redLED_On();
            printf("STATE: RED LED ON\n");
            GummyTimer_Start(3000);           // 3 seconds for RED
            g_state = GUMMY_STATE_EMIT_RED;
            break;

        case GUMMY_STATE_EMIT_RED:
            if (GummyTimer_CheckExpired())
            {
                g_redBit = (uint8_t)checkGPIOC6();

                printf("  RED: PC6 = %u\n", g_redBit);

                redLED_Off();
                greenLED_On();
                printf("STATE: GREEN LED ON\n");

                GummyTimer_Start(3000);       // 3 seconds for GREEN
                g_state = GUMMY_STATE_EMIT_GREEN;
            }
            break;

        case GUMMY_STATE_EMIT_GREEN:
            if (GummyTimer_CheckExpired())
            {
                g_greenBit = (uint8_t)checkGPIOC6();

                printf("  GREEN: PC6 = %u\n", g_greenBit);

                greenLED_Off();
                blueLED_On();
                printf("STATE: BLUE LED ON\n");

                GummyTimer_Start(3000);       // 3 seconds for BLUE
                g_state = GUMMY_STATE_EMIT_BLUE;
            }
            break;

        case GUMMY_STATE_EMIT_BLUE:
            if (GummyTimer_CheckExpired())
            {
                g_blueBit = (uint8_t)checkGPIOC6();

                printf("  BLUE: PC6 = %u\n", g_blueBit);

                blueLED_Off();
                g_state = GUMMY_STATE_DECIDE;
            }
            break;

        case GUMMY_STATE_DECIDE:
        {
            // Decide color from bits
            gummy_color_t color = classifyColor(g_redBit, g_greenBit, g_blueBit);

            g_lastColor        = color;
            g_hasNewResultFlag = 1;


            // Final RESULT line
            printf("RESULT: gummy = %s (R=%u,G=%u,B=%u)\n",
                   colorToString(color),
                   g_redBit, g_greenBit, g_blueBit);

            // Short pause before next scan
            GummyTimer_Start(1000);           // 1 second pause
            g_state = GUMMY_STATE_WAIT;
            break;
        }

        case GUMMY_STATE_WAIT:
            if (GummyTimer_CheckExpired())
            {
                g_state = GUMMY_STATE_IDLE;
            }
            break;

        default:
            g_state = GUMMY_STATE_IDLE;
            break;
    }
}


uint8_t GummySM_HasNewResult(void)
{
    return g_hasNewResultFlag;
}

gummy_color_t GummySM_GetLastColor(void)
{
    g_hasNewResultFlag = 0;
    return g_lastColor;
}






