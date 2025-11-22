// gummy_color.c
// Implementation of gummy color classification helpers.

#include "gummy_color.h"

gummy_color_t classifyColor(uint32_t r, uint32_t g, uint32_t b)
{
    /*
     * IMPORTANT:
     *   1 = light detected, 0 = no light.
     *   This mapping based on
     *   the actual bits we see for each gummy.
     *    Based on what colors let what colors passs through them.
     *
     * 
     *   WHITE:  r=1, g=1, b=1
     *   RED:    r=1, g=0, b=0
     *   GREEN:  r=0, g=1, b=0
     *   ORANGE: r=1, g=1, b=0
     */
    if (r && g && b)      return GUMMY_COLOR_WHITE;
    if (r && !g && !b)    return GUMMY_COLOR_RED;
    if (!r && g && !b)    return GUMMY_COLOR_GREEN;
    if (r && g && !b)     return GUMMY_COLOR_ORANGE;

    return GUMMY_COLOR_UNKNOWN;
}

const char* colorToString(gummy_color_t c)
{
    switch (c) {
        case GUMMY_COLOR_RED:     return "RED";
        case GUMMY_COLOR_GREEN:   return "GREEN";
        case GUMMY_COLOR_ORANGE:  return "ORANGE";
        case GUMMY_COLOR_WHITE:   return "WHITE";
        default:                  return "UNKNOWN";
    }
}
