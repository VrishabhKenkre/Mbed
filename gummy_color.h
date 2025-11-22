// gummy_color.h
// Common gummy color enum + helper functions.

#ifndef GUMMY_COLOR_H
#define GUMMY_COLOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {               
#endif

typedef enum {
    GUMMY_COLOR_UNKNOWN = 0,
    GUMMY_COLOR_RED,
    GUMMY_COLOR_GREEN,
    GUMMY_COLOR_ORANGE,
    GUMMY_COLOR_WHITE
} gummy_color_t;

/*
 * classifyColor
 * - Given sensor bits r,g,b (0 or 1),
 *   return which gummy color we think it is.
 */
gummy_color_t classifyColor(uint32_t r, uint32_t g, uint32_t b);

/*
 * colorToString
 * - Convert a gummy_color_t into a human-readable string.
 */
const char* colorToString(gummy_color_t c);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // GUMMY_COLOR_H
