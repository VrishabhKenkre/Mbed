// gummy_sm.h
// State machine for scanning RGB LEDs and detecting gummy color.

#ifndef GUMMY_SM_H
#define GUMMY_SM_H

#include <stdint.h>
#include "gummy_color.h"   // <-- use the shared gummy_color_t

#ifdef __cplusplus
extern "C" {
#endif

void GummySM_Init(void);
void GummySM_Run(void);

/* SM exposes latest detected color via this API */
uint8_t       GummySM_HasNewResult(void);
gummy_color_t GummySM_GetLastColor(void);

#ifdef __cplusplus
}
#endif

#endif // GUMMY_SM_H