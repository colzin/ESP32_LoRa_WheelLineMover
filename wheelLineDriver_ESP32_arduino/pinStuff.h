#ifndef PINSTUFF_H_
#define PINSTUFF_H_

#include "defines.h"

#include <stdint.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef enum
{
    led_off = 0,
    led_on,
    led_weak,
    ledMode_max
} ledMode_t;

#define VEXT_REQ_BIT_OLED 0x01
#define VEXT_REQ_BIT_RGB 0x02

/*******************************************************************************
 * Functions
 ******************************************************************************/

void pinStuff_setLED(ledMode_t desired);

// Returns true if it was already on
bool pinStuff_requestVEXT(uint8_t bitToSet);
void pinStuff_releaseVEXT(uint8_t bitToClear);

void pinStuff_init(void);
void pinStuff_poll(void);

#endif /* PINSTUFF_H_ */