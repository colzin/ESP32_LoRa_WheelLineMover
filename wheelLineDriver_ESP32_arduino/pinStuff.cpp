#include "pinStuff.h"

#include "Arduino.h"

#include "globalInts.h"

#include <stdint.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

// GPIOs for buttons
#define KILL_OPEN_RELAY 7         // GPIO7, J3 pin 18
#define START_RELAY 6             // GPIO6, J3 pin 17
#define EXTENDED_DEFAULT_RELAY 5  // GPIO5, JP3 pin 16. Make this out by default
#define RETRACTED_DEFAULT_RELAY 4 // GPIO4, JP3 pin 15. Make this one in by default

#define RELAY_ACTIVE 0 // TODO define
#define RELAY_INACTIVE 1

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t g_vextBits;

/*******************************************************************************
 * Code
 ******************************************************************************/

void pinStuff_setLED(ledMode_t desired)
{
  switch (desired)
  {
  case led_on:
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);
    break;
  case led_weak:
    pinMode(LED, INPUT_PULLUP);
    break;
  case led_off:
    // no-break fallthrough
  default:
    pinMode(LED, ANALOG);
    break;
  }
}
bool pinStuff_requestVEXT(uint8_t bitToSet)
{
  if (g_vextBits)
  {
    // Log that we want it to stay on.
    g_vextBits |= bitToSet;
    // Return true for already on
    return true;
  }
  g_vextBits |= bitToSet;
  pinMode(Vext, OUTPUT);   // Active low ENABLE for VEXT to be on
  digitalWrite(Vext, LOW); // SET POWER
  return false;
}

void pinStuff_releaseVEXT(uint8_t bitsToClear)
{
  // AND off the ones that are set here, they don't need it any more.
  g_vextBits = g_vextBits & ~bitsToClear;
  // If nobody needs it, turn it off
  if (!g_vextBits)
  {
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, HIGH);
  }
}

void pinStuff_init(void)
{
  // Default all to inactive state, poller will drive appropriately
  digitalWrite(KILL_OPEN_RELAY, RELAY_INACTIVE);
  pinMode(KILL_OPEN_RELAY, OUTPUT);
  digitalWrite(KILL_OPEN_RELAY, RELAY_INACTIVE);

  digitalWrite(START_RELAY, RELAY_INACTIVE);
  pinMode(START_RELAY, OUTPUT);
  digitalWrite(START_RELAY, RELAY_INACTIVE);

  digitalWrite(EXTENDED_DEFAULT_RELAY, RELAY_INACTIVE);
  pinMode(EXTENDED_DEFAULT_RELAY, OUTPUT);
  digitalWrite(EXTENDED_DEFAULT_RELAY, RELAY_INACTIVE);

  digitalWrite(RETRACTED_DEFAULT_RELAY, RELAY_INACTIVE);
  pinMode(RETRACTED_DEFAULT_RELAY, OUTPUT);
  digitalWrite(RETRACTED_DEFAULT_RELAY, RELAY_INACTIVE);
}

void pinStuff_poll(void)
{
  switch (globalInts_getMachineState())
  {
  case machState_justPoweredOn:
  case machState_killEngine:
    digitalWrite(KILL_OPEN_RELAY, RELAY_INACTIVE);
    digitalWrite(START_RELAY, RELAY_INACTIVE);
    digitalWrite(EXTENDED_DEFAULT_RELAY, RELAY_INACTIVE);
    digitalWrite(RETRACTED_DEFAULT_RELAY, RELAY_INACTIVE);
    break;
  case machState_startEngine:
    digitalWrite(KILL_OPEN_RELAY, RELAY_ACTIVE);
    digitalWrite(START_RELAY, RELAY_ACTIVE);
    digitalWrite(EXTENDED_DEFAULT_RELAY, RELAY_INACTIVE);
    digitalWrite(RETRACTED_DEFAULT_RELAY, RELAY_INACTIVE); // Hydraulic idle With both relaxed
    break;
  case machState_runEngineHydIdle:
    digitalWrite(KILL_OPEN_RELAY, RELAY_ACTIVE);
    digitalWrite(START_RELAY, RELAY_INACTIVE);
    digitalWrite(EXTENDED_DEFAULT_RELAY, RELAY_INACTIVE);
    digitalWrite(RETRACTED_DEFAULT_RELAY, RELAY_INACTIVE); // Hydraulic idle
    break;
  case machState_runEngineHydFwd:
    digitalWrite(KILL_OPEN_RELAY, RELAY_ACTIVE);
    digitalWrite(START_RELAY, RELAY_INACTIVE);
    digitalWrite(EXTENDED_DEFAULT_RELAY, RELAY_ACTIVE); // Suck it in to pull to the right
    digitalWrite(RETRACTED_DEFAULT_RELAY, RELAY_ACTIVE);

    break;
  case machState_runEngineHydRev:
    digitalWrite(KILL_OPEN_RELAY, RELAY_ACTIVE);
    digitalWrite(START_RELAY, RELAY_INACTIVE);
    digitalWrite(EXTENDED_DEFAULT_RELAY, RELAY_INACTIVE);
    digitalWrite(RETRACTED_DEFAULT_RELAY, RELAY_ACTIVE); // Extend it to push to the left
    break;
  }
}