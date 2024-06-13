#include "pinStuff.h"

#include "Arduino.h"

#include "globalInts.h"

#include <stdint.h>

#include "esp32-hal-adc.h"

// NOTE: only use functions in esp32-hal-gpio.h
#include "esp32-hal-gpio.h"
// NOTE: Use pin defs from pins_arduino.h
#include "pins_arduino.h"

#include "utils.h"

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


#define VBAT_ADC_PIN 1
#define ADC_CTRL_GPIO 37

#define ADC_POLL_ITVL_MS 1230

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t g_vextBits;

static int32_t g_lastBatt_mV;
static uint32_t g_lastAdcPoll_ms;

/*******************************************************************************
 * Code
 ******************************************************************************/

int32_t pinStuff_getBatterymV(void)
{
  return g_lastBatt_mV;
}

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

void pollOutputs(void)
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
    digitalWrite(RETRACTED_DEFAULT_RELAY, RELAY_INACTIVE);

    break;
  case machState_runEngineHydRev:
    digitalWrite(KILL_OPEN_RELAY, RELAY_ACTIVE);
    digitalWrite(START_RELAY, RELAY_INACTIVE);
    digitalWrite(EXTENDED_DEFAULT_RELAY, RELAY_INACTIVE);
    digitalWrite(RETRACTED_DEFAULT_RELAY, RELAY_ACTIVE); // Extend it to push to the left
    break;
  }
}

void pinStuff_init(void)
{
  pinMode(ADC_CTRL_GPIO, OUTPUT_OPEN_DRAIN | INPUT);
  digitalWrite(ADC_CTRL_GPIO, 1);
  // Set up analog pin to read battery voltage:
  // analogSetClockDiv(255); // 1338mS
  //  analogSetCycles(8);                    // Set number of cycles per sample, default is 8 and provides an optimal result, range is 1 - 255
  //  analogSetSamples(1);                   // Set number of samples in the range, default is 1, it has an effect on sensitivity has been multiplied
  analogSetClockDiv(1); // Set the divider for the whole ADC clock, default is 1, range is 1 - 255
  /* Pin on Heltec esp32 lora v3 is on a 390k / 100k divider, so 4.9 divider. Multiply our reading to get actual mV
   * ESP32 S3 has max of 950mV, so VREF is 950mV.
   * 950mV*4.9=4655mV, which is too much for a LiPo, so enough for us
   */
  analogSetAttenuation(ADC_0db);
  analogSetPinAttenuation(VBAT_ADC_PIN, ADC_0db);

  if (!adcAttachPin(VBAT_ADC_PIN))
  {
    Serial.printf("adcAttachPin failed\n");
  }

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

static void adcPoll(void)
{
  if (utils_elapsedU32Ticks(g_lastAdcPoll_ms, millis()) >= ADC_POLL_ITVL_MS)
  {
    if (digitalRead(ADC_CTRL_GPIO))
    { // If high, drive low. Then read on next poll
      digitalWrite(ADC_CTRL_GPIO, 0);
      // Serial.println("Muxed ADC to read on next poll");
    }
    else
    { // It is low, it should have stabilized since last poll, so poll ADC, then de-mux
      uint32_t counts = analogRead(VBAT_ADC_PIN);
      /* Now we have counts (0-4095), need to get to voltage. [mV] = VREF/4095*counts
       * ADC has VREF of 950mV ( see https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/adc.html)
       * mV = counts * Vref (950) / max count (4095);
       * Pin on Heltec esp32 lora v3 is on a 390k / 100k divider, so 4.9 divider. Multiply our reading to get actual mV
       * so for us, mV = counts * 950[mV] * 4.9 / 4095 [counts/mV]
       */
      counts *= 950;
      counts *= 49;
      counts /= 10;
      counts /= 4095;
      // CBM measured 4186 actual, MCU reported 4260
      counts *= 4186;
      counts /= 4260;
      g_lastBatt_mV = counts;
      digitalWrite(ADC_CTRL_GPIO, 1); // Drive high to save power (de-selects ADC mux)
      g_lastAdcPoll_ms = millis();
      // Serial.printf("Read batt %d mV at %d. Pin now %s\n", g_lastBatt_mV, g_lastAdcPoll_ms, isLow(ADC_CTRL_GPIO) ? "low" : "high");
    }
  }
}

void pinStuff_poll(void)
{
  pollOutputs();
  adcPoll();
}