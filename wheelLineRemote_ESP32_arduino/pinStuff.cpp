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
#define BUTTON_START_PIN 0 // GPIO0 is 0, pin 8 on J2. Also USER_KEY on dev board, and must be high to boot.
#define BUTTON_FWD_PIN 33
#define BUTTON_REV_PIN 34

#define VBAT_ADC_PIN 1
#define ADC_CTRL_GPIO 37

#define ADC_POLL_ITVL_MS 1230

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t g_vextBits;

static int g_lastStartButton, g_lastFwdButton, g_lastRevButton;

static int32_t g_lastBatt_mV;
static uint32_t g_lastAdcPoll_ms;

/*******************************************************************************
 * Code
 ******************************************************************************/

int32_t pinStuff_getBatterymV(void)
{
  return g_lastBatt_mV;
}

static bool isLow(uint8_t pinNo)
{
  if (digitalRead(pinNo))
  {
    return false;
  }
  return true;
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

void pinStuff_initButtons(void)
{
  // filter, function 0, enable input, pullup
  pinMode(BUTTON_START_PIN, INPUT_PULLUP);
  pinMode(BUTTON_FWD_PIN, INPUT_PULLUP);
  pinMode(BUTTON_REV_PIN, INPUT_PULLUP);

  Serial.printf("\nConfig for START: 0x%x, FWD: 0x%x, REV: 0x%x\n\n", *((uint32_t *)GPIO_PIN_REG_0), *((uint32_t *)GPIO_PIN_REG_33), *((uint32_t *)GPIO_PIN_REG_34));
}

static bool checkToStart(void)
{
  if (isLow(BUTTON_START_PIN))
  {
    Serial.println("Moving to START state");
    globalInts_setMachineState(machState_startEngine);
    return true;
  }
  return false;
}

static void pollButtons(void)
{
  // Get button states
  machineState_t currentState = globalInts_getMachineState();
  switch (currentState)
  {
  case machState_justPoweredOn:
  case machState_killEngine:
    checkToStart();
    break;
  case machState_startEngine:
    if (!isLow(BUTTON_START_PIN))
    {
      Serial.println("Start released, releasing starter, enging HYD idle");
      globalInts_setMachineState(machState_runEngineHydIdle);
    }
    break;
  case machState_runEngineHydIdle:
    if (checkToStart())
    {
    }
    else if (isLow(BUTTON_FWD_PIN))
    {
      Serial.println("FWD pressed, engine HYD FWD");
      globalInts_setMachineState(machState_runEngineHydFwd);
    }
    else if (isLow(BUTTON_REV_PIN))
    {
      Serial.println("REV pressed, engine HYD REV");
      globalInts_setMachineState(machState_runEngineHydRev);
    }
    break;
  case machState_runEngineHydFwd:
    if (checkToStart())
    {
    }
    if (!isLow(BUTTON_FWD_PIN))
    {
      Serial.println("FWD released, engine HYD idle");
      globalInts_setMachineState(machState_runEngineHydIdle);
    }
    break;
  case machState_runEngineHydRev:
    if (checkToStart())
    {
    }
    if (!isLow(BUTTON_REV_PIN))
    {
      Serial.println("REV released, engine HYD idle");
      globalInts_setMachineState(machState_runEngineHydIdle);
    }
    break;
  default:
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

  // Init the buttons for the Remote
  pinStuff_initButtons();
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
  pollButtons();
  adcPoll();
}