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
// GPIO0 is 0, pin 8 on J2. Also USER_KEY on dev board, and must be high to boot. Don't use this one, it's glitchy
#define BUTTON_START_PIN 26 // p26 is labeled 15
#define BUTTON_FWD_PIN 33
#define BUTTON_REV_PIN 34

#define VBAT_ADC_PIN 1
#define ADC_CTRL_GPIO 37

#define ADC_POLL_ITVL_MS 1230

#define BUTTONS_DEBOUNCE_PRESS_MS 68 // 50 was very short, 500 too long.
#define BUTTONS_DEBOUNCE_RELEASE_MS BUTTONS_DEBOUNCE_PRESS_MS

#define BUTTON_HOLD_MIN_MS 900 // todo TUNE

typedef struct
{
  uint8_t pinNumber;
  bool pressedThisPoll, pressedLastPoll;
  uint32_t inState_ms;
} buttonInfo_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t g_vextBits;

static buttonInfo_t g_startButtonInfo, g_fwdButtonInfo, g_revButtonInfo;
static uint32_t g_lastButtonPoll_ms;

static bool m_fwdPressed, m_revPressed;
static uint32_t m_fwdPressStart_ms, m_revPressStart_ms;

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

void pinStuff_initButtons(void)
{
  // filter, function 0, enable input, pullup
  pinMode(BUTTON_START_PIN, INPUT_PULLUP);
  pinMode(BUTTON_FWD_PIN, INPUT_PULLUP);
  pinMode(BUTTON_REV_PIN, INPUT_PULLUP);
  // Serial.printf("\nConfig for START: 0x%x, FWD: 0x%x, REV: 0x%x\n\n", *((uint32_t *)GPIO_PIN_REG_0), *((uint32_t *)GPIO_PIN_REG_33), *((uint32_t *)GPIO_PIN_REG_34));
  g_startButtonInfo.pinNumber = BUTTON_START_PIN;
  g_startButtonInfo.inState_ms = 0;
  g_startButtonInfo.pressedLastPoll = false;
  g_startButtonInfo.pressedThisPoll = false;
  g_fwdButtonInfo.pinNumber = BUTTON_FWD_PIN;
  g_fwdButtonInfo.inState_ms = 0;
  g_fwdButtonInfo.pressedLastPoll = false;
  g_fwdButtonInfo.pressedThisPoll = false;
  g_revButtonInfo.pinNumber = BUTTON_REV_PIN;
  g_revButtonInfo.inState_ms = 0;
  g_revButtonInfo.pressedLastPoll = false;
  g_revButtonInfo.pressedThisPoll = false;

  m_fwdPressed = false;
  m_revPressed = false;
}

static void updateButtonState(buttonInfo_t *p, uint32_t ms_now, uint32_t lastPoll_ms)
{
  // Move current to last
  p->pressedLastPoll = p->pressedThisPoll;
  // update current
  if (digitalRead(p->pinNumber))
  { // High, so not pressed (active low)
    p->pressedThisPoll = false;
  }
  else
  { // Active low
    p->pressedThisPoll = true;
  }
  if (p->pressedLastPoll == p->pressedThisPoll)
  {
    p->inState_ms += utils_elapsedU32Ticks(lastPoll_ms, ms_now);
  }
  else
  { // Zero the counter on state change
    Serial.printf("Pin %d change to %d at %d\n", p->pinNumber, p->pressedThisPoll, millis());
    p->inState_ms = 0;
  }
}

static void checkToStart(void)
{
  if (g_startButtonInfo.pressedThisPoll && g_startButtonInfo.inState_ms >= BUTTONS_DEBOUNCE_PRESS_MS)
  {
    Serial.printf("Moving from %s to START state\n", globalInts_getMachStateString(globalInts_getMachineState()));
    globalInts_setMachineState(machState_startEngine);
  }
}

static void checkStartRelease(void)
{
  if (!g_startButtonInfo.pressedThisPoll && g_startButtonInfo.inState_ms >= BUTTONS_DEBOUNCE_PRESS_MS)
  {
    Serial.printf("Moving from %s to RunIdle state\n", globalInts_getMachStateString(globalInts_getMachineState()));
    globalInts_setMachineState(machState_runEngineHydIdle);
  }
}

static void checkFwdRelease(void)
{
  if (m_fwdPressed && !g_fwdButtonInfo.pressedThisPoll && (g_fwdButtonInfo.inState_ms >= BUTTONS_DEBOUNCE_RELEASE_MS))
  {                       // If released for debounce period, check time to see if we should increment count or release HYD now
    m_fwdPressed = false; // Mark released
    uint32_t pressed_ms = utils_elapsedU32Ticks(m_fwdPressStart_ms, millis());
    if (pressed_ms < BUTTON_HOLD_MIN_MS)
    { // On release of FWD button, increment the counter, we need to move in the negative direction
      Serial.printf("FWD released after %d seconds, need %d to hold, increment rotations\n", pressed_ms, BUTTON_HOLD_MIN_MS);
      globalInts_setNumRotations(globalInts_getNumRotations() + 1);
    }
    else
    {
      Serial.printf("FWD released after %d, to HYD idle at %d\n", pressed_ms, millis());
      globalInts_setNumRotations(0);
      globalInts_setMachineState(machState_runEngineHydIdle);
    }
  }
}

static void checkRevRelease(void)
{
  if (m_revPressed && !g_revButtonInfo.pressedThisPoll && (g_revButtonInfo.inState_ms >= BUTTONS_DEBOUNCE_RELEASE_MS))
  {                       // If released for debounce period, check time to see if we should increment count or release HYD now
    m_revPressed = false; // Mark released
    uint32_t pressed_ms = utils_elapsedU32Ticks(m_revPressStart_ms, millis());
    if (pressed_ms < BUTTON_HOLD_MIN_MS)
    { // On release of REV button, decrement the counter, we need to move in the positive direction
      Serial.printf("REV released after %d seconds, need %d to hold, decrement rotations\n", pressed_ms, BUTTON_HOLD_MIN_MS);
      globalInts_setNumRotations(globalInts_getNumRotations() - 1);
    }
    else
    {
      Serial.printf("REV released after %d, to HYD idle at %d\n", pressed_ms, millis());
      globalInts_setNumRotations(0);
      globalInts_setMachineState(machState_runEngineHydIdle);
    }
  }
}

static void checkFwdPress(void)
{
  // In this state, check for a debounced press of the FWD button
  if (!m_fwdPressed && g_fwdButtonInfo.pressedThisPoll && (g_fwdButtonInfo.inState_ms >= BUTTONS_DEBOUNCE_PRESS_MS))
  {
    Serial.printf("FWD pressed for %d, from state %s to FWD state at %d\n", g_fwdButtonInfo.inState_ms, globalInts_getMachStateString(globalInts_getMachineState()), millis());
    m_fwdPressed = true;
    m_fwdPressStart_ms = millis();
    globalInts_setMachineState(machState_runEngineHydFwd);
  }
}

static void checkRevPress(void)
{
  if (!m_revPressed && g_revButtonInfo.pressedThisPoll && (g_revButtonInfo.inState_ms >= BUTTONS_DEBOUNCE_PRESS_MS))
  {
    Serial.printf("REV pressed for %d, from state %s to REV state at %d\n", g_revButtonInfo.inState_ms, globalInts_getMachStateString(globalInts_getMachineState()), millis());
    m_revPressed = true;
    m_revPressStart_ms = millis();
    globalInts_setMachineState(machState_runEngineHydRev);
  }
}

static void pollButtons(void)
{
  // Update button states
  uint32_t ms_now = millis();
  updateButtonState(&g_startButtonInfo, ms_now, g_lastButtonPoll_ms);
  updateButtonState(&g_fwdButtonInfo, ms_now, g_lastButtonPoll_ms);
  updateButtonState(&g_revButtonInfo, ms_now, g_lastButtonPoll_ms);

  // Change state based on button pressed
  machineState_t currentState = globalInts_getMachineState();
  switch (currentState)
  {
  case machState_justPoweredOn:
  case machState_killEngine:
    // In these states, check for a debounced press of the Start button
    checkToStart();
    break;
  case machState_startEngine:
    // Check for user release of Start button
    checkStartRelease();
    break;
  case machState_runEngineHydIdle:
    // In this state, check for a debounced press of the Start button
    checkToStart();
    checkFwdPress();
    checkRevPress();
    break;
  case machState_runEngineHydFwd:
    checkFwdRelease();
    checkToStart();
    checkFwdPress(); // check for another press
    checkRevPress();
    break;
  case machState_runEngineHydRev:
    checkRevRelease();
    checkToStart();
    checkFwdPress();
    checkRevPress(); // check for another press
    break;
  default:
    Serial.printf("UNKNOWN state %d, going to justPoweredOn\n", currentState);
    globalInts_setMachineState(machState_justPoweredOn);
    break;
  }

  g_lastButtonPoll_ms = ms_now;
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