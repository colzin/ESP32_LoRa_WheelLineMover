#include "pinStuff.h"

#include "Arduino.h"

#include "globalInts.h"

#include <stdint.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

// GPIOs for buttons
#define BUTTON_START_PIN GPIO_NUM_0 // GPIO0 is USER_KEY, TODO put in 3 buttons (start, FWD, Rev)
#define BUTTON_REV_PIN GPIO_NUM_33
#define BUTTON_FWD_PIN GPIO_NUM_34

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t g_vextBits;

static int g_lastStartButton, g_lastFwdButton, g_lastRevButton;
/*******************************************************************************
 * Code
 ******************************************************************************/

static bool isPressed(gpio_num_t pinNo)
{
  if (gpio_get_level(pinNo))
  { // High=false, pulled up
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
  gpio_config_t pinCfg;
  pinCfg.intr_type = GPIO_INTR_DISABLE;
  pinCfg.mode = GPIO_MODE_INPUT;
  pinCfg.pin_bit_mask = (1U << BUTTON_START_PIN) | (1U << BUTTON_FWD_PIN) | (1U << BUTTON_REV_PIN);
  pinCfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  pinCfg.pull_up_en = GPIO_PULLUP_ENABLE; // Even if HW pullup, pull all up
  esp_err_t ret = gpio_config(&pinCfg);
  if (ESP_OK != ret)
  {
    Serial.printf("gpio_config error 0x%x\n", ret);
  }
}

void pinStuff_pollButtons(void)
{
  // Get button states
  machineState_t currentState = globalInts_getMachineState();
  switch (currentState)
  {
  case machState_justPoweredOn:
  case machState_killEngine:
    if (isPressed(BUTTON_START_PIN))
    {
      Serial.println("Moving to START state");
      globalInts_setMachineState(machState_startEngine);
    }
    break;
  case machState_startEngine:
    if (!isPressed(BUTTON_START_PIN))
    {
      Serial.println("Start released, releasing starter, enging HYD idle");
      globalInts_setMachineState(machState_runEngineHydIdle);
    }
    break;
  case machState_runEngineHydIdle:
    if (isPressed(BUTTON_START_PIN))
    {
      Serial.println("Moving to START state");
      globalInts_setMachineState(machState_startEngine);
    }
    else if (isPressed(BUTTON_FWD_PIN))
    {
      Serial.println("FWD pressed, engine HYD FWD");
      globalInts_setMachineState(machState_runEngineHydFwd);
    }
    else if (isPressed(BUTTON_REV_PIN))
    {
      Serial.println("REV pressed, engine HYD REV");
      globalInts_setMachineState(machState_runEngineHydRev);
    }
    break;
  case machState_runEngineHydFwd:
    if (!isPressed(BUTTON_FWD_PIN))
    {
      Serial.println("FWD released, engine HYD idle");
      globalInts_setMachineState(machState_runEngineHydIdle);
    }
    break;
  case machState_runEngineHydRev:
    if (!isPressed(BUTTON_REV_PIN))
    {
      Serial.println("REV released, engine HYD idle");
      globalInts_setMachineState(machState_runEngineHydIdle);
    }
    break;
  default:
    break;
  }
}