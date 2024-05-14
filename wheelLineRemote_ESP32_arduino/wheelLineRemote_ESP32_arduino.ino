
#include "Arduino.h"

#include "defines.h" // Controls compile and runtime

#include "ESP32_Mcu.h"
#include "esp_system.h"
#include "globalInts.h" // For machine state
#include "lis2dh.h"     // for accelerometer on REMOTE ONLY
#include "loraStuff.h"
#include "oledStuff.h"
#include "packetParser.h"
#include "pinStuff.h"
#include "utils.h"

#include "serialInput.h"

#include "task.h" // for vTaskList

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static uint64_t g_chipID;

#if USE_TASK_WATCHDOG
#include <esp_task_wdt.h>
#define TASK_WDT_TIMEOUT_SEC 5 // Wait this long before WDT panic
#endif                         // #if USE_TASK_WATCHDOG

static uint32_t g_lastMachStateSend_ms;

/*******************************************************************************
 * Code
 ******************************************************************************/

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("WheelLineRemote start");
  Mcu.begin();
  g_chipID = ESP.getEfuseMac();
  Serial.printf("ESP32ChipID 0x%08x%08x\n", (uint32_t)(g_chipID >> 32), (uint32_t)g_chipID);
  Serial.printf("ESP32 Chip model = %s Rev %d, %d cores\n", ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores());

  // Serial.printf("\nROM_core0 reason %d, ROM_core1 reason %d, rtc_0 reason %d, RTC_1 reason %d\n",
  //                 esp_rom_get_reset_reason(0), esp_rom_get_reset_reason(1), rtc_get_reset_reason(0), rtc_get_reset_reason(1));

  pinStuff_init();
  pinStuff_setLED(led_weak);

  lis2dh_init();
  oledStuff_displayInit();

  // oledStuff_printESPInfo(ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(), ESP.getFlashChipSize(), g_chipID);

  // Init packet parser, then radio

#if LORA
  packetParser_init(g_chipID);
  loraStuff_initRadio();

#endif // #if LORA

#if USE_TASK_WATCHDOG
  Serial.println("Configuring WDT...");
  esp_task_wdt_init(TASK_WDT_TIMEOUT_SEC, true); // enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                        // add current thread to WDT watch
#endif                                           // #if USE_TASK_WATCHDOG

  Serial.println("setup done");
}

void loop()
{
  pinStuff_setLED(led_off); // Set off while we run, then back on weak when done

  pinStuff_poll(); // Read HW buttons

#if USE_TASK_WATCHDOG
  esp_task_wdt_reset();
#endif // #if USE_TASK_WATCHDOG

  serialInput_poll();
#if LORA
  loraStuff_radioPoll();
  packetParser_poll();

  lis2dh_poll();

  if (utils_elapsedU32Ticks(g_lastMachStateSend_ms, millis()) > MACHSTATE_SEND_ITVL_MS)
  {
    Serial.printf("Sending machine state %d at %d\n", globalInts_getMachineState(), millis());
    packetParser_sendMachStateV1Packet((uint8_t)globalInts_getMachineState());
    g_lastMachStateSend_ms = millis();
  }
#endif // #if LORA

  oledStuff_printersPoll();
  pinStuff_setLED(led_weak); // Back to weak for sleep. If we never wake, it'll be constant
  yield();                   // Yield until the next tick
}
