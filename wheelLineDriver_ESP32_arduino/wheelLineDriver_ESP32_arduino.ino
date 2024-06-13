
#include "Arduino.h"

#include "defines.h" // Controls compile and runtime

#include "ESP32_Mcu.h"
#include "esp_system.h"
#include "globalInts.h" // For machine state
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

#define LAST_PACKET_TO_IDLE_MS 5000 // todo LOWER FOR SAFETY

/*******************************************************************************
 * Variables
 ******************************************************************************/

#if USE_TASK_WATCHDOG
#include <esp_task_wdt.h>
#define TASK_WDT_TIMEOUT_SEC 5 // Wait this long before WDT panic
#endif                         // #if USE_TASK_WATCHDOG

/*******************************************************************************
 * Code
 ******************************************************************************/

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("WheelLineDriver start");
  Mcu.begin();
  globalInts_setChipIDU64(ESP.getEfuseMac());
  Serial.printf("ESP32ChipID 0x%08x%08x\n", (uint32_t)(globalInts_getChipIDU64() >> 32), (uint32_t)globalInts_getChipIDU64());
  Serial.printf("ESP32 Chip model = %s Rev %d, %d cores\n", ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores());

  // Serial.printf("\nROM_core0 reason %d, ROM_core1 reason %d, rtc_0 reason %d, RTC_1 reason %d\n",
  //                 esp_rom_get_reset_reason(0), esp_rom_get_reset_reason(1), rtc_get_reset_reason(0), rtc_get_reset_reason(1));

  globalInts_setMachineState(machState_killEngine); // Default to kill unless told otherwise
  pinStuff_init();
  pinStuff_setLED(led_weak);
  oledStuff_displayInit();

  oledStuff_printESPInfo(ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(), ESP.getFlashChipSize(), globalInts_getChipIDU64());

  // Init packet parser, then radio

#if LORA
  packetParser_init(globalInts_getChipIDU64());
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
  // pinStuff_setLED(led_off); // Set off while we run, then back on weak when done

#if USE_TASK_WATCHDOG
  esp_task_wdt_reset();
#endif // #if USE_TASK_WATCHDOG

  // Look for commands on LoRa
#if LORA
  loraStuff_radioPoll();
  packetParser_poll();
#endif // #if LORA

  // Look for commands on UART
  serialInput_poll();
  // See if we need to time out, or have received commands within the keep alive time.
  if (machState_killEngine != globalInts_getMachineState())
  {
    uint32_t ago_ms = utils_elapsedU32Ticks(packetParser_lastMachV1PacketTimestamp(), millis());
    if (ago_ms > LAST_PACKET_TO_IDLE_MS)
    {
      Serial.printf("Killing engine after %d ms of no command (last at %d, now at %d)\n", ago_ms, packetParser_lastMachV1PacketTimestamp(), millis());
      globalInts_setMachineState(machState_killEngine);
    }
  }
  // Drive the pins
  pinStuff_poll();
  // Update screen too
  oledStuff_printersPoll();
  // done, sleep
  // pinStuff_setLED(led_weak); // Back to weak for sleep. If we never wake, it'll be constant
  yield(); // Yield until the next tick
}
