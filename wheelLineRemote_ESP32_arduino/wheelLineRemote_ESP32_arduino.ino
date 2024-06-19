
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

#if USE_TASK_WATCHDOG
#include <esp_task_wdt.h>
#define TASK_WDT_TIMEOUT_SEC 5 // Wait this long before WDT panic
#endif                         // #if USE_TASK_WATCHDOG

static uint32_t g_lastMachStateSend_ms;

static uint8_t g_destID[CHIPID_LEN_BYTES]; // The destination, that we want to control

/*******************************************************************************
 * Code
 ******************************************************************************/

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("WheelLineRemote start");
  Mcu.begin();
  globalInts_setChipIDU64(ESP.getEfuseMac());
  Serial.printf("ESP32ChipID 0x%08x%08x\n", (uint32_t)(globalInts_getChipIDU64() >> 32), (uint32_t)globalInts_getChipIDU64());
  Serial.printf("ESP32 Chip model = %s Rev %d, %d cores\n", ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores());

  // Serial.printf("\nROM_core0 reason %d, ROM_core1 reason %d, rtc_0 reason %d, RTC_1 reason %d\n",
  //                 esp_rom_get_reset_reason(0), esp_rom_get_reset_reason(1), rtc_get_reset_reason(0), rtc_get_reset_reason(1));

  pinStuff_init();
  pinStuff_setLED(led_weak);

  globalInts_setNumRotations(0);
  lis2dh_init();
  oledStuff_displayInit();

  oledStuff_printESPInfo(ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(), ESP.getFlashChipSize(), globalInts_getChipIDU64());

  // Init packet parser, then radio

#if LORA
  // Init packet parser with our ID
  packetParser_init(globalInts_getChipIDU64());
  // Set our destination ID
  // // bc 7f 43 fa 12 f4
  // g_destID[0] = 0xbc;
  // g_destID[1] = 0x7f;
  // g_destID[2] = 0x43;
  // g_destID[3] = 0xfa;
  // g_destID[4] = 0x12;
  // g_destID[5] = 0xf4;
  // 34 f2 69 33 e8 64
  g_destID[0] = 0x34;
  g_destID[1] = 0xf2;
  g_destID[2] = 0x69;
  g_destID[3] = 0x33;
  g_destID[4] = 0xe8;
  g_destID[5] = 0x64;

  Serial.print("Destination (other end) ID: 0x ");
  for (uint8_t i = 0; i < CHIPID_LEN_BYTES; i++)
  {
    Serial.printf("%02x ", g_destID[i]);
  }
  Serial.printf("\n");

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
    // Serial.printf("Sending machine state %d at %d\n", globalInts_getMachineState(), millis());
    packetParser_sendMachStateV1Packet((uint8_t)globalInts_getMachineState(), g_destID);
    g_lastMachStateSend_ms = millis();
  }
#endif // #if LORA

  oledStuff_printersPoll();
  // done, sleep
  // pinStuff_setLED(led_weak); // Back to weak for sleep. If we never wake, it'll be constant
  yield(); // Yield until the next tick
}
