
#include "Arduino.h"

#include "defines.h"
#include "loraStuff.h"
#include "oledStuff.h"
#include "packetParser.h"
#include "pinStuff.h"
#include "utils.h"

#include "ESP32_Mcu.h"

#include "esp_system.h"

#include "serialInput.h"

#include "task.h" // for vTaskList

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#if WIFI
#include <WiFi.h>
#include <WiFiClient.h>

#include "passwords.h"

#endif // #if WIFI

#if WEBSERVER
#include <WebServer.h>
#include <ESPmDNS.h>
#endif // #if WEBSERVER

/*******************************************************************************
 * Variables
 ******************************************************************************/

static uint64_t g_chipID;

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
  pinStuff_setLED(led_weak);
  pinStuff_initButtons();
  g_chipID = ESP.getEfuseMac();
  Serial.printf("ESP32ChipID 0x%08x%08x\n", (uint32_t)(g_chipID >> 32), (uint32_t)g_chipID);
  Serial.printf("ESP32 Chip model = %s Rev %d, %d cores\n", ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores());

  // Serial.printf("\nROM_core0 reason %d, ROM_core1 reason %d, rtc_0 reason %d, RTC_1 reason %d\n",
  //                 esp_rom_get_reset_reason(0), esp_rom_get_reset_reason(1), rtc_get_reset_reason(0), rtc_get_reset_reason(1));

  oledStuff_displayInit();

  // oledStuff_printESPInfo(ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(), ESP.getFlashChipSize(), g_chipID);

  // Init packet parser, then radio

#if LORA
  packetParser_init(g_chipID);
  loraStuff_initRadio();

#endif // #if LORA

#if BATT_MACHSTATE_PRINT_TO_OLED

  // analogSetClockDiv(255); // 1338mS
  //  analogSetCycles(8);                    // Set number of cycles per sample, default is 8 and provides an optimal result, range is 1 - 255
  //  analogSetSamples(1);                   // Set number of samples in the range, default is 1, it has an effect on sensitivity has been multiplied
  analogSetClockDiv(1);                 // Set the divider for the ADC clock, default is 1, range is 1 - 255
  analogSetAttenuation(ADC_11db);       // Sets the input attenuation for ALL ADC inputs, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db
  analogSetPinAttenuation(1, ADC_11db); // Sets the input attenuation, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db

  if (!adcAttachPin(1)) // GPIO1 is ADC input
  {
    Serial.printf("ADCATTACH 1 failed\n");
  }
  // adcAttachPin(37);

#endif // #if BATT_MACHSTATE_PRINT_TO_OLED

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

  pinStuff_pollButtons(); // Read HW buttons

#if USE_TASK_WATCHDOG
  esp_task_wdt_reset();
#endif // #if USE_TASK_WATCHDOG

  serialInput_poll();
#if LORA
  loraStuff_radioPoll();
  packetParser_poll();
#endif // #if LORA

#if WEBSERVER
  server.handleClient();
#endif // #if WEBSERVER

#if MQTT
  mqttStuff_poll();
#endif // #if MQTT

  oledStuff_printersPoll();
  pinStuff_setLED(led_weak); // Back to weak for sleep. If we never wake, it'll be constant
  yield();                   // Yield until the next tick
}
