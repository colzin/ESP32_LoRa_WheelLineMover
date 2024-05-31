
#include "oledStuff.h"

#include "Arduino.h"
// the display on the LoRa 32
#include "HT_SSD1306Wire.h"

#include "globalInts.h"
#include "packetParser.h"
#include "pinStuff.h"

#include "utils.h"

#if BATT_MACHSTATE_PRINT_TO_OLED
// #include "board.h"
#include "esp32-hal-adc.h"
#endif // #if BATT_MACHSTATE_PRINT_TO_OLED

/*******************************************************************************
 * Definitions
 ******************************************************************************/

// for printers
typedef enum
{
  printer_rx = 0,
  printer_battMachState,
  printer_max,
} oledPrinter_t;

#define MAX_SCREEN_WIDTH_CHARS 31 // about 31

/*******************************************************************************
 * Variables
 ******************************************************************************/

// SSD1306Wire display(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10); // addr , freq , i2c group , resolution , rst
SSD1306Wire displayInstance(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

static uint32_t g_delayToPrint_ms, g_lastPrint_ms;

static oledPrinter_t g_currentPrinter;
static uint32_t g_printStateChange_ms;

/*******************************************************************************
 * Code
 ******************************************************************************/

static void oledStuff_displayOn(void)
{
  if (!pinStuff_requestVEXT(VEXT_REQ_BIT_OLED))
  { // Wait for OLED screen to boot, TODO find time
    delay(10);
  }
  // Always re-init and clear it.
  displayInstance.init();
  displayInstance.setTextAlignment(TEXT_ALIGN_LEFT);
  displayInstance.clear();
  displayInstance.display();
}

static void oledStuff_displayOff(void)
{
  // Set it blank, in case we can't turn off VEXT
  displayInstance.clear();
  displayInstance.display();
  pinStuff_releaseVEXT(VEXT_REQ_BIT_OLED);
}

void oledStuff_displayInit(void)
{
  oledStuff_displayOn();
  // Each function that uses the display can set its own font size, and must clear on start
}

void oledStuff_printESPInfo(const char *chipModel, uint8_t chipRev, uint8_t chipCores, uint32_t flashChipSize, uint64_t chipID)
{
  displayInstance.clear();
  displayInstance.setFont(ArialMT_Plain_16);
  char str[MAX_SCREEN_WIDTH_CHARS + 1];
  // First line
  uint32_t index = 0;
  index += snprintf(str + index, MAX_SCREEN_WIDTH_CHARS - index, "%sr%d,%dcores", chipModel, chipRev, chipCores);
  str[index] = 0;
  displayInstance.drawString(0, 0, str);
  // Second line
  index = 0;
  index += snprintf(str + index, MAX_SCREEN_WIDTH_CHARS - index, "flash%d b", flashChipSize);
  str[index] = 0;
  displayInstance.drawString(0, 16, str);
  // Third line
  index = 0;
  index += snprintf(str + index, MAX_SCREEN_WIDTH_CHARS - index, "Chip ID:0x%04x", (uint16_t)(chipID >> 48));
  str[index] = 0;
  displayInstance.drawString(0, 32, str);
  // Fourth line
  index = 0;
  index += snprintf(str + index, MAX_SCREEN_WIDTH_CHARS - index, "%04x%08x", (uint16_t)(chipID >> 32), (uint32_t)(chipID));
  str[index] = 0;
  displayInstance.drawString(0, 48, str);
  displayInstance.display();
}

static uint32_t snprintfIDHex(char *ptr, uint32_t maxlen, uint8_t *pID)
{
  uint32_t ret = 0; // How many we have printed
  Serial.print("0x");
  for (uint8_t i = 0; i < CHIPID_LEN_BYTES; i++)
  {
    ret += snprintf(ptr + ret, maxlen - ret, "%02x ", pID[i]);
  }
}

// Clears and fills the first two lines
static void printPacketHeader(rxPacket_t *pRxPacket)
{
  displayInstance.clear();
  displayInstance.setFont(ArialMT_Plain_10);
  char str[MAX_SCREEN_WIDTH_CHARS + 1];
  uint32_t index = 0;
  index += snprintf(str + index, MAX_SCREEN_WIDTH_CHARS - index, "%x,TX%d,RX%d,SNR%d", pRxPacket->pktType, pRxPacket->txdBm, pRxPacket->rxRSSI, pRxPacket->rxSNR);
  str[index] = 0;
  // Serial.printf("%s,\n", str);
  displayInstance.drawString(0, 0, str);

  // Second line
  index = 0;
  index += snprintf(str + index, MAX_SCREEN_WIDTH_CHARS - index, "ID 0x");
  index += snprintfIDHex(str + index, MAX_SCREEN_WIDTH_CHARS - index, pRxPacket->sourceChipID);
  str[index] = 0;
  // Serial.printf("%s,\n", str);
  displayInstance.drawString(0, 16, str);
}

void oledStuff_printMachStateV1Packet(rxPacket_t *pRxPacket, machStateV1Packet_t *pData)
{
  // Clears and fills the first two lines
  printPacketHeader(pRxPacket);
  // Third line
  char str[MAX_SCREEN_WIDTH_CHARS + 1];
  uint32_t index = 0;
  index += snprintf(str + index, MAX_SCREEN_WIDTH_CHARS - index, "st: %d ", pData->machState);
  str[index] = 0;
  // Serial.printf("%s,\n", str);
  displayInstance.drawString(0, 32, str);

  // Fourth line, nothing on this packet

  displayInstance.display(); // Send it all
  g_lastPrint_ms = millis();
  g_delayToPrint_ms = SCREEN_STATE_CHANGE_ITVL_MS;
}

#if BATT_MACHSTATE_PRINT_TO_OLED

static void battMachStatePrint(void)
{
  displayInstance.clear();
  displayInstance.setFont(ArialMT_Plain_16);
  char str[64]; // Max of about 120 wide?
  // First line
  uint32_t index = sprintf(str, "Batt:%dmV,ID24:\n", pinStuff_getBatterymV());
  str[index] = 0;
  // Serial.printf(str);
  displayInstance.drawString(0, 0, str);

  // Second line, ID
  index = sprintf(str, "0x%04x %08x", (uint32_t)(globalInts_getChipIDU64() >> 32), (uint32_t)globalInts_getChipIDU64());
  str[index] = 0;
  displayInstance.drawString(0, 16, str);

  // Third line, machine state
  index = sprintf(str, "State:%d,count %d", globalInts_getMachineState(), packetParser_getLastMachStV1SeqNo());
  str[index] = 0;
  displayInstance.drawString(0, 32, str);

  // Fourth line, LoRa stats
  index = snprintf(str, sizeof(str), "Tx Pwr:%d, ", loraStuff_getCurrentTxdBm());
  switch (loraStuff_getRadioState())
  {
  case LoRa_IDLE:
    index += snprintf(str + index, sizeof(str) - index, "idle");
    break;
  case LoRa_RX:
    index += snprintf(str + index, sizeof(str) - index, " RX ");
    break;
  case LoRa_TX:
    index += snprintf(str + index, sizeof(str) - index, " TX ");
    break;
  case LoRa_CAD:
    index += snprintf(str + index, sizeof(str) - index, "CAD ");
    break;
  }
  str[index] = 0;
  displayInstance.drawString(0, 48, str);

  displayInstance.display(); // Send it all
}
#endif // #if BATT_MACHSTATE_PRINT_TO_OLED

void oledStuff_printersPoll(void)
{
  if (utils_elapsedU32Ticks(g_lastPrint_ms, millis()) < MIN_SCREEN_PRINT_ITVL_MS)
  {
    return;
  }
  if (utils_elapsedU32Ticks(g_lastPrint_ms, millis()) < g_delayToPrint_ms)
  {
    return;
  }
  switch (g_currentPrinter)
  {
  case printer_rx: // The RX printer gets precedence
    g_lastPrint_ms = millis();
    break;
  case printer_battMachState:
#if BATT_MACHSTATE_PRINT_TO_OLED
    battMachStatePrint();
    g_lastPrint_ms = millis();
#else
    g_currentPrinter = printer_rx;
#endif // #if BATT_MACHSTATE_PRINT_TO_OLED
    break;
  default:
    g_currentPrinter = printer_battMachState;
  }
  if (utils_elapsedU32Ticks(g_printStateChange_ms, millis()) > SCREEN_STATE_CHANGE_ITVL_MS)
  {
    g_currentPrinter = (oledPrinter_t)((uint32_t)g_currentPrinter + 1);
    g_printStateChange_ms = millis();
    return;
  }
}
