

#ifndef OLEDSTUFF_H_
#define OLEDSTUFF_H_

#include "defines.h"

#if WIFI
#include "WiFiType.h"
#endif // #if WIFI

#include "packetParser.h" // for struct defs

#include "HT_SSD1306Wire.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define MIN_SCREEN_PRINT_ITVL_MS 80      // How long to wait between redrawing the screen
#define SCREEN_STATE_CHANGE_ITVL_MS 3000 // How long to keep polling the same printer

/*******************************************************************************
 * Functions
 ******************************************************************************/

void oledStuff_displayInit(void);

void oledStuff_printESPInfo(const char *chipModel, uint8_t chipRev, uint8_t chipCores, uint32_t flashChipSize, uint64_t chipID);

void oledStuff_printMachStateV1Packet(rxPacket_t *pRxPacket, machStateV1Packet_t *pData);

void oledStuff_printersPoll(void);

#endif // #define OLEDSTUFF_H_