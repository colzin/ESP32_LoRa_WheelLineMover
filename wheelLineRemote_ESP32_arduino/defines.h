#ifndef DEFINES_H_
#define DEFINES_H_

#include <stdint.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define USE_TASK_WATCHDOG 1

#define SERIAL_BAUD_RATE 115200 // Would do 921600, but VSCode terminal won't support

#define LORA 1

#define UNIT_1 0
#if !UNIT_1
#define UNIT_2 1
#if !UNIT_2
#error "Select the target unit
#endif // #if !UNIT_2

#endif // #if !UNIT_1

#include "loraStuff.h"
#define MACHSTATE_SEND_ITVL_MS (MIN_TX_ITVL_MS + 300) // How often to send out the machine state

#define BATT_MACHSTATE_PRINT_TO_OLED 1
#define LORA_RX_PRINT_TO_OLED 1
#define PACKET_PRINT_TO_OLED 1 // to print each RX packet to screen.

#define MIN_SCREEN_PRINT_ITVL_MS 160     // How long to wait between redrawing the screen
#define SCREEN_STATE_CHANGE_ITVL_MS 3000 // How long to keep polling the same printer, before auto-updating

#endif // #define DEFINES_H_