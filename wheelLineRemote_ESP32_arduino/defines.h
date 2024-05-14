#ifndef DEFINES_H_
#define DEFINES_H_

#include <stdint.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define USE_TASK_WATCHDOG 1

#define SERIAL_BAUD_RATE 115200 // Would do 921600, but VSCode terminal won't support

#define LORA 0

#define MACHSTATE_SEND_ITVL_MS 1300 // How often to send out the machine state


#define BATT_MACHSTATE_PRINT_TO_OLED 1
#define LORA_RX_PRINT_TO_OLED 1
#define PACKET_PRINT_TO_OLED 1 // to print each RX packet to screen.

#define MIN_SCREEN_PRINT_ITVL_MS 160     // How long to wait between redrawing the screen
#define SCREEN_STATE_CHANGE_ITVL_MS 3000 // How long to keep polling the same printer, before auto-updating

#endif // #define DEFINES_H_