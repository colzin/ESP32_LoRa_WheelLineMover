#ifndef LORASTUFF_H_
#define LORASTUFF_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /*******************************************************************************
     * Definitions
     ******************************************************************************/

/* Transmission of 5-byte packet with bandwitdh 0, SF 8, codingrate 4, preamble 8, CRC on
 * took 6ms on pins, but printouts said 131ms
 */
#define LORA_TX_TIMEOUT 150 // Transmission timeout [ms]

// Wait for it to receive (), parse, then send a reply in this much time.
#define RX_AND_PARSE_TIMEOUT_MS 50
#define AWAIT_ACK_MS (RX_AND_PARSE_TIMEOUT_MS + LORA_TX_TIMEOUT) // How long to wait for an ACK after a TX packet

#define MIN_TX_ITVL_MS (LORA_TX_TIMEOUT + AWAIT_ACK_MS + 1)

    typedef enum
    {
        LoRa_IDLE = 0, //!< The radio is idle
        LoRa_RX,       //!< The radio is in reception state
        LoRa_TX,       //!< The radio is in transmission state
        LoRa_CAD,      //!< The radio is doing channel activity detection
    } loraRadioState_t;

    /*******************************************************************************
     * Functions
     ******************************************************************************/

    void loraStuff_initRadio(void);

    void loraStuff_setExpectingReply(bool isReplyExpected);

    void loraStuff_radioPoll(void);

    int8_t loraStuff_getCurrentTxdBm(void);

    loraRadioState_t loraStuff_getRadioState(void);

    void loraStuff_adjustTxPwr(int16_t theirRSSI);

    typedef enum
    {
        send_success = 0,
        sendFail_txAlreadyRunning,
        sendFail_awaitingReply,
        sendFail_error,
        sendFail_inOtherState,
        sendFail_dataTooLong,
        sendFail_needToWait,
    } sendFail_t;
    sendFail_t loraStuff_send(uint8_t *txPtr, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif // #define LORASTUFF_H_