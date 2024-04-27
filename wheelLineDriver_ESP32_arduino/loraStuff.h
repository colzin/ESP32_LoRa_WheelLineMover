#ifndef LORASTUFF_H_
#define LORASTUFF_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void loraStuff_initRadio(void);

    void loraStuff_setExpectingReply(bool isReplyExpected);

    void loraStuff_radioPoll(void);

    int8_t loraStuff_getCurrentTxdBm(void);
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