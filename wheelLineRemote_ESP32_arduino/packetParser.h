#ifndef PACKETPARSER_H_
#define PACKETPARSER_H_

#include "tagDefinitions.h"

typedef struct
{
    uint64_t uuid64;
    uint16_t packetLen;
    int16_t rxRSSI;
    int8_t rxSNR, txdBm;
    uint8_t *pData;
    uint8_t pktType;
    uint8_t seqNo;
} rxPacket_t;

void packetParser_parseLoRaData(const uint8_t *pData, const uint16_t dataLen, const int16_t rxRSSI, const int8_t rxSNR);


void packetParser_poll(void);

void packetParser_init(uint64_t ourChipID);

#endif // PACKETPARSER_H_