#ifndef PACKETPARSER_H_
#define PACKETPARSER_H_

#include "tagDefinitions.h"

#if (1 == PACKET_HEADER_VERSION_NUM)
typedef struct
{
    // don't need to store the dest chip ID. If it's not for us, ignore it
    uint8_t sourceChipID[CHIPID_LEN_BYTES];
    int16_t rxRSSI;
    int8_t rxSNR, txdBm;
    uint8_t *pData;
    uint8_t pktType;
    uint8_t seqNo;
} rxPacket_t;
#else
#error "Define header"
#endif // #if (1 == PACKET_HEADER_VERSION_NUM)

void packetParser_parseLoRaData(const uint8_t *pData, uint16_t dataLen, const int16_t rxRSSI, const int8_t rxSNR);

bool packetParser_sendMachStateV1Packet(uint8_t machState, uint8_t *pDestID);


rxPacket_t *packetParser_getLastMachStateV1Header(void);
uint8_t packetParser_getLastMachStV1SeqNo(void);

uint32_t packetParser_lastMachV1PacketTimestamp(void);

void packetParser_poll(void);

void packetParser_init(uint64_t ourChipIDU64);

#endif // PACKETPARSER_H_