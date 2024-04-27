#ifndef TAGDEFINITIONS_H_
#define TAGDEFINITIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

        /*******************************************************************************
         * Definitions
         ******************************************************************************/

#define MAX_PACKET_LEN 255 // Max number of bytes per packet

#define SEND_TX_POWER 1 // Send first byte of each packet
#define ALWAYS_SEND_CHIPID 1

#if ALWAYS_SEND_CHIPID && SEND_TX_POWER
#define PACKET_HEADER_SIZE (8 + 1 + 1 + 2) // 8 for 64-bit UUID, 1 for TX power 1 for type, 2 for length
#else
#define PACKET_HEADER_SIZE (1 + 2) // 1 for type, 2 for length
#endif                             // #if ALWAYS_SEND_CHIPID

        /*
                Packets have a type [uint8], then len[uint16], then data [bytes]
                Len is number of bytes in Value.
                Len can be 0, so that there is no value.

        */
        typedef enum
        {
                packetType_ack,
                packetType_ackAck,
                packetType_machStateV1,
        } packetType_t;

        typedef struct
        {
                uint64_t idBeingAcked; // The intended recipient of this ACK packet
                int16_t rxPacketRSSI;  // RSSI that the other side received our packet with
                uint8_t seqNoToAck;    // This is the seqNo that we are sending the ACK to.
                uint8_t seqNo;         // This is the seqNo of the ack(s)
                int8_t rxPacketSNR;    // SNR that the other side received our packet with
        } ackPacket_t;
#define ACK_PKTLEN_BYTES (5 + 8)

        typedef struct
        {
                uint64_t idBeingAckAcked;
                int16_t rxPacketRSSI;  // RSSI that the other side received our packet with
                int16_t ackPacketRSSI; // RSSI that we received the ACK with
                uint8_t seqNoToAckAck; // The seqNo that they are acking, that we AckAck
                uint8_t seqNoToAck;    // The seqNo of the ACK that we received, their seq no
                uint8_t seqNo;         // How many times we have ACKACKed their ACK
                int8_t rxPacketSNR;    // SNR that the other side received our packet with
                int8_t ackPacketSNR;   // SNR that we received the ACK with
        } ackAckPacket_t;
#define ACKACK_PKTLEN_BYTES (9 + 8)

        typedef struct
        {
                uint8_t machState;
                uint8_t seqNo; // Sequence number of this packet
        } machStateV1Packet_t;
#define MACHSTATE_V1_PKTLEN_BYTES (1 + 1)

#ifdef __cplusplus
}
#endif

#endif // #define TAGDEFINITIONS_H_