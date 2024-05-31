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

#define MAX_PACKET_LEN 254 // Sanity check

#define PACKET_HEADER_VERSION_NUM 1 // TODO increment for each change in header's bytes

#if (1 == PACKET_HEADER_VERSION_NUM)
/* Header will be as follows:
        Destination chip ID[num bytes] // Can ignore if not for us
        Source chip ID[num bytes]
        TX power [int8]
        Packet type [uint8]
*/
#define CHIPID_LEN_BYTES 6 // How many bytes to send for chip ID
// For this version's header:
#define TX_POWER_NUMBYTES 1
#define PACKET_TYPE_NUMBYTES 1
// No length, let type byte imply that
#define PACKET_HEADER_NUMBYTES (CHIPID_LEN_BYTES + CHIPID_LEN_BYTES + TX_POWER_NUMBYTES + PACKET_TYPE_NUMBYTES)

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
                int16_t rxPacketRSSI;   // RSSI that the other side received our packet with
                uint8_t seqNoToAck;     // This is the seqNo that we are sending the ACK to.
                uint8_t ackResendCount; // This is the seqNo of the ack(s), aka resend count
                int8_t rxPacketSNR;     // SNR that the other side received our packet with
        } ackPacket_t;
#define ACK_PKTLEN_BYTES (CHIPID_LEN_BYTES + 5)

        typedef struct
        {
                int16_t rxPacketRSSI;   // RSSI that the other side received our packet with
                int16_t ackPacketRSSI;  // RSSI that we received the ACK with
                int8_t rxPacketSNR;     // SNR that the other side received our packet with
                int8_t ackPacketSNR;    // SNR that we received the ACK with
                uint8_t seqNo;          // The original packet's sequence number being acked
                uint8_t ackResendCount; // How many times they have repeated this ACK to us

        } ackAckPacket_t;
#define ACKACK_PKTLEN_BYTES (CHIPID_LEN_BYTES + 9)

        typedef struct
        {
                uint8_t machState; // State to put receiver into
                uint8_t seqNo;     // Sequence number of this packet
        } machStateV1Packet_t;
#define MACHSTATE_V1_PKTLEN_BYTES (1 + 1)

#else
#error "Please define packet header for this version "
#endif // #if (1 == PACKET_HEADER_VERSION_NUM)

#ifdef __cplusplus
}
#endif

#endif // #define TAGDEFINITIONS_H_