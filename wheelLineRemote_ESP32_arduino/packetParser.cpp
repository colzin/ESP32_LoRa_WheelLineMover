#include "packetParser.h"
#include "defines.h"

#if PACKET_PRINT_TO_OLED
#include "oledStuff.h"
#endif // #if PACKET_PRINT_TO_OLED

#include "Arduino.h"

#include "loraStuff.h"
#include "pinStuff.h"
#include "tagDefinitions.h"

#include "utils.h"

#include <string.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef enum
{
    senderState_idle,
    senderState_tryingToSend,
    senderState_awaitingAck,

} senderState_t;

typedef struct
{
    uint8_t pBuffer[MAX_PACKET_LEN];
    uint32_t size;
    bool expectReply;
} txPacket_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint64_t g_chipID;
// Keep track of the stats on the last packet received, and last ack packet received.
static rxPacket_t g_lastReceivedPacket;
// Store data from the last received ACK
static int16_t g_receivedAckRSSI;
static int8_t g_receivedAckSNR;
static ackPacket_t g_receivedAckData;

// for whether to send ack or ackAck in reply
static uint64_t g_idToAck, g_idToAckAck;

// TODO sequence number tracking/ACKing later

static uint8_t g_seqNoToBeAcked, g_ackResendCnt, g_ackAckResendCnt; // How many ACKs we have sent

static senderState_t g_sendState;

#define MAX_TX_PACKETS 5
static txPacket_t g_txSlots[MAX_TX_PACKETS];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

// bools and data so poll can send later, instead of sending from the rx parser.
static void sendAckAckPacket(uint64_t idToSendTo, ackPacket_t *pAckPacket, int16_t ackRxRSSI, int8_t ackRxSNR);
static void sendAckPacket(rxPacket_t *pPacketToAck);

/*******************************************************************************
 * Code
 ******************************************************************************/

static bool enqueuePacket(uint8_t *pkt, uint32_t len, bool expectReply)
{
    for (uint8_t i = 0; i < MAX_TX_PACKETS; i++)
    {
        if (!g_txSlots[i].size)
        { // Copy in the data, populate the length
            memcpy(&g_txSlots[i].pBuffer, pkt, len);
            g_txSlots[i].size = len;
            g_txSlots[i].expectReply = expectReply;
            // Serial.printf("Filled Tx slot %d with len %d\n", i, len);
            return true;
        }
    }
    Serial.printf("No room to enqueue TX packet, dropping!\n");
    return false;
}

static void parseAckAckPacket(rxPacket_t *pkt)
{
    // Parse the bytes in the ackAck tag
    if (ACKACK_PKTLEN_BYTES != pkt->packetLen)
    {
        Serial.printf("    ERROR parsing ackAck tag, len %d should be %d\n", pkt->packetLen, ACKACK_PKTLEN_BYTES);
        return;
    }
    // Check if this AckAck is for us
    uint64_t thisAckAckID;
    pkt->pData = ReadBEUint64(pkt->pData, &thisAckAckID); // The chip they are sending the ACK to
    if (g_chipID != thisAckAckID)
    {
        Serial.printf("Received AckAck destined for ID 0x%08x%08x, we are 0x%08x%08x, ignoring\n",
                      (uint32_t)(thisAckAckID >> 32), (uint32_t)(thisAckAckID), (uint32_t)(g_chipID >> 32), (uint32_t)(g_chipID));
        return;
    }
    ackAckPacket_t ackAckData;
    ackAckData.idBeingAckAcked = thisAckAckID;
    pkt->pData = ReadBEUint8(pkt->pData, &ackAckData.seqNo);         // How many times we have ACKACKed
    pkt->pData = ReadBEUint8(pkt->pData, &ackAckData.seqNoToAck);    // The seq number of the ACK that we received.
    pkt->pData = ReadBEUint8(pkt->pData, &ackAckData.seqNoToAckAck); // The seq number they are ACKing.
    pkt->pData = ReadBEInt16(pkt->pData, &ackAckData.rxPacketRSSI);
    pkt->pData = ReadBEInt8(pkt->pData, &ackAckData.rxPacketSNR);
    pkt->pData = ReadBEInt16(pkt->pData, &ackAckData.ackPacketRSSI);
    pkt->pData = ReadBEInt8(pkt->pData, &ackAckData.ackPacketSNR);
    // If they sent us an ackAck, that tells us how strong they received our ack packet, so we can adjust TX strength.
    Serial.printf("    We received ackACK that said they received our ack packet at RSSI %d, SNR %d\n", ackAckData.ackPacketRSSI, ackAckData.ackPacketSNR);
    loraStuff_adjustTxPwr(ackAckData.ackPacketRSSI);
    // TODO sequence number tracking/ACKing later
    // Serial.printf("    Received ACKACK for our seq no %d, ack %d, ackAck %d!\n", ackAckData.seqNoToAckAck, ackAckData.seqNoToAck, ackAckData.seqNo);
    // Do not send reply in parser

#if PACKET_PRINT_TO_OLED
    // oledStuff_printAckAckPacket(pkt, &ackAckData); Too verbose
#endif // #if PACKET_PRINT_TO_OLED
}

// Return true if we should AckAck this Ack packet
static bool parseAckPacket(rxPacket_t *pkt)
{
    // Parse the bytes in the ack tag
    if (ACK_PKTLEN_BYTES != pkt->packetLen)
    {
        Serial.printf("    ERROR parsing ack tag, len %d should be %d\n", pkt->packetLen, ACK_PKTLEN_BYTES);
        return false;
    }
    // Check if this ACK is for us
    uint64_t thisAckID;
    pkt->pData = ReadBEUint64(pkt->pData, &thisAckID); // The chip they are sending the ACK to
    if (g_chipID != thisAckID)
    {
        Serial.printf("Received ACK destined for ID 0x%08x%08x, we are 0x%08x%08x, ignoring\n",
                      (uint32_t)(thisAckID >> 32), (uint32_t)(thisAckID), (uint32_t)(g_chipID >> 32), (uint32_t)(g_chipID));
        return false;
    }
    // Read the fields into the struct
    g_receivedAckData.idBeingAcked = thisAckID;                          // The chip they are sending the ACK to
    pkt->pData = ReadBEUint8(pkt->pData, &g_receivedAckData.seqNoToAck); // The seq number they are ACKing.
    pkt->pData = ReadBEUint8(pkt->pData, &g_receivedAckData.seqNo);      // How many times they have ACKed
    pkt->pData = ReadBEInt16(pkt->pData, &g_receivedAckData.rxPacketRSSI);
    pkt->pData = ReadBEInt8(pkt->pData, &g_receivedAckData.rxPacketSNR);
    // If they sent us an ACK, that tells us how strong they received our original packet, so we can adjust TX strength.
    Serial.printf("    We received ACK that said they received our packet at RSSI %d, SNR %d\n", g_receivedAckData.rxPacketRSSI, g_receivedAckData.rxPacketSNR);
    loraStuff_adjustTxPwr(g_receivedAckData.rxPacketRSSI);
    // TODO sequence number tracking/ACKing later
    Serial.printf("    Received ACK for our seq no %d, they sent %d\n", g_seqNoToBeAcked, g_receivedAckData.seqNo);
    // Do not send reply in parser
#if PACKET_PRINT_TO_OLED
    // oledStuff_printAckPacket(pkt, &ackData); Too verbose
#endif           // #if PACKET_PRINT_TO_OLED
    return true; // We should AckAck this one
}

static void parseMachstateV1Packet(rxPacket_t *pkt)
{
    // Parse the bytes in the ack tag
    if (MACHSTATE_V1_PKTLEN_BYTES != pkt->packetLen)
    {
        Serial.printf("  ERROR parsing machStateV1 packet len %d, should be %d\n", pkt->packetLen, MACHSTATE_V1_PKTLEN_BYTES);
        return;
    }
    machStateV1Packet_t data;
    // Read the fields into the struct
    pkt->pData = ReadBEUint8(pkt->pData, &data.seqNo);
    pkt->pData = ReadBEUint8(pkt->pData, &data.machState);
    Serial.printf("  Seq no %d, Machine state %d\n", data.seqNo, data.machState);
#if PACKET_PRINT_TO_OLED
    oledStuff_printMachStateV1Packet(pkt, &data);
#endif // #if PACKET_PRINT_TO_OLED
}

void packetParser_parseLoRaData(const uint8_t *pData, const uint16_t dataLen, const int16_t rxRSSI, const int8_t rxSNR)
{
    if (dataLen < PACKET_HEADER_SIZE || dataLen > MAX_PACKET_LEN)
    {
        Serial.printf(" parseTag called with INVALID len %d, IGNORING\n", dataLen);
        return;
    }
    rxPacket_t thisRXPkt;
    thisRXPkt.rxRSSI = rxRSSI;
    thisRXPkt.rxSNR = rxSNR;
    thisRXPkt.pData = (uint8_t *)pData;
#if ALWAYS_SEND_CHIPID
    thisRXPkt.pData = ReadBEUint64(thisRXPkt.pData, &thisRXPkt.uuid64);
#endif // #if ALWAYS_SEND_CHIPID
#if SEND_TX_POWER
    thisRXPkt.pData = ReadBEInt8(thisRXPkt.pData, &thisRXPkt.txdBm);
#endif // #if SEND_TX_POWER
    // Read type, a uint8
    thisRXPkt.pData = ReadBEUint8(thisRXPkt.pData, &thisRXPkt.pktType);
    // Read length, a uint16
    thisRXPkt.pData = ReadBEUint16(thisRXPkt.pData, &thisRXPkt.packetLen);
    // rxPacket.pData has now been advanced to start of packet data
    if (thisRXPkt.packetLen != dataLen - PACKET_HEADER_SIZE)
    {
        Serial.printf(" ERROR: Rx %d bytes, read pktType %d, pktLen %d dataLen-PACKET_HEADER_SIZE is %d bytes!\n", dataLen, thisRXPkt.pktType, thisRXPkt.packetLen, dataLen - PACKET_HEADER_SIZE);
        // Bail out
        return;
    }
    // Read pktLen bytes of data for the tag.
    Serial.printf(" Rx from ID 0x%08x%08x packet type %d, RSSI %d, SNR %d, len %d:\n", (uint32_t)(thisRXPkt.uuid64 >> 32), (uint32_t)thisRXPkt.uuid64, thisRXPkt.pktType, thisRXPkt.rxRSSI, thisRXPkt.rxSNR, thisRXPkt.packetLen);
    bool savePacket = false;
    switch ((packetType_t)thisRXPkt.pktType)
    {
    case packetType_ack:
        // use savePacket, but don't save to g_lastReceivedPacket
        savePacket = parseAckPacket(&thisRXPkt); // Save to send AckAck if this is for us
        if (savePacket)
        {
            g_idToAckAck = thisRXPkt.uuid64; // We should send an AckAck since they sent an Ack, so set the ID of the one to ACK
            g_receivedAckRSSI = thisRXPkt.rxRSSI;
            g_receivedAckSNR = thisRXPkt.rxSNR;
            savePacket = false; // Do not save down below since this was an ACK packet and we stored the info in g_receivedAckxxx
        }
        break;
    case packetType_ackAck:
        g_ackResendCnt = 0; // Zero this counter, since it ACKACKed our ACK
        parseAckAckPacket(&thisRXPkt);
        savePacket = false; // Don't need to save this one.
        break;

    case packetType_machStateV1:
        g_idToAck = thisRXPkt.uuid64; // We should send an ACK, so set the ID of the one to ACK
        parseMachstateV1Packet(&thisRXPkt);
        savePacket = true; // We are the hub, and need to ACK these packets
        break;

    default:
        savePacket = false;
        Serial.printf(" Don't know how to parse pktType of %d, len %d, ignoring\n", thisRXPkt.pktType, thisRXPkt.packetLen);
        break;
    }
    if (savePacket)
    { // copy the packet into our lastReceivedPacket struct
        g_lastReceivedPacket = thisRXPkt;
    }
    // return the number of bytes parsed
    return;
}

static uint8_t *populateHeaderMinusLen(uint8_t *pData, packetType_t pktType)
{
#if ALWAYS_SEND_CHIPID
    pData = WriteBEUint64(pData, g_chipID);
#endif // #if ALWAYS_SEND_CHIPID
#if SEND_TX_POWER
    pData = WriteBEInt8(pData, loraStuff_getCurrentTxdBm());
#endif // #if SEND_TX_POWER
       // Put in the packet type and length, which is 3 for this packet
    pData = WriteBEUint8(pData, (uint8_t)pktType);
    return pData;
}

static void sendAckAckPacket(uint64_t idToSendTo, ackPacket_t *pAckPacket, int16_t ackRxRSSI, int8_t ackRxSNR)
{
    uint8_t txPacket[PACKET_HEADER_SIZE + ACKACK_PKTLEN_BYTES];
    uint8_t *ptr = populateHeaderMinusLen(txPacket, packetType_ackAck);
    // We put in our length
    ptr = WriteBEUint16(ptr, ACKACK_PKTLEN_BYTES);
    // Now we populate stuff unique to ackAck packet
    // Write the ID of the sender of the Ack packet that we last received
    ptr = WriteBEUint64(ptr, idToSendTo);
    // Set the seq number of the original packet.
    ptr = WriteBEUint8(ptr, pAckPacket->seqNoToAck);
    // Set the seq no of the ACK we received.
    ptr = WriteBEUint8(ptr, pAckPacket->seqNo);
    // Set the seqNo of the ackAck, incrementing every re-send
    ptr = WriteBEUint8(ptr, g_ackAckResendCnt++);
    // RSSI that the receiver of the packet ACKed with
    ptr = WriteBEInt16(ptr, pAckPacket->rxPacketRSSI);
    ptr = WriteBEInt8(ptr, pAckPacket->rxPacketSNR);
    // RSSI and SNR that the ACKer said they received
    ptr = WriteBEInt16(ptr, ackRxRSSI);
    ptr = WriteBEInt8(ptr, ackRxSNR);
    enqueuePacket(txPacket, (uint32_t)(ptr - txPacket), false);
    Serial.printf("Enqueued AckAck packet to 0x%08x%08x seqNoToAck %d, seqNo %d, ACK RX RSSI %d, their RSSI %d\n", (uint32_t)(idToSendTo >> 32), (uint32_t)(idToSendTo), pAckPacket->seqNoToAck, pAckPacket->seqNo, ackRxRSSI, pAckPacket->rxPacketRSSI);
}

static void sendAckPacket(rxPacket_t *pPacketToAck)
{
    uint8_t txPacket[PACKET_HEADER_SIZE + ACK_PKTLEN_BYTES];
    uint8_t *ptr = populateHeaderMinusLen(txPacket, packetType_ack);
    // We put in our length
    ptr = WriteBEUint16(ptr, ACK_PKTLEN_BYTES);
    // Now populate stuff unique to ACK packet
    // Put the sender's ID in
    ptr = WriteBEUint64(ptr, pPacketToAck->uuid64);
    // Set the seq number to ack.
    ptr = WriteBEUint8(ptr, pPacketToAck->seqNo);
    // Set the seqNo of the ACK, incrementing every re-send
    ptr = WriteBEUint8(ptr, g_ackResendCnt++);
    ptr = WriteBEInt16(ptr, pPacketToAck->rxRSSI);
    ptr = WriteBEInt8(ptr, pPacketToAck->rxSNR);
    enqueuePacket(txPacket, (uint32_t)(ptr - txPacket), true);
    Serial.printf("Enqueued ack packet to 0x%08x%08x seq no %d, received with RSSI %d, SNR %d\n", (uint32_t)(pPacketToAck->uuid64 >> 32), (uint32_t)(pPacketToAck->uuid64), pPacketToAck->seqNo, pPacketToAck->rxRSSI, pPacketToAck->rxSNR);
}

bool packetParser_sendMachStateV1Packet(uint8_t machState)
{
    uint8_t txPacket[PACKET_HEADER_SIZE + MACHSTATE_V1_PKTLEN_BYTES];
    uint8_t *ptr = populateHeaderMinusLen(txPacket, packetType_machStateV1);
    // We put in our length
    ptr = WriteBEUint16(ptr, MACHSTATE_V1_PKTLEN_BYTES);
    // Set our packet's bytes
    ptr = WriteBEUint8(ptr, g_seqNoToBeAcked++);
    ptr = WriteBEUint8(ptr, machState);
    bool ret = enqueuePacket(txPacket, (uint32_t)(ptr - txPacket), true);
    if (!ret)
    {
        Serial.printf("FAILED to enqueue machStateV1 packet!\n");
    }
    return ret;
}

uint8_t packetParser_getLastMachStV1SeqNo(void)
{
    return g_seqNoToBeAcked;
}

#define SEND_ERR_PRINT_ITVL_MS 500
#if SEND_ERR_PRINT_ITVL_MS
static bool g_lastSendErrored;
static uint32_t g_lastSendErrPrint_ms;
#endif // #if SEND_ERR_PRINT_ITVL_MS
void packetParser_poll(void)
{
    // check if we should send ACK and send it here.
    if (g_idToAck)
    {
        // Serial.printf("Should send ack:\n");
        sendAckPacket(&g_lastReceivedPacket);
        g_idToAck = 0; // Zero the ID to invalidate
        Serial.printf("Sent ack at %d, awaiting ackAck\n", millis());
    }
    if (g_idToAckAck)
    {
        // Serial.printf("Should send ackAck:\n");
        sendAckAckPacket(g_idToAckAck, &g_receivedAckData, g_receivedAckRSSI, g_receivedAckSNR);
        g_idToAckAck = 0; // Zero the ID to invalidate
        Serial.printf("Sent ackAck at %d, done awaiting replies\n", millis());
    }
    for (uint8_t i = 0; i < MAX_TX_PACKETS; i++)
    {
        if (g_txSlots[i].size)
        { // Copy in the data, populate the length
            sendFail_t sendRet = loraStuff_send(g_txSlots[i].pBuffer, g_txSlots[i].size);
            switch (sendRet)
            {
            case send_success:
            {
                Serial.printf(" Just sent slot %d, length %d\n", i, g_txSlots[i].size);
                g_txSlots[i].size = 0;
                loraStuff_setExpectingReply(g_txSlots[i].expectReply);
#if SEND_ERR_PRINT_ITVL_MS
                // Set so first error will print
                g_lastSendErrored = false;
#endif // #if SEND_ERR_PRINT_ITVL_MS
                return;
            }
            break;
            case sendFail_dataTooLong:
                Serial.printf("Data too long error, throw out packet");
                g_txSlots[i].size = 0;
                break;
            default:
            {
#if SEND_ERR_PRINT_ITVL_MS
                if (!g_lastSendErrored || utils_elapsedU32Ticks(g_lastSendErrPrint_ms, millis()) > SEND_ERR_PRINT_ITVL_MS)
                {
                    g_lastSendErrPrint_ms = millis();
                    Serial.printf(" Slot %d, length %d couldn't send at %d ms, error %d, retry later\n", i, g_txSlots[i].size, g_lastSendErrPrint_ms, sendRet);
                }
                g_lastSendErrored = true;
#endif // #if SEND_ERR_PRINT_ITVL_MS
                return;
            }
            break;
            }
        }
    }
}

void packetParser_init(uint64_t ourChipID)
{
    g_seqNoToBeAcked = 0;
    g_ackResendCnt = 0;
    g_ackAckResendCnt = 0;
    g_chipID = ourChipID;
}