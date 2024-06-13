#include "packetParser.h"
#include "defines.h"

#if PACKET_PRINT_TO_OLED
#include "oledStuff.h"
#endif // #if PACKET_PRINT_TO_OLED

#include "Arduino.h"

#include "globalInts.h"
#include "loraStuff.h"

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
    packetType_t type;
} txPacket_t;

#define SEND_ERR_PRINT_ITVL_MS 102

/*******************************************************************************
 * Variables
 ******************************************************************************/

static uint8_t g_ourChipID[CHIPID_LEN_BYTES]; // for us to keep track of, in terms of chipID passed to us

// Remote: keep track of our TX sequence number
static uint8_t g_txMachStateSeqNo;

// Keep track of the stats on the last packet received, and last ack packet received.
static bool g_shouldAckMachState = false;
static rxPacket_t g_lastMachStateV1Header; // The packet header (contains seq no, RSSI, SNR)
static uint8_t g_idToAck[CHIPID_LEN_BYTES];

static uint8_t g_ackResendCnt; // How many times we have re-sent this ACK

// Store data from the last received ACK
static bool g_shouldAckAck = false;
static uint8_t g_idToAckAck[CHIPID_LEN_BYTES];
static int16_t g_receivedAckRSSI;
static int8_t g_receivedAckSNR;
static ackPacket_t g_receivedAckData; // The data that they sent in their ACK to us

static senderState_t g_sendState;

#define MAX_TX_PACKETS 5
static txPacket_t g_txSlots[MAX_TX_PACKETS];

static uint32_t g_lastV1PacketRx_ms;

static const char *g_sendErrors[sendFail_count] = {"Success", "txAlreadyRunning", "awaitingReply", "error", "inOtherState", "dataTooLong", "sendFail_needToWait"};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

// bools and data so poll can send later, instead of sending from the rx parser.
static void sendAckAckPacket(uint64_t idToSendTo, ackPacket_t *pAckPacket, int16_t ackRxRSSI, int8_t ackRxSNR);
static void sendAckPacket(void);

/*******************************************************************************
 * Code
 ******************************************************************************/

static bool enqueuePacket(uint8_t *pkt, uint32_t len, packetType_t packetType)
{
    for (uint8_t i = 0; i < MAX_TX_PACKETS; i++)
    {
        if (!g_txSlots[i].size)
        { // Copy in the data, populate the length
            memcpy(&g_txSlots[i].pBuffer, pkt, len);
            g_txSlots[i].size = len;
            g_txSlots[i].type = packetType;
            // Serial.printf("Filled Tx slot %d with type %d, len %d\n", i, packetType, len);
            return true;
        }
    }
    Serial.printf("No room to enqueue TX packet, dropping!\n");
    return false;
}

static uint8_t *writeU64ChipID(uint8_t *pBuffer, uint64_t u64)
{
    // Serial.printf("Starting with pBuf 0x%x\n", (uint32_t)pBuffer);
    for (uint8_t i = 1; i <= CHIPID_LEN_BYTES; i++)
    { // Shift down, put in buffer MSByte first
        uint8_t byteVal = (uint8_t)((u64 >> ((CHIPID_LEN_BYTES - i) * 8)) & 0xFF);
        // Serial.printf("i=%d, pBuffer 0x%x, byteVal 0x%02x\n", i, (uint32_t)pBuffer, byteVal);
        *pBuffer++ = byteVal;
    }
    return pBuffer;
}

static void printID(uint8_t *pChipID, bool printNewline)
{
    for (uint8_t i = 0; i < CHIPID_LEN_BYTES; i++)
    {
        Serial.printf("%02x ", pChipID[i]);
    }
    if (printNewline)
    {
        Serial.printf("\n");
    }
}

static bool isPacketForUs(uint8_t *pChipID)
{
    int ret = memcmp(g_ourChipID, pChipID, CHIPID_LEN_BYTES);
    if (ret)
    { // memcmp returns 0 if all bytes are same
        // Print just for debugging
        Serial.print("Packet destined for ID ");
        printID(pChipID, false);
        Serial.print(", we are ");
        printID(g_ourChipID, false);
        Serial.println(" . Not for us.");
        return false;
    }
    else
    {
        return true;
    }
}

static void parseAckPacket(rxPacket_t *pkt)
{
    // Read the fields into the struct
    ackPacket_t ackData;
    pkt->pData = ReadBEInt16(pkt->pData, &g_receivedAckData.rxPacketRSSI);
    pkt->pData = ReadBEUint8(pkt->pData, &g_receivedAckData.seqNoToAck);     // The seq number they are ACKing.
    pkt->pData = ReadBEUint8(pkt->pData, &g_receivedAckData.ackResendCount); // The seq number of the packet we sent to them
    pkt->pData = ReadBEInt8(pkt->pData, &g_receivedAckData.rxPacketSNR);     // RSSI of the packet by them
    // If they sent us an ACK, that tells us how strong they received our original packet, so we can adjust TX strength.
    // Serial.printf("    We received ACK that said they received our packet at RSSI %d, SNR %d\n", g_receivedAckData.rxPacketRSSI, g_receivedAckData.rxPacketSNR);
    loraStuff_adjustTxPwr(g_receivedAckData.rxPacketRSSI);
    // TODO sequence number tracking/ACKing later
    // Serial.printf("    Received ACK for our seq no %d, they sent %d times\n", g_lastMachStateV1Header.seqNo, g_receivedAckData.ackResendCount);
    // Do not send reply in parser
#if PACKET_PRINT_TO_OLED
    // oledStuff_printAckPacket(pkt, &ackData); Too verbose
#endif // #if PACKET_PRINT_TO_OLED
    // Store in global variables, to ACK later
    memcpy(g_idToAckAck, pkt->sourceChipID, CHIPID_LEN_BYTES);
    g_receivedAckRSSI = pkt->rxRSSI;
    g_receivedAckSNR = pkt->rxSNR;
    // TODO trigger send of ACKACK packet
    g_shouldAckAck = true;
    Serial.printf("Parsed ack at %d\n", millis());
}

static void parseAckAckPacket(rxPacket_t *pkt)
{
    // Parse the bytes in the ackAck tag
    ackAckPacket_t ackAckData;
    pkt->pData = ReadBEInt16(pkt->pData, &ackAckData.rxPacketRSSI);
    pkt->pData = ReadBEInt16(pkt->pData, &ackAckData.ackPacketRSSI);
    pkt->pData = ReadBEInt8(pkt->pData, &ackAckData.rxPacketSNR);
    pkt->pData = ReadBEInt8(pkt->pData, &ackAckData.ackPacketSNR);
    pkt->pData = ReadBEUint8(pkt->pData, &ackAckData.seqNo);          // SeqNo of the original packet that they sent
    pkt->pData = ReadBEUint8(pkt->pData, &ackAckData.ackResendCount); // SeqNo of the ACK we sent to them

    // If they sent us an ackAck, that tells us how strong they received our ack packet, so we can adjust TX strength.
    // Serial.printf("    We received ackACK that said they received our ack packet at RSSI %d, SNR %d\n", ackAckData.ackPacketRSSI, ackAckData.ackPacketSNR);
    loraStuff_adjustTxPwr(ackAckData.ackPacketRSSI);
    // TODO sequence number tracking/ACKing later
    // Serial.printf("    Received ACKACK for our seq no %d, ackResendCount %d!\n", ackAckData.seqNo, ackAckData.ackResendCount);
    // Do not send reply in parser

#if PACKET_PRINT_TO_OLED
    // oledStuff_printAckAckPacket(pkt, &ackAckData); Too verbose
#endif // #if PACKET_PRINT_TO_OLED

    Serial.printf("Parsed ackAck at %d\n", millis());
}

static void parseMachstateV1Packet(rxPacket_t *pkt)
{
    // Parse the bytes in the ack tag
    machStateV1Packet_t data;
    // Read the fields into the struct
    pkt->pData = ReadBEUint8(pkt->pData, &data.seqNo);
    pkt->pData = ReadBEUint8(pkt->pData, &data.machState);
    g_lastV1PacketRx_ms = millis();
    Serial.printf("Rx Seq no %d, Machine state %d at %d\n", data.seqNo, data.machState, g_lastV1PacketRx_ms);
    globalInts_setMachineState((machineState_t)data.machState);

    // Print, act on, and ACK packet on Driver
#if PACKET_PRINT_TO_OLED
    g_lastMachStateV1Header = *pkt; // Copy the header into last received
    // oledStuff_printMachStateV1Packet(pkt, &data);
#endif // #if PACKET_PRINT_TO_OLED

    // Store data in global vars to trigger an ACK send
    memcpy(g_idToAck, pkt->sourceChipID, CHIPID_LEN_BYTES);
    g_shouldAckMachState = true;
}

rxPacket_t *packetParser_getLastMachStateV1Header(void)
{
    return &g_lastMachStateV1Header;
}

uint32_t packetParser_lastMachV1PacketTimestamp(void)
{
    return g_lastV1PacketRx_ms;
}

void packetParser_parseLoRaData(const uint8_t *pData, uint16_t dataLen, const int16_t rxRSSI, const int8_t rxSNR)
{
    if (dataLen < PACKET_HEADER_NUMBYTES || dataLen > MAX_PACKET_LEN)
    {
        Serial.printf(" parseTag called with INVALID len %d, IGNORING\n", dataLen);
        return;
    }
    // Start a struct for this packet's info
    rxPacket_t thisRXPkt;
    thisRXPkt.rxRSSI = rxRSSI;
    thisRXPkt.rxSNR = rxSNR;
    // Start the pData off right at the header, parse header, then parse packet depending on type
    thisRXPkt.pData = (uint8_t *)pData;
#if (1 == PACKET_HEADER_VERSION_NUM)
    // Dest ID[numBytes]. Don't need to store. Check and ignore if it's not for us
    // Ignore if it's not for us
    if (!isPacketForUs(thisRXPkt.pData))
    {
        return;
    }
    thisRXPkt.pData += CHIPID_LEN_BYTES;
    // Source ID[numBytes]
    memcpy(thisRXPkt.sourceChipID, thisRXPkt.pData, CHIPID_LEN_BYTES);
    thisRXPkt.pData += CHIPID_LEN_BYTES;
    // TX power, int8
    thisRXPkt.pData = ReadBEInt8(thisRXPkt.pData, &thisRXPkt.txdBm);
    // Type, uint8
    thisRXPkt.pData = ReadBEUint8(thisRXPkt.pData, &thisRXPkt.pktType);
    // done with header, can parse by packet type now
#else
#error "Define RX"
#endif // #if (1 == PACKET_HEADER_VERSION_NUM)
    // rxPacket.pData has now been advanced to start of packet data

    dataLen -= PACKET_HEADER_NUMBYTES; // Subtract the header bytes to let len equal the number of bytes remaining
    Serial.printf("packet type %d, RSSI %d, SNR %d, len %d: ", thisRXPkt.pktType, thisRXPkt.rxRSSI, thisRXPkt.rxSNR, dataLen);
    switch ((packetType_t)thisRXPkt.pktType)
    {
    case packetType_ack:
        // use savePacket, but don't save to g_lastReceivedPacket
        if (ACK_PKTLEN_BYTES != dataLen)
        {
            Serial.printf("    RX ACK error, len %d should be %d\n", dataLen, ACK_PKTLEN_BYTES);
            return; // Break out
        }
        parseAckPacket(&thisRXPkt); // Save to send AckAck if this is for us
        break;
    case packetType_ackAck:
        if (ACKACK_PKTLEN_BYTES != dataLen)
        {
            Serial.printf("    ERROR parsing ackAck tag, len %d should be %d\n", dataLen, ACKACK_PKTLEN_BYTES);
            return;
        }
        g_ackResendCnt = 0; // Zero this counter, since it ACKACKed our ACK
        parseAckAckPacket(&thisRXPkt);
        break;

    case packetType_machStateV1:
        if (MACHSTATE_V1_PKTLEN_BYTES != dataLen)
        {
            Serial.printf("  ERROR parsing machStateV1 packet len %d, should be %d\n", dataLen, MACHSTATE_V1_PKTLEN_BYTES);
            return;
        }
        parseMachstateV1Packet(&thisRXPkt);
        break;

    default:
        Serial.printf(" Don't know how to parse pktType of %d, len %d, ignoring\n", thisRXPkt.pktType, dataLen);
        break;
    }
    // return the number of bytes parsed
    return;
}

static uint8_t *populateHeader(uint8_t *pToFill, uint8_t *pDestID, packetType_t pktType)
{
#if (1 == PACKET_HEADER_VERSION_NUM)
    // Dest chip ID
    memcpy(pToFill, pDestID, CHIPID_LEN_BYTES);
    pToFill += CHIPID_LEN_BYTES;
    // Source chip ID (us)
    pToFill = writeU64ChipID(pToFill, globalInts_getChipIDU64());
    pToFill = WriteBEInt8(pToFill, loraStuff_getCurrentTxdBm());
    pToFill = WriteBEUint8(pToFill, (uint8_t)pktType);
    return pToFill;
#else
#error "Define packet header"
#endif // #if (1 == PACKET_HEADER_VERSION_NUM)
}

static void sendAckPacket(void)
{
    uint8_t txPacket[PACKET_HEADER_NUMBYTES + ACK_PKTLEN_BYTES];
    uint8_t *ptr = populateHeader(txPacket, g_idToAck, packetType_ack);
    // Now populate stuff unique to ACK packet
    // RSSI that we received their command at
    ptr = WriteBEInt16(ptr, g_lastMachStateV1Header.rxRSSI);
    // Set the seq number to ack.
    ptr = WriteBEUint8(ptr, g_lastMachStateV1Header.seqNo);
    // Set the seqNo of this ACK, incrementing every re-send
    ptr = WriteBEUint8(ptr, g_ackResendCnt++);
    // Set the SNR that we received machState at
    ptr = WriteBEInt8(ptr, g_lastMachStateV1Header.rxSNR);
    bool ret = enqueuePacket(txPacket, (uint32_t)(ptr - txPacket), packetType_ack);
    if (ret)
    {
        Serial.printf("Enqueued ack packet at %d\n", millis());
        // Serial.print(" to ");
        // printID(g_idToAck, false);
        // Serial.printf(", seq no %d, received with RSSI %d, SNR %d\n", g_lastMachStateV1Header.seqNo, g_lastMachStateV1Header.rxRSSI, g_lastMachStateV1Header.rxSNR);
    }
}

static void sendAckAckPacket(int16_t ackRxRSSI, int8_t ackRxSNR)
{
    uint8_t txPacket[PACKET_HEADER_NUMBYTES + ACKACK_PKTLEN_BYTES];
    uint8_t *ptr = populateHeader(txPacket, g_idToAckAck, packetType_ackAck);
    // Now we populate stuff unique to ackAck packet
    // RSSI that they received our original packet with
    ptr = WriteBEInt16(ptr, g_receivedAckData.rxPacketRSSI);
    // RSSI that we received their ACK with
    ptr = WriteBEInt16(ptr, ackRxRSSI);
    // SNR that they received our packet with
    ptr = WriteBEInt8(ptr, g_receivedAckData.rxPacketSNR);
    // SNR that we received their ACK with
    ptr = WriteBEInt8(ptr, ackRxSNR);
    // Set the seq number of the original packet.
    ptr = WriteBEUint8(ptr, g_receivedAckData.seqNoToAck);
    // Set the seq no of the ACK we received.
    ptr = WriteBEUint8(ptr, g_receivedAckData.ackResendCount);
    bool ret = enqueuePacket(txPacket, (uint32_t)(ptr - txPacket), packetType_ackAck);
    if (ret)
    {
        Serial.printf("Enqueued AckAck packet at %d\n", millis());
        // Serial.print(" to ");
        // printID(g_idToAckAck, false);
        // Serial.printf(", seqNoToAck %d, ACK retries %d, ACK RX RSSI %d, their RSSI %d\n", g_receivedAckData.seqNoToAck, g_receivedAckData.ackResendCount, ackRxRSSI, g_receivedAckData.rxPacketRSSI);
    }
}

bool packetParser_sendMachStateV1Packet(uint8_t machState, uint8_t *pDestID)
{
    uint8_t txPacket[PACKET_HEADER_NUMBYTES + MACHSTATE_V1_PKTLEN_BYTES];
    uint8_t *ptr = populateHeader(txPacket, pDestID, packetType_machStateV1);
    // Set our packet's bytes
    ptr = WriteBEUint8(ptr, g_txMachStateSeqNo++);
    ptr = WriteBEUint8(ptr, machState);
    bool ret = enqueuePacket(txPacket, (uint32_t)(ptr - txPacket), packetType_machStateV1);
    if (!ret)
    {
        Serial.printf("FAILED to enqueue machStateV1 packet!\n");
    }
    return ret;
}

uint8_t packetParser_getLastRxSeqNo(void)
{
    return g_lastMachStateV1Header.seqNo;
}

#if SEND_ERR_PRINT_ITVL_MS
static sendFail_t g_lastSendError;
static uint32_t g_lastSendErrPrint_ms;
#endif // #if SEND_ERR_PRINT_ITVL_MS
void packetParser_poll(void)
{
    // check if we should send ACK and send it here.
    if (g_shouldAckMachState)
    {
        sendAckPacket();
        g_shouldAckMachState = false;
    }
    if (g_shouldAckAck)
    {
        sendAckAckPacket(g_receivedAckRSSI, g_receivedAckSNR);
        g_shouldAckAck = false;
    }
    for (uint8_t i = 0; i < MAX_TX_PACKETS; i++)
    {
        bool bailOut = false;
        if (g_txSlots[i].size)
        { // Copy in the data, populate the length
            uint32_t sendStart_ms = millis();
            sendFail_t sendRet = loraStuff_send(g_txSlots[i].pBuffer, g_txSlots[i].size, g_txSlots[i].type);
            switch (sendRet)
            {
            case send_success:
            {
                // Serial.printf(" Just sent slot %d, length %d, at %d\n", i, g_txSlots[i].size, sendStart_ms);
                g_txSlots[i].size = 0;
                bailOut = true; // It will be in TX, wait
                return;
            }
            break;
            case sendFail_dataTooLong:
                // Serial.printf("Data too long error, throw out packet");
                g_txSlots[i].size = 0;
                bailOut = false; // try another
                break;
            default:
            {
                // Serial.printf("Unknown send error %d\n", sendRet);
                bailOut = true; // Maybe come back later
            }
            break;
            }
#if SEND_ERR_PRINT_ITVL_MS
            // if ((g_lastSendError != sendRet) || (utils_elapsedU32Ticks(g_lastSendErrPrint_ms, millis()) > SEND_ERR_PRINT_ITVL_MS))
            if (g_lastSendError != sendRet)
            {
                g_lastSendErrPrint_ms = millis();
                Serial.printf(" Slot %d, type %d, length %d sendRet %s at %d\n", i, g_txSlots[i].type, g_txSlots[i].size, g_sendErrors[sendRet], g_lastSendErrPrint_ms);
                g_lastSendError = sendRet;
            }
#endif // #if SEND_ERR_PRINT_ITVL_MS
        }
        if (bailOut)
        {
            return;
        }
    }
}

void packetParser_init(uint64_t ourChipIDU64)
{
    writeU64ChipID(g_ourChipID, ourChipIDU64);
    Serial.print("PacketParser init as 0x ");
    printID(g_ourChipID, true);
    g_txMachStateSeqNo = 0; // Only on Remote
    g_ackResendCnt = 0;
}