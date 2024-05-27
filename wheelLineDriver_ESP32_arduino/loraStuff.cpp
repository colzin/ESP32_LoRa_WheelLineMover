#include "loraStuff.h"
#include "pinStuff.h"

#include "Arduino.h"

#include "ESP32_LoRaWan_102.h"
// #include "radio/radio.h"
// #include "radio.h"
#include "driver/sx126x.h"

#include "packetParser.h"

#include <stdbool.h>
#include <stdint.h>
#include "utils.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define RF_FREQUENCY 915000000 // Hz for USA is 915M

// https://mylorawan.blogspot.com/2016/05/spread-factor-vs-payload-size-on-lora.html

// This is specced by the radio transceiver
#define MIN_TX_OUTPUT_POWER -9 //
#define MAX_TX_OUTPUT_POWER 21 // dBm

#define ALLOW_CHANGING_TX_POWER 1 // 1 to allow changing of our RX power
#if ALLOW_CHANGING_TX_POWER
#define OUR_TX_POWER MIN_TX_OUTPUT_POWER // Start here, can go up from here
#define MIN_RSSI -100                    // Try to turn up if receiver hears us weaker than this
#define MAX_RSSI -80                     // Try to turn down if receiver hears us stronger than this
#else
#define OUR_TX_POWER MAX_TX_OUTPUT_POWER // hard-coded setting
#endif                                   // #if ALLOW_CHANGING_TX_POWER

/* Each doubling of the bandwidth correlates to almost 3dB less link budget.
    More bandwidth means less range
*/
#define LORA_BANDWIDTH 0 // [0: 125 kHz, \
                                 //  1: 250 kHz, \
                                 //  2: 500 kHz, \
                                 //  3: Reserved]

/* Each step up in spreading factor doubles the time on air to transmit a symbol.
    Each unit increase in SF correlates to about 2.5dB extra link budget.
*/
#define LORA_SPREADING_FACTOR 8 // [SF7..SF12]

/* increasing the coding rate increases reliability while decreasing data rate.
 */
#define LORA_CODINGRATE 4            // [1: 4/5, \
                                 //  2: 4/6, \
                                 //  3: 4/7, \
                                 //  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8       // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0        // Symbols
#define LORA_CRC_ON 1                // 1 to enable, 0 to disable
#define LORA_FIX_LENGTH_PAYLOAD_ON 0 // 1 to enable, 0 to disable
#define LORA_FREQ_HOP_ON 0           // 1 to enable, 0 to disable
#define LORA_IQ_INVERSION_ON 0       // 1 to enable, 0 to disable
#define LORA_RX_CONTINUOUS false     // false for single mode, true for continuous RX
#define LORA_TX_TIMEOUT 300          // Transmission timeout [ms]

#define AWAIT_ACK_MS (110 + LORA_TX_TIMEOUT) // How long to wait for an ACK after a TX packet

#define MSEC_TO_RESET_RX 32000 // Probes should report every 30 sec

/*******************************************************************************
 * Variables
 ******************************************************************************/

static int16_t g_currentTxdBm;
static bool g_expectingReply;
static uint32_t g_expectingReplyStart_ms;
static uint32_t g_lastRx_ms, g_lastRadioReset_ms;
static int16_t rssi, rxSize;

static RadioEvents_t g_radioEvents;

// No LoRaWAN on these, they are simple to test with

/*******************************************************************************
 * Code
 ******************************************************************************/

static void setLoRaTxConfig(int32_t txPower)
{
    // make sure power is within limits of the chip.
    if (txPower > MAX_TX_OUTPUT_POWER)
    {
        txPower = MAX_TX_OUTPUT_POWER;
    }
    if (txPower < MIN_TX_OUTPUT_POWER)
    {
        txPower = MIN_TX_OUTPUT_POWER;
    }
    Radio.SetTxConfig(MODEM_LORA, txPower, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      LORA_CRC_ON, LORA_FREQ_HOP_ON, 0, LORA_IQ_INVERSION_ON, LORA_TX_TIMEOUT);
    g_currentTxdBm = txPower;
}

static void setLoRaConfig(int32_t txPower)
{
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, LORA_CRC_ON, LORA_FREQ_HOP_ON, 0, LORA_IQ_INVERSION_ON, LORA_RX_CONTINUOUS);

    setLoRaTxConfig(txPower);
}

static void onRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    g_expectingReply = false;
    Radio.Sleep();            // Sleep here, to stop Rx timers.
    pinStuff_setLED(led_off); // Turn off when not RX or TX
    Serial.printf("onRxDone, set expectingReply false, parsing then going to sleep at %d.\n", millis());
    // TODO offload parsing to later? For now, try to keep this function short.
    g_lastRx_ms = millis();
    packetParser_parseLoRaData(payload, size, rssi, snr);
}
static void onRxTimeout(void)
{
    if (g_expectingReply)
    { // If we didn't receive an ACK, max out our power
        // loraStuff_adjustTxPwr(MIN_RSSI - 40);
        g_expectingReply = false;
        // Serial.println("Set g_expectingReply false");
    }
    Radio.Sleep();            // Leave in sleep
    pinStuff_setLED(led_off); // Turn off when not RX or TX
    Serial.printf("RxTimeout, idling at %d\n", millis());
}
static void onTxDone(void)
{
    Radio.Sleep();
    pinStuff_setLED(led_off); // Turn off when done
    if (g_expectingReply)
    {
        Radio.Rx(AWAIT_ACK_MS);    // Rx for ACK
        pinStuff_setLED(led_weak); // Turn weak for RX
        Serial.printf("onTxDone, going to RX for %d at %d\n", AWAIT_ACK_MS, millis());
        g_expectingReplyStart_ms = millis();
    }
}
static void onTxTimeout(void)
{
    Radio.Sleep();
    pinStuff_setLED(led_off); // Turn off when done
    Serial.printf("onTxTimeout, going to sleep at %d\n", millis());
}

static void dumpRegs(uint16_t startAddr, uint16_t len)
{
    uint8_t regVals[len];
    Radio.ReadBuffer(startAddr, regVals, len);
    Serial.printf("    Radio registers from 0x%04x to 0x%04x:\n", startAddr, startAddr + len);
    for (uint16_t i = 0; i < len; i++)
    {
        Serial.printf("0x%04x, 0x%02x\n", startAddr + i, regVals[i]);
    }
}

void loraStuff_initRadio(void)
{
    g_radioEvents.TxDone = onTxDone;
    g_radioEvents.TxTimeout = onTxTimeout;
    g_radioEvents.RxDone = onRxDone;
    g_radioEvents.RxTimeout = onRxTimeout;
    Radio.Init(&g_radioEvents);
    Radio.SetChannel(RF_FREQUENCY);
#if ALLOW_CHANGING_TX_POWER
    setLoRaConfig(0);
#else
    setLoRaConfig(OUR_TX_POWER); // Set to hard-coded desired power
#endif // #if ALLOW_CHANGING_TX_POWER
    Radio.Sleep();
    pinStuff_setLED(led_off); // Turn off when not RX or TX
    g_expectingReply = false;
}

void loraStuff_setExpectingReply(bool isReplyExpected)
{ // Don't allow sending more packets while we await a reply
    // Serial.printf("Setting g_expectingReply to %d\n", isReplyExpected);
    g_expectingReply = isReplyExpected;
}

#define REG_DUMP_POLL_ITVL_MS 0
#if REG_DUMP_POLL_ITVL_MS
#define REG_DUMP_NUM_REGS 0xFF
uint32_t g_lastRegDump_ms;
uint16_t g_lastRegdumpAddr;
#include "utils.h"
#endif // #if REG_DUMP_POLL_ITVL_MS

void loraStuff_radioPoll(void)
{
    Radio.IrqProcess();
#if REG_DUMP_POLL_ITVL_MS
    if (utils_elapsedU32Ticks(g_lastRegDump_ms, millis()) > REG_DUMP_POLL_ITVL_MS)
    {
        g_lastRegdumpAddr += REG_DUMP_NUM_REGS;
        dumpRegs(g_lastRegdumpAddr, REG_DUMP_NUM_REGS);
        g_lastRegDump_ms = millis();
    }
#endif // #if REG_DUMP_POLL_ITVL_MS

    if (RF_IDLE == Radio.GetStatus())
    {
        Serial.println(" Detected radio idle, starting infinite RX until next packet");
        g_expectingReply = false;
        Radio.Rx(0);
        pinStuff_setLED(led_weak); // Turn weak for RX
    }
    uint32_t ms_now = millis();

    if (utils_elapsedU32Ticks(g_lastRx_ms, ms_now) > MSEC_TO_RESET_RX)
    {
        if (utils_elapsedU32Ticks(g_lastRadioReset_ms, ms_now) > MSEC_TO_RESET_RX)
        {
            RadioState_t preSleepState = Radio.GetStatus();
            Serial.printf("Radio status %d. Has been over %d ms since last rx. Sleeping then resetting Rx\n", preSleepState, MSEC_TO_RESET_RX);
            Radio.Sleep();
            pinStuff_setLED(led_off); // Turn off when not RX or TX
            RadioState_t sleepState = Radio.GetStatus();
            Radio.Rx(0);
            pinStuff_setLED(led_weak); // Turn weak for RX
            RadioState_t rxState = Radio.GetStatus();
            Serial.printf("State at start: %d. After sleep command: %d. After Rx: %d\n", preSleepState, sleepState, rxState);
            g_lastRadioReset_ms = ms_now;
        }
    }
}

int8_t loraStuff_getCurrentTxdBm(void)
{
    return g_currentTxdBm;
}

void loraStuff_adjustTxPwr(int16_t theirRSSI)
{
    int16_t oldTxdBm = g_currentTxdBm;
    int16_t newDesiredTxPower = g_currentTxdBm;
#if ALLOW_CHANGING_TX_POWER
    if (theirRSSI < MIN_RSSI)
    { // Increase our TX power if the receiver told use they have a low RSSI
        // If they are at -109 and we want them to be at -100, then increase by 9.
        // To find that, subtract actual - min, then invert, so (-109)-(-100)=-9, then invert
        newDesiredTxPower = g_currentTxdBm - (theirRSSI - MIN_RSSI);
    }
    if (theirRSSI > MAX_RSSI)
    { // Decrease our TX power if the receiver told use they have a high RSSI
        int16_t oldTxdBm = g_currentTxdBm;
        // If they are at -30 and we want them to be at -60, then decrease by 30
        // To find that, subtract actual - min, then invert, so (-30)-(-60)=+30, then invert
        newDesiredTxPower = g_currentTxdBm - (theirRSSI - MIN_RSSI);
    }
#else  // Set to hard-coded TX power
    newDesiredTxPower = OUR_TX_POWER;
#endif // #if ALLOW_CHANGING_TX_POWER
    // Make sure it's in the limits of the transmitter
    if (newDesiredTxPower > MAX_TX_OUTPUT_POWER)
    {
        newDesiredTxPower = MAX_TX_OUTPUT_POWER;
    }
    if (newDesiredTxPower < MIN_TX_OUTPUT_POWER)
    {
        newDesiredTxPower = MIN_TX_OUTPUT_POWER;
    }
    if (newDesiredTxPower != g_currentTxdBm)
    {
        // Serial.println("About to adjust TX power, standby radio:");
        // delay(1);
        // add current setting to get next desired setting, floor at -9 and ceiling at +22 for  1262
        Radio.Standby(); // Put radio into standby, then set TX power
        // Set power, which is an int8
        g_currentTxdBm = SX126xSetTxParams((int8_t)newDesiredTxPower, RADIO_RAMP_200_US);
        Serial.printf("     CHANGED OUR RADIO TX from %d to %d because their RSSI was %d\n", oldTxdBm, g_currentTxdBm, theirRSSI);
    }
}

sendFail_t loraStuff_send(uint8_t *txPtr, uint32_t len)
{
    if (len > 255)
    {
        Serial.printf("LoRa can only send 255 byte packets, can't send %d\n", len);
        return sendFail_dataTooLong;
    }
    if (utils_elapsedU32Ticks(g_lastRx_ms, millis()) < 100)
    {
        // Serial.printf("<100ms since RX, need to wait\n");
        return sendFail_needToWait;
    }
    if (g_expectingReply)
    {
        // Serial.println("Expecting reply, wait"); Happens all the time
        if (utils_elapsedU32Ticks(g_expectingReplyStart_ms, millis()) < AWAIT_ACK_MS)
        {
            return sendFail_awaitingReply;
        }
        else
        {
            Serial.printf("Stopping Rx at %d, timed out waiting\n", millis());
            loraStuff_adjustTxPwr(MIN_RSSI - 10); // Go up by 10dBm
            Radio.Sleep();
            pinStuff_setLED(led_off); // Turn off when not RX or TX
            g_expectingReply = false;
        }
    }
    RadioState_t radioState = Radio.GetStatus();
    switch (radioState)
    {
    case RF_IDLE:
        pinStuff_setLED(led_on); // Turn on for TX
        Radio.Send(txPtr, len);
        // Serial.printf("  Sent %d bytes, %d\n", len, millis());
        return send_success;
        break;
    case RF_TX_RUNNING:
        // Don't print error, this is fine, retry later.
        Serial.println("TX already running");
        return sendFail_txAlreadyRunning;
        break;
    case RF_RX_RUNNING:
        // Check above if we are awaiting a reply, no matter what mode we are in at the moment.
        // If here, we know we are not awaiting a reply, so we can change modes.
        Serial.printf("  Sleeping radio to send\n");
        Radio.Sleep();
        pinStuff_setLED(led_off); // Turn off when not RX or TX
        radioState = Radio.GetStatus();
        switch (radioState)
        {
        case RF_IDLE:
            pinStuff_setLED(led_on); // Turn on for TX
            Radio.Send(txPtr, len);
            Serial.printf("   Sent %d bytes, %d\n", len, millis());
            return send_success;
            break;
        default:
            Serial.printf("   Radio still in state %d, return false\n", radioState);
            return sendFail_error;
            break;
        }
        break;
    default:
        Serial.printf("  RadioState is %d, can't send now\n", radioState);
        return sendFail_inOtherState;
        break;
    }
    Serial.println("SendFail_ERROR?");
    return sendFail_error;
}
