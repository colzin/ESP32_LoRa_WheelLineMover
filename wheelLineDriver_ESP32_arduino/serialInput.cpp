#include "serialInput.h"
#include "defines.h"

#include "Arduino.h"

#include "globalInts.h"
#include "loraStuff.h"
#include "pinStuff.h"
#include "tagDefinitions.h"

#include "utils.h"

#include <string.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define LF_CHAR 0xA
#define CR_CHAR 0xD

typedef enum
{
    parserState_default,
    parserState_receivingPower,
} parserState_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

// Parser
static parserState_t m_parserState;
static int32_t m_accumulator;
static bool m_negative;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

static void parsePowerdigit(uint8_t byte)
{
    if ('-' == byte)
    {
        m_negative = true;
    }
    else if ('0' <= byte && '9' >= byte)
    {
        m_accumulator *= 10;
        m_accumulator += (byte - '0');
    }
    else if (LF_CHAR == byte || CR_CHAR == byte)
    {
        if (m_negative)
        {
            m_accumulator = -1 * m_accumulator;
        }
        // cc1101_setOutputPower((int8_t)m_accumulator);
        Serial.printf("TODO set TX power to %d\n", m_accumulator);
        m_parserState = parserState_default;
    }
    else
    {
        Serial.printf("Error, moving to default\n");
        m_parserState = parserState_default;
    }
}

static void defaultParser(uint8_t byte)
{
    switch (byte)
    {
    case 'c':
    {
        Serial.printf(" running on core %d\n", xPortGetCoreID());
        Serial.printf("   Getting taskList:\n");
        // char taskStats[40 * 10];
        // vTaskList(taskStats); Can't seem to use this function https://github.com/espressif/arduino-esp32/issues/1089
        // Serial.printf("%s\n", taskStats);
        // Serial.printf("   Getting current tasks state:\n");
        // uxTaskGetSystemState()
        Serial.printf("There are %d tasks running\n", uxTaskGetNumberOfTasks());
    }
    break;

    case 'i':
        Serial.printf(" TODO Setting LoRa to IDLE state\n");
        break;
    case 'o':
        Serial.printf("Setting engine ON idle mode\n");
        globalInts_setMachineState(machState_runEngineHydIdle);
        break;
    case 'r':
        Serial.println("Rebooting in 3ms:");
        delay(3);
        ESP.restart();
        break;
    case 't':
    case 'T':
        Serial.println("Enter TX power");
        m_accumulator = 0;
        m_negative = false;
        m_parserState = parserState_receivingPower;
        break;
    case LF_CHAR:
        break;
    case CR_CHAR:
        break;
    default:
        Serial.println("Enter i, o, p, r, t");
        break;
    }
}

void serialInput_poll(void)
{
    while (Serial.available())
    {
        uint8_t rxByte = Serial.read();
        Serial.printf("Received 0x%x\n", rxByte);
        switch (m_parserState)
        {
        case parserState_default:
            defaultParser(rxByte);
            break;
        case parserState_receivingPower:
            parsePowerdigit(rxByte);
            break;
        }
    }
}
