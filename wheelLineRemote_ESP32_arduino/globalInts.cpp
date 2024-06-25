/*
 * globalInts.c
 *
 *  Created on: Feb 10, 2024
 *      Author: Collin Moore
 */

#include "globalInts.h"

#include "Arduino.h"

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

/*************************************************************************************
 *  Variables
 ************************************************************************************/

// static int32_t m_int32s[GLOBALINTs_NUM_INT32s];
static machineState_t m_machState;
static uint32_t m_machStateStart_ms;
const char *m_machineStateStrings[] = {"PON", "Start", "runIdle", "runFWD", "runREV", "kill"};

static uint64_t g_chipID;

static int8_t g_numRotations;
/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

// int32_t globalInts_getInt32(globalInt32s_t toGet)
//{
//     return m_int32s[toGet];
// }
// void globalInts_setIntInt32(globalInt32s_t toSet, int32_t val)
//{
//     m_int32s[toSet] = val;
// }
machineState_t globalInts_getMachineState(void)
{
    return m_machState;
}

const char *globalInts_getMachStateString(machineState_t st)
{
    if (st <= machState_killEngine)
    {
        return m_machineStateStrings[st];
    }
    return "invalid";
}

void globalInts_setMachineState(machineState_t st)
{
    if (st <= machState_killEngine)
    {
        if (st != m_machState)
        {
            Serial.printf("Machine state from %s to %s\n", globalInts_getMachStateString(m_machState), globalInts_getMachStateString(st));
        }
        if (machState_runEngineHydFwd != st && machState_runEngineHydRev != st)
        {
            Serial.println("Clearing numRots for machine state that shouldn't move.");
            globalInts_setNumRotations(0);
        }
        m_machState = st;
    }
    else
    {
        Serial.printf("Tried to set invalid state %d, setting to killEngine\n", st);
        m_machState = machState_killEngine;
    }
    m_machStateStart_ms = millis();
}

uint32_t globalInts_getMachStateStart_ms(void)
{
    return m_machStateStart_ms;
}

uint64_t globalInts_getChipIDU64(void)
{
    return g_chipID;
}
void globalInts_setChipIDU64(uint64_t chipID)
{
    g_chipID = chipID;
}

int8_t globalInts_getNumRotations(void)
{
    return g_numRotations;
}
void globalInts_setNumRotations(int8_t num)
{
    if (num != g_numRotations)
    {
        Serial.printf("\tChanged revs from %d to %d\n", g_numRotations, num);
    }
    g_numRotations = num;
}
