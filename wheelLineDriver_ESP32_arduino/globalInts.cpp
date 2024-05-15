/*
 * globalInts.c
 *
 *  Created on: Feb 10, 2024
 *      Author: Collin Moore
 */

#include "globalInts.h"

#include "Arduino.h"

#include "packetParser.h"
#include "utils.h"

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

/*************************************************************************************
 *  Variables
 ************************************************************************************/

// static int32_t m_int32s[GLOBALINTs_NUM_INT32s];
static machineState_t m_machState;
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
void globalInts_setMachineState(machineState_t st)
{
    if (st <= machState_killEngine)
    {
        if (st != m_machState)
        {
            Serial.printf("Machine state from %d to %d\n", m_machState, st);
        }
        m_machState = st;
    }
    else
    {
        Serial.printf("Tried to set invalid state %d, setting to kill engine\n", st);
        m_machState = machState_killEngine;
    }
}
