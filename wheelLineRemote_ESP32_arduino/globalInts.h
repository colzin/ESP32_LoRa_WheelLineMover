/*
 * globalInts.h
 *
 *  Created on: Feb 10, 2024
 *      Author: Collin Moore
 */

#ifndef SRC_GLOBALINTS_H_
#define SRC_GLOBALINTS_H_

#include <stdint.h>

typedef enum
{
    machState_justPoweredOn = 0,
    machState_startEngine,
    machState_runEngineHydIdle,
    machState_runEngineHydFwd,
    machState_runEngineHydRev,
    // TODO add any states here
    machState_killEngine // Keep this one last, to sanity check enum value on sets

} machineState_t;

machineState_t globalInts_getMachineState(void);
const char* globalInts_getMachStateString(machineState_t st);
void globalInts_setMachineState(machineState_t st);
uint32_t globalInts_getMachStateStart_ms(void);

uint64_t globalInts_getChipIDU64(void);
void globalInts_setChipIDU64(uint64_t chipID);

int8_t globalInts_getNumRotations(void);
void globalInts_setNumRotations(int8_t num);

#endif /* SRC_GLOBALINTS_H_ */
