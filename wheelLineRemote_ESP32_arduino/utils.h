#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // little endian (LE)
    uint8_t *WriteLEUint32(uint8_t *pBuffer, uint32_t data);
    uint8_t *WriteLEUint16(uint8_t *pBuffer, uint16_t data);

    // big endian (BE)
    uint8_t *ReadBEUint32(uint8_t *pBuffer, uint32_t *data);
    uint8_t *WriteBEUint32(uint8_t *pBuffer, uint32_t data);
    uint8_t *ReadBEInt32(uint8_t *pBuffer, int32_t *data);
    uint8_t *WriteBEInt32(uint8_t *pBuffer, int32_t data);

    uint8_t *ReadBEUint16(uint8_t *pBuffer, uint16_t *data);
    uint8_t *WriteBEUint16(uint8_t *pBuffer, uint16_t data);
    uint8_t *ReadBEInt16(uint8_t *pBuffer, int16_t *data);
    uint8_t *WriteBEInt16(uint8_t *pBuffer, int16_t data);

    uint8_t *WriteBEUint8(uint8_t *pBuffer, uint8_t data);
    uint8_t *ReadBEUint8(uint8_t *pBuffer, uint8_t *data);
    uint8_t *WriteBEInt8(uint8_t *pBuffer, int8_t data);
    uint8_t *ReadBEInt8(uint8_t *pBuffer, int8_t *data);

    uint8_t *WriteBEUint24(uint8_t *pBuffer, uint32_t data);
    uint8_t *ReadBEUint24(uint8_t *pBuffer, uint32_t *data);

    uint8_t *WriteBEUint64(uint8_t *pBuffer, uint64_t data);
    uint8_t *ReadBEUint64(uint8_t *pBuffer, uint64_t *data);
    uint8_t *WriteBEInt64(uint8_t *pBuffer, int64_t data);
    uint8_t *ReadBEInt64(uint8_t *pBuffer, int64_t *data);

    uint8_t *ReadBEFloat32(uint8_t *pBuffer, float *data);
    uint8_t *WriteBEFloat32(uint8_t *pBuffer, float data);

    uint8_t *ReadBEDouble64(uint8_t *pBuffer, double *data);
    uint8_t *WriteBEDouble64(uint8_t *pBuffer, double data);

    uint8_t *WriteBEData(uint8_t *pBuffer, uint8_t *data, uint32_t length);
    uint8_t *ReadBEData(uint8_t *pBuffer, uint8_t *data, uint32_t length);

    uint32_t utils_elapsedU32Ticks(uint32_t timestart, uint32_t currtime);

    uint8_t utils_calcCarryChk(const uint8_t *const data, uint32_t len);

    int32_t utils_leastSquaresSlope(int32_t *data, int32_t numData, int32_t dataMultiplier);

    uint64_t utils_BEMacToUint64(const uint8_t *const macValPtr);

    int32_t utils_getScaledValue(int32_t value, int32_t base, int32_t exponent);

    int32_t utils_getFracPart(double val, int n);

#ifdef __cplusplus
}
#endif

#endif /* UTILS_H_ */
