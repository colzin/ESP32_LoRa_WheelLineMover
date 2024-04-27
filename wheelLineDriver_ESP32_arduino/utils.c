#include <string.h>
#include "utils.h"

// Handy overlays

typedef union longDataOverlay
{
    int32_t s32;
    uint32_t u32;
} longDataOverlay;

typedef union longLongDataOverlay
{
    int64_t s64;
    uint64_t u64;
} longLongDataOverlay;

typedef union wordDataOverlay
{
    int16_t s16;
    uint16_t u16;
} wordDataOverlay;

typedef union byteDataOverlay
{
    int8_t s8;
    uint8_t u8;
} byteDataOverlay;

typedef union floatDataOverlay
{
    float f;
    uint32_t u32;
} floatDataOverlay;

typedef union doubleDataOverlay
{
    double d;
    uint64_t u64;
} doubleDataOverlay;

uint8_t* WriteLEUint32(uint8_t* pBuffer, uint32_t data)
{
    *pBuffer++ = (uint8_t)(data & 0xFF);
    *pBuffer++ = (uint8_t)((data >> 8) & 0xFF);
    *pBuffer++ = (uint8_t)((data >> 16) & 0xFF);
    *pBuffer++ = (uint8_t)((data >> 24) & 0xFF);
    return pBuffer;
}

uint8_t* WriteLEUint16(uint8_t* pBuffer, uint16_t data)
{
    *pBuffer++ = (uint8_t)(data & 0xFF);
    *pBuffer++ = (uint8_t)((data >> 8) & 0xFF);
    return pBuffer;
}

uint8_t* ReadBEUint32(uint8_t* pBuffer, uint32_t* data)
{
    unsigned int i;
    *data = 0;
    for (i = 0; i < sizeof(uint32_t); i++)
    {
        *data <<= 8;
        *data += (uint32_t)*pBuffer++;
    }
    return pBuffer;
}

uint8_t* WriteBEUint32(uint8_t* pBuffer, uint32_t data)
{
    *pBuffer++ = (uint8_t)((data >> 24) & 0xFF);
    *pBuffer++ = (uint8_t)((data >> 16) & 0xFF);
    *pBuffer++ = (uint8_t)((data >> 8) & 0xFF);
    *pBuffer++ = (uint8_t)(data & 0xFF);
    return pBuffer;
}

uint8_t* ReadBEInt32(uint8_t* pBuffer, int32_t* data)
{
    longDataOverlay d;
    pBuffer = ReadBEUint32(pBuffer, &d.u32);
    *data = d.s32;
    return pBuffer;
}

uint8_t* WriteBEInt32(uint8_t* pBuffer, int32_t data)
{
    longDataOverlay d;
    d.s32 = data;
    return WriteBEUint32(pBuffer, d.u32);
}

uint8_t* ReadBEUint16(uint8_t* pBuffer, uint16_t* data)
{
    unsigned int i;
    *data = 0;
    for (i = 0; i < sizeof(uint16_t); i++)
    {
        *data = (uint16_t)(*data << 8);
        *data = (uint16_t)(*data + *pBuffer);
        pBuffer++;
    }
    return pBuffer;
}

uint8_t* WriteBEUint16(uint8_t* pBuffer, uint16_t data)
{
    *pBuffer++ = (uint8_t)((data >> 8) & 0xFF);
    *pBuffer++ = (uint8_t)(data & 0xFF);
    return pBuffer;
}

uint8_t* ReadBEInt16(uint8_t* pBuffer, int16_t* data)
{
    wordDataOverlay d;
    pBuffer = ReadBEUint16(pBuffer, &d.u16);
    *data = d.s16;
    return pBuffer;
}

uint8_t* WriteBEInt16(uint8_t* pBuffer, int16_t data)
{
    wordDataOverlay d;
    d.s16 = data;
    return WriteBEUint16(pBuffer, d.u16);
}

uint8_t* WriteBEUint8(uint8_t* pBuffer, uint8_t data)
{
    *pBuffer++ = data;
    return pBuffer;
}

uint8_t* ReadBEUint8(uint8_t* pBuffer, uint8_t* data)
{
    *data = *pBuffer++;
    return pBuffer;
}

uint8_t* WriteBEInt8(uint8_t* pBuffer, int8_t data)
{
    byteDataOverlay d;
    d.s8 = data;
    return WriteBEUint8(pBuffer, d.u8);
}

uint8_t* ReadBEInt8(uint8_t* pBuffer, int8_t* data)
{
    *data = (int8_t)*pBuffer++;
    return pBuffer;
}

uint8_t* WriteBEUint24(uint8_t* pBuffer, uint32_t data)
{
    *pBuffer++ = (uint8_t)((data >> 16) & 0xFF);
    *pBuffer++ = (uint8_t)((data >> 8) & 0xFF);
    *pBuffer++ = (uint8_t)(data & 0xFF);
    return pBuffer;
}

uint8_t* ReadBEUint24(uint8_t* pBuffer, uint32_t* data)
{
    unsigned int i;
    *data = 0;
    for (i = 0; i < 3; ++i)
    {
        *data <<= 8;
        *data += (uint32_t)*pBuffer++;
    }
    return pBuffer;
}

uint8_t* WriteBEUint64(uint8_t* pBuffer, uint64_t data)
{
    *pBuffer++ = (uint8_t)((data >> 56) & 0xFF);
    *pBuffer++ = (uint8_t)((data >> 48) & 0xFF);
    *pBuffer++ = (uint8_t)((data >> 40) & 0xFF);
    *pBuffer++ = (uint8_t)((data >> 32) & 0xFF);
    *pBuffer++ = (uint8_t)((data >> 24) & 0xFF);
    *pBuffer++ = (uint8_t)((data >> 16) & 0xFF);
    *pBuffer++ = (uint8_t)((data >> 8) & 0xFF);
    *pBuffer++ = (uint8_t)(data & 0xFF);
    return pBuffer;
}

uint8_t* ReadBEUint64(uint8_t* pBuffer, uint64_t* data)
{
    unsigned int i;
    *data = 0;
    for (i = 0; i < sizeof(uint64_t); ++i)
    {
        *data <<= 8;
        *data += (uint64_t)*pBuffer++;
    }
    return pBuffer;
}

uint8_t* WriteBEInt64(uint8_t* pBuffer, int64_t data)
{
    longLongDataOverlay d;
    d.s64 = data;
    return WriteBEUint64(pBuffer, d.u64);
}

uint8_t* ReadBEInt64(uint8_t* pBuffer, int64_t* data)
{
    longLongDataOverlay d;
    pBuffer = ReadBEUint64(pBuffer, &d.u64);
    *data = d.s64;
    return pBuffer;
}

uint8_t* ReadBEFloat32(uint8_t* pBuffer, float* data)
{
    floatDataOverlay d;
    pBuffer = ReadBEUint32(pBuffer, &d.u32);
    *data = d.f;
    return pBuffer;
}

uint8_t* WriteBEFloat32(uint8_t* pBuffer, float data)
{
    floatDataOverlay d;
    d.f = data;
    return WriteBEUint32(pBuffer, d.u32);
}

uint8_t* ReadBEDouble64(uint8_t* pBuffer, double* data)
{
    doubleDataOverlay d;
    pBuffer = ReadBEUint64(pBuffer, &d.u64);
    *data = d.d;
    return pBuffer;
}

uint8_t* WriteBEDouble64(uint8_t* pBuffer, double data)
{
    doubleDataOverlay d;
    d.d = data;
    return WriteBEUint64(pBuffer, d.u64);
}

uint8_t* WriteBEData(uint8_t* pBuffer, uint8_t* data, uint32_t length)
{
    unsigned int i;
    for (i = 0; i < length; ++i)
    {
        *pBuffer++ = *data++;
    }
    return (pBuffer);
}

uint8_t* ReadBEData(uint8_t* pBuffer, uint8_t* data, uint32_t length)
{
    unsigned int i;
    for (i = 0; i < length; ++i)
    {
        *data++ = *pBuffer++;
    }
    return (pBuffer);
}

uint32_t utils_elapsedU32Ticks(uint32_t timestart, uint32_t currtime)
{
    if (currtime >= timestart)
    {
        return (currtime - timestart);
    }
    return (currtime + ((uint32_t)(-1) - timestart));
}

uint8_t utils_calcCarryChk(const uint8_t* const data, uint32_t len)
{
    int16_t checkCarry = 0;
    uint8_t sum = 0;
    uint32_t i;
    for (i = 0; i < len; ++i)
    {
        checkCarry = sum;
        checkCarry = (int16_t)(checkCarry + data[i]);
        sum = (uint8_t)(sum + data[i]);
        if (checkCarry & 0xFF00)
        {
            sum++;
        }
    }
    return sum;
}

int32_t utils_leastSquaresSlope(int32_t* data, int32_t numData, int32_t dataMultiplier)
{
    int32_t x;
    int32_t sumX = 0;
    int32_t sumY = 0;

    int32_t Sxy = 0;
    int32_t Sxx = 0;
    int32_t xMean = 0;
    int32_t yMean = 0;
    int32_t slope;

    for (x = 0; x < numData; x++)
    {
        data[x] *= dataMultiplier;
        sumY += data[x];
        sumX += x;
    }

    xMean = sumX / numData;
    yMean = sumY / numData;

    for (x = 0; x < numData; x++)
    {
        Sxy += (x - xMean) * (data[x] - yMean);
        Sxx += (x - xMean) * (x - xMean);
    }

    slope = Sxy / Sxx;

    return slope;
}

uint64_t utils_BEMacToUint64(const uint8_t* const macValPtr)
{
    uint64_t ret = 0;
    for (uint32_t macByteIndex = 0; macByteIndex < 6; macByteIndex++)
    { // Store MAC in big-endian in uint64, right-justified
        ret |= macValPtr[macByteIndex];
        if (macByteIndex < 5)
        {
            ret <<= 8;
        }
    }
    return ret;
}


// turns a base and exponent into a multiplier, multiplies the base by that, and returns the value.
int32_t utils_getScaledValue(int32_t value, int32_t base, int32_t exponent)
{
    if (exponent)
    {
        if (exponent > 0) // if the scaler is positive, then we will multiply the base by 10^exp to make a multiplier
        {
            for (int32_t i = 0; i < exponent; i++)
            {
                value *= base;
//                NRF_LOG_DEBUG(
//                              "Exponent was %d, Iteration %d, value is now %d\n",
//                              exponent,
//                              i,
//                              value);
            }
        }
        else
        { // if the scaler is negative, we will divide the base by 10^exp to make a multiplier for the data.
            for (int32_t i = 0; i > exponent; i--)
            {
                value /= base;
//                NRF_LOG_DEBUG(
//                              "Exponent was %d, Iteration %d, value is now %d\n",
//                              exponent,
//                              i,
//                              value);
            }
        }
    }
//    NRF_LOG_DEBUG("Value is now %d\n", value);
    return value;
}


int32_t utils_getFracPart(double val, int n)
{
  return (int32_t)((val - (int32_t)(val)) * pow(10, n));
}