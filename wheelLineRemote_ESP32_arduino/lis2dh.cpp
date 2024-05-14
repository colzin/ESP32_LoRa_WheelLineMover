/*
 * lis2dh.c
 *
 *  Created on: Jan 22, 2024
 *      Author: Collin Moore
 */

#include "lis2dh.h"

#include "Arduino.h" // for Serial, millis, all Arduino functions
#include "i2c1.h"

#include "utils.h"

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

/* Register definitions taken from
 * https://github.com/STMicroelectronics/lis2dh12-pid/blob/master/lis2dh12_reg.h
 * AND
 * lis2dh12.pdf datasheet, rev 6
 */

/** I2C Device Address 8 bit format if SA0=0 -> 31 if SA0=1 -> 33 **/
#define LIS2DH12_I2C_ADD_L 0x31U
#define LIS2DH12_I2C_ADD_H 0x33U

// See datasheet 6.1.1 - I2C Operation
#define AUTO_INCREMENT_MASK 0x80 // If this mask is applied, the I2C slave auto-increments after a write or read.

#define STATUS_REG_AUX_REGADDR 0x07U
// b7 not used
#define STATUS_REG_AUX_TEMP_OVERRUN 0x04
// b5:3 not used
#define STATUS_REG_AUX_TEMP_AVAIL 0x04;
// b1:0 not used

#define OUT_TEMP_L_REGADDR 0x0CU

#define OUT_TEMP_H_REGADDR 0x0DU

#define WHO_AM_I_REGADDR 0x0FU
#define WHO_AM_I_VALUE 0x33U

#define CTRL_REG0_REGADDR 0x1EU
#define CTRL_REG0_DISABLE_SDO_PU 0x80
#define CTRL_REG0_B6_0_VALUE 0x10 // MUST keep these bits at these values

#define TEMP_CFG_REGADDR 0x1FU
#define TEMP_CFG_TEMP_ENABLED 0xC0 // 0b11 for temp enabled, 0b00 for disabled
// b5:0 unused

#define CTRL_REG1_REGADDR 0x20U
#define CTRL_REG1_ODR(x) ((x << 4) & 0xF0) // b7:4 are ODR 3:0
typedef enum
{
    odr_powerDown = 0,
    odr_1Hz,
    odr_10Hz,
    odr_25Hz,
    odr_50Hz,
    odr_100Hz,
    odr_200Hz,
    odr_400Hz,
    odr_lowpower_1620Hz,
    odr_norm_hr_1344Hz_lowpwr_5376Hz,
    // states 10-15 not used
} ctrl1_odr_t;
#define CTRL_REG1_LPEN 0x08 // low-power enabled if set, else normal mode
#define CTRL_REG1_Z_EN 0x04
#define CTRL_REG1_Y_EN 0x02
#define CTRL_REG1_X_EN 0x01

#define CTRL_REG2_REGADDR 0x21U
#define CTRL_REG2_HPM(x) ((x << 6) & 0xC0) // b7:6 are HPM
typedef enum
{
    highPass_normalMode = 0,
    highPass_referenceForFiltering,
    highPass_normalModeAlso,
    highPass_autoresetOnInt,
} ctrl2_hpm_t;
#define CTRL_REG2_HPCF(x) ((x << 4) & 0x30) // b5:4 are HPCF
typedef enum
{
    highpassCutoff_most = 0,
    highpassCutoff_more,
    highpassCutoff_some,
    highpassCutoff_less,
} ctrl2_hpcf_t;

#define CTRL_REG2_FILTERED_DATA 0x08
#define CTRL_REG2_HP_CLICK 0x04
#define CTRL_REG2_HP_IA2 0x02
#define CTRL_REG2_HP_IA1 0x01

#define CTRL_REG3_REGADDR 0x22U
#define CTRL_REG3_I1_CLICK 0x80
#define CTRL_REG3_I1_IA1 0x40
#define CTRL_REG3_I1_IA2 0x20
#define CTRL_REG3_I1_ZYXDA 0x10
// b3 must be zero
#define CTRL_REG3_I1_WTM 0x04
#define CTRL_REG3_I1_OVER 0x02
// b0 unused

#define CTRL_REG4_REGADDR 0x23U
#define CTRL_REG4_BLOCKDATAUPDATE 0x80
#define CTRL_REG4_DATALITTLEENDIAN 0x40            // little endian if set, big if clear
#define CTRL_REG4_FULLSCALE(x) (((x) << 4) & 0x30) // b5:4 is fs 1:0
typedef enum
{
    fullScale_2g = 0,
    fullScale_4g,
    fullScale_8g,
    fullScale_16g
} ctrl_reg4_fullscale_t;
#define CTRL_REG4_HIGHRES 0x10
#define CTRL_REG4_SELFTST(x) ((x << 2) & 0x0C) // st 1:0
typedef enum
{
    selfTest_disabled = 0,
    selfTest_0,
    selfTest_1,
    selfTest_NA,
} ctrl_reg4_st_t;               // See lis2dh12.pdf table 39
#define CTRL_REG4_SPI3WIRE 0x01 // 4-wire if 0

#define CTRL_REG5_REGADDR 0x24U
#define CTRL_REG5_MEM_REBOOT 0x80
#define CTRL_REG5_FIFO_EN 0x40
// b5, b44 unused
#define CTRL_REG5_LIR_INT1 0x08
#define CTRL_REG5_D4D_INT1 0x04
#define CTRL_REG5_LIR_INT2 0x02
#define CTRL_REG5_D4D_INT2 0x01

#define CTRL_REG6_REGADDR 0x25U
#define CTRL_REG6_I2_CLICK 0x80
#define CTRL_REG6_I2_IA1 0x40
#define CTRL_REG6_I2_IA2 0x20
#define CTRL_REG6_I2_BOOT 0x10
#define CTRL_REG6_I2_ACT 0x08
// b2 unused
#define CTRL_REG6_INT_POLARITY 0x02
// b0 unused

#define REFERENCE_REGADDR 0x26U
// Reference value for interrupt

#define STATUS_REGADDR 0x27U
#define STATUS_REG_XYZ_OVERRUN 0x80
#define STATUS_REG_Z_OVERRUN 0x40
#define STATUS_REG_Y_OVERRUN 0x20
#define STATUS_REG_X_OVERRUN 0x10
#define STATUS_REG_XYZ_NEWDATA 0x80
#define STATUS_REG_Z_NEWDATA 0x40
#define STATUS_REG_Y_NEWDATA 0x20
#define STATUS_REG_X_NEWDATA 0x10

#define OUT_X_L_REGADDR 0x28U
#define OUT_X_H_REGADDR 0x29U
#define OUT_Y_L_REGADDR 0x2AU
#define OUT_Y_H_REGADDR 0x2BU
#define OUT_Z_L_REGADDR 0x2CU
#define OUT_Z_H_REGADDR 0x2DU

#define FIFO_CTRL_REGADDR 0x2EU
#define FIFO_CTRL_MODE(x) (((x) << 6) & 0xC0)
typedef enum
{
    fifoMode_bypass = 0,
    fifoMode_fifo,
    fifoMode_stream,
    fifoMode_streamToFifo,
} fifoCtrlMode_t;
#define FIFO_CTRL_TRIG_INT2 0x20 // 0 for trigger signal on INT1
#define FIFO_CTRL_THR(x) ((x) & 0x1F)

#define FIFO_SRC_REGADDR 0x2FU
#define FIFO_SRC_WTM 0x80
#define FIFO_SRC_OVRN 0x40
#define FIFO_SRC_EMPTY 0x20
#define FIFO_SRC_NUMUNREAD(x) ((x) & 0x1F)

#define INT1_CFG_REGADDR 0x30U
#define INT1_CFG_AOI 0x80
#define INT1_CFG_6D 0x40
#define INT1_CFG_ZHIE 0x20
#define INT1_CFG_ZLIE 0x10
#define INT1_CFG_YHIE 0x08
#define INT1_CFG_YLIE 0x04
#define INT1_CFG_XHIE 0x02
#define INT1_CFG_XLIE 0x01

#define INT1_SRC_REGADDR 0x31U
// b7 not used
#define INT1_SRC_IA 0x40
#define INT1_SRC_ZH 0x20
#define INT1_SRC_ZL 0x10
#define INT1_SRC_YH 0x08
#define INT1_SRC_YL 0x04
#define INT1_SRC_XH 0x02
#define INT1_SRC_XL 0x01

#define INT1_THS_REGADDR 0x32U
// b7 not used
#define INT1_THS_THS(x) ((x) & 0x7F)

#define INT1_DURATION_REGADDR 0x33U
// b7 not used
#define INT1_DURATION(x) ((x) & 0x7F)

#define INT2_CFG_REGADDR 0x34U
#define INT2_CFG_AOI 0x80
#define INT2_CFG_6D 0x40
#define INT2_CFG_ZHIE 0x20
#define INT2_CFG_ZLIE 0x10
#define INT2_CFG_YHIE 0x08
#define INT2_CFG_YLIE 0x04
#define INT2_CFG_XHIE 0x02
#define INT2_CFG_XLIE 0x01

#define INT2_SRC_REGADDR 0x35U
// b7 not used
#define INT2_SRC_IA 0x40
#define INT2_SRC_ZH 0x20
#define INT2_SRC_ZL 0x10
#define INT2_SRC_YH 0x08
#define INT2_SRC_YL 0x04
#define INT2_SRC_XH 0x02
#define INT2_SRC_XL 0x01

#define INT2_THS_REGADDR 0x36U
// b7 not used
#define INT2_THS_THS(x) ((x) & 0x7F)

#define INT2_DURATION_REGADDR 0x37U
// b7 not used
#define INT2_DURATION(x) ((x) & 0x7F)

#define CLICK_CFG_REGADDR 0x38U
// b7,6 not used
#define CLICK_CFG_ZDOUBLE 0x20
#define CLICK_CFG_ZSINGLE 0x10
#define CLICK_CFG_YDOUBLE 0x08
#define CLICK_CFG_YSINGLE 0x04
#define CLICK_CFG_XDOUBLE 0x02
#define CLICK_CFG_XSINGLE 0x01

#define CLICK_SRC_REGADDR 0x39U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
    uint8_t x : 1;
    uint8_t y : 1;
    uint8_t z : 1;
    uint8_t sign : 1;
    uint8_t sclick : 1;
    uint8_t dclick : 1;
    uint8_t ia : 1;
    uint8_t not_used_01 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
    uint8_t not_used_01 : 1;
    uint8_t ia : 1;
    uint8_t dclick : 1;
    uint8_t sclick : 1;
    uint8_t sign : 1;
    uint8_t z : 1;
    uint8_t y : 1;
    uint8_t x : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dh12_click_src_t;

#define CLICK_THS_REGADDR 0x3AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
    uint8_t ths : 7;
    uint8_t lir_click : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
    uint8_t lir_click : 1;
    uint8_t ths : 7;
#endif /* DRV_BYTE_ORDER */
} lis2dh12_click_ths_t;

#define TIME_LIMIT_REGADDR 0x3BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
    uint8_t tli : 7;
    uint8_t not_used_01 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
    uint8_t not_used_01 : 1;
    uint8_t tli : 7;
#endif /* DRV_BYTE_ORDER */
} lis2dh12_time_limit_t;

#define TIME_LATENCY_REGADDR 0x3CU
typedef struct
{
    uint8_t tla : 8;
} lis2dh12_time_latency_t;

#define TIME_WINDOW_REGADDR 0x3DU
typedef struct
{
    uint8_t tw : 8;
} lis2dh12_time_window_t;

#define ACT_THS_REGADDR 0x3EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
    uint8_t acth : 7;
    uint8_t not_used_01 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
    uint8_t not_used_01 : 1;
    uint8_t acth : 7;
#endif /* DRV_BYTE_ORDER */
} lis2dh12_act_ths_t;

#define ACT_DUR_REGADDR 0x3FU
typedef struct
{
    uint8_t actd : 8;
} lis2dh12_act_dur_t;

typedef enum
{
    opMode_highRes_12bit = 0,
    opMode_normal_10bit = 1,
    opMode_lowPower_8bit = 2,
    opMode_invalid,
} opMode_t;

/*************************************************************************************
 *  Variables
 ************************************************************************************/

static uint8_t m_i2cAddr;
static bool m_initted;
static opMode_t m_opMode;

static uint32_t m_lastPoll_ms;
static uint32_t m_desiredPollItvl_ms;

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

static esp_err_t writeAccelReg(uint8_t regAddr, uint8_t byte)
{
    uint8_t toWrite[2];
    toWrite[0] = regAddr;
    toWrite[1] = byte;
    return i2c1_writeBytes(m_i2cAddr, toWrite, 2);
}

static esp_err_t readAccelReg(uint8_t regAddr, uint8_t *pRxByte)
{
    return i2c1_readByte(m_i2cAddr, regAddr, pRxByte);
}

static esp_err_t setOpMode(opMode_t desiredMode)
{
    uint8_t ctrl_reg1;
    uint8_t ctrl_reg4;
    esp_err_t ret;

    ret = readAccelReg(CTRL_REG1_REGADDR, &ctrl_reg1);
    if (ESP_OK == ret)
    {
        ret = readAccelReg(CTRL_REG4_REGADDR, &ctrl_reg4);
    }
    if (ESP_OK == ret)
    {
        if (desiredMode == opMode_highRes_12bit)
        {
            ctrl_reg1 &= (uint8_t)(~CTRL_REG1_LPEN); // Clear Low power EN
            ctrl_reg4 |= CTRL_REG4_HIGHRES;          // Set High res
        }
        if (desiredMode == opMode_normal_10bit)
        {
            ctrl_reg1 &= (uint8_t)(~CTRL_REG1_LPEN);    // Clear Low power EN
            ctrl_reg4 &= (uint8_t)(~CTRL_REG4_HIGHRES); // Clear High Resolution
        }
        if (desiredMode == opMode_lowPower_8bit)
        {
            ctrl_reg1 |= CTRL_REG1_LPEN;                // Set low-power
            ctrl_reg4 &= (uint8_t)(~CTRL_REG4_HIGHRES); // Clear High Resolution
        }
        ret |= writeAccelReg(CTRL_REG1_REGADDR, ctrl_reg1);
        ret |= writeAccelReg(CTRL_REG4_REGADDR, ctrl_reg4);
    }
    if (ESP_OK == ret)
    {
        m_opMode = desiredMode;
    }
    return ret;
}
static esp_err_t accelInit(void)
{
    //    NRF_LOG_DEBUG("Starting accel init:");
    uint8_t byte = 0x00;
    m_i2cAddr = LIS2DH12_I2C_ADD_H >> 1;
    esp_err_t ret = readAccelReg(WHO_AM_I_REGADDR, &byte);
    if (ESP_OK != ret || WHO_AM_I_VALUE != byte)
    {
        Serial.printf("LIS2DH12 not found at 0x%x, returned 0x%x, need 0x%x\n", m_i2cAddr, byte, WHO_AM_I_VALUE);
        m_i2cAddr = LIS2DH12_I2C_ADD_L >> 1;
        ret = readAccelReg(WHO_AM_I_REGADDR, &byte);
        if (ESP_OK != ret || WHO_AM_I_VALUE != byte)
        {
            Serial.printf("LIS2DH12 not found at 0x%x, returned 0x%x, need 0x%x\n", m_i2cAddr, byte,
                          WHO_AM_I_VALUE);
            return ret;
        }
    }
    ret = 0;
    // reset the chip to defaults
    ret = writeAccelReg(CTRL_REG5_REGADDR, CTRL_REG5_MEM_REBOOT);
    if (ESP_OK != ret)
    {
        Serial.println("Error rebooting LIS2DH12");
    }
    // LIS2DH12 specifies 5ms max boot time
    delay(5);
    ret = readAccelReg(WHO_AM_I_REGADDR, &byte);
    if (ESP_OK != ret || WHO_AM_I_VALUE != byte)
    {
        Serial.printf("Error after reboot of LIS2DH12, WHOAMI 0x%x, need 0x%x\n", byte, WHO_AM_I_VALUE);
    }

    // Set up all registers with values we need.
    //    ret |= writeAccelReg(CTRL_REG0_REGADDR, CTRL_REG0_B6_0_VALUE);
    //    ret |= writeAccelReg(TEMP_CFG_REGADDR, 0x00);
    byte = CTRL_REG1_ODR(odr_50Hz);
    // TODO can we ignore one of the axes when mounted properly?
    byte |= (CTRL_REG1_X_EN | CTRL_REG1_Y_EN | CTRL_REG1_Z_EN);
    // TODO should we enable LP mode? Probably don't need to.
    ret |= writeAccelReg(CTRL_REG1_REGADDR, byte);

    // We do NOT want high-pass filtering, we need absolute positioning
    //    byte = CTRL_REG2_HPM(highPass_normalMode);
    //    ret |= writeAccelReg(CTRL_REG2_REGADDR, byte);
    // Don't enable any interrupts. TODO, wire INT pin to MCU, and interrupt on fifo watermark?
    //    ret |= writeAccelReg(CTRL_REG3_REGADDR, 0x00);
    byte = CTRL_REG4_BLOCKDATAUPDATE;
    byte |= CTRL_REG4_FULLSCALE(fullScale_2g);
    // TODO do we need to set high-res mode?
    //    byte |= CTRL_REG4_HIGHRES;
    m_opMode = opMode_normal_10bit;
    ret |= writeAccelReg(CTRL_REG4_REGADDR, byte);
    // Do ctrl_reg5 AFTER setting up the FIFO parameters
    //    byte = CTRL_REG6_INT_POLARITY;
    //    ret |= writeAccelReg(CTRL_REG6_REGADDR, byte);
    // Read Reference register, to reset filtering block, as recommended.
    ret |= readAccelReg(REFERENCE_REGADDR, &byte);
    // Enable Stream mode, bits 4:0 FIFO size, set as large as possible
    byte = FIFO_CTRL_MODE(fifoMode_stream); // Stream so it always has the latest data.
    byte |= FIFO_CTRL_THR(0xFF);            // Threshold as large as possible
    ret |= writeAccelReg(FIFO_CTRL_REGADDR, byte);

    /* Set poll interval based on ODR and FIFO size
     * At 100Hz ODR, FIFO 31 samples, we will get a sample every 10ms, can poll every 310ms.
     * At 50Hz ODR, FIFO 31 samples, we will get 31 samples every 600ms
     * TODO change if ODR changes, to read all samples.
     */
    m_desiredPollItvl_ms = 580;

    byte = CTRL_REG5_FIFO_EN;
    ret |= writeAccelReg(CTRL_REG5_REGADDR, byte);

    // Disable INT1 interrupts
    //    ret |= writeAccelReg(INT1_CFG_REGADDR, 0x00);
    // Disable INT2 interrupts
    //    ret |= writeAccelReg(INT2_CFG_REGADDR, 0x00);
    // Disable Click detection
    //    ret |= writeAccelReg(CLICK_CFG_REGADDR, 0x00);
    // Don't set up activity
    if (ESP_OK == ret)
    {
        Serial.printf("LIS2DH12 init success, I2C address 0x%x\n", m_i2cAddr);
        m_initted = true;
    }
    else
    { // Log error(s)
        Serial.printf("LIS2DH init error somewhere, ORed error %d\n", ret);
    }
    return ret;
}

static void convertSamples(uint8_t *pRawData, int16_t *pConvertedSamples, uint32_t numSamples)
{
    for (uint32_t i = 0; i < numSamples; i++)
    {
        /* Each 6 bytes is 1 sample of X, Y, and Z.
         * All data is left-justified, and LSByte is first, eg OUT_X_L=0x28, OUT_X_H=0x29
         * Left shift generates zeroes to the right, right-shifts should generate sign bits to the left (usually).
         * So, put uint8[1] into upper 8 bits, uint8[0] into lower 8 bits.
         * Then shift right by however many bits, as per resolution mode bits: 12bit, 10bit, or 8bit.
         */
        pConvertedSamples[i * 3] = (int16_t)(((int16_t)pRawData[i * 6 + 1]) << 8);
        // OR in the LSByte
        pConvertedSamples[i * 3] |= pRawData[i * 3];
        pConvertedSamples[i * 3 + 1] = (int16_t)(((int16_t)pRawData[i * 6 + 3]) << 8);
        pConvertedSamples[i * 3 + 1] |= pRawData[i * 6 + 2];
        pConvertedSamples[i * 3 + 2] = (int16_t)(((int16_t)pRawData[i * 6 + 5]) << 8);
        pConvertedSamples[i * 3 + 2] |= pRawData[i * 6 + 4];
        switch (m_opMode)
        {
        case opMode_highRes_12bit:
            // Shift down by 16-12=4 bits
            pConvertedSamples[i * 3] >>= 4;
            pConvertedSamples[i * 3 + 1] >>= 4;
            pConvertedSamples[i * 3 + 2] >>= 4;
            break;
        case opMode_normal_10bit:
            // Shift down by 16-10=6 bits
            pConvertedSamples[i * 3] >>= 6;
            pConvertedSamples[i * 3 + 1] >>= 6;
            pConvertedSamples[i * 3 + 2] >>= 6;
            break;
        case opMode_lowPower_8bit:
            // Shift down by 16-8=8 bits
            pConvertedSamples[i * 3] >>= 8;
            pConvertedSamples[i * 3 + 1] >>= 8;
            pConvertedSamples[i * 3 + 2] >>= 8;
            break;
        default:
            Serial.printf("opMode %d not supported\n", m_opMode);
            break;
        }
        //        NRF_LOG_DEBUG("Sample %d: X: %d, Y: %d, Z: %d", i, pConvertedSamples[i * 3 + 0], pConvertedSamples[i * 3 + 1],
        //                      pConvertedSamples[i * 3 + 2]);
    }
}

static void findMinMaxAverage(int16_t *pSamples, uint32_t numSamples, int16_t *pXavg, int16_t *pYavg, int16_t *pZavg)
{
    int16_t minX = 1000, maxX = -1000, minY = 1000, maxY = -1000, minZ = 1000, maxZ = -1000;
    int32_t avgX = 0, avgY = 0, avgZ = 0;
    for (uint32_t i = 0; i < numSamples; i++)
    {
        if (pSamples[i * 3] < minX)
        {
            minX = pSamples[i * 3];
        }
        if (pSamples[i * 3] > maxX)
        {
            maxX = pSamples[i * 3];
        }
        if (pSamples[i * 3 + 1] < minY)
        {
            minY = pSamples[i * 3 + 1];
        }
        if (pSamples[i * 3 + 1] > maxY)
        {
            maxY = pSamples[i * 3 + 1];
        }
        if (pSamples[i * 3 + 2] < minZ)
        {
            minZ = pSamples[i * 3 + 2];
        }
        if (pSamples[i * 3 + 2] > maxZ)
        {
            maxZ = pSamples[i * 3 + 2];
        }
        avgX += pSamples[i * 3];
        avgY += pSamples[i * 3 + 1];
        avgZ += pSamples[i * 3 + 2];
    }
    avgX /= (int32_t)numSamples;
    avgY /= (int32_t)numSamples;
    avgZ /= (int32_t)numSamples;
    *pXavg = (int16_t)avgX;
    *pYavg = (int16_t)avgY;
    *pZavg = (int16_t)avgZ;
    // Serial.printf("%d Sample min:     %d,\t%d,\t%d.\n", numSamples, minX, minY, minZ);
    // Serial.printf("%d Sample average: %d,\t%d,\t%d.\n", numSamples, *pXavg, *pYavg, *pZavg);
    // Serial.printf("%d Sample max:     %d,\t%d,\t%d.\n", numSamples, maxX, maxY, maxZ);
}

static void readSamples(void)
{
    uint8_t byte;
    //        readAccelReg(STATUS_REGADDR, &byte);
    //            NRF_LOG_DEBUG("Status reg before: 0x%x", byte);
    readAccelReg(FIFO_SRC_REGADDR, &byte);
    if (byte & FIFO_SRC_EMPTY)
    {
        Serial.printf("no FIFO samples\n");
        return;
    }
    uint32_t numSamples = FIFO_SRC_NUMUNREAD(byte);
    // Serial.printf("FIFO has %d samples\n", numSamples);

    /* Datasheet says that when using FIFO, start by reading X_OUT_L (0x28), and read 6 bytes.
     * After reading 6 bytes, the address pointer will wrap to 0x28 again. So,
     * just read 6*numSamples bytes to read all the samples at once */
    uint8_t rawDataBuf[numSamples * 6];
    // Read all
    esp_err_t ret = i2c1_readBytes(m_i2cAddr, OUT_X_L_REGADDR | AUTO_INCREMENT_MASK, rawDataBuf, 6 * numSamples);
    if (ESP_OK != ret)
    {
        Serial.printf("readSamples I2C error 0x%x\n", ret);
        return;
    }
    // If here, we have read samples
    //    NRF_LOG_HEXDUMP_INFO(rawDataBuf, sizeof(rawDataBuf));
    // Now convert samples to int16 readings. X, Y, Z so 3 axes per sample.
    int16_t convertedSamples[numSamples * 3];
    convertSamples(rawDataBuf, convertedSamples, numSamples);

    int16_t avgX, avgY, avgZ;
    findMinMaxAverage(convertedSamples, numSamples, &avgX, &avgY, &avgZ);
    // TODO find roll angle.

    //     Should be no more samples now
    //    readAccelReg(STATUS_REGADDR, &byte);
    //    NRF_LOG_DEBUG("Status reg after: 0x%x", byte);
    //    readAccelReg(FIFO_SRC_REGADDR, &byte);
    //    if (byte & FIFO_SRC_EMPTY)
    //    {
    //        NRF_LOG_DEBUG("no FIFO samples after reading %d samples", numSamples);
    //    }
    //    else
    //    {
    //        NRF_LOG_DEBUG("FIFO has %d samples after reading %d", FIFO_SRC_NUMUNREAD(byte), numSamples);
    //    }
}

void lis2dh_poll(void)
{
    if (utils_elapsedU32Ticks(m_lastPoll_ms, millis()) > m_desiredPollItvl_ms)
    { // Wait a while between re-inits
        if (!m_initted)
        {
            if (ESP_OK != accelInit())
            {
                m_desiredPollItvl_ms = 1200; // retry every 1200ms
            }
            else
            {
                // Initted, init should set poll interval. Don't reset poll inter
            }
        }
        else
        { // If initted, do poll stuff
            readSamples();
        }
        m_lastPoll_ms = millis();
    }
}

void lis2dh_init(void)
{
    m_initted = false;
    m_lastPoll_ms = millis();
    m_desiredPollItvl_ms = 100;
    i2c1_init(); // Make sure bus is enabled.

    // Init params, set it up the way we want, if we can.
    esp_err_t ret = accelInit();
    if (ESP_OK != ret)
    {
        Serial.printf("LIS2DH12 not found at address 0x%x, bailing out\n", m_i2cAddr);
        return;
        // Don't poll if we don't exist
    }
    // pollers_registerPoller(lis2dh12Poll);
}
