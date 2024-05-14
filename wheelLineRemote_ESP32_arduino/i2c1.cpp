/*
 * i2c.c
 *
 *  Created on: Apr 29, 2024
 *      Author: Collin Moore
 */

#include "i2c1.h"

#include "Arduino.h" // includes esp_hal_i2c.h

#include "utils.h"

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

#define I2C1_I2C_NUM 1 // 0 is used by OLED

#define I2C1_SCL_PIN 47
#define I2C1_SDA_PIN 48

#define NO_HW_PULLUPS 0 //
#if NO_HW_PULLUPS
#warning "Please enable HW pullups to speed I2C up"
#endif // #if NO_HW_PULLUPS

#define SCANNER_POLL_ITVL_MS 0 // 2000 // Define non-zero to scan every ms

/*************************************************************************************
 *  Variables
 ************************************************************************************/

static bool m_twi1Enabled = false;

#if SCANNER_POLL_ITVL_MS
static uint32_t m_lastPoll_ms;
#endif // #if SCANNER_POLL_ITVL_MS

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

#if SCANNER_POLL_ITVL_MS
static void scanAllAddresses(void)
{
    esp_err_t ret;
    uint8_t data;
    for (uint32_t i = 0; i < 256; i++)
    {
        size_t numRead;
        ret = i2cRead(I2C1_I2C_NUM, (uint8_t)i, &data, 1, 50, &numRead);
        if (ESP_OK == ret)
        {
            Serial.printf("Device detected at address 0x%x\n", i);
        }
    }
}
static void i2c1Poll(void)
{
    if (utils_elapsedU32Ticks(m_lastPoll_ms, millis()) > SCANNER_POLL_ITVL_MS)
    {
        scanAllAddresses();
        m_lastPoll_ms = millis();
    }
}
#endif // #if SCANNER_POLL_ITVL_MS

esp_err_t i2c1_readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *pData)
{ // Write the address, then do a stop, then read 1 byte
    return i2c1_readBytes(devAddr, regAddr, pData, 1);
}

esp_err_t i2c1_readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t *pData, uint32_t len)
{ // Write the address, then do a stop, then read as many bytes as caller wants
    esp_err_t ret = i2cWrite(I2C1_I2C_NUM, devAddr, &regAddr, 1, 10);
    if (ESP_OK != ret)
    {
        // if (NRFX_ERROR_DRV_TWI_ERR_ANACK == ret)
        // {
        //     NRF_LOG_WARNING("i2c1_readByte Address 0x%x NACKed", devAddr);
        // }
        // else if (NRFX_ERROR_DRV_TWI_ERR_DNACK == ret)
        // {
        //     NRF_LOG_WARNING("i2c1_readByte Data byte NACKed");
        // }
        // else
        // {
        Serial.printf("i2c1_readByte Error 0x%x writing devAddr 0x%x\n", ret, devAddr);
        // }
        return ret;
    }
    size_t readCount = 0;
    ret = i2cRead(I2C1_I2C_NUM, devAddr, pData, len, 100, &readCount);
    if (ESP_OK != ret)
    {
        Serial.printf("i2c1_readByte Error 0x%x reading byte\n", ret);
        return ret;
    }
    if (len != readCount)
    {
        Serial.printf("Tried to read 1 byte, read %d\n", readCount);
        return ESP_FAIL;
    }
    return ret;
}

esp_err_t i2c1_writeBytes(uint8_t devAddr, uint8_t *pByte, uint32_t len)
{ // Write the address, then data bytes
    esp_err_t ret = i2cWrite(I2C1_I2C_NUM, devAddr, pByte, len, 10 * len);
    if (ESP_OK != ret)
    {
        // if (NRFX_ERROR_DRV_TWI_ERR_ANACK == ret)
        // {
        //     NRF_LOG_WARNING("i2c1_writeBytes Address 0x%x NACKed", devAddr);
        // }
        // else if (NRFX_ERROR_DRV_TWI_ERR_DNACK == ret)
        // {
        //     NRF_LOG_WARNING("i2c1_writeBytes Data byte 0x%x NACKed", *pByte);
        // }
        // else
        // {
        Serial.printf("i2c1_writeBytes Error 0x%x writing %d bytes to devAddr 0x%x\n", ret, len, devAddr);
        // }
        return ret;
    }
    return ret;
}
#if 0
esp_err_t i2c1_tx(uint8_t devAddr, uint8_t *pByte, uint32_t len, bool repeatedStart)
{ // Write the address, then data bytes
    esp_err_t ret = nrfx_twi_tx(&m_twi1, devAddr, pByte, len, repeatedStart);
    if (ESP_OK != ret)
    {
        if (NRFX_ERROR_DRV_TWI_ERR_ANACK == ret)
        {
            NRF_LOG_WARNING("i2c1_tx Address 0x%x NACKed", devAddr);
        }
        else if (NRFX_ERROR_DRV_TWI_ERR_DNACK == ret)
        {
            NRF_LOG_WARNING("i2c1_tx Data byte 0x%x NACKed", *pByte);
        }
        else
        {
            NRF_LOG_ERROR("i2c1_tx Error 0x%x writing %d bytes to devAddr 0x%x", ret, len, devAddr);
        }
        return ret;
    }
    return ret;
}

esp_err_t i2c1_rx(uint8_t devAddr, uint8_t *pData, uint32_t len)
{
    esp_err_t ret = nrfx_twi_rx(&m_twi1, devAddr, pData, len);
    if (ESP_OK != ret)
    {
        NRF_LOG_ERROR("i2c1_rx Error 0x%x reading byte", ret);
        return ret;
    }
}
#endif // #if 0

esp_err_t i2c1_init(void)
{
    if (m_twi1Enabled)
    {
        return ESP_OK;
    }
    // If we provide a handler, calls will be non-blocking.
    for (uint8_t i = 0; i < 4; i++)
    {
        if (i2cIsInit(i))
        {
            Serial.printf("I2C inst %d is used already\n", i);
        }
    }

    esp_err_t ret = i2cInit(I2C1_I2C_NUM, I2C1_SDA_PIN, I2C1_SCL_PIN, 100000);
    if (ESP_OK != ret)
    {
        Serial.printf("i2cInit err 0x%x\n", ret);
        return ret;
    }
#if NO_HW_PULLUPS
    // Mux pullups on the pins for the bus to work, ONLY if no HW pullup is on the lines
    NRF_P0->PIN_CNF[m_twi1Cfg.scl] |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
    NRF_P0->PIN_CNF[m_twi1Cfg.sda] |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
#endif // #if NO_HW_PULLUPS

    m_twi1Enabled = true;
#if SCANNER_POLL_ITVL_MS
    m_lastPoll_ms = millis();
    pollers_registerPoller(i2c1Poll);
#endif // #if SCANNER_POLL_ITVL_MS

    return ESP_OK;
}
