/**
 * @file analog.cpp
 * @author Augusto De Conto
 * @brief
 * @version 0.2
 * @date 2023-06-25
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <math.h>

#include "analog.h"

/**
 * I2C
 */
#define I2C_MASTER_SCL_IO  (GPIO_NUM_15)
#define I2C_MASTER_SDA_IO  (GPIO_NUM_14)
#define I2C_MASTER_NUM     (I2C_NUM_0)
#define I2C_MASTER_FREQ_HZ 1000000

static const char *TAG = "analog";

SemaphoreHandle_t Analog::adsMutex = nullptr;
bool              Analog::isSetup  = false;

/**
 * @brief Construct a new Analog::Analog object
 *
 */
Analog::Analog()
{
    if (this->adsMutex == nullptr)
    {
        this->adsMutex = xSemaphoreCreateMutex();
        this->isSetup  = false;
    }
    return;
}

/**
 * @brief Setup analog.
 * Config and install i2c.
 *
 * @return int
 */
int Analog::setup()
{
    esp_err_t espErr = i2cSetup();
    if (espErr != ESP_OK)
    {
        ESP_LOGE(TAG, "setup::i2cSetup failed esp_err 0x%04X", espErr);
        return 0;
    }

    // Test ADS1115 connection:
    // read configuration register.
    // Should return 0x8583 after reset. There is no assured reset before setup.
    // So we can't use this to test. We noticed it returns 0xFFFF when fails.
    uint16_t cfgRead = readRegister(ads1115ConfigReg);
    if (cfgRead == 0xFFFF)
    {
        ESP_LOGE(TAG, "setup failed to read ADC");
        return 0;
    }

    this->isSetup = true;
    return 0;
}

/**
 * @brief Reads board analog channel.
 *
 * @param[in] channel Board input channel
 * @param[out] readVolts Analog value read [Volts]
 * @param[out] isValid true if value read is valid, false if an error occured
 * @return int Error code
 */
int Analog::read(Channel channel, float &readVolts, bool &isValid)
{
    esp_err_t espError = ESP_OK;

    switch (channel)
    {
        case (Channel::Pot1):
            espError = this->read(ads1115InputMux::AIN0GND, readVolts, isValid);
            break;
        case (Channel::Pot2):
            espError = this->read(ads1115InputMux::AIN1GND, readVolts, isValid);
            break;
    }

    return 0;
}

/**
 * @brief Communicate to ADC to read analog input.
 *
 * @param[in] inputMux ADS1115 input multiplexer
 * @param[out] readVolts Analog value read [Volts]
 * @param[out] isValid true if value read is valid, false if an error occured
 * @return int Error code
 */
int Analog::read(ads1115InputMux inputMux, float &readVolts, bool &isValid)
{
    uint16_t raw = 0;
    uint16_t cfgRead;
    bool     timeout = false;

    xSemaphoreTake(this->adsMutex, portMAX_DELAY);

#ifdef ANALOG_DEBUG
    lbi = 0; // clears log buffer
#endif

    TickType_t timeoutTick = xTaskGetTickCount() + pdMS_TO_TICKS(readTimeout);

    do
    {
        cfgRead = readRegister(ads1115ConfigReg);
        timeout = (xTaskGetTickCount() >= timeoutTick);
    } while (!(cfgRead & ads1115OS) && !timeout); // while performing conversion
    // Config register at ads1115OS flag (cfgRead & ads1115OS)
    // 0 : Device is currently performing a conversion
    // 1 : Device is not currently performing a conversion

    if (!timeout)
    {
        requestSingleShot(inputMux);
        timeout = (xTaskGetTickCount() >= timeoutTick);
    }

    if (!timeout)
    {
        do
        {
            cfgRead = readRegister(ads1115ConfigReg);
            timeout = (xTaskGetTickCount() >= timeoutTick);
        } while (!(cfgRead & ads1115OS) &&
                 !timeout); // while performing conversion
    }

    if (!timeout)
    {
        raw = readRegister(ads1115ConversionReg);
    }

    if (!timeout)
    {
        readVolts = rawToV * raw;
        isValid   = true;
    }
    else
    {
        readVolts = NAN;
        isValid   = false;
    }
    xSemaphoreGive(this->adsMutex);

#ifdef ANALOG_DEBUG
    printLog();
#endif

    return 0;
}

/**
 * @brief Setup i2c communication to ADS1115
 *
 * @return esp_err_t ESP error
 */
esp_err_t Analog::i2cSetup()
{
    esp_err_t espErr;

    i2c_config_t i2c_config = {
        .mode          = I2C_MODE_MASTER,
        .sda_io_num    = I2C_MASTER_SDA_IO,
        .scl_io_num    = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master        = {.clk_speed = I2C_MASTER_FREQ_HZ},
        .clk_flags     = I2C_SCLK_SRC_FLAG_FOR_NOMAL};

    espErr = i2c_param_config(I2C_MASTER_NUM, &i2c_config);
    espErr = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);

    return espErr;
}

/**
 * @brief Request ADS1115 to convert input on single shot mode.
 *
 * @param inputMux ADS1115 input multiplexer
 * @return esp_err_t ESP error
 */
esp_err_t Analog::requestSingleShot(ads1115InputMux inputMux)
{
    uint16_t configuration = ((uint16_t)inputMux << 12) | ads1115Cfg;
    writeRegister(ads1115ConfigReg, configuration);
    return ESP_OK;
}

/**
 * @brief Communicate to ADS1115 to read a register
 *
 * @param addrPtrReg register address
 * @return uint16_t register value
 */
uint16_t Analog::readRegister(uint8_t addrPtrReg)
{
    uint8_t          data[2] = {0};
    i2c_cmd_handle_t cmd;
    esp_err_t        espErr = ESP_OK;

    // Implements Figure 30. Timing Diagram for Reading From ADS111x
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ads1115Addr | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, addrPtrReg, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ads1115Addr | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);

#ifdef ANALOG_DEBUG
    logBuff[lbi++] = 0xF2;
    logBuff[lbi++] = ads1115Addr | I2C_MASTER_WRITE;
    logBuff[lbi++] = addrPtrReg;
    logBuff[lbi++] = ads1115Addr | I2C_MASTER_READ;
    logBuff[lbi++] = data[0];
    logBuff[lbi++] = data[1];
#endif

    espErr =
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(readTimeout));
    i2c_cmd_link_delete(cmd);

    int16_t result = (data[0] << 8) | data[1];

    if (espErr != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read adc with i2c  err 0x%04X", espErr);
        result = 0;
    }

    return result;
}

/**
 * @brief Communicate to ADS1115 to write a register
 *
 * @param addrPtrReg register address
 * @param value value to write
 */
void Analog::writeRegister(uint8_t addrPtrReg, uint16_t value)
{
    i2c_cmd_handle_t cmd;

    // Implements Figure 31. Timing Diagram for Writing to ADS111x
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ads1115Addr | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, addrPtrReg, true);
    i2c_master_write_byte(cmd, (value & 0xFF00) >> 8, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, (value & 0x00FF), I2C_MASTER_ACK);
    i2c_master_stop(cmd);

#ifdef ANALOG_DEBUG
    logBuff[lbi++] = 0xF1;
    logBuff[lbi++] = ads1115Addr | I2C_MASTER_WRITE;
    logBuff[lbi++] = addrPtrReg;
    logBuff[lbi++] = (value & 0xFF00) >> 8;
    logBuff[lbi++] = (value & 0x00FF);
#endif

    esp_err_t espErr =
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(readTimeout));
    i2c_cmd_link_delete(cmd);
    if (espErr != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write adc with i2c. Esp error 0x%04X", espErr);
    }

    return;
}

/**
 * @brief Tests analog performace troggling input channels.
 * This method is for development proposes.
 */
void Analog::testToggleRate(void)
{
    const int readCount = 10;
    float     readings[readCount][2];
    uint64_t  currentTime[readCount + 1][2];
    int       i;

    Analog analog;
    analog.setup();

    ESP_LOGI("adcTest", "start read");
    for (i = 0; i < readCount; i++)
    {
        bool isValid;
        currentTime[i][0] = esp_timer_get_time();
        analog.read(Channel::Pot1, readings[i][0], isValid);
        currentTime[i][1] = esp_timer_get_time();
        analog.read(Channel::Pot2, readings[i][1], isValid);
    }
    currentTime[readCount][0] = esp_timer_get_time();

    for (i = 0; i < readCount; i++)
    {
        ESP_LOGI(
            "adcTest",
            "Gear %.3f  Throttle %.3f    Read time [%9lu %9lu] microseconds",
            readings[i][0], readings[i][1],
            (uint32_t)(currentTime[i][1] - currentTime[i][0]),
            (uint32_t)(currentTime[i + 1][0] - currentTime[i][1]));
    }

    return;
}

#ifdef ANALOG_DEBUG
void Analog::printLog()
{
    int  n = lbi * 3;
    char msg[n + 1];

    int j = 0;

    for (int i = 0; i < lbi; i++)
    {
        if (i == (lbi - 1))
        {
            snprintf(msg + j, sizeof(msg) - j, "%02x", logBuff[i]);
            j = j + 2;
        }
        else
        {
            snprintf(msg + j, sizeof(msg) - j, "%02x,", logBuff[i]);
            j = j + 3;
        }
        msg[j] = '\0';
    }
    ESP_LOGI("analog", "%2d  [%s]", lbi, msg);
}
#endif
