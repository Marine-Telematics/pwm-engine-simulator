/**
 * @file analog.h
 * @author Augusto De Conto (augustodeconto@gmail.com)
 * @brief Interface with controller board analog inputs.
 * @version 0.1
 * @date 2023-12-13
 *
 *
 * Implementation is thread safe, may have multiple Analog instances.
 * Analog Digital Converter ads1115 using i2c.
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef ANALOG_H_INCLUDED
#define ANALOG_H_INCLUDED

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdint.h>

//#define ANALOG_DEBUG

class Analog
{
    public:
    enum class Channel
    {
        Pot1,
        Pot2
    };

    Analog();
    int setup();
    int read(Channel channel, float &readVolts, bool &isValid);

    static const uint16_t ads1115PFSOC =
        0x7FFF; // Positive Full Scale Output Code
    static constexpr float ads1115FSR = 4.096; // Full-Scale Range [V]
    static constexpr float rawToV =
        ads1115FSR / ads1115PFSOC; // Constant to convert raw value to Volts

    private:
    static SemaphoreHandle_t adsMutex;
    static bool              isSetup;

    // Input multiplexer configuration, field MUX[2:0]
    enum class ads1115InputMux
    {
        // You would say it is not necessary to define the value
        // for each enum element. Yes, BUT, since it is extremely
        // critial for system to read the correct input, there is
        // no excess been explicit.
        AIN01   = 0b000, // 000 : AINP = AIN0 and AINN = AIN1 (default)
        AIN03   = 0b001, // 001 : AINP = AIN0 and AINN = AIN3
        AIN13   = 0b010, // 010 : AINP = AIN1 and AINN = AIN3
        AIN23   = 0b011, // 011 : AINP = AIN2 and AINN = AIN3
        AIN0GND = 0b100, // 100 : AINP = AIN0 and AINN = GND
        AIN1GND = 0b101, // 101 : AINP = AIN1 and AINN = GND
        AIN2GND = 0b110, // 110 : AINP = AIN2 and AINN = GND
        AIN3GND = 0b111  // 111 : AINP = AIN3 and AINN = GND
    };

    int       read(ads1115InputMux mux, float &readVolts, bool &isValid);
    esp_err_t i2cSetup();
    esp_err_t requestSingleShot(ads1115InputMux input);
    uint16_t  readRegister(uint8_t addrPtrReg);
    void      writeRegister(uint8_t addrPtrReg, uint16_t value);

    static const uint8_t ads1115Addr =
        (0b1001000 << 1); // ADDR pin connection GND
    static const uint8_t ads1115ConversionReg =
        0b00;                                     // 00 : Conversion register
    static const uint8_t ads1115ConfigReg = 0b01; // 01 : Config register

    // ADS1115 configuration
    static const uint16_t ads1115OS =
        (0b0001 << 15); // When writing  1 : Start a single conversion
    static const uint16_t ads1115PGA = (0b0001 << 9); // 001 : FSR = ±4.096 V
    static const uint16_t ads1115Mode =
        (0b0001 << 8); // 1   : Single-shot mode or power-down state
    static const uint16_t ads1115DR = (0b0111 << 5); // 111 : 860 SPS
    static const uint16_t ads1115CQue =
        0b0011; // 11  : Disable comparator and set ALERT/RDY pin to
                // high-impedance
    static const uint16_t ads1115Cfg =
        ads1115OS | ads1115PGA | ads1115Mode | ads1115DR | ads1115CQue;

    static const uint16_t readTimeout =
        25; // Configured timeout to read ADC [ms]

#ifdef ANALOG_DEBUG
    static const int LOG_BUFF_SIZE = 64;
    uint8_t          logBuff[LOG_BUFF_SIZE];
    int              lbi = 0; // log buffer index
    void             printLog();
#endif

    // Tests should not be here!
    // they should be in a specific test file
    // (but I still didn't have time to setup tests environment)
    public:
    static void testToggleRate();
};

#endif /* ANALOG_H_INCLUDED */
