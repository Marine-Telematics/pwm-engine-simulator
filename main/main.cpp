#include <freertos/FreeRTOS.h>
#include <stdio.h>

#include "analog.h"
#include "drive.h"

extern "C" void app_main()
{
    Drive::preInitSafety();

    vTaskDelay(pdMS_TO_TICKS(1000));

    static Analog analog{};
    analog.setup();

    static Drive drive{};
    drive.setup();

    float volts = 0.0F;
    float duty  = 0.0F;
    bool  valid = false;

    static constexpr float IDLE_DUTY = 50.0F;
    static constexpr float SLOPE     = (3.3F - 0.10F) / (100.0F - IDLE_DUTY);

    for (;;)
    {
        analog.read(Analog::Channel::Pot1, volts, valid);

        duty = IDLE_DUTY + (volts / SLOPE);
        drive.setDutyCycle(Drive::Channel::Motor1, duty);

        printf("volts1 %.2fV  duty1 %.2f%%", volts, duty);

        analog.read(Analog::Channel::Pot2, volts, valid);

        duty = IDLE_DUTY + (volts / SLOPE);
        drive.setDutyCycle(Drive::Channel::Motor2, duty);

        printf("    volts2 %.2fV  duty2 %.2f%%\n", volts, duty);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
