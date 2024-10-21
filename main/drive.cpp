/**
 * @file drive.cpp
 * @author Augusto De Conto (augustodeconto@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-10-20
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <driver/gpio.h>
#include <driver/mcpwm.h>
#include <esp_err.h>
#include <esp_log.h>

#include "drive.h"

/**
 * Motor 1
 */
#define MOTOR1_IO_RPWM (GPIO_NUM_32) // VLOAD2A
#define MOTOR1_IO_LPWM (GPIO_NUM_33) // VLOAD2B
#define MOTOR1_IO_SD   (GPIO_NUM_22)

/**
 * Motor 2
 */
#define MOTOR2_IO_RPWM (GPIO_NUM_18) // VLOAD1A
#define MOTOR2_IO_LPWM (GPIO_NUM_19) // VLOAD1B
#define MOTOR2_IO_SD   (GPIO_NUM_23)

#define PWM_FREQUENCY 15000 // L298 Typical frequency (15KHz). / 40KHz max.
#define SD_OUTPUT_SEL ((1ULL << MOTOR2_IO_SD) | (1ULL << MOTOR1_IO_SD))

static const float DUTY_DEAD_ZONE        = 0.1;
static const float DUTY_SATURATION_LEVEL = 95.0;

// static const char *TAG = "drive";

// TODO: evaluate if we can use the same MCPWM unit with different timers to
// both motors

#define MOTOR1_UNIT  MCPWM_UNIT_1
#define MOTOR1_TIMER MCPWM_TIMER_1

#define MOTOR2_UNIT  MCPWM_UNIT_0
#define MOTOR2_TIMER MCPWM_TIMER_0

/**
 * @brief Prevents electric current rushes during initialization by setting
 * H-bridge pins to high.
 *
 * This function configures the specified H-bridge pins as outputs and sets them
 * all to a high level to prevent current rushes when the system initializes. It
 * is designed to be called before the main initialization process to ensure a
 * safe power-on state.
 *
 * @return none
 */
void Drive::preInitSafety(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR1_IO_SD) | (1ULL << MOTOR1_IO_LPWM) |
                        (1ULL << MOTOR1_IO_RPWM) | (1ULL << MOTOR2_IO_SD) |
                        (1ULL << MOTOR2_IO_LPWM) | (1ULL << MOTOR2_IO_RPWM),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    // Set all H-bridge pins to high
    gpio_set_level(MOTOR1_IO_SD, 1);
    gpio_set_level(MOTOR2_IO_SD, 1);
    gpio_set_level(MOTOR1_IO_LPWM, 1);
    gpio_set_level(MOTOR1_IO_RPWM, 1);
    gpio_set_level(MOTOR2_IO_LPWM, 1);
    gpio_set_level(MOTOR2_IO_RPWM, 1);
}

int Drive::setup(void)
{
    esp_err_t espError;

    do
    {
        gpio_config_t io_conf = {};
        io_conf.intr_type     = GPIO_INTR_DISABLE;
        io_conf.mode          = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask  = SD_OUTPUT_SEL;
        io_conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en    = GPIO_PULLUP_DISABLE;

        espError = gpio_config(&io_conf);
        if (espError != ESP_OK)
            break;

        mcpwm_config_t pwm_config = {};
        pwm_config.frequency      = PWM_FREQUENCY;
        pwm_config.cmpr_a         = 0;
        pwm_config.cmpr_b         = 0;
        pwm_config.counter_mode   = MCPWM_UP_COUNTER;
        pwm_config.duty_mode      = MCPWM_DUTY_MODE_0;

        // Motor 1
        espError = gpio_set_level(MOTOR1_IO_SD, 1);
        if (espError != ESP_OK)
            break;
        espError = mcpwm_gpio_init(MOTOR1_UNIT, MCPWM1A, MOTOR1_IO_RPWM);
        if (espError != ESP_OK)
            break;
        espError = mcpwm_gpio_init(MOTOR1_UNIT, MCPWM1B, MOTOR1_IO_LPWM);
        if (espError != ESP_OK)
            break;
        espError = mcpwm_init(MOTOR1_UNIT, MOTOR1_TIMER, &pwm_config);
        if (espError != ESP_OK)
            break;
        this->setDutyCycle(Channel::Motor1, 0.0);

        // Motor 2
        espError = gpio_set_level(MOTOR2_IO_SD, 1);
        if (espError != ESP_OK)
            break;
        espError = mcpwm_gpio_init(MOTOR2_UNIT, MCPWM0A, MOTOR2_IO_RPWM);
        if (espError != ESP_OK)
            break;
        espError = mcpwm_gpio_init(MOTOR2_UNIT, MCPWM0B, MOTOR2_IO_LPWM);
        if (espError != ESP_OK)
            break;
        espError = mcpwm_init(MOTOR2_UNIT, MOTOR2_TIMER, &pwm_config);
        if (espError != ESP_OK)
            break;
        this->setDutyCycle(Channel::Motor2, 0.0);
    } while (0); // goto fail pattern

    return 0;
}

/**
 * @brief Commands drivers to a duty cycle.
 *
 * @param channel select Drive::Channel to command
 * @param duty Duty cycle to be commanded. Value range is -100.0 to 100.0.
 * Negative value reverses HBridge direction.
 * If  -DUTY_DEAD_ZONE < duty < DUTY_DEAD_ZONE, HBridge will stop.
 */
esp_err_t Drive::setDutyCycle(Channel channel, float duty)
{
    float absDuty;
    this->dutyCommand[(int)channel] = duty;
    // modulus
    absDuty = (duty >= 0) ? (duty) : (-duty);
    // saturate
    absDuty =
        (absDuty <= DUTY_SATURATION_LEVEL) ? (absDuty) : DUTY_SATURATION_LEVEL;
    // ESP_LOGI(TAG, "channel %d  duty %5.1f  cmd %5.1f", (int)channel, duty,
    // absDuty);

    // duty is float, thus cannot be compared to zero. It is required an
    // epsilon. DUTY_DEAD_ZONE is appropriated.
    if (duty >= -DUTY_DEAD_ZONE && duty <= DUTY_DEAD_ZONE) // Stop
    {
        if (channel == Channel::Motor1)
        {
            // ESP_LOGI(TAG, "(absDuty < DUTY_DEAD_ZONE)  (channel ==
            // Channel::Motor1)");
            ESP_ERROR_CHECK(gpio_set_level(MOTOR1_IO_SD, 1));
            ESP_ERROR_CHECK(
                mcpwm_set_signal_high(MOTOR1_UNIT, MOTOR1_TIMER, MCPWM_OPR_A));
            ESP_ERROR_CHECK(
                mcpwm_set_signal_high(MOTOR1_UNIT, MOTOR1_TIMER, MCPWM_OPR_B));
        }
        else if (channel == Channel::Motor2)
        {
            // ESP_LOGI(TAG, "(absDuty < DUTY_DEAD_ZONE)  (channel ==
            // Channel::Motor2)");
            ESP_ERROR_CHECK(gpio_set_level(MOTOR2_IO_SD, 1));
            ESP_ERROR_CHECK(
                mcpwm_set_signal_high(MOTOR2_UNIT, MOTOR2_TIMER, MCPWM_OPR_A));
            ESP_ERROR_CHECK(
                mcpwm_set_signal_high(MOTOR2_UNIT, MOTOR2_TIMER, MCPWM_OPR_B));
        }
    }
    else if (duty > 0)
    {
        if (channel == Channel::Motor1)
        {
            // ESP_LOGI(TAG, "(duty > 0)  (channel == Channel::Motor1)");
            ESP_ERROR_CHECK(
                mcpwm_set_signal_high(MOTOR1_UNIT, MOTOR1_TIMER, MCPWM_OPR_B));
            ESP_ERROR_CHECK(mcpwm_set_duty(
                MOTOR1_UNIT, MOTOR1_TIMER, MCPWM_OPR_A, absDuty));
            ESP_ERROR_CHECK(mcpwm_set_duty_type(
                MOTOR1_UNIT, MOTOR1_TIMER, MCPWM_OPR_A, MCPWM_DUTY_MODE_1));
            ESP_ERROR_CHECK(gpio_set_level(MOTOR1_IO_SD, 0));
        }
        else if (channel == Channel::Motor2)
        {
            // ESP_LOGI(TAG, "(duty > 0)  (channel == Channel::Motor2)");
            ESP_ERROR_CHECK(
                mcpwm_set_signal_high(MOTOR2_UNIT, MOTOR2_TIMER, MCPWM_OPR_B));
            ESP_ERROR_CHECK(mcpwm_set_duty(
                MOTOR2_UNIT, MOTOR2_TIMER, MCPWM_OPR_A, absDuty));
            ESP_ERROR_CHECK(mcpwm_set_duty_type(
                MOTOR2_UNIT, MOTOR2_TIMER, MCPWM_OPR_A, MCPWM_DUTY_MODE_1));
            ESP_ERROR_CHECK(gpio_set_level(MOTOR2_IO_SD, 0));
        }
    }
    else // (duty < 0)
    {
        if (channel == Channel::Motor1)
        {
            // ESP_LOGI(TAG, "(duty < 0)  (channel == Channel::Motor1)");
            ESP_ERROR_CHECK(
                mcpwm_set_signal_high(MOTOR1_UNIT, MOTOR1_TIMER, MCPWM_OPR_A));
            ESP_ERROR_CHECK(mcpwm_set_duty(
                MOTOR1_UNIT, MOTOR1_TIMER, MCPWM_OPR_B, absDuty));
            ESP_ERROR_CHECK(mcpwm_set_duty_type(
                MOTOR1_UNIT, MOTOR1_TIMER, MCPWM_OPR_B, MCPWM_DUTY_MODE_1));
            ESP_ERROR_CHECK(gpio_set_level(MOTOR1_IO_SD, 0));
        }
        else if (channel == Channel::Motor2)
        {
            // ESP_LOGI(TAG, "(duty < 0)  (channel == Channel::Motor2)");
            ESP_ERROR_CHECK(
                mcpwm_set_signal_high(MOTOR2_UNIT, MOTOR2_TIMER, MCPWM_OPR_A));
            ESP_ERROR_CHECK(mcpwm_set_duty(
                MOTOR2_UNIT, MOTOR2_TIMER, MCPWM_OPR_B, absDuty));
            ESP_ERROR_CHECK(mcpwm_set_duty_type(
                MOTOR2_UNIT, MOTOR2_TIMER, MCPWM_OPR_B, MCPWM_DUTY_MODE_1));
            ESP_ERROR_CHECK(gpio_set_level(MOTOR2_IO_SD, 0));
        }
    }

    return 0;
}

/**
 * @brief Reads the commanded duty cycle
 *
 * @param channel select Drive::Channel read
 * @return float commanded duty cycle
 */
float Drive::getDutyCycle(Channel channel)
{
    return this->dutyCommand[(int)channel];
}
