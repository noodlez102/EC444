#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/i2c.h"
#include <string.h>
#include <math.h>
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

#define DEFAULT_VREF 1100 // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 128  // Multisampling

static const char *TAG = "Sonar";
int sensor_delay = 10; //ms

// servo settings
#define SERVO_MIN_PULSEWIDTH_US_1 100        // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US_1 2700       // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE -90                 // Minimum angle
#define SERVO_MAX_DEGREE 90                  // Maximum angle
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD 20000          // 20000 ticks, 20ms
mcpwm_cmpr_handle_t servo_comparator;
int servo_gpio_pin = 12;

// thermistor
static esp_adc_cal_characteristics_t *thermistor_adc_chars;
static const adc_channel_t thermistor_channel = ADC_CHANNEL_6; // GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t thermistor_atten = ADC_ATTEN_DB_11;
static const adc_unit_t thermistor_unit = ADC_UNIT_1;

// IR range sensor
static esp_adc_cal_characteristics_t *IR_adc_chars;
static const adc_channel_t IR_channel = ADC_CHANNEL_3; // GPIO39 if ADC1, GPIO15 if ADC2
static const adc_atten_t IR_atten = ADC_ATTEN_DB_11;
static const adc_unit_t IR_unit = ADC_UNIT_1;

// ultrasonic range sensor
static esp_adc_cal_characteristics_t *ultrasonic_adc_chars;
static const adc_channel_t ultrasonic_channel = ADC_CHANNEL_0; // GPIO36 if ADC1
static const adc_atten_t ultrasonic_atten = ADC_ATTEN_DB_0;
static const adc_unit_t ultrasonic_unit = ADC_UNIT_1;

float thermistor_T = 0.0;
float IR_d = 0.0;
float ultrasonic_d = 0.0;
int current_angle = 0;

// Servo functions
static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US_1 - SERVO_MIN_PULSEWIDTH_US_1) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US_1;
}

static mcpwm_cmpr_handle_t servo_init(int gpio_pin)
{
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL; // Create PWM timer
    mcpwm_timer_config_t timer_config = {
        // Configure PWM timer
        .group_id = 0,                                 // Pick PWM group 0
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,        // Default clock source
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ, // Hz
        .period_ticks = SERVO_TIMEBASE_PERIOD,         // Set servo period (20ms -- 50 Hz)
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,       // Count up
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL; // Create PWM operator
    mcpwm_operator_config_t operator_config = {
        // Configure PWM operator
        .group_id = 0, // operator same group and PWM timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator"); // Connect PWM timer and PWM operator
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t comparator = NULL; // Create PWM comparator
    mcpwm_comparator_config_t comparator_config = {
        // Updates when timer = zero
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL; // Create generator
    mcpwm_generator_config_t generator_config = {
        // Output to GPIO pin
        .gen_gpio_num = gpio_pin,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));                                // Enable
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP)); // Run continuously
    return comparator;
}

// ADC functions

static void check_efuse(void)
{
    // Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("eFuse Two Point: NOT supported\n");
    }

    // Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        printf("eFuse Vref: Supported\n");
    }
    else
    {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        printf("Characterized using Two Point Value\n");
    }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        printf("Characterized using eFuse Vref\n");
    }
    else
    {
        printf("Characterized using Default Vref\n");
    }
}

static void task_thermistor()
{
    // Configure ADC
    if (thermistor_unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(thermistor_channel, thermistor_atten);
    }
    else
    {
        adc2_config_channel_atten((adc2_channel_t)thermistor_channel, thermistor_atten);
    }

    // Characterize ADC
    thermistor_adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(thermistor_unit, thermistor_atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, thermistor_adc_chars);
    print_char_val_type(val_type);

    // Continuously sample ADC1
    while (1)
    {
        uint32_t adc_reading = 0;
        // Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            if (thermistor_unit == ADC_UNIT_1)
            {
                adc_reading += adc1_get_raw((adc1_channel_t)thermistor_channel);
            }
            else
            {
                int raw;
                adc2_get_raw((adc2_channel_t)thermistor_channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, thermistor_adc_chars) - 150;
        float steinhart;
        float thermistor_b = 3435.0;
        float thermistor_t0 = 298.15;
        float thermistor_r0 = 10000.0;
        steinhart = 1.0 / thermistor_t0 + (1.0 / thermistor_b) * log((3300.0 / (float)voltage) - 1.0);
        steinhart = 1.0 / steinhart; // Invert
        steinhart -= 273.15;         // Convert to Celsius
        thermistor_T = steinhart;
        vTaskDelay(pdMS_TO_TICKS(sensor_delay));
    }
}

static void task_IR_range_sensor()
{
    // Configure ADC
    if (IR_unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(IR_channel, IR_atten);
    }
    else
    {
        adc2_config_channel_atten((adc2_channel_t)IR_channel, IR_atten);
    }

    // Characterize ADC
    IR_adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(IR_unit, IR_atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, IR_adc_chars);
    print_char_val_type(val_type);

    // Continuously sample ADC1
    while (1)
    {
        uint32_t adc_reading = 0;
        // Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            if (IR_unit == ADC_UNIT_1)
            {
                adc_reading += adc1_get_raw((adc1_channel_t)IR_channel);
            }
            else
            {
                int raw;
                adc2_get_raw((adc2_channel_t)IR_channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, IR_adc_chars);
        float distance = 0.01 * (1.0 / (0.00004 * voltage - 0.005)); // m
        IR_d = distance;
        vTaskDelay(pdMS_TO_TICKS(sensor_delay));
    }
}

static void task_ultrasonic_sensor()
{
    // Configure ADC
    if (ultrasonic_unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(ultrasonic_channel, ultrasonic_atten);
    }
    else
    {
        adc2_config_channel_atten((adc2_channel_t)ultrasonic_channel, ultrasonic_atten);
    }

    // Characterize ADC
    ultrasonic_adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ultrasonic_unit, ultrasonic_atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, ultrasonic_adc_chars);
    print_char_val_type(val_type);

    // Continuously sample ADC1
    while (1)
    {
        uint32_t adc_reading = 0;
        // Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            if (ultrasonic_unit == ADC_UNIT_1)
            {
                adc_reading += adc1_get_raw((adc1_channel_t)ultrasonic_channel);
            }
            else
            {
                int raw;
                adc2_get_raw((adc2_channel_t)ultrasonic_channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, ultrasonic_adc_chars) - 10;
        float distance = (5.0 / (3300.0 / 4096.0)) * (float)voltage - 100; // convert voltage to distance in mm
        ultrasonic_d = distance / 1000.0;
        vTaskDelay(pdMS_TO_TICKS(sensor_delay));
    }
}

static void task_reporting()
{
    float time_ms = 0;
    int delay = 100;
    while (1)
    {
        time_ms += delay;
        float time_sec = time_ms / 1000.0;

        // printf("Time: %fs\t", time_sec);
        // printf("Thermistor Temperature: %fC\t", thermistor_T);
        // printf("IR Range Sensor Distance: %fm\t", IR_d);
        // printf("Ultrasonic Range Sensor Distance: %fm\t", ultrasonic_d);
        // printf("Servo Angle: %d\n", current_angle);

        printf("%f,ultrasonic,%f,%d\n", time_sec, ultrasonic_d, current_angle);
        printf("%f,IR,%f,%d\n", time_sec, IR_d, current_angle);
        printf("%f,thermistor,%f,0\n", time_sec, thermistor_T);

        vTaskDelay(pdMS_TO_TICKS(delay));
    }
}

static void task_servo()
{
    int angle = 0;
    int step = 1;
    int max_angle = 60;
    while (1)
    {
        current_angle = angle;
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(servo_comparator, example_angle_to_compare(-angle)));
        // Add delay, since it takes time for servo to rotate, usually 200ms/60degree rotation under 5V power supply
        vTaskDelay(pdMS_TO_TICKS(50));
        if ((angle + step) > max_angle || (angle + step) < -max_angle)
        {
            step *= -1;
        }
        angle += step;
    }
}

void app_main(void)
{
    // Check if Two Point or Vref are burned into eFuse
    check_efuse();

    servo_comparator = servo_init(servo_gpio_pin);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(servo_comparator, example_angle_to_compare(0)));
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    xTaskCreate(task_thermistor, "task_thermistor", 4096, NULL, 5, NULL);
    xTaskCreate(task_IR_range_sensor, "task_IR_range_sensor", 4096, NULL, 5, NULL);
    xTaskCreate(task_ultrasonic_sensor, "task_ultrasonic_sensor", 4096, NULL, 5, NULL);
    xTaskCreate(task_servo, "task_servo", 4096, NULL, 5, NULL);
    xTaskCreate(task_reporting, "task_reporting", 4096, NULL, 5, NULL);
}