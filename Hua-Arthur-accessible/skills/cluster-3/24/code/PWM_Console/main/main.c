#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define LED_PIN 26 // Example GPIO pin for the LED

#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0

#define LEDC_TEST_DUTY         (4095) // Duty cycle for maximum brightness

void init_pwm_led() {
    // LEDC setup
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // 13-bit duty resolution
        .freq_hz = 5000,                      // 5 kHz frequency
        .speed_mode = LEDC_HS_MODE,
        .timer_num = LEDC_HS_TIMER
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = LED_PIN,
        .speed_mode = LEDC_HS_MODE,
        .channel = LEDC_HS_CH0_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_HS_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

void set_led_brightness(int brightness) {
    ledc_set_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL, brightness * (LEDC_TEST_DUTY / 9));
    ledc_update_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL);
}

void app_main() {
    // Initialize LED pin as GPIO output
    gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    // Initialize PWM for LED
    init_pwm_led();

    int brightness = 0;
    char input[10];

    while(1) {
        printf("Enter brightness (0-9): ");
        fgets(input, sizeof(input), stdin);
        sscanf(input, "%d", &brightness);
        if (brightness < 0) {
            brightness = 0;
        } else if (brightness > 9) {
            brightness = 9;
        }

        set_led_brightness(brightness);

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second before asking for input again
    }
}
