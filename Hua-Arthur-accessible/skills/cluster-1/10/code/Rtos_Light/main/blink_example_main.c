/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"


static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

static uint8_t s_led_state = 0;

#ifdef CONFIG_BLINK_LED_RMT
static led_strip_t *pStrip_a;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        pStrip_a->set_pixel(pStrip_a, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        pStrip_a->refresh(pStrip_a, 100);
    } else {
        /* Set all LED off to clear all pixels */
        pStrip_a->clear(pStrip_a, 50);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    pStrip_a = led_strip_init(CONFIG_BLINK_LED_RMT_CHANNEL, BLINK_GPIO, 1);
    /* Set all LED off to clear all pixels */
    pStrip_a->clear(pStrip_a, 50);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(int x)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(x);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(x, GPIO_MODE_OUTPUT);
}

#endif
bool up = false;


void blink_task_does()
{

    /* Configure the peripheral according to the LED type */
    configure_led(26);
    configure_led(25);
    configure_led(27);
    configure_led(33);

    int i =0;
    while (1) {
        
        if (i == 0) {
            gpio_set_level(26, true);
            gpio_set_level(25, true);
            gpio_set_level(27, true);
            gpio_set_level(33, true);
            up = !up;
        }
        else if (i == 1) {
            gpio_set_level(26, true);
            gpio_set_level(25, true);
            gpio_set_level(27, true);
            gpio_set_level(33, false);
        }
        else if (i == 2) {
            gpio_set_level(26, true);
            gpio_set_level(25, true);
            gpio_set_level(27, false);
            gpio_set_level(33, true);
        }
        else if (i == 3) {
            gpio_set_level(26, true);
            gpio_set_level(25, true);
            gpio_set_level(27, false);
            gpio_set_level(33, false);
        }
        else if (i == 4) {
            gpio_set_level(26, true);
            gpio_set_level(25, false);
            gpio_set_level(27, true);
            gpio_set_level(33, true);
        }
        else if (i == 5) {
            gpio_set_level(26, true);
            gpio_set_level(25, false);
            gpio_set_level(27, true);
            gpio_set_level(33, false);
        }
        else if (i == 6) {
            gpio_set_level(26, true);
            gpio_set_level(25, false);
            gpio_set_level(27, false);
            gpio_set_level(33, true);
        }
        else if (i == 7) {
            gpio_set_level(26, true);
            gpio_set_level(25, false);
            gpio_set_level(27, false);
            gpio_set_level(33, false);
        }
        else if (i == 8) {
            gpio_set_level(26, false);
            gpio_set_level(25, true);
            gpio_set_level(27, true);
            gpio_set_level(33, true);
        }
        else if (i == 9) {
            gpio_set_level(26, false);
            gpio_set_level(25, true);
            gpio_set_level(27, true);
            gpio_set_level(33, false);
        }
        else if (i == 10) {
            gpio_set_level(26, false);
            gpio_set_level(25, true);
            gpio_set_level(27, false);
            gpio_set_level(33, true);
        }
        else if (i == 11) {
            gpio_set_level(26, false);
            gpio_set_level(25, true);
            gpio_set_level(27, false);
            gpio_set_level(33, false);
        }
        else if (i == 12) {
            gpio_set_level(26, false);
            gpio_set_level(25, false);
            gpio_set_level(27, true);
            gpio_set_level(33, true);
        }
        else if (i == 13) {
            gpio_set_level(26, false);
            gpio_set_level(25, false);
            gpio_set_level(27, true);
            gpio_set_level(33, false);
        }
        else if (i == 14) {
            gpio_set_level(26, false);
            gpio_set_level(25, false);
            gpio_set_level(27, false);
            gpio_set_level(33, true);
        }
        else if (i == 15) {
            gpio_set_level(26, false);
            gpio_set_level(25, false);
            gpio_set_level(27, false);
            gpio_set_level(33, false);
            up = !up;
        }

        if(up){
            i++;
        }else{
            i--;
        }
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
// Hardware interrupt definitions
#define GPIO_INPUT_IO_1       4
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL    1ULL<<GPIO_INPUT_IO_1 // casting GPIO input to bitmap

int flag = 0;     // Global flag for signaling from ISR

// Define button interrupt handler -- just sets the flag on interrupt
static void IRAM_ATTR gpio_isr_handler(void* arg){
  flag = 1;
}


// Intialize the GPIO to detect button press as interrupt
static void button_init() {
  gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE; // interrupt of rising edge
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL; // bit mask of the pins, use GPIO4 here
    io_conf.mode = GPIO_MODE_INPUT;            // set as input mode
    io_conf.pull_up_en = 1;                    // enable resistor pull-up mode on pin
  gpio_config(&io_conf);                       // apply parameters
  gpio_intr_enable(GPIO_INPUT_IO_1 );          // enable interrupts on pin
  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);   //install gpio isr service
  gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1); //hook isr handler for specific gpio pin
}

void Button_task_dont(){
    while(1) {                               // loop forever in this task
        if(flag) {
            up=!up;
            flag = 0;                            // set the flag to false
        }
    }
}

void blue_task_does(){
    configure_led(15);
    while(1){
        gpio_set_level(15, up);
    }
}
TaskHandle_t blink_task_handle = NULL;
TaskHandle_t button_task_handle = NULL;
TaskHandle_t blue_task_handle = NULL;

void app_main(void)
{    
    button_init();
    xTaskCreate(Button_task_dont,"Button_task_dont", 4096, NULL, 5, NULL);
    xTaskCreate(blink_task_does,"blink_task_handle", 4096, NULL, 5, NULL);
    xTaskCreate(blue_task_does,"blue_task_handle", 4096, NULL, 5, NULL);

}
