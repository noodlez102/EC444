#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"

// components: servo+ display + RTOS + GPTimers + (UART)
#include "driver/mcpwm_prelude.h"
#include "driver/i2c.h"
#include "driver/gptimer.h"
#include "driver/uart.h"

// servo settings
#define SERVO_MIN_PULSEWIDTH_US_1 100  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US_1 2700 // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE -90                 // Minimum angle
#define SERVO_MAX_DEGREE 90                  // Maximum angle
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD 20000          // 20000 ticks, 20ms

// uart settings
#define EX_UART_NUM UART_NUM_0
#define ECHO_TEST_TXD (UART_PIN_NO_CHANGE)
#define ECHO_TEST_RXD (UART_PIN_NO_CHANGE)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)
#define BUF_SIZE (1024)

// display settings
#define SLAVE_ADDR 0x70              // alphanumeric address
#define OSC 0x21                     // oscillator cmd
#define HT16K33_BLINK_DISPLAYON 0x01 // Display on cmd
#define HT16K33_BLINK_OFF 0          // Blink off cmd
#define HT16K33_BLINK_CMD 0x80       // Blink cmd
#define HT16K33_CMD_BRIGHTNESS 0xE0  // Brightness cmd

// I2C settings for display
#define I2C_EXAMPLE_MASTER_SCL_IO 22        // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO 23        // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_0    // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE 0 // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE 0 // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ 100000   // i2c master clock freq
#define WRITE_BIT I2C_MASTER_WRITE          // i2c master write
#define READ_BIT I2C_MASTER_READ            // i2c master read
#define ACK_CHECK_EN true                   // i2c master will check ack
#define ACK_CHECK_DIS false                 // i2c master will not check ack
#define ACK_VAL 0x00                        // i2c ack value
#define NACK_VAL 0xFF                       // i2c nack value

static const char *TAG = "robot";
static const char *TAG_TIMER = "robot: timer";
static const uint16_t alphafonttable[] = {

    0b0000000000000001,
    0b0000000000000010,
    0b0000000000000100,
    0b0000000000001000,
    0b0000000000010000,
    0b0000000000100000,
    0b0000000001000000,
    0b0000000010000000,
    0b0000000100000000,
    0b0000001000000000,
    0b0000010000000000,
    0b0000100000000000,
    0b0001000000000000,
    0b0010000000000000,
    0b0100000000000000,
    0b1000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0001001011001001,
    0b0001010111000000,
    0b0001001011111001,
    0b0000000011100011,
    0b0000010100110000,
    0b0001001011001000,
    0b0011101000000000,
    0b0001011100000000,
    0b0000000000000000, //
    0b0000000000000110, // !
    0b0000001000100000, // "
    0b0001001011001110, // #
    0b0001001011101101, // $
    0b0000110000100100, // %
    0b0010001101011101, // &
    0b0000010000000000, // '
    0b0010010000000000, // (
    0b0000100100000000, // )
    0b0011111111000000, // *
    0b0001001011000000, // +
    0b0000100000000000, // ,
    0b0000000011000000, // -
    0b0100000000000000, // .
    0b0000110000000000, // /
    0b0000110000111111, // 0
    0b0000000000000110, // 1
    0b0000000011011011, // 2
    0b0000000010001111, // 3
    0b0000000011100110, // 4
    0b0010000001101001, // 5
    0b0000000011111101, // 6
    0b0000000000000111, // 7
    0b0000000011111111, // 8
    0b0000000011101111, // 9
    0b0001001000000000, // :
    0b0000101000000000, // ;
    0b0010010000000000, // <
    0b0000000011001000, // =
    0b0000100100000000, // >
    0b0001000010000011, // ?
    0b0000001010111011, // @
    0b0000000011110111, // A
    0b0001001010001111, // B
    0b0000000000111001, // C
    0b0001001000001111, // D
    0b0000000011111001, // E
    0b0000000001110001, // F
    0b0000000010111101, // G
    0b0000000011110110, // H
    0b0001001000001001, // I
    0b0000000000011110, // J
    0b0010010001110000, // K
    0b0000000000111000, // L
    0b0000010100110110, // M
    0b0010000100110110, // N
    0b0000000000111111, // O
    0b0000000011110011, // P
    0b0010000000111111, // Q
    0b0010000011110011, // R
    0b0000000011101101, // S
    0b0001001000000001, // T
    0b0000000000111110, // U
    0b0000110000110000, // V
    0b0010100000110110, // W
    0b0010110100000000, // X
    0b0001010100000000, // Y
    0b0000110000001001, // Z
    0b0000000000111001, // [
    0b0010000100000000, //
    0b0000000000001111, // ]
    0b0000110000000011, // ^
    0b0000000000001000, // _
    0b0000000100000000, // `
    0b0001000001011000, // a
    0b0010000001111000, // b
    0b0000000011011000, // c
    0b0000100010001110, // d
    0b0000100001011000, // e
    0b0000000001110001, // f
    0b0000010010001110, // g
    0b0001000001110000, // h
    0b0001000000000000, // i
    0b0000000000001110, // j
    0b0011011000000000, // k
    0b0000000000110000, // l
    0b0001000011010100, // m
    0b0001000001010000, // n
    0b0000000011011100, // o
    0b0000000101110000, // p
    0b0000010010000110, // q
    0b0000000001010000, // r
    0b0010000010001000, // s
    0b0000000001111000, // t
    0b0000000000011100, // u
    0b0010000000000100, // v
    0b0010100000010100, // w
    0b0010100011000000, // x
    0b0010000000001100, // y
    0b0000100001001000, // z
    0b0000100101001001, // {
    0b0001001000000000, // |
    0b0010010010001001, // }
    0b0000010100100000, // ~
    0b0011111111111111

};
typedef struct
{
    uint64_t event_count;
} example_queue_element_t;
bool flag = false;
example_queue_element_t ele;
QueueHandle_t timer_queue;
int alarm_interval = 1; // sec
int servo1_gpio_pin = 32;
int servo2_gpio_pin = 33;
int up_button_gpio_pin = 21;
int down_button_gpio_pin = 17;
int switch_button_gpio_pin = 16;
int led_pin = 27;

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

// UART functions
static void uart_init()
{
    /* Configure parameters of the UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(EX_UART_NUM, &uart_config);
    uart_set_pin(EX_UART_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
}

void input(char jump_c, char text[], int str_len)
{
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    int count = 0;
    char c;

    // char text[9] = "";
    // int str_len = 8;

    for (int i = 0; i < str_len; i++)
    {
        text[i] = '\0';
    }
    printf("Enter here (max = %d) \n", str_len);
    while (1)
    {
        int len = uart_read_bytes(EX_UART_NUM, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0)
        {
            c = data[0];
            if (c == jump_c)
            {
                break;
            }
            if (count < str_len)
            {
                text[count] = c;
                count++;
            }
            printf("Enter %d Characters: %s\n", count, text);
        }
    }
    printf("Confirm %d Characters: %s\n", count, text);
    free(data);
}

// GPTimeer functions

static bool IRAM_ATTR timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{ // Timer interrupt handler -- callback timer function -- from GPTimer guide example
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t timer_queue1 = (QueueHandle_t)user_data; // represents state info passed to callback, if needed
    example_queue_element_t ele = {
        .event_count = edata->count_value // Retrieve count value and send to queue
    };
    flag = true;
    xQueueSendFromISR(timer_queue1, &ele, &high_task_awoken); // Puts data into queue and alerts other recipients
    return (high_task_awoken == pdTRUE);                      // pdTRUE indicates data posted successfully
}

static void alarm_init()
{ // Timer configuration -- from GPTimer guide example
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer)); // instantiates timer

    gptimer_event_callbacks_t cbs = {
        // Set alarm callback
        .on_alarm = timer_on_alarm_cb, // This is a specific supported callback from callbacks list
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, timer_queue)); // This registers the callback
    ESP_ERROR_CHECK(gptimer_enable(gptimer));                                      // Enables timer interrupt ISR

    ESP_LOGI(TAG_TIMER, "Start timer, update alarm value dynamically and auto reload");
    gptimer_alarm_config_t alarm_config = {
        // Configure the alarm
        .reload_count = 0, // counter will reload with 0 on alarm event
        .alarm_count = alarm_interval * 1000000,
        .flags.auto_reload_on_alarm = true, // enable auto-reload
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config)); // this enacts the alarm config
    ESP_ERROR_CHECK(gptimer_start(gptimer));                           // this starts the timer
}

// Display + I2C functions
static void i2c_example_master_init()
{ // Function to initiate i2c -- note the MSB declaration!
    // Debug
    printf("\n>> i2c Config\n");
    int err;

    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                        // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;        // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;        // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ; // CLK frequency
    conf.clk_flags = 0;                                 // <-- UNCOMMENT IF YOU GET ERRORS (see readme.md)
    err = i2c_param_config(i2c_master_port, &conf);     // Configure
    if (err == ESP_OK)
    {
        printf("- parameters: ok\n");
    }

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                             I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                             I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
    if (err == ESP_OK)
    {
        printf("- initialized: yes\n\n");
    }

    // Dat in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

int testConnection(uint8_t devAddr, int32_t timeout)
{ // Utility function to test for I2C device address -- not used in deploy
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

static void i2c_scanner()
{ // Utility function to scan for i2c device
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."
           "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++)
    {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK)
        {
            printf("- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0)
        printf("- No I2C devices found!"
               "\n");
    printf("\n");
}

int alpha_oscillator()
{ // Turn on oscillator for alpha display
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    return ret;
}

int no_blink()
{ // Set blink rate to off
    int ret;
    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
    i2c_master_start(cmd2);
    i2c_master_write_byte(cmd2, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
    i2c_master_stop(cmd2);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd2);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    return ret;
}

int set_brightness_max(uint8_t val)
{ // Set Brightness
    int ret;
    i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
    i2c_master_start(cmd3);
    i2c_master_write_byte(cmd3, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
    i2c_master_stop(cmd3);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd3);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    return ret;
}

int display(char text[])
{
    int ret = 0;
    int count = strlen(text);
    int delay = 150;
    static int prev_count = 0;
    static uint16_t displaybuffer[8] = {0};

    if (count != prev_count)
    {
        for (int i = 0; i < 8; i++)
        {
            displaybuffer[i] = 0;
        }
        prev_count = count;
    }

    for (int i = 0; i <= count; i++) // set <= because we want to clear the display at the end of scrolling
    {
        if (count <= 4)
        {
            if (i == count)
            {
                displaybuffer[i] = 0;
            }
            else
            {
                displaybuffer[i] = alphafonttable[text[i] - 0];
            }
            delay = 0;
        }
        else
        {
            for (int j = 0; j < 4; j++)
            {
                int k = i + j;
                if (k < count)
                {
                    displaybuffer[j] = alphafonttable[text[k] - 0];
                }
                else
                {
                    displaybuffer[j] = 0;
                }
            }
        }

        // printf("time: %d\n", time);
        // Send commands characters to display over I2C
        i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
        i2c_master_start(cmd4);
        i2c_master_write_byte(cmd4, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
        for (uint8_t i = 0; i < 8; i++)
        {
            i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
            i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
        }
        i2c_master_stop(cmd4);
        ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd4);
        vTaskDelay(delay / portTICK_PERIOD_MS);
    }
    return ret;
}

static void display_init()
{
    i2c_example_master_init();
    i2c_scanner();

    // Debug
    int ret;

    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if (ret == ESP_OK)
    {
        printf("- oscillator: ok \n");
    }
    // Set display blink off
    ret = no_blink();
    if (ret == ESP_OK)
    {
        printf("- blink: off \n");
    }
    ret = set_brightness_max(0xF);
    if (ret == ESP_OK)
    {
        printf("- brightness: max \n");
    }
    display("");
}

// GPIO functions
static void gpio_init()
{
    gpio_reset_pin(up_button_gpio_pin);
    gpio_set_direction(up_button_gpio_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(up_button_gpio_pin, GPIO_PULLUP_ONLY);
    gpio_reset_pin(down_button_gpio_pin);
    gpio_set_direction(down_button_gpio_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(down_button_gpio_pin, GPIO_PULLUP_ONLY);
    gpio_reset_pin(switch_button_gpio_pin);
    gpio_set_direction(switch_button_gpio_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(switch_button_gpio_pin, GPIO_PULLUP_ONLY);
    gpio_reset_pin(led_pin);
    gpio_set_direction(led_pin, GPIO_MODE_OUTPUT);
}

// Main Program

// variables
int current_time = 0; // sec
int target_time = 0;  // sec
int steps = 3;
bool walking = false;
int angle = 30;
int mode = 0; // 0: running, 1: set_hour, 2: set_minute, 3: set_second
int set_hour = 0;
int set_minute = 0;
int set_second = 0;
int hour_range[] = {0, 23};
int minute_range[] = {0, 59};
int second_range[] = {0, 59};
int long_press_time = 1000; // ms

mcpwm_cmpr_handle_t servo1_comparator;
mcpwm_cmpr_handle_t servo2_comparator;

// helper functions
void walk(int steps, int angle)
{
    walking = true;
    for (int i = 0; i < steps; i++)
    {
        printf("Step %d\n", i + 1);
        
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(servo1_comparator, example_angle_to_compare(angle)));
        vTaskDelay(200 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(servo2_comparator, example_angle_to_compare(-angle)));
        vTaskDelay(200 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(servo1_comparator, example_angle_to_compare(-angle)));
        vTaskDelay(200 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(servo2_comparator, example_angle_to_compare(angle)));
        vTaskDelay(200 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(servo1_comparator, example_angle_to_compare(0)));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(servo2_comparator, example_angle_to_compare(0)));

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    walking = false;
}

// Tasks
static void task_main(void *arg)
{
    while (1)
    {
        if (mode == 0)
        {
            if (flag && !walking)
            {
                printf("Time: %ds\n", current_time);
                flag = false;
                if (current_time <= 1)
                {
                    walk(steps, angle);
                    current_time = target_time;
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                else
                {
                    current_time--;
                }
            }
        }
    }
}

static void task_display(void *arg)
{
    char text[1024] = "";
    while (1)
    {
        if (walking)
        {
            strcpy(text, "WALK");
        }
        else if (mode == 0)
        {
            // sprintf(text, "%d", current_time);
            int current_hour = current_time / 3600;
            int current_minute = (current_time % 3600) / 60;
            int current_second = current_time % 60;
            if (current_hour != 0)
            {
                sprintf(text, "%dh%dm%ds", current_hour, current_minute, current_second);
            }
            else if (current_minute != 0)
            {
                sprintf(text, "%dm%ds", current_minute, current_second);
            }
            else
            {
                sprintf(text, "%ds", current_second);
            }
        }
        else if (mode == 1)
        {
            sprintf(text, "%2dh", set_hour);
        }
        else if (mode == 2)
        {
            sprintf(text, "%2dm", set_minute);
        }
        else if (mode == 3)
        {
            sprintf(text, "%2ds", set_second);
        }
        display(text);
    }
}

// static void task_evt_time(void *arg)
// {
//     while (1)
//     {
//         // Transfer from queue and do something if triggered
//         // xQueueReceive() returns pdTRUE if data was successfully received from the queue, otherwise pdFALSE.
//         // it has three parameters: the queue to receive from, the buffer to receive data into, and the maximum time to wait for data to be available.
//         if (xQueueReceive(timer_queue, &ele, pdMS_TO_TICKS(2000)))
//         {
//             flag = true; // Set a flag to be used elsewhere
//         }
//     }
// }

static void task_switch_button()
{
    int time = 0;
    while (1)
    {
        // printf("Switch button: %d\n", gpio_get_level(switch_button_gpio_pin));
        if (gpio_get_level(switch_button_gpio_pin) == 0)
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            time++;
            if (time > long_press_time / 100 && mode == 0)
            {
                printf("press time: %ds\n", time / 10);
                gpio_set_level(led_pin, 1);
                current_time = target_time;
                flag = false;
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                if (gpio_get_level(up_button_gpio_pin) != 0)
                {
                    gpio_set_level(led_pin, 0);
                    time = 0;
                }
            }
            else if (gpio_get_level(switch_button_gpio_pin) != 0)
            {
                switch (mode)
                {
                case 0:
                    mode = 1;
                    break;
                case 1:
                    mode = 2;
                    break;
                case 2:
                    mode = 3;
                    break;
                case 3:
                    int new_target_time = set_hour * 3600 + set_minute * 60 + set_second;
                    if (target_time != new_target_time)
                    {
                        target_time = new_target_time;
                        current_time = target_time;
                    }
                    // vTaskDelay(1000 / portTICK_PERIOD_MS);
                    flag = false;
                    mode = 0;
                    break;
                default:
                    mode = 0;
                    break;
                }
                printf("Switch Mode: %d\n", mode);
            }
        }
    }
}

static void task_up_button()
{
    int time = 0;
    while (1)
    {
        // printf("Up button: %d\n", gpio_get_level(up_button_gpio_pin));
        if (gpio_get_level(up_button_gpio_pin) == 0)
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            time++;
            if (gpio_get_level(up_button_gpio_pin) != 0 || time > long_press_time / 100)
            {
                // printf("Up button pressed\n");
                if (mode == 1 && set_hour < hour_range[1])
                {
                    set_hour++;
                }
                else if (mode == 2 && set_minute < minute_range[1])
                {
                    set_minute++;
                }
                else if (mode == 3 && set_second < second_range[1])
                {
                    set_second++;
                }
                if (gpio_get_level(up_button_gpio_pin) != 0)
                {
                    time = 0;
                }
            }
        }
    }
}

static void task_down_button()
{
    int time = 0;
    while (1)
    {
        // printf("Down button: %d\n", gpio_get_level(down_button_gpio_pin));
        if (gpio_get_level(down_button_gpio_pin) == 0)
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            time++;
            if (gpio_get_level(down_button_gpio_pin) != 0 || time > long_press_time / 100)
            {
                // printf("Down button pressed\n");
                if (mode == 1 && set_hour > hour_range[0])
                {
                    set_hour--;
                }
                else if (mode == 2 && set_minute > minute_range[0])
                {
                    set_minute--;
                }
                else if (mode == 3 && set_second > second_range[0])
                {
                    set_second--;
                }
                if (gpio_get_level(down_button_gpio_pin) != 0)
                {
                    time = 0;
                }
            }
        }
    }
}

void app_main(void)
{
    // Timer queue initialize
    timer_queue = xQueueCreate(10, sizeof(example_queue_element_t));
    if (!timer_queue)
    {
        ESP_LOGE(TAG_TIMER, "Creating queue failed");
        return;
    }
    servo1_comparator = servo_init(servo1_gpio_pin);
    servo2_comparator = servo_init(servo2_gpio_pin);
    uart_init();
    display_init();
    alarm_init();
    gpio_init();
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(servo1_comparator, example_angle_to_compare(0)));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(servo2_comparator, example_angle_to_compare(0)));
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // initialize time
    set_second = 5;
    target_time = set_hour * 3600 + set_minute * 60 + set_second;
    current_time = target_time;

    //xTaskCreate(task_evt_time, "task_evt_time", 2048, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(task_main, "task_main", 2048, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(task_display, "task_display", 2048, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(task_switch_button, "task_switch_button", 2048, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(task_up_button, "task_up_button", 2048, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(task_down_button, "task_down_button", 2048, NULL, tskIDLE_PRIORITY, NULL);
}
