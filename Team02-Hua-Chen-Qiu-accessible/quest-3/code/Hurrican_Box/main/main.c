/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include <stdbool.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_adc_cal.h"
#include "protocol_examples_common.h"
#include "driver/i2c.h"
#include <string.h>
#include <math.h>
#include "./ADXL343.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "addr_from_stdin.h"




const int GPIO_0 = 12;
int LED_LEVEL = 1;
#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR "192.168.1.137"
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR "192.168.1.137"
#else
#define HOST_IP_ADDR "192.168.1.137"
#endif

#define PORT 3333
#define DEFAULT_VREF 1100 // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 128 // Multisampling

static const char *TAG = "example";
static char payload[500] = "Message from ESP32 ";

// Master I2C
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
#define NACK_VAL 0x01                       // i2c nack value (Was FF)

// ADXL343
#define SLAVE_ADDR ADXL343_ADDRESS // 0x53

// thermistor
static esp_adc_cal_characteristics_t *thermistor_adc_chars;
static const adc_channel_t thermistor_channel = ADC_CHANNEL_3; // GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t thermistor_atten = ADC_ATTEN_DB_11;
static const adc_unit_t thermistor_unit = ADC_UNIT_1;

// bettery voltage
static esp_adc_cal_characteristics_t *bettery_adc_chars;
static const adc_channel_t bettery_channel = ADC_CHANNEL_0; // GPIO39 if ADC1, GPIO15 if ADC2
static const adc_atten_t bettery_atten = ADC_ATTEN_DB_11;
static const adc_unit_t bettery_unit = ADC_UNIT_1;

float thermistor_T = 0.0;
float bettery_percent = 0.0;
float acc_x, acc_y, acc_z;
float roll, pitch;
int sensor_delay = 10; // ms


static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    while (1) {

#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
        struct sockaddr_in6 dest_addr = { 0 };
        inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_storage dest_addr = { 0 };
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_DGRAM, &ip_protocol, &addr_family, &dest_addr));
#endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        while (1) {

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent");
            
            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);

            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG, "%s", rx_buffer);
                LED_LEVEL = atoi(rx_buffer);
                if (strncmp(rx_buffer, "OK: ", 4) == 0) {
                    ESP_LOGI(TAG, "Received expected message, reconnecting");
                    break;
                }
            }
            
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void led_task(){
    while(1){
            gpio_set_level(GPIO_0,LED_LEVEL);
            vTaskDelay(1000/ portTICK_PERIOD_MS);
    }
}


///thermister accelerometor battery
// Function to initiate i2c -- note the MSB declaration!
static void i2c_master_init()
{
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
    conf.clk_flags = 0;                                 // Optional, you can use a clock flag
    err = i2c_param_config(i2c_master_port, &conf);     // Configure
    if (err == ESP_OK)
    {
        printf("- parameters: ok\n");
    }

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                             I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                             I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    if (err == ESP_OK)
    {
        printf("- initialized: yes\n");
    }

    // Data in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility  Functions //////////////////////////////////////////////////////////

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Utility function to scan for i2c device
static void i2c_scanner()
{
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
    {
        printf("- No I2C devices found!"
               "\n");
    }
}

////////////////////////////////////////////////////////////////////////////////

// ADXL343 Functions ///////////////////////////////////////////////////////////

// Get Device ID
int getDeviceID(uint8_t *data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                              // 1. Start
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);            // 2.-3.
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);                                      // 4.-5.
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);                                     // 4.-5.
    i2c_master_stop(cmd);                                                               // 11. Stop
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS); // This starts the I2C communication
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Read register
uint8_t readRegister(uint8_t reg)
{
    // YOUR CODE HERE
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                        // 1. Start
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);      // 2.-3.
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);                                // 4.-5.
    i2c_master_start(cmd);                                                        // 6.
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);       // 7.-8.
    i2c_master_read_byte(cmd, &data, ACK_CHECK_DIS);                              // 9.-10.
    i2c_master_stop(cmd);                                                         // 11. Stop
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS); // This starts the I2C communication
    i2c_cmd_link_delete(cmd);
    return data;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg)
{
    // YOUR CODE HERE
    uint8_t data[2];
    data[0] = readRegister(reg);
    data[1] = readRegister(reg + 1);
    return (int16_t)(data[0] | (data[1] << 8));
}

void setRange(range_t range)
{
    /* Red the data format register to preserve bits */
    uint8_t format = readRegister(ADXL343_REG_DATA_FORMAT);

    /* Update the data rate */
    format &= ~0x0F;
    format |= range;

    /* Make sure that the FULL-RES bit is enabled for range scaling */
    format |= 0x08;

    /* Write the register back to the IC */
    writeRegister(ADXL343_REG_DATA_FORMAT, format);
}

range_t getRange(void)
{
    /* Red the data format register to preserve bits */
    return (range_t)(readRegister(ADXL343_REG_DATA_FORMAT) & 0x03);
}

dataRate_t getDataRate(void)
{
    return (dataRate_t)(readRegister(ADXL343_REG_BW_RATE) & 0x0F);
}

// function to get acceleration
void getAccel(float *xp, float *yp, float *zp)
{
    *xp = read16(ADXL343_REG_DATAX0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    *yp = read16(ADXL343_REG_DATAY0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    *zp = read16(ADXL343_REG_DATAZ0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
     printf("X: %.2f \t Y: %.2f \t Z: %.2f\n", *xp, *yp, *zp);
    acc_x = *xp;
    acc_y = *yp;
    acc_z = *zp;
}

// function to print roll and pitch
// explanation of the math: https://www.nxp.com/docs/en/application-note/AN3461.pdf
void calcRP(float x, float y, float z)
{
    // YOUR CODE HERE
    roll = atan2(y, z) * 180 / M_PI;
    pitch = atan2((-x), sqrt(y * y + z * z)) * 180 / M_PI;
    // printf("roll: %.2f \t pitch: %.2f \n", roll, pitch);
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

// tasks
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

static void task_bettery_level()
{
    // Configure ADC
    if (bettery_unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(bettery_channel, bettery_atten);
    }
    else
    {
        adc2_config_channel_atten((adc2_channel_t)bettery_channel, bettery_atten);
    }

    // Characterize ADC
    bettery_adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(bettery_unit, bettery_atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, bettery_adc_chars);
    print_char_val_type(val_type);

    // Continuously sample ADC1
    while (1)
    {
        uint32_t adc_reading = 0;
        // Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            if (bettery_unit == ADC_UNIT_1)
            {
                adc_reading += adc1_get_raw((adc1_channel_t)bettery_channel);
            }
            else
            {
                int raw;
                adc2_get_raw((adc2_channel_t)bettery_channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, bettery_adc_chars);
        float bettery_voltage = voltage / 1000.0;
        bettery_percent = (bettery_voltage / 3.1) * 100.0;
        // bettery_percent = bettery_voltage;
        vTaskDelay(pdMS_TO_TICKS(sensor_delay));
    }
}

static void test_adxl343()
{
    while (1)
    {
        float xVal, yVal, zVal;
        getAccel(&xVal, &yVal, &zVal);
        calcRP(xVal, yVal, zVal);
        vTaskDelay(500 / portTICK_PERIOD_MS);
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
        sprintf(payload, "Time:%.2fs , Thermistor: %.2fC , Battery: %.2f%% ,Accel X: %.2fm/s^2 , Accel Y: %.2fm/s^2  , Accel Z: %.2fm/s^2  ,Roll: %.2f° , Pitch: %.2f°",
            time_sec, thermistor_T, bettery_percent, acc_x, acc_y, acc_z, roll, pitch);

        /*
        printf("Time: %.2fs\t", time_sec);
        printf("Thermistor: %.2fC\t", thermistor_T);
        printf("Bettery: %.2f%%\t", bettery_percent);
        printf("Accel X: %.2fm/s^2\t", acc_x);
        printf("Accel Y: %.2fm/s^2\t", acc_y);
        printf("Accel Z: %.2fm/s^2\t", acc_z);
        printf("Roll: %.2f°\t", roll);
        printf("Pitch: %.2f°\n", pitch);
        */
        vTaskDelay(pdMS_TO_TICKS(delay));
    }
}

// init
void init_accelerometer()
{
    // Check for ADXL343
    uint8_t deviceID;
    getDeviceID(&deviceID);
    if (deviceID == 0xE5)
    {
        printf("\n>> Found ADAXL343\n");
    }

    // Disable interrupts
    writeRegister(ADXL343_REG_INT_ENABLE, 0);

    // Set range
    setRange(ADXL343_RANGE_16_G);
    // Display range
    printf("- Range:         +/- ");
    switch (getRange())
    {
    case ADXL343_RANGE_16_G:
        printf("16 ");
        break;
    case ADXL343_RANGE_8_G:
        printf("8 ");
        break;
    case ADXL343_RANGE_4_G:
        printf("4 ");
        break;
    case ADXL343_RANGE_2_G:
        printf("2 ");
        break;
    default:
        printf("?? ");
        break;
    }
    printf(" g\n");

    // Display data rate
    printf("- Data Rate:    ");
    switch (getDataRate())
    {
    case ADXL343_DATARATE_3200_HZ:
        printf("3200 ");
        break;
    case ADXL343_DATARATE_1600_HZ:
        printf("1600 ");
        break;
    case ADXL343_DATARATE_800_HZ:
        printf("800 ");
        break;
    case ADXL343_DATARATE_400_HZ:
        printf("400 ");
        break;
    case ADXL343_DATARATE_200_HZ:
        printf("200 ");
        break;
    case ADXL343_DATARATE_100_HZ:
        printf("100 ");
        break;
    case ADXL343_DATARATE_50_HZ:
        printf("50 ");
        break;
    case ADXL343_DATARATE_25_HZ:
        printf("25 ");
        break;
    case ADXL343_DATARATE_12_5_HZ:
        printf("12.5 ");
        break;
    case ADXL343_DATARATE_6_25HZ:
        printf("6.25 ");
        break;
    case ADXL343_DATARATE_3_13_HZ:
        printf("3.13 ");
        break;
    case ADXL343_DATARATE_1_56_HZ:
        printf("1.56 ");
        break;
    case ADXL343_DATARATE_0_78_HZ:
        printf("0.78 ");
        break;
    case ADXL343_DATARATE_0_39_HZ:
        printf("0.39 ");
        break;
    case ADXL343_DATARATE_0_20_HZ:
        printf("0.20 ");
        break;
    case ADXL343_DATARATE_0_10_HZ:
        printf("0.10 ");
        break;
    default:
        printf("???? ");
        break;
    }


    writeRegister(ADXL343_REG_POWER_CTL, 0x08);
    printf(" Hz\n\n");

}

void app_main(void)
{   
    //intiliaze LED
    gpio_reset_pin(GPIO_0);
    gpio_set_direction(GPIO_0,GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ///
    i2c_master_init();
    i2c_scanner();
    // Check if Two Point or Vref are burned into eFuse
    check_efuse();

    // init accelerometer
    init_accelerometer();
    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    xTaskCreate(led_task, "LED_client",4096,NULL,5,NULL );
    xTaskCreate(task_thermistor, "task_thermistor", 4096, NULL, 5, NULL);
    xTaskCreate(task_bettery_level, "task_IR_range_sensor", 4096, NULL, 5, NULL);
    xTaskCreate(test_adxl343, "test_adxl343", 4096, NULL, 5, NULL);
    xTaskCreate(task_reporting, "task_reporting", 4096, NULL, 5, NULL);
}