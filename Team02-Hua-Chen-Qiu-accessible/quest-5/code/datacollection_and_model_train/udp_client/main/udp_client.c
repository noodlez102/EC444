/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include <stdlib.h>
#include <stdio.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "driver/mcpwm_prelude.h"
#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include <sys/types.h>
#include <sys/wait.h>
#include "driver/gpio.h"
#include "sdkconfig.h"
// Consult the ESC before you change these parameters for the buggy speed control

// Please consult the datasheet of your servo before changing the following parameters
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle

#define SERVO_PULSE_GPIO             4 // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

#define SERVO_PULSE_GPIO_STEER            25
// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

// Lidate Lite v4
#define LIDAR_OLD 0x62 // This is the orignal I2C address of the Garmin Lidar Lite (v4)
#define LIDAR_NEW 0x24 // Put new address here

static void i2c_master_init(){
  // Debug
  printf("\n>> i2c Config\n");
  int err;

  // Port configuration
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

  /// Define I2C configurations
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;                              // Master mode
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
  conf.clk_flags = 0;   
  err = i2c_param_config(i2c_master_port, &conf);           // Configure
  if (err == ESP_OK) {printf("- parameters: ok\n");}

  // Install I2C driver
  err = i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
  if (err == ESP_OK) {printf("- initialized: yes\n");}

  // Data in MSB mode
  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility function to test for I2C device address
int testConnection(uint8_t devAddr, int32_t timeout) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
  int32_t scanTimeout = 1000;
  printf("\n>> I2C scanning ..."  "\n");
  uint8_t count = 0;
  for (uint8_t i = 1; i < 127; i++) {
    // printf("0x%X%s",i,"\n");
    if (testConnection(i, scanTimeout) == ESP_OK) {
      printf( "- Device found at address: 0x%X%s", i, "\n");
      count++;
    }
  }
  if (count == 0) {printf("- No I2C devices found!" "\n");}
}


// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data, uint8_t i2cAddr) {
    //adapted from esp idf i2c_simple_main program
    uint8_t write_buf[2] = {reg, data};
    int ret;
    ret = i2c_master_write_to_device(I2C_EXAMPLE_MASTER_NUM, i2cAddr, write_buf, sizeof(write_buf), 1000/ portTICK_PERIOD_MS);
    return ret;
}

// Read register
uint8_t readRegister(uint8_t reg, uint8_t i2cAddr) {
    //adapted from esp idf i2c_simple_main program
    uint8_t data;
    i2c_master_write_read_device(I2C_EXAMPLE_MASTER_NUM, i2cAddr, &reg, 1, &data, 1, 1000 / portTICK_PERIOD_MS);
    return data;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg, uint8_t lidarliteaddress) {
  int16_t data, MSB, LSB;
  LSB = readRegister(reg, lidarliteaddress);
  MSB = readRegister(reg + 1, lidarliteaddress);
  data = MSB;
  data = LSB | (MSB << 8);
  return data;
}

static void change() {
    uint8_t dataBytes[5];
    dataBytes[0] = 0x11;
    writeRegister(0xEA, dataBytes[0], LIDAR_OLD);
    vTaskDelay(100/portTICK_PERIOD_MS);

    // serial number
    uint8_t reg = 0x16;
    i2c_master_write_read_device(I2C_EXAMPLE_MASTER_NUM, LIDAR_OLD, &reg, 1, dataBytes, 5, 1000 / portTICK_PERIOD_MS);

    uint8_t write_buf[6] = {reg, 0, 0, 0, 0, 0};
    dataBytes[4] = LIDAR_NEW;
    for (int i = 0; i < 5; i ++) {
        write_buf[i + 1] = dataBytes[i];
    }

    i2c_master_write_to_device(I2C_EXAMPLE_MASTER_NUM, LIDAR_OLD, write_buf, sizeof(write_buf), 1000/ portTICK_PERIOD_MS);
    vTaskDelay(100/portTICK_PERIOD_MS);

    // disable default
    dataBytes[0] = 0x01; // set bit to disable default address
    writeRegister(0x1b, dataBytes[0], LIDAR_NEW);

    // Wait for the I2C peripheral to be restarted with new device address
    vTaskDelay(100/portTICK_PERIOD_MS);

    // dataBytes[0] = 0;
    writeRegister(0xEA, dataBytes[0], LIDAR_NEW);
    vTaskDelay(100/portTICK_PERIOD_MS);
    
}

mcpwm_cmpr_handle_t comparator = NULL;         // Create PWM comparator
mcpwm_cmpr_handle_t comparator_steer = NULL;  



static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US)
      / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}


#ifdef CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN
#include "addr_from_stdin.h"
#endif

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR "192.168.1.137"
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR "192.168.1.137"
#else
#define HOST_IP_ADDR "192.168.1.137"
#endif

#define PORT 3333

static const char *TAG = "example";
static char payload[500] = "0,0,0,0";
char drivingvar[128];

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

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

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
                strcpy(drivingvar,rx_buffer);
                if (strncmp(rx_buffer, "OK: ", 4) == 0) {
                    ESP_LOGI(TAG, "Received expected message, reconnecting");
                    break;
                }
            }

            vTaskDelay(50 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}


int angle_drive = 0;
int steer_angle = -7;
int16_t distance = 0;
void drive(){

    while(true){

        writeRegister(0x00, 0x04, 0x24); 
        
        distance = read16(0x10,0x24);
        printf("Distance: %d cm\n", distance);
        sprintf(payload, "%d,%d,%d",angle_drive,steer_angle,distance);

        if (strncmp(drivingvar,"0",1)==0){
                angle_drive =0;
            
        }
        if(strncmp(drivingvar,"1",1)==0){
            
                angle_drive = 10;
           
        }
        if(strncmp(drivingvar,"2",1)==0){
            ESP_LOGI(TAG,"steering");
            if(steer_angle < 50){
                steer_angle = 45;
            }
            
        }
        if(strncmp(drivingvar,"3",1)==0){
            if(steer_angle > -50){
                steer_angle = -45;
            }
        }
        if(strncmp(drivingvar,"4",1)==0){
            steer_angle = -15;
            angle_drive = angle_drive;
        }

         if(distance  <= 60 ){
            
             angle_drive = 0;
         }    
        ESP_LOGI(TAG,"%d",steer_angle);
        ESP_LOGI(TAG,"%d",angle_drive);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_steer, example_angle_to_compare(steer_angle)));
        vTaskDelay(pdMS_TO_TICKS(50));

        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle_drive)));
        vTaskDelay(pdMS_TO_TICKS(50));
    }   

}

void app_main(void)
{   ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;             // Create PWM timer
    mcpwm_timer_config_t timer_config = {          // Configure PWM timer 
      .group_id = 0,                               // Pick PWM group 0
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,      // Default clock source 
      .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ, // Hz
      .period_ticks = SERVO_TIMEBASE_PERIOD,       // Set servo period (20ms -- 50 Hz)
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,     // Count up
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;               // Create PWM operator 
    mcpwm_operator_config_t operator_config = {    // Configure PWM operator
      .group_id = 0,                               // operator same group and PWM timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");   // Connect PWM timer and PWM operator
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_comparator_config_t comparator_config = {// Updates when timer = zero
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator_steer));

    mcpwm_gen_handle_t generator = NULL;            // Create generator
    mcpwm_gen_handle_t generator_steer = NULL;  
    mcpwm_generator_config_t generator_config = {   // Output to GPIO pin 
        .gen_gpio_num = SERVO_PULSE_GPIO,
    };

    mcpwm_generator_config_t generator_config_steer = {   // Output to GPIO pin 
        .gen_gpio_num = SERVO_PULSE_GPIO_STEER,
    };
    i2c_master_init();
    i2c_scanner();
    vTaskDelay(30/portTICK_PERIOD_MS);
    change();
    i2c_scanner();
    


    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config_steer, &generator_steer));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_steer, example_angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_steer,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator_steer,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_steer, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));       // Enable
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP)); // Run continuously

    // This code drives the servo over a range
    // The PWM is changed slowly allow the (mechanical) servo to keep up 

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_steer, example_angle_to_compare(45)));
    vTaskDelay(pdMS_TO_TICKS(1000)); // Do for at least 3s, and leave in neutral state
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_steer, example_angle_to_compare(-45)));
    vTaskDelay(pdMS_TO_TICKS(1000)); // Do for at least 3s, and leave in neutral state
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_steer, example_angle_to_compare(-15)));
    vTaskDelay(pdMS_TO_TICKS(1000)); // Do for at least 3s, and leave in neutral state
    
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    xTaskCreate(drive, "drive", 4096, NULL, 5, NULL);
}
