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

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


#include "driver/mcpwm_prelude.h"
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
static const char *payload = "Message from ESP32 ";
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

            vTaskDelay(2000 / portTICK_PERIOD_MS);
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
int steer_angle = 0;
void drive(){

    while(true){
        if (strncmp(drivingvar,"0",1)==0){
            if(angle_drive > 0){
                angle_drive =0;
            }
        }
        if(strncmp(drivingvar,"1",1)==0){
            if(angle_drive < 15){
                angle_drive += 5;
            }
        }
        if(strncmp(drivingvar,"2",1)==0){
            ESP_LOGI(TAG,"steering");
            if(steer_angle < 90){
                steer_angle = steer_angle + 10;
            }
            
        }
        if(strncmp(drivingvar,"3",1)==0){
            if(steer_angle > -90){
                steer_angle = steer_angle - 10;
            }
        }
        if(strncmp(drivingvar,"4",1)==0){
            steer_angle = steer_angle;
            angle_drive = angle_drive;
        }
        ESP_LOGI(TAG,"%d",steer_angle);
        ESP_LOGI(TAG,"%d",angle_drive);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_steer, example_angle_to_compare(steer_angle)));
        vTaskDelay(pdMS_TO_TICKS(1000)); 
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle_drive)));
        vTaskDelay(pdMS_TO_TICKS(1000));
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
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));
    vTaskDelay(pdMS_TO_TICKS(3500)); // Do for at least 3s, and leave in neutral state

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(15)));
    vTaskDelay(pdMS_TO_TICKS(1000)); // Do for at least 3s, and leave in neutral state
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));
    vTaskDelay(pdMS_TO_TICKS(1000)); // Do for at least 3s, and leave in neutral state
    
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_steer, example_angle_to_compare(45)));
    vTaskDelay(pdMS_TO_TICKS(1000)); // Do for at least 3s, and leave in neutral state
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_steer, example_angle_to_compare(-45)));
    vTaskDelay(pdMS_TO_TICKS(1000)); // Do for at least 3s, and leave in neutral state
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_steer, example_angle_to_compare(0)));
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
