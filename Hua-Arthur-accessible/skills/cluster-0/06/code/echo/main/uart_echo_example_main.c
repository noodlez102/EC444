/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <string.h>

/**
 * This is an example which echos any data it receives on UART0 back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: use the default pins rather than making changes
 */

#define EX_UART_NUM UART_NUM_0
#define ECHO_TEST_TXD  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_RXD  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE (1024)


void app_main(void)
{
    /* Configure parameters of the UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(EX_UART_NUM, &uart_config);
    uart_set_pin(EX_UART_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    gpio_reset_pin(13);
    gpio_set_direction(13, GPIO_MODE_OUTPUT);

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    // Display prompt
    printf("Toggle Mode \n");
    int toggle =0;
    int arraySize = INT_MAX;
    char *dynamicArray = (char *)malloc(4*sizeof(char));
    int index =0;
    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(EX_UART_NUM, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        // Write data back to the UART
        if(len>0){

            if(*data != 13 || *data != '\r'){
                dynamicArray[index]= *data;
                index++;
            }else{
                // printf("You typed in ");
                // for(int i =0; i<=index;i++){
                //     printf("%c",dynamicArray[i]);
                // }
                // printf("\n");

                if(*dynamicArray == 's' ){
                    toggle++;
                    if(toggle ==3){
                        toggle =0;
                        printf("Toggle mode\n");
                    }else if(toggle ==1){
                        printf("echo mode\n");
                    }else if(toggle ==2){
                        printf("echo dec to hex mode\nEnter an Integer:\n");
                    }
                }else if(toggle== 0){
                // fgets((char *)data, BUF_SIZE, stdin);
                    printf("read: %s\n", dynamicArray);
                    if(strcmp(dynamicArray, "t") == 0){
                        gpio_set_level(13, 1);
                        vTaskDelay(100 / portTICK_PERIOD_MS);
                        gpio_set_level(13, 0);

                    }
                    
                }else if(toggle ==1){
                    
                    printf("echo: %s\n", dynamicArray);

                }else if(toggle ==2){
                    int s;  
                    sscanf(dynamicArray, "%d", &s);
                    printf("HEX: %x\n", s);
                    printf("Enter an Integer:\n");
                } 

                for(int i = 0; i<index+1;i++){
                    dynamicArray[i] = '\0';
                }
                index =0;

            }     
        } 

        // if(len>0){
        //     printf("Read: %c \n\nEnter value: \n", *data);
        // }
    }        

}