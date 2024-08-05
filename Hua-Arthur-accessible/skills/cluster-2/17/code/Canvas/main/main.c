/* From design pattern Serial IO Example */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_vfs_dev.h"
// This is associated with VFS -- virtual file system interface and abstraction -- see the docs

void app_main()
{
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,
      256, 0, 0, NULL, 0) );

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    int time = 0;
    int count = 0;
    while(1) {
       if (count > 9) {
	     count=0;
	 }
       time++;
       count++;
       printf("%d %d\n", count, time); // could add values here separated by spaces
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}