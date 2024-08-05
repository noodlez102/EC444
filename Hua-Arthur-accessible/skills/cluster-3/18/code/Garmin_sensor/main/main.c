// Changing I2C addresses for Lidar Lite v4 (Garmin)
// Thanks to Lukas Chin, Shamir Legaspi, David Li, 2023-11-09

#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include <sys/types.h>
#include <sys/wait.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

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

void app_main() {
    i2c_master_init();
    i2c_scanner();
    vTaskDelay(30/portTICK_PERIOD_MS);
    change();
    i2c_scanner();
    int16_t distance;
    
    while (1) {
        writeRegister(0x00, 0x04, 0x24); 
        
        distance = read16(0x10,0x24);
        printf("Distance: %d cm\n", distance);    
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}