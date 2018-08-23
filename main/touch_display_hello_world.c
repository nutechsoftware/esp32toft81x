/**
 *  @file    touch_display_hello_world.c
 *  @author  Sean Mathews <coder@f34r.com>
 *  @date    08/01/2018
 *  @version 1.0
 *
 *  @brief Example usage of the API to communicate with the FT81X
 *  chip from an ESP32 uP
 *
 *  @copyright Copyright (C) 2018 Nu Tech Software Solutions, Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "ft81x.h"

/*
 * Defines
 */ 

/*
 * Constants/Statics
 */ 
// Debug tag
static const char* TAG = "HELLO_WORLD";


/*
 * Types
 */

 
/*
 * Prototypes
 */ 
// Enable GPIO pin for bit bang debugging good for timing tests :c)
bool enable_gpio_debug_pin();

// restart the esp32
void restart_esp32();

/*
 * Functions
 */
 
/*
 * FreeRTOS main() entry point
 *
 */
void app_main()
{
    bool res;
    
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    // Enable GPIO debug output pin
    enable_gpio_debug_pin();

#if 0
    // delay test before spi to allow power on of device and usb connect time
    ESP_LOGW(TAG, "initGPU: delay");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
#endif

    // Initialize our SPI driver on ESP32 VSPI pins for the FT81X
    res = ft81x_initSPI();
    if (!res) {
      printf("ESP32 SPI init failed\n");
      restart_esp32();
    } else {
      printf("ESP32 SPI init success\n");
    }


    // Initialize the FT81X GPU
    res = ft81x_initGPU();
    if (!res) {
      printf("FT81X init failed\n");
      restart_esp32();
    } else {
      printf("FT81X init success ID 0x%04x\n", ft81x_chip_id);
    }

    // Testing done    
    restart_esp32();
}

/*
 * Restart ESP32 chip
 */
void restart_esp32() {
  for (int i = 10; i >= 0; i--) {
      printf("Restarting in %d seconds...\n", i);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  printf("Restarting now.\n");
  fflush(stdout);
  esp_restart();
}


/* 
 * Enable an output pin for bit banging debug data
 */
bool enable_gpio_debug_pin() {
  esp_err_t ret;
  gpio_config_t io_conf = {
  .intr_type = GPIO_PIN_INTR_DISABLE,
  .mode = GPIO_MODE_OUTPUT,
  .pin_bit_mask = 1LL << GPIO_NUM_16,
  };

  ret = gpio_config(&io_conf);
  if(ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add DEBUG pin");
    return false;
  }
  return true;
}
