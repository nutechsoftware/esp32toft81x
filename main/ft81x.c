/**
 *  @file    ft81x.c
 *  @author  Sean Mathews <coder@f34r.com>
 *  @date    08/01/2018
 *  @version 1.0
 *
 *  @brief API to communicate with a FT81X chip from a ESP32 uP
 *
 *  This code provides an API to communicate with the FT81X chip
 *  from an ESP32. It simplifies the complexity of SPI on the ESP32
 *  correctly formatting the SPI communications to work with
 *  the FT81X. It also allow for QUAD SPI communications where
 *  permitted to increase data transfer speeds.
 *
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
 * Constants/Statics
 */ 
// Debug tag
static const char* TAG = "ESP32-FT81X";
// SPI device driver handle for our ft81x
spi_device_handle_t ft81x_spi;
// FT81X section 5.2 Chip ID
uint16_t ft81x_chip_id = 0;
// FT81X command buffer free space
uint16_t ft81x_freespace = 0;
// FT81X command buffer write pointer
uint16_t ft81x_wp = 0;
// FT81X enable QIO modes
uint8_t ft81x_qio = 0;

// Our device config configured for ESP32+FT81X
// to work around no variable dummy byte and more issues.
spi_device_interface_config_t ft81x_spi_devcfg={
.clock_speed_hz = FT81X_SPI_SPEED,
.mode = 0,
.spics_io_num = -1,
.queue_size = 7, //We want to be able to queue 7 transactions at a time
.flags = SPI_DEVICE_HALFDUPLEX,
.address_bits = 0,
.dummy_bits = 0,
.command_bits = 0,
};

/*
 * Core functions
 */
 
/*
 * Initialize the ESP32 spi device driver and connection to our FT81X chip.
 * This is a shared bus for devices like a flash disk. 
 * see spi_master.c for more details on how the driver switches baud,
 * bits, modes and such on the fly depending on what device it is 
 * currently in a transaction with. This does require using transactions
 * on all of the code unless it was assured the spi port settings were 
 * set correctly at the time of the transmission.
 */
bool ft81x_initSPI() {
  esp_err_t ret;

  spi_bus_config_t buscfg={
  .miso_io_num = GPIO_NUM_19,
  .mosi_io_num = GPIO_NUM_23,
  .sclk_io_num = GPIO_NUM_18,
  .quadwp_io_num = GPIO_NUM_22,
  .quadhd_io_num = GPIO_NUM_21  
  };

  //Initialize the SPI bus
  // For now disable DMA it was only letting me get back 1 byte.
  // https://esp32.com/viewtopic.php?t=2519
  ret = spi_bus_initialize(VSPI_HOST, &buscfg, 0);
  if(ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add SPI device");
    return false;
  }

  ret = spi_bus_add_device(VSPI_HOST, &ft81x_spi_devcfg, &ft81x_spi);
  if(ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add SPI device");
    return false;
  }

  gpio_config_t io_conf = {
  .intr_type = GPIO_PIN_INTR_DISABLE,
  .mode = GPIO_MODE_OUTPUT,
  .pin_bit_mask = 1LL << GPIO_NUM_5,
  };

  ret = gpio_config(&io_conf);
  if(ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add CS pin");
    return false;
  }
  
  gpio_set_level(GPIO_NUM_5, 1);
  
  return true;
}

/*
 * Initialize the FT81x GPU and test for a valid chip response
 */
bool ft81x_initGPU() {
    
  // Init global ft81x_chip_id to 0
  ft81x_chip_id = 0;

  // FIXME: Just in case we are stuck in QUAD mode I need to get out
  // or no commands will be taken
  ft81x_qio = 1;
  ft81x_wr16(REG_SPI_WIDTH, 0b00);    
  ft81x_qio = 0;

  // Put the FT81x to sleep
  ft81x_hostcmd(CMD_SLEEP, 0x00);
  // CLKSEL default
  ft81x_hostcmd(CMD_CLKSEL_A, 0x00);
  // ACTIVE
  ft81x_hostcmd(CMD_ACTIVE, 0x00);
  // CLKINT  
  ft81x_hostcmd(CMD_CLKINT, 0x00);
  // PD_ROMS all powered on
  ft81x_hostcmd(CMD_PD_ROMS, 0x00);
  // RST_PULSE
  ft81x_hostcmd(CMD_RST_PULSE, 0x00);
  
  // Read our CHIP ID address until it returns a valid result
  uint16_t ret = 0;
  uint16_t count = 0;
  while (((ret = ft81x_rd16(0xc0000UL)) & 0xff) != 0x08) {
    //ESP_LOGW(TAG, "HWID: 0x%04x", ret);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    count++;
    // Timeout fail gracefully
    if (count > 100)
      return false;
  };
  //ESP_LOGW(TAG, "HWID: 0x%04x", ret);
  // Save our CHIP ID
  ft81x_chip_id = ret;
  
  // Set the PWM to 0 turn off the backlight
  ft81x_wr(REG_PWM_DUTY, 0);
 
  // Setup the FT81X GPIO PINS
  #if 0
  //// FIXME: Needs tech data review and change to GPIOX calls
  ft81x_wr16(REG_GPIOX_DIR, 0x8000);
  ft81x_wr16(REG_GPIOX, ft81x_rd16(REG_GPIOX) | 0x8000);
  #else
  // set GPIO7 to OUTPUT. Display enable? Not in the docs but needed.
  ft81x_wr(REG_GPIO_DIR, 0x80);
  ft81x_wr(REG_GPIO, ft81x_rd(REG_GPIO) | 0x80);
  #endif
  
  // Enable QUAD spi mode if configured
  #ifdef FT81X_QUADSPI
    // Enable QAUD spi mode no dummy
    ft81x_wr16(REG_SPI_WIDTH, 0b010);
    ft81x_qio = 1;
    //ft81x_wr16(REG_SPI_WIDTH, 0b00);    
    //ft81x_qio = 0;
  #else
    // Enable single channel spi mode
    ft81x_wr16(REG_SPI_WIDTH, 0b0);
    ft81x_qio = 0;
  #endif
  
  // Screen specific settings
  // NHD-7.0-800480FT-CSXV-CTP
  // http://newhavendisplay.com/learnmore/EVE2_7-CSXV-CTP/
  ft81x_wr32(REG_HCYCLE, 900);
  ft81x_wr32(REG_HOFFSET, 43);
  ft81x_wr32(REG_HSIZE, 800);
  ft81x_wr32(REG_HSYNC0, 0);
  ft81x_wr32(REG_HSYNC1, 41);
  ft81x_wr32(REG_VCYCLE, 500);
  ft81x_wr32(REG_VOFFSET, 12);
  ft81x_wr32(REG_VSIZE, 480);
  ft81x_wr32(REG_VSYNC0, 0);
  ft81x_wr32(REG_VSYNC1, 10);
  ft81x_wr32(REG_DITHER, 1);
  ft81x_wr32(REG_PCLK_POL, 1);
  ft81x_wr32(REG_PCLK, 3);
  ft81x_wr(REG_ROTATE, 0);
  ft81x_wr(REG_SWIZZLE, 0);
  
  // Get our screen size W,H to confirm
  int w = ft81x_rd16(REG_HSIZE);
  int h = ft81x_rd16(REG_VSIZE);
  printf("FT81X REG_HSIZE:%i  REG_VSIZE:%i\n", w, h);
  
  // Work-around issue with bitmap sizes not being reset
  for (uint8_t i = 0; i < 32; i++) {
    ft81x_BitmapHandle(i);  //BITMAP_HANDLE(0x05)
    ft81x_cmd32(0x28000000UL); //BITMAP_LAYOUT_H(0x28)
    ft81x_cmd32(0x29000000UL); //BITMAP_SIZE_H(0x29)
  }

  ft81x_wr(REG_PWM_DUTY, 20);

  // Initialize our command buffer pointer state vars
  ft81x_wp = 0;
  ft81x_freespace = 4096 - 4;

  return true;
}

/*
 * Write 16 bit command+arg and dummy byte
 * See 4.1.5 Host Command
 * Must never be sent in QUAD spi mode except for reset?
 * A total of 24 bytes will be on the SPI BUS.
 */
void ft81x_hostcmd(uint8_t command, uint8_t args) {
  // setup trans memory
  spi_transaction_ext_t trans;
  memset(&trans, 0, sizeof(trans));

  // arg plus dummy 16 bits 
  uint16_t dargs = args << 8;
  
  // set trans options.
  trans.base.flags = SPI_TRANS_VARIABLE_CMD;
  
  // fake dummy byte shift left 8  
  trans.command_bits = 8;
  trans.base.cmd = command;
  trans.base.tx_buffer = &dargs;
  trans.base.length = 16;
    
  trans.base.rx_buffer = NULL;

  // start the transaction ISR watches the CS bit 
  gpio_set_level(GPIO_NUM_5, 0); //active CS
  
  // transmit our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans);
  
  // end the transaction
  gpio_set_level(GPIO_NUM_5, 1); //inactive CS

}

/*
 * Write 24 bit address + dummy byte 
 * and read the 8 bit result. 
 * A total of 6 bytes will be on the SPI BUS.
 */
uint8_t ft81x_rd(uint32_t addr)
{
  // setup trans memory
  spi_transaction_ext_t trans;
  memset(&trans, 0, sizeof(trans));

  // allocate memory for our return data
  char *recvbuf=heap_caps_malloc(1, MALLOC_CAP_DMA);
  
  // set trans options.
  trans.base.flags = SPI_TRANS_VARIABLE_ADDR;  
  if (ft81x_qio) {
    // Tell the ESP32 SPI ISR to accept MODE_XXX for QIO and DIO
    trans.base.flags |= SPI_TRANS_MODE_DIOQIO_ADDR;
    // Set this transaction to QIO
    trans.base.flags |= SPI_TRANS_MODE_QIO;
  }
  
  // fake dummy byte shift left 8
  trans.address_bits = 32;  
  trans.base.addr = addr << 8;
  
  trans.base.length = 8;
  trans.base.rxlength = 8;
  
  // point to our RX buffer.
  trans.base.rx_buffer = recvbuf; // RX buffer

  // start the transaction ISR watches CS bit
  gpio_set_level(GPIO_NUM_5, 0); //active CS

  // transmit our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans);

  // grab our return data  
  uint8_t ret = *((uint8_t *)recvbuf);
  
  // end the transaction
  gpio_set_level(GPIO_NUM_5, 1); //inactive CS
  
  // cleanup
  free(recvbuf);
  
  return ret;
}

/*
 * Write 24 bit address + dummy byte 
 * and read the 16 bit result. 
 * A total of 6 bytes will be on the SPI BUS.
 */
uint16_t ft81x_rd16(uint32_t addr)
{
  // setup trans memory
  spi_transaction_ext_t trans;
  memset(&trans, 0, sizeof(trans));

  // allocate memory for our return data
  char *recvbuf=heap_caps_malloc(2, MALLOC_CAP_DMA);
  
  // set trans options.
  trans.base.flags = SPI_TRANS_VARIABLE_ADDR;
  if (ft81x_qio) {
    // Tell the ESP32 SPI ISR to accept MODE_XXX for QIO and DIO
    trans.base.flags |= SPI_TRANS_MODE_DIOQIO_ADDR;
    // Set this transaction to QIO
    trans.base.flags |= SPI_TRANS_MODE_QIO;
  }
  
  // fake dummy byte shift left 8
  trans.address_bits = 32;  
  trans.base.addr = addr << 8;
  
  trans.base.length = 16;
  trans.base.rxlength = 16;
  
  // point to our RX buffer.
  trans.base.rx_buffer = recvbuf; // RX buffer

  // start the transaction ISR watches CS bit
  gpio_set_level(GPIO_NUM_5, 0); //active CS

  // transmite our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans);

  // grab our return data  
  uint16_t ret = *((uint16_t *)recvbuf);
  
  // end the transaction
  gpio_set_level(GPIO_NUM_5, 1); //inactive CS
  
  // cleanup
  free(recvbuf);
  
  return ret;
}

/*
 * Write 24 bit address + 8 bit value
 * A total of 4 bytes will be on the SPI BUS.
 */
void ft81x_wr(uint32_t addr, uint8_t byte) {
  // setup trans memory
  spi_transaction_ext_t trans;
  memset(&trans, 0, sizeof(trans));

  // set trans options.
  trans.base.flags = SPI_TRANS_VARIABLE_ADDR;
  if (ft81x_qio) {
    // Tell the ESP32 SPI ISR to accept MODE_XXX for QIO and DIO
    trans.base.flags |= SPI_TRANS_MODE_DIOQIO_ADDR;
    // Set this transaction to QIO
    trans.base.flags |= SPI_TRANS_MODE_QIO;
  }

  // no dummy byte for writes
  trans.address_bits = 24;
  trans.base.addr = addr | 0x800000; // set write bit

  trans.base.length = 8;
  trans.base.rx_buffer = NULL;
  trans.base.tx_buffer = &byte;
  
  // start the transaction ISR watches CS bit
  gpio_set_level(GPIO_NUM_5, 0); //active CS

  // transmite our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans);

  // end the transaction
  gpio_set_level(GPIO_NUM_5, 1); //inactive CS
  
}

/*
 * Write 24 bit address + 16 bit value
 * A total of 5 bytes will be on the SPI BUS.
 */
void ft81x_wr16(uint32_t addr, uint16_t word) {
  // setup trans memory
  spi_transaction_ext_t trans;
  memset(&trans, 0, sizeof(trans));

  // set trans options.
  trans.base.flags = SPI_TRANS_VARIABLE_ADDR;
  if (ft81x_qio) {
    // Tell the ESP32 SPI ISR to accept MODE_XXX for QIO and DIO
    trans.base.flags |= SPI_TRANS_MODE_DIOQIO_ADDR;
    // Set this transaction to QIO
    trans.base.flags |= SPI_TRANS_MODE_QIO;
  }

  // no dummy byte for writes
  trans.address_bits = 24;
  trans.base.addr = addr | 0x800000; // set write bit

  trans.base.length = 16;
  trans.base.rx_buffer = NULL;
  trans.base.tx_buffer = &word;
  
  // start the transaction ISR watches CS bit
  gpio_set_level(GPIO_NUM_5, 0); //active CS

  // transmite our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans);

  // end the transaction
  gpio_set_level(GPIO_NUM_5, 1); //inactive CS
  
}

/*
 * Write 24 bit address + 16 bit value
 * A total of 7 bytes will be on the SPI BUS.
 */
void ft81x_wr32(uint32_t addr, uint32_t word) {
  // setup trans memory
  spi_transaction_ext_t trans;
  memset(&trans, 0, sizeof(trans));

  // set trans options.
  trans.base.flags = SPI_TRANS_VARIABLE_ADDR;
  if (ft81x_qio) {
    // Tell the ESP32 SPI ISR to accept MODE_XXX for QIO and DIO
    trans.base.flags |= SPI_TRANS_MODE_DIOQIO_ADDR;
    // Set this transaction to QIO
    trans.base.flags |= SPI_TRANS_MODE_QIO;
  }

  // no dummy byte for writes
  trans.address_bits = 24;
  trans.base.addr = addr | 0x800000; // set write bit

  trans.base.length = 32;
  trans.base.rx_buffer = NULL;
  trans.base.tx_buffer = &word;

  // start the transaction ISR watches CS bit
  gpio_set_level(GPIO_NUM_5, 0); //active CS

  // transmite our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans);

  // end the transaction
  gpio_set_level(GPIO_NUM_5, 1); //inactive CS
  
}

/*
 * Read the FT81x command pointer
 */
uint16_t ft81x_rp() {
  uint16_t rp = ft81x_rd16(REG_CMD_READ);
  if (rp == 0xfff) {
    printf("FT81X COPROCESSOR EXCEPTION");
  }
  return rp;
}

/*
 * ft81x_getfree space from FT81x until it reports
 * we have required space
 */
void ft81x_getfree(uint16_t required)
{
  ft81x_wp &= 0xfff;
  //__end(); //FIXME
  ft81x_wr16(REG_CMD_WRITE, ft81x_wp & 0xffc);
  do {
    uint16_t howfull = (ft81x_wp - ft81x_rp()) & 4095;
    ft81x_freespace = (4096 - 4) - howfull;
  } while (ft81x_freespace < required);
  // stream(); //FIXME
}

/*
 * Write 32 bit command into our command fifo buffer
 * A total of 4 bytes will be on the SPI BUS.
 */
void ft81x_cmd32(uint32_t word) {
  if (ft81x_freespace < 4) {
    ft81x_getfree(4);
  }
  ft81x_wp += 4;
  ft81x_freespace -= 4;
  #if 0 // FIXME
  union {
    uint32_t c;
    uint8_t b[4];
  };
  c = x;
  SPI.transfer(b[0]);
  SPI.transfer(b[1]);
  SPI.transfer(b[2]);
  SPI.transfer(b[3]);
  #endif
}

/*
 * Write out a bitmap handle into the command buffer
 * A total of 7 bytes will be on the SPI BUS.
 */
void ft81x_BitmapHandle(uint8_t handle) {
  ft81x_cmd32((5UL << 24) | handle);
}
