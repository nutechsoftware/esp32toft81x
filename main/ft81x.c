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
  
  ft81x_cs(1); // inactive CS
  
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
 
  // Enable QUAD spi mode if configured
  #if (FT81X_QUADSPI)
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

  // Reset the command fifo state vars
  ft81x_reset_fifo();
  
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
  ft81x_wr(REG_ROTATE, 0);
  ft81x_wr(REG_SWIZZLE, 0);
  
  // Get our screen size W,H to confirm
  int w = ft81x_rd16(REG_HSIZE);
  int h = ft81x_rd16(REG_VSIZE);
  printf("FT81X REG_HSIZE:%i  REG_VSIZE:%i\n", w, h);

  // Build a black display and display it
  ft81x_stream_start(); // Start streaming
  ft81x_cmd_dlstart();  // Set REG_CMD_DL when done
  ft81x_cmd_swap();     // Set AUTO swap at end of display list    
  ft81x_clear_color_rgb32(0x000000);
  ft81x_clear();
  ft81x_display();
  ft81x_getfree(0);     // trigger FT81x to read the command buffer
  ft81x_stream_stop();  // Finish streaming to command buffer

  // Enable the backlight
  ft81x_wr(REG_PWM_DUTY, 3);

  // Setup the FT81X GPIO PINS
  // set bit 7 to OUTPUT enable the display
  ft81x_wr16(REG_GPIOX_DIR, 0x8000);
  ft81x_wr16(REG_GPIOX, ft81x_rd16(REG_GPIOX) | 0x8000 | 0b1000);

  // Enable the pixel clock
  ft81x_wr32(REG_PCLK, 3);

#if 0 // Display the built in FTDI logo animation  
  ft81x_logo();
#endif

#if 1 // Fill the screen with a solid color
  // SPI Debugging
  gpio_set_level(GPIO_NUM_16, 1);
  gpio_set_level(GPIO_NUM_16, 0);
  
  ft81x_stream_start(); // Start streaming
  ft81x_cmd_dlstart();  // Set REG_CMD_DL when done
  ft81x_cmd_swap();     // Set AUTO swap at end of display list    
  ft81x_clear_color_rgb32(0x000000);
  ft81x_clear();
  ft81x_color_rgb32(0xffffff);
  ft81x_bgcolor_rgb32(0xffffff);
  ft81x_fgcolor_rgb32(0xffffff);
  ft81x_cmd_text(240, 100, 30, OPT_CENTERX, "Hello World");
  ft81x_cmd_number(140, 200, 30, 0, 123);
  ft81x_display();
  ft81x_getfree(0);     // trigger FT81x to read the command buffer
  ft81x_stream_stop();  // Finish streaming to command buffer
  
  // SPI Debugging
  gpio_set_level(GPIO_NUM_16, 1);
  gpio_set_level(GPIO_NUM_16, 0);    
#endif

#if 0 // Fill the screen with a solid color cycling colors
  uint32_t rgb = 0xff0000;
  for (int x=0; x<300; x++) {
    // SPI Debugging
    gpio_set_level(GPIO_NUM_16, 1);
    gpio_set_level(GPIO_NUM_16, 0);
    
    ft81x_stream_start(); // Start streaming
    ft81x_cmd_dlstart();  // Set REG_CMD_DL when done
    ft81x_cmd_swap();     // Set AUTO swap at end of display list    
    ft81x_clear_color_rgb32(rgb);
    ft81x_clear();
    ft81x_color_rgb32(0xffffff);
    ft81x_bgcolor_rgb32(0xffffff);
    ft81x_fgcolor_rgb32(0xffffff);
    ft81x_display();
    ft81x_getfree(0);     // trigger FT81x to read the command buffer
    ft81x_stream_stop();  // Finish streaming to command buffer
    
    // rotate colors
    rgb>>=8; if(!rgb) rgb=0xff0000;

    // Sleep
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // SPI Debugging
    gpio_set_level(GPIO_NUM_16, 1);
    gpio_set_level(GPIO_NUM_16, 0);    
  }
#endif

#if 0 // Draw some dots of rand size, location and color.
  for (int x=0; x<300; x++) {
    // SPI Debugging
    gpio_set_level(GPIO_NUM_16, 1);
    gpio_set_level(GPIO_NUM_16, 0);

    ft81x_stream_start(); // Start streaming
    ft81x_cmd_dlstart();  // Set REG_CMD_DL when done
    ft81x_cmd_swap();     // Set AUTO swap at end of display list    
    ft81x_clear_color_rgb32(0x000000);
    ft81x_clear();
    
    ft81x_fgcolor_rgb32(0xffffff);
    ft81x_bgcolor_rgb32(0xffffff);
    
    uint8_t rred, rgreen, rblue;
    rred = rand()%((253+1)-0) + 0;
    rgreen = rand()%((253+1)-0) + 0;
    rblue = rand()%((253+1)-0) + 0;            
    ft81x_color_rgb888(rred, rgreen, rblue);
    ft81x_begin(POINTS);

    uint16_t size = rand()%((600+1)-0) + 0;
    uint16_t rndx = rand()%((800+1)-0) + 0;
    uint16_t rndy = rand()%((480+1)-0) + 0;
    ft81x_point_size(size);    
    ft81x_vertex2ii(rndx,rndy,0,0);
    
    ft81x_display();
    ft81x_getfree(0);     // trigger FT81x to read the command buffer
    ft81x_stream_stop();  // Finish streaming to command buffer
  }

  // Sleep
  vTaskDelay(10 / portTICK_PERIOD_MS);

  // SPI Debugging
  gpio_set_level(GPIO_NUM_16, 1);
  gpio_set_level(GPIO_NUM_16, 0);    

#endif

  return true;
}

/*
 * Initialize our command buffer pointer state vars
 * for streaming mode.
 */
void ft81x_reset_fifo() {
  ft81x_wp = 0;
  ft81x_freespace = 4096 - 4;  
}

/*
 * Wrapper for CS to allow easier debugging
 */
void ft81x_cs(uint8_t n) {
  gpio_set_level(GPIO_NUM_5, n);
#if 1 // Testing SPI
  ets_delay_us(10);
#endif  
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

  gpio_set_level(GPIO_NUM_16, 1);
  gpio_set_level(GPIO_NUM_16, 0);

  // start the transaction ISR watches the CS bit 
  ft81x_cs(0);
    
  // transmit our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans);
  
  // end the transaction
  ft81x_cs(1);
  
  gpio_set_level(GPIO_NUM_16, 1);
  gpio_set_level(GPIO_NUM_16, 0);

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
  ft81x_cs(0);

  // transmit our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans);

  // grab our return data  
  uint8_t ret = *((uint8_t *)recvbuf);
  
  // end the transaction
  ft81x_cs(1);
  
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
  ft81x_cs(0);

  // transmit our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans);

  // grab our return data  
  uint16_t ret = *((uint16_t *)recvbuf);
  
  // end the transaction
  ft81x_cs(1);
  
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
  ft81x_cs(0);

  // transmit our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans);

  // end the transaction
  ft81x_cs(1); //inactive CS
  
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
  ft81x_cs(0); //active CS

  // transmit our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans);

  // end the transaction
  ft81x_cs(1); //inactive CS
  
}

/*
 * Write 24 bit address + 32 bit value
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
  ft81x_cs(0); // active CS

  // transmit our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans);

  // end the transaction
  ft81x_cs(1); // inactive CS
  
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
 * Start a new transactions to write to the command buffer at the current write pointer
 * rw = 1 is a write operation and will set the write bit.
 * Use tx_buffer to transmit only 24 bits.
 */
void ft81x_start(uint32_t addr, uint8_t write) {
  // setup trans memory
  spi_transaction_ext_t trans;
  memset(&trans, 0, sizeof(trans));

  // set trans options.
  if (ft81x_qio) {
    // Tell the ESP32 SPI ISR to accept MODE_XXX for QIO and DIO
    trans.base.flags |= SPI_TRANS_MODE_DIOQIO_ADDR;
    // Set this transaction to QIO
    trans.base.flags |= SPI_TRANS_MODE_QIO;
  }
  
  // set write bit if rw=1
  addr |= (write ? 0x800000 : 0);
  addr = SPI_REARRANGE_DATA(addr, 24);

  trans.base.length = 24;
  trans.base.tx_buffer = &addr;

  // start the transaction ISR watches CS bit
  ft81x_cs(0); // active CS  

  // transmit our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans); 
}

/*
 * Start a new transactions to write to the command buffer at the current write pointer.
 * Assert CS pin.
 */
void ft81x_stream_start() {
  // be sure we ended the last tranaction
  ft81x_stream_stop();
  // begin a new write transaction.
  ft81x_start(RAM_CMD + (ft81x_wp & 0xfff), WRITE_OP);
}

/*
 * Close any open transactions.
 * De-assert CS pin  
 */
void ft81x_stream_stop() {
  // end the transaction
  ft81x_cs(1); // inactive CS  
}
 
/*
 * Get free space from FT81x until it reports
 * we have required space. Also triggers processing
 * of any display commands in the fifo buffer.
 */
void ft81x_getfree(uint16_t required)
{
  ft81x_wp &= 0xfff;
  ft81x_stream_stop();  
  ft81x_wr16(REG_CMD_WRITE, ft81x_wp & 0xffc);
  do {
    uint16_t rp = ft81x_rp();
    uint16_t howfull = (ft81x_wp - rp) & 4095;
    ft81x_freespace = (4096 - 4) - howfull;
#if 0
    ESP_LOGW(TAG, "fifoA: wp:0x%04x rp:0x%04x fs:0x%04x", ft81x_wp, rp, ft81x_freespace);
#endif
  } while (ft81x_freespace < required);
  ft81x_stream_start();
}

/*
 * wrapper to send out a 32bit command into the fifo buffer
 * while in stream() mode.
 */
void ft81x_cI(uint32_t word) {
  //word = SPI_REARRANGE_DATA(word, 32);
  ft81x_cmd32(word);
}

/*
 * wrapper to send a 8bit command into the fifo buffer
 * while in stream() mode.
 */
void ft81x_cFFFFFF(uint8_t byte) {
  ft81x_cmd32(byte | 0xffffff00);
}

/*
 * Write 32 bit command into our command fifo buffer
 * while in stream() mode.
 * Use tx_buffer to transmit the 32 bits.
 */
void ft81x_cmd32(uint32_t word) {

  // check that we have space in our fifo buffer
  // block until the FT81X says we do.
  if (ft81x_freespace < sizeof(word)) {
    ft81x_getfree(4);
#if 0
    ESP_LOGW(TAG, "free: 0x%04x", ft81x_freespace);
#endif
  }

  ft81x_wp += sizeof(word);
  ft81x_freespace -= sizeof(word);
 
  // setup trans memory
  spi_transaction_ext_t trans;
  memset(&trans, 0, sizeof(trans));

  // set trans options.
  if (ft81x_qio) {
    // Tell the ESP32 SPI ISR to accept MODE_XXX for QIO and DIO
    trans.base.flags |= SPI_TRANS_MODE_DIOQIO_ADDR;
    // Set this transaction to QIO
    trans.base.flags |= SPI_TRANS_MODE_QIO;
  }

  trans.base.length = 32;
  trans.base.tx_buffer = &word;

  // transmit our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans);

}

/*
 * Write N bits command into our command fifo buffer
 * while in stream() mode.
 * Use tx_buffer to transmit the bits.
 */
void ft81x_cN(uint8_t *buffer, uint16_t size) {
  // check that we have space in our fifo buffer
  // block until the FT81X says we do.
  if (ft81x_freespace < size) {
    ft81x_getfree(size);
    ESP_LOGW(TAG, "free: 0x%04x", ft81x_freespace);    
  }
  ft81x_wp += size;
  ft81x_freespace -= size;
 
  // setup trans memory
  spi_transaction_ext_t trans;
  memset(&trans, 0, sizeof(trans));

  // set trans options.
  if (ft81x_qio) {
    // Tell the ESP32 SPI ISR to accept MODE_XXX for QIO and DIO
    trans.base.flags |= SPI_TRANS_MODE_DIOQIO_ADDR;
    // Set this transaction to QIO
    trans.base.flags |= SPI_TRANS_MODE_QIO;
  }

  trans.base.length = size * 8;
  trans.base.tx_buffer = buffer;

  // transmit our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans);

}

/*
 * Programming Guide section 4.6 BITMAP_HANDLE
 * Specify the bitmap handle
 */
void ft81x_BitmapHandle(uint8_t handle) {
  ft81x_cI((0x05UL << 24) | handle);
}

/*
 * Swap the display
 */
void ft81x_swap() {
  ft81x_display(); // end current display list
  ft81x_cmd_swap(); // Set AUTO swap at end of display list
  //ft81x_cmd_loadidentity();
  ft81x_cmd_dlstart(); // Set REG_CMD_DL when done
  ft81x_getfree(0); // trigger display processing
}

/*
 * Programming Guide section 4.21 CLEAR
 * Clear buffers to preset values
 */
void ft81x_clear() {
  ft81x_cI((0x26UL << 24) | 7);
}

/*
 * Programming Guide section 4.23 CLEAR_COLOR_RGB
 * Specify clear values for red, green and blue channels
 */
void ft81x_clear_color_rgb32(uint32_t rgb) {
   ft81x_cI((0x2UL << 24) | (rgb & 0xffffffL));
}

void ft81x_clear_color_rgb888(uint8_t red, uint8_t green, uint8_t blue) {
   ft81x_clear_color_rgb32(((red & 255L) << 16) | ((green & 255L) << 8) | ((blue & 255L) << 0));
}

/*
 * Programming Guide section 4.28 COLOR_RGB
 * Set the current color red, green, blue
 */
void ft81x_color_rgb32(uint32_t rgb) {
   ft81x_cI((0x4UL << 24) | (rgb & 0xffffffL));
}

void ft81x_color_rgb888(uint8_t red, uint8_t green, uint8_t blue) {
   ft81x_color_rgb32(((red & 255L) << 16) | ((green & 255L) << 8) | ((blue & 255L) << 0));
}

/*
 * Programming Guide section 5.30 CMD_FGCOLOR
 * set the foreground color 
 */
void ft81x_fgcolor_rgb32(uint32_t rgb) {
   ft81x_cFFFFFF(0x0a);
   ft81x_cI(rgb << 8);
}
void ft81x_fgcolor_rgb888(uint8_t red, uint8_t green, uint8_t blue) {
  ft81x_fgcolor_rgb32(((red & 255L) << 16) | ((green & 255L) << 8) | ((blue & 255L) << 0));
}

// 5.31 CMD_BGCOLOR - set the background color 
/*
 * Programming Guide section 5.31 CMD_BGCOLOR
 * set the background color 
 */
void ft81x_bgcolor_rgb32(uint32_t rgb) {
  ft81x_cFFFFFF(0x09);
  ft81x_cI(rgb << 8);
}
void ft81x_bgcolor_rgb888(uint8_t red, uint8_t green, uint8_t blue) {
   ft81x_bgcolor_rgb32(((red & 255L) << 16) | ((green & 255L) << 8) | ((blue & 255L) << 0));
}

/*
 * Programming Guide section 5.12 CMD_SWAP
 * Swap the current display list
 */
void ft81x_cmd_swap() {
   ft81x_cFFFFFF(0x01);
}

/*
 * Programming Guide section 4.29 DISPLAY
 * End the display list. FT81X will ignore all commands following this command.
 */
void ft81x_display() {
   ft81x_cI((0x0UL << 24));
}

/*
 * Programming Guide section 5.44 CMD_LOADIDENTITY
 * Set the current matrix to the identity matrix
 * 
 */
void ft81x_cmd_loadidentity() {
   ft81x_cFFFFFF(0x26);
}

/*
 * Programming Guide section 5.11 CMD_DLSTART
 * Start a new display list
 */
void ft81x_cmd_dlstart() {
   ft81x_cFFFFFF(0x00);
}

/*
 * Programming Guide section 5.11 CMD_SETROTATE
 * Start a new display list
 */
void ft81x_cmd_setrotate(uint32_t r) {
   ft81x_cFFFFFF(0x36);
   ft81x_cI(r);
   //FIXME: Update our own internal H/W to reflect display
}

/*
 * Programming Guide section 5.41 CMD_TEXT
 * Draw text
 * FIXME: Needs padding to be 4 byte aligned
 */
void ft81x_cmd_text(int16_t x, int16_t y, int16_t font, uint16_t options, const char *s) {
   ft81x_cFFFFFF(0x0c);
   uint16_t b[4];
   b[0] = x;
   b[1] = y;
   b[2] = font;
   b[3] = options;
   ft81x_cN((uint8_t *)b,sizeof(b));
   ft81x_cN((uint8_t *)s,strlen(s)+1);
   ESP_LOGW(TAG, "FIXME PADDING: 0x%04x", strlen(s)+1);
}

/*
 * Programming Guide section 5.43 CMD_NUMBER
 * Draw number
 */
void ft81x_cmd_number(int16_t x, int16_t y, int16_t font, uint16_t options, int32_t n) {
   uint16_t b[6];
   b[0] = x;
   b[1] = y;
   b[2] = font;
   b[3] = options;
   b[4] = n;
   b[5] = 0; // dummy pad
   ft81x_cFFFFFF(0x2e);
   ft81x_cN((uint8_t *)&b,sizeof(b));
}

/*
 * Programming Guide section 5.66 CMD_LOGO
 * Play FTDI logo animation wait till it is done
 */
void ft81x_logo() {
  ft81x_stream_start(); // Start streaming
  ft81x_cmd_dlstart();  // Set REG_CMD_DL when done
  ft81x_cFFFFFF(0x31);  // Logo command
  ft81x_cmd_swap();     // Set AUTO swap at end of display list    
  ft81x_getfree(0);     // trigger FT81x to read the command buffer
  ft81x_stream_stop();  // Finish streaming to command buffer
  // Wait till the Logo is finished  
  ft81x_wait_finish();  
  // AFAIK the only command that will set the RD/WR to 0 when finished
  ft81x_reset_fifo();
}

/*
 * Programming Guide section 4.47 VERTEX2F
 * Start the operation of graphics primitives at the specified
 * screen coordinate, in the pixel precision defined by VERTEX_FORMAT
 */
void ft81x_vertex2f(int16_t x, int16_t y) {
  ft81x_cI((1UL << 30) | ((x & 32767L) << 15) | ((y & 32767L) << 0));
}

/*
 * Programming Guide section 4.48 VERTEX2ii
 * Start the operation of graphics primitives at the specified
 * screen coordinate, in the pixel precision defined by VERTEX_FORMAT
 */
void ft81x_vertex2ii(int16_t x, int16_t y, uint8_t handle, uint8_t cell) {
  uint8_t b[4];
  b[0] = (cell & 127) | ((handle & 1) << 7);
  b[1] = (handle >> 1) | (y << 4);
  b[2] = (y >> 4) | (x << 5);
  b[3] = (2 << 6) | (x >> 3);
  ft81x_cI(*((uint32_t *)&b[0]));
}
 
/*
 * Programming Guide section 4.36 POINT_SIZE
 * Specify the radius of points
 */
void ft81x_point_size(uint16_t size) {
  ft81x_cI((0x0dUL << 24) | ((size & 8191L) << 0));
}

/*
 * Programming Guide section 4.5 BEGIN
 * Begin drawing a graphics primitive
 */
void ft81x_begin( uint8_t prim) {
  ft81x_cI((0x1fUL << 24) | prim);
}

/*
 * Programming Guide section 4.30 END
 * End drawing a graphics primitive
 */
void ft81x_end() {
  ft81x_cI((0x21UL << 24));
}

/*
 * Finish. Wait till READ and WRITE pointers are 0.
 * presumes not currently streaming commands
 */
void ft81x_wait_finish() {
  // Block until animation is done READ and WRITE will be 0  
  while ( (ft81x_rd16(REG_CMD_WRITE) != 0) ||
          (ft81x_rd16(REG_CMD_READ) != 0)
        );  
}
