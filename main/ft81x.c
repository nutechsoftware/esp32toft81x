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
 *  permitted to increase data transfer speeds. The FT81X and the
 *  ESP32 are both little-endian so byte order does not need to
 *  be adjusted.
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
#include "transparent-test-file.h"

/*
 * Constants/Statics/Globals
 */
// Debug tag
static const char *TAG = "ESP32-FT81X";
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
// FT81x screen width
uint16_t ft81x_width = 0;
// FT81x screen height
uint16_t ft81x_height = 0;

// MEDIA FIFO state vars
uint32_t mf_wp = 0;
uint32_t mf_size = 0;
uint32_t mf_base = 0;


// FT813 touch screen state loaded by calls to get_touch_inputs
//// Capacitive touch state
struct ft81x_ctouch_t ft81x_ctouch;

//// touch tracker state
struct ft81x_touch_t ft81x_touch;

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

  // Enable QUAD spi mode if configured
  #if (FT81X_QUADSPI)
    // Enable QAUD spi mode no dummy
    ft81x_wr16(REG_SPI_WIDTH, 0b010);
    ft81x_qio = 1;
  #else
    // Enable single channel spi mode
    ft81x_wr16(REG_SPI_WIDTH, 0b000);
    ft81x_qio = 0;
  #endif

  // Set the PWM to 0 turn off the backlight
  ft81x_wr(REG_PWM_DUTY, 0);

  // Reset the command fifo state vars
  ft81x_reset_fifo();

  // Screen specific settings
  // NHD-7.0-800480FT-CSXV-CTP
  // http://newhavendisplay.com/learnmore/EVE2_7-CSXV-CTP/
  ft81x_width = 800;
  ft81x_height = 480;
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
  int ft81x_width = ft81x_rd16(REG_HSIZE);
  int ft81x_height = ft81x_rd16(REG_VSIZE);
  printf("FT81X REG_HSIZE:%i  REG_VSIZE:%i\n", ft81x_width, ft81x_height);

  // Initialize the touch and audio settings
  ft81x_wr(REG_CTOUCH_EXTENDED, 1);   // Turn on=0/(off=1) multi-touch
  ft81x_wr(REG_VOL_PB, 0);            // Turn playback volume off
  ft81x_wr(REG_VOL_SOUND, 0);         // Turn synthesizer volume off
  ft81x_wr(REG_SOUND, MUTE);          // Set synthesizer to 'Mute'

#if 1 // Test with black screen before starting display clock
  // Build a black display and display it
  ft81x_stream_start(); // Start streaming
  ft81x_cmd_dlstart();  // Set REG_CMD_DL when done
  ft81x_cmd_swap();     // Set AUTO swap at end of display list
  ft81x_clear_color_rgb32(0x000000);
  ft81x_clear();
  ft81x_display();
  ft81x_getfree(0);     // trigger FT81x to read the command buffer
  ft81x_stream_stop();  // Finish streaming to command buffer
  ft81x_wait_finish();  // Wait till the GPU is finished processing the commands
  // Sleep a little
  vTaskDelay(100 / portTICK_PERIOD_MS);
#endif

  // Enable the backlight
  ft81x_wr(REG_PWM_DUTY, 12);

  // Setup the FT81X GPIO PINS
  // set bit 7 to OUTPUT enable the display
  // turn of GPIO power to 10ma for SPI pins
  ft81x_wr16(REG_GPIOX_DIR, 0x8000);
  ft81x_wr16(REG_GPIOX, ft81x_rd16(REG_GPIOX) | 0x8000  | (0x1 << 10));

  // Enable the pixel clock
  ft81x_wr32(REG_PCLK, 3);

  // Sleep a little
  vTaskDelay(100 / portTICK_PERIOD_MS);

#if 0 // Display the built in FTDI logo animation and then calibrate
  // SPI Debugging
  gpio_set_level(GPIO_NUM_16, 1);
  gpio_set_level(GPIO_NUM_16, 0);

  ft81x_logo();
  ft81x_calibrate();

  // SPI Debugging
  gpio_set_level(GPIO_NUM_16, 1);
  gpio_set_level(GPIO_NUM_16, 0);
#endif

#if 1 // Test LOAD_IMAGE ON and OFF with a transparent PNG and update when touched
  uint32_t imgptr, widthptr, heightptr;
  uint32_t ptron, ptroff, ptrnext, width, height;

  // SPI Debugging
  gpio_set_level(GPIO_NUM_16, 1);
  gpio_set_level(GPIO_NUM_16, 0);

  // Load the OFF image to the MEDIA FIFO
  //// Start streaming
  ft81x_stream_start();

  //// Configure MEDIA FIFO
  ft81x_cmd_mediafifo(0x100000UL-0x40000UL, 0x40000UL);

  //// Trigger FT81x to read the command buffer
  ft81x_getfree(0);

  //// Finish streaming to command buffer
  ft81x_stream_stop();

  //// Wait till the GPU is finished
  ft81x_wait_finish();

  //// stop media fifo
  ft81x_wr32(REG_MEDIAFIFO_WRITE, 0);

  //// Load the image at address 0
  ptroff = 0;

  // Load the OFF image
  //// Start streaming
  ft81x_stream_start();
  
  //// USE MEDIA_FIFO
  //// Load the image at address transparent_test_file_png_len+1
  ft81x_cmd_loadimage(ptroff, OPT_RGB565 | OPT_NODL | OPT_MEDIAFIFO);

  //// Get the decompressed image properties
  ft81x_cmd_getprops(&imgptr, &widthptr, &heightptr);

  //// Trigger FT81x to read the command buffer
  ft81x_getfree(0);

  //// Finish streaming to command buffer
  ft81x_stream_stop();

  //// Send the image to the media fifo
  ft81x_cSPOOL_MF(transparent_test_file_off_png, transparent_test_file_off_png_len);

  //// Wait till the GPU is finished
  ft81x_wait_finish();

  //// Dump results
  ptron = ft81x_rd32(imgptr); // pointer to end of image and start of next free memory
  width = ft81x_rd32(widthptr);
  height = ft81x_rd32(heightptr);
  ESP_LOGW(TAG, "loadimage off: start:0x%04x end: 0x%04x width: 0x%04x height: 0x%04x", ptroff, ptron-1, width, height);

  // SPI Debugging
  gpio_set_level(GPIO_NUM_16, 1);
  gpio_set_level(GPIO_NUM_16, 0);

  // Load the OFF image
  //// Start streaming
  ft81x_stream_start();

  //// USING CMD BUFFER. Max size is ~4k
  //// Load the image at address transparent_test_file_png_len+1 using CMD buffer
  ft81x_cmd_loadimage(ptron, OPT_RGB565 | OPT_NODL);

  //// spool the image to the FT81X
  ft81x_cSPOOL((uint8_t *)transparent_test_file_on_png, transparent_test_file_on_png_len);

  //// Get the decompressed image properties
  ft81x_cmd_getprops(&imgptr, &widthptr, &heightptr);

  //// Trigger FT81x to read the command buffer
  ft81x_getfree(0);

  //// Finish streaming to command buffer
  ft81x_stream_stop();

  //// Wait till the GPU is finished
  ft81x_wait_finish();

  //// Dump results
  ptrnext = ft81x_rd32(imgptr); // pointer to end of image and start of next free memory
  width = ft81x_rd32(widthptr);
  height = ft81x_rd32(heightptr);
  ESP_LOGW(TAG, "loadimage on: start:0x%04x end: 0x%04x width: 0x%04x height: 0x%04x", ptron, ptrnext-1, width, height);

  // SPI Debugging
  gpio_set_level(GPIO_NUM_16, 1);
  gpio_set_level(GPIO_NUM_16, 0);

  //ft81x_get_touch_inputs();
  //ESP_LOGW(TAG, "ctouch mode: 0x%04x extended: 0x%04x", ft81x_ctouch.mode, ft81x_ctouch.extended);

  // Capture input events and update the image if touched
  for (int x=0; x<300; x++) {

      // Start streaming
      ft81x_stream_start();

      // Define the bitmap we want to draw
      ft81x_cmd_dlstart();  // Set REG_CMD_DL when done
      ft81x_cmd_swap();     // Set AUTO swap at end of display list

      // Draw ON/OFF based upon touch
      if (ft81x_ctouch.tag0) {
        ESP_LOGW(TAG, "touched");

        // Clear the display
        ft81x_clear_color_rgb32(0x28e800);        
        ft81x_clear();
        // Draw the image
        ft81x_bitmap_source(ptron);        
        
      } else {
        // Clear the display
        ft81x_clear_color_rgb32(0xfdfdfd);        
        ft81x_clear();
        // Draw the image
        ft81x_bitmap_source(ptroff);
        
      }


      // Turn on tagging
      ft81x_tag_mask(1);

      // Track touches for a specific object
      //ft81x_cmd_track(1, 1, ft81x_width, ft81x_height, 3); // track touches to the tag

      ft81x_bitmap_layout(ARGB4, 75*2, 75);
      ft81x_bitmap_size(NEAREST, BORDER, BORDER, 75, 75);
      ft81x_begin(BITMAPS);

      ft81x_tag(3); // tag the image button #3
      ft81x_vertex2ii(100, 100, 0, 0);

      // stop tagging
      ft81x_tag_mask(0);

      // end of commands
      ft81x_end();
      ft81x_display();

      // Trigger FT81x to read the command buffer
      ft81x_getfree(0);

      // Finish streaming to command buffer
      ft81x_stream_stop();

      //// Wait till the GPU is finished
      ft81x_wait_finish();

     // download the display touch memory into ft81x_touch
     ft81x_get_touch_inputs();
#if 0
     ESP_LOGW(TAG, "tag0: %i xy0: 0x%08x", ft81x_ctouch.tag0, ft81x_ctouch.tag0_xy);
     // multitouch
     // ESP_LOGW(TAG, "tag1: %i xy0: 0x%08x", ft81x_ctouch.tag1, ft81x_ctouch.tag1_xy);
#endif
     // Sleep
     vTaskDelay(10 / portTICK_PERIOD_MS);

  }

  // SPI Debugging
  gpio_set_level(GPIO_NUM_16, 1);
  gpio_set_level(GPIO_NUM_16, 0);

#endif

#if 0 // Test memory operation(s) and CRC32 on 6 bytes of 0x00 will be 0xB1C2A1A3
  // SPI Debugging
  gpio_set_level(GPIO_NUM_16, 1);
  gpio_set_level(GPIO_NUM_16, 0);

  // Start streaming
  ft81x_stream_start();

  // Write a predictable sequence of bytes to a memory location
  ft81x_cmd_memzero(0, 0x0006);

  // Calculate crc on the bytes we wrote
  uint32_t r = ft81x_cmd_memcrc(0, 0x0006);

  // Trigger FT81x to read the command buffer
  ft81x_getfree(0);

  // Finish streaming to command buffer
  ft81x_stream_stop();

  // Wait till the GPU is finished
  ft81x_wait_finish();

  // Dump results
  uint32_t res = ft81x_rd32(r);
  ESP_LOGW(TAG, "crc: ptr: 0x%04x val: 0x%4x expected: 0xB1C2A1A3", r, res);

  // SPI Debugging
  gpio_set_level(GPIO_NUM_16, 1);
  gpio_set_level(GPIO_NUM_16, 0);
#endif

#if 0 // Draw a gray screen and write Hello World, 123, button etc.
  // SPI Debugging
  gpio_set_level(GPIO_NUM_16, 1);
  gpio_set_level(GPIO_NUM_16, 0);

  // Wait till the GPU is finished
  for (int x=0; x<300; x++) {
      // Sleep
      vTaskDelay(200 / portTICK_PERIOD_MS);

      // download the display touch memory into ft81x_touch
      ft81x_get_touch_inputs();

      ft81x_stream_start(); // Start streaming
      ft81x_cmd_dlstart();  // Set REG_CMD_DL when done
      ft81x_cmd_swap();     // Set AUTO swap at end of display list
      ft81x_clear_color_rgb32(0xfdfdfd);
      ft81x_clear();
      ft81x_color_rgb32(0x101010);
      ft81x_bgcolor_rgb32(0xff0000);
      ft81x_fgcolor_rgb32(0x0000ff);

      // Turn off tagging
      ft81x_tag_mask(0);

      // Draw some text and a number display value of dial
      ft81x_cmd_text(240, 300, 30, OPT_CENTERY, "Hello World");

      ft81x_cmd_text(130, 200, 30, OPT_RIGHTX, "TAG");
      ft81x_cmd_number(140, 200, 30, 0, ft81x_touch.tracker0.tag_value);

      ft81x_cmd_text(130, 230, 30, OPT_RIGHTX, "VALUE");
      ft81x_cmd_number(140, 230, 30, 0, ft81x_touch.tracker0.track_value * 100 / 65535);

      ft81x_bgcolor_rgb32(0x007f7f);
      ft81x_cmd_clock(730,80,50,0,12,1,2,4);

      // Turn on tagging
      ft81x_tag_mask(1);

      ft81x_tag(3); // tag the button #3
      ft81x_cmd_track(10, 10, 140, 100, 3); // track touches to the tag
      ft81x_cmd_button(10, 10, 140, 100, 31, 0, "OK");

      ft81x_tag(4); // tag the button #4
      ft81x_cmd_track(300, 100, 1, 1, 4); // track touches to the tag
      ft81x_cmd_dial(300, 100, 100, OPT_FLAT, x * 100);

      uint8_t tstate = rand()%((253+1)-0) + 0;
      if(tstate > 128)
        ft81x_bgcolor_rgb32(0x00ff00);
      else
        ft81x_bgcolor_rgb32(0xff0000);

      ft81x_tag(5); // tag the spinner #5
      ft81x_cmd_toggle(500, 100, 100, 30, 0, tstate > 128 ? 0 : 65535, "YES\xffNO");

      // Turn off tagging
      ft81x_tag_mask(0);

      // Draw a keyboard
      ft81x_cmd_keys(10, 400, 300, 50, 26, 0, "12345");

      // FIXME: Spinner if used above does odd stuff? Seems ok at the end of the display.
      ft81x_cmd_spinner(500, 200, 3, 0);

      ft81x_display();
      ft81x_getfree(0);     // trigger FT81x to read the command buffer
      ft81x_stream_stop();  // Finish streaming to command buffer

      //// Wait till the GPU is finished
      ft81x_wait_finish();
  }

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
    vTaskDelay(100 / portTICK_PERIOD_MS);

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
    //ft81x_alpha_funct(0b111, 0b00000000);
    //ft81x_bitmap_handle(0b10101010);
    ft81x_bitmap_layout(0b11111, 0x00, 0x00);

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
    uint16_t rndx = rand()%((ft81x_width+1)-0) + 0;
    uint16_t rndy = rand()%((ft81x_height+1)-0) + 0;
    ft81x_point_size(size);
    ft81x_vertex2f(rndx<<4,rndy<<4); // defaut is 1/16th pixel precision
    ESP_LOGW(TAG, "c: x:%i y:%i z:%i", rndx, rndy, size);
    ft81x_display();
    ft81x_getfree(0);     // trigger FT81x to read the command buffer
    ft81x_stream_stop();  // Finish streaming to command buffer
    vTaskDelay(100 / portTICK_PERIOD_MS);

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
 * Get our current fifo write state location
 */
uint32_t ft81x_getwp() {
  return RAM_CMD + (ft81x_wp & 0xffc);
}

/*
 * Wrapper for CS to allow easier debugging
 */
void ft81x_cs(uint8_t n) {
  gpio_set_level(GPIO_NUM_5, n);
#if 0 // Testing SPI
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

  // start the transaction ISR watches the CS bit
  ft81x_cs(0);

  // transmit our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans);

  // end the transaction
  ft81x_cs(1);
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
  char *recvbuf=heap_caps_malloc(4, MALLOC_CAP_DMA);

  // set trans options.
  trans.base.flags = SPI_TRANS_VARIABLE_ADDR;
  if (ft81x_qio) {
    // Tell the ESP32 SPI ISR to accept MODE_XXX for QIO and DIO
    trans.base.flags |= SPI_TRANS_MODE_DIOQIO_ADDR;
    // Set this transaction to QIO
    trans.base.flags |= SPI_TRANS_MODE_QIO;
  }

  // Set the address
  trans.address_bits = 24;
  trans.base.addr = addr;

  // 1 byte is our dummy byte we will throw it away later
  trans.base.length = 24;
  trans.base.rxlength = 24;

  // point to our RX buffer.
  trans.base.rx_buffer = recvbuf;

  // start the transaction ISR watches CS bit
  ft81x_cs(0);

  // transmit our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans);

  // grab our return data skip dummy byte
  uint16_t ret = *((uint16_t *)&recvbuf[1]);

  // end the transaction
  ft81x_cs(1);

  // cleanup
  free(recvbuf);

  return ret;
}

/*
 * Write 24 bit address + dummy byte
 * and read the 32 bit result.
 * A total of 8 bytes will be on the SPI BUS.
 */
uint32_t ft81x_rd32(uint32_t addr)
{
  // setup trans memory
  spi_transaction_ext_t trans;
  memset(&trans, 0, sizeof(trans));

  // allocate memory for our return data
  char *recvbuf=heap_caps_malloc(5, MALLOC_CAP_DMA);

  // set trans options.
  trans.base.flags = SPI_TRANS_VARIABLE_ADDR;
  if (ft81x_qio) {
    // Tell the ESP32 SPI ISR to accept MODE_XXX for QIO and DIO
    trans.base.flags |= SPI_TRANS_MODE_DIOQIO_ADDR;
    // Set this transaction to QIO
    trans.base.flags |= SPI_TRANS_MODE_QIO;
  }

  // Set the address
  trans.address_bits = 24;
  trans.base.addr = addr;

  // 1 byte is our dummy byte we will throw it away later
  trans.base.length = 40;
  trans.base.rxlength = 40;

  // point to our RX buffer.
  trans.base.rx_buffer = recvbuf;

  // start the transaction ISR watches CS bit
  ft81x_cs(0);

  // transmit our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans);

  // grab our return data skip dummy byte
  uint32_t ret = *((uint32_t *)&recvbuf[1]);

  // end the transaction
  ft81x_cs(1);

  // cleanup
  free(recvbuf);

  return ret;
}

/*
 * Spool a large block of memory in chunks from the FT81X
 *
 * Currently the max chunk size on ESP32 with DMA of 0 is 32 bytes.
 */
 void ft81x_rdN(uint32_t addr, uint8_t *results, int8_t len)
 {
   while(len) {
     if (len < CHUNKSIZE) {
       ft81x_rdn(addr, results, len);
       // all done
       break;
     } else {
       ft81x_rdn(addr, results, CHUNKSIZE);
       len-=CHUNKSIZE; results+=CHUNKSIZE; addr+=CHUNKSIZE;
       if(len<0) len=0;
     }
   }
 }

void ft81x_rdn(uint32_t addr, uint8_t *results, int8_t len) {
  // setup trans memory
  spi_transaction_ext_t trans;
  memset(&trans, 0, sizeof(trans));

  // allocate memory for our return data and dummy byte
  char *recvbuf=heap_caps_malloc(len+1, MALLOC_CAP_DMA);

  // set trans options.
  trans.base.flags = SPI_TRANS_VARIABLE_ADDR;
  if (ft81x_qio) {
    // Tell the ESP32 SPI ISR to accept MODE_XXX for QIO and DIO
    trans.base.flags |= SPI_TRANS_MODE_DIOQIO_ADDR;
    // Set this transaction to QIO
    trans.base.flags |= SPI_TRANS_MODE_QIO;
  }

  // Set the address
  trans.address_bits = 24;
  trans.base.addr = addr;

  // 1 byte is our dummy byte we will throw it away later
  uint32_t lenN = (len * 8) + 8;
  trans.base.length = lenN;
  trans.base.rxlength = lenN;

  // point to our RX buffer.
  trans.base.rx_buffer = recvbuf; // RX buffer

  // start the transaction ISR watches CS bit
  ft81x_cs(0);

  // transmit our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans);

  // grab our return data skip dummy byte
  memcpy(results, &recvbuf[1], len);

  // end the transaction
  ft81x_cs(1);

  // cleanup
  free(recvbuf);
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
 * Write 24 bit address leave CS open for data to be written
 * A total of 3 bytes will be on the SPI BUS.
 */
void ft81x_wrA(uint32_t addr) {
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
  addr |= 0x800000;
  addr = SPI_REARRANGE_DATA(addr, 24);

  trans.base.length = 24;
  trans.base.tx_buffer = &addr;

  // start the transaction ISR watches CS bit
  ft81x_cs(0); // active CS

  // transmit our transaction to the ISR
  spi_device_transmit(ft81x_spi, (spi_transaction_t*)&trans);
  
}

/*
 * Write bytes to the spi port no tracking.
 */
void ft81x_wrN(uint8_t *buffer, uint8_t size) {
  
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

void ft81x_wrE(uint32_t addr) {
  // end the transaction
  ft81x_cs(1); // inactive CS  
}

/*
 * Spool a large block of memory in chunks into the FT81X
 * using the MEDIA FIFO registers. Currently the max chunk size
 * on ESP32 with DMA of 0 is 32 bytes.
 *
 * If the size is > the available space this routine will block.
 * To avoid blocking check free space before calling this routine.
 */
void ft81x_cSPOOL_MF(uint8_t *buffer, int32_t size) {
  uint32_t fullness;
  size_t rds;
  size_t ts;
  uint8_t stopped = 1; // Stopped at startup
  size_t written = 0;
  
  // Get the read pointer where the GPU is working currently
  uint32_t mf_rp = ft81x_rd32(REG_MEDIAFIFO_READ);

  // Calculate how full our fifo is based upon our read/write pointers
  fullness = (mf_wp - mf_rp) & (mf_size - 1);
#if 0
  ESP_LOGI(TAG, "rp1 0x%08x wp 0x%08x full 0x%08x", mf_rp, mf_wp, fullness);
#endif
  // Blocking! Keep going until all the data is sent. 
  do {

    // Wait till we have enough room to send a whole buffer
    if ( !(fullness < (mf_size - CHUNKSIZE)) ) {

      // Release the SPI bus
      ft81x_stream_stop(); stopped = 1;
      
      // Did we write anything? If so tell the FT813
      if (written) {
        ft81x_wr32(REG_MEDIAFIFO_WRITE, mf_wp);
        written = 0;
      }

      // sleep a little let other processes go.
      vTaskDelay(1 / portTICK_PERIOD_MS);

      // Get the read pointer where the GPU is working currently
      // consuming bytes.
      mf_rp = ft81x_rd32(REG_MEDIAFIFO_READ);

      // Calculate how full our fifo is based upon our read/write pointers
      fullness = (mf_wp - mf_rp) & (mf_size - 1);
      
      continue;
    }

    // resume streaming data if needed
    if (stopped) {
      // Start streaming to our fifo starting with our address
      // same as ft81x_stream_start() but different address area
      // and no auto wrapping :(
      ft81x_wrA(mf_base + mf_wp); stopped = 0;
    }

    // NOTE: AFAIK the FT81X MEDIA FIFO engine does not do automatic wrapping.
    // Do not write past end of buffer!
    // This is not in the documentation for the FIFO commands AFAIK.
    // FIXME: better description/docs
    rds = (mf_size - mf_wp);
    if (rds >= CHUNKSIZE) {
      rds = CHUNKSIZE;
    }

    // default write size to chunk size or enough to max the FIFO
    ts = rds;

    // if we have less than a chunk then update our write size
    if (size < CHUNKSIZE) {
      ts = size;
    }

    //// write the block to the FT81X
    ft81x_wrN((uint8_t *)buffer, ts);

    // increment the pointers/counters
    buffer+=ts;
    mf_wp+=ts;
    fullness+=ts;
    size-=ts;
    written+=ts;

    // If true it must have reached the end of the fifo buffer.
    // Reset state by forcing an overflow event above.
    if ( rds != CHUNKSIZE ) {
        fullness = mf_size;
    }

#if 0    
    ESP_LOGI(TAG, "rp2 0x%08x wp 0x%08x full 0x%08x", mf_rp, mf_wp, fullness);
    // sleep a little let other processes go.
    vTaskDelay(10 / portTICK_PERIOD_MS);
#endif

    // loop around if we overflow the fifo.
    mf_wp&=(mf_size-1);
    
  } while (size);

  // Release the SPI bus
  ft81x_stream_stop(); stopped = 1;
  
  // Did we write anything? If so tell the FT813
  if (written) {
    ft81x_wr32(REG_MEDIAFIFO_WRITE, mf_wp);
    written = 0;
  }
}

/*
 * Read the FT81x command pointer
 */
uint16_t ft81x_rp() {
  uint16_t rp = ft81x_rd16(REG_CMD_READ);
  if (rp == 0xfff) {
    printf("FT81X COPROCESSOR EXCEPTION\n");
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
  return rp;
}

/*
 * Write out padded bits to be sure we are 32 bit aligned
 * as required by the FT81X
 */
void ft81x_align(uint32_t written) {
  uint8_t dummy[4] = {0x00, 0x00, 0x00, 0x00};
  int8_t align = 4 - (written & 0x3);
  if (align & 0x3)
    ft81x_cN((uint8_t *)dummy, align);
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

  // force command to complete write to CMD_WRITE
  ft81x_wr16(REG_CMD_WRITE, ft81x_wp & 0xffc);

  do {
    vTaskDelay(10 / portTICK_PERIOD_MS);
    uint16_t rp = ft81x_rp();
    uint16_t howfull = (ft81x_wp - rp) & 4095;
    ft81x_freespace = (4096 - 4) - howfull;
#if 0
    ESP_LOGW(TAG, "fifoA: wp:0x%04x rp:0x%04x fs:0x%04x", ft81x_wp, rp, ft81x_freespace);
    vTaskDelay(25 / portTICK_PERIOD_MS);
#endif
  } while (ft81x_freespace < required);
  ft81x_stream_start();
}

void ft81x_checkfree(uint16_t required) {
  // check that we have space in our fifo buffer
  // block until the FT81X says we do.
  if (ft81x_freespace < required) {
    ft81x_getfree(required);
#if 0
    ESP_LOGW(TAG, "free: 0x%04x", ft81x_freespace);
#endif
  }
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
 * Write N bytes command into our command fifo buffer
 * while in stream() mode.
 * Use tx_buffer to transmit the bits. Must be less
 * than buffer size.
 */
void ft81x_cN(uint8_t *buffer, uint16_t size) {

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
 * Spool a large block of memory in chunks into the FT81X
 *
 * Currently the max chunk size on ESP32 with DMA of 0 is 32 bytes.
 */
void ft81x_cSPOOL(uint8_t *buffer, int32_t size) {
  int32_t savesize = size;

  while(size) {
    if (size < CHUNKSIZE) {
      // check that we have enough space then send command
      ft81x_checkfree(size);
      ft81x_cN((uint8_t *)buffer, size);
      // all done
      break;
    } else {
      ft81x_checkfree(CHUNKSIZE);
      ft81x_cN((uint8_t *)buffer, CHUNKSIZE);
      size-=CHUNKSIZE; buffer+=CHUNKSIZE;
      if(size<0) size=0;
    }
  }

  int8_t align = (4 - (savesize & 0x3)) & 0x3;
  ft81x_checkfree(align);
  ft81x_align(savesize);
}

/*
 * Wait until REG_CMD_READ == REG_CMD_WRITE indicating
 * the GPU has processed all of the commands in the
 * circular commad buffer
 */
void ft81x_wait_finish() {
#if 0
  uint32_t twp = ft81x_wp;
#endif
  uint16_t rp;
  ft81x_wp &= 0xffc;
  #if 0
    ESP_LOGW(TAG, "waitfin: twp:0x%04x wp:0x%04x rp:0x%04x", twp, ft81x_wp, ft81x_rp());
  #endif  
  while ( ((rp=ft81x_rp()) != ft81x_wp) ) {
  }
}

/*
 * Read the FT813 tracker and ctouch registers
 * and update our global touch state structure
 */
void ft81x_get_touch_inputs() {

  // read in the tracker memory to our local structure
  ft81x_rdN(REG_TRACKER, (uint8_t *)&ft81x_touch, sizeof(ft81x_touch));

  // read in a few other touch registers
  ft81x_ctouch.mode = ft81x_rd32(REG_CTOUCH_MODE) & 0x03;
  ft81x_ctouch.extended = ft81x_rd32(REG_CTOUCH_EXTENDED) & 0x01;

  // Read in the ctouch registers depending on multitouch mode some or all.
  if (!ft81x_ctouch.extended) {
    // Read in our all ctouch registers. Most are continuous but the last 3 are not :(
    // If we are in single touch mode then we only need a few registers.
    ft81x_rdN(REG_CTOUCH_TOUCH1_XY, (uint8_t *)&ft81x_ctouch.touch1_xy, sizeof(ft81x_ctouch)-(3*4));
    //// Patch in the last 3 values.
    ft81x_ctouch.touch4_x = ft81x_rd32(REG_CTOUCH_TOUCH4_X);
    ft81x_ctouch.touch2_xy = ft81x_rd32(REG_CTOUCH_TOUCH2_XY);
    ft81x_ctouch.touch3_xy = ft81x_rd32(REG_CTOUCH_TOUCH3_XY);
  } else {
    // Only bring in TOUCH0_XY, TAG0_XY and TAG0 for single touch mode
    ft81x_rdN(REG_CTOUCH_TOUCH0_XY, (uint8_t *)&ft81x_ctouch.touch0_xy, 4*3);
  }
#if 0
  ESP_LOGW(TAG, "ttag: %i val: 0x%08x", ft81x_touch.tracker0.tag_value, ft81x_touch.tracker0.track_value);
  ESP_LOGW(TAG, "ctag0: %i xy: 0x%08x", ft81x_ctouch.tag0, ft81x_ctouch.tag0_xy);
#endif
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
 * Programming Guide sections
 *
 */

/*
 * 4.4 ALPHA_FUNCT
 * Specify the alpha test function
 * SM20180828:QA:PASS
 */
void ft81x_alpha_funct(uint8_t func, uint8_t ref) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI(
      ( 0x09UL                 << 24) | // CMD 0x09      24 - 31
                                        // RESERVED      11 - 23
      ( (func       & 0x7L)    <<  8) | // func           8 - 10
      ( (ref        & 0xffL)   <<  0)   // ref            0 -  7
  );
}

/*
 * 4.5 BEGIN
 * Begin drawing a graphics primitive
 * SM20180828:QA:PASS
 */
void ft81x_begin(uint8_t prim) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x1fUL << 24) | (prim & 0x0f));
}

/*
 * 4.6 BITMAP_HANDLE
 * Specify the bitmap handle
 * SM20180828:QA:PASS
 */
void ft81x_bitmap_handle(uint8_t handle) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x05UL << 24) | (handle & 0x1f));
}

/*
 * 4.7 BITMAP_LAYOUT
 * Specify the source bitmap memory format and layout for the current handle
 * SM20180828:QA:PASS
 */
void ft81x_bitmap_layout(uint8_t format, uint16_t linestride, uint16_t height) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI(
      ( 0x07UL                 << 24) | // CMD 0x07      24 - 31
      ( (format     & 0x1fL)   << 19) | // format        19 - 23
      ( (linestride & 0x3ffL)  <<  9) | // linestride     9 - 18
      ( (height     & 0x1ffL)  <<  0)   // height         0 -  8
  );
}

/*
 * 4.8 BITMAP_LAYOUT_H
 * Specify the 2 most significant bits of the source bitmap memory format and layout for the current handle
 * SM20180828:QA:PASS
 */
void ft81x_bitmap_layout_h(uint8_t linestride, uint8_t height) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI(
      ( 0x28UL                 << 24) | // CMD 0x28      24 - 31
                                        // RESERVED       4 - 23
      ( (linestride & 0x3L)    <<  2) | // linestride     2 -  3
      ( (height     & 0x3L)    <<  0)   // height         0 -  1
  );
}

/*
 * 4.9 BITMAP_SIZE
 * Specify the screen drawing of bitmaps for the current handle
 * SM20180828:QA:PASS
 */
void ft81x_bitmap_size(uint8_t filter, uint8_t wrapx, uint8_t wrapy, uint16_t width, uint16_t height) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI(
      ( 0x08UL                 << 24) | // CMD 0x08      24 - 31
                                        // RESERVED      21 - 23
      ( (filter     & 0x1L)    << 20) | // filter        20 - 20
      ( (wrapx      & 0x1L)    << 19) | // wrapx         19 - 19
      ( (wrapy      & 0x1L)    << 18) | // wrapy         18 - 18
      ( (width      & 0x1ffL)   <<  9) | // width          9 - 17
      ( (height     & 0x1ffL)   <<  0)   // height         0 -  8
  );
}

/*
 * 4.10 BITMAP_SIZE_H
 * Specify the source address of bitmap data in FT81X graphics memory RAM_G
 * SM20180828:QA:PASS
 */
void ft81x_bitmap_size_h(uint8_t width, uint8_t height) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x29UL << 24) | (((width) & 0x3) << 2) | (((height) & 0x3) << 0));
}

/*
 * 4.11 BITMAP_SOURCE
 * Specify the source address of bitmap data in FT81X graphics memory RAM_G
 * SM20180828:QA:PASS
 */
void ft81x_bitmap_source(uint32_t addr) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x01UL << 24) | ((addr & 0x3fffffL) << 0));
}

/*
 * 4.12 BITMAP_TRANSFORM_A
 * Specify the A coefficient of the bitmap transform matrix
 * SM20180828:QA:PASS
 */
void ft81x_bitmap_transform_a(uint32_t a) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x15UL << 24) | ((a & 0xffffL) << 0));
}

/*
 * 4.13 BITMAP_TRANSFORM_B
 * Specify the B coefficient of the bitmap transform matrix
 * SM20180828:QA:PASS
 */
void ft81x_bitmap_transform_b(uint32_t b) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x16UL << 24) | ((b & 0xffffL) << 0));
}

/*
 * 4.14 BITMAP_TRANSFORM_C
 * Specify the C coefficient of the bitmap transform matrix
 * SM20180828:QA:PASS
 */
void ft81x_bitmap_transform_c(uint32_t c) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x17UL << 24) | ((c & 0xffffffL) << 0));
}

/*
 * 4.15 BITMAP_TRANSFORM_D
 * Specify the D coefficient of the bitmap transform matrix
 * SM20180828:QA:PASS
 */
void ft81x_bitmap_transform_d(uint32_t d) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x18UL << 24) | ((d & 0xffffL) << 0));
}

/*
 * 4.16 BITMAP_TRANSFORM_E
 * Specify the E coefficient of the bitmap transform matrix
 * SM20180828:QA:PASS
 */
void ft81x_bitmap_transform_e(uint32_t e) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x19UL << 24) | ((e & 0xffffL) << 0));
}

/*
 * 4.17 BITMAP_TRANSFORM_F
 * Specify the F coefficient of the bitmap transform matrix
 * SM20180828:QA:PASS
 */
void ft81x_bitmap_transform_f(uint32_t f) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x1aUL << 24) | ((f & 0xffffffL) << 0));
}

/*
 * 4.18 BLEND_FUNC
 * Specify pixel arithmetic
 * SM20180828:QA:PASS
 */
void ft81x_blend_func(uint8_t src, uint8_t dst) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x0bUL << 24) | ((src & 0x7L) << 3) | ((dst & 0x7L) << 0));
}

/*
 * 4.19 CALL
 * Execute a sequence of commands at another location in the display list
 * SM20180828:QA:PASS
 */
void ft81x_call(uint16_t dest) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x1dUL << 24) | ((dest & 0xffffL) << 0));
}

/*
 * 4.20 CELL
 * Specify the bitmap cell number for the VERTEX2F command
 * SM20180828:QA:PASS
 */
void ft81x_cell(uint8_t cell) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x06UL << 24) | ((cell & 0x7fL) << 0));
}

/*
 * 4.21 CLEAR
 * Clear buffers to preset values
 * SM20180828:QA:PASS
 */
void ft81x_clear() {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x26UL << 24) | 0x7);
}

void ft81x_clearCST(uint8_t color, uint8_t stencil, uint8_t tag) {
  uint8_t cst = 0;
  cst = color & 0x01;
  cst <<=1;
  cst |= (stencil & 0x01);
  cst <<=1;
  cst |= (tag & 0x01);

  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x26UL << 24) | cst);
}

/*
 * 4.21 CLEAR_COLOR_A
 * Specify clear value for the alpha channel
 * SM20180828:QA:PASS
 */
void ft81x_clear_color_a(uint8_t alpha) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x0fUL << 24) | (alpha & 0xffL));
}

/*
 * 4.23 CLEAR_COLOR_RGB
 * Specify clear values for red, green and blue channels
 * SM20180828:QA:PASS
 */
void ft81x_clear_color_rgb32(uint32_t rgb) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x2UL << 24) | (rgb & 0xffffffL));
}

void ft81x_clear_color_rgb888(uint8_t red, uint8_t green, uint8_t blue) {
   ft81x_clear_color_rgb32(((red & 0xffL) << 16) | ((green & 0xffL) << 8) | ((blue & 0xffL) << 0));
}


/*
 * 4.24 CLEAR_STENCIL
 * Specify clear value for the stencil buffer
 * SM20180828:QA:PASS
 */
void ft81x_clear_stencil(uint8_t stencil) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x11UL << 24) | ((stencil & 0xffL) << 0));
}

/*
 * 4.25 CLEAR_TAG
 * Specify clear value for the tag buffer
 * SM20180828:QA:PASS
 */
void ft81x_clear_tag(uint8_t tag) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x12UL << 24) | ((tag & 0xffL) << 0));
}

/*
 * 4.26 COLOR_A
 * Set the current color alpha
 * SM20180828:QA:PASS
 */
void ft81x_color_a(uint8_t alpha) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x10UL << 24) | ((alpha & 0xffL) << 0));
}

/*
 * 4.27 COLOR_MASK
 * Enable or disable writing of color components
 * SM20180828:QA:PASS
 */
void ft81x_color_mask(uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x20L << 24) | (((red & 0x1) << 3) | ((green & 0x1) << 2) | ((blue & 0x1) << 1) | ((alpha & 0x1) << 0)));
}

/*
 * 4.28 COLOR_RGB
 * Set the current color red, green, blue
 * SM20180828:QA:PASS
 */
void ft81x_color_rgb32(uint32_t rgb) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x4UL << 24) | (rgb & 0xffffffL));
}

void ft81x_color_rgb888(uint8_t red, uint8_t green, uint8_t blue) {
   ft81x_color_rgb32(((red & 0xffL) << 16) | ((green & 0xffL) << 8) | ((blue & 0xffL) << 0));
}

/*
 * 4.29 DISPLAY
 * End the display list. FT81X will ignore all commands following this command.
 * SM20180828:QA:PASS
 */
void ft81x_display() {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x0UL << 24));
}

/*
 * 4.30 END
 * End drawing a graphics primitive
 * SM20180828:QA:PASS
 */
void ft81x_end() {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x21UL << 24));
}

/*
 * 4.31 JUMP
 * Execute commands at another location in the display list
 * SM20180828:QA:PASS
 */
void ft81x_jump(uint16_t dest) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x1eUL << 24) | (dest & 0xffffL));
}

/*
 * 4.32 LINE_WIDTH
 * Specify the width of lines to be drawn with primitive LINES in 1/16 pixel precision
 * SM20180828:QA:PASS
 */
void ft81x_line_width(uint16_t width) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x0eUL << 24) | (width & 0xfff));
}

/*
 * 4.33 MACRO
 * Execute a single command from a macro register
 * SM20180828:QA:PASS
 */
void ft81x_macro(uint8_t macro) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x25UL << 24) | (macro & 0x1L));
}

/*
 * 4.34 NOP
 * No Operation
 * SM20180828:QA:PASS
 */
void ft81x_nop() {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x2dUL << 24));
}

/*
 * 4.35 PALETTE_SOURCE
 * Specify the base address of the palette
 * SM20180828:QA:PASS
 */
void ft81x_palette_source(uint32_t addr) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x2aUL << 24) | ((addr) & 0x3fffffUL));
}

/*
 * 4.36 POINT_SIZE
 * Specify the radius of points
 * SM20180828:QA:PASS
 */
void ft81x_point_size(uint16_t size) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x0dUL << 24) | ((size & 0x1fffL) << 0));
}

/*
 * 4.37 RESTORE_CONTEXT
 * Restore the current graphics context from the context stack
 * SM20180828:QA:PASS
 */
void ft81x_restore_context() {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x23UL << 24));
}

/*
 * 4.38 RETURN
 * Return from a previous CALL command
 * SM20180828:QA:PASS
 */
void ft81x_return() {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x24UL << 24));
}

/*
 * 4.39 SAVE_CONTEXT
 * Push the current graphics context on the context stack
 * SM20180828:QA:PASS
 */
void ft81x_save_context() {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x22UL << 24));
}

/*
 * 4.40 SCISSOR_SIZE
 * Specify the size of the scissor clip rectangle
 * SM20180828:QA:PASS
 */
void ft81x_scissor_size(uint16_t width, uint16_t height) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x1cUL << 24) | ((width & 0xfffL) << 12) | ((height & 0xfffL) << 0));
}

/*
 * 4.41 SCISSOR_XY
 * Specify the top left corner of the scissor clip rectangle
 * SM20180828:QA:PASS
 */
void ft81x_scissor_xy(uint16_t x, uint16_t y) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x1bUL << 24) | ((x & 0x7ffL) << 11) | ((y & 0x7ffL) << 0));
}

/*
 * 4.42 STENCIL_FUNC
 * Set function and reference value for stencil testing
 * SM20180828:QA:PASS
 */
void ft81x_stencil_func(uint8_t func, uint8_t ref, uint8_t mask) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x0aUL << 24) | ((func & 0xfL) << 16) | ((ref & 0xffL) << 8) | ((mask & 0xffL) << 0));
}

/*
 * 4.43 STENCIL_MASK
 * Control the writing of individual bits in the stencil planes
 * SM20180828:QA:PASS
 */
void ft81x_stencil_mask(uint8_t mask) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x013UL << 24) | ((mask & 0xffL) << 0));
}

/*
 * 4.44 STENCIL_OP
 * Set stencil test actions
 * SM20180828:QA:PASS
 */
void ft81x_stencil_op(uint8_t sfail, uint8_t spass) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x0cUL << 24) | ((sfail & 0x7L) << 3) | ((spass & 0x7L) << 0));
}

/*
 * 4.45 TAG
 * Attach the tag value for the following graphics objects
 * drawn on the screen. The initial tag buffer value is 255.
 * SM20180828:QA:PASS
 */
void ft81x_tag(uint8_t s) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x3UL << 24) | ((s & 0xffL) << 0));
}

/*
 * 4.46 TAG_MASK
 * Control the writing of the tag buffer
 * SM20180828:QA:PASS
 */
void ft81x_tag_mask(uint8_t mask) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x14UL << 24) | ((mask & 1L) << 0));
}

/*
 * 4.47 VERTEX2F
 * Start the operation of graphics primitives at the specified
 * screen coordinate, in the pixel precision defined by VERTEX_FORMAT
 * SM20180828:QA:PASS
 */
void ft81x_vertex2f(int16_t x, int16_t y) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x1UL << 30) | ((x & 0x7fffL) << 15) | ((y & 0x7fffL) << 0));
}

/*
 * 4.48 VERTEX2ii
 * Start the operation of graphics primitives at the specified
 * screen coordinate, in the pixel precision defined by VERTEX_FORMAT
 * SM20180828:QA:PASS
 */
void ft81x_vertex2ii(int16_t x, int16_t y, uint8_t handle, uint8_t cell) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI(
      ( 0x02UL                 << 30) | // CMD 0x02      30 - 31
      ( (x          & 0x1ffL)  << 21) | // x             21 - 29
      ( (y          & 0x1ffL)  << 12) | // y             12 - 20
      ( (handle     & 0x1fL)   <<  7) | // handle         7 - 11
      ( (cell       & 0x7fL)   <<  0)   // cell           0 -  6
  );
}

/*
 * 4.49 VERTEX_FORMAT
 * Set the precision of VERTEX2F coordinates
 * SM20180828:QA:PASS
 */
void ft81x_vertex_format(int8_t frac) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x27UL << 24) | (((frac) & 0x7) << 0));
}

/*
 * 4.50 VERTEX_TRANSLATE_X
 * Specify the vertex transformation’s X translation component
* SM20180828:QA:PASS
 */
void ft81x_vertex_translate_x(uint32_t x) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x2bUL << 24) | (((x) & 0x1ffffUL) << 0));
}

/*
 * 4.51 VERTEX_TRANSLATE_Y
 * Specify the vertex transformation’s Y translation component
* SM20180828:QA:PASS
 */
void ft81x_vertex_translate_y(uint32_t y) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cI((0x2cUL << 24) | (((y) & 0x1ffffUL) << 0));
}

/*
 * 5.11 CMD_DLSTART
 * Start a new display list
 * SM20180828:QA:PASS
 */
void ft81x_cmd_dlstart() {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cFFFFFF(0x00);
}

/*
 * 5.12 CMD_SWAP
 * Swap the current display list
 * SM20180828:QA:PASS
 */
void ft81x_cmd_swap() {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cFFFFFF(0x01);
}

/*
 * 5.13 CMD_COLDSTART
 * This command sets the co-processor engine to default reset states
 * SM20180828:QA:PASS
 */
void ft81x_cmd_coldstart() {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cFFFFFF(0x32);
}

/*
 * 5.14 CMD_INTERRUPT
 * trigger interrupt INT_CMDFLAG
 * SM20180828:QA:PASS
 */
void ft81x_cmd_interrupt(uint32_t ms) {
  // check that we have enough space then send command
  ft81x_checkfree(8);
  ft81x_cFFFFFF(0x02);
  ft81x_cI(ms);
}

/*
 * 5.15 CMD_APPEND
 * Append more commands to current display list
 * SM20180828:QA:PASS
 */
void ft81x_cmd_append(uint32_t ptr, uint32_t num) {
  // check that we have enough space then send command
  ft81x_checkfree(12);
  ft81x_cFFFFFF(0x1e);
  ft81x_cI(ptr);
  ft81x_cI(num);
}

/*
 * 5.16 CMD_REGREAD
 * Read a register value
 * FIXME
 */
void ft81x_cmd_regread(uint32_t ptr, uint32_t* result) {
  // check that we have enough space then send command
  ft81x_checkfree(8);
  ft81x_cFFFFFF(0x19);
  ft81x_cI(ptr);

  // The data will be written starting here in the buffer so get the pointer
  uint32_t r = ft81x_getwp();

  // Fill in memory where results will go with dummy data
  ft81x_cI(0xffffffff); // Will be result

  // report back memory locations of the results to caller
  *result = r;
}

/*
 * 5.17 CMD_MEMWRITE
 * Write bytes into memory
 */
void ft81x_cmd_memwrite(uint32_t ptr, uint32_t num) {
  // check that we have enough space then send command
  ft81x_checkfree(12);
  ft81x_cFFFFFF(0x1a);
  ft81x_cI(ptr);
  ft81x_cI(num);
}

/*
 * 5.18 CMD_INFLATE
 * Decompress data into memory
 */
void ft81x_cmd_inflate(uint32_t ptr) {
  // check that we have enough space then send command
  ft81x_checkfree(8);
  ft81x_cFFFFFF(0x22);
  ft81x_cI(ptr);
}

/*
 * 5.19 CMD_LOADIMAGE
 * Load a JPEG or PNG image
 * SM20180828:QA:PASS
 */
void ft81x_cmd_loadimage(uint32_t ptr, uint32_t options) {
  // check that we have enough space then send command
  ft81x_checkfree(12);
  ft81x_cFFFFFF(0x24);
  ft81x_cI(ptr);
  ft81x_cI(options);
}

/*
 * 5.20 CMD_MEDIAFIFO
 * set up a streaming media FIFO in RAM_G
 * SM20180828:QA:PASS
 */
void ft81x_cmd_mediafifo(uint32_t base, uint32_t size) {
  mf_wp = 0;
  mf_size = size;
  mf_base = base;
  
  // check that we have enough space then send command
  ft81x_checkfree(12);
  ft81x_cFFFFFF(0x39);
  ft81x_cI(base);
  ft81x_cI(size);
}

/*
 * 5.21 CMD_PLAYVIDEO
 * Video playback
 */
void ft81x_cmd_playvideo(uint32_t options) {
  // check that we have enough space then send command
  ft81x_checkfree(8);
  ft81x_cFFFFFF(0x3a);
  ft81x_cI(options);
}

/*
 * 5.22 CMD_VIDEOSTART
 * Initialize the AVI video decoder
 */
void ft81x_cmd_videostart() {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cFFFFFF(0x40);
}

/*
 * 5.23 CMD_VIDEOFRAME
 * Loads the next frame of video
 */
void ft81x_cmd_videoframe(uint32_t dst, uint32_t ptr) {
  // check that we have enough space then send command
  ft81x_checkfree(12);
  ft81x_cFFFFFF(0x40);
  ft81x_cI(dst);
  ft81x_cI(ptr);
}

/*
 * 5.24 CMD_MEMCRC
 * Compute a CRC-32 for memory
 */
uint32_t ft81x_cmd_memcrc(uint32_t ptr, uint32_t num) {
  // check that we have enough space then send command
  ft81x_checkfree(16);

  ft81x_cFFFFFF(0x18);
  ft81x_cI(ptr);
  ft81x_cI(num);

  // The data will be written here in the buffer so get the pointer
  uint32_t r = ft81x_getwp();

  // Dummy data where our results will go
  ft81x_cI(0xffffffff);
  return r;
}

/*
 * 5.25 CMD_MEMZERO
 * Write zero to a block of memory
 */
void ft81x_cmd_memzero(uint32_t ptr, uint32_t num) {
  // check that we have enough space then send command
  ft81x_checkfree(12);
  ft81x_cFFFFFF(0x1c);
  ft81x_cI(ptr);
  ft81x_cI(num);
}

/*
 * 5.26 CMD_MEMSET
 * Fill memory with a byte value
 */
void ft81x_cmd_memset(uint32_t ptr, uint32_t value, uint32_t num) {
  // check that we have enough space then send command
  ft81x_checkfree(12);
  ft81x_cFFFFFF(0x1b);
  ft81x_cI(value);
  ft81x_cI(num);
}

/*
 * 5.27 CMD_MEMCPY
 * Copy a block of memory
 */
void ft81x_cmd_memcpy(uint32_t dest, uint32_t src, uint32_t num) {
  // check that we have enough space then send command
  ft81x_checkfree(16);
  ft81x_cFFFFFF(0x1d);
  ft81x_cI(dest);
  ft81x_cI(src);
  ft81x_cI(num);
}

/*
 * 5.28 CMD_BUTTON
 * Draw a button
 */
void ft81x_cmd_button(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t font, uint16_t options, const char *s) {
  uint16_t b[6];
  b[0] = x;
  b[1] = y;
  b[2] = w;
  b[3] = h;
  b[4] = font;
  b[5] = options;
  uint32_t len = strlen(s)+1;
  int8_t align = (4 - (len & 0x3)) & 0x3;

  // check that we have enough space then send command
  ft81x_checkfree(4+sizeof(b)+len+align);
  ft81x_cFFFFFF(0x0d);
  ft81x_cN((uint8_t *)b, sizeof(b));
  ft81x_cN((uint8_t *)s, len);
  ft81x_align(len);
}

/*
 * 5.29 CMD_CLOCK
 * Draw an analog clock
 */
void ft81x_cmd_clock(uint16_t x, uint16_t y, uint16_t r, uint16_t options, uint16_t h, uint16_t m, uint16_t s, uint16_t ms) {
  uint16_t b[8];
  b[0] = x;
  b[1] = y;
  b[2] = r;
  b[3] = options;
  b[4] = h;
  b[5] = m;
  b[6] = s;
  b[7] = ms;

  // check that we have enough space then send command
  ft81x_checkfree(4+sizeof(b));
  ft81x_cFFFFFF(0x14);
  ft81x_cN((uint8_t *)b, sizeof(b));
}


/*
 * 5.30 CMD_FGCOLOR
 * set the foreground color
* SM20180828:QA:PASS
 */
void ft81x_fgcolor_rgb32(uint32_t rgb) {
  // check that we have enough space then send command
  ft81x_checkfree(8);
  ft81x_cFFFFFF(0x0a);
  ft81x_cI(rgb);
}
void ft81x_fgcolor_rgb888(uint8_t red, uint8_t green, uint8_t blue) {
  ft81x_fgcolor_rgb32(((red & 255L) << 16) | ((green & 255L) << 8) | ((blue & 255L) << 0));
}

/*
 * 5.31 CMD_BGCOLOR
 * Set the background color
 */
void ft81x_bgcolor_rgb32(uint32_t rgb) {
  // check that we have enough space then send command
  ft81x_checkfree(8);
  ft81x_cFFFFFF(0x09);
  ft81x_cI(rgb);
}
void ft81x_bgcolor_rgb888(uint8_t red, uint8_t green, uint8_t blue) {
   ft81x_bgcolor_rgb32(((red & 255L) << 16) | ((green & 255L) << 8) | ((blue & 255L) << 0));
}

/*
 * 5.32 CMD_GRADCOLOR
 * Set the 3D button highlight color
 */
void ft81x_gradcolor_rgb32(uint32_t rgb) {
  // check that we have enough space then send command
  ft81x_checkfree(8);
  ft81x_cFFFFFF(0x34);
  ft81x_cI(rgb);
}
void ft81x_gradcolor_rgb888(uint8_t red, uint8_t green, uint8_t blue) {
   ft81x_gradcolor_rgb32(((red & 255L) << 16) | ((green & 255L) << 8) | ((blue & 255L) << 0));
}

/*
 * 5.33 CMD_GAUGE
 * Draw a gauge
 */
void ft81x_cmd_gauge(int16_t x, int16_t y, int16_t r, uint16_t options, uint16_t major, uint16_t minor, uint16_t val, uint16_t range) {
  uint16_t b[7];
  b[0] = x;
  b[1] = y;
  b[2] = r;
  b[3] = options;
  b[4] = major;
  b[4] = minor;
  b[5] = val;
  b[6] = range;
  // check that we have enough space then send command
  ft81x_checkfree(4+sizeof(b));
  ft81x_cFFFFFF(0x13);
  ft81x_cN((uint8_t *)b,sizeof(b));
}

/*
 * 5.34 CMD_GRADIENT
 * Draw a smooth color gradient
 */
void ft81x_cmd_gradient(int16_t x0, int16_t y0, uint32_t rgb0, int16_t x1, int16_t y1, uint32_t rgb1) {
  uint16_t b[8];
  b[0] = x0;
  b[1] = y0;
  b[2] = rgb0 >> 16;
  b[3] = rgb0 & 0xffff;
  b[4] = x1;
  b[5] = y1;
  b[6] = rgb1 >> 16;
  b[7] = rgb1 & 0xffff;

  // check that we have enough space then send command
  ft81x_checkfree(4+sizeof(b));
  ft81x_cFFFFFF(0x0b);
  ft81x_cN((uint8_t *)b,sizeof(b));
}

/*
 * 5.35 CMD_KEYS
 * Draw a row of keys
 */
void ft81x_cmd_keys(int16_t x, int16_t y, int16_t w, int16_t h, int16_t font, uint16_t options, const char *s) {
  uint16_t b[6];
  b[0] = x;
  b[1] = y;
  b[2] = w;
  b[3] = h;
  b[4] = font;
  b[5] = options;

  uint32_t len = strlen(s)+1;
  int8_t align = (4 - (len & 0x3)) & 0x3;

  // check that we have enough space then send command
  ft81x_checkfree(4+sizeof(b)+len+align);
  ft81x_cFFFFFF(0x0e);
  ft81x_cN((uint8_t *)b,sizeof(b));
  ft81x_cN((uint8_t *)s, len);
  ft81x_align(len);
}

/*
 * 5.36 CMD_PROGRESS
 * Draw a progress bar
 */
void ft81x_cmd_progress(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t options, uint16_t val, uint16_t range) {
  uint16_t b[8];
  b[0] = x;
  b[1] = y;
  b[2] = w;
  b[3] = h;
  b[4] = options;
  b[5] = val;
  b[6] = range;
  b[7] = 0; // dummy pad

  // check that we have enough space then send command
  ft81x_checkfree(4+sizeof(b));
  ft81x_cFFFFFF(0x0f);
  ft81x_cN((uint8_t *)b,sizeof(b));
}

/*
 * 5.37 CMD_SCROLLBAR
 * Draw a scroll bar
 */
void ft81x_cmd_scrollbar(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t options, uint16_t val, uint16_t size, uint16_t range) {
  uint16_t b[8];
  b[0] = x;
  b[1] = y;
  b[2] = w;
  b[3] = h;
  b[4] = options;
  b[5] = val;
  b[6] = size;
  b[7] = range;

  // check that we have enough space then send command
  ft81x_checkfree(4+sizeof(b));
  ft81x_cFFFFFF(0x11);
  ft81x_cN((uint8_t *)b,sizeof(b));
}

/*
 * 5.38 CMD_SLIDER
 * Draw a slider
 */
void ft81x_cmd_slider(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t options, uint16_t val, uint16_t range) {
  uint16_t b[8];
  b[0] = x;
  b[1] = y;
  b[2] = w;
  b[3] = h;
  b[4] = options;
  b[5] = val;
  b[6] = range;
  b[7] = 0; // dummy pad

  // check that we have enough space then send command
  ft81x_checkfree(4+sizeof(b));
  ft81x_cFFFFFF(0x10);
  ft81x_cN((uint8_t *)b,sizeof(b));
}

/*
 * 5.39 CMD_DIAL
 * Draw a rotary dial control
 */
void ft81x_cmd_dial(int16_t x, int16_t y, int16_t r, uint16_t options, uint16_t val) {
  uint16_t b[6];
  b[0] = x;
  b[1] = y;
  b[2] = r;
  b[3] = options;
  b[4] = val;
  b[5] = 0; // dummy pad

  // check that we have enough space then send command
  ft81x_checkfree(4+sizeof(b));
  ft81x_cFFFFFF(0x2d);
  ft81x_cN((uint8_t *)b,sizeof(b));
}

/*
 * 5.40 CMD_TOGGLE
 * Draw a toggle switch
 */
void ft81x_cmd_toggle(int16_t x, int16_t y, int16_t w, int16_t font, uint16_t options, uint16_t state, const char *s) {
  uint16_t b[6];
  b[0] = x;
  b[1] = y;
  b[2] = w;
  b[3] = font;
  b[4] = options;
  b[5] = state;
  uint32_t len = strlen(s)+1;
  int8_t align = (4 - (len & 0x3)) & 0x3;

   // check that we have enough space then send command
  ft81x_checkfree(4+sizeof(b)+len+align);
  ft81x_cFFFFFF(0x12);
  ft81x_cN((uint8_t *)b, sizeof(b));
  ft81x_cN((uint8_t *)s, len);
  ft81x_align(len);
}

/*
 * 5.41 CMD_TEXT
 * Draw text
 */
void ft81x_cmd_text(int16_t x, int16_t y, int16_t font, uint16_t options, const char *s) {
  uint16_t b[4];
  b[0] = x;
  b[1] = y;
  b[2] = font;
  b[3] = options;
  uint32_t len = strlen(s)+1;
  int8_t align = (4 - (len & 0x3)) & 0x3;

  // check that we have enough space then send command
  ft81x_checkfree(4+sizeof(b)+len+align);
  ft81x_cFFFFFF(0x0c);
  ft81x_cN((uint8_t *)b, sizeof(b));
  ft81x_cN((uint8_t *)s, len);
  ft81x_align(len);
}

/*
 * 5.42 CMD_SETBASE
 * Set the base for number output
 */
void ft81x_cmd_setbase(uint32_t b) {
  // check that we have enough space then send command
  ft81x_checkfree(8);
  ft81x_cFFFFFF(0x38);
  ft81x_cI(b);
}

/*
 * 5.43 CMD_NUMBER
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

   // check that we have enough space then send command
   ft81x_checkfree(sizeof(b)+4);
   ft81x_cFFFFFF(0x2e);
   ft81x_cN((uint8_t *)&b,sizeof(b));
}

/*
 * 5.44 CMD_LOADIDENTITY
 * Set the current matrix to the identity matrix
 */
void ft81x_cmd_loadidentity() {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cFFFFFF(0x26);
}

/*
 * 5.45 CMD_SETMATRIX
 * Write the current matrix to the display list
 */
void ft81x_cmd_setmatrix() {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cFFFFFF(0x2a);
}

/*
 * 5.46 CMD_GETMATRIX
 * Retrieves the current matrix within the context of the co-processor engine
 */
void ft81x_cmd_getmatrix(int32_t *a, int32_t *b, int32_t *c, int32_t *d, int32_t *e, int32_t *f) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cFFFFFF(0x33);

  // The data will be written starting here in the buffer so get the pointer
  uint32_t r = ft81x_getwp();

  // Fill in memory where results will go with dummy data
  ft81x_cI(0xffffffff); // Will be a
  ft81x_cI(0xffffffff); // Will be b
  ft81x_cI(0xffffffff); // Will be c
  ft81x_cI(0xffffffff); // Will be d
  ft81x_cI(0xffffffff); // Will be e
  ft81x_cI(0xffffffff); // Will be f

  // report back memory locations of the results to caller
  *a = r;
  r+=4;
  *b = r;
  r+=4;
  *c = r;
  r+=4;
  *d = r;
  r+=4;
  *e = r;
  r+=4;
  *f = r;
}

/*
 * 5.47 CMD_GETPTR
 * Get the end memory address of data inflated by CMD_INFLATE
 */
void ft81x_cmd_getptr(uint32_t *result) {
  // check that we have enough space then send command
  ft81x_checkfree(8);
  ft81x_cFFFFFF(0x23);

  // The data will be written starting here in the buffer so get the pointer
  uint32_t r = ft81x_getwp();

  // Fill in memory where results will go with dummy data
  ft81x_cI(0xffffffff); // Will be ptr

  // report back memory locations of the results to caller
  *result = r;
}

/*
 * 5.48 CMD_GETPROPS
 * Get the image properties decompressed by CMD_LOADIMAGE
 * BLOCKING CALL, expects to be in streaming mode
 */
void ft81x_cmd_getprops(uint32_t *ptr, uint32_t *width, uint32_t *height) {

  // check that we have enough space then send command
  ft81x_checkfree(16);
  ft81x_cFFFFFF(0x25);

  // The data will be written starting here in the buffer so get the pointer
  uint32_t r = ft81x_getwp();

  // Fill in memory where results will go with dummy data
  ft81x_cI(0xffffffff); // Will be ptr
  ft81x_cI(0xffffffff); // Will be width
  ft81x_cI(0xffffffff); // Will be height

  // report back memory locations of the results to caller
  *ptr = r;
  r+=4;
  *width = r;
  r+=4;
  *height = r;
}

/*
 * 5.49 CMD_SCALE
 * Apply a scale to the current matrix
 */
void ft81x_cmd_scale(int32_t sx, int32_t sy) {
  // check that we have enough space then send command
  ft81x_checkfree(12);
  ft81x_cFFFFFF(0x28);
  ft81x_cI(sx);
  ft81x_cI(sy);
}

/*
 * 5.50 CMD_ROTATE
 * Apply a rotation to the current matrix
 */
void ft81x_cmd_rotate(int32_t a) {
  // check that we have enough space then send command
  ft81x_checkfree(8);
  ft81x_cFFFFFF(0x29);
  ft81x_cI(a);
}

/*
 * 5.51 CMD_TRANSLATE
 * Apply a translation to the current matrix
 */
void ft81x_cmd_translate(int32_t tx, int32_t ty) {
  // check that we have enough space then send command
  ft81x_checkfree(12);
  ft81x_cFFFFFF(0x27);
  ft81x_cI(tx);
  ft81x_cI(tx);
}

/*
 * 5.52 CMD_CALIBRATE
 * Execute the touch screen calibration routine
 */
void ft81x_cmd_calibrate(uint32_t *result) {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cFFFFFF(0x15);

  // The data will be written starting here in the buffer so get the pointer
  uint32_t r = ft81x_getwp();

  // Fill in memory where results will go with dummy data
  ft81x_cI(0xffffffff); // Will be result

  // report back memory locations of the results to caller
  *result = r;
}

void ft81x_calibrate() {
  ft81x_stream_start();  // Start streaming
  ft81x_clear_color_rgb32(0xffffff);
  ft81x_color_rgb32(0xffffff);
  ft81x_bgcolor_rgb32(0x402000);
  ft81x_fgcolor_rgb32(0x703800);
  ft81x_cmd_dlstart();   // Set REG_CMD_DL when done
  ft81x_clear();         // Clear the display
  ft81x_getfree(0);      // trigger FT81x to read the command buffer

  ft81x_cmd_text(180, 30, 40, OPT_CENTER, "Please tap on the dot..");
  //ft81x_cmd_calibrate(0);// Calibration command
  //ft81x_cmd_swap();      // Set AUTO swap at end of display list

  ft81x_stream_stop();   // Finish streaming to command buffer
  // Wait till the Logo is finished
  ft81x_wait_finish();
}

/*
 * 5.53 CMD_SETROTATE
 * Rotate the screen
 */
void ft81x_cmd_setrotate(uint32_t r) {
  // check that we have enough space then send command
  ft81x_checkfree(8);
  ft81x_cFFFFFF(0x36);
  ft81x_cI(r);

  // Get our screen size W,H to confirm
  ft81x_width = ft81x_rd16(REG_HSIZE);
  ft81x_height = ft81x_rd16(REG_VSIZE);

  // portrait mode swap w & h
  if (r & 2) {
    int t = ft81x_height;
    ft81x_height = ft81x_width;
    ft81x_width = t;
  }
}

/*
 * 5.54 CMD_SPINNER
 * Start an animated spinner
 */
void ft81x_cmd_spinner(int16_t x, int16_t y, int16_t style, int16_t scale) {
  uint16_t b[4];
  b[0] = x;
  b[1] = y;
  b[2] = style;
  b[3] = scale;

  // check that we have enough space to run the command
  ft81x_checkfree(sizeof(b)+4);
  ft81x_cFFFFFF(0x16);
  ft81x_cN((uint8_t *)&b,sizeof(b));
}

/*
 * 5.55 CMD_SCREENSAVER
 * Start an animated screensaver
 */
void ft81x_cmd_screensaver() {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cFFFFFF(0x2f);
}

/*
 * 5.56 CMD_SKETCH
 * Start a continuous sketch update
 */
void ft81x_cmd_sketch(int16_t x, int16_t y, int16_t w, int16_t h, int16_t ptr, int16_t format) {
  uint16_t b[6];
  b[0] = x;
  b[1] = y;
  b[2] = w;
  b[3] = h;
  b[4] = ptr;
  b[5] = format; // dummy pad

  // check that we have enough space then send command
  ft81x_checkfree(sizeof(b)+4);
  ft81x_cFFFFFF(0x30);
  ft81x_cN((uint8_t *)&b,sizeof(b));
}

/*
 * 5.57 CMD_STOP
 * Stop any active spinner, screensaver or sketch
 */
void ft81x_cmd_stop() {
  // check that we have enough space then send command
  ft81x_checkfree(4);
  ft81x_cFFFFFF(0x17);
}

/*
 * 5.58 CMD_SETFONT
 * Set up a custom font
 */
void ft81x_cmd_setfont(uint32_t font, uint32_t ptr) {
  // check that we have enough space then send command
  ft81x_checkfree(12);
  ft81x_cFFFFFF(0x2b);
  ft81x_cI(font);
  ft81x_cI(ptr);
}

/*
 * 5.59 CMD_SETFONT2
 * Set up a custom font
 */
void ft81x_cmd_setfont2(uint32_t handle, uint32_t font, uint32_t ptr, uint32_t firstchar) {
  // check that we have enough space then send command
  ft81x_checkfree(16);
  ft81x_cFFFFFF(0x3b);
  ft81x_cI(font);
  ft81x_cI(ptr);
  ft81x_cI(firstchar);
}

/*
 * 5.60 CMD_SETSCRATCH
 * Set the scratch bitmap for widget use
 */
void ft81x_cmd_setscratch(uint32_t handle) {
  // check that we have enough space then send command
  ft81x_checkfree(8);
  ft81x_cFFFFFF(0x3c);
  ft81x_cI(handle);
}

/*
 * 5.61 CMD_ROMFONT
 * Load a ROM font into bitmap handle
 */
void ft81x_cmd_romfont(uint32_t font, uint32_t slot) {
  // check that we have enough space then send command
  ft81x_checkfree(12);
  ft81x_cFFFFFF(0x3f);
  ft81x_cI(font);
  ft81x_cI(slot);
}


/*
 * 5.62 CMD_TRACK
 * Track touches for a graphics object
 */
void ft81x_cmd_track(int16_t x, int16_t y, int16_t width, int16_t height, int16_t tag) {
  uint16_t b[6];
  b[0] = x;
  b[1] = y;
  b[2] = width;
  b[3] = height;
  b[4] = tag;
  b[5] = 0; // dummy pad

  // check that we have enough space then send command
  ft81x_checkfree(4+sizeof(b));
  ft81x_cFFFFFF(0x2c);
  ft81x_cN((uint8_t *)&b,sizeof(b));
}

/*
 * 5.63 CMD_SNAPSHOT
 * Take a snapshot of the current screen
 */
void ft81x_cmd_snapshot(uint32_t ptr) {
  // check that we have enough space then send command
  ft81x_checkfree(8);
  ft81x_cFFFFFF(0x1f);
  ft81x_cI(ptr);
}

/*
 * 5.64 CMD_SNAPSHOT2
 * Take a snapshot of the current screen
 */
void ft81x_cmd_snapshot2(uint32_t fmt, uint32_t ptr, uint16_t x, uint16_t y, uint16_t width, uint16_t height) {
  uint16_t b[4];
  b[0] = x;
  b[1] = y;
  b[2] = width;
  b[3] = height;

  // check that we have enough space then send command
  ft81x_checkfree(12+sizeof(b));
  ft81x_cFFFFFF(0x37);
  ft81x_cI(fmt);
  ft81x_cI(ptr);
  ft81x_cN((uint8_t *)&b,sizeof(b));
}

/*
 * 5.65 CMD_SETBITMAP
 * Set up display list for bitmap
 */
void ft81x_cmd_setbitmap(uint32_t addr, uint16_t fmt, uint16_t width, uint16_t height) {
  uint16_t b[4];
  b[0] = fmt;
  b[1] = width;
  b[2] = height;
  b[3] = 0;

  // check that we have enough space then send command
  ft81x_checkfree(8+sizeof(b));
  ft81x_cFFFFFF(0x43);
  ft81x_cI(addr);
  ft81x_cN((uint8_t *)&b,sizeof(b));
}

/*
 * 5.66 CMD_LOGO
 * Play FTDI logo animation wait till it is done
 * FIXME: freespace()
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
