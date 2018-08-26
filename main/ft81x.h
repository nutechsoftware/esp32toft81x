/**
 *  @file    ft81x.h
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

#ifndef _FT81X_H_
#define _FT81X_H_

/*
 * Defines
 */ 

// General settings/utils
//// 5-8mhz is the best I see so far
#define FT81X_SPI_SPEED 8000000

//// QUAD SPI is not stable yet. Noise or ...?
//// Enable QUAD spi mode on ESP32 and FT81X
#define FT81X_QUADSPI 0            

#define SPI_SHIFT_DATA(data, len) __builtin_bswap32((uint32_t)data<<(32-len))
#define SPI_REARRANGE_DATA(data, len) (__builtin_bswap32(data)>>(32-len))
#define WRITE_OP 1
#define READ_OP 0
#define ORIENTATION 0

// OPTIONS
#define OPT_3D            0
#define OPT_RGB565        0
#define OPT_MONO          1
#define OPT_NODL          2
#define OPT_FLAT        256
#define OPT_SIGNED      256
#define OPT_CENTERX     512
#define OPT_CENTERY    1024
#define OPT_CENTER     1536
#define OPT_RIGHTX     2048
#define OPT_NOBACK     4096
#define OPT_NOTICKS    8192
#define OPT_NOHM      16384
#define OPT_NOPOINTER 16384
#define OPT_NOSECS    32768
#define OPT_NOHANDS   49152
#define OPT_NOTEAR        4
#define OPT_FULLSCREEN    8
#define OPT_MEDIAFIFO    16
#define OPT_SOUND        32

// Table 6 FT81X graphics primitive operation definition
#define BITMAPS           1
#define POINTS            2
#define LINES             3
#define LINE_STRIP        4
#define EDGE_STRIP_R      5
#define EDGE_STRIP_L      6
#define EDGE_STRIP_A      7
#define EDGE_STRIP_B      8
#define RECTS             9


// MEMORY MAP DEFINES
#define RAM_G          0x000000UL // General purpose graphics RAM
#define ROM_FONT       0x1e0000UL // Font table and bitmap
#define ROM_FONT_ADDR  0x2ffffcUL // Font table pointer address
#define RAM_DL         0x300000UL // Display List RAM
#define RAM_REG        0x302000UL // Registers
#define RAM_CMD        0x308000UL // Command Buffer

// REGISTERS                       BITSRW DESC
#define REG_ID         0x302000UL //  8RO Chip id 0x7ch
#define REG_FRAMES     0x302004UL // 32RO frame counter since reset
#define REG_CLOCK      0x302008UL // 32RO Clock cyc since reset
#define REG_FREQUENCY  0x30200cUL // 28RW Main clock frequency (Hz)
#define REG_RENDERMODE 0x302010UL //  1RW Render mode : 0 = normal, 1 = single line
#define REG_SNAPY      0x302014UL // 11RW Scanline select for RENDERMODE 1
#define REG_SNAPSHOT   0x302018UL //  1RW Trigger for RENDERMODE 1
#define REG_SNAPFORMAT 0x30201cUL //  6RW Pixel format for scanline redout
#define REG_TAP_CRC    0x302024UL // 32RO Live video tap crc. Frame CRC is
#define REG_TAP_MASK   0x302028UL // 32RW Live video tap mask
#define REG_HCYCLE     0x30202CUL // 12RW Horizontal total cycle count
#define REG_HOFFSET    0x302030UL // 12RW Horizontal display start offset
#define REG_HSIZE      0x302034UL // 12RW Horizontal display pixel count
#define REG_HSYNC0     0x302038UL // 12RW Horizontal sync fall offset
#define REG_HSYNC1     0x30203cUL // 12RW Horizontal sync rise offset

#define REG_VCYCLE     0x302040UL // 12RW Vertical total cycle count
#define REG_VOFFSET    0x302044UL // 12RW Vertical display start offset
#define REG_VSIZE      0x302048UL // 12RW Vertical display line count
#define REG_VSYNC0     0x30204cUL // 10RW Vertical sync fall offset
#define REG_VSYNC1     0x302050UL // 10RW Vertical sync rise offset
#define REG_ROTATE     0x302058UL //  3RW Screen rot control. norm, invert, mirror portrait etc.

#define REG_DITHER     0x302060UL //  1RW Output dither enable
#define REG_PCLK_POL   0x30206cUL //  1RW PCLK Polarity out on edge 0=rise 1=fall
#define REG_PCLK       0x302070UL //  8RW PCLK frequency divider, 0=disable

#define REG_SWIZZLE    0x302064UL //  4RW Output RGB signal swizzle
//#define REG_     0x3020UL // 


#define REG_GPIO_DIR   0x302090UL //  8RW Legacy GPIO pin direction 0=In 1=Out
#define REG_GPIO       0x302094UL //  8RW Legacy GPIO pin direction
#define REG_GPIOX_DIR  0x302098UL // 16RW Extended GPIO pin direction 0=In 1=Out
#define REG_GPIOX      0x30209cUL // 16RW Extended GPIO read/write
#define REG_PWM_DUTY   0x3020d4UL //  8RW Back-light PWM duty cycle

#define REG_CMD_READ   0x3020f8UL // 12RW Command buffer read pointer
#define REG_CMD_WRITE  0x3020fcUL // 12RO Command buffer write pointer

#define REG_SPI_WIDTH  0x302188UL //  3RW QSPI bus width and dummy cycle setting

// COMMANDS 
#define CMD_ACTIVE     0x00       // Switch from standby/sleep/pwrdown to active mode
#define CMD_STANDBY    0x41       // Put FT81X into standby mode
#define CMD_SLEEP      0x42       // Put FT81X core to sleep mode
#define CMD_PWRDOWN    0x43       // Switch off 1.2v core voltage. SPI still on.
#define CMD_PD_ROMS    0x49       // Select power down individual ROMs.
#define CMD_CLKEXT     0x44       // Select PLL input from external osc.
#define CMD_CLKINT     0x48       // Select PLL input from internal osc.
#define CMD_CLKSEL_A   0x61       // Set clock in sleep mode. TODO: why 2?
#define CMD_CLKSEL_B   0x62       // ""
#define CMD_RST_PULSE  0x68       // Send reset pulse to FT81x core.
#define CMD_PINDRIVE   0x70       // Pin driver power levels
#define PIN_PD_STATE   0x71       // Pin state when powered down Float/Pull-Down

/*
 * Types
 */

/*
 * Prototypes
 */ 

// Initialize the ESP32 SPI device driver for the FT81X chip attached to the VSPI pins
bool ft81x_initSPI();

// Initialize FT81X GPU
bool ft81x_initGPU();

// reset the fifo state vars
void ft81x_reset_fifo();

// Wrapper for CS to allow easier debugging
void ft81x_cs(uint8_t n);

// Send a Host Command to the FT81X chip see 4.1.5 Host Command
void ft81x_hostcmd(uint8_t command, uint8_t args);

// Send a 16 bit address + dummy and read by the 8 bit response
uint8_t ft81x_rd(uint32_t addr);

// Send a 16 bit address + dummy and read by the 16 bit response
uint16_t ft81x_rd16(uint32_t addr);

// Send a 16 bit address and write 8 bits of data
void ft81x_wr(uint32_t addr, uint8_t byte);

// Send a 16 bit address and write 16 bits of data
void ft81x_wr16(uint32_t addr, uint16_t word);

// Send a 16 bit address and write 32 bits of data
void ft81x_wr32(uint32_t addr, uint32_t word);

// Read the FT81x command pointer
uint16_t ft81x_rp();

// Set the address and write mode pointer to the command buffer
// leaving the CS line enabled for further data
void ft81x_start(uint32_t addr, uint8_t write);

// Set the current address and write mode to the fifo comand buffer
// leaving the CS line enabled
void ft81x_stream_start();

// Disable the CS line finish the transaction
void ft81x_stream_stop();

// Get command buffer free block till we have 'required' space
void ft81x_getfree(uint16_t required);

// Write a 32bit command to the command buffer blocking if
// free space is needed until enough is free to send the 4 bytes
void ft81x_cI(uint32_t word);

// Write a 8bit+24bits of 0xff to the command buffer blocking if
// free space is needed until enough is free to send the 4 bytes
void ft81x_cFFFFFF(uint8_t byte);

// While in stream() mode send a 32 word into the command buffer.
void ft81x_cmd32(uint32_t word);

// While in stream() mode send a char buffer into the command buffer.
void ft81x_cN(uint8_t *buffer, uint16_t size);

// While in stream() mode send out a bitmap handle command into the command fifo
void ft81x_BitmapHandle(uint8_t byte);

// Series of commands to swap the display
void ft81x_swap();

// 4.21 CLEAR - Clears buffers to preset values
void ft81x_clear();

// 4.23 CLEAR_COLOR_RGB - Specify clear values for red,green and blue channels
void ft81x_clear_color_rgb32(uint32_t rgb);
void ft81x_clear_color_rgb888(uint8_t red, uint8_t green, uint8_t blue);

// 4.28 COLOR_RGB - Set the current color red, green, blue
void ft81x_color_rgb32(uint32_t rgb);
void ft81x_color_rgb888(uint8_t red, uint8_t green, uint8_t blue);

// 5.30 CMD_FGCOLOR - set the foreground color 
void ft81x_fgcolor_rgb32(uint32_t rgb);
void ft81x_fgcolor_rgb888(uint8_t red, uint8_t green, uint8_t blue);

// 5.31 CMD_BGCOLOR - set the background color 
void ft81x_bgcolor_rgb32(uint32_t rgb);
void ft81x_bgcolor_rgb888(uint8_t red, uint8_t green, uint8_t blue);

// 5.12 CMD_SWAP - swap the current display list
void ft81x_cmd_swap();

// 4.29 DISPLAY - End the display list. FT81X will ignore all commands following this command.
void ft81x_display();

// 5.44 CMD_LOADIDENTITY - Set the current matrix to the identity matrix
void ft81x_cmd_loadidentity();

// 5.11 CMD_DLSTART - start a new display list
void ft81x_cmd_dlstart();

// 5.53 CMD_SETROTATE - rotate the screen
void ft81x_cmd_setrotate(uint32_t r);

// 5.41 CMD_TEXT - draw text
void ft81x_cmd_text(int16_t x, int16_t y, int16_t font, uint16_t options, const char *s);

// 5.43 CMD_NUMBER - draw number
void ft81x_cmd_number(int16_t x, int16_t y, int16_t font, uint16_t options, int32_t n);

// 5.66 CMD_LOGO - play FTDI logo animation
void ft81x_logo();

// 4.47 VERTEX2F - Start the operation of graphics primitives at the specified screen coordinate
void ft81x_vertex2f(int16_t x, int16_t y);

// 4.48 VERTEX2II - Start the operation of graphics primitives at the specified screen coordinate
void ft81x_vertex2ii(int16_t x, int16_t y, uint8_t handle, uint8_t cell);

// 4.36 POINT_SIZE - Specify the radius of points
void ft81x_point_size(uint16_t size);

// 4.5 BEGIN - Begin drawing a graphics primitive
void ft81x_begin( uint8_t prim);

// 4.30 END - End drawing a graphics primitive
void ft81x_end();

// Wait for READ and WRITE command ptrs to be 0
void ft81x_wait_finish();

extern uint16_t ft81x_chip_id;

#endif
