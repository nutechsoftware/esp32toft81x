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
#define FT81X_SPI_SPEED 1000000

//// QUAD SPI is not stable yet. Noise or ...?
//// Enable QUAD spi mode on ESP32 and FT81X
#define FT81X_QUADSPI         0

#define SPI_SHIFT_DATA(data, len) __builtin_bswap32((uint32_t)data<<(32-len))
#define SPI_REARRANGE_DATA(data, len) (__builtin_bswap32(data)>>(32-len))
#define WRITE_OP              1
#define READ_OP               0
#define ORIENTATION           0

// Table 6 FT81X graphics primitive operation definition
#define BITMAPS               1
#define POINTS                2
#define LINES                 3
#define LINE_STRIP            4
#define EDGE_STRIP_R          5
#define EDGE_STRIP_L          6
#define EDGE_STRIP_A          7
#define EDGE_STRIP_B          8
#define RECTS                 9

// Table 7 BITMAP_LAYOUT format list 
#define ARGB1555              0
#define L1                    1
#define L4                    2
#define L8                    3
#define RGB332                4
#define ARGB2                 5
#define ARGB4                 6
#define RGB565                7
#define PALETTED              8
#define TEXT8X8               9
#define TEXTVGA              10
#define BARGRAPH             11
#define PALETTED565          14
#define PALETTED4444         15
#define PALETTED8            16
#define L2                   17


#define NEAREST              0
#define BILINEAR             1
#define BORDER               0
#define REPEAT               1

// Table 8 BLEND_FUNC constant value definition
#define ZERO                  0
#define ONE                   1
#define SRC_ALPHA             2
#define DST_ALPHA             3
#define ONE_MINUS_SRC_ALPHA   4
#define ONE_MINUS_DST_ALPHA   5

// Table 13 FT81X OPTIONS
#define OPT_3D                0
#define OPT_RGB565            0
#define OPT_MONO              1
#define OPT_NODL              2
#define OPT_FLAT            256
#define OPT_SIGNED          256
#define OPT_CENTERX         512
#define OPT_CENTERY        1024
#define OPT_CENTER         1536
#define OPT_RIGHTX         2048
#define OPT_NOBACK         4096
#define OPT_NOTICKS        8192
#define OPT_NOHM          16384
#define OPT_NOPOINTER     16384
#define OPT_NOSECS        32768
#define OPT_NOHANDS       49152
#define OPT_NOTEAR            4
#define OPT_FULLSCREEN        8
#define OPT_MEDIAFIFO        16
#define OPT_SOUND            32

// Figure 5 FT81X The constants of ALPHA_FUNC
#define NEVER                 0
#define LESS                  1
#define LEQUAL                2
#define GREATER               3
#define GEQUAL                4
#define EQUAL                 5
#define NOTEQUAL              6
#define ALWAYS                7

// Figure 10 FT81X STENCIL_OP constants definition
#define ZERO                  0
#define KEEP                  1
#define REPLACE               2
#define INCR                  3
#define DECR                  4
#define INVERT                5

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
#define CMD_ACTIVE         0x00   // Switch from standby/sleep/pwrdown to active mode
#define CMD_STANDBY        0x41   // Put FT81X into standby mode
#define CMD_SLEEP          0x42   // Put FT81X core to sleep mode
#define CMD_PWRDOWN        0x43   // Switch off 1.2v core voltage. SPI still on.
#define CMD_PD_ROMS        0x49   // Select power down individual ROMs.
#define CMD_CLKEXT         0x44   // Select PLL input from external osc.
#define CMD_CLKINT         0x48   // Select PLL input from internal osc.
#define CMD_CLKSEL_A       0x61   // Set clock in sleep mode. TODO: why 2?
#define CMD_CLKSEL_B       0x62   // ""
#define CMD_RST_PULSE      0x68   // Send reset pulse to FT81x core.
#define CMD_PINDRIVE       0x70   // Pin driver power levels
#define PIN_PD_STATE       0x71   // Pin state when powered down Float/Pull-Down

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

// Get our current fifo write state location
uint32_t ft81x_getwp();

// Wrapper for CS to allow easier debugging
void ft81x_cs(uint8_t n);

// Send a Host Command to the FT81X chip see 4.1.5 Host Command
void ft81x_hostcmd(uint8_t command, uint8_t args);

// Send a 16 bit address + dummy and read the 8 bit response
uint8_t ft81x_rd(uint32_t addr);

// Send a 16 bit address + dummy and read the 16 bit response
uint16_t ft81x_rd16(uint32_t addr);

// Send a 16 bit address + dummy and read the 32 bit response
uint32_t ft81x_rd32(uint32_t addr);

// Send a 16 bit address and write 8 bits of data
void ft81x_wr(uint32_t addr, uint8_t byte);

// Send a 16 bit address and write 16 bits of data
void ft81x_wr16(uint32_t addr, uint16_t word);

// Send a 16 bit address and write 32 bits of data
void ft81x_wr32(uint32_t addr, uint32_t word);

// Read the FT81x command pointer
uint16_t ft81x_rp();

// Write out padded bits to be sure we are 32 bit aligned as required by the FT81X
void ft81x_align(uint32_t written);

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

// Spool a large block of memory in chunks into the FT81X in 1k chunks
void ft81x_cSPOOL(uint8_t *buffer, int32_t size);

// While in stream() mode send out a bitmap handle command into the command fifo
void ft81x_BitmapHandle(uint8_t byte);

// Series of commands to swap the display
void ft81x_swap();

// 4.4 ALPHA_FUNCT - Specify the alpha test function
void ft81x_alpha_funct(uint8_t func, uint8_t ref);

// 4.5 BEGIN - Begin drawing a graphics primitive
void ft81x_begin( uint8_t prim);

// 4.6 BEGIN_HANDLE - Specify the bitmap handle
void ft81x_bitmap_handle(uint8_t handle);

// 4.7 BITMAP_LAYOUT - Specify the source bitmap memory format and layout for the current handle
void ft81x_bitmap_layout(uint8_t format, uint16_t linestride, uint16_t height);

// 4.8 BITMAP_LAYOUT_H - Specify the 2 most significant bits of the source bitmap memory format and layout for the current handle
void ft81x_bitmap_layout_h(uint8_t linestride, uint8_t height);

// 4.9 BITMAP_SIZE - Specify the screen drawing of bitmaps for the current handle
void ft81x_bitmap_size(uint8_t filter, uint8_t wrapx, uint8_t wrapy, uint16_t width, uint16_t height);

// 4.10 BITMAP_SIZE_H - Specify the source address of bitmap data in FT81X graphics memory RAM_G
void ft81x_bitmap_size_h(uint8_t width, uint8_t height);

// 4.11 BITMAP_SOURCE - Specify the source address of bitmap data in FT81X graphics memory RAM_G
void ft81x_bitmap_source(uint32_t addr);

// 4.12 BITMAP_TRANSFORM_A - Specify the A coefficient of the bitmap transform matrix
void ft81x_bitmap_transform_a(uint32_t a);

// 4.13 BITMAP_TRANSFORM_B - Specify the B coefficient of the bitmap transform matrix
void ft81x_bitmap_transform_b(uint32_t b);

// 4.14 BITMAP_TRANSFORM_C - Specify the C coefficient of the bitmap transform matrix
void ft81x_bitmap_transform_c(uint32_t c);

// 4.15 BITMAP_TRANSFORM_D - Specify the D coefficient of the bitmap transform matrix
void ft81x_bitmap_transform_d(uint32_t d);

// 4.16 BITMAP_TRANSFORM_E - Specify the E coefficient of the bitmap transform matrix
void ft81x_bitmap_transform_e(uint32_t e);

// 4.17 BITMAP_TRANSFORM_F - Specify the F coefficient of the bitmap transform matrix
void ft81x_bitmap_transform_f(uint32_t f);

// 4.18 BLEND_FUNC - Specify pixel arithmetic
void ft81x_blend_func(uint8_t src, uint8_t dst);

// 4.19 CALL - Execute a sequence of commands at another location in the display list
void ft81x_call(uint16_t dest);

// 4.20 CELL - Specify the bitmap cell number for the VERTEX2F command
void ft81x_cell(uint8_t cell);

// 4.21 CLEAR - Clears buffers to preset values
void ft81x_clear();
void ft81x_clearCST(uint8_t color, uint8_t stencil, uint8_t tag);

// 4.21 CLEAR_COLOR_A - Specify clear value for the alpha channel
void ft81x_clear_color_a(uint8_t alpha);

// 4.23 CLEAR_COLOR_RGB - Specify clear values for red,green and blue channels
void ft81x_clear_color_rgb32(uint32_t rgb);
void ft81x_clear_color_rgb888(uint8_t red, uint8_t green, uint8_t blue);

// 4.24 CLEAR_STENCIL - Specify clear value for the stencil buffer
void ft81x_clear_stencil(uint8_t stencil);

// 4.25 CLEAR_TAG - Specify clear value for the tag buffer
void ft81x_clear_tag(uint8_t tag);

// 4.26 COLOR_A - Set the current color alpha
void ft81x_color_a(uint8_t alpha);

// 4.27 COLOR_MASK - Enable or disable writing of color components
void ft81x_color_mask(uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha);

// 4.28 COLOR_RGB - Set the current color red, green, blue
void ft81x_color_rgb32(uint32_t rgb);
void ft81x_color_rgb888(uint8_t red, uint8_t green, uint8_t blue);

// 4.29 DISPLAY - End the display list. FT81X will ignore all commands following this command.
void ft81x_display();

// 4.30 END - End drawing a graphics primitive
void ft81x_end();

// 4.31 JUMP - Execute commands at another location in the display list
void ft81x_jump(uint16_t dest);

// 4.32 LINE_WIDTH - Specify the width of lines to be drawn with primitive LINES in 1/16 pixel precision
void ft81x_line_width(uint16_t width);

// 4.33 MACRO - Execute a single command from a macro register
void ft81x_macro(uint8_t m);

// 4.34 NOP - No Operation
void ft81x_nop();

// 4.35 PALETTE_SOURCE - Specify the base address of the palette
void ft81x_palette_source(uint32_t addr);

// 4.36 POINT_SIZE - Specify the radius of points
void ft81x_point_size(uint16_t size);

// 4.37 RESTORE_CONTEXT - Restore the current graphics context from the context stack
void ft81x_restore_context();

// 4.38 RETURN - Return from a previous CALL command
void ft81x_return();

// 4.39 SAVE_CONTEXT - Push the current graphics context on the context stack
void ft81x_save_context();

// 4.40 SCISSOR_SIZE - Specify the size of the scissor clip rectangle
void ft81x_scissor_size(uint16_t width, uint16_t height);

// 4.41 SCISSOR_XY - Specify the top left corner of the scissor clip rectangle
void ft81x_scissor_xy(uint16_t x, uint16_t y);

// 4.42 STENCIL_FUNC - Set function and reference value for stencil testing
void ft81x_stencil_func(uint8_t func, uint8_t ref, uint8_t mask);

// 4.43 STENCIL_MASK - Control the writing of individual bits in the stencil planes
void ft81x_stencil_mask(uint8_t mask);

// 4.44 STENCIL_OP - Set stencil test actions
void ft81x_stencil_op(uint8_t sfail, uint8_t spass);

// 4.45 TAG - Attach the tag value for the following graphics objects drawn on the screen. def. 0xff
void ft81x_tag(uint8_t s);

// 4.46 TAG_MASK - Control the writing of the tag buffer
void ft81x_tag_mask(uint8_t mask);

// 4.47 VERTEX2F - Start the operation of graphics primitives at the specified screen coordinate
void ft81x_vertex2f(int16_t x, int16_t y);

// 4.48 VERTEX2II - Start the operation of graphics primitives at the specified screen coordinate
void ft81x_vertex2ii(int16_t x, int16_t y, uint8_t handle, uint8_t cell);

// 4.49 VERTEX_FORMAT - Set the precision of VERTEX2F coordinates
void ft81x_vertex_format(int8_t frac);

// 4.50 VERTEX_TRANSLATE_X - Specify the vertex transformation’s X translation component
void ft81x_vertex_translate_x(uint32_t x);

// 4.51 VERTEX_TRANSLATE_Y - Specify the vertex transformation’s Y translation component
void ft81x_vertex_translate_y(uint32_t y);

// 5.11 CMD_DLSTART - Start a new display list
void ft81x_cmd_dlstart();

// 5.12 CMD_SWAP - Swap the current display list
void ft81x_cmd_swap();

// 5.13 CMD_COLDSTART - This command sets the co-processor engine to default reset states
void ft81x_cmd_coldstart();

// 5.14 CMD_INTERRUPT - trigger interrupt INT_CMDFLAG
void ft81x_cmd_interrupt(uint32_t ms);

// 5.15 CMD_APPEND - Append more commands to current display list
void ft81x_cmd_append(uint32_t ptr, uint32_t num);

// 5.16 CMD_REGREAD - Read a register value
void ft81x_cmd_regread(uint32_t ptr, uint32_t *result);

// 5.17 CMD_MEMWRITE - Write bytes into memory
void ft81x_cmd_memwrite(uint32_t ptr, uint32_t num, void *mem);

// 5.18 CMD_INFLATE - Decompress data into memory
void ft81x_cmd_inflate(uint32_t ptr);

// 5.19 CMD_LOADIMAGE - Load a JPEG or PNG image
void ft81x_cmd_loadimage(uint32_t ptr, uint32_t options);

// 5.20 CMD_MEDIAFIFO - set up a streaming media FIFO in RAM_G
void ft81x_cmd_mediafifo(uint32_t ptr, uint32_t size);

// 5.21 CMD_PLAYVIDEO - Video playback
void ft81x_cmd_playvideo(uint32_t options, uint8_t *data);

// 5.22 CMD_VIDEOSTART - Initialize the AVI video decoder
void ft81x_cmd_videostart();

// 5.23 CMD_VIDEOFRAME - Load the next frame of video
void ft81x_cmd_videoframe(uint32_t dst, uint32_t ptr);

// 5.24 CMD_MEMCRC - Compute a CRC-32 for memory
uint32_t ft81x_cmd_memcrc(uint32_t ptr, uint32_t num);

// 5.25 CMD_MEMZERO - Write zero to a block of memory 
void ft81x_cmd_memzero(uint32_t ptr, uint32_t num);

// 5.26 CMD_MEMSET - Fill memory with a byte value
void ft81x_cmd_memset(uint32_t ptr, uint32_t value, uint32_t num);

// 5.27 CMD_MEMCPY - Copy a block of memory
void ft81x_cmd_memcpy(uint32_t dest, uint32_t src, uint32_t num);

// 5.28 CMD_BUTTON - Draw a button
void ft81x_cmd_button(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t font, uint16_t options, const char* s);

// 5.30 CMD_FGCOLOR - set the foreground color 
void ft81x_fgcolor_rgb32(uint32_t rgb);
void ft81x_fgcolor_rgb888(uint8_t red, uint8_t green, uint8_t blue);

// 5.31 CMD_BGCOLOR - set the background color 
void ft81x_bgcolor_rgb32(uint32_t rgb);
void ft81x_bgcolor_rgb888(uint8_t red, uint8_t green, uint8_t blue);

// 5.39 CMD_DIAL - Draw a rotary dial control
void ft81x_cmd_dial(int16_t x, int16_t y, int16_t r, uint16_t options, uint16_t val);

// 5.40 CMD_TOGGLE - Draw a toggle switch
void ft81x_cmd_toggle(int16_t x, int16_t y, int16_t w, int16_t font, uint16_t options, uint16_t state, const char* s);

// 5.41 CMD_TEXT - Draw text
void ft81x_cmd_text(int16_t x, int16_t y, int16_t font, uint16_t options, const char *s);

// 5.42 CMD_SETBASE - Set the background color 
void ft81x_cmd_setbase(uint32_t b);

// 5.43 CMD_NUMBER - Draw number
void ft81x_cmd_number(int16_t x, int16_t y, int16_t font, uint16_t options, int32_t n);

// 5.44 CMD_LOADIDENTITY - Set the current matrix to the identity matrix
void ft81x_cmd_loadidentity();

// 5.45 CMD_SETMATRIX FIXME - Write the current matrix to the display list
void ft81x_cmd_setmatrix();
  
// 5.46 CMD_GETMATRIX FIXME - Retrieves the current matrix within the context of the co-processor engine
void ft81x_cmd_getmatrix(int32_t *a, int32_t *b, int32_t *c, int32_t *d, int32_t *e, int32_t *f);
  
// 5.47 CMD_GETPTR FIXME - Get the end memory address of data inflated by CMD_INFLATE
void ft81x_cmd_getptr(uint32_t *result);

// 5.48 CMD_GETPROPS FIXME - Get the image properties decompressed by CMD_LOADIMAGE
void ft81x_cmd_getprops(uint32_t *ptr, uint32_t *width, uint32_t *height);

// 5.49 CMD_SCALE - Apply a scale to the current matrix
void ft81x_cmd_scale(int32_t sx, int32_t sy);

// 5.50 CMD_ROTATE - Apply a rotation to the current matrix
void ft81x_cmd_rotate(int32_t a);

// 5.51 CMD_TRANSLATE - Apply a translation to the current matrix
void ft81x_cmd_translate(int32_t tx, int32_t ty);

// 5.52 CMD_CALIBRATE - Execute the touch screen calibration routine
void ft81x_cmd_calibrate(uint32_t *result);
void ft81x_calibrate();

// 5.53 CMD_SETROTATE - Rotate the screen
void ft81x_cmd_setrotate(uint32_t r);

// 5.54 CMD_SPINNER - Start an animated spinner
void ft81x_cmd_spinner(int16_t x, int16_t y, int16_t style, int16_t scale);

// 5.55 CMD_SCREENSAVER - Start an animated screensaver
void ft81x_cmd_screensaver();

// 5.56 CMD_SKETCH - Start a continuous sketch update
void ft81x_cmd_sketch(int16_t x, int16_t y, int16_t w, int16_t h, int16_t ptr, int16_t format);

// 5.57 CMD_STOP - Stop any active spinner, screensaver or sketch
void ft81x_cmd_stop();

// 5.58 CMD_SETFONT - Set up a custom font
void ft81x_cmd_setfont(uint32_t font, uint32_t ptr);

// 5.59 CMD_SETFONT2 - Set up a custom font
void ft81x_cmd_setfont2(uint32_t handle, uint32_t font, uint32_t ptr, uint32_t firstchar);

// 5.60 CMD_SETSCRATCH - Set the scratch bitmap for widget use
void ft81x_cmd_setscratch(uint32_t handle);
  
// 5.61 CMD_ROMFONT - Load a ROM font into bitmap handle
void ft81x_cmd_romfont(uint32_t font, uint32_t slot);

// 5.62 CMD_TRACK - Track touches for a graphics object
void ft81x_cmd_track(int16_t x, int16_t y, int16_t width, int16_t height, int16_t tag);

// 5.63 CMD_SNAPSHOT - Take a snapshot of the current screen
void ft81x_cmd_snapshot(uint32_t ptr);

// 5.64 CMD_SNAPSHOT2 - Take a snapshot of the current screen
void ft81x_cmd_snapshot2(uint32_t fmt, uint32_t ptr, uint16_t x, uint16_t y, uint16_t width, uint16_t height);

// 5.65 CMD_SETBITMAP - Set up display list for bitmap
void ft81x_cmd_setbitmap(uint32_t addr, uint16_t fmt, uint16_t width, uint16_t height);

// 5.66 CMD_LOGO - play FTDI logo animation
void ft81x_logo();

// Wait for READ and WRITE circular buffer command pointers to be equal
void ft81x_wait_finish();

extern uint16_t ft81x_chip_id;

#endif
