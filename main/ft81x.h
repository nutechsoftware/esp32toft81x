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
#define FT81X_SPI_SPEED 2000000 // 2mhz testing
#define FT81X_QUADSPI 1         // Enable QUAD spi mode on ESP32 and FT81X
#define SPI_SHIFT_DATA(data, len) __builtin_bswap32((uint32_t)data<<(32-len))
#define SPI_REARRANGE_DATA(data, len) (__builtin_bswap32(data)>>(32-len))


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

// Send a 32 word into our command buffer
void ft81x_cmd32(uint32_t word);

// Send out a bitmap handle command into our command fifo
void ft81x_BitmapHandle(uint8_t byte);

// Read the FT81x command pointer
uint16_t ft81x_rp();

// get command buffer free block till we have 'required' space
void ft81x_getfree(uint16_t required);


extern uint16_t ft81x_chip_id;

#endif
