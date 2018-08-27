# ESP32 FT81X driver and sample
Display "Hello World" on a [**NHD-7.0-800480FT-CSXV-CTP**](http://newhavendisplay.com/learnmore/EVE2_7-CSXV-CTP/) display using a [**ESP32 Thing**](https://www.sparkfun.com/products/13907).

This code provides an API to communicate with the FT81X chip from an ESP32 Thing. It simplifies the complexity of SPI on the ESP32 by formatting the SPI communications to work correctly with the FT81X. It also allows for QUAD SPI communications where permitted to increase data transfer speeds.

## Wiring development environment
- Using the VSPI pins of the ESP32 connected to the FT81X display

| COLOR  | ESP_SIGNAL | ESP_32_PIN | DISPLAY_PIN | DISPLAY_SIGNAL |
| ------ | ---------- | ---------- | ----------- | -------------- |
| RED    | VCC 3.3v   | 3v3        | 1           | 3v3            |
| ORANGE | VDD 3.3-5v | NA         | 17          | BL_VDD         |
| BLACK  | GND        | GND        | 2           | GND            |
| WHITE  | CS0        | 5          | 6           | FT_CS          |
| PURPLE | SCLK       | 18         | 3           | FT_CLK         |
| YELLOW | MOSI       | 23         | 5           | FT_MOSI/IO0    |
| GREEN  | MISO       | 19         | 4           | FT_MISO/IO1    |
| GRAY   | QUADWP     | 22         | 11          | FT_GPIO0/IO2   |
| BLUE   | QUADHD     | 21         | 12          | FT_GPIO1/IO3   |

## Configuring
- Under menuconfig in **Serial flasher config** set the **Default serial port**
```console
foo@bar:~$ make menuconfig
```

## Building
- Requires an esp-idf development environment and **IDF-PATH** environment variable set to the path of your esp-idf folder (e.g., **IDF_PATH=/opt/esp/esp-idf** or **IDF_PATH=~/esp/esp-idf**).
  - https://esp-idf.readthedocs.io/en/latest/get-started/index.html
- Compile and program the ESP32 Thing
```console
foo@bar:~$ make flash
```

## Notes
- Power / Noise
 - You can power the ESP32 and NHD-7.0 from the ESP32 Thing connected to a USB port
   - Insufficient current will result in unpredictable behavior that may cause damage
   - Connect ***BL_VDD*** to ***3v3***
   - Keep the display PWM low(<20)
   - Use proper bypass caps near power connection of the ESP32 Thing
 - A **REG_PWM_DUTY** of 20 uses ~280ma and a PWM of 128 uses ~950ma.
   - This is based upon the default REG_PWM_HZ frequency of 250hz.

## Contributors
 - Submit issues and contribute improvements on [github/nutechsoftware](https://github.com/nutechsoftware)

## Authors
 - Sean Mathews <coder@f34r.com> - Initial skeleton and R&D

## License
 - [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
