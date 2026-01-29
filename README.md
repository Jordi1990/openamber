![GitHub Release](https://img.shields.io/github/v/release/Jordi1990/openamber)

# OpenAmber
Open source control firmware based on ESPHome for Itho Daalderop Amber heat pump, replaces the WinCE controller with an ESP32.

> [!CAUTION]
> This firmware is very much experimental and far from a finished product.

HomeAssistant dashboard example
![HomeAssistant example](/docs/images/openamber-hass-example.png)

Supply temperature regulation with PID-based compressor Hz modulation
![Supply temperature regulation with PID-based compressor Hz modulation](/docs/images/openamber-pid.png)

## Features
- Domestic Hot Water/Three way valve control
- Heating
- Pump P0 full PWM control on customizable interval/duration/speed.
- Compressor modulation based on PID controller.
- Configure maximum power modes
- Frost protection in 2 stages (similair to original software).
- Software thermostat based on Tr sensor.
- Heat curve(5-points).
- Backup heater
- Boost after defrost using backup heater.
- Sensors for error codes
- Support external thermostats (Heating/Cooling input)

## Todo
- Support cooling
- Tv1/Tv2 buffer support
- Support Smart Grid inputs
- Extend configuration options
- Error handling
- Support OpenTherm thermostats
- EEPROM updates
- .. More

## Hardware
The firmware can be used on any ESP32 with an RS485 board.

### Off-the-shelf boards:
#### Itho Daalderop Amber Control Module (https://electropaultje.nl/product/itho-daalderop-amber-control-module/)
The firmware in this repository targets this module, I currently use this as well.

#### Waveshare ESP32-S3 Touch LCD 5'' (https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-5)
This is the best replacement for the WinCE controller as there is a touchscreen, RS485 and 24V input available.
We would also need a 3D printed bracket to mount it properly.

![UI](/docs/images/openamber-waveshare-display-ui.png)

## Installation
**Software**

Download release to flash through web interface(*ota.bin) or from ESP web tools(*factory.bin), or copy src files into ESPHome config directory and flash it through ESPHome builder.
Connect to the OpenAmber access point(password openamber) to setup WiFi.

**Physicial connection**

Remove modbus cables from port 7 and 8 of the WinCE module and connect them to the ESP32 RS485 board.

Remove A and B modbus wires from WinCE Controller

![Remove A and B modbus wires from WinCE Controller](/docs/images/wince-modbus-wires.png)

Connect wires to your hardware RS485 port (Itho Daalderop Amber Control Module used in picture

![Connect wires to your hardware RS485 port (Itho Daalderop Amber Control Module used in picture)](/docs/images/itho-module-install.png)

When using the Waveshare ESP32-S3 Touch LCD 5'' you need to set the RS485 resistor dip switch to OFF state. Connect the modbus to RS485 A and B. For power move the 24V positive and ground of the WinCE to the VIN and GND pins of the display.

## How can I contribute?
- Test and provide feedback/bugs/suggestions
- Sniff modbus traffic for TODO items
- Analyze and document behavior of WinCE logic on TODO functionality.
- Design bracket for the Waveshare ESP32-S3 5'' screen.

## Modbus addresses overview
### Inside unit (Address 11)
#### Read
| Address    | Function                            |
|:----------:|:-----------------------------------:|
| 1199       | Room temperature (Tr)               |
| 1200       | DHW Temperature (Tw)                |
| 1201       | Heat/Cool water temperature (Tc)    |
| 1202       | Outlet water temperature (Tuo)      |
| 1203       | Inlet water temperature (Tui)       |
| 1204       | Outdoor coil temperature (Tp)       |
| 1207       | Outdoor coil temperature (Tv1)      |
| 1212       | Pump P0 active                      |
| 1213       | External cooling signal             |
| 1214       | External heating signal             |
| 1300       | Pump P1 active                      |
| 1301       | Pump P2 active                      |
| 1302       | DHW pump active                     |
| 1305       | Threeway valve state                |
| 1309       | Backup heater active                |
| 1208       | Outdoor coil temperature (Tv2)      |
| 1303       | Internal pump state                 |

#### Write
| Address    | Function                                           |
|:----------:|:--------------------------------------------------:|
| 1099       | Pump P0 relay                                      |
| 1100       | Pump P1 relay                                      |
| 1101       | Pump P2 relay                                      |
| 1103       | Pump P0 duty cycle (1000=OFF, 0=HIGH)              |
| 1104       | Unknown, needed for initialization                 |
| 1105       | 3-way valve DHW                                    |
| 1109       | Backup heater stage 1 relay                        |
| 1110       | Backup heater stage 2 relay                        |
| 1112       | 3-way valve SWW                                    |

### Outside unit (Address 2)
#### Read
| Address    | Function                            |
|:----------:|:-----------------------------------:|
| 2100       | Voltage                             |
| 2101       | Current                             |
| 2102       | Compressor frequency setpoint (Hz)  |
| 2103       | Actual compressor frequency (Hz)    |
| 2104       | Fan RPM setpoint                    |
| 2105       | Fan actual RPM                      |
| 2107       | Expansion valve state               |
| 2108       | State (see below)                   |
| 2110       | Ambient temperature (Ta)            |
| 2111       | Indoor coil temperature (Tup)       |
| 2112       | Discharge temperature (Td)          |
| 2113       | Suction temperature (Ts)            |
| 2116       | Low pressure (Ps)                   |
| 2117       | High pressure (Pd)                  |
| 2118       | Defrost active                      |
| 2119       | Alarm codes (see below)             |
| 2220       | Alarm codes (see below)             |
| 2221       | Alarm codes (see below)             |
| 2122       | Firmware version part 1             |
| 2123       | EEPROM version                      |
| 2131       | Temperature?                        |
| 2132       | Temperature?                        |
| 3509       | Firmware version part 2             |
| 3510       | Firmware version part 3             |

#### Write
| Address    | Function                            |
|:----------:|:-----------------------------------------------------------------:|
| 1999       | Compressor mode(0-10)                                             |
| 3999       | Working mode(0=OFF,1=COOL,2=HEAT,5=EEPROM UPDATE,6=MAINTENANCE)   |

#### State (Address 2108)

| Bitmask    | Function                            |
|:----------:|:-----------------------------------:|
| 0x01       | Fan low speed mode                  |
| 0x02       | Compressor crankcase heater         |
| 0x04       | Bottom heater active                |
| 0x10       | Fan defrost speed mode              |
| 0x20       | Fan high speed mode                 |
| 0x40       | 4-way valve switching               |
| 0x80       | Expansion valve lock active         |

#### Alarm codes (Address 2119)

| Bitmask    | Function |
|:----------:|:--------:|
| 0x01       | P01      |
| 0x02       | P02      |
| 0x04       | P03      |
| 0x08       | P04      |
| 0x10       | P05      |
| 0x20       | P06      |
| 0x40       | P07      |
| 0x80       | P08      |
| 0x100      | P09      |
| 0x200      | P10      |
| 0x400      | P11      |
| 0x800      | P12      |
| 0x1000     | P13      |

#### Alarm codes (Address 2220)

| Bitmask    | Function |
|:----------:|:--------:|
| 0x01       | F01      |
| 0x02       | F02      |
| 0x04       | F03      |
| 0x08       | F04      |
| 0x10       | F05      |
| 0x20       | F06      |
| 0x40       | F07      |
| 0x80       | F08      |
| 0x100      | F09      |

#### Alarm codes (Address 2221)

| Bitmask    | Function |
|:----------:|:--------:|
| 0x01       | E01      |
| 0x02       | E02      |
| 0x04       | E03      |
| 0x08       | E04      |
| 0x10       | E05      |
| 0x20       | E06      |
| 0x40       | E07      |
| 0x80       | E08      |
| 0x100      | F10      |
| 0x200      | F16      |
| 0x400      | F17      |
| 0x800      | F18      |
