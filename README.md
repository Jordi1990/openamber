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
- Heating
- Pump running on customizable interval/duration/speed.
- Compressor modulation based on PID controller.
- Frost protection in 2 stages (similair to original software).
- Software thermostat based on Tr sensor.
- Heat curve(5-points).
- Backup heater
- Support external thermostats (Heating/Cooling input)

## Todo
- Support cooling
- Domestic Hot Water/Three way valve control
- Tv1/Tv2 buffer support
- Extend configuration options
  - Enable specific compressor frequencies (silent mode)
- Error handling
- Support OpenTherm thermostats
- .. More

## Hardware
The firmware can be used on any ESP32 with an RS485 board.

### Off-the-shelf boards:
#### Itho Daalderop Amber Control Module (https://electropaultje.nl/product/itho-daalderop-amber-control-module/)
The firmware in this repository targets this module, I currently use this as well.

#### Waveshare ESP32-S3 Touch LCD 5'' (https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-5)
This is the best replacement for the WinCE controller as there is a touchscreen, RS485 and 24V input available.
But I haven't been able to get the RS485 working on my unit so far.
We would also need a 3D printed bracket to mount it properly.

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
| 1103       | Pump P0 state (1000=OFF, 500=LOW, 250=MED, 0=HIGH) |
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
| 2122       | Firmware version part 1             |
| 2123       | EEPROM version                      |
| 2131       | Temperature?                        |
| 2132       | Temperature?                        |
| 3509       | Firmware version part 2             |
| 3510       | Firmware version part 3             |

#### Write
| Address    | Function                            |
|:----------:|:-----------------------------------:|
| 1999       | Compressor mode(0-10)               |
| 3999       | Working mode(0=OFF,1=COOL,3=HEAT)   |

#### State (Address 2108)
Not all bits are known, so the ones we know of are listed here.

| Bitmask    | Function                            |
|:----------:|:-----------------------------------:|
| 0x01       | Fan low speed mode                  |
| 0x04       | Bottom heater active                |
| 0x10       | Fan defrost speed mode              |
| 0x20       | Fan high speed mode                 |
| 0x40       | 4-way valve switching               |
| 0x80       | Expansion valve lock active         |

#### Alarm codes (Address 2119)
Not all bits are known, so the ones we know of are listed here.

| Bitmask    | Function                            |
|:----------:|:-----------------------------------:|
| 0x04       | Oil return cycle (P04)              |
