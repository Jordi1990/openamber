# Inside unit firmware and HeatStar — reverse-engineering reference

Reference for people and AI agents working on OpenAmber. It collects what was learned from:

- a flash dump of the **inside unit controller** (STM32F103, 64 KB dump, firmware ~26 KB),
- a RAM/EEPROM dump of the same board,
- the original **HeatStar** WinCE controller software (`HeatStar.exe`, ARM, MFC/C++) and its SQLite database `HeatStar.db` (taken from an Itho Daalderop Amber),
- field tests with OpenAmber on a real Amber (Oct 2026).

Everything below comes from static analysis unless marked **[field-tested]**. Items marked **[unverified]** are inferences that still need confirmation on hardware.

## Conventions

- **All register numbers in this document are 0-based Modbus PDU addresses**, exactly as used in the OpenAmber YAML (`address:`). Tools that show 1-based "holding register" numbers display them +1.
- Inside unit = `modbus_inside_unit` (slave 11 by default). Outdoor unit = `modbus_outside_unit` (slave 2).
- Bus: RS485, 19200 baud, 8E1 (default; configurable on the inside unit, see EEPROM).
- "Bit 0x0008" means `value & 0x0008`.

## Do NOT write these inside-unit registers

| Register | Effect |
|---|---|
| 999 = 255 | Test mode: cycles all relays and analog outputs every ~10 s. |
| 1404 = 1 | Starts automatic sensor calibration against 25.0 °C (expects reference resistors). Overwrites the factory calibration and saves it to EEPROM. |
| 1799 (any value) | Saves the RAM image of the settings to EEPROM (value 1 = recompute CRC first). |
| 3999–4002 | Alternative unlock/save sequence (active when 999 == 5). |
| 1399–1499, 1899–1931, 2999–3510 (writes) | Change the RAM copy of the EEPROM settings; persisted on the next save. Reading is safe. |

## Inside unit hardware summary

- MCU: STM32F103V (100-pin, high-density; likely VC/VD/VE or a GD32/APM32 clone). Clock: HSE × 8 (64 MHz with an 8 MHz crystal). ST HAL, Keil.
- Firmware is complete in the first 26 KB of flash; no bootloader; no flash-stored settings.
- Settings live in an external 24C16 I²C EEPROM (bit-banged, SCL = PB8, SDA = PB9). Only the first 512 bytes are used.
- Two identical Modbus RTU slaves: USART1 (RS485 DE = PA11) and USART3 (DE = PB12). Same register map on both. **The unit also always answers slave address 10.**
- Function codes: 03, 06, 16 only.
- Independent watchdog: ~26 s.
- Main loop: 4 ms tick (TIM6 at 20 kHz / 80), 5 round-robin slots → each task every 20 ms; status logic every ~1 s.
- Temperatures:
  - 4 precision channels via a Chipsea **CS1237** 24-bit ADC (single-wire on PA2), lookup table −30…+100 °C in 0.5 °C steps, factory-calibrated per unit (see EEPROM bytes 6–9).
  - 8 NTC channels via the internal ADC, lookup table −30…+100 °C in 1 °C steps (≈10 kΩ NTC), 10-sample trimmed mean.
- Slave address = 11 + 4·PE8 + 2·PE10 + PE9 (DIP switches; inputs are active-low).

## Inside unit register map (read)

Readable ranges (requests outside these are rejected): 999–1009, 1099–1119, 1199–1229, 1299–1319, 1399–1499, 1799–1801, 1899–1931, 2099–2109, 2999–3511, 3999–4009, 4999–5148 (bytes), 5149–5298 (words), 5299+ (raw floats).

Aliases: 2099–2109 and 3999–4009 mirror 999–1009. 1399–1499, 1899–1931 and 2999–3510 mirror the EEPROM image (see below).

### Status (999–1009)

| Reg | Meaning |
|---|---|
| 999 | Operating mode (0 normal; 255 = test mode; 5 = unlock state for the 3999–4002 save sequence). |
| 1003 | Status code: 4 = settings saved OK, 5 = EEPROM CRC error at boot (defaults in use), 6 = EEPROM write error, 7 = CRC invalid on save. |
| 1004–1008 | General purpose, written via 4004–4008. |

### Outputs (1099–1113, read/write)

Relay outputs use bit 0 of the register.

| Reg | Output | MCU pin | Notes |
|---|---|---|---|
| 1099 | Pump P0 relay | PD9 | OpenAmber: `pump_p0_relay_switch` |
| 1100 | Pump P1 relay | PD10 | |
| 1101 | Pump P2 relay | PD11 | |
| 1102 | DHW pump relay | PD12 | |
| 1103 | Pump P0 PWM | TIM3 CH1 (PA6) | 0–1000 = duty in 0.1 %, 1 kHz. Values > 900 are forced to 1000 (with the PWM1 pump profile: pump off/standby). **Only active if input 1215 was active at boot**, otherwise forced to 0 every cycle. |
| 1104 | Relay PD13 | PD13 | Probably the "HO" relay (HeatStar: DHW circulation pump or "on whenever powered"). OpenAmber turns it on at init. **[unverified]** |
| 1105 | 3-way valve heat/cool | PD5 | |
| 1106 | Analog out zone 1 (mixing valve 1) | TIM3 CH2 (PA7) | 0–100 %, closed loop against ADC readback (1222). Values > 100 clamped. |
| 1107 | Analog out zone 2 (mixing valve 2) | TIM3 CH3 (PB0) | Same, readback 1223. |
| 1108 | Relay PD15 | PD15 | OpenAmber: external backup heating relay |
| 1109 | Backup heater stage 1 | PD14 | |
| 1110 | Backup heater stage 2 | PD4 | |
| 1111 | Relay PD6 | PD6 | **Forced ON in failsafe.** Possibly relay "RE2" (mode signal / DHW circulation). **[unverified]** |
| 1112 | 3-way valve DHW | PD7 | |
| 1113 | Relay PD8 | PD8 | Unknown function. |

### Temperatures and inputs (1199–1228)

Temperatures: `°C = (raw − 3000) / 100`. A broken (open/short) sensor reads **exactly raw 3000 (0.00 °C)** and sets a fault bit in 1225. Values are clamped to −30/+100 °C without a fault bit.

| Reg | Meaning | Source |
|---|---|---|
| 1199 | Tr room temperature | NTC PC0 |
| 1200 | Tw DHW temperature | CS1237 ch0 (calibrated, EEPROM byte 6) |
| 1201 | Tc system water temperature | CS1237 ch1 (byte 7) |
| 1202 | Tuo outlet water | CS1237 ch2 (byte 8) |
| 1203 | Tui inlet water | CS1237 ch3 (byte 9) |
| 1204 | Tup | NTC PC5 |
| 1205 | spare NTC | PA3 |
| 1206 | spare NTC | PA5 |
| 1207 | Tv1 mixing circuit 1 | NTC PC1 |
| 1208 | Tv2 mixing circuit 2 | NTC PC2 |
| 1209 | spare NTC | PC3 |
| 1210 | spare NTC | PC4 |
| 1211 | Input PE2 — SG-Ready A | 1 = active (pin low) |
| 1212 | Input PE7 — P0 running / flow switch | |
| 1213 | Input PE3 — external cooling demand | |
| 1214 | Input PE4 — external heating demand (zone 1) | |
| 1215 | Input PE11 — **configuration**: enables the P0 PWM output; read once at boot | HeatStar does not use it for control |
| 1216–1218 | DIP switches PE8, PE10, PE9 (slave address) | |
| 1219 | Input PE5 — **external heating demand zone 2** | Used as zone-2 demand by HeatStar |
| 1220 | Bit field: bit0 PE6 = SG-Ready B, bit1 PE14, bit2 PE15 | Use `bitmask: 0x1` for SG-B |
| 1221 | P0 PWM feedback duty, 0.1 % | Input PD3 ("FB" terminal). See "Pump feedback". |
| 1222 | Analog out 1 readback (≈0–112) | ADC PA0 |
| 1223 | Analog out 2 readback | ADC PA1 |
| 1225 | Status/fault bits (see below) | |
| 1226 | Bits 0/1: long-duration alarms (>180 s); meaning unknown | |
| 1227 | **Firmware version constant 0x0106 (V1.06)**, written at every boot | |

#### Register 1225 bits

| Bit | Meaning | Itho code |
|---|---|---|
| 0x0001 | Tr sensor fault (1199) | F13 |
| 0x0002 | Tw sensor fault (1200) | F14 |
| 0x0004 | Tc sensor fault (1201) | F15 |
| 0x0008 | Tuo sensor fault (1202) | F16 |
| 0x0010 | Tui sensor fault (1203) | F17 |
| 0x0020 | Tup sensor fault (1204) | F18 |
| 0x0040 | sensor 1205 fault | |
| 0x0080 | sensor 1206 fault | |
| 0x0100 | Tv1 sensor fault (1207) | F21 |
| 0x0200 | Tv2 sensor fault (1208) | F22 |
| 0x0400 | sensor 1209 fault | |
| 0x0800 | sensor 1210 fault | |
| 0x1000 | Communication failsafe active (see Failsafe) | |
| 0x4000 | Status register 1003 > 4 (EEPROM problem) | |
| 0x8000 | P0 PWM feedback alarm (feedback ≤ 0.2 % or > 80 % while 1103 ≤ 900) | |

The F-codes follow the order of the Itho texts in `HeatStar.db`; F14 is confirmed by HeatStar's error history (F14 logged with Tw = −99).

### Output readback and flow (1299–1314)

| Reg | Meaning |
|---|---|
| 1299–1302 | Actual state of PD9, PD10, PD11, PD12 (1099–1102) |
| 1303 | Copy of 1103 (PWM setpoint) |
| 1304 | PD13 (1104) |
| 1305 | PD5 (1105) |
| 1306, 1307 | Copies of 1106, 1107 |
| 1308–1313 | PD15, PD14, PD4, PD6, PD7, PD8 (1108–1113) |
| 1314 | Flow from the inside unit's **pulse flow-sensor input** (TIM1 CH3), **× 0.1 L/min** (scale confirmed by HeatStar). Internally ≈ (0.73·f ± 0.2)·10 with f in Hz, clamped at 206 Hz. **Reads 0 on the Amber — no sensor connected.** [field-tested] |

### EEPROM image (1399–1499, 1899–1931, 2999–3510)

One register per EEPROM byte (low byte). Register 2999 + n = EEPROM byte n; 1399 + n = byte n (n ≤ 100).

| Byte | Reg | Meaning |
|---|---|---|
| 0 | 2999 / 1399 | Unused by firmware (255 on the analysed unit) |
| 1 | 3000 / 1400 | Baud rate: 0 = 1200, 1 = 2400, 2 = 4800, 3 = 9600, anything else = 19200 |
| 2 | 3001 / 1401 | Parity: 0 = none (8N1), 2 = odd (probably), anything else = even (8E1) |
| 5 | 3004 / 1404 | Calibration state: 1 = run auto-calibration (do not write), 2 = done |
| 6–9 | 3005–3008 / 1405–1408 | Calibration of Tw, Tc, Tuo, Tui: `T_corrected = T − (value − 100) / 10` (100 = no correction). Analysed unit: 104, 105, 104, 102. |
| 510–511 | 3509–3510 | CRC16 (Modbus) over bytes 0–509, low byte first |

There is no serial number or model number in the inside-unit EEPROM or firmware. The STM32 96-bit unique ID (0x1FFFF7E8) is not exposed over Modbus.

### Failsafe

If no valid Modbus request addressed to the unit arrives for ~120 s (counter on the USART3 port), the unit sets 1225 bit 0x1000 and forces:

- all relays off **except 1111 (PD6), which is switched on**,
- 1103 = 950 (P0 off), 1106 = 1107 = 0,
- 999 = 0 and **registers 1099–1113 cleared to 0**.

When traffic returns, outputs follow the registers again — which are now 0. A master must therefore re-write its outputs. HeatStar does this by reading 1099–1118 every cycle and re-writing any output that differs from its desired value. A reboot of the inside unit (watchdog, power dip) has the same effect.

## Pump P0 feedback (1221) [field-tested]

- P0 on the Amber is a **Shimge APM25-9-130 PWM1** (sold as replacement for Wilo Para 25/9 iPWM-130; 4–95 W, max 9 m, 75 L/min).
- Wilo Para iPWM feedback protocol: 75 Hz; 0 % interface fault, 2 % standby, 5–75 % running (power or flow info), 80 % warning, 85–90 % error, 95 % permanent failure, 100 % no power. Input PWM1: < 5 % max speed, 5–85 % linear max→min, > 93 % stop.
- Pump signal cable colours (Wilo convention): brown = PWM input, blue = signal ground, **black = PWM output (feedback)**.
- The inside unit measures 1221 on input PD3 (interrupt on both edges, 50 µs ticks, mean of 75 high and 75 low periods). The board has a terminal **"24+ / GND / FB"**; FB is very likely this input.
- **On the analysed Amber the FB terminal is not connected.** 1221 then reads interference: ~50 % ± 6 % while running, a small shift at 100 % speed, and erratic 0–85 % when the pump relay is off. It is **not** usable for flow estimation as installed.
- Measured flow (external pulse meter) vs pump PWM on the CH circuit: 70 % → 14.9 L/min, 80 % → 18.4 L/min, 100 % → 23.0 L/min (Tuo ≈ 28.7 °C). 1221 averaged 49.6 / 49.8 / 54.6 % in the same runs.
- To use the feedback, wire the pump's black wire to FB and blue to GND (never to 24+), e.g. over a spare UTP pair, or measure it directly with the ESP32 (opto-coupler / divider to 3.3 V).

## Inside-unit wiring notes (Amber)

- Mains terminal block towards the outdoor unit: **BL, N, EL, P0, PE** — 230 V (P0 = switched phase for pump P0). Not signal lines.
- Relay terminals on the inside board: **CY, WA, CO, HO, HE** (230 V switched outputs). Mapping to registers is not yet confirmed; HO is the HeatStar "HO relay".
- Low-voltage links to the outdoor unit: two UTP cables ("sensor" and "signal", the latter carries Modbus).

## HeatStar (original WinCE controller)

### Device objects

HeatStar creates one object per bus device type: 0x65 inside unit (slave 11–19, default 11), 0x66 outdoor unit (slave 1–8, default 2), 0x64 another inside-board variant, 0x67 unknown device using coils/input registers, 0x69 WiFi module (AT commands), 0x6a external Modbus slave port (address 1–254, up to 115200 baud), 0x76 simulator. `t_Machine` in `HeatStar.db` lists two heat pumps (slave 1 and 2) next to the inside unit (slave 11): HeatStar supports a cascade.

### Inside-unit polling (≈ every 300 ms)

| Request | Purpose |
|---|---|
| FC3 999 ×1 | mode |
| FC3 1099 ×20 | output readback buffer (basis for re-writing) |
| FC3 1199 ×30 | temperatures, inputs, 1221–1228 |
| FC3 1299 ×20 | output states and flow (1314 × 0.1 L/min) |
| FC6 999, 1099–1113 | write an output only when the read-back value differs from the desired value |
| FC16 1099 ×19 (at init) | all outputs 0, 1103 = 1000 (pump off) |
| 10999+, 11099+, 11199+ | extended blocks for another board generation; this firmware rejects them and HeatStar skips them |

Control behaviour worth copying:

- Sensor fault bits from 1225 → value shown as −99 °C and a fault flag.
- Zone-2 heat demand = **1219**, used only when zone 2 is enabled in the settings; 1215 is not used in control.
- 1213/1214 (cooling/heating demand) are debounced and interlocked.
- 1220 is treated as a bit field.

### Flow sources (parameter 385)

- 0 = inside unit register 1314 × 0.1 L/min.
- 1 = from P0 PWM feedback: feedback clamped to 5–70 % (outside → 0); the formula is most likely `Q[L/min] = (0.09·Tuo + (16/15)·duty%) / 4.88`, with a second coefficient set (0.05 and 1.69) for another pump type. The exact arithmetic of the floating-point helpers is **[unverified]**; X = Tuo is confirmed.
- 2 = outdoor unit.
- If the resulting flow is ≈ 0, HeatStar adds outdoor EEPROM parameter 3494 ("water flow correction value") / 60, i.e. a per-model nominal flow in L/h.

Parameter 341 selects whether Tuo/Tui/Tup, pump status and flow switch come from the inside unit (0) or the outdoor unit (1–5). The analysed Amber uses 0 for both 341 and 385.

### Outdoor-unit traffic

- Reads: 1999 ×20, 2099 ×40 (or ×30), 2199 ×100 (only for certain EEPROM versions), 2999–3510 (EEPROM, at startup, in chunks of 100), 3999, 9999 ×5 (probably version/identification, not decoded), 11004 ×6, 11099 ×81, 11219 ×20 (newer models).
- Writes (FC6) that OpenAmber already does: 1999, 2001, 2005, 2008, 2010, 3000–3021, 3999.
- Writes that OpenAmber does **not** do: 2000, 2006, 2007, 2011, 3028 (EEV control mode), 3248 (fan control mode), 3274 (defrost control mode), 3281 (ambient frequency-limit control mode), 3493 = 1 ("inlet/outlet water temperature and indoor coil temperature enable" — the outdoor unit then uses temperatures sent by the master), 4004.
- 2005 and 2011 are written as `T·100 + 3000`; 2005 is the outlet temperature, 2011 most likely the inlet temperature **[unverified]**.
- HeatStar combines outdoor EEPROM 3510·1000 + 3509 (checksum bytes) as the "EEPROM version" — the same value OpenAmber shows.

### HeatStar.db

- `t_System0` holds ~990 parameters (current values), `t_System1`/`t_System2` factory/installer values; each row has a Chinese description with range.
- `t_Language`: 50 languages; Itho Amber uses column `language39` (Dutch).
- `t_HPErr`: error history with a snapshot (mode, compressor frequency, Tr, Tw, Th, Ta, To, Ti, Tup, Td, Ts, Tp, Tv1, Tv2, pressures, current, voltage, EEV opening, fan speeds, PWM).
- Useful parameters: 335 HO relay type (0 = DHW circulation pump, 1 = on when powered), 748 RE2 relay type (0 = DHW circulation pump, 1 = mode signal), 385 flow source, 341 outdoor sensor use, 747 inlet/outlet ΔT protection, 336–338 screed drying profile 2 (30 °C start, 55 °C max, 24 h at max).
- Itho's Dutch text for the "electrical utility lock" input is "CV-ketel aansturing" (boiler release).
- `t_User` and `t_WifiDev` contain passwords — do not commit the database.

### Itho error texts (order in `t_Language`)

| Code | Text (EN) |
|---|---|
| P01 | Main line current protection |
| P02 | Compressor phase current protection |
| P03 | IPM module protection |
| P04 | Compressor oil return protection |
| P05 | Compressor shut down due to abnormal high/low pressure |
| P06 | Compressor speed down due to abnormal high pressure |
| P07 | Compressor preheating |
| P08 | Compressor discharge temperature too high |
| P09 | Outdoor evaporator coil temperature protection |
| P10 | AC over high/low voltage protection |
| P11 | Compressor shut down due to too high/low outdoor temperature |
| F01–F07 | Ta, Tp, Td, Ts, Ps, Pd sensor failures; pressure switch failure |
| F08 | Water flow switch failure (monoblock) |
| F09, F10 | DC fan A / B failure |
| F11, F12 | Evaporating / condensing pressure failure |
| E01–E08 | Master/slave communication; outdoor PCB–module communication; compressor phase current failure; compressor overcurrent; compressor driver failure (E05); VDC module voltage; AC current; outdoor EEPROM |
| F13–F22 | Tr, Tw, Tc, Tuo, Tui, Tup, source-side sensors, Tv1, Tv2 (see register 1225) |
| E05-1 … E05-37 | Inverter sub-codes, from "IPM over current" to "PFC overheat" |

OpenAmber's names for P03, P06, P08 and P11 differ from these texts and should be checked against Itho documentation.

## Open questions

- Which relay terminals (CY, WA, CO, HO, HE) belong to 1104, 1111 and 1113.
- Whether Amber installations with a second outdoor unit (slave 1) exist.
- Meaning of outdoor registers 2000, 2006, 2007, 2011, 4004 and 9999–10003.
- Real P0 feedback signal once the FB terminal is wired (power vs flow encoding, quality of the inside unit's measurement).
- Which USART the failsafe counter watches in practice (assumed to be the port OpenAmber uses, since the failsafe does not trigger during normal operation).
