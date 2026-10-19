# EEBUS Component for ESPHome

A generic, loosely coupled native EEBUS component for ESPHome enabling bidirectional communication with energy management systems (EMS/CEM) such as [evcc](https://evcc.io), SMA, or grid operators (§14a EnWG).

## Features

- **Native On-Device EEBUS Stack**: Runs directly on ESP32 (including ESP32-S3 with PSRAM).
- **SHIP (Smart Grid High Level Protocol)**: mDNS service advertisement (`_ship._tcp`), TLS (mbedTLS) server, WebSocket framing, pairing, and certificate/SKI management with NVS persistence.
- **SPINE (Smart Premises Interoperable Neutral Message Exchange)**: Datagram framing, node management, device classification, and feature routing.
- **Supported Use Cases**:
  - **OHPCF (Optimization of Self-Consumption by Heat Pump Compressor Flexibility)**: Allows CEM to request/schedule boost or optional consumption.
  - **LPC (Load Power Control / §14a EnWG)**: Remote power limitation and dimming/blocking.
  - **MPC (Measurement of Power Consumption)**: Live power, energy, voltage, and current measurement reports.
  - **MDT (Measurement of Domestic Hot Water Temperature)**: Storage temperature reporting.
- **Decoupled Mock Mode**: Headless simulation and test-bridge hooks are fully decoupled into separate files (`eebus_mock.cpp`, `eebus_mock.h`) and filtered out on hardware targets via ESPHome's native `FILTER_SOURCE_FILES()`. Real hardware builds compile zero mock code or conditional compilation guards.

## Installation

Include the component in your ESPHome configuration via `external_components`:

```yaml
external_components:
  - source:
      type: local
      path: components # Directory containing the 'eebus' folder
    components: [eebus]
```

## Configuration Reference

```yaml
eebus:
  # Device Identity
  device_sku: "MyBrand-ESP32"          # SKU reported in mDNS / SPINE (default: EEBUS-Device)
  brand: "MyBrand"                    # Brand name (default: Generic)
  model: "HeatPump"                   # Model name (default: Device)
  device_name: "LivingRoom-HeatPump"  # Optional human-friendly name (default: brand)
  device_category: "heatPump"         # SPINE device category (default: heatPump)
  device_type: "HeatGenerationSystem" # SPINE device type (e.g. HeatGenerationSystem, ChargingStation, BatterySystem, ControllableSystem)
  entity_type: "HeatPumpAppliance"    # SPINE entity type, case-sensitive (e.g. HeatPumpAppliance, EVSE, Battery, ControllableSystem)
  enable_ohpcf: true                  # Enable OHPCF (heat pump boost/compressor) use-case (default: auto-detected)
  enable_mdt: true                    # Enable MDT (DHW hot water temperature) use-case (default: auto-detected)
  failsafe_duration: 2h               # Auto-release dim/block if connection lost (default: 2h)

  # Host State & Telemetry Callbacks (Lambdas)
  is_enabled: "return id(eebus_enable_switch).state;"
  read_model: "return id(detected_model_sensor).state;" # Dynamic model detection callback
  read_power: "return id(active_power_sensor).state;"
  read_dhw_temp: "return id(dhw_temp_sensor).state;"
  read_voltage: "return id(voltage_sensor).state;"
  read_current: "return id(current_sensor).state;"
  read_energy: "return id(total_energy_sensor).state;" # Total energy in Wh
  read_power_estimate: "return 2500.0f;"               # Estimated power during boost (W)
  is_boost_active: "return id(boost_active_sensor).state;"

  # Host Control Callbacks (Action Execution)
  apply_optional: |-
    // Called when the EMS requests boost (value=true) or cancels boost (value=false).
    // Return true if accepted by host safety logic, false if rejected.
    if (value) {
      id(sg_ready_boost_switch).turn_on();
    } else {
      id(sg_ready_boost_switch).turn_off();
    }
    return true;

  apply_dim: |-
    // Called when the EMS requests dimming/blocking (§14a 0W limit).
    if (value) {
      id(compressor_lockout_switch).turn_on();
    } else {
      id(compressor_lockout_switch).turn_off();
    }
    return true;

  apply_limit: |-
    // Called when the EMS sets a power consumption limit (in Watts).
    // active: whether limit is in effect, limit_w: maximum allowed power.
    // If continuous/multi-stage power limiting is not implemented on the host, return false.
    return false;

  # Diagnostic Entities (Exposed to Home Assistant / UI)
  status:
    name: "EEBUS Status"
    id: eebus_status_sensor
  last_action:
    name: "EEBUS Last Action"
  connected:
    name: "EEBUS Connected"
    id: eebus_connected_sensor
  power_limit:
    name: "EEBUS Power Limit"
    id: eebus_power_limit_sensor
  ski:
    name: "EEBUS SKI"
    id: eebus_ski_sensor
  ems:
    name: "EEBUS EMS"
    id: eebus_ems_sensor

  update_interval: 5s
```

## How It Works

1. **First Boot & TLS Pairing**:
   On first boot, the component dynamically generates an EC P-256 keypair and self-signed X.509 certificate. The Subject Key Identifier (SKI) is stored in ESP32 NVS flash.
2. **Discovery**:
   The component advertises itself on mDNS as `_ship._tcp` with the device SKI and identity parameters.
3. **Connection**:
   An EMS (e.g. evcc) connects via TLS to port 4712. Pairing is verified using the SKI.
4. **Operation**:
   - The EMS periodically receives power and temperature measurements via MPC / MDT notifications.
   - When surplus solar power is available, the EMS schedules an OHPCF boost cycle.
   - During grid stress events (§14a EnWG), the EMS can issue an LPC power limit or dimming command.
