# OpenAmber Automated Testing Framework

This test suite provides end-to-end automated testing for OpenAmber, combining real LVGL UI interactions (button clicks, navigation, widget visibility, labels) with ESPHome sensor/switch/number/select simulation, virtual time progression, and workflow verification (such as external thermostat heat demand, PID controllers, and safety shutoff flows).

---

## Architecture Overview

```
┌────────────────────────────────────────────────────────┐
│                   Pytest Test Runner                   │
│   (tests/test_external_thermostat.py, test_pid, …)     │
└──────────────────────────┬─────────────────────────────┘
                           │ JSON over TCP (Port 8888)
                           ▼
┌────────────────────────────────────────────────────────┐
│         OpenAmber Host Simulator (ESPHome SDL)        │
│                                                        │
│  ┌──────────────────────────────────────────────────┐  │
│  │     test_bridge External Component (C++)         │  │
│  │  - Dispatches LVGL events (clicks, gestures)    │  │
│  │  - Inspects LVGL widget tree (text, hidden, etc)│  │
│  │  - Injects entity states (sensor, number, etc.)  │  │
│  │  - Warps virtual time (millis / micros wraps)    │  │
│  │  - Queries entity states                         │  │
│  └──────────────────────────────────────────────────┘  │
│  ┌─────────────────────────┐  ┌─────────────────────┐  │
│  │      LVGL UI Engine     │  │  ESPHome Component  │  │
│  │  (Screens, Widgets)     │  │  (Automation/Logic) │  │
│  └─────────────────────────┘  └─────────────────────┘  │
└────────────────────────────────────────────────────────┘
```

1. **`test_bridge` (ESPHome Component in `src/openamber/test_bridge/`)**:
   - A non-blocking TCP server running on port 8888 inside the ESPHome host application.
   - Automatically registers all ~1,400 declared LVGL widgets (`lv_obj_t*`) alongside all ESPHome entities (`sensor`, `number`, `switch`, `binary_sensor`, `select`, `climate`).
   - Dispatches real LVGL events (e.g. `LV_EVENT_CLICKED`) via `lv_obj_send_event(obj, ...)`.
   - Returns widget metadata (`visible`, `hidden`, `text`, `checked`, `disabled`).
   - Injects states (`publish_state()`, `turn_on()`, `turn_off()`, `set_option()`, `make_call()`).
   - **Virtual Time Engine**: Uses GNU ld symbol wrapping (`-Wl,--wrap=_ZN7esphome6millisEv`, `_ZN7esphome9millis_64Ev`, `_ZN7esphome6microsEv`) to advance simulation time instantaneously without sleeping, allowing PID integral windup, delayed automations, and settle timers to be tested in milliseconds.

2. **`OpenAmberClient` (`tests/openamber_client.py`)**:
   - High-level Python client library providing synchronous methods to control and inspect the application.

3. **Pytest Harness (`tests/conftest.py`)**:
   - Automatically starts the OpenAmber host binary in headless mode (`SDL_VIDEODRIVER=dummy`) and manages lifecycle during tests.
   - Reuses existing graphical sessions if the OpenAmber virtual display window is already open.

---

## Test Directory Structure

The tests are organized into modular, domain-specific test suites:

| File | Scope & Tested Functionality |
| :--- | :--- |
| [`test_external_thermostat.py`](file:///g:/openamber/tests/test_external_thermostat.py) | External thermostat happy flow (wired contact `external_heat_demand_wired` $\rightarrow$ `heat_demand_active_sensor` $\rightarrow$ UI flame icon), manual heat switch flow, and `Intern` vs `Extern` mode switching. |
| [`test_heat_demand_flow.py`](file:///g:/openamber/tests/test_heat_demand_flow.py) | Heat demand expectation to heat, SG Ready block mode suppression, system fault (`error_active`) suppression, stop delta cutoffs, and post-compressor pump cycle timing. |
| [`test_frost_protection_flow.py`](file:///g:/openamber/tests/test_frost_protection_flow.py) | Frost protection stage 1 & 2 detection, instant pump interval reset (`reset_pump_interval`), and immediate pump start in IDLE without waiting for interval. |
| [`test_defrost_flow.py`](file:///g:/openamber/tests/test_defrost_flow.py) | Defrost cycle entry & exit, secondary pump P1 stop/start, pump P0 defrost PWM speed (`pump_p0_pid_defrost_pwm`), post-defrost settle period (`COMPRESSOR_SETTLE_TIME_AFTER_DEFROST_S = 300s`), compressor recovery boost mode (+3 steps), low ambient backup heater boost ($T_a \le -3^\circ\text{C}$), and DHW defrost flow. |
| [`test_compressor_limits_flow.py`](file:///g:/openamber/tests/test_compressor_limits_flow.py) | Compressor mode limiting settings across DHW base modes (`dhw_compressor_mode`: Beperkt..Maximaal $\rightarrow$ modes 4..10), DHW winter modes (`dhw_compressor_mode_max`: Gemiddeld..Maximaal $\rightarrow$ modes 7..10 when $T_a \le$ threshold), dynamic ambient temperature threshold crossings and adjustments, Space Heating compressor mode limits (`heat_compressor_mode`: 4..10) & soft-start capping, and Space Cooling compressor mode limits (`cool_compressor_mode`: 1..7). |
| [`test_virtual_time_settle.py`](file:///g:/openamber/tests/test_virtual_time_settle.py) | Virtual time engine (`advance_time`, `reset_time`), timer expiration, and backup heater 5-minute prediction settle time (`BACKUP_HEATER_PREDICTION_SETTLE_TIME_S = 300s`). |
| [`test_pid_control.py`](file:///g:/openamber/tests/test_pid_control.py) | PID climate controller setpoint queries, error handling, virtual time integration ($K_i \cdot \int e dt$), and deadband stabilization. |
| [`test_mode_switching_flows.py`](file:///g:/openamber/tests/test_mode_switching_flows.py) | Mode transitions between DHW, HEAT, and COOL modes: HEAT $\rightarrow$ DHW $\rightarrow$ HEAT, COOL $\rightarrow$ DHW $\rightarrow$ COOL, HEAT $\rightarrow$ COOL, COOL $\rightarrow$ HEAT, IDLE $\rightarrow$ DHW $\rightarrow$ IDLE, and simultaneous demand priority conflict resolution. |
| [`test_dhw_flows.py`](file:///g:/openamber/tests/test_dhw_flows.py) | Domestic Hot Water demand, 3-way valve control (`three_way_valve_dhw_switch`), target temperature shutoff, backup heater failure escalation, direct vs $\Delta T$ pump start modes, and compressor limit modes. |
| [`test_dhw_flow.py`](file:///g:/openamber/tests/test_dhw_flow.py) | Domestic Hot Water demand, navbar tapwater icon (`nav_status_dhw_icon`), and circulation pump spinner animation (`dhw_circulation_spinner`, `dhw_pump_state_label`). |
| [`test_cooling_flow.py`](file:///g:/openamber/tests/test_cooling_flow.py) | External cooling demand, navbar snowflake icon (`nav_status_cool_icon`), and heating-over-cooling priority conflict resolution. |
| [`test_backup_heater_flow.py`](file:///g:/openamber/tests/test_backup_heater_flow.py) | Backup heater relay activation, UI badge updates (`tile_backup_state`), temperature reaching setpoint + shutoff delta, and shutdown propagation. |
| [`test_settings_workflow.py`](file:///g:/openamber/tests/test_settings_workflow.py) | Settings sidebar tabs (Algemeen, Verwarmen, Tapwater, Bijverwarmen) and mixing valve pager navigation (`1 / 2` $\leftrightarrow$ `2 / 2`). |
| [`test_ui_navigation.py`](file:///g:/openamber/tests/test_ui_navigation.py) | Header bar title and navbar navigation between Home, Settings, Service, and System pages. |

---

## Running the Tests

### Quick Run (All Tests)
From the repository root:
```powershell
python run_tests.py
```
Or with pytest directly:
```powershell
pytest -v tests/
```

### Running Specific Test Suites
```powershell
pytest -v tests/test_external_thermostat.py
pytest -v tests/test_virtual_time_settle.py
pytest -v tests/test_pid_control.py
```

---

## Test Client API

The `openamber` fixture in tests provides:

### Virtual Time
- `openamber.advance_time(seconds=0, ms=0)`: Instantly advances simulation time by the specified duration and allows loop cycles to execute.
- `openamber.reset_time()`: Resets the virtual time offset back to 0.
- `openamber.get_time()`: Returns `{"millis": int, "offset_ms": int}`.

### UI Widget Interactions
- `openamber.click(widget_id)`: Dispatches `LV_EVENT_CLICKED` to the target widget.
- `openamber.is_visible(widget_id)`: Returns `True` if the widget does not have `LV_OBJ_FLAG_HIDDEN`.
- `openamber.is_hidden(widget_id)`: Returns `True` if hidden.
- `openamber.get_label(widget_id)`: Retrieves text from a label widget (or its child label).
- `openamber.get_widget(widget_id)`: Returns dict with `{"visible": bool, "hidden": bool, "checked": bool, "disabled": bool, "text": str}`.

### Entity State Manipulation & Inspection
- `openamber.set_select(select_id, option)`: Sets the active option of a select entity (e.g. `thermostat_mode_select`).
- `openamber.set_climate(climate_id, target_temperature=..., mode=...)`: Modifies climate target temperature or mode.
- `openamber.set_sensor(sensor_id, value)`: Injects a float state to a sensor entity.
- `openamber.set_number(number_id, value)`: Injects a float state to a number entity.
- `openamber.set_switch(switch_id, value)`: Turns switch `True` (on) or `False` (off).
- `openamber.set_binary_sensor(sensor_id, value)`: Publishes boolean state to a binary sensor.
- `openamber.get_entity(entity_id)`: Returns the current state of an entity.
- `openamber.step(ms=100)`: Advances the ESPHome execution loop by waiting `ms` milliseconds.
