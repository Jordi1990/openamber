"""Automated tests for sensor fault handling and inside unit register 1225.

Covers:
1. Simulating raw 3000 (0.00 °C) and fault bit 0x0008 for Tuo (F16):
   - error_flag_f16 becomes True.
   - error_active becomes True.
   - outlet_temperature_tuo publishes NaN (None in client).
   - Space heating compressor stops.
   - Clearing fault bit restores normal operation.
2. Simulating F14 (0x0002) for Tw:
   - When DHW is enabled (dhw_enabled_switch=True), error_flag_f14 and error_active become True,
     stopping both DHW and space heating demand.
   - Turning off DHW setting (dhw_enabled_switch=False) clears error_flag_f14 and error_active,
     allowing space heating to work without errors.
3. Simulating F13 (0x0001) for Tr:
   - When thermostat mode is 'Intern', F13 sets error_flag_f13 and error_active to True,
     blocking demand.
   - Switching thermostat mode to 'Extern' clears error_flag_f13 and error_active,
     allowing external thermostat heating demand to work without errors.
4. Simulating F15, F21, F22:
   - F15 (Tc, 0x0004) triggers error_flag_f15 and error_active when Tc is used; switching control source
     to Tv1 (with mixing valves off) clears error_flag_f15 and error_active.
   - F21 (Tv1, 0x0100) triggers error_flag_f21 and error_active when Tv1 is used (source=Tv1 or mixing zone 1 enabled);
     disabling zone 1 mixing valve and selecting Tc clears error_flag_f21 and error_active.
   - F22 (Tv2, 0x0200) triggers error_flag_f22 and error_active when mixing zone 2 is enabled;
     disabling mixing zone 2 clears error_flag_f22 and error_active.
5. Simulating non-error warnings (0x1000, 0x4000, 0x8000):
   - 0x1000: error_flag_inside_comm_failsafe (warning_active=True, error_active=False).
   - 0x4000: error_flag_inside_eeprom (warning_active=True, error_active=False).
   - 0x8000: error_flag_pump_p0_alarm (warning_active=True, error_active=False).
"""

import pytest
from helpers import advance_until_state, start_space_heating


def test_sensor_fault_tuo_stops_compressor_and_sets_error_flag(clean_system):
    """
    Verify simulating raw 3000 plus fault bit for Tuo (0x0008 in register 1225):
    - Compressor is running in space heating.
    - Setting fault bit 0x0008 and raw 3000 for Tuo triggers error_flag_f16 and error_active.
    - outlet_temperature_tuo publishes NaN (None in client).
    - Compressor shuts down and transitions away from 'Compressor running'.
    - Clearing fault bit clears error_flag_f16 and error_active.
    """
    openamber = clean_system

    # 1. Start space heating and wait until compressor is running
    start_space_heating(openamber, tc=28.0, target=35.0)
    assert openamber.get_entity("state_machine_state_heat_cool") == "Compressor running"
    assert openamber.get_entity("error_active") is False
    assert openamber.get_entity("error_flag_f16") is False

    # 2. Simulate raw 3000 (0.00 °C) and fault bit 0x0008 for Tuo
    assert openamber.set_sensor("inside_unit_status_1225_raw", 8.0) is True
    assert openamber.set_sensor("outlet_temperature_tuo", 0.0) is True
    openamber.step(ms=100)

    # 3. Assert error flags and NaN publication
    assert openamber.get_entity("error_flag_f16") is True, "error_flag_f16 should be True when 1225 bit 0x0008 is set"
    assert openamber.get_entity("error_active") is True, "error_active should be True when F16 is active"
    tuo_val = openamber.get_entity("outlet_temperature_tuo")
    assert tuo_val is None, f"outlet_temperature_tuo should publish NaN (None in client), got {tuo_val}"

    # 4. Advance time and verify compressor stops
    stopped = advance_until_state(openamber, "state_machine_state_heat_cool", "Idle", max_s=120, step=5) or \
              advance_until_state(openamber, "state_machine_state_heat_cool", "Wait compressor stop", max_s=30, step=5)
    assert stopped, "Heat/cool controller must stop compressor and leave 'Compressor running' when error_active is set"

    # 5. Clear fault register and restore normal temperature
    assert openamber.set_sensor("inside_unit_status_1225_raw", 0.0) is True
    assert openamber.set_sensor("outlet_temperature_tuo", 35.0) is True
    openamber.step(ms=100)

    assert openamber.get_entity("error_flag_f16") is False
    assert openamber.get_entity("error_active") is False


def test_sensor_fault_f14_stops_all_unless_dhw_disabled(clean_system):
    """
    Verify simulating F14 (bit 0x0002 in register 1225):
    - When dhw_enabled_switch is True, F14 sets error_flag_f14=True and error_active=True,
      blocking both DHW and space heating.
    - When dhw_enabled_switch is turned False, error_flag_f14 and error_active become False,
      and space heating works normally without errors.
    """
    openamber = clean_system

    # Set up normal space heating and DHW demand
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_switch("heat_demand_switch", True)
    openamber.set_switch("dhw_enabled_switch", True)
    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_number("dhw_restart_dhw_delta", 5.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
    openamber.step(ms=100)

    assert openamber.get_entity("dhw_demand_active_sensor") is True, "DHW demand should initially be True"
    assert openamber.get_entity("heat_demand_active_sensor_zone_1") is True, "Space heating demand should initially be True"

    # Simulate F14: bit 0x0002 and raw 3000 (0.00 °C)
    assert openamber.set_sensor("inside_unit_status_1225_raw", 2.0) is True
    assert openamber.set_sensor("dhw_temperature_tw_sensor", 0.0) is True
    openamber.step(ms=100)

    # Verify F14 error flag and error_active both become True, stopping everything
    assert openamber.get_entity("error_flag_f14") is True, "error_flag_f14 should be True when DHW is enabled"
    assert openamber.get_entity("error_active") is True, "error_active must be True when F14 is active"
    assert openamber.get_entity("dhw_demand_active_sensor") is False, "DHW demand must be False when error_active is True"
    assert openamber.get_entity("heat_demand_active_sensor_zone_1") is False, "Space heating must also be blocked when error_active is True"
    tw_val = openamber.get_entity("dhw_temperature_tw_sensor")
    assert tw_val is None, f"dhw_temperature_tw_sensor should publish NaN (None in client), got {tw_val}"

    # Disable DHW setting -> error_flag_f14 and error_active must clear, and space heating must work
    assert openamber.set_switch("dhw_enabled_switch", False) is True
    openamber.step(ms=100)

    assert openamber.get_entity("error_flag_f14") is False, "error_flag_f14 must be False when DHW is disabled"
    assert openamber.get_entity("error_active") is False, "error_active must be False when DHW is disabled"
    assert openamber.get_entity("heat_demand_active_sensor_zone_1") is True, "Space heating must work without errors when DHW is disabled"

    # Re-enable DHW -> error returns while fault bit is still set
    assert openamber.set_switch("dhw_enabled_switch", True) is True
    openamber.step(ms=100)
    assert openamber.get_entity("error_flag_f14") is True
    assert openamber.get_entity("error_active") is True

    # Clear fault
    assert openamber.set_sensor("inside_unit_status_1225_raw", 0.0) is True
    assert openamber.set_sensor("dhw_temperature_tw_sensor", 40.0) is True
    openamber.step(ms=100)

    assert openamber.get_entity("error_flag_f14") is False
    assert openamber.get_entity("error_active") is False
    assert openamber.get_entity("dhw_demand_active_sensor") is True


def test_sensor_fault_f13_stops_all_unless_external_thermostat(clean_system):
    """
    Verify simulating F13 (bit 0x0001 in register 1225):
    - When thermostat is 'Intern', F13 sets error_flag_f13=True and error_active=True, blocking demand.
    - Switching thermostat to 'Extern' clears error_flag_f13 and error_active, allowing heating to work.
    """
    openamber = clean_system

    openamber.set_select("thermostat_mode_select", "Intern")
    openamber.set_sensor("room_temperature", 18.0)
    openamber.set_climate("climate_controller", target_temperature=20.0)
    openamber.step(ms=100)

    assert openamber.get_entity("error_flag_f13") is False
    assert openamber.get_entity("error_active") is False

    # Simulate F13: bit 0x0001 and raw 3000 (0.00 °C)
    assert openamber.set_sensor("inside_unit_status_1225_raw", 1.0) is True
    assert openamber.set_sensor("room_temperature", 0.0) is True
    openamber.step(ms=100)

    assert openamber.get_entity("error_flag_f13") is True
    assert openamber.get_entity("error_active") is True
    tr_val = openamber.get_entity("room_temperature")
    assert tr_val is None, f"room_temperature should publish NaN (None in client), got {tr_val}"
    assert openamber.get_entity("heat_demand_active_sensor_zone_1") is False

    # Switch thermostat mode to 'Extern' -> F13 and error_active clear
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_switch("heat_demand_switch", True)
    openamber.step(ms=100)

    assert openamber.get_entity("error_flag_f13") is False
    assert openamber.get_entity("error_active") is False
    assert openamber.get_entity("heat_demand_active_sensor_zone_1") is True

    # Clear fault
    assert openamber.set_sensor("inside_unit_status_1225_raw", 0.0) is True
    assert openamber.set_sensor("room_temperature", 22.0) is True
    openamber.step(ms=100)


def test_sensor_fault_f15_f21_f22_and_warnings(clean_system):
    """
    Verify F15, F21, F22 and warning bits in register 1225:
    - F15 (Tc, 0x0004) sets error_flag_f15 and error_active when Tc is used; clears when Tv1 is selected (and mixing valves off).
    - F21 (Tv1, 0x0100) sets error_flag_f21 and error_active when Tv1 is used; clears when Tc is selected and zone 1 mixing valve is off.
    - F22 (Tv2, 0x0200) sets error_flag_f22 and error_active when zone 2 mixing valve is enabled; clears when disabled.
    - 0x1000, 0x4000, 0x8000 activate warning_active but NOT error_active.
    """
    openamber = clean_system

    # 1. F15 (0x0004) - Tc fault (always active)
    assert openamber.set_sensor("inside_unit_status_1225_raw", 4.0) is True
    openamber.step(ms=50)
    assert openamber.get_entity("error_flag_f15") is True
    assert openamber.get_entity("error_active") is True
    assert openamber.get_entity("heat_cool_temperature_tc") is None

    # Clear F15
    assert openamber.set_sensor("inside_unit_status_1225_raw", 0.0) is True
    assert openamber.set_sensor("heat_cool_temperature_tc", 35.0) is True
    openamber.step(ms=50)
    assert openamber.get_entity("error_flag_f15") is False
    assert openamber.get_entity("error_active") is False

    # 2. F21 (0x0100 = 256) - Tv1 fault
    # With source = Tc and mixing_valve_zone1_enabled = False, F21 does not trigger an error
    assert openamber.set_sensor("inside_unit_status_1225_raw", 256.0) is True
    openamber.step(ms=50)
    assert openamber.get_entity("error_flag_f21") is False
    assert openamber.get_entity("error_active") is False
    assert openamber.get_entity("heat_cool_temperature_tv1") is None

    # Selecting Tv1 as control source activates F21 and error_active
    openamber.set_select("heat_cool_control_temperature_source_select", "CV-aanvoer zone 1 (Tv1)")
    openamber.step(ms=50)
    assert openamber.get_entity("error_flag_f21") is True
    assert openamber.get_entity("error_active") is True

    # Switching back to Tc clears F21 and error_active
    openamber.set_select("heat_cool_control_temperature_source_select", "CV-aanvoer (Tc)")
    openamber.step(ms=50)
    assert openamber.get_entity("error_flag_f21") is False
    assert openamber.get_entity("error_active") is False

    # Enabling mixing valve zone 1 also activates F21 and error_active
    openamber.set_switch("mixing_valve_zone1_enabled", True)
    openamber.step(ms=50)
    assert openamber.get_entity("error_flag_f21") is True
    assert openamber.get_entity("error_active") is True

    openamber.set_switch("mixing_valve_zone1_enabled", False)
    openamber.step(ms=50)
    assert openamber.get_entity("error_flag_f21") is False
    assert openamber.get_entity("error_active") is False

    # 3. F22 (0x0200 = 512) - Tv2 fault
    # With mixing_valve_zone2_enabled = False, F22 does not trigger an error
    assert openamber.set_sensor("inside_unit_status_1225_raw", 512.0) is True
    openamber.step(ms=50)
    assert openamber.get_entity("error_flag_f22") is False
    assert openamber.get_entity("error_active") is False
    assert openamber.get_entity("heat_cool_temperature_tv2") is None

    # Enabling mixing valve zone 2 activates F22 and error_active, blocking zone 2 demand
    openamber.set_switch("mixing_valve_zone2_enabled", True)
    openamber.set_switch("heat_demand_zone_2_switch", True)
    openamber.step(ms=50)
    assert openamber.get_entity("error_flag_f22") is True
    assert openamber.get_entity("error_active") is True
    assert openamber.get_entity("heat_demand_active_sensor_zone_2") is False

    # Disabling mixing valve zone 2 clears F22 and error_active
    openamber.set_switch("mixing_valve_zone2_enabled", False)
    openamber.step(ms=50)
    assert openamber.get_entity("error_flag_f22") is False
    assert openamber.get_entity("error_active") is False

    # 4. Warnings: 0x1000 (4096), 0x4000 (16384), 0x8000 (32768)
    assert openamber.set_sensor("inside_unit_status_1225_raw", 4096.0) is True
    openamber.step(ms=50)
    assert openamber.get_entity("error_flag_inside_comm_failsafe") is True
    assert openamber.get_entity("warning_active") is True
    assert openamber.get_entity("error_active") is False

    assert openamber.set_sensor("inside_unit_status_1225_raw", 16384.0) is True
    openamber.step(ms=50)
    assert openamber.get_entity("error_flag_inside_eeprom") is True
    assert openamber.get_entity("warning_active") is True
    assert openamber.get_entity("error_active") is False

    assert openamber.set_sensor("inside_unit_status_1225_raw", 32768.0) is True
    openamber.step(ms=50)
    assert openamber.get_entity("error_flag_pump_p0_alarm") is True
    assert openamber.get_entity("warning_active") is True
    assert openamber.get_entity("error_active") is False

    # Clear all
    assert openamber.set_sensor("inside_unit_status_1225_raw", 0.0) is True
    openamber.step(ms=50)
    assert openamber.get_entity("warning_active") is False
    assert openamber.get_entity("error_active") is False
