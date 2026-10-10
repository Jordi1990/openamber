"""Automated tests for mode switching flows between DHW, HEAT, and COOL modes.
Verifies proper priority, valve control, compressor min on/off constraints, and state transitions.
"""

import pytest


def test_switch_heat_to_dhw_and_return_to_heat(clean_system):
    """
    Mode Switching Flow: HEAT -> DHW -> HEAT
    1. System starts in space heating mode (heat demand active, Tc = 28°C < setpoint).
    2. Pump starts on interval (900s), stabilizes (120s), and compressor runs in heating mode (working_mode = 2, valve = CV).
    3. DHW demand activates (tank temperature Tw drops to 38°C < setpoint 50°C - delta 5°C).
    4. Heating finishes compressor min-on-time (600s), stops compressor and pump.
    5. 3-way valve switches to DHW (three_way_valve_dhw_switch = True) and waits 60s switch time.
    6. System enters DHW mode (state_machine_state_main == "DHW") and runs DHW compressor and pump.
    7. DHW tank reaches setpoint (Tw = 52°C >= 50°C), DHW demand clears.
    8. DHW compressor finishes min-on-time (600s), stops compressor and DHW pump.
    9. 3-way valve switches back to CV (three_way_valve_dhw_switch = False) and waits 60s switch time.
    10. System returns to Heat/Cool mode (state_machine_state_main == "Heat/Cool") and resumes space heating.
    """
    openamber = clean_system

    # Step 1: Start Space Heating
    openamber.set_switch("heat_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.step(ms=50)

    # Advance virtual time through IDLE pump interval (900s) + pump settle (120s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=50)

    # Verify Space Heating is running
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool"
    assert openamber.get_entity("three_way_valve_dhw_switch") is False
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"

    # Step 2: DHW Demand occurs during active heating
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.step(ms=50)
    assert openamber.get_entity("dhw_demand_active_sensor") is True

    # Step 3: Advance through space heating compressor min-on-time (600s) + valve switch (60s) + DHW compressor start (140s)
    openamber.advance_time(seconds=800, step_s=20)
    openamber.step(ms=50)

    # Verify transition to DHW mode
    assert openamber.get_entity("state_machine_state_main") == "DHW"
    assert openamber.get_entity("three_way_valve_dhw_switch") is True
    assert openamber.get_entity("three_way_valve_active_sensor") is True
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0
    assert openamber.get_entity("dhw_pump_relay_switch") is True

    # Step 4: DHW tank reaches target temperature
    openamber.set_sensor("dhw_temperature_tw_sensor", 52.0)
    openamber.step(ms=50)
    assert openamber.get_entity("dhw_demand_active_sensor") is False

    # Step 5: Advance through DHW compressor min-on-time (600s) + valve switch back to CV (60s)
    openamber.advance_time(seconds=700, step_s=20)
    openamber.step(ms=50)

    # Verify valve returned to CV and main state is Heat/Cool
    assert openamber.get_entity("three_way_valve_dhw_switch") is False
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool"

    # Step 6: Advance through next pump interval (900s) + pump settle (120s) to resume space heating
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=50)

    assert int(openamber.get_entity("compressor_control_select") or 0) > 0
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"


def test_switch_cool_to_dhw_and_return_to_cool(clean_system):
    """
    Mode Switching Flow: COOL -> DHW -> COOL
    1. System starts in space cooling mode (cool demand active, Tc = 25°C > target 18°C + start delta).
    2. Pump starts on interval (900s), stabilizes, and compressor runs in cooling mode (working_mode = 1, navbar snowflake visible).
    3. DHW demand activates (tank temperature Tw drops to 38°C).
    4. Cooling finishes compressor min-on-time (600s), stops compressor and pump.
    5. 3-way valve switches to DHW and waits 60s switch time.
    6. System enters DHW mode and runs DHW compressor and pump in heating mode.
    7. DHW tank reaches setpoint (Tw = 52°C), DHW demand clears.
    8. DHW compressor finishes min-on-time (600s), stops compressor and DHW pump.
    9. 3-way valve switches back to CV.
    10. System returns to Heat/Cool mode and resumes space cooling (working_mode = 1, cooling compressor running).
    """
    openamber = clean_system

    # Step 1: Start Space Cooling
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_switch("cool_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 25.0)
    openamber.set_sensor("heat_cool_temperature_tc", 25.0)  # > 18°C target + 3°C delta
    openamber.set_sensor("outlet_temperature_tuo", 25.0)
    openamber.set_sensor("inlet_temperature_tui", 25.0)
    openamber.step(ms=50)

    # Advance virtual time through IDLE pump interval (900s) + pump settle (120s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=50)

    # Verify Space Cooling is running
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool"
    assert openamber.get_entity("cool_demand_active_sensor") is True
    assert openamber.is_visible("nav_status_cool_icon")
    assert openamber.get_entity("three_way_valve_dhw_switch") is False
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0
    assert openamber.get_entity("working_mode_switch") == "Koelen"

    # Step 2: DHW Demand occurs during active cooling
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.step(ms=50)
    assert openamber.get_entity("dhw_demand_active_sensor") is True

    # Step 3: Advance through cooling compressor min-on-time (600s) + valve switch (60s) + DHW start (140s)
    openamber.advance_time(seconds=800, step_s=20)
    openamber.step(ms=50)

    # Verify transition to DHW mode
    assert openamber.get_entity("state_machine_state_main") == "DHW"
    assert openamber.get_entity("three_way_valve_dhw_switch") is True
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0
    assert openamber.get_entity("dhw_pump_relay_switch") is True
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"

    # Step 4: DHW tank reaches target temperature
    openamber.set_sensor("dhw_temperature_tw_sensor", 52.0)
    openamber.step(ms=50)
    assert openamber.get_entity("dhw_demand_active_sensor") is False

    # Step 5: Advance through DHW compressor min-on-time (600s) + valve switch back to CV (60s)
    openamber.advance_time(seconds=700, step_s=20)
    openamber.step(ms=50)

    # Verify valve returned to CV and main state is Heat/Cool
    assert openamber.get_entity("three_way_valve_dhw_switch") is False
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool"

    # Step 6: Advance through next pump interval (900s) + pump settle (120s) to resume space cooling
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("cool_demand_active_sensor") is True
    assert openamber.is_visible("nav_status_cool_icon")
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0
    assert openamber.get_entity("working_mode_switch") == "Koelen"


def test_switch_heat_to_cool_mode(clean_system):
    """
    Mode Switching Flow: HEAT -> COOL
    1. System starts in space heating mode (heat_demand_switch = True, Tc = 28°C).
    2. Compressor runs in heating mode (working_mode = 2).
    3. Heating demand ends (heat_demand_switch = False) and cooling demand starts (cool_demand_switch = True, Tc = 25°C).
    4. Heating compressor finishes min-on-time (600s) and stops.
    5. System respects compressor min-off-time (120s), settles, and starts cooling on next pump cycle.
    6. System transitions to cooling mode: working_mode = 1 (Koelen), nav_status_cool_icon visible, compressor runs.
    """
    openamber = clean_system

    # Step 1: Start Heating
    openamber.set_switch("heat_demand_switch", True)
    openamber.set_switch("cool_demand_switch", False)
    openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.step(ms=50)

    # Advance virtual time through IDLE pump interval (900s) + pump settle (120s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("heat_demand_active_sensor") is True
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0

    # Step 2: Switch demands from Heat to Cool
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_switch("cool_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 25.0)
    openamber.set_sensor("heat_cool_temperature_tc", 25.0)  # > cooling target 18 + delta 3
    openamber.set_sensor("outlet_temperature_tuo", 25.0)
    openamber.set_sensor("inlet_temperature_tui", 25.0)
    openamber.step(ms=50)

    assert openamber.get_entity("heat_demand_active_sensor") is False
    assert openamber.get_entity("cool_demand_active_sensor") is True
    assert openamber.is_visible("nav_status_cool_icon")

    # Step 3: Advance virtual time past heating compressor min-on-time (600s) + pump cycle finish (130s)
    openamber.advance_time(seconds=750, step_s=20)
    openamber.step(ms=50)

    # Step 4: Advance through next IDLE pump cycle (900s) + pump settle (120s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=50)

    # Verify system has transitioned to Cooling mode
    assert openamber.get_entity("working_mode_switch") == "Koelen"
    assert openamber.is_visible("nav_status_cool_icon")
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0


def test_switch_cool_to_heat_mode(clean_system):
    """
    Mode Switching Flow: COOL -> HEAT
    1. System starts in space cooling mode (cool_demand_switch = True, Tc = 25°C).
    2. Compressor runs in cooling mode (working_mode = 1, snowflake visible).
    3. Heating demand activates (heat_demand_switch = True, Tc = 28°C).
    4. Heating demand immediately suppresses cooling demand (heat_demand_active = True, cool_demand_active = False).
    5. Cooling compressor finishes min-on-time (600s) and stops.
    6. System respects compressor min-off-time (120s), settles, and starts heating on next pump cycle.
    7. System transitions to heating mode: working_mode = 2 (Verwarmen), snowflake icon hidden, compressor runs.
    """
    openamber = clean_system

    # Step 1: Start Cooling
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_switch("cool_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 25.0)
    openamber.set_sensor("heat_cool_temperature_tc", 25.0)
    openamber.set_sensor("outlet_temperature_tuo", 25.0)
    openamber.set_sensor("inlet_temperature_tui", 25.0)
    openamber.step(ms=50)

    # Advance virtual time through IDLE pump interval (900s) + pump settle (120s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("cool_demand_active_sensor") is True
    assert openamber.is_visible("nav_status_cool_icon")
    assert openamber.get_entity("working_mode_switch") == "Koelen"
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0

    # Step 2: Heating demand activates
    openamber.set_switch("heat_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.step(ms=50)

    # Heating demand suppresses cooling demand immediately
    assert openamber.get_entity("heat_demand_active_sensor") is True
    assert openamber.get_entity("cool_demand_active_sensor") is False
    assert openamber.is_hidden("nav_status_cool_icon")

    # Step 3: Advance virtual time past cooling compressor min-on-time (600s) + pump cycle finish (130s)
    openamber.advance_time(seconds=750, step_s=20)
    openamber.step(ms=50)

    # Step 4: Advance through next IDLE pump cycle (900s) + pump settle (120s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=50)

    # Verify system has transitioned to Heating mode
    assert openamber.get_entity("working_mode_switch") == "Verwarmen"
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0


def test_switch_dhw_demand_during_idle_and_return_to_cv(clean_system):
    """
    Mode Switching Flow: IDLE -> DHW -> IDLE (CV)
    1. System is in IDLE (Heat/Cool main state, 3-way valve in CV position).
    2. DHW demand activates (Tw = 38°C < setpoint 50°C - delta 5°C).
    3. 3-way valve immediately switches to DHW circuit (three_way_valve_dhw_switch = True).
    4. After valve switch time (60s), system enters DHW mode (state_machine_state_main == "DHW").
    5. DHW compressor and pump run to heat the tank.
    6. Tank reaches target temperature (Tw = 52°C), DHW demand clears.
    7. After DHW compressor min-on-time (600s), compressor and DHW pump stop.
    8. 3-way valve switches back to CV (three_way_valve_dhw_switch = False).
    9. After valve switch time (60s), system returns to Heat/Cool idle.
    """
    openamber = clean_system

    # Verify initial idle state with valve in CV position
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool"
    assert openamber.get_entity("three_way_valve_dhw_switch") is False
    assert int(openamber.get_entity("compressor_control_select") or 0) == 0

    # Step 1: DHW Demand occurs while in IDLE
    openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    openamber.step(ms=50)
    assert openamber.get_entity("dhw_demand_active_sensor") is True

    # Step 2: Advance time for 3-way valve switch (60s) + DHW compressor start (140s)
    openamber.advance_time(seconds=200, step_s=10)
    openamber.step(ms=50)

    # Verify transition to DHW mode
    assert openamber.get_entity("state_machine_state_main") == "DHW"
    assert openamber.get_entity("three_way_valve_dhw_switch") is True
    assert openamber.get_entity("three_way_valve_active_sensor") is True
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0
    assert openamber.get_entity("dhw_pump_relay_switch") is True

    # Step 3: DHW tank reaches setpoint
    openamber.set_sensor("dhw_temperature_tw_sensor", 52.0)
    openamber.step(ms=50)
    assert openamber.get_entity("dhw_demand_active_sensor") is False

    # Step 4: Advance past DHW compressor min-on-time (600s) + valve switch back to CV (60s)
    openamber.advance_time(seconds=700, step_s=20)
    openamber.step(ms=50)

    # Verify valve returned to CV position and main state returned to Heat/Cool idle
    assert openamber.get_entity("three_way_valve_dhw_switch") is False
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool"
    assert int(openamber.get_entity("compressor_control_select") or 0) == 0


def test_simultaneous_demands_heat_priority_over_cooling(clean_system):
    """
    Priority Flow: Simultaneous HEAT & COOL demands
    1. Both heating demand and cooling demand are activated at the same time.
    2. Heating takes absolute priority: heat_demand_active is True, cool_demand_active is suppressed (False).
    3. Heating compressor runs.
    4. Heating demand ends while cooling demand remains active.
    5. System transitions safely from heating mode to cooling mode after min-on and min-off times.
    """
    openamber = clean_system

    # Step 1: Simultaneously activate Heat and Cool demands
    openamber.set_switch("heat_demand_switch", True)
    openamber.set_switch("cool_demand_switch", True)
    openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    openamber.set_sensor("outlet_temperature_tuo", 28.0)
    openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.step(ms=50)

    # Verify heating priority: cool demand suppressed
    assert openamber.get_entity("heat_demand_active_sensor") is True
    assert openamber.get_entity("cool_demand_active_sensor") is False
    assert openamber.is_visible("nav_status_heat_icon")
    assert openamber.is_hidden("nav_status_cool_icon")

    # Step 2: Advance virtual time to start Heating compressor
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=50)

    assert openamber.get_entity("working_mode_switch") == "Verwarmen"
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0

    # Step 3: Turn off heating demand, keeping cooling demand on
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_sensor("current_water_temperature_tc_sensor", 25.0)
    openamber.set_sensor("heat_cool_temperature_tc", 25.0)
    openamber.set_sensor("outlet_temperature_tuo", 25.0)
    openamber.set_sensor("inlet_temperature_tui", 25.0)
    openamber.step(ms=50)

    # Now cooling demand activates and heating demand deactivates
    assert openamber.get_entity("heat_demand_active_sensor") is False
    assert openamber.get_entity("cool_demand_active_sensor") is True
    assert openamber.is_visible("nav_status_cool_icon")
    assert openamber.is_hidden("nav_status_heat_icon")

    # Step 4: Advance past heating compressor min-on-time (600s) + pump cycle finish (130s)
    openamber.advance_time(seconds=750, step_s=20)
    openamber.step(ms=50)

    # Step 5: Advance through next IDLE pump cycle (900s) + pump settle (120s)
    openamber.advance_time(seconds=900, step_s=30)
    openamber.advance_time(seconds=140, step_s=10)
    openamber.step(ms=50)

    # Verify transition to Cooling mode
    assert openamber.get_entity("working_mode_switch") == "Koelen", "Working mode should transition to Koelen"
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0, "Compressor should run in cooling mode"


def test_simultaneous_demands_dhw_priority_over_heat_and_cool(clean_system):
    """
    Priority Flow: Simultaneous DHW, HEAT & COOL demands
    1. Activate DHW, HEAT, and COOL simultaneously.
    2. DHW must take absolute top priority: 3-way valve switches to DHW, system enters DHW mode.
    3. Both Heat and Cool are held while DHW is running.
    4. Once DHW is satisfied, system returns to Heat/Cool, and HEAT takes priority over COOL.
    """
    openamber = clean_system

    # Step 1: Simultaneously activate DHW, Heat, and Cool demands
    assert openamber.set_switch("heat_demand_switch", True)
    assert openamber.set_switch("cool_demand_switch", True)
    assert openamber.set_sensor("dhw_temperature_tw_sensor", 38.0)
    assert openamber.set_sensor("current_water_temperature_tc_sensor", 28.0)
    assert openamber.set_sensor("heat_cool_temperature_tc", 28.0)
    assert openamber.set_sensor("outlet_temperature_tuo", 28.0)
    assert openamber.set_sensor("inlet_temperature_tui", 26.0)
    openamber.step(ms=50)

    assert openamber.get_entity("dhw_demand_active_sensor") is True, "DHW demand should be active"
    # In Heat/Cool demand evaluation, Heat takes priority over Cool
    assert openamber.get_entity("heat_demand_active_sensor") is True, "Heat demand should be active"
    assert openamber.get_entity("cool_demand_active_sensor") is False, "Cool demand should be suppressed by heat demand"

    # Step 2: Advance time for valve switch (60s) + DHW compressor start (140s)
    openamber.advance_time(seconds=200, step_s=10)
    openamber.step(ms=50)

    # DHW mode takes priority over both Heat and Cool
    assert openamber.get_entity("state_machine_state_main") == "DHW", "Main state machine should enter DHW"
    assert openamber.get_entity("three_way_valve_dhw_switch") is True, "3-way valve must align to DHW"
    assert int(openamber.get_entity("compressor_control_select") or 0) > 0, "Compressor must run for DHW"

    # Step 3: DHW satisfied
    assert openamber.set_sensor("dhw_temperature_tw_sensor", 52.0)
    openamber.step(ms=50)
    assert openamber.get_entity("dhw_demand_active_sensor") is False, "DHW demand should clear at setpoint"

    # Step 4: Advance past DHW min-on-time (600s) + valve switch back to CV (60s)
    openamber.advance_time(seconds=700, step_s=20)
    openamber.step(ms=50)

    # Returned to Heat/Cool, where Heat demand immediately takes priority over Cool
    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool", "System should return to Heat/Cool"
    assert openamber.get_entity("three_way_valve_dhw_switch") is False, "3-way valve must return to CV"
    assert openamber.get_entity("heat_demand_active_sensor") is True, "Heat demand should still be active"
    assert openamber.get_entity("cool_demand_active_sensor") is False, "Cool demand must remain suppressed by heat priority"

    # Cleanup
    assert openamber.set_switch("heat_demand_switch", False)
    assert openamber.set_switch("cool_demand_switch", False)
    openamber.step(ms=50)


