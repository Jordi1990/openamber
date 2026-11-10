"""Automated tests for the controlled stop ("Gecontroleerd stoppen").

The user can stop the system cleanly, for maintenance or before switching it off. A controlled stop does not wait
for the minimum compressor on time, but finishes a running defrost or oil return cycle first. When the compressor
and pumps are off the main state is "Standby" and the status says it is safe to switch off. Regulation resumes when
the switch goes off, after 30 minutes, or when frost protection becomes active.
"""

from helpers import advance_until_state, start_space_heating

SWITCH = "controlled_stop_enabled"
STATUS = "controlled_stop_status"
SAFE = "Gestopt - veilig om uit te schakelen"


def compressor_mode(openamber):
    return int(openamber.get_entity("compressor_control_select") or 0)


def stop_and_wait_until_safe(openamber, max_s=600):
    openamber.set_switch(SWITCH, True)
    openamber.step(ms=50)
    assert advance_until_state(openamber, STATUS, SAFE, max_s=max_s, step=5), (
        f"Never reached a safe stop (status: {openamber.get_entity(STATUS)}, "
        f"main: {openamber.get_entity('state_machine_state_main')})"
    )


def resume(openamber):
    openamber.set_switch(SWITCH, False)
    openamber.step(ms=50)
    # The status follows within one controller update (5 s).
    assert advance_until_state(openamber, STATUS, "Uit", max_s=10, step=1)


def test_controlled_stop_during_heating_skips_minimum_on_time(clean_system):
    """The compressor stops right away, well before the 10 min minimum on time, then the pumps; then Standby."""
    openamber = clean_system
    start_space_heating(openamber)
    assert compressor_mode(openamber) > 0

    openamber.set_switch(SWITCH, True)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    assert compressor_mode(openamber) == 0, "A controlled stop must not wait for the minimum on time"

    assert advance_until_state(openamber, STATUS, SAFE, max_s=600, step=5)
    assert openamber.get_entity("state_machine_state_main") == "Standby"
    assert openamber.get_entity("pump_p0_relay_switch") is False

    # No new start while stopped, although there is still heat demand.
    openamber.advance_time(seconds=600, step_s=10)
    openamber.step(ms=50)
    assert compressor_mode(openamber) == 0
    assert openamber.get_entity("state_machine_state_main") == "Standby"

    resume(openamber)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=1800), (
        "Heating must start again after resuming"
    )
    openamber.set_switch("heat_demand_switch", False)
    openamber.advance_time(seconds=30, step_s=5)


def test_controlled_stop_during_dhw(clean_system):
    """DHW stops as well while the tank still asks for heat."""
    openamber = clean_system
    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_number("dhw_restart_dhw_delta", 5.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 40.0)
    openamber.step(ms=100)
    assert advance_until_state(openamber, "state_machine_state_dhw", "Compressor running", max_s=900, step=5)

    stop_and_wait_until_safe(openamber)
    assert compressor_mode(openamber) == 0
    assert openamber.get_entity("dhw_demand_active_sensor") is True, "The tank still asks for heat"

    resume(openamber)
    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    openamber.advance_time(seconds=120, step_s=10)


def test_controlled_stop_finishes_oil_return_cycle_first(clean_system):
    """A running oil return cycle is finished before the compressor stops."""
    openamber = clean_system
    start_space_heating(openamber)
    assert openamber.set_sensor("error_register_2119_raw", 8.0)
    openamber.step(ms=50)

    openamber.set_switch(SWITCH, True)
    openamber.advance_time(seconds=60, step_s=5)
    openamber.step(ms=50)
    assert compressor_mode(openamber) > 0, "The compressor must run until the oil return cycle ends"
    assert openamber.get_entity(STATUS) == "Wacht op defrost/oliereturn"

    assert openamber.set_sensor("error_register_2119_raw", 0.0)
    openamber.advance_time(seconds=10, step_s=2)
    openamber.step(ms=50)
    assert compressor_mode(openamber) == 0
    assert advance_until_state(openamber, STATUS, SAFE, max_s=600, step=5)

    resume(openamber)
    openamber.set_switch("heat_demand_switch", False)
    openamber.advance_time(seconds=30, step_s=5)


def test_controlled_stop_finishes_defrost_first(clean_system):
    """A running defrost is finished before the compressor stops."""
    openamber = clean_system
    start_space_heating(openamber)
    openamber.set_binary_sensor("defrost_active_sensor", True)
    openamber.step(ms=50)

    openamber.set_switch(SWITCH, True)
    openamber.advance_time(seconds=60, step_s=5)
    openamber.step(ms=50)
    assert compressor_mode(openamber) > 0, "The compressor must run until the defrost ends"
    assert openamber.get_entity(STATUS) == "Wacht op defrost/oliereturn"

    openamber.set_binary_sensor("defrost_active_sensor", False)
    assert advance_until_state(openamber, STATUS, SAFE, max_s=900, step=5)
    assert compressor_mode(openamber) == 0

    resume(openamber)
    openamber.set_switch("heat_demand_switch", False)
    openamber.advance_time(seconds=30, step_s=5)


def test_controlled_stop_resumes_after_30_minutes(clean_system):
    """A forgotten controlled stop ends by itself after 30 minutes."""
    openamber = clean_system
    stop_and_wait_until_safe(openamber)

    openamber.advance_time(seconds=29 * 60, step_s=30)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_main") == "Standby"

    openamber.advance_time(seconds=90, step_s=10)
    openamber.step(ms=50)
    assert openamber.get_entity(SWITCH) is False, "The switch must turn off after 30 minutes"
    assert advance_until_state(openamber, STATUS, "Uit", max_s=10, step=1)
    assert advance_until_state(openamber, "state_machine_state_main", "Heat/Cool", max_s=300, step=5)


def test_controlled_stop_resumes_for_frost_protection(clean_system):
    """Frost protection takes precedence over a controlled stop."""
    openamber = clean_system
    stop_and_wait_until_safe(openamber)

    openamber.set_sensor("temperature_outside_ta", 2.0)
    assert advance_until_state(openamber, SWITCH, False, max_s=600, step=10), "Frost protection must end the stop"
    assert advance_until_state(openamber, "state_machine_state_main", "Heat/Cool", max_s=300, step=5)
    openamber.set_sensor("temperature_outside_ta", 7.0)
    openamber.advance_time(seconds=400, step_s=20)


def test_system_tab_button_and_status(openamber):
    """The button on System > Beheer asks for confirmation, then stops; the status line follows."""
    assert openamber.click("nav_system")
    openamber.step(ms=100)
    assert openamber.get_label("service_controlled_stop_button_label") == "Stoppen"

    assert openamber.click("service_controlled_stop_button")
    openamber.step(ms=100)
    assert openamber.is_visible("service_confirm_modal"), "Stopping asks for confirmation"
    assert openamber.click("service_confirm_button")
    openamber.step(ms=100)
    assert openamber.get_entity(SWITCH) is True
    assert openamber.get_label("service_controlled_stop_button_label") == "Hervatten"
    assert advance_until_state(openamber, STATUS, SAFE, max_s=600, step=5)
    openamber.step(ms=100)
    assert "veilig om uit te schakelen" in openamber.get_label("service_controlled_stop_status")

    # Resuming does not ask for confirmation.
    assert openamber.click("service_controlled_stop_button")
    openamber.step(ms=100)
    assert openamber.get_entity(SWITCH) is False
    assert openamber.get_label("service_controlled_stop_button_label") == "Stoppen"
    assert openamber.click("nav_home")
    openamber.step(ms=100)
