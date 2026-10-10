"""Automated tests for deaeration routine (ontluchting).

Covers:
1. Deaeration routine cannot start outside maintenance mode.
2. Short deaeration routine (deaeration_short_switch) in maintenance mode:
   - Starts pump P0.
   - Sets state_machine_state_routine from "Inactief" to active phase.
   - Turning off switch stops deaeration, restores pump P0 and "Inactief".
3. Extended deaeration routine (deaeration_extended_switch) in maintenance mode:
   - Starts pump P0 and sets active state.
   - Turning off switch aborts deaeration and restores "Inactief".
4. Exiting maintenance mode while deaeration is running automatically aborts deaeration.
"""

import pytest


def test_deaeration_cannot_start_outside_maintenance_mode(clean_system):
    """
    Verify that deaeration routine cannot be started when not in maintenance mode.
    """
    openamber = clean_system

    # Advance past initialization to normal operation
    openamber.advance_time(seconds=20, step_s=5)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_main") == "Heat/Cool", "System should be in Heat/Cool"
    assert openamber.get_entity("service_mode_enabled") is False, "service_mode_enabled should be False"
    assert openamber.get_entity("state_machine_state_routine") in ("", "Inactief"), (
        f"Routine state should be unstarted/Inactief, got {openamber.get_entity('state_machine_state_routine')}"
    )

    # Attempt to start short deaeration
    assert openamber.set_switch("deaeration_short_switch", True) is True
    openamber.step(ms=50)

    # Deaeration must not have started
    assert openamber.get_entity("deaeration_short_switch") is False, (
        "deaeration_short_switch should not stay on outside maintenance mode"
    )
    assert openamber.get_entity("state_machine_state_routine") in ("", "Inactief"), (
        "Routine state should remain unstarted/Inactief"
    )


def test_deaeration_short_routine_start_and_stop(clean_system):
    """
    Verify short deaeration routine lifecycle in maintenance mode:
    - Entering maintenance mode.
    - Starting deaeration_short_switch starts pump P0 and publishes phase state.
    - Stopping deaeration_short_switch stops pump P0 and restores Inactief.
    """
    openamber = clean_system

    # Advance past initialization
    openamber.advance_time(seconds=20, step_s=5)
    openamber.step(ms=50)

    # Enter maintenance mode
    assert openamber.set_switch("service_mode_enabled", True) is True
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_main") == "Maintenance"

    # Initially routine state is Inactief / unstarted
    assert openamber.get_entity("state_machine_state_routine") in ("", "Inactief")
    assert openamber.get_entity("pump_p0_relay_switch") is False

    # Start short deaeration routine
    assert openamber.set_switch("deaeration_short_switch", True) is True
    openamber.step(ms=50)

    assert openamber.get_entity("deaeration_short_switch") is True, "deaeration_short_switch should be True"
    assert openamber.get_entity("pump_p0_relay_switch") is True, "Pump P0 relay should be turned on"
    phase_text = openamber.get_entity("state_machine_state_routine")
    assert phase_text != "Inactief", f"Routine state should be active, got: {phase_text}"
    assert "Pomp starten" in phase_text or "Hoog" in phase_text or "CV" in phase_text, (
        f"Unexpected routine state text: {phase_text}"
    )

    # Stop short deaeration routine
    assert openamber.set_switch("deaeration_short_switch", False) is True
    openamber.step(ms=50)

    assert openamber.get_entity("deaeration_short_switch") is False, "deaeration_short_switch should be False"
    assert openamber.get_entity("pump_p0_relay_switch") is False, "Pump P0 should be stopped"
    assert openamber.get_entity("state_machine_state_routine") == "Inactief", (
        f"Routine state should return to Inactief, got {openamber.get_entity('state_machine_state_routine')}"
    )


def test_deaeration_extended_routine_start_and_stop(clean_system):
    """
    Verify extended deaeration routine lifecycle in maintenance mode:
    - Starting deaeration_extended_switch starts pump P0 and publishes phase state.
    - Stopping deaeration_extended_switch stops pump P0 and restores Inactief.
    """
    openamber = clean_system

    # Advance past initialization
    openamber.advance_time(seconds=20, step_s=5)
    openamber.step(ms=50)

    # Enter maintenance mode
    assert openamber.set_switch("service_mode_enabled", True) is True
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_main") == "Maintenance"

    # Start extended deaeration routine
    assert openamber.set_switch("deaeration_extended_switch", True) is True
    openamber.step(ms=50)

    assert openamber.get_entity("deaeration_extended_switch") is True, "deaeration_extended_switch should be True"
    assert openamber.get_entity("pump_p0_relay_switch") is True, "Pump P0 relay should be turned on"
    phase_text = openamber.get_entity("state_machine_state_routine")
    assert phase_text != "Inactief", f"Routine state should be active, got: {phase_text}"

    # Stop extended deaeration routine
    assert openamber.set_switch("deaeration_extended_switch", False) is True
    openamber.step(ms=50)

    assert openamber.get_entity("deaeration_extended_switch") is False, "deaeration_extended_switch should be False"
    assert openamber.get_entity("pump_p0_relay_switch") is False, "Pump P0 should be stopped"
    assert openamber.get_entity("state_machine_state_routine") == "Inactief", (
        f"Routine state should return to Inactief, got {openamber.get_entity('state_machine_state_routine')}"
    )


def test_exiting_maintenance_mode_aborts_active_deaeration(clean_system):
    """
    Verify that disabling service_mode_enabled while deaeration is running
    automatically aborts the deaeration routine and shuts down the pump.
    """
    openamber = clean_system

    # Advance past initialization
    openamber.advance_time(seconds=20, step_s=5)
    openamber.step(ms=50)

    # Enter maintenance mode
    assert openamber.set_switch("service_mode_enabled", True) is True
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=50)
    assert openamber.get_entity("state_machine_state_main") == "Maintenance"

    # Start short deaeration routine
    assert openamber.set_switch("deaeration_short_switch", True) is True
    openamber.step(ms=50)
    assert openamber.get_entity("pump_p0_relay_switch") is True

    # Disable maintenance mode directly
    assert openamber.set_switch("service_mode_enabled", False) is True
    openamber.step(ms=50)

    # Advance time for component update to detect service_mode_enabled == False
    openamber.advance_time(seconds=10, step_s=1)
    openamber.step(ms=50)

    assert openamber.get_entity("state_machine_state_routine") == "Inactief", (
        "Routine state should be reset to Inactief after exiting maintenance mode"
    )
    assert openamber.get_entity("deaeration_short_switch") is False, (
        "deaeration_short_switch should be reset to False"
    )
