import pytest
from openamber_client import OpenAmberClient


def test_eebus_initial_mock_state(openamber: OpenAmberClient):
    """Verify that EEBUS mock entities initialize properly on host."""
    openamber.step(ms=100)
    # Ensure EEBUS switch is enabled
    openamber.set_switch("eebus_enabled_switch", True)
    openamber.step(ms=100)

    ski = openamber.get_entity("eebus_ski")
    assert ski is not None and len(str(ski)) > 0
    assert "MOCK" in str(ski) or len(str(ski)) == 40 or "0123" in str(ski)


def test_eebus_fake_boost_and_restore(openamber: OpenAmberClient):
    """Verify that fake EEBUS boost request switches SG-Ready mode to Boost and back."""
    openamber.set_switch("eebus_enabled_switch", True)
    openamber.set_select("sg_ready_mode_select", "Normaal")
    openamber.step(ms=100)

    # 1. Trigger EEBUS boost request
    assert openamber.eebus_request_boost(True)
    openamber.step(ms=100)

    assert openamber.get_entity("sg_ready_mode_select") == "Boost"
    assert openamber.get_entity("eebus_status_sensor") == "Boost active"
    assert openamber.get_entity("eebus_connected_sensor") is True
    assert "EVCC" in str(openamber.get_entity("eebus_ems_sensor"))

    # 2. Release EEBUS boost request
    assert openamber.eebus_request_boost(False)
    openamber.step(ms=100)

    assert openamber.get_entity("sg_ready_mode_select") == "Normaal"
    assert openamber.get_entity("eebus_status_sensor") == "Normal operation"


def test_eebus_fake_dim_and_restore(openamber: OpenAmberClient):
    """Verify that fake EEBUS dim (§14a) request switches SG-Ready mode to Blokkeren and back."""
    openamber.set_switch("eebus_enabled_switch", True)
    openamber.set_select("sg_ready_mode_select", "Normaal")
    openamber.step(ms=100)

    # 1. Trigger EEBUS dim request
    assert openamber.eebus_request_dim(True)
    openamber.step(ms=100)

    assert openamber.get_entity("sg_ready_mode_select") == "Blokkeren"
    assert openamber.get_entity("eebus_status_sensor") == "Dimmed (§14a)"

    # 2. Release EEBUS dim request
    assert openamber.eebus_request_dim(False)
    openamber.step(ms=100)

    assert openamber.get_entity("sg_ready_mode_select") == "Normaal"
    assert openamber.get_entity("eebus_status_sensor") == "Normal operation"


def test_eebus_disconnect(openamber: OpenAmberClient):
    """Verify that fake EEBUS disconnect clears connected sensor and EMS info."""
    openamber.set_switch("eebus_enabled_switch", True)
    assert openamber.eebus_connect("EVCC (192.168.1.100)")
    openamber.step(ms=100)

    assert openamber.get_entity("eebus_connected_sensor") is True
    assert openamber.get_entity("eebus_ems_sensor") == "EVCC (192.168.1.100)"

    assert openamber.eebus_disconnect()
    openamber.step(ms=100)

    assert openamber.get_entity("eebus_connected_sensor") is False
    assert openamber.get_entity("eebus_ems_sensor") == "–"
    assert openamber.get_entity("eebus_status_sensor") == "Disconnected"
