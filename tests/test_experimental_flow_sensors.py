"""End-to-end sensor wiring in the host simulator."""
import pytest


def test_estimates_publish_with_pump_and_compressor_off(clean_system):
    client = clean_system
    client.set_binary_sensor("internal_pump_active", False)
    client.set_sensor("current_compressor_frequency", 0)
    client.set_sensor("inside_unit_status_1225_raw", 0)
    client.set_sensor("outlet_temperature_tuo", 30)
    client.set_sensor("pump_feedback_1221_raw", 250)
    client.set_sensor("flow_correction_3494_raw", -120)
    try:
        client.advance_time(seconds=6, step_s=1)
        assert client.get_entity("pump_feedback_1221_raw") == 250
        assert client.get_entity("experimental_flow_high") == pytest.approx(22.486667, abs=0.0001)
        assert client.get_entity("experimental_flow_medium_low") == pytest.approx(24.476667, abs=0.0001)

        client.set_sensor("pump_feedback_1221_raw", 0)
        client.advance_time(seconds=6, step_s=1)
        assert client.get_entity("pump_feedback_1221_raw") == 0
        assert client.get_entity("experimental_flow_high") == 0
        assert client.get_entity("experimental_flow_medium_low") == 0

        client.set_sensor("pump_feedback_1221_raw", 1000)
        client.advance_time(seconds=6, step_s=1)
        assert client.get_entity("pump_feedback_1221_raw") == 1000
        assert client.get_entity("experimental_flow_high") == pytest.approx(70.486671, abs=0.0001)
        assert client.get_entity("experimental_flow_medium_low") == pytest.approx(72.476669, abs=0.0001)
    finally:
        client.set_sensor("pump_feedback_1221_raw", 250)
        client.set_sensor("flow_correction_3494_raw", 0)


@pytest.mark.parametrize("status", [0x8000, 0x0008])
def test_fault_makes_estimates_unavailable_and_recovers(clean_system, status):
    client = clean_system
    client.set_sensor("pump_feedback_1221_raw", 250)
    client.set_sensor("flow_correction_3494_raw", 60)
    client.set_sensor("inside_unit_status_1225_raw", status)
    client.set_sensor("outlet_temperature_tuo", 30)
    try:
        client.advance_time(seconds=6, step_s=1)
        assert client.get_entity("pump_feedback_1221_raw") == 250
        assert client.get_entity("experimental_flow_high") is None
        assert client.get_entity("experimental_flow_medium_low") is None
        client.set_sensor("inside_unit_status_1225_raw", 0)
        client.set_sensor("outlet_temperature_tuo", 30)
        client.advance_time(seconds=6, step_s=1)
        assert client.get_entity("experimental_flow_high") == pytest.approx(25.486667, abs=0.0001)
        assert client.get_entity("experimental_flow_medium_low") == pytest.approx(27.476667, abs=0.0001)
    finally:
        client.set_sensor("inside_unit_status_1225_raw", 0)
        client.set_sensor("outlet_temperature_tuo", 30)
        client.set_sensor("flow_correction_3494_raw", 0)
