import os
import subprocess
import time
import pytest
from openamber_client import OpenAmberClient


@pytest.fixture(scope="session")
def openamber_proc():
    """Launch OpenAmber headless simulator process (Linux native or WSL)."""
    # Check if an instance is already listening on port 8888
    try:
        test_client = OpenAmberClient(host="127.0.0.1", port=8888, timeout=0.5)
        test_client.connect(retries=1, delay=0.1)
        if test_client.ping():
            test_client.close()
            # Already running, reuse it
            yield None
            return
        test_client.close()
    except Exception:
        pass

    started_proc = None
    if os.name == "nt":
        # Windows host: invoke via WSL Debian
        subprocess.run(["wsl", "-d", "Debian", "pkill", "-9", "program"], capture_output=True)
        time.sleep(0.3)
        cmd = [
            "wsl", "-d", "Debian", "bash", "-c",
            "SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy /mnt/g/openamber/src/.esphome/build/openamber-test/.pioenvs/openamber-test/program"
        ]
        started_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        # Native Linux / CI
        subprocess.run(["pkill", "-9", "program"], capture_output=True)
        time.sleep(0.3)
        bin_path = os.environ.get(
            "OPENAMBER_BIN",
            os.path.join(os.path.dirname(__file__), "..", "src", ".esphome", "build", "openamber-test", ".pioenvs", "openamber-test", "program")
        )
        env = os.environ.copy()
        env["SDL_VIDEODRIVER"] = "dummy"
        env["SDL_AUDIODRIVER"] = "dummy"
        started_proc = subprocess.Popen([bin_path], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    time.sleep(1.0)
    yield started_proc

    if started_proc:
        try:
            started_proc.terminate()
            started_proc.wait(timeout=2.0)
        except Exception:
            if os.name == "nt":
                subprocess.run(["wsl", "-d", "Debian", "pkill", "-9", "program"], capture_output=True)
            else:
                subprocess.run(["pkill", "-9", "program"], capture_output=True)


@pytest.fixture(scope="session")
def openamber(openamber_proc):
    """Provide a connected OpenAmberClient fixture for test suites."""
    client = OpenAmberClient(host="127.0.0.1", port=8888, timeout=5.0)
    client.connect(retries=30, delay=0.5)
    yield client
    client.close()


@pytest.fixture
def clean_system(openamber):
    """Reset system to clean idle state with fully completed initialization."""
    openamber.set_switch("heat_demand_switch", False)
    openamber.set_switch("cool_demand_switch", False)
    openamber.set_switch("emergency_mode_enabled", False)
    openamber.set_switch("dhw_enabled_switch", True)
    openamber.set_binary_sensor("external_heat_demand_wired", False)
    openamber.set_binary_sensor("external_cool_demand_wired", False)
    openamber.set_binary_sensor("sg_ready_block_mode_active_sensor", False)
    openamber.set_binary_sensor("defrost_active_sensor", False)
    openamber.set_number("dhw_setpoint_temperature", 50.0)
    openamber.set_number("dhw_restart_dhw_delta", 5.0)
    openamber.set_sensor("dhw_temperature_tw_sensor", 55.0)
    openamber.set_sensor("temperature_outside_ta", 7.0)
    openamber.set_sensor("current_water_temperature_tc_sensor", 35.0)
    openamber.set_sensor("heat_cool_temperature_tc", 35.0)
    openamber.set_sensor("outlet_temperature_tuo", 35.0)
    openamber.set_sensor("inlet_temperature_tui", 30.0)
    openamber.set_select("thermostat_mode_select", "Extern")
    openamber.set_select("heat_mode_select", "Stooklijn")
    openamber.set_select("cool_mode_select", "Intern setpoint")
    openamber.set_number("cooling_setpoint_number", 18.0)
    openamber.set_select("dhw_compressor_mode", "Maximaal")
    openamber.set_select("heat_compressor_mode", "Maximaal")
    openamber.set_select("cool_compressor_mode", "Maximaal")
    openamber.set_select("dhw_pump_start_mode_select", "Samen met compressor")
    openamber.set_switch("pump_p0_pid_enabled", False)

    openamber.step(ms=50)

    # If compressor is running, or valve is in DHW, advance past COMPRESSOR_MIN_ON_S (600s) + valve switch to allow complete shutdown
    comp = openamber.get_entity("compressor_control_select")
    valve_dhw = openamber.get_entity("three_way_valve_dhw_switch")
    dhw_state = openamber.get_entity("state_machine_state_dhw")
    if (comp is not None and int(comp) > 0) or valve_dhw is True or (dhw_state not in ("Stand-by", "IDLE", None)):
        openamber.advance_time(seconds=640, step_s=20)
        openamber.step(ms=50)

    # If pump is running in PUMP_RUNNING, advance past pump_duration (120s) to allow it to stop
    if openamber.get_entity("internal_pump_active") is True:
        openamber.advance_time(seconds=130, step_s=10)
        openamber.step(ms=50)

    # Allow system to settle into steady HEAT_COOL idle
    for _ in range(12):
        if openamber.get_entity("state_machine_state_main") == "Heat/Cool":
            break
        openamber.advance_time(seconds=10, step_s=5)
        openamber.step(ms=50)
    openamber.step(ms=50)

    assert openamber.get_entity("three_way_valve_dhw_switch") is False, "Valve should be in CV position in idle"
    return openamber

