"""Shared helpers for flow tests.

They advance virtual time until the controller reaches a state, instead of assuming a fixed
timeline. The time at which the pump interval elapses depends on when the previous test last
stopped the pump, so hard-coded "900s + 130s + 190s" sequences land in different states
depending on test order.
"""


def offset_s(openamber):
    """Current virtual time offset in seconds."""
    return openamber.get_time().get("offset_ms", 0) / 1000.0


def now_s(openamber):
    """Simulator time in seconds: millis() of the simulator, which is the real time since its start plus the
    virtual time offset. Use this to measure durations; offset_s() leaves out the real time that passes while a
    test runs (about 5 % extra when stepping in 5 s steps)."""
    return openamber.get_time().get("millis", 0) / 1000.0


def elapsed_s(start_s, end_s):
    """Seconds from start_s to end_s (both from now_s), also across the 32-bit millis() rollover."""
    return (end_s - start_s) % (2 ** 32 / 1000.0)


def advance_until_state(openamber, entity, wanted, max_s, step=10):
    """Advance virtual time in `step`-second increments until `entity` equals `wanted`.

    Returns True when the state was reached within `max_s` seconds.
    """
    waited = 0
    while True:
        if openamber.get_entity(entity) == wanted:
            return True
        if waited >= max_s:
            return False
        cur_step = min(step, max_s - waited)
        openamber.advance_time(seconds=cur_step, step_s=cur_step)
        openamber.step(ms=20)
        waited += cur_step


def set_outside_temperature(openamber, ta):
    """Set Ta and restart the averaged Ta used by the heating curves at the same value.

    The average is an exponential_moving_average sensor with a 60 s throttle on its output: after the reset
    the new value is published at the latest 60 s later, so this advances until it shows. Right after a
    defrost the Ta hold (5 min) drops values first, so it may take up to about 6 min.
    """
    openamber.set_switch("mock_reset_outside_temperature_average", True)
    openamber.step(ms=20)
    openamber.set_sensor("temperature_outside_ta", ta)
    openamber.step(ms=20)
    for _ in range(80):
        value = openamber.get_entity("temperature_outside_ta_avg_1h")
        if value is not None and abs(float(value) - ta) < 0.01:
            return
        openamber.advance_time(seconds=5, step_s=5)
        openamber.step(ms=20)
    raise AssertionError(f"Average Ta did not restart at {ta} (got {openamber.get_entity('temperature_outside_ta_avg_1h')})")


def start_space_heating(openamber, tc=28.0, target=35.0, use_curve=False):
    """Start space heating and advance until the heat/cool controller is in 'Compressor running'.

    use_curve=True keeps the 'Stooklijn' heat mode, so the setpoint comes from the heating curves.
    Otherwise 'Extern setpoint' with manual_setpoint = target is used.
    """
    openamber.set_select("thermostat_mode_select", "Extern")
    if not use_curve:
        openamber.set_select("heat_mode_select", "Extern setpoint")
        openamber.set_number("manual_setpoint", target)
        openamber.set_climate("pid_heat_temperature_control", target_temperature=target)
    openamber.set_select("heat_compressor_mode", "Maximaal")
    openamber.set_number("compressor_start_delta_heating", 2.0)
    openamber.set_number("backup_heater_degmin_threshold", 9999.0)
    openamber.set_sensor("heat_cool_temperature_tc", tc)
    openamber.set_sensor("outlet_temperature_tuo", tc)
    openamber.set_sensor("inlet_temperature_tui", tc - 2.0)
    openamber.set_switch("heat_demand_switch", True)
    openamber.step(ms=50)
    assert advance_until_state(openamber, "state_machine_state_heat_cool", "Compressor running", max_s=1800), (
        f"Heating never reached 'Compressor running' (state: {openamber.get_entity('state_machine_state_heat_cool')})"
    )


class FlowMonitor:
    """Advances virtual time in small steps and checks, after every step, rules that must hold in every flow.

    The rules describe what the hardware needs, independent of how the controller gets there:
    - the 3-way valve only moves while the compressor, pump P0 and the backup heater are off;
    - the compressor is only requested with flow (pump P0 running) and a heating/cooling working mode;
    - the compressor is not requested again within the minimum off time, and not stopped within the
      minimum on time (unless the flow allows a short run, see `allow_short_run`);
    - the backup heater only runs with flow;
    - pump P1 and the DHW pump match the valve position;
    - only the controller that owns the main state is active.
    """

    MIN_OFF_S = 200
    MIN_ON_S = 600

    def __init__(self, openamber, allow_short_run=False):
        self.openamber = openamber
        self.allow_short_run = allow_short_run
        self.previous = None
        self.compressor_stopped_at = None
        self.compressor_running_since = None
        self.history = []

    def snapshot(self):
        get = self.openamber.get_entity
        return {
            "t": now_s(self.openamber),
            "main": get("state_machine_state_main"),
            "heat_cool": get("state_machine_state_heat_cool"),
            "dhw": get("state_machine_state_dhw"),
            "compressor": int(get("compressor_control_select") or 0),
            "frequency": float(get("current_compressor_frequency") or 0),
            "pump": get("internal_pump_active") is True,
            "valve_dhw": get("three_way_valve_dhw_switch") is True,
            "working_mode": get("working_mode_switch"),
            "backup": get("backup_heater_relay") is True or get("external_backup_heating_relay") is True,
            "dhw_pump": get("dhw_pump_relay_switch") is True,
            "p1": get("pump_p1_relay_switch") is True,
        }

    def _fail(self, message, now):
        trail = "\n".join(
            f"  t={s['t']:.0f} main={s['main']} hc={s['heat_cool']} dhw={s['dhw']} comp={s['compressor']} "
            f"pump={s['pump']} valve_dhw={s['valve_dhw']} mode={s['working_mode']} backup={s['backup']}"
            for s in self.history[-12:]
        )
        raise AssertionError(f"{message} at t={now['t']:.0f}s\n{trail}")

    def check(self):
        now = self.snapshot()
        self.history.append(now)
        prev = self.previous
        self.previous = now

        if now["compressor"] > 0:
            if not now["pump"]:
                self._fail("Compressor requested without flow (pump P0 not running)", now)
            if now["working_mode"] not in ("Verwarmen", "Koelen"):
                self._fail(f"Compressor requested in working mode {now['working_mode']}", now)
            if now["valve_dhw"] and now["working_mode"] != "Verwarmen":
                self._fail("Compressor requested on the DHW circuit while not in heating mode", now)
        if now["backup"] and not now["pump"]:
            self._fail("Backup heater on without flow (pump P0 not running)", now)
        if now["valve_dhw"] and now["p1"]:
            self._fail("Pump P1 runs while the 3-way valve is on DHW", now)
        if now["dhw_pump"] and not now["valve_dhw"]:
            self._fail("DHW pump runs while the 3-way valve is on heating/cooling", now)
        if now["main"] == "DHW" and now["heat_cool"] != "Idle":
            self._fail("Heat/cool controller active while the main state is DHW", now)
        if now["main"] == "Heat/Cool" and now["dhw"] != "Idle":
            self._fail("DHW controller active while the main state is Heat/Cool", now)

        if prev is None:
            if now["compressor"] > 0:
                self.compressor_running_since = now["t"]
            return now

        if prev["valve_dhw"] != now["valve_dhw"]:
            for sample in (prev, now):
                if sample["compressor"] > 0 or sample["frequency"] > 0:
                    self._fail("3-way valve switched while the compressor runs", now)
                if sample["backup"]:
                    self._fail("3-way valve switched while the backup heater is on", now)
            if now["pump"]:
                self._fail("3-way valve switched while pump P0 runs", now)

        if prev["compressor"] == 0 and now["compressor"] > 0:
            if self.compressor_stopped_at is not None and elapsed_s(self.compressor_stopped_at, now["t"]) < self.MIN_OFF_S:
                self._fail(
                    f"Compressor requested {elapsed_s(self.compressor_stopped_at, now['t']):.0f}s after it stopped "
                    f"(minimum off time {self.MIN_OFF_S}s)", now)
            self.compressor_running_since = now["t"]
        if prev["compressor"] > 0 and now["compressor"] == 0:
            self.compressor_stopped_at = now["t"]
            ran = elapsed_s(self.compressor_running_since, now["t"]) if self.compressor_running_since is not None else None
            if ran is not None and ran < self.MIN_ON_S and not self.allow_short_run:
                self._fail(f"Compressor stopped after {ran:.0f}s (minimum on time {self.MIN_ON_S}s)", now)
            self.compressor_running_since = None
        return now

    def advance(self, seconds, step=5):
        """Advance `seconds` of virtual time, checking the rules after every `step` seconds."""
        waited = 0
        while waited < seconds:
            self.openamber.advance_time(seconds=step, step_s=step)
            self.openamber.step(ms=20)
            self.check()
            waited += step

    def advance_until(self, entity, wanted, max_s, step=5):
        """Advance until `entity` equals `wanted`, checking the rules after every step. Returns True when reached."""
        waited = 0
        while True:
            if self.openamber.get_entity(entity) == wanted:
                return True
            if waited >= max_s:
                return False
            self.advance(step, step=step)
            waited += step
