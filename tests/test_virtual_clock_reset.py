"""Every test starts at the same simulated wall clock time.

clean_system sets the clock to the fixed start (Friday 2026-10-09 12:00 UTC), so time-of-day and weekday logic does not
depend on how much virtual time earlier tests used. Setting the clock leaves millis() alone, so controller timers keep
running normally.
"""

START_EPOCH = 1791547200


def test_clean_system_starts_at_fixed_clock(clean_system):
    epoch = clean_system.get_time()["epoch"]
    assert START_EPOCH <= epoch < START_EPOCH + 600, f"Test must start near Friday 12:00 UTC, got epoch {epoch}"


def test_set_clock_resets_wall_clock_but_not_millis(clean_system):
    openamber = clean_system
    openamber.advance_time(seconds=16 * 3600, step_s=600)  # well past the end of a 10:00-17:00 window
    before = openamber.get_time()
    assert before["epoch"] >= START_EPOCH + 16 * 3600

    assert openamber.set_clock()
    after = openamber.get_time()
    assert START_EPOCH <= after["epoch"] < START_EPOCH + 60, "set_clock must bring the wall clock back to the start"
    assert after["millis"] >= before["millis"], "set_clock must not move millis() backwards"
