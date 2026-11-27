"""Replay of measured Tuo/Tui/Tc series from a real unit (tests/data/*.csv).

The CSV files come from the analytics data dump of a running unit: one row per snapshot (every 5 min, plus a
snapshot right after every state machine transition). replay_frames() turns them into frames every 15 s, the rate
at which the inside unit is read: temperatures are interpolated linearly between two rows, the defrost flag holds
its value until the next row. Columns:

    time_local      HH:MM:SS
    defrost_2118    1 while register 2118 (defrost) was set
    compressor_hz   reported compressor frequency
    tuo, tui, tc    temperatures in °C
    state_heat_cool state of the heat/cool controller at that time (for reference only)

The replay is open loop: the recorded temperatures do not react to what the firmware does. That is enough to check
that measured conditions do not trigger a safety check.
"""

import csv
import os
from typing import List

from temperature_profiles import DEFROST_STANDBY_HZ, READ_INTERVAL_S, Frame

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
# Above this frequency the outdoor unit is still in the defrost pre-phase (standby frequency + margin).
PREPHASE_MIN_HZ = DEFROST_STANDBY_HZ + 5.0


def _seconds(hhmmss: str) -> int:
    h, m, s = (int(part) for part in hhmmss.split(":"))
    return h * 3600 + m * 60 + s


def load_rows(name: str) -> List[dict]:
    with open(os.path.join(DATA_DIR, name), newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    for row in rows:
        row["t"] = _seconds(row["time_local"])
        row["defrost"] = row["defrost_2118"] == "1"
        for key in ("compressor_hz", "tuo", "tui", "tc"):
            row[key] = float(row[key])
    return rows


def replay_frames(name: str, hold_s: int = 0) -> List[Frame]:
    """Frames every READ_INTERVAL_S from the first to the last row, plus hold_s seconds of the last values."""
    rows = load_rows(name)
    start = rows[0]["t"]
    end = rows[-1]["t"]
    frames: List[Frame] = []
    t = start
    index = 0
    while t <= end + hold_s:
        while index + 1 < len(rows) and rows[index + 1]["t"] <= t:
            index += 1
        row = rows[index]
        nxt = rows[index + 1] if index + 1 < len(rows) else row
        span = nxt["t"] - row["t"]
        fraction = 0.0 if span <= 0 else min(1.0, (t - row["t"]) / span)

        def value(key):
            return round(row[key] + (nxt[key] - row[key]) * fraction, 2)

        # During the pre-phase report the recorded frequency, so the firmware keeps heating until the actual
        # defrost; otherwise let the mock follow the requested level (36 Hz while 2118 is set).
        prephase = row["defrost"] and row["compressor_hz"] > PREPHASE_MIN_HZ
        frames.append(Frame(
            t_s=t - start,
            tuo=value("tuo"),
            tui=value("tui"),
            tc=value("tc"),
            defrost=row["defrost"],
            frequency_hz=row["compressor_hz"] if prephase else -1.0,
            label=f"{name} {row['time_local']}",
        ))
        t += READ_INTERVAL_S
    return frames
