#!/usr/bin/env python3
"""Static memory report for OpenAmber firmware builds.

Reads the section headers of an ESP-IDF firmware ELF and sums the sections
that end up in each memory region. Every byte of static internal RAM
(.data/.bss/IRAM) is a byte the heap cannot use at runtime, so a PR that grows
these regions lowers heap_free and usually heap_max (largest free block) on the
device.

Subcommands:
  measure  ELF -> JSON with per-region sizes for one build
  merge    several measure JSONs -> one report JSON
  compare  base report + PR report -> Markdown for the PR comment

Only the Python standard library is used, so this runs on any CI runner.
"""

from __future__ import annotations

import argparse
import json
import re
import struct
import sys
from pathlib import Path

COMMENT_MARKER = "<!-- openamber-memory-report -->"

# Region -> section name prefixes, using ESP-IDF's linker section names
# (the same on ESP32, ESP32-S3 and ESP32-P4).
REGIONS = {
    "dram_data": (".dram0.data", ".data"),
    "dram_bss": (".dram0.bss", ".bss", ".noinit", ".dram0.heap_start"),
    "iram": (".iram0.", ".iram"),
    "rtc": (".rtc", ".lp_", ".rtc_noinit", ".rtc_reserved"),
    "tcm": (".tcm",),
    "psram": (".ext_ram",),
    "flash_text": (".flash.text", ".flash_text"),
    "flash_rodata": (".flash.", ".flash_rodata"),
}

# Placeholder sections the linker scripts use to reserve address space (e.g.
# .dram0.dummy mirrors IRAM on the S3, .ext_ram.dummy skips the mapped flash).
# They hold no data of the application, counting them would double count.
IGNORED_SUBSTRINGS = ("dummy", "noload")

SHF_ALLOC = 0x2

# Metrics shown in the PR comment, in order: key, label.
METRICS = [
    ("internal_ram", "Static internal RAM (DRAM + IRAM)"),
    ("dram_data", "DRAM .data"),
    ("dram_bss", "DRAM .bss"),
    ("iram", "IRAM"),
    ("psram", "PSRAM (static .bss)"),
    ("psram_flash_copy", "PSRAM (flash copied for XIP)"),
    ("flash_text", "Flash code"),
    ("flash_rodata", "Flash rodata"),
    ("bin_size", "Firmware .bin"),
]


def read_sections(elf_path: Path) -> list[tuple[str, int, int, int]]:
    """Return (name, type, flags, size) for every section in a 32-bit ELF."""
    data = elf_path.read_bytes()
    if data[:4] != b"\x7fELF":
        raise ValueError(f"{elf_path} is not an ELF file")
    if data[4] != 1:
        raise ValueError(f"{elf_path} is not a 32-bit ELF (ESP32 targets are 32-bit)")
    endian = "<" if data[5] == 1 else ">"

    shoff = struct.unpack_from(endian + "I", data, 0x20)[0]
    shentsize, shnum, shstrndx = struct.unpack_from(endian + "HHH", data, 0x2E)

    raw = []
    for i in range(shnum):
        off = shoff + i * shentsize
        name, typ, flags, _addr, offset, size = struct.unpack_from(endian + "IIIIII", data, off)
        raw.append((name, typ, flags, offset, size))

    strtab_offset = raw[shstrndx][3]

    def name_at(idx: int) -> str:
        end = data.index(b"\0", strtab_offset + idx)
        return data[strtab_offset + idx:end].decode("ascii", "replace")

    return [(name_at(n), typ, flags, size) for (n, typ, flags, _o, size) in raw]


def classify(name: str) -> str | None:
    for region, prefixes in REGIONS.items():
        if name.startswith(prefixes):
            return region
    return None


def read_sdkconfig(path: Path | None) -> set[str]:
    """Return the enabled options from an ESP-IDF config/sdkconfig.h."""
    if not path or not path.exists():
        return set()
    enabled = set()
    for line in path.read_text(errors="replace").splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[0] == "#define" and parts[2] == "1":
            enabled.add(parts[1])
    return enabled


def flash_copied_to_psram(config: set[str]) -> tuple[bool, bool]:
    """Whether flash code / rodata are copied into PSRAM at boot.

    The ELF cannot tell: with XIP from PSRAM the MMU maps the same virtual
    flash addresses to PSRAM pages, so the section addresses do not change.
    Only the build configuration says where the bytes end up.
    """
    xip = "CONFIG_SPIRAM_XIP_FROM_PSRAM" in config
    return (
        xip or "CONFIG_SPIRAM_FETCH_INSTRUCTIONS" in config,
        xip or "CONFIG_SPIRAM_RODATA" in config,
    )


def measure(elf_path: Path, bin_path: Path | None, sdkconfig: Path | None = None) -> dict:
    regions = {key: 0 for key in REGIONS}
    other: dict[str, int] = {}
    for name, typ, flags, size in read_sections(elf_path):
        if not (flags & SHF_ALLOC) or size == 0 or any(x in name for x in IGNORED_SUBSTRINGS):
            continue
        region = classify(name)
        if region is None:
            other[name] = size
        else:
            regions[region] += size

    result = dict(regions)
    result["internal_ram"] = regions["dram_data"] + regions["dram_bss"] + regions["iram"]
    # Flash sections stay counted as flash (that is where they are stored);
    # psram_flash_copy is the PSRAM they additionally occupy at runtime.
    text_in_psram, rodata_in_psram = flash_copied_to_psram(read_sdkconfig(sdkconfig))
    result["psram_flash_copy"] = (
        (regions["flash_text"] if text_in_psram else 0)
        + (regions["flash_rodata"] if rodata_in_psram else 0)
    )
    result["other_sections"] = other
    if bin_path and bin_path.exists():
        result["bin_size"] = bin_path.stat().st_size
    return result


def cmd_measure(args: argparse.Namespace) -> int:
    sdkconfig = Path(args.sdkconfig) if args.sdkconfig else Path(args.elf).parent / "config" / "sdkconfig.h"
    report = measure(Path(args.elf), Path(args.bin) if args.bin else None, sdkconfig)
    report["name"] = args.name
    Path(args.output).write_text(json.dumps(report, indent=2, sort_keys=True))
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


def cmd_merge(args: argparse.Namespace) -> int:
    builds = {}
    for path in args.inputs:
        entry = json.loads(Path(path).read_text())
        builds[entry["name"]] = entry
    out = {"commit": args.commit, "ref": args.ref, "pr": args.pr, "builds": builds}
    Path(args.output).write_text(json.dumps(out, indent=2, sort_keys=True))
    return 0


def fmt_bytes(value: int | None) -> str:
    if value is None:
        return "–"
    if abs(value) >= 1024:
        return f"{value / 1024:,.1f} kB"
    return f"{value:,} B"


def fmt_delta(base: int | None, pr: int | None, threshold: int) -> str:
    if base is None or pr is None:
        return "–"
    delta = pr - base
    if delta == 0:
        return "0"
    sign = "+" if delta > 0 else "−"
    text = f"{sign}{fmt_bytes(abs(delta))}"
    if base:
        text += f" ({sign}{abs(delta) / base * 100:.2f}%)"
    if delta >= threshold:
        text = f"🔺 **{text}**"
    elif delta <= -threshold:
        text = f"🟢 {text}"
    return text


def render(base: dict | None, pr: dict, threshold: int) -> str:
    lines = [COMMENT_MARKER, "## 🧠 Memory impact", ""]
    if base is None:
        lines += [
            "_No baseline for the target branch yet, so only the PR values are shown. "
            "The baseline is built on every push to the target branch._",
            "",
        ]
    else:
        lines += [
            f"Compared to `{base.get('ref', '?')}` @ `{str(base.get('commit', '?'))[:7]}`.",
            "",
        ]

    grew = False
    base_builds = (base or {}).get("builds", {})
    for name in sorted(pr["builds"]):
        cur = pr["builds"][name]
        old = base_builds.get(name)
        delta_internal = (cur.get("internal_ram", 0) - old.get("internal_ram", 0)) if old else 0
        grew |= delta_internal >= threshold
        lines += [
            f"### `{name}`",
            "",
            "| Region | Base | PR | Δ |",
            "| --- | ---: | ---: | ---: |",
        ]
        for key, label in METRICS:
            b = old.get(key) if old else None
            p = cur.get(key)
            if not b and not p:
                continue
            row_label = f"**{label}**" if key == "internal_ram" else label
            lines.append(f"| {row_label} | {fmt_bytes(b)} | {fmt_bytes(p)} | {fmt_delta(b, p, threshold)} |")
        lines.append("")

    if grew:
        lines += [
            f"> [!WARNING]\n> Static internal RAM grew by {fmt_bytes(threshold)} or more. "
            "Each byte here is taken from the heap: expect `heap_free` and possibly "
            "`heap_max` (largest free block) to drop by at least the same amount on the device.",
            "",
        ]

    lines += [
        "<details><summary>What is measured?</summary>",
        "",
        "Sizes come from the section headers of the firmware ELF built in this PR. "
        "Static internal RAM (`.data`, `.bss`, IRAM) is reserved at boot, so it directly "
        "lowers the heap the firmware can use. With `CONFIG_SPIRAM_XIP_FROM_PSRAM` (or "
        "`SPIRAM_FETCH_INSTRUCTIONS`/`SPIRAM_RODATA`) flash code/rodata is copied into PSRAM "
        "at boot, shown as *PSRAM (flash copied for XIP)*; that lowers the free PSRAM heap. "
        "This does not yet measure runtime "
        "`heap_free`, `heap_max` or `heap_frag`: those depend on what the firmware allocates "
        "while running and need a real device.",
        "",
        "</details>",
    ]
    return "\n".join(lines) + "\n"


_SAFE_NAME = re.compile(r"^[A-Za-z0-9._/-]{1,100}$")
_SAFE_COMMIT = re.compile(r"^[0-9a-f]{0,40}$")
_NUMERIC_KEYS = [key for key, _ in METRICS] + list(REGIONS)


def sanitize(report: dict) -> dict:
    """Keep only well-formed fields.

    The PR report is produced by the PR's own workflow run, which for a fork is
    untrusted, so nothing from it reaches the comment unless it is a plain
    number or a strictly matched name.
    """
    ref = str(report.get("ref", ""))
    commit = str(report.get("commit", ""))
    builds = {}
    for name, entry in dict(report.get("builds", {})).items():
        if not _SAFE_NAME.match(str(name)) or not isinstance(entry, dict):
            continue
        builds[name] = {k: int(entry[k]) for k in _NUMERIC_KEYS if isinstance(entry.get(k), int)}
    return {
        "ref": ref if _SAFE_NAME.match(ref) else "?",
        "commit": commit if _SAFE_COMMIT.match(commit) else "?",
        "builds": builds,
    }


def cmd_compare(args: argparse.Namespace) -> int:
    pr = sanitize(json.loads(Path(args.pr).read_text()))
    base = None
    if args.base and Path(args.base).exists():
        base = sanitize(json.loads(Path(args.base).read_text()))
    Path(args.output).write_text(render(base, pr, args.threshold), encoding="utf-8")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("measure", help="measure one firmware ELF")
    p.add_argument("--elf", required=True)
    p.add_argument("--bin", help="factory/ota .bin for the image size")
    p.add_argument("--name", required=True, help="build name, e.g. openamber-leejow")
    p.add_argument("--sdkconfig", help="config/sdkconfig.h of the build (default: next to the ELF)")
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_measure)

    p = sub.add_parser("merge", help="merge measure outputs into one report")
    p.add_argument("--commit", default="")
    p.add_argument("--ref", default="")
    p.add_argument("--pr", type=int, default=None, help="pull request number, for the comment workflow")
    p.add_argument("--output", required=True)
    p.add_argument("inputs", nargs="+")
    p.set_defaults(func=cmd_merge)

    p = sub.add_parser("compare", help="render the PR comment")
    p.add_argument("--base", help="baseline report JSON (optional)")
    p.add_argument("--pr", required=True)
    p.add_argument("--threshold", type=int, default=1024, help="bytes before a change is highlighted")
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_compare)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
