"""Static guards for the gateway's pinned flash layout.

Both the partition table and the platform version are baked into every
published release artifact — the flashing documentation names concrete
offsets, and a factory image freezes the layout permanently. A silent change
to either would invalidate instructions that are already in users' hands, so
they are asserted here rather than left to whatever the toolchain defaults to
on the day CI runs.

The committed CSV is byte-identical to the Arduino ``default_8MB.csv`` this
project has always built with; it is committed rather than referenced by name
so a platform update cannot redefine it underneath us.
"""

from __future__ import annotations

import configparser
import re
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
PLATFORMIO_INI = REPO_ROOT / "platformio.ini"
PARTITION_CSV = REPO_ROOT / "partitions" / "racelink_gateway_8MB.csv"

# The layout every release artifact and every flashing instruction assumes.
EXPECTED_PARTITIONS = (
    ("nvs", "data", "nvs", 0x9000, 0x5000),
    ("otadata", "data", "ota", 0xE000, 0x2000),
    ("app0", "app", "ota_0", 0x10000, 0x330000),
    ("app1", "app", "ota_1", 0x340000, 0x330000),
    ("spiffs", "data", "spiffs", 0x670000, 0x180000),
    ("coredump", "data", "coredump", 0x7F0000, 0x10000),
)

# Where the application image is written. Named separately because it is the
# single number the release workflow, the docs and the web flasher all repeat.
EXPECTED_APP_OFFSET = 0x10000

# A pinned platform looks like "espressif32@7.0.1"; a bare "espressif32" means
# CI installs whatever is newest on the day it runs.
PINNED_PLATFORM_PATTERN = re.compile(r"^espressif32@\d+\.\d+\.\d+$")

PARTITIONS_OPTION = "board_build.partitions"


def _load_config() -> configparser.ConfigParser:
    parser = configparser.ConfigParser(inline_comment_prefixes=(";",))
    parser.read(PLATFORMIO_INI, encoding="utf-8")
    return parser


def _resolve(parser: configparser.ConfigParser, value: str) -> str:
    """Resolve a single ``${section.option}`` reference, if present."""
    match = re.fullmatch(r"\$\{([^.}]+)\.([^}]+)\}", value.strip())
    if not match:
        return value.strip()
    section, option = match.groups()
    return parser.get(section, option).strip()


def _parse_partition_csv(path: Path) -> list[tuple[str, str, str, int, int]]:
    rows: list[tuple[str, str, str, int, int]] = []
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        fields = [field.strip() for field in line.split(",")]
        if len(fields) < 5:
            raise AssertionError(f"Malformed partition row: {raw_line!r}")
        name, ptype, subtype, offset, size = fields[:5]
        rows.append((name, ptype, subtype, int(offset, 0), int(size, 0)))
    return rows


class PartitionTableTests(unittest.TestCase):
    def test_committed_table_matches_the_released_layout(self) -> None:
        self.assertEqual(_parse_partition_csv(PARTITION_CSV), list(EXPECTED_PARTITIONS))

    def test_application_partition_starts_at_the_documented_offset(self) -> None:
        table = {row[0]: row for row in _parse_partition_csv(PARTITION_CSV)}
        self.assertIn("app0", table, "Partition table has no app0 entry")
        self.assertEqual(table["app0"][3], EXPECTED_APP_OFFSET)

    def test_table_entries_do_not_overlap_and_stay_within_8mb(self) -> None:
        rows = sorted(_parse_partition_csv(PARTITION_CSV), key=lambda row: row[3])
        for previous, current in zip(rows, rows[1:]):
            end = previous[3] + previous[4]
            self.assertLessEqual(
                end,
                current[3],
                f"{previous[0]} overlaps {current[0]}",
            )
        last = rows[-1]
        self.assertLessEqual(last[3] + last[4], 8 * 1024 * 1024)


class PlatformIniTests(unittest.TestCase):
    def setUp(self) -> None:
        self.parser = _load_config()
        self.envs = [s for s in self.parser.sections() if s.startswith("env:")]

    def test_release_environments_exist(self) -> None:
        self.assertTrue(self.envs, "platformio.ini declares no build environments")

    def test_every_environment_pins_the_platform_version(self) -> None:
        for env in self.envs:
            with self.subTest(env=env):
                platform = _resolve(self.parser, self.parser.get(env, "platform"))
                self.assertRegex(platform, PINNED_PLATFORM_PATTERN)

    def test_every_environment_uses_the_committed_partition_table(self) -> None:
        expected = PARTITION_CSV.relative_to(REPO_ROOT).as_posix()
        for env in self.envs:
            with self.subTest(env=env):
                self.assertTrue(
                    self.parser.has_option(env, PARTITIONS_OPTION),
                    f"{env} does not set {PARTITIONS_OPTION}",
                )
                configured = _resolve(self.parser, self.parser.get(env, PARTITIONS_OPTION))
                self.assertEqual(configured.replace("\\", "/"), expected)

    def test_committed_partition_table_exists(self) -> None:
        self.assertTrue(PARTITION_CSV.is_file(), f"Missing {PARTITION_CSV}")


if __name__ == "__main__":
    unittest.main()
