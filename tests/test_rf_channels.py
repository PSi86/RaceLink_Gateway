"""The channel table, and the header generated from it.

A channel means something operationally: "channel 4" has to name the same
frequency in the host, in the web flasher and on a node, or devices that agree
they are on channel 4 do not hear each other. The table is vendored rather than
re-derived for that reason, and the C header is generated from it rather than
typed a second time.

These tests hold the two ends together: the header cannot drift from the table,
and the table cannot drift into something the firmware would reject.
"""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

TABLE = ROOT / "data" / "rf_channels.json"

# Region -> band, mirroring scripts/generate_rf_channels.py in RaceLink_WLED.
# The gateway publishes the table but generates no header from it: it has no
# settings page to render, only the band lock in the shared rf_config_nvs.h.
REGIONS = {"EU868": ("EU", 868), "US915": ("FCC", 915)}

# The window RfConfigNvs::validate() accepts. A channel outside it would be
# stored and then silently discarded on the next load.
MIN_HZ, MAX_HZ = 863_000_000, 928_000_000


def _table() -> dict:
    return json.loads(TABLE.read_text(encoding="utf-8"))


class RegionTests(unittest.TestCase):
    def test_every_region_has_a_band(self) -> None:
        # Without a band the band lock cannot decide what a region belongs to
        # and would let a 915 board be moved onto it.
        for region in _table()["regions"]:
            with self.subTest(region=region):
                self.assertIn(region, REGIONS)


class ChannelTableTests(unittest.TestCase):
    def test_channels_are_inside_the_window_the_firmware_accepts(self) -> None:
        for region, channels in _table()["regions"].items():
            for channel in channels:
                with self.subTest(region=region, channel=channel["id"]):
                    self.assertGreaterEqual(channel["freq_hz"], MIN_HZ)
                    self.assertLessEqual(channel["freq_hz"], MAX_HZ)

    def test_channels_sit_in_the_band_their_region_declares(self) -> None:
        # A channel filed under US915 that is actually an 868 frequency would
        # be rejected by the band lock on the very boards it is meant for.
        windows = {868: (863_000_000, 870_000_000), 915: (902_000_000, 928_000_000)}
        for region, channels in _table()["regions"].items():
            low, high = windows[REGIONS[region][1]]
            for channel in channels:
                with self.subTest(region=region, channel=channel["id"]):
                    self.assertTrue(low <= channel["freq_hz"] <= high)

    def test_channel_ids_are_unique_and_one_based(self) -> None:
        for region, channels in _table()["regions"].items():
            ids = [c["id"] for c in channels]
            with self.subTest(region=region):
                self.assertEqual(sorted(ids), list(range(1, len(ids) + 1)))

    def test_channels_are_far_enough_apart(self) -> None:
        # Two channels closer than the declared separation are not two
        # channels; a node on one would hear the other.
        separation = _table()["minSeparationHz"]
        for region, channels in _table()["regions"].items():
            freqs = sorted(c["freq_hz"] for c in channels)
            for lower, upper in zip(freqs, freqs[1:]):
                with self.subTest(region=region, pair=(lower, upper)):
                    self.assertGreaterEqual(upper - lower, separation)

    def test_no_two_channels_share_a_phy_setup(self) -> None:
        # The reverse lookup in racelink_rf_channels.h matches on the whole
        # tuple. Two identical entries would make "which channel am I on?"
        # ambiguous, and the answer would depend on table order.
        seen = set()
        for region, channels in _table()["regions"].items():
            for c in channels:
                key = tuple(c[k] for k in sorted(c) if k != "id" and k != "name")
                with self.subTest(region=region, channel=c["id"]):
                    self.assertNotIn(key, seen)
                seen.add(key)

    def test_channel_numbering_is_the_same_in_every_region(self) -> None:
        # The settings page emits one channel dropdown for all regions, on
        # the grounds that only the frequencies differ. If that stops being
        # true the dropdown starts lying and has to be split per region.
        shapes = {
            region: [(c["id"], c["name"]) for c in sorted(channels, key=lambda x: x["id"])]
            for region, channels in _table()["regions"].items()
        }
        self.assertEqual(len(set(map(tuple, shapes.values()))), 1, shapes)


if __name__ == "__main__":
    unittest.main()
