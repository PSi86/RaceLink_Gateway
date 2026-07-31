"""Tests for the release-artifact helpers.

The guardrails that matter here are the ones whose failure mode is a published
artifact that flashes cleanly and never boots: an unmapped SoC, a bootloader
offset taken from the wrong source, or an application offset that no longer
matches the partition table shipped inside the same image.
"""

from __future__ import annotations

import struct
import unittest

from scripts.release_artifacts import (
    APP_KIND,
    BOOTLOADER_OFFSETS,
    CHIP_NAMES,
    FACTORY_KIND,
    FILENAME_TOKENS,
    application_offset,
    artifact_name,
    bootloader_offset_for_chip,
    device_type,
    flash_images_from_metadata,
    manifest_name,
    merge_command,
    parse_partition_table,
    read_chip_name,
)


def _esp_image(chip_id: int, magic: int = 0xE9) -> bytes:
    header = bytearray(24)
    header[0] = magic
    header[1] = 5  # segment count
    struct.pack_into("<H", header, 12, chip_id)
    return bytes(header)


def _partition_entry(name: str, ptype: int, subtype: int, offset: int, size: int) -> bytes:
    entry = bytearray(32)
    entry[0:2] = b"\xaa\x50"
    entry[2] = ptype
    entry[3] = subtype
    struct.pack_into("<II", entry, 4, offset, size)
    entry[12:28] = name.encode("utf-8").ljust(16, b"\x00")
    return bytes(entry)


def _partition_table() -> bytes:
    return b"".join(
        [
            _partition_entry("nvs", 0x01, 0x02, 0x9000, 0x5000),
            _partition_entry("otadata", 0x01, 0x00, 0xE000, 0x2000),
            _partition_entry("app0", 0x00, 0x10, 0x10000, 0x330000),
            _partition_entry("app1", 0x00, 0x11, 0x340000, 0x330000),
            _partition_entry("spiffs", 0x01, 0x82, 0x670000, 0x180000),
        ]
    ) + b"\xff" * 32


class ChipDetectionTests(unittest.TestCase):
    def test_reads_the_chip_from_the_image_header(self) -> None:
        self.assertEqual(read_chip_name(_esp_image(0x0009)), "esp32s3")
        self.assertEqual(read_chip_name(_esp_image(0x0002)), "esp32s2")
        self.assertEqual(read_chip_name(_esp_image(0x0000)), "esp32")

    def test_rejects_a_non_esp_image(self) -> None:
        with self.assertRaises(ValueError):
            read_chip_name(_esp_image(0x0009, magic=0x00))

    def test_rejects_an_unmapped_chip_id(self) -> None:
        with self.assertRaises(ValueError):
            read_chip_name(_esp_image(0x00FF))

    def test_every_known_chip_has_a_bootloader_offset(self) -> None:
        for chip in CHIP_NAMES.values():
            with self.subTest(chip=chip):
                self.assertIn(chip, BOOTLOADER_OFFSETS)

    def test_bootloader_offsets_match_the_documented_layout(self) -> None:
        # The two families that differ; getting these wrong is the classic
        # unbootable-factory-image bug.
        self.assertEqual(bootloader_offset_for_chip("esp32"), 0x1000)
        self.assertEqual(bootloader_offset_for_chip("esp32s2"), 0x1000)
        self.assertEqual(bootloader_offset_for_chip("esp32s3"), 0x0000)
        self.assertEqual(bootloader_offset_for_chip("esp32c3"), 0x0000)

    def test_rejects_an_unmapped_chip_name(self) -> None:
        with self.assertRaises(ValueError):
            bootloader_offset_for_chip("esp32p4")


class PartitionTableTests(unittest.TestCase):
    def test_parses_entries_until_the_table_ends(self) -> None:
        partitions = parse_partition_table(_partition_table())

        self.assertEqual([p.name for p in partitions], ["nvs", "otadata", "app0", "app1", "spiffs"])
        self.assertEqual(partitions[0].offset, 0x9000)

    def test_application_offset_is_the_first_app_partition(self) -> None:
        self.assertEqual(application_offset(parse_partition_table(_partition_table())), 0x10000)

    def test_rejects_a_table_without_an_application(self) -> None:
        data = _partition_entry("nvs", 0x01, 0x02, 0x9000, 0x5000)
        with self.assertRaises(ValueError):
            application_offset(parse_partition_table(data))

    def test_rejects_an_empty_table(self) -> None:
        with self.assertRaises(ValueError):
            parse_partition_table(b"\xff" * 32)


class MetadataTests(unittest.TestCase):
    METADATA = {
        "WirelessStickV3-ESP32S3": {
            "defines": ["PLATFORMIO=60119", "DEV_TYPE=1", 'DEV_TYPE_STR="RaceLink_Gateway_v4"'],
            "extra": {
                "flash_images": [
                    {"offset": "0x0000", "path": "/build/bootloader.bin"},
                    {"offset": "0x8000", "path": "/build/partitions.bin"},
                    {"offset": "0xe000", "path": "/framework/boot_app0.bin"},
                ]
            },
        }
    }

    def test_reads_flash_images_for_an_environment(self) -> None:
        images = flash_images_from_metadata(self.METADATA, "WirelessStickV3-ESP32S3")

        self.assertEqual([image.offset for image in images], [0x0, 0x8000, 0xE000])

    def test_accepts_single_environment_idedata(self) -> None:
        single = self.METADATA["WirelessStickV3-ESP32S3"]

        images = flash_images_from_metadata(single, "WirelessStickV3-ESP32S3")

        self.assertEqual(len(images), 3)

    def test_rejects_metadata_without_flash_images(self) -> None:
        with self.assertRaises(ValueError):
            flash_images_from_metadata({"env": {"extra": {}}}, "env")

    def test_reads_the_device_type_from_the_build_defines(self) -> None:
        defines = self.METADATA["WirelessStickV3-ESP32S3"]["defines"]

        self.assertEqual(device_type(defines), 1)

    def test_device_type_is_optional(self) -> None:
        self.assertIsNone(device_type(["PLATFORMIO=60119"]))


class NamingTests(unittest.TestCase):
    def test_the_two_published_names(self) -> None:
        self.assertEqual(
            artifact_name(version="0.1.7", env="WirelessStickV3-ESP32S3", kind=APP_KIND),
            "WirelessStickV3-ESP32S3-0.1.7-ota.bin",
        )
        self.assertEqual(
            artifact_name(version="0.1.7", env="WirelessStickV3-ESP32S3", kind=FACTORY_KIND),
            "WirelessStickV3-ESP32S3-0.1.7-usbflash.bin",
        )

    def test_the_two_names_cannot_be_confused(self) -> None:
        # Forcing the factory image through an OTA path is the one mistake with
        # a real cost, so neither name may contain the other -- a glob or a
        # glance for one must never land on the other.
        app = artifact_name(version="0.1.7", env="WirelessStickV3-ESP32S3", kind=APP_KIND)
        factory = artifact_name(version="0.1.7", env="WirelessStickV3-ESP32S3", kind=FACTORY_KIND)

        self.assertNotIn(FILENAME_TOKENS[APP_KIND], factory)
        self.assertNotIn(FILENAME_TOKENS[FACTORY_KIND], app)

    def test_the_environment_is_spelled_out_verbatim(self) -> None:
        # The env is the key the sidecar, the build configuration and the web
        # flasher's data/devices.json already agree on. Abbreviating it here
        # would introduce a fourth spelling, and an abbreviation rule is what
        # starts colliding the day a target is added.
        for env in ("RaceLink_Node_v3_s2_llcc68_epaper", "WirelessStickV3-ESP32S3"):
            with self.subTest(env=env):
                name = artifact_name(version="0.1.9", env=env, kind=APP_KIND)
                self.assertTrue(name.startswith(f"{env}-"))

    def test_the_name_carries_no_product_device_type_or_upstream_ref(self) -> None:
        name = artifact_name(version="0.1.9", env="RaceLink_Node_v4_s3_llcc68", kind=APP_KIND)

        self.assertEqual(name, "RaceLink_Node_v4_s3_llcc68-0.1.9-ota.bin")
        # All three are in the assets.json sidecar. Repeating them in the
        # filename was about half its length, for fields nobody was meant to
        # read off a filename in the first place.
        for absent in ("RaceLink_WLED-", "TYPE12", "wled_v"):
            with self.subTest(absent=absent):
                self.assertNotIn(absent, name)

    def test_the_version_stays_in_the_name(self) -> None:
        # The tag identifies it on the release page; the filename has to keep
        # doing so once it is sitting in somebody's Downloads folder.
        self.assertIn("-0.1.9-", artifact_name(version="0.1.9", env="env", kind=APP_KIND))

    def test_rejects_a_kind_without_a_filename_token(self) -> None:
        # A newly published kind has to name itself deliberately rather than
        # inherit whatever its sidecar value happens to be.
        with self.assertRaises(ValueError):
            artifact_name(version="0.1.7", env="WirelessStickV3-ESP32S3", kind="bootloader")

    def test_sidecar_kinds_are_the_machine_contract(self) -> None:
        # The web flasher selects the image it mirrors on kind == "factory".
        # These values are independent of the filename tokens above and must
        # not move when a name changes.
        self.assertEqual(APP_KIND, "app")
        self.assertEqual(FACTORY_KIND, "factory")

    def test_sidecar_name(self) -> None:
        self.assertEqual(
            manifest_name("RaceLink_Gateway", "0.1.7"), "RaceLink_Gateway-0.1.7-assets.json"
        )


class MergeCommandTests(unittest.TestCase):
    def test_orders_images_by_offset(self) -> None:
        argv = merge_command(
            chip="esp32s3",
            output="factory.bin",
            images=[(0x10000, "firmware.bin"), (0x0, "bootloader.bin"), (0x8000, "partitions.bin")],
        )

        self.assertEqual(
            argv[argv.index("-o") + 2 :],
            ["0x0", "bootloader.bin", "0x8000", "partitions.bin", "0x10000", "firmware.bin"],
        )

    def test_names_the_chip_and_uses_the_dashed_subcommand(self) -> None:
        argv = merge_command(chip="esp32s2", output="out.bin", images=[(0x1000, "bootloader.bin")])

        self.assertIn("--chip", argv)
        self.assertEqual(argv[argv.index("--chip") + 1], "esp32s2")
        # esptool 5 renamed merge_bin to merge-bin; the workflow pins esptool 5.
        self.assertIn("merge-bin", argv)

    def test_runs_esptool_from_the_current_interpreter(self) -> None:
        argv = merge_command(
            chip="esp32s3",
            output="out.bin",
            images=[(0x0, "bootloader.bin")],
            python_executable="/venv/bin/python",
        )

        self.assertEqual(argv[:3], ["/venv/bin/python", "-m", "esptool"])

    def test_does_not_override_the_compiled_flash_header(self) -> None:
        argv = merge_command(chip="esp32s3", output="out.bin", images=[(0x0, "bootloader.bin")])

        # merge-bin defaults these to "keep"; passing them is how a merged image
        # ends up mismatching the board it was built for.
        for flag in ("--flash-mode", "--flash-freq", "--flash-size", "--flash_mode"):
            self.assertNotIn(flag, argv)

    def test_rejects_an_empty_image_list(self) -> None:
        with self.assertRaises(ValueError):
            merge_command(chip="esp32s3", output="out.bin", images=[])


if __name__ == "__main__":
    unittest.main()
