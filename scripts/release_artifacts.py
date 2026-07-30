"""Pure helpers for staging RaceLink release artifacts.

Kept byte-identical in RaceLink_Gateway and RaceLink_WLED: both publish the same
asset shapes, and a fix to the offset or chip logic has to reach both. The
module is I/O-free so the release workflows' naming, offset and chip-detection
logic can be exercised without a toolchain; each repository's CLI wrapper does
the copying and calls out to esptool.

Design notes:

* **Offsets come from the build, never from a table.** PlatformIO's project
  metadata reports the flash images (bootloader, partition table, boot_app0)
  together with the offsets the platform itself would flash them at, so a new
  SoC — where the bootloader moves from ``0x0`` to ``0x1000`` — needs no change
  here. :func:`bootloader_offset_for_chip` exists only as a cross-check.
* **The chip is read out of the image being merged.** The ESP image header
  carries the chip id, so the value handed to ``esptool merge-bin`` cannot
  disagree with the binary it merges.
* **The application offset is read from the generated partition table**, which
  is the same file that ends up in the factory image.
"""

from __future__ import annotations

import struct
import sys
from dataclasses import dataclass
from typing import Iterable, Sequence

# --- ESP application image ------------------------------------------------

ESP_IMAGE_MAGIC = 0xE9
_CHIP_ID_STRUCT = struct.Struct("<H")
_CHIP_ID_OFFSET = 12

# esp_chip_id_t. An unmapped id is an error rather than a guess: merging with
# the wrong --chip produces an image that flashes cleanly and never boots.
CHIP_NAMES = {
    0x0000: "esp32",
    0x0002: "esp32s2",
    0x0005: "esp32c3",
    0x0009: "esp32s3",
    0x000C: "esp32c2",
    0x000D: "esp32c6",
    0x0010: "esp32h2",
}

# Second-stage bootloader offset per chip. Only used to verify what PlatformIO
# reported — see the module docstring.
BOOTLOADER_OFFSETS = {
    "esp32": 0x1000,
    "esp32s2": 0x1000,
    "esp32c3": 0x0000,
    "esp32s3": 0x0000,
    "esp32c2": 0x0000,
    "esp32c6": 0x0000,
    "esp32h2": 0x0000,
}

# --- Partition table ------------------------------------------------------

PARTITION_ENTRY_MAGIC = b"\xaa\x50"
PARTITION_ENTRY_SIZE = 32
PARTITION_TYPE_APP = 0x00


@dataclass(frozen=True)
class Partition:
    name: str
    type: int
    subtype: int
    offset: int
    size: int

    @property
    def is_app(self) -> bool:
        return self.type == PARTITION_TYPE_APP


@dataclass(frozen=True)
class FlashImage:
    """One pre-application image and the offset it is flashed at."""

    offset: int
    path: str


def read_chip_name(image: bytes) -> str:
    """Return the esptool chip name for an ESP application or bootloader image."""
    if len(image) < _CHIP_ID_OFFSET + _CHIP_ID_STRUCT.size:
        raise ValueError("Image is too short to contain an ESP image header")
    if image[0] != ESP_IMAGE_MAGIC:
        raise ValueError(f"Not an ESP image: magic is 0x{image[0]:02X}, expected 0xE9")
    (chip_id,) = _CHIP_ID_STRUCT.unpack_from(image, _CHIP_ID_OFFSET)
    try:
        return CHIP_NAMES[chip_id]
    except KeyError:
        raise ValueError(
            f"Unknown ESP chip id 0x{chip_id:04X}. Add it to CHIP_NAMES and to "
            "BOOTLOADER_OFFSETS before releasing for this SoC."
        ) from None


def bootloader_offset_for_chip(chip: str) -> int:
    """Expected second-stage bootloader offset, for cross-checking the build."""
    try:
        return BOOTLOADER_OFFSETS[chip]
    except KeyError:
        raise ValueError(
            f"No bootloader offset mapped for chip {chip!r}. Add it before "
            "releasing for this SoC — a wrong offset yields an unbootable image."
        ) from None


def parse_partition_table(data: bytes) -> list[Partition]:
    """Parse a generated ``partitions.bin`` into its entries."""
    partitions: list[Partition] = []
    for start in range(0, len(data), PARTITION_ENTRY_SIZE):
        entry = data[start : start + PARTITION_ENTRY_SIZE]
        if len(entry) < PARTITION_ENTRY_SIZE or not entry.startswith(PARTITION_ENTRY_MAGIC):
            break
        offset, size = struct.unpack_from("<II", entry, 4)
        partitions.append(
            Partition(
                name=entry[12:28].rstrip(b"\x00").decode("utf-8"),
                type=entry[2],
                subtype=entry[3],
                offset=offset,
                size=size,
            )
        )
    if not partitions:
        raise ValueError("Partition table contains no entries")
    return partitions


def application_offset(partitions: Sequence[Partition]) -> int:
    """Offset of the first application partition — where firmware.bin goes."""
    for partition in sorted(partitions, key=lambda p: p.offset):
        if partition.is_app:
            return partition.offset
    raise ValueError("Partition table declares no application partition")


def flash_images_from_metadata(metadata: dict, env: str) -> list[FlashImage]:
    """Extract the pre-application images PlatformIO would flash for ``env``.

    Accepts either the per-environment metadata mapping produced by
    ``pio project metadata --json-output`` or a single environment's idedata.
    """
    entry = metadata.get(env, metadata)
    extra = entry.get("extra") or {}
    images = extra.get("flash_images") or []
    if not images:
        raise ValueError(
            f"PlatformIO reported no flash images for {env!r}; without the "
            "bootloader and partition table a factory image cannot be built."
        )
    return [FlashImage(offset=int(str(i["offset"]), 0), path=str(i["path"])) for i in images]


def define_value(defines: Iterable[str], name: str) -> str | None:
    """Return the value of a ``NAME=value`` build define, if present."""
    prefix = f"{name}="
    for define in defines:
        text = str(define)
        if text.startswith(prefix):
            return text[len(prefix) :]
    return None


def device_type(defines: Iterable[str]) -> int | None:
    """Parse ``-D DEV_TYPE=<n>`` — the feature-set handle the master looks up."""
    raw = define_value(defines, "DEV_TYPE")
    if raw is None:
        return None
    return int(raw, 0)


# --- Artifact naming ------------------------------------------------------
#
# Field order: product, release version, environment, device type, variant,
# kind. The kind goes last because it is the most visible token in a file
# picker, and the factory image must never be mistaken for something OTA-able.
# The three pre-application images carry neither device type nor variant — they
# are build by-products of the environment, not firmware identity.
#
# `variant` carries the upstream version a build was made against, so a
# RaceLink_WLED asset states both its own release and the WLED release it wraps
# ("wled_v0.15.3"). Fields never contain "-", so the name stays parseable, but
# consumers should read the assets.json sidecar rather than the filename.

FACTORY_KIND = "factory-usb-serial-only"
APP_KIND = "app"


def artifact_name(
    *,
    product: str,
    version: str,
    env: str,
    kind: str,
    dev_type: int | None = None,
    variant: str | None = None,
    extension: str = ".bin",
) -> str:
    parts = [product, version, env]
    if dev_type is not None:
        parts.append(f"TYPE{dev_type}")
    if variant is not None:
        parts.append(variant)
    parts.append(kind)
    return "-".join(parts) + extension


def checksum_name(product: str, version: str) -> str:
    return f"{product}-{version}-sha256.txt"


def manifest_name(product: str, version: str) -> str:
    return f"{product}-{version}-assets.json"


def merge_command(
    *,
    chip: str,
    output: str,
    images: Sequence[tuple[int, str]],
    python_executable: str | None = None,
) -> list[str]:
    """Build the ``esptool merge-bin`` argv.

    Flash mode, frequency and size are deliberately not passed: ``merge-bin``
    defaults all three to ``keep``, which preserves the header the bootloader
    was compiled with. Overriding them is how a merged image ends up
    mismatching the board it was built for.

    The interpreter defaults to the running one so esptool is taken from the
    same environment that installed it, rather than from whatever ``python``
    happens to be first on PATH.
    """
    if not images:
        raise ValueError("Refusing to merge an empty image list")
    interpreter = python_executable or sys.executable
    argv = [interpreter, "-m", "esptool", "--chip", chip, "merge-bin", "-o", output]
    for offset, path in sorted(images, key=lambda item: item[0]):
        argv += [hex(offset), path]
    return argv
