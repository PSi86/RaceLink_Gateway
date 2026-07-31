"""Stage one build environment's output as release assets.

Kept byte-identical in RaceLink_Gateway and RaceLink_WLED, for the same reason
as :mod:`scripts.release_artifacts`: this is where a mistake produces an image
that flashes cleanly and never boots, so there should be exactly one
implementation of it. The repositories differ in how they drive this — the
gateway iterates a flat environment list, RaceLink_WLED stages one profile at a
time into an external WLED checkout — but not in what a staged environment
looks like.

Produced per environment, two files:

* ``<env>-<version>-ota.bin`` -- the application image, which is what OTA and
  the host's firmware dialog take,
* ``<env>-<version>-usbflash.bin`` -- bootloader, partition table, OTA selector
  and application merged into one image, written at offset 0.

The three pre-application images are still merged into the factory image, but
they are no longer published on their own. They were 60% of a release's files
and about 1% of its bytes, every one of them a byte-for-byte copy of something
the factory image already contains -- and in a six-target release the eighteen
of them held only five distinct payloads. ``esptool write_flash 0x0
<…-usbflash.bin>`` reaches the same state in one write, with no per-SoC
bootloader offset to get wrong, which was the one number that route asked
somebody to look up.

What those files carried that was worth keeping -- where each block sits, how
big it is and what it hashes to -- is now the ``parts`` list in the sidecar. It
costs no files at all.
"""

from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
from pathlib import Path

from scripts.release_artifacts import (
    APP_KIND,
    FACTORY_KIND,
    application_offset,
    artifact_name,
    bootloader_offset_for_chip,
    device_type,
    flash_images_from_metadata,
    led_defaults,
    manifest_name,
    merge_command,
    parse_partition_table,
    read_chip_name,
)

# PlatformIO names the pre-application images by file; map them to the block
# name so the sidecar reads as intent rather than as a build-directory
# filename.
PART_KINDS = {
    "bootloader.bin": "bootloader",
    "partitions.bin": "partitions",
    "boot_app0.bin": "boot_app0",
}


def sha256_of(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _stage(source: Path, target: Path) -> Path:
    if not source.is_file():
        raise SystemExit(f"Expected build artifact not found: {source}")
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)
    return target


def stage_environment(
    *,
    env: str,
    version: str,
    build_dir: Path,
    dist_dir: Path,
    metadata: dict,
) -> dict:
    """Stage both published assets for one environment and return its sidecar entry."""
    firmware = build_dir / "firmware.bin"
    if not firmware.is_file():
        raise SystemExit(f"Expected firmware artifact not found: {firmware}")

    chip = read_chip_name(firmware.read_bytes())
    env_metadata = metadata.get(env, metadata)
    flash_images = flash_images_from_metadata(metadata, env)

    by_name = {Path(image.path).name: image for image in flash_images}
    for required in ("bootloader.bin", "partitions.bin"):
        if required not in by_name:
            raise SystemExit(f"{env}: PlatformIO reported no {required}")

    partitions = parse_partition_table(Path(by_name["partitions.bin"].path).read_bytes())
    app_offset = application_offset(partitions)

    # Cross-check the offset PlatformIO reported against the one this chip is
    # known to need. Either source alone could be wrong silently; disagreement
    # cannot be. This is the guard that catches an S2 or classic-ESP32 target
    # whose bootloader sits at 0x1000 rather than 0x0.
    expected_offset = bootloader_offset_for_chip(chip)
    if by_name["bootloader.bin"].offset != expected_offset:
        raise SystemExit(
            f"{env}: PlatformIO flashes the bootloader at "
            f"0x{by_name['bootloader.bin'].offset:x} but {chip} expects "
            f"0x{expected_offset:x}. Refusing to publish a factory image."
        )

    defines = env_metadata.get("defines") or []
    dev_type = device_type(defines)
    # What this build drives if nothing is seeded. Absent for the gateway,
    # which is also how the flasher knows not to offer an LED step for it.
    leds = led_defaults(defines)
    assets: list[dict] = []

    def record(path: Path, kind: str, offset: int) -> None:
        assets.append(
            {
                "file": path.name,
                "kind": kind,
                "offset": offset,
                "size": path.stat().st_size,
                "sha256": sha256_of(path),
            }
        )

    app_target = _stage(
        firmware,
        dist_dir / artifact_name(version=version, env=env, kind=APP_KIND),
    )
    record(app_target, APP_KIND, app_offset)

    # The pre-application images go into the merge but not into the release.
    # `parts` keeps the layout on the record -- which block sits where, how big
    # it is, what it hashes to -- so the information those three files used to
    # carry survives without shipping them. The fourth block is the
    # application, at `app_offset`, published in its own right above.
    parts: list[dict] = []
    merge_inputs: list[tuple[int, str]] = [(app_offset, str(firmware))]
    for image in flash_images:
        source = Path(image.path)
        kind = PART_KINDS.get(source.name)
        if kind is None:
            raise SystemExit(f"{env}: unexpected flash image {source.name}")
        parts.append(
            {
                "kind": kind,
                "offset": image.offset,
                "size": source.stat().st_size,
                "sha256": sha256_of(source),
            }
        )
        merge_inputs.append((image.offset, str(source)))

    factory_target = dist_dir / artifact_name(version=version, env=env, kind=FACTORY_KIND)
    command = merge_command(chip=chip, output=str(factory_target), images=merge_inputs)
    print(" ".join(command), flush=True)
    subprocess.run(command, check=True)
    record(factory_target, FACTORY_KIND, 0)

    entry = {
        "env": env,
        "chip": chip,
        "dev_type": dev_type,
        "app_offset": app_offset,
        "parts": sorted(parts, key=lambda part: part["offset"]),
        "assets": assets,
    }
    if leds is not None:
        entry["led_defaults"] = leds
    return entry


def write_release_index(
    *,
    dist_dir: Path,
    product: str,
    version: str,
    environments: list[dict],
    extra: dict | None = None,
) -> Path:
    """Write the assets.json sidecar.

    There is no companion sha256.txt any more. It restated, in a second format,
    exactly the digests this file already carries per asset -- and nothing read
    it that could not read this one just as easily.
    """
    if not environments:
        raise SystemExit("No environments were staged")

    manifest = {"product": product, "version": version, **(extra or {})}
    manifest["environments"] = environments
    manifest_path = dist_dir / manifest_name(product, version)
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest_path
