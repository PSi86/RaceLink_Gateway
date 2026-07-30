"""Stage one build environment's output as release assets.

Kept byte-identical in RaceLink_Gateway and RaceLink_WLED, for the same reason
as :mod:`scripts.release_artifacts`: this is where a mistake produces an image
that flashes cleanly and never boots, so there should be exactly one
implementation of it. The repositories differ in how they drive this — the
gateway iterates a flat environment list, RaceLink_WLED stages one profile at a
time into an external WLED checkout — but not in what a staged environment
looks like.

Produced per environment:

* the application image, which is what OTA and the host's firmware dialog take,
* the three pre-application images (bootloader, partition table, boot_app0), so
  a blank chip can be commissioned without a local toolchain,
* a merged factory image covering all four, written at offset 0.
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
    checksum_name,
    device_type,
    flash_images_from_metadata,
    manifest_name,
    merge_command,
    parse_partition_table,
    read_chip_name,
)

# PlatformIO names the pre-application images by file; map them to the asset
# kind so the sidecar reads as intent rather than as a filename.
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
    product: str,
    version: str,
    build_dir: Path,
    dist_dir: Path,
    metadata: dict,
    variant: str | None = None,
) -> dict:
    """Stage every asset for one environment and return its manifest entry."""
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

    dev_type = device_type(env_metadata.get("defines") or [])
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
        dist_dir
        / artifact_name(
            product=product,
            version=version,
            env=env,
            kind=APP_KIND,
            dev_type=dev_type,
            variant=variant,
        ),
    )
    record(app_target, APP_KIND, app_offset)

    merge_inputs: list[tuple[int, str]] = [(app_offset, str(firmware))]
    for image in flash_images:
        source = Path(image.path)
        kind = PART_KINDS.get(source.name)
        if kind is None:
            raise SystemExit(f"{env}: unexpected flash image {source.name}")
        target = _stage(
            source,
            dist_dir / artifact_name(product=product, version=version, env=env, kind=kind),
        )
        record(target, kind, image.offset)
        merge_inputs.append((image.offset, str(source)))

    factory_target = dist_dir / artifact_name(
        product=product,
        version=version,
        env=env,
        kind=FACTORY_KIND,
        dev_type=dev_type,
        variant=variant,
    )
    command = merge_command(chip=chip, output=str(factory_target), images=merge_inputs)
    print(" ".join(command), flush=True)
    subprocess.run(command, check=True)
    record(factory_target, "factory", 0)

    return {
        "env": env,
        "chip": chip,
        "dev_type": dev_type,
        "app_offset": app_offset,
        "assets": assets,
    }


def write_release_index(
    *,
    dist_dir: Path,
    product: str,
    version: str,
    environments: list[dict],
    extra: dict | None = None,
) -> tuple[Path, Path]:
    """Write the assets.json sidecar and the SHA-256 manifest."""
    if not environments:
        raise SystemExit("No environments were staged")

    manifest = {"product": product, "version": version, **(extra or {})}
    manifest["environments"] = environments
    manifest_path = dist_dir / manifest_name(product, version)
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    checksum_lines = [
        f"{asset['sha256']}  {asset['file']}"
        for environment in environments
        for asset in environment["assets"]
    ]
    checksum_path = dist_dir / checksum_name(product, version)
    checksum_path.write_text("\n".join(checksum_lines) + "\n", encoding="utf-8")

    return manifest_path, checksum_path
