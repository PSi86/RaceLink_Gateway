"""Stage RaceLink_Gateway build output as release assets.

Produces, per PlatformIO environment:

* the application image (what WLED-style OTA and the host's OTA dialog consume),
* the three pre-application images — bootloader, partition table, boot_app0 —
  so a blank chip can be commissioned from a release without a local toolchain,
* a merged factory image covering all four, for USB flashing and the web
  flasher,

plus one SHA-256 manifest and one machine-readable ``…-assets.json`` sidecar
describing every file, its kind, its flash offset and its digest. The sidecar
exists so consumers never have to parse filenames.

The factory image is deliberately named ``…-factory-usb-serial-only.bin``: sent
over OTA it is rejected in the normal case, but with WLED's *Ignore firmware
validation* ticked it is written into the inactive slot and bricks the device
until someone re-flashes over serial.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.release_artifacts import (  # noqa: E402
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


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _stage(source: Path, target: Path) -> Path:
    if not source.is_file():
        raise SystemExit(f"Expected build artifact not found: {source}")
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
) -> dict:
    """Stage every asset for one environment and return its manifest entry."""
    firmware = build_dir / "firmware.bin"
    if not firmware.is_file():
        raise SystemExit(f"Expected firmware artifact not found: {firmware}")

    chip = read_chip_name(firmware.read_bytes())
    env_metadata = metadata.get(env, metadata)
    flash_images = flash_images_from_metadata(metadata, env)

    partition_source = next(
        (image for image in flash_images if Path(image.path).name == "partitions.bin"),
        None,
    )
    if partition_source is None:
        raise SystemExit(f"PlatformIO reported no partition table for {env}")
    partitions = parse_partition_table(Path(partition_source.path).read_bytes())
    app_offset = application_offset(partitions)

    # Cross-check the offset PlatformIO reported against the one this chip is
    # known to need. Either source alone could be wrong silently; disagreement
    # cannot be.
    bootloader = next(
        (image for image in flash_images if Path(image.path).name == "bootloader.bin"),
        None,
    )
    if bootloader is None:
        raise SystemExit(f"PlatformIO reported no bootloader for {env}")
    expected_offset = bootloader_offset_for_chip(chip)
    if bootloader.offset != expected_offset:
        raise SystemExit(
            f"{env}: PlatformIO flashes the bootloader at 0x{bootloader.offset:x} but "
            f"{chip} expects 0x{expected_offset:x}. Refusing to publish a factory image."
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
                "sha256": _sha256(path),
            }
        )

    app_target = _stage(
        firmware,
        dist_dir
        / artifact_name(
            product=product, version=version, env=env, kind=APP_KIND, dev_type=dev_type
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
        product=product, version=version, env=env, kind=FACTORY_KIND, dev_type=dev_type
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--product", default="RaceLink_Gateway")
    parser.add_argument("--version", required=True)
    parser.add_argument("--env", dest="envs", action="append", required=True)
    parser.add_argument("--build-root", type=Path, default=Path(".pio/build"))
    parser.add_argument("--dist-dir", type=Path, default=Path("dist"))
    parser.add_argument(
        "--metadata",
        type=Path,
        required=True,
        help="JSON written by `pio project metadata --json-output-path`.",
    )
    args = parser.parse_args()

    args.dist_dir.mkdir(parents=True, exist_ok=True)
    metadata = json.loads(args.metadata.read_text(encoding="utf-8"))

    environments = [
        stage_environment(
            env=env,
            product=args.product,
            version=args.version,
            build_dir=args.build_root / env,
            dist_dir=args.dist_dir,
            metadata=metadata,
        )
        for env in args.envs
    ]

    manifest = {
        "product": args.product,
        "version": args.version,
        "environments": environments,
    }
    manifest_path = args.dist_dir / manifest_name(args.product, args.version)
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    checksum_lines = [
        f"{asset['sha256']}  {asset['file']}"
        for environment in environments
        for asset in environment["assets"]
    ]
    checksum_path = args.dist_dir / checksum_name(args.product, args.version)
    checksum_path.write_text("\n".join(checksum_lines) + "\n", encoding="utf-8")

    print(f"Staged {len(checksum_lines)} artifacts into {args.dist_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
