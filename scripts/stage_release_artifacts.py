"""Stage RaceLink_Gateway build output as release assets.

Thin driver around :mod:`scripts.release_staging`, which is byte-identical to
its RaceLink_WLED counterpart. The repositories differ only in how they iterate:
the gateway has a flat list of environments, RaceLink_WLED stages one profile at
a time into an external WLED checkout.

Produced per environment: the application image and a merged factory image —
plus one ``…-assets.json`` sidecar describing every file, so consumers never
have to parse filenames.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.release_staging import stage_environment, write_release_index  # noqa: E402

PRODUCT = "RaceLink_Gateway"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--product", default=PRODUCT)
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
            version=args.version,
            build_dir=args.build_root / env,
            dist_dir=args.dist_dir,
            metadata=metadata,
        )
        for env in args.envs
    ]

    manifest_path = write_release_index(
        dist_dir=args.dist_dir,
        product=args.product,
        version=args.version,
        environments=environments,
    )

    staged = sum(len(environment["assets"]) for environment in environments)
    print(f"Staged {staged} artifacts into {args.dist_dir}")
    print(f"  {manifest_path.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
