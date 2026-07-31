"""Guards for the release workflow's artifact staging.

The failure modes these cover are all silent: an esptool major bump that
renames the merge subcommand, a staging step that runs before the metadata it
reads exists, or a publish glob that quietly drops the new assets.
"""

from __future__ import annotations

import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
RELEASE_WORKFLOW = ROOT / ".github" / "workflows" / "release.yml"


class ReleaseWorkflowTests(unittest.TestCase):
    def setUp(self) -> None:
        self.source = RELEASE_WORKFLOW.read_text(encoding="utf-8")

    def test_esptool_major_is_pinned(self) -> None:
        # esptool 5 renamed merge_bin to merge-bin; scripts/release_artifacts.py
        # emits the dashed form, so an unpinned major would break a release.
        self.assertIn('python -m pip install "esptool>=5,<6"', self.source)

    def test_metadata_is_collected_before_the_build(self) -> None:
        # `pio project metadata` cleans the build directory whenever it changes
        # PlatformIO's project checksum — resolving a dependency the build had
        # not is enough — so collecting it after `pio run` deletes the
        # firmware.bin about to be staged. RaceLink_WLED hit exactly that.
        metadata = self.source.index("--json-output-path metadata.json")
        build = self.source.index("python -m platformio run")
        staging = self.source.index("scripts/stage_release_artifacts.py")

        self.assertLess(metadata, build, "project metadata must run before the build")
        self.assertLess(build, staging, "staging needs the built firmware")

    def test_staging_is_delegated_to_the_tested_script(self) -> None:
        self.assertIn("python scripts/stage_release_artifacts.py", self.source)
        self.assertIn("--metadata metadata.json", self.source)

    def test_every_staged_asset_is_published(self) -> None:
        # A narrower glob is how the factory images or the assets.json sidecar
        # would silently fail to ship.
        self.assertIn("files: dist/*", self.source)
        self.assertIn("path: dist/*", self.source)

    def test_release_notes_warn_about_the_factory_image(self) -> None:
        self.assertIn("USB serial only", self.source)

    def test_release_notes_do_not_promise_files_that_no_longer_ship(self) -> None:
        # The bootloader, partition table and OTA selector are merged into the
        # factory image and no longer published on their own. Notes that still
        # list them send people looking for assets that are not there.
        for gone in ("-bootloader.bin", "-partitions.bin", "-boot_app0.bin", "-sha256.txt"):
            with self.subTest(gone=gone):
                self.assertNotIn(gone, self.source)

    def test_tests_run_before_anything_is_published(self) -> None:
        tests = self.source.index('python -m unittest discover -s tests -p "test_*.py"')
        tag = self.source.index('git tag "${{ steps.release_version.outputs.tag }}"')

        self.assertLess(tests, tag)


if __name__ == "__main__":
    unittest.main()
