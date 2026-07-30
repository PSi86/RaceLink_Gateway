"""Guards for the CI workflows.

``build.yml`` compiles every environment declared in ``platformio.ini`` and
publishes nothing; ``release.yml`` ships a fixed list. The two can drift — a new
hardware target gets built on every pull request but silently never released —
so the lists are compared here.
"""

from __future__ import annotations

import pathlib
import re
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
BUILD_WORKFLOW = ROOT / ".github" / "workflows" / "build.yml"
RELEASE_WORKFLOW = ROOT / ".github" / "workflows" / "release.yml"
PLATFORMIO_INI = ROOT / "platformio.ini"

ENV_SECTION_PATTERN = re.compile(r"^\[env:(?P<name>[^\]]+)\]\s*$", re.MULTILINE)
RELEASE_ENVS_PATTERN = re.compile(r'^\s*RELEASE_ENVS:\s*"(?P<envs>[^"]*)"', re.MULTILINE)


def _declared_environments() -> set[str]:
    source = PLATFORMIO_INI.read_text(encoding="utf-8")
    return {match.group("name").strip() for match in ENV_SECTION_PATTERN.finditer(source)}


def _released_environments() -> set[str]:
    source = RELEASE_WORKFLOW.read_text(encoding="utf-8")
    match = RELEASE_ENVS_PATTERN.search(source)
    if match is None:
        raise AssertionError("release.yml no longer declares RELEASE_ENVS")
    return set(match.group("envs").split())


class BuildWorkflowTests(unittest.TestCase):
    def test_build_workflow_runs_on_pull_requests_and_main(self) -> None:
        source = BUILD_WORKFLOW.read_text(encoding="utf-8")

        self.assertIn("pull_request:", source)
        self.assertIn("push:", source)
        self.assertIn("workflow_dispatch:", source)

    def test_build_workflow_runs_tests_and_builds_every_environment(self) -> None:
        source = BUILD_WORKFLOW.read_text(encoding="utf-8")

        self.assertIn('python -m unittest discover -s tests -p "test_*.py"', source)
        # Deliberately without -e: platformio.ini sets no default_envs, so a bare
        # `run` covers every environment, including ones added later.
        self.assertIn("python -m platformio run\n", source)

    def test_build_workflow_collects_metadata_before_building(self) -> None:
        # See tests/test_release_workflow.py — `project metadata` cleans the
        # build directory, so the rehearsal has to collect it first too.
        source = BUILD_WORKFLOW.read_text(encoding="utf-8")

        self.assertLess(
            source.index("project metadata --json-output-path"),
            source.index("python -m platformio run\n"),
        )

    def test_build_workflow_publishes_nothing(self) -> None:
        source = BUILD_WORKFLOW.read_text(encoding="utf-8")

        self.assertNotIn("action-gh-release", source)
        self.assertNotIn("git tag", source)
        self.assertNotIn("git push", source)
        self.assertIn("contents: read", source)


class ReleaseEnvironmentCoverageTests(unittest.TestCase):
    def test_every_released_environment_exists(self) -> None:
        missing = _released_environments() - _declared_environments()

        self.assertFalse(
            missing,
            f"release.yml ships environments platformio.ini does not declare: {sorted(missing)}",
        )

    def test_every_declared_environment_is_released(self) -> None:
        unreleased = _declared_environments() - _released_environments()

        self.assertFalse(
            unreleased,
            "platformio.ini declares environments release.yml does not ship: "
            f"{sorted(unreleased)}. If that is deliberate, record why here rather "
            "than leaving the omission indistinguishable from an oversight.",
        )


if __name__ == "__main__":
    unittest.main()
