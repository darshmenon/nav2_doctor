# nav2_doctor Quality Declaration

This declaration follows [REP-2004](https://reps.openrobotics.org/rep-2004/). **nav2_doctor claims Quality Level 5**, the lowest level: it is a developer tool that attaches to a running Nav2 stack and reports why the robot is not moving, it is not a runtime dependency, and it does not meet the higher-level requirements below.

## Summary of REP-2004 sections

| # | Section | Status |
|---|---|---|
| 1 | Version policy | Semantic Versioning (`package.xml`, `CHANGELOG.md`). Currently 0.6.0 (< 1.0.0: the API may change between minor releases). |
| 2 | Change control | Public GitHub repository; every change lands on `main` through a commit and runs CI. No required review (single maintainer). |
| 3 | Documentation | `README.md` with usage and limits, `CHANGELOG.md`, `CONTRIBUTING.md`. No generated API docs (no rosdoc2 config). |
| 4 | Testing | pytest unit tests (see `test/`), run in CI on ROS 2 Humble and Jazzy containers. No code-coverage measurement yet. |
| 5 | Dependencies | ROS 2 packages only (see `package.xml`); no non-ROS runtime dependencies beyond the Python standard library. |
| 6 | Platform support | Ubuntu 22.04 / ROS 2 Humble and Ubuntu 24.04 / ROS 2 Jazzy (CI). Not tested on other platforms. |
| 7 | Security | See `SECURITY.md`. The tool is read-only or opt-in (see README) and opens no network ports beyond ROS 2 DDS. |

## Known gaps to reach a higher level

- No coverage measurement or coverage target.
- Linters: flake8 is configured in `setup.cfg`; `ament_lint` tests (copyright, pep257) are not wired in.
- No generated API documentation.
- No release to the ROS buildfarm (installed from source).
