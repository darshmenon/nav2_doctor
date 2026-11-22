# Changelog

## Unreleased
- Fix: a controller that goes silent while a goal is active (hung, paused or crashed) is now reported as `NO_CMD_VEL` ("silent for N s"), not `CMD_ZERO` ("cmd_vel is zero"), because nothing is being published. A published zero command stays `CMD_ZERO`. Found by the 9-fault scoreboard against a live headless Nav2 stack (`nav2_fault_inject run --check`, 2026-11-22: SIGSTOP of `controller_server` was labelled `CMD_ZERO`). Regression test in `test_doctor.py`.
- `--expect STREAM:HZ` (repeatable, e.g. `--expect scan:10`): state the normal rate yourself and get `RATE_DROP` / `RATE_SPIKE` without a `--learn` run; overrides a learned value for that stream. Unit-tested, and run against the fake Nav2 graph: quiet at the true scan rate, `RATE_DROP` when 100 Hz is expected, exit 2 on a malformed value.
- Per-robot baselines: `--learn SECS` records the normal message rate of each stream, `--baseline` then warns with `RATE_DROP` / `RATE_SPIKE` when a stream drops below half or exceeds double that. Plain statistics (median messages per second), no model. Unit-tested, and run live against the fake Nav2 graph (learn, quiet when unchanged, `RATE_DROP` when the scan slows to a tenth). The live test found and fixed a bug: TF arrives in bursts, so a median-interval rate was absurd; the rate is now counted per second.

## 0.7.0
- New `LOG_*` warnings from `/rosout`: `LOG_NO_PROGRESS` ("Failed to make progress"), `LOG_SENSOR_OUT_OF_BOUNDS`, `LOG_TF_LOOKUP` (invalid frame, no connection, extrapolation, timed out waiting for transform), `LOG_PLAN_FAILED`, `LOG_LOOP_SLOW`. Only warn/error lines from Nav2 nodes, counted over 30 s, each with a hint. Unit-tested on the message patterns; not yet run live against a stack that emits them.
- Release workflow: removed the `pypi` job (it failed on every tag without a trusted publisher, and rclpy cannot come from PyPI anyway). Packaging metadata stays in `setup.py`.

## 0.6.1
- Fix: `CLOCK_SKEW` no longer fires falsely under Gazebo; when `/clock` is published, TF stamps are compared with sim time instead of the wall clock. Found by running against the rosnav Gazebo sim (now reports HEALTHY there).
- PyPI: `setup.py` carries the long description and metadata, and `release.yml` has a `pypi` job (needs a trusted publisher configured on pypi.org). rclpy still comes from the ROS install, not PyPI.
- Tests: the two costmap-filter integration tests skip when `nav2_msgs` is not installed (the `ros-base` CI image lacks it); CI was red on that import.
- Lint: flake8 clean (per-file E501 ignore for the long hint strings in `doctor.py`), `QUALITY_DECLARATION.md` (REP-2004 level 5).
- New `FILTER_NO_MASK` warning: a `nav2_msgs/CostmapFilterInfo` topic (keepout, speed zones) names a mask topic that nothing publishes, so the filter is inert. Found by discovering the info topics and checking publishers on each mask; a latched mask that was already received counts as fine.
- New `FILTER_NO_INFO` warning: the local costmap lists `filters` but no `CostmapFilterInfo` publisher exists.
- Unit tests and two integration tests (info without mask flagged, info with mask clean). Not yet tried on a live Nav2 stack with real filters.

## 0.6.0
- Rotating in place now counts as movement: `NO_PROGRESS` and `CMD_NO_MOTION` no longer fire while the robot turns at least `--progress-yaw` radians (default 0.2) in the window. Odometry without orientation behaves as before.
- `--diagnostics` with `--ns robotN` publishes on `/robotN/diagnostics` instead of the global `/diagnostics`.
- Integration tests for `--json` output, `--diagnostics` (global and namespaced) and in-place rotation.

## 0.5.0
- `--ns robotN` now also reads the shared global `/tf` (keeping only `robotN/...` frames), so multi-robot stacks that remap every robot's TF onto one `/tf` no longer report `TF_NONE`.
- `RECOVERY_LOOP` no longer counts control/wrapper nodes (`NavigateRecovery`, `RecoveryActions`, fallbacks) — they go RUNNING on every normal navigation and made the check fire constantly.
- New `SCAN_TF_AHEAD` warning: scan stamps are >0.5 s ahead of the newest odom->base_link TF, so lookups at the scan time fail with "extrapolation into the future".
- New `MAP_STALL` warning: robot commanded to move for 10 s+ in the last minute but `/map` free cells did not grow — SLAM is not integrating scans.
- New `IDLE_NO_GOALS` warning: stack alive 45 s+, no goal ever sent and the robot never commanded — the goal source (e.g. an explorer) probably quit early.
- `--ns robotN` now reads `/tf_static` too: a fixed `map->robotN/odom` (static localization mode) counts as present and is never reported stale, so `NO_MAP_ODOM` no longer fires for those robots.

## 0.4.0
- `--ready [SECS]`: pre-flight mode. Watches SECS (default 10), prints hints and a `READY` / `NOT READY` verdict, exits 1 on any blocking ERROR.

## 0.3.0
- New checks: `NO_CMD_SUBSCRIBER`, `CMD_NO_MOTION`, `CLOCK_SKEW`, `INFLATION_SMALL`, `NO_NAV2_VISIBLE`.
- `RECOVERY_LOOP`: repeated Spin/BackUp/Wait/ClearCostmap recoveries from `/behavior_tree_log`.
- `INFLATION_SMALL` now uses the footprint polygon's inscribed radius when no `robot_radius` is set.
- `--hints` prints a suggested next step under each finding; `--json` findings now carry a `hint` field.
- `--diagnostics` publishes findings to `/diagnostics` as `DiagnosticArray`.
- New `NO_PROGRESS` warning: a goal is active but the robot has not moved (odometry) for `--progress-time` seconds.
- Tunable thresholds: `--stale`, `--flap-count`, `--flap-window`, `--progress-time`, `--progress-dist`.
- `AMCL_UNCERTAIN` warning when `/amcl_pose` x/y covariance exceeds 0.5 m^2.
- `--json` prints one JSON object per report.

## 0.2.1
- Goal flapping now counts goals CANCELED/ABORTED before finishing. Before, a healthy explorer that sent a new goal after each success was flagged.

## 0.2.0
- `--startup-grace` flag; `--once` exits 1 when errors are found (usable in CI/scripts).
- 28 unit tests and 12 integration tests that run the real CLI against a fake Nav2 graph.

## 0.1.1
- Startup grace period: inactive lifecycle nodes warn for 60s before erroring.
- Clean exit on shutdown (no traceback); `--version` flag.
- CI matrix (Humble, Jazzy) and tag-triggered release workflow.

## 0.1.0
- Initial release: TF freeze/stale-edge, odom/scan/costmap stall, lifecycle, goal flapping, zero cmd_vel.
