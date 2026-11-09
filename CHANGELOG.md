# Changelog

## Unreleased
- `RECOVERY_LOOP` no longer counts control/wrapper nodes (`NavigateRecovery`, `RecoveryActions`, fallbacks) — they go RUNNING on every normal navigation and made the check fire constantly.
- New `SCAN_TF_AHEAD` warning: scan stamps are >0.5 s ahead of the newest odom->base_link TF, so lookups at the scan time fail with "extrapolation into the future".
- New `MAP_STALL` warning: robot commanded to move for 10 s+ in the last minute but `/map` free cells did not grow — SLAM is not integrating scans.
- New `IDLE_NO_GOALS` warning: stack alive 45 s+, no goal ever sent and the robot never commanded — the goal source (e.g. an explorer) probably quit early.

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
