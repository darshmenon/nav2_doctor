# TODO

Ideas from surveying similar tools (Nav2 Ready, TopicForge diagnose-nav2, EasyNav diagnostic_recovery).

- [x] `NO_CMD_SUBSCRIBER`: cmd_vel published but nothing subscribes (topic name mismatch)
- [x] `CMD_NO_MOTION`: cmd_vel non-zero but odom not moving (chassis/driver problem)
- [x] `CLOCK_SKEW`: TF stamps differ from the local clock
- [x] `INFLATION_SMALL`: inflation_radius smaller than robot_radius
- [x] `--diagnostics`: publish findings as `diagnostic_msgs/DiagnosticArray` on `/diagnostics`
- [x] `NO_NAV2_VISIBLE`: hint about ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY / --ns when nothing is visible
- [x] Machine-readable output (`--json`, already shipped in 0.2.x)
- [x] `--hints`: suggested next step per finding
- [x] Footprint-polygon inscribed radius
- [x] `/initialpose`: covered by the `NO_MAP_ODOM` hint
- [x] `RECOVERY_LOOP` from `/behavior_tree_log`
- [x] `/tf_static` read for static map->odom (NO_MAP_ODOM false positive)
- [x] Namespaced `/diagnostics` topic

## Next up
- [x] `NO_PROGRESS`: goal active but odometry shows no movement
- [x] Tunable thresholds (`--stale`, `--flap-count`, `--flap-window`, `--progress-time`, `--progress-dist`)
- [x] Run the integration tests (`test/test_integration.py`) in CI against the new flags and `NO_PROGRESS`; they are slow locally
- [x] Integration test for `--json` output and `--diagnostics`
- [x] Repeating recovery behaviours check (spin/backup/wait loops) — `RECOVERY_LOOP`, wrapper nodes ignored
- [x] AMCL covariance too high (`AMCL_UNCERTAIN`)
- [x] Rotating in place no longer counts as "no progress" (yaw change is movement, `--progress-yaw`)
- [x] Tag and release 0.3.0 (0.4.0 released since)
- [x] `IDLE_NO_GOALS`, `MAP_STALL`, `SCAN_TF_AHEAD` checks (unreleased)
- [x] Cut a release with the three new checks + `RECOVERY_LOOP` fix (0.5.0)
- [x] Inert costmap filters: `FILTER_NO_MASK`, `FILTER_NO_INFO` (unreleased)
- [ ] Map-distortion check, multi-robot auto-discovery, /rosout errors tied to the cause

## From the 2026-11-20 research (web + GitHub; see ~/rosnav/todo.md for the full list)
- [ ] Ingest `/diagnostics` and show ERROR/WARN entries as findings (hardware status; ros2_control publishes `DiagnosticArray` in `example_17`)
- [ ] Check we cover Nav2 Ready's list: sensor-to-base transform for the scan frame, `cmd_vel` subscriber, ROS time settings; add `SCAN_FRAME_NO_TF` if the scan frame has no TF to the base
- [ ] `--expect /scan:10` style per-topic minimum rates
- [ ] Offline replay on a rosbag2 (`--bag`), so a recorded failure can be diagnosed after the fact
- [ ] Closed-loop scoreboard against `nav2_fault_inject`: recall per injected fault in the README
