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
- [ ] Namespaced `/diagnostics` topic

## Next up
- [x] `NO_PROGRESS`: goal active but odometry shows no movement
- [x] Tunable thresholds (`--stale`, `--flap-count`, `--flap-window`, `--progress-time`, `--progress-dist`)
- [x] Run the integration tests (`test/test_integration.py`) in CI against the new flags and `NO_PROGRESS`; they are slow locally
- [ ] Integration test for `--json` output and `--diagnostics`
- [x] Repeating recovery behaviours check (spin/backup/wait loops) — `RECOVERY_LOOP`, wrapper nodes ignored
- [x] AMCL covariance too high (`AMCL_UNCERTAIN`)
- [ ] Rotating in place counts as "no progress" today; consider yaw change as movement
- [x] Tag and release 0.3.0 (0.4.0 released since)
- [x] `IDLE_NO_GOALS`, `MAP_STALL`, `SCAN_TF_AHEAD` checks (unreleased)
- [ ] Cut a release with the three new checks + `RECOVERY_LOOP` fix
