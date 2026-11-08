# nav2_doctor

[![CI](https://github.com/darshmenon/nav2_doctor/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/darshmenon/nav2_doctor/actions/workflows/ci.yml)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-blue)](https://docs.ros.org/)
[![License](https://img.shields.io/badge/license-Apache%202.0-green)](LICENSE)

Why isn't my robot moving? Attach to any running Nav2 stack and find out.

```bash
ros2 run nav2_doctor nav2_doctor                 # live, every 2s
ros2 run nav2_doctor nav2_doctor --ns robot1     # namespaced robot
ros2 run nav2_doctor nav2_doctor --once 5        # watch 5s, one report
```

Detects: TF freeze, stale single TF edge, missing map->odom, silent odom/scan,
stale local costmap, inactive lifecycle nodes, goal flapping, zero `cmd_vel`
during an active goal. Works on stock Nav2 (Humble/Jazzy), no sim needed.

```
ERROR [TF_FREEZE] /tf silent for 4.1s - TF froze; costmap filters go inert and Nav2 stalls
WARN  [GOAL_FLAPPING] 5 new goals in 30s - explorer/goal source is oscillating (add hysteresis)
```

## Why

When Nav2 sits still, the cause is usually one of a handful of things, and finding it means juggling `ros2 topic hz`, `tf2_echo`, `ros2 lifecycle get` and the logs. nav2_doctor runs those checks together and prints a one-line cause. It is read-only, needs no rebuild, and works on any Nav2 stack.

Build: `colcon build --packages-select nav2_doctor`. Tests: `python3 -m pytest test`.
