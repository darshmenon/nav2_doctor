#!/usr/bin/env python3
"""nav2_doctor - attach to a running Nav2 stack and say why the robot isn't moving.

Pure-logic `Doctor` (no ROS imports, unit-testable) + a thin rclpy `main()`.
"""
import argparse
import json
import math
import re
import sys
import time
from collections import Counter
from dataclasses import dataclass

__version__ = '0.7.0'

NAV2_NODES = ['controller_server', 'planner_server', 'bt_navigator',
              'behavior_server', 'smoother_server', 'waypoint_follower']
STALE_S = 2.0          # a stream is "stale" if silent this long
NAV_STALE_S = 1.0      # controller output silence while FollowPath runs (controllers publish at ~20 Hz)
FLAP_WINDOW_S = 30.0   # goal-flapping look-back
STARTUP_S = 60.0       # grace period before inactive nodes count as errors
FLAP_COUNT = 4         # goals cancelled/aborted inside the window => flapping
PROGRESS_S = 15.0      # goal active this long with no movement => NO_PROGRESS
AMCL_COV_MAX = 0.5     # AMCL x/y covariance (m^2) above which localisation is uncertain
PROGRESS_M = 0.1       # minimum displacement (m) that counts as movement
PROGRESS_YAW = 0.2     # minimum heading change (rad) that counts as movement (rotating in place)
MAP_WIN_S = 60.0       # window for MAP_STALL
MAP_ACTIVE_S = 10.0    # seconds of non-zero cmd_vel in the window before an unchanged map is suspicious
MAP_RESET_CELLS = 500    # free cells lost at once (and over half the map) = a reset, not a stall
SCAN_LEAD_S = 0.5      # scan stamps this far ahead of the newest odom TF => SCAN_TF_AHEAD
IDLE_S = 45.0          # stack up this long, no goal ever seen and robot never commanded => IDLE_NO_GOALS
DRIVE_S = 5.0         # cmd_vel non-zero this long with no odom movement => CMD_NO_MOTION
SKEW_S = 1.0           # TF stamp vs wall clock difference that counts as clock skew


HINTS = {
    'TF_NONE': 'check ROS_DOMAIN_ID/--ns; is robot_state_publisher or the driver running? try `ros2 topic list | grep tf`',
    'TF_FREEZE': 'check the TF publisher (sim paused? driver crashed?) with `ros2 run tf2_ros tf2_monitor`',
    'TF_EDGE_STALE': 'the named edge\'s publisher died; `ros2 run tf2_ros tf2_echo <parent> <child>` to confirm',
    'NO_MAP_ODOM': 'start AMCL/SLAM and publish /initialpose (RViz "2D Pose Estimate")',
    'ODOM_NONE': 'check the base driver/odom topic name and namespace',
    'ODOM_STALE': 'odometry source stopped; check the base driver or sim bridge',
    'SCAN_NONE': 'check the lidar driver/bridge and the scan topic name',
    'SCAN_STALE': 'lidar stopped; check the driver or sim bridge',
    'COSTMAP_STALE': 'fix TF/scan first; then `ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap {}`',
    'NODE_MISSING': 'launch the node or check lifecycle_manager node_names; `ros2 node list`',
    'NODE_STARTING': 'wait for startup to finish; if it persists the lifecycle_manager is stuck',
    'NODE_INACTIVE': 'see the lifecycle_manager log; usually a YAML param error. `ros2 lifecycle get /<node>`',
    'GOAL_FLAPPING': 'add hysteresis/min goal distance to the exploration or goal source',
    'NO_CMD_VEL': 'controller_server isn\'t producing output; check its log and that the goal checker/plugins loaded',
    'RATE_DROP': 'a stream is much slower than this robot\'s learned normal: check CPU load, a throttled bridge or a degraded driver; re-learn with --learn if the change is intended',
    'RATE_SPIKE': 'a stream is much faster than this robot\'s learned normal: a duplicate publisher or a changed driver setting; re-learn with --learn if intended',
    'CMD_ZERO': 'planner or controller failed; check bt_navigator and controller_server logs, clear costmaps',
    'NO_PROGRESS': 'check progress_checker params and for an obstacle; see recovery behaviours in the BT log',
    'AMCL_UNCERTAIN': 'publish /initialpose in RViz, check the map matches the world, and that /scan is reaching AMCL',
    'NO_CMD_SUBSCRIBER': 'remap: `ros2 topic info /cmd_vel -v`; match the driver topic (cmd_vel_smoothed, velocity_smoother)',
    'MAP_STALL': 'SLAM gets no usable scans: check scan->odom TF errors ("extrapolation"), the scan topic/frame and that SLAM subscribes to the right /scan',
    'SCAN_FRAME_NO_TF': 'publish the lidar mount transform (static_transform_publisher / robot_state_publisher) from base_link to the scan frame_id; compare with `ros2 run tf2_tools view_frames`',
    'SCAN_TF_AHEAD': 'TF is lagging the laser (slow sim / overloaded TF publisher): consumers doing lookup at the scan stamp fail with "extrapolation into the future" - slow the sim down, fix the TF source rate, or let consumers fall back to the latest transform',
    'IDLE_NO_GOALS': 'the goal source (explorer / mission node) may have quit early - check its log for "exploration complete" or filtered frontiers',
    'CMD_NO_MOTION': 'check e-stop, motor enable and that the driver converts cmd_vel; `ros2 topic echo /odom`',
    'CLOCK_STALLED': 'check that the simulator is unpaused and the /clock bridge is running',
    'CLOCK_SKEW': 'sync clocks (chrony) or set use_sim_time consistently on every node',
    'INFLATION_SMALL': 'raise inflation_layer.inflation_radius above robot_radius (or footprint circumradius)',
    'RECOVERY_LOOP': 'find what keeps failing planning/control: read planner/controller logs, check costmap for phantom obstacles, loosen goal tolerance',
    'FILTER_NO_MASK': 'start nav2_map_server for the filter mask (and make it active in the lifecycle manager); `ros2 topic info <mask topic> -v` shows the publisher',
    'FILTER_BAD_LIMIT': 'check the mask encoding and base/multiplier: in map_server scale mode black pixels are 100 and white are 0',
    'FILTER_BELOW_MIN_SPEED': 'raise the mask speed floor to at least min_speed/max_speed (for both xy and theta), or lower DWB min_speed_xy / min_speed_theta',
    'FILTER_MOSTLY_SLOW': 'view the mask in RViz; in map_server scale mode black pixels are 100 and white are 0, so a mask written the other way round slows free ground',
    'SPEED_LIMIT_UNHEARD': 'set controller_server `speed_limit_topic` to the topic the SpeedFilter publishes (its `speed_limit_topic`); check with `ros2 topic info <topic> -v`',
    'FILTER_NO_INFO': 'start the costmap_filter_info_server (and add it to the lifecycle manager); the filter in the costmap `filters` list has nothing to read',
    'LOG_NO_PROGRESS': 'controller_server logged "Failed to make progress": check goal/progress checker tolerances, an obstacle in front, and that cmd_vel reaches the base',
    'LOG_SENSOR_OUT_OF_BOUNDS': 'a sensor origin is outside the costmap, so it cannot raytrace: enlarge the costmap, or check the sensor frame height and TF',
    'LOG_TF_LOOKUP': 'a node cannot look up a transform (missing frame, extrapolation): `ros2 run tf2_tools view_frames`, check frame names and use_sim_time on every node',
    'LOG_PLAN_FAILED': 'planner_server cannot make a path: the goal may be in an obstacle or unreachable; check the global costmap and the goal pose',
    'LOG_LOOP_SLOW': 'controller loop is missing its rate: machine overloaded or controller_frequency too high for the plugin',
    'NO_NAV2_VISIBLE': 'match ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY, RMW_IMPLEMENTATION and --ns with the robot',
}


# log lines worth naming: (regex on the message, finding code)
LOG_PATTERNS = (
    (re.compile(r'failed to make progress', re.I), 'LOG_NO_PROGRESS'),
    (re.compile(r'sensor origin .*out of map bounds|cannot raytrace', re.I), 'LOG_SENSOR_OUT_OF_BOUNDS'),
    (re.compile(r'invalid frame id|could not find a connection between|extrapolation|lookup would require|timed out waiting for transform', re.I),
     'LOG_TF_LOOKUP'),
    (re.compile(r'failed to create a plan|no valid path|goal .*unreachable|start occupied|goal occupied', re.I),
     'LOG_PLAN_FAILED'),
    (re.compile(r'control loop missed its desired rate', re.I), 'LOG_LOOP_SLOW'),
)
LOG_NODES = ('controller_server', 'planner_server', 'bt_navigator', 'behavior_server', 'smoother_server',
             'waypoint_follower', 'local_costmap', 'global_costmap', 'amcl', 'velocity_smoother')
LOG_WINDOW_S = 30.0
SLOW_PCT = 25       # a speed-mask cell limited below this percentage counts as "slow"
SLOW_SHARE = 0.5    # warn when at least this share of the mask is slow


def classify_log(msg):
    """Finding code for a /rosout message, or None."""
    for rx, code in LOG_PATTERNS:
        if rx.search(msg):
            return code
    return None


RECOVERY_WORDS = ('spin', 'backup', 'back_up', 'wait', 'clearentire', 'clear_entire')
# control/wrapper nodes (RecoveryNode, Fallback, ...) go RUNNING on every normal navigation - not a recovery
RECOVERY_IGNORE = ('recovery', 'fallback', 'actions', 'sequence', 'retry')
RECOVERY_COUNT = 3     # recovery behaviours started inside the window => RECOVERY_LOOP
RECOVERY_WINDOW_S = 60.0


def inscribed_radius(footprint):
    """Smallest distance from the origin to the footprint polygon edges, from a Nav2
    string like '[[0.3,0.2],[0.3,-0.2],[-0.3,-0.2],[-0.3,0.2]]'. None if unparseable."""
    try:
        pts = [tuple(map(float, p)) for p in json.loads(footprint)]
    except (ValueError, TypeError):
        return None
    if len(pts) < 3 or any(len(p) != 2 for p in pts):
        return None
    best = None
    for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1]):
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, -(x1 * dx + y1 * dy) / L2))
        d = ((x1 + t * dx) ** 2 + (y1 + t * dy) ** 2) ** 0.5
        best = d if best is None else min(best, d)
    return best


ARRIVALS_KEEP = 1500    # arrival times kept per stream for baselines


@dataclass
class Finding:
    level: str   # ERROR | WARN | OK
    code: str
    msg: str


class Doctor:
    """Feed it events with `seen()` / `state()`; ask `diagnose(now)`."""

    def __init__(self, startup_s=STARTUP_S, stale_s=STALE_S, flap_window_s=FLAP_WINDOW_S,
                 flap_count=FLAP_COUNT, progress_s=PROGRESS_S, progress_m=PROGRESS_M, progress_yaw=PROGRESS_YAW):
        self.startup_s = startup_s
        self.stale_s = stale_s
        self.nav_stale_s = NAV_STALE_S
        self.follow_running = None   # None until the BT log has mentioned FollowPath
        self.follow_since = None
        self.flap_window_s = flap_window_s
        self.flap_count = flap_count
        self.progress_s = progress_s
        self.progress_m = progress_m
        self.progress_yaw = progress_yaw
        self.poses = []          # (time, x, y, yaw) odometry samples; yaw is None when unknown
        self.goal_since = None   # when the current goal became active
        self.last = {}           # stream name -> last receipt time
        self.arrivals = {}       # stream name -> recent receipt times (for baselines)
        self.baseline = None     # learned per-robot rates (see baseline.py), or None
        self.tf_edges = {}       # (parent, child) -> last receipt time
        self.static_edges = set()   # (parent, child) from /tf_static: published once, never stale
        self.lifecycle = {}      # node -> state label or None (unreachable)
        self.goal_times = []     # times a goal id first appeared
        self.goal_active = False
        self.last_cmd_nonzero = None
        self.started = None
        self.cmd_since = None       # start of the current run of non-zero cmd_vel
        self.cmd_subscribers = None  # other nodes subscribed to cmd_vel (None = unknown)
        self.skew = None            # worst recent |TF stamp - clock| in seconds
        self.sim_clock_t = None     # last time /clock arrived; stays None without a simulator
        self.recoveries = []        # times a recovery behaviour started
        self.params = {}            # costmap params: inflation_radius, robot_radius
        self.map_hist = []          # (time, free_cells) per /map message
        self.cmd_secs = set()       # whole seconds in which cmd_vel was non-zero
        self.scan_stamp = None      # newest scan header stamp (s)
        self.scan_frame = None      # frame_id of the newest scan
        self.tf_stamp = None        # newest odom->base TF header stamp (s)
        self.ever_goal = False      # a goal was ever active/cancelled/aborted
        self.filter_infos = {}      # CostmapFilterInfo topic -> {'mask': mask topic, 'since': first seen}
        self.filter_mask_seen = set()   # mask topics a message was received on
        self.filter_pubs = {}       # mask topic -> number of publishers
        self.filter_stats = {}      # speed mask topic -> (fraction invalid, fraction below SLOW_PCT)
        self.filter_min_limit = {}  # percent speed mask topic -> lowest limit it can set (%)
        self.dwb = {}               # DWB max/min speed parameters of the controller
        self.speed_unheard = {}     # speed limit topic controller_server misses -> first seen
        self.speed_heard = []       # speed limit topics controller_server does subscribe to
        self.filters_configured = []    # names in the costmap `filters` parameter
        self.filter_info_pubs = None    # publishers on the CostmapFilterInfo topics (None = unknown)
        self.logs = {}              # LOG_* code -> [message, [times]]
        self.amcl_cov = None        # worst of AMCL x/y variance (m^2) from /amcl_pose

    # ---- inputs -------------------------------------------------------
    def seen(self, stream, now):
        self.started = self.started if self.started is not None else now
        self.last[stream] = now
        a = self.arrivals.setdefault(stream, [])
        a.append(now)
        if len(a) > ARRIVALS_KEEP:
            del a[:len(a) - ARRIVALS_KEEP]

    def sim_clock(self, now):
        self.sim_clock_t = now

    def amcl(self, var_x, var_y, now):
        self.seen('amcl', now)
        cov = max(var_x, var_y)
        # NaN compares False against every threshold, so a diverged filter (NaN covariance) used to look healthy;
        # a non-finite covariance is the worst localization there is.
        self.amcl_cov = cov if math.isfinite(cov) else float('inf')

    def log(self, msg, now):
        code = classify_log(msg)
        if code:
            entry = self.logs.setdefault(code, [msg, []])
            entry[0] = msg
            entry[1].append(now)

    def tf(self, parent, child, now, skew=None):
        self.seen('tf', now)
        if skew is not None:
            self.skew = abs(skew)
        self.tf_edges[(parent.lstrip('/'), child.lstrip('/'))] = now

    def _frame_reaches_base(self, frame):
        """True if `frame` is linked (either direction) through known TF edges to a base_link/base_footprint frame."""
        adj = {}
        for p, c in set(self.tf_edges) | self.static_edges:
            adj.setdefault(p, set()).add(c)
            adj.setdefault(c, set()).add(p)
        seen, todo = {frame}, [frame]
        while todo:
            n = todo.pop()
            if n.endswith(('base_link', 'base_footprint')):
                return True
            for m in adj.get(n, set()) - seen:
                seen.add(m); todo.append(m)
        return False

    def tf_static(self, parent, child):
        """A /tf_static edge (e.g. a fixed map->odom when localization is static): counts as present, never stale."""
        self.static_edges.add((parent.lstrip('/'), child.lstrip('/')))

    def filter_mask(self, topic, data, info_topic):
        """Check a speed filter's mask values: Nav2 turns each non-zero cell into `base + multiplier * value`."""
        self.filter_mask_seen.add(topic)
        fi = self.filter_infos.get(info_topic, {})
        if fi.get('type') not in (1, 2):    # 1 = speed in percent, 2 = absolute m/s; keepout masks have nothing to grade
            return
        counts = Counter(v for v in data if v >= 0)
        total = sum(counts.values())
        if not total:
            return
        if fi['type'] == 1:
            limits = [fi['base'] + fi['multiplier'] * v for v in counts if v]
            limits = [x for x in limits if 0 < x <= 100]
            if limits:
                self.filter_min_limit[topic] = min(limits)
            else:
                self.filter_min_limit.pop(topic, None)
        invalid = slow = 0
        for value, n in counts.items():
            if value == 0:
                continue
            speed = fi['base'] + fi['multiplier'] * value
            if speed <= 0 or (fi['type'] == 1 and speed > 100):
                invalid += n
            elif fi['type'] == 1 and speed < SLOW_PCT:
                slow += n
        self.filter_stats[topic] = (invalid / total, slow / total)

    def filter_info(self, info_topic, mask_topic, now, ftype=None, base=0.0, multiplier=1.0):
        """A costmap filter info message (nav2_msgs/CostmapFilterInfo): says which topic carries the filter mask."""
        self.seen('filter_info', now)
        fi = self.filter_infos.setdefault(info_topic, {'mask': mask_topic, 'since': now})
        fi.update(mask=mask_topic, type=ftype, base=base, multiplier=multiplier)

    def speed_limits(self, published, heard, now):
        """`published`: SpeedLimit topics with a publisher; `heard`: those controller_server subscribes to."""
        self.speed_heard = sorted(heard)
        # quiet if controller_server has no such subscription (not configured, or not visible), and if a topic it
        # does listen to is fed (a relay may be forwarding the other one)
        broken = heard and not set(heard) & set(published)
        self.speed_unheard = {t: self.speed_unheard.get(t, now) for t in published} if broken else {}

    def bt_status(self, node_name, status, now):
        """A behaviour tree status change. Controller output is only expected while FollowPath is RUNNING."""
        if node_name == 'FollowPath':
            running = status == 'RUNNING'
            if running and not self.follow_running:
                self.follow_since = now
            self.follow_running = running

    def cmd_nav(self, now):
        """The controller's own output (/cmd_vel_nav). Downstream smoothers keep /cmd_vel alive for a while after the
        controller dies, so a hung controller_server is only visible here."""
        self.seen('cmd_vel_nav', now)

    def cmd(self, linear, angular, now):
        self.seen('cmd_vel', now)
        if abs(linear) > 1e-3 or abs(angular) > 1e-3:
            self.cmd_secs.add(int(now))
            self.last_cmd_nonzero = now
            if self.cmd_since is None:
                self.cmd_since = now
        else:
            self.cmd_since = None

    def stamps(self, scan=None, tf=None, scan_frame=None):
        if scan_frame is not None:
            self.scan_frame = scan_frame.lstrip('/')
        if scan is not None:
            self.scan_stamp = scan
        if tf is not None:
            self.tf_stamp = tf

    def map_msg(self, free_cells, now):
        self.seen('map', now)
        if self.map_hist:
            last = self.map_hist[-1][1]
            if last - free_cells >= MAP_RESET_CELLS and free_cells < last / 2:
                self.map_hist = []   # map was reset: judge growth from here, not against the old map
        self.map_hist.append((now, free_cells))
        self.map_hist = [m for m in self.map_hist if now - m[0] <= MAP_WIN_S * 2]

    def recovery(self, node_name, now):
        n = node_name.lower()
        if any(w in n for w in RECOVERY_WORDS) and not any(w in n for w in RECOVERY_IGNORE):
            self.recoveries.append(now)
            self.recoveries = [t for t in self.recoveries if now - t <= RECOVERY_WINDOW_S]

    def goal(self, now):
        self.ever_goal = True
        self.goal_times.append(now)
        self.goal_times = [t for t in self.goal_times if now - t <= self.flap_window_s]

    def pose(self, x, y, now, yaw=None):
        self.seen('odom', now)
        self.poses.append((now, x, y, yaw))
        keep = max(self.progress_s, self.stale_s) * 2
        self.poses = [p for p in self.poses if now - p[0] <= keep]

    def set_goal_active(self, active, now):
        if active:
            self.ever_goal = True
        if active and not self.goal_active:
            self.goal_since = now
        elif not active:
            self.goal_since = None
        self.goal_active = active

    # ---- helpers ------------------------------------------------------
    def _moved(self, win):
        """(metres, radians) the robot moved away from the first sample of the window."""
        _, x0, y0, yaw0 = win[0]
        metres = max(((x - x0) ** 2 + (y - y0) ** 2) ** 0.5 for _, x, y, _ in win)
        rad = 0.0
        if yaw0 is not None:
            for _, _, _, yaw in win:
                if yaw is not None:
                    rad = max(rad, abs(math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0))))
        return metres, rad

    def _age(self, stream, now):
        t = self.last.get(stream)
        return None if t is None else now - t

    def _edge_ages(self, now, suffixes):
        return {e: now - t for e, t in self.tf_edges.items()
                if e[1].endswith(suffixes[0]) or e[0].endswith(suffixes[1])}

    # ---- diagnosis ----------------------------------------------------
    def diagnose(self, now):
        f = self._diagnose(now)
        if self.baseline:
            from nav2_doctor import baseline as bl
            f.extend(Finding(lv, code, msg) for lv, code, msg in bl.check(self.baseline, self.arrivals, now))
        return f

    def _diagnose(self, now):
        f = []
        age_tf = self._age('tf', now)
        if age_tf is None:
            f.append(Finding('ERROR', 'TF_NONE', 'no /tf received - sim/driver not running or wrong namespace/domain'))
        elif age_tf > self.stale_s:
            f.append(Finding('ERROR', 'TF_FREEZE',
                             f'/tf silent for {age_tf:.1f}s - TF froze; costmap filters go inert and Nav2 stalls'))
        else:
            for (p, c), t in self.tf_edges.items():
                a = now - t
                if a > self.stale_s:
                    f.append(Finding('ERROR', 'TF_EDGE_STALE',
                                     f'transform {p}->{c} stopped {a:.1f}s ago while other TF is alive'))

        has_map_odom = any(p.endswith('map') and c.endswith('odom') for p, c in set(self.tf_edges) | self.static_edges)
        if age_tf is not None and not has_map_odom:
            f.append(Finding('WARN', 'NO_MAP_ODOM', 'no map->odom transform - SLAM/AMCL not localizing'))

        if self.scan_frame and age_tf is not None and not self._frame_reaches_base(self.scan_frame):
            f.append(Finding('ERROR', 'SCAN_FRAME_NO_TF',
                             f'scan frame "{self.scan_frame}" has no TF connection to the robot base - '
                             'costmaps and SLAM cannot transform the scan'))

        for stream, label in (('odom', 'odometry'), ('scan', 'laser scan')):
            a = self._age(stream, now)
            if a is None:
                f.append(Finding('WARN', f'{stream.upper()}_NONE', f'no {label} messages seen'))
            elif a > self.stale_s:
                f.append(Finding('ERROR', f'{stream.upper()}_STALE', f'{label} silent for {a:.1f}s'))

        a = self._age('local_costmap', now)
        if a is not None and a > self.stale_s:
            f.append(Finding('ERROR', 'COSTMAP_STALE',
                             f'local costmap not updating ({a:.1f}s) - usually a TF or scan problem upstream'))

        if self.amcl_cov is not None and self.amcl_cov > AMCL_COV_MAX:
            f.append(Finding('WARN', 'AMCL_UNCERTAIN',
                             f'AMCL pose covariance {self.amcl_cov:.2f} m^2 (> {AMCL_COV_MAX}) - robot is poorly localized, set /initialpose or check the map/scan'))

        for n, st in self.lifecycle.items():
            if st is None:
                f.append(Finding('WARN', 'NODE_MISSING', f'{n}: lifecycle service unreachable (not launched?)'))
            elif st != 'active':
                if self.started is not None and now - self.started < self.startup_s:
                    f.append(Finding('WARN', 'NODE_STARTING', f'{n} is "{st}" - Nav2 may still be starting up'))
                else:
                    f.append(Finding('ERROR', 'NODE_INACTIVE', f'{n} is "{st}", not active - lifecycle manager stuck'))

        if len(self.goal_times) >= self.flap_count:
            f.append(Finding('WARN', 'GOAL_FLAPPING',
                             f'{len(self.goal_times)} goals cancelled/aborted in {self.flap_window_s:.0f}s - explorer/goal source is oscillating (add hysteresis)'))

        if self.goal_active:
            a = self._age('cmd_vel', now)
            moving = self.last_cmd_nonzero is not None and now - self.last_cmd_nonzero < self.stale_s
            if a is None:
                f.append(Finding('ERROR', 'NO_CMD_VEL', 'goal active but nothing publishes cmd_vel - controller not producing output'))
            elif a >= self.stale_s:
                # it published before and then went silent (hung / paused / crashed controller): that is NOT a zero command
                f.append(Finding('ERROR', 'NO_CMD_VEL', f'goal active but cmd_vel has been silent for {a:.1f}s - controller stopped producing output'))
            elif not moving:
                f.append(Finding('ERROR', 'CMD_ZERO', 'goal active but cmd_vel is zero - planner/controller failed or path blocked'))
            nav_a = self._age('cmd_vel_nav', now)
            # follow_running False: a recovery or replanning, the controller is not meant to publish
            if nav_a is not None and self.follow_running is not False and not any(x.code == 'NO_CMD_VEL' for x in f):
                if self.follow_running:      # known from the BT log: the shorter threshold is safe
                    since, limit = self.follow_since, self.nav_stale_s
                else:                        # no BT log seen: conservative, a new goal gets stale_s to start
                    since, limit = self.goal_since, self.stale_s
                if since is not None:
                    nav_a = min(nav_a, now - since)
                if nav_a >= limit:
                    f.append(Finding('ERROR', 'NO_CMD_VEL', f'goal active but the controller output (cmd_vel_nav) has been silent for {nav_a:.1f}s - '
                                     'controller_server hung or paused (a smoother may still be repeating cmd_vel)'))

        if self.goal_active and self.goal_since is not None and now - self.goal_since >= self.progress_s:
            win = [p for p in self.poses if now - p[0] <= self.progress_s]
            if len(win) >= 2 and win[0][0] - (now - self.progress_s) < self.stale_s:
                moved, turned = self._moved(win)
                if moved < self.progress_m and turned < self.progress_yaw:
                    f.append(Finding('WARN', 'NO_PROGRESS',
                                     f'goal active for {now - self.goal_since:.0f}s but robot moved only {moved:.2f}m in {self.progress_s:.0f}s - stuck, recovering, or blocked'))

        if self.cmd_subscribers == 0 and self.last_cmd_nonzero is not None \
                and now - self.last_cmd_nonzero < self.stale_s:
            f.append(Finding('ERROR', 'NO_CMD_SUBSCRIBER',
                             'cmd_vel is being published but nothing subscribes - base driver listens on a different '
                             'topic (e.g. cmd_vel_smoothed) or is not running'))

        if self.cmd_since is not None and now - self.cmd_since >= DRIVE_S \
                and self.last_cmd_nonzero is not None and now - self.last_cmd_nonzero < self.stale_s:
            win = [p for p in self.poses if now - p[0] <= DRIVE_S]
            if len(win) >= 2 and win[0][0] - (now - DRIVE_S) < self.stale_s:
                moved, turned = self._moved(win)
                if moved < self.progress_m and turned < self.progress_yaw:
                    f.append(Finding('ERROR', 'CMD_NO_MOTION',
                                     f'cmd_vel non-zero for {now - self.cmd_since:.0f}s but odom moved {moved:.2f}m - '
                                     'chassis/driver not obeying, e-stop, or odom frozen'))

        if (not self.ever_goal and self.last_cmd_nonzero is None and age_tf is not None
                and age_tf <= self.stale_s and self.started is not None
                and now - self.started >= max(IDLE_S, self.startup_s)):
            f.append(Finding('WARN', 'IDLE_NO_GOALS',
                             f'stack alive for {now - self.started:.0f}s but no goal was ever sent and the robot '
                             'was never commanded to move - goal source may have quit early'))

        if self.scan_stamp is not None and self.tf_stamp is not None \
                and self.scan_stamp - self.tf_stamp > SCAN_LEAD_S:
            f.append(Finding('WARN', 'SCAN_TF_AHEAD',
                             f'scan stamps are {self.scan_stamp - self.tf_stamp:.1f}s ahead of the newest odom TF - '
                             'transform lookups at the scan time will fail (extrapolation into the future)'))

        win = [m for m in self.map_hist if now - m[0] <= MAP_WIN_S]
        active = len([t for t in self.cmd_secs if now - t <= MAP_WIN_S])
        if (len(win) >= 2 and win[-1][0] - win[0][0] >= MAP_WIN_S * 0.75 and active >= MAP_ACTIVE_S
                and win[-1][1] - win[0][1] < max(20, 0.002 * max(win[-1][1], 1))):
            f.append(Finding('WARN', 'MAP_STALL',
                             f'cmd_vel active {active}s of the last {MAP_WIN_S:.0f}s but /map free cells '
                             f'went {win[0][1]} -> {win[-1][1]} - SLAM is not integrating scans'))

        if self.sim_clock_t is not None and now - self.sim_clock_t > max(self.stale_s, SIM_CLOCK_FRESH_S):
            f.append(Finding('ERROR', 'CLOCK_STALLED',
                             f'/clock silent for {now - self.sim_clock_t:.1f}s - simulator paused, frozen or '
                             'its bridge stopped; every use_sim_time node is stuck'))

        if self.skew is not None and self.skew > SKEW_S:
            f.append(Finding('WARN', 'CLOCK_SKEW',
                             f'/tf stamps differ from this machine\'s clock by {self.skew:.1f}s - clock skew or '
                             'use_sim_time mismatch ("extrapolation into the future/past" errors)'))

        infl, rad = self.params.get('inflation_radius'), self.params.get('robot_radius')
        if infl is not None and rad is not None and infl < rad:
            f.append(Finding('WARN', 'INFLATION_SMALL',
                             f'inflation_radius {infl:.2f}m < robot_radius {rad:.2f}m - planner can route through '
                             'cells the robot cannot fit, causing aborts near obstacles'))

        if len([t for t in self.recoveries if now - t <= RECOVERY_WINDOW_S]) >= RECOVERY_COUNT:
            f.append(Finding('WARN', 'RECOVERY_LOOP',
                             f'{len(self.recoveries)} recovery behaviours in {RECOVERY_WINDOW_S:.0f}s - robot is '
                             'spinning/backing up repeatedly instead of making progress'))

        for info, fi in self.filter_infos.items():
            mask = fi['mask']
            if (mask and mask not in self.filter_mask_seen and self.filter_pubs.get(mask) == 0
                    and now - fi['since'] >= self.startup_s):
                f.append(Finding('WARN', 'FILTER_NO_MASK',
                                 f'costmap filter {info} points at mask topic {mask} but nothing publishes it - '
                                 'the filter is inert (keepout / speed zones are not applied)'))
        for mask, (invalid, slow) in self.filter_stats.items():
            if invalid > 0:
                f.append(Finding('WARN', 'FILTER_BAD_LIMIT',
                                 f'speed mask {mask}: {100 * invalid:.0f}% of cells give a limit outside (0, 100], '
                                 'which Nav2 treats as no limit'))
            if slow >= SLOW_SHARE:
                f.append(Finding('WARN', 'FILTER_MOSTLY_SLOW',
                                 f'speed mask {mask}: {100 * slow:.0f}% of cells are limited below {SLOW_PCT}% speed '
                                 '(inverted mask?)'))
        for mask, pct in self.filter_min_limit.items():
            blocked = []
            for hi, lo, what in (('max_speed_xy', 'min_speed_xy', 'drive slower than'),
                                 ('max_vel_theta', 'min_speed_theta', 'turn slower than')):
                top, bottom = self.dwb.get(hi), self.dwb.get(lo)
                if top and bottom and top > 0 and top * pct / 100.0 < bottom - 1e-6:
                    blocked.append(f'{hi} drops to {top * pct / 100.0:.3f}, under {lo} {bottom:g}')
            if blocked:     # DWB scales the max speeds with the limit but never the min speeds
                both = len(blocked) == 2
                f.append(Finding('WARN', 'FILTER_BELOW_MIN_SPEED',
                                 f'speed mask {mask} can limit to {pct:.0f}%: ' + '; '.join(blocked)
                                 + (' - DWB rejects every sample there' if both else ' - DWB loses motions there')))
        if (self.filters_configured and not self.filter_infos and self.filter_info_pubs == 0
                and self.started is not None and now - self.started >= self.startup_s):
            f.append(Finding('WARN', 'FILTER_NO_INFO',
                             f'costmap lists filters {self.filters_configured} but no CostmapFilterInfo is published - '
                             'the filters are inert'))

        for topic, since in self.speed_unheard.items():
            if now - since >= self.startup_s:
                f.append(Finding('WARN', 'SPEED_LIMIT_UNHEARD',
                                 f'speed limit is published on {topic} but controller_server listens on '
                                 f'{", ".join(self.speed_heard)} - speed zones have no effect'))

        for code, (msg, times) in self.logs.items():
            recent = [t for t in times if now - t <= LOG_WINDOW_S]
            if recent:
                f.append(Finding('WARN', code, f'{len(recent)}x in the last {LOG_WINDOW_S:.0f}s: {msg[:140]}'))

        if self.lifecycle and all(v is None for v in self.lifecycle.values()) and not self.last:
            f.append(Finding('WARN', 'NO_NAV2_VISIBLE',
                             'no Nav2 nodes or topics visible - check ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY, RMW '
                             'implementation and --ns match the running stack'))

        if not any(x.level == 'ERROR' for x in f):
            f.append(Finding('OK', 'HEALTHY', 'no stall causes detected'))
        return f


SIM_CLOCK_FRESH_S = 2.0   # a /clock message newer than this means a simulator is driving time


def reference_time(wall_now, sim_time=None, sim_age=None):
    """Time that TF stamps should be compared with: the wall clock without /clock, sim time while it is live,
    and None once it has gone quiet (sim stamps cannot be compared with either then)."""
    if sim_time is None or sim_age is None:
        return wall_now
    return sim_time if sim_age < SIM_CLOCK_FRESH_S else None


def tf_belongs_to(ns, parent, child):
    """True if a transform on the shared /tf touches robot `ns` (frames prefixed `ns/`)."""
    pre = ns.strip('/') + '/'
    return parent.lstrip('/').startswith(pre) or child.lstrip('/').startswith(pre)


def fmt(findings, hints=False):
    icon = {'ERROR': '\033[31mERROR\033[0m', 'WARN': '\033[33mWARN \033[0m', 'OK': '\033[32mOK   \033[0m'}
    out = []
    for x in findings:
        out.append(f'{icon[x.level]} [{x.code}] {x.msg}')
        if hints and x.code in HINTS and x.level != 'OK':
            out.append(f'      -> {HINTS[x.code]}')
    return '\n'.join(out)


def readiness_verdict(findings):
    errors = [x.code for x in findings if x.level == 'ERROR']
    warns = [x.code for x in findings if x.level == 'WARN']
    if errors:
        return f'\033[31mNOT READY\033[0m - {len(errors)} blocking: {", ".join(errors)}'
    return '\033[32mREADY\033[0m' + (f' - {len(warns)} warning(s): {", ".join(warns)}' if warns else '')


def to_json(findings, ns, now_wall):
    return json.dumps({'time': now_wall, 'ns': ns,
                       'ok': not any(x.level == 'ERROR' for x in findings),
                       'findings': [dict(x.__dict__, hint=HINTS.get(x.code)) for x in findings]})


def yaw_of(q):
    """Heading (rad) from a geometry_msgs quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def diagnostics_topic(ns):
    """/diagnostics for a single robot, <ns>/diagnostics when --ns is given."""
    return (('/' + ns.strip('/')) if ns else '') + '/diagnostics'


def speed_limit_graph(node, ns):
    """(SpeedLimit topics with a publisher, SpeedLimit topics controller_server subscribes to) for one robot."""
    root = ns or '/'
    published, heard = [], []
    for topic, types in node.get_topic_names_and_types():
        if 'nav2_msgs/msg/SpeedLimit' not in types:
            continue
        if any(i.node_namespace == root or i.node_namespace.startswith(f'{ns}/')
               for i in node.get_publishers_info_by_topic(topic)):
            published.append(topic)
        if any(i.node_name == 'controller_server' and i.node_namespace == root
               for i in node.get_subscriptions_info_by_topic(topic)):
            heard.append(topic)
    return published, heard


def main(argv=None):
    try:
        import rclpy
    except ImportError:
        # `pip install nav2-doctor` brings the doctor but not rclpy: that only comes from a ROS 2 install
        print('nav2_doctor needs a ROS 2 environment (rclpy was not found). Install ROS 2 Humble or Jazzy and run '
              '`source /opt/ros/<distro>/setup.bash` first.', file=sys.stderr)
        raise SystemExit(2)
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile
    from tf2_msgs.msg import TFMessage
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from action_msgs.msg import GoalStatusArray
    from lifecycle_msgs.srv import GetState
    from rcl_interfaces.srv import GetParameters
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--version', action='version', version=f'nav2_doctor {__version__}')
    ap.add_argument('--ns', default='', help='robot namespace, e.g. robot1')
    ap.add_argument('--period', type=float, default=2.0, help='report period (s)')
    ap.add_argument('--startup-grace', type=float, default=STARTUP_S,
                    help='seconds before inactive lifecycle nodes count as errors')
    ap.add_argument('--stale', type=float, default=STALE_S, help='seconds of silence before a stream is stale')
    ap.add_argument('--flap-count', type=int, default=FLAP_COUNT, help='cancelled/aborted goals that count as flapping')
    ap.add_argument('--flap-window', type=float, default=FLAP_WINDOW_S, help='flapping look-back window (s)')
    ap.add_argument('--progress-time', type=float, default=PROGRESS_S,
                    help='seconds with an active goal and no movement before NO_PROGRESS')
    ap.add_argument('--progress-dist', type=float, default=PROGRESS_M, help='metres that count as movement')
    ap.add_argument('--progress-yaw', type=float, default=PROGRESS_YAW,
                    help='radians of heading change that count as movement (rotating in place) [0.2]')
    ap.add_argument('--json', action='store_true', help='print one JSON object per report')
    ap.add_argument('--hints', action='store_true', help='print a suggested next step under each finding')
    ap.add_argument('--diagnostics', action='store_true',
                    help='also publish findings as diagnostic_msgs/DiagnosticArray on /diagnostics (<ns>/diagnostics with --ns)')
    ap.add_argument('--once', type=float, default=0.0, help='watch N seconds, print one report, exit')
    ap.add_argument('--learn', type=float, default=0.0, metavar='SECS',
                    help='watch a healthy robot for SECS, write per-stream normal rates to --baseline-file, exit')
    ap.add_argument('--baseline-file', default='nav2_baseline.json', help='where --learn writes, and --baseline reads')
    ap.add_argument('--expect', action='append', default=[], metavar='STREAM:HZ',
                    help='expected rate for a stream, e.g. --expect scan:10 (repeatable); flags it when far slower/faster, '
                         'no --learn needed, overrides a learned value')
    ap.add_argument('--baseline', action='store_true', help='also flag streams far slower or faster than the learned baseline')
    ap.add_argument('--ready', nargs='?', type=float, const=10.0, default=None, metavar='SECS',
                    help='pre-flight: watch SECS (default 10), print hints and a READY / NOT READY verdict; '
                         'exit 1 if not ready (lifecycle nodes must already be active)')
    args, ros_args = ap.parse_known_args(argv)
    if args.ready is not None:
        args.once, args.hints, args.startup_grace = args.ready, True, 0.0

    rclpy.init(args=ros_args)
    node = Node('nav2_doctor')
    d = Doctor(args.startup_grace, args.stale, args.flap_window, args.flap_count,
               args.progress_time, args.progress_dist, args.progress_yaw)
    ns = ('/' + args.ns.strip('/')) if args.ns else ''
    if args.baseline:
        from nav2_doctor import baseline as bl
        try:
            d.baseline = bl.load(args.baseline_file)
        except ValueError as e:
            ap.error(str(e))
    if args.expect:
        from nav2_doctor import baseline as bl
        try:
            expected = bl.parse_expect(args.expect)
        except ValueError as e:
            ap.error(str(e))
        d.baseline = {**(d.baseline or {}), **expected}
        print('[nav2_doctor] expecting rates: ' + ', '.join(f'{k}={v["hz"]:g} Hz' for k, v in sorted(expected.items())))
    if args.learn:
        args.once = args.learn
    clock = time.monotonic
    sd = qos_profile_sensor_data
    rel = QoSProfile(depth=100)

    sim_clock = {}   # latest /clock message (sim time) and when it arrived; set only when a sim publishes /clock

    def on_clock(m):
        sim_clock['t'] = m.clock.sec + m.clock.nanosec / 1e9
        sim_clock['seen'] = time.monotonic()
        d.sim_clock(clock())

    def now_ref():
        # Under Gazebo the stamps are sim time, so compare against /clock, not the wall clock
        age = time.monotonic() - sim_clock['seen'] if 'seen' in sim_clock else None
        return reference_time(node.get_clock().now().nanoseconds / 1e9, sim_clock.get('t'), age)

    def on_tf(m, shared=False):
        for t in m.transforms:
            # Stacks that remap every robot's TF onto one global /tf use frame prefixes (robot1/odom ...):
            # on that shared topic keep only this robot's frames.
            if shared and not tf_belongs_to(args.ns, t.header.frame_id, t.child_frame_id):
                continue
            st = t.header.stamp
            skew = None
            ref = now_ref()
            if ref is not None and (st.sec or st.nanosec):   # zero stamp = unstamped, nothing to compare
                skew = ref - (st.sec + st.nanosec / 1e9)
            d.tf(t.header.frame_id, t.child_frame_id, clock(), skew)
            if t.child_frame_id.lstrip('/').endswith('base_link') and t.header.frame_id.lstrip('/').endswith('odom') \
                    and (st.sec or st.nanosec):
                d.stamps(tf=st.sec + st.nanosec / 1e9)
    from rosgraph_msgs.msg import Clock
    from rcl_interfaces.msg import Log

    def on_rosout(m):
        base = m.name.split('.')[0].split('/')[-1]
        if m.level >= 30 and any(n in (m.name or '') or n == base for n in LOG_NODES):
            d.log(m.msg, clock())
    node.create_subscription(Log, '/rosout', on_rosout, QoSProfile(depth=200))
    node.create_subscription(Clock, '/clock', on_clock, sd)
    node.create_subscription(TFMessage, f'{ns}/tf', on_tf, rel)
    if ns:   # shared global /tf with frame prefixes (multi-robot): see on_tf
        node.create_subscription(TFMessage, '/tf', lambda m: on_tf(m, True), rel)

    def on_tf_static(m, shared=False):
        for t in m.transforms:
            if shared and not tf_belongs_to(args.ns, t.header.frame_id, t.child_frame_id):
                continue
            d.tf_static(t.header.frame_id, t.child_frame_id)
    from rclpy.qos import DurabilityPolicy as _Dur
    latched = QoSProfile(depth=100, durability=_Dur.TRANSIENT_LOCAL)
    node.create_subscription(TFMessage, f'{ns}/tf_static', on_tf_static, latched)
    if ns:
        node.create_subscription(TFMessage, '/tf_static', lambda m: on_tf_static(m, True), latched)
    node.create_subscription(Odometry, f'{ns}/odom', lambda m: d.pose(m.pose.pose.position.x, m.pose.pose.position.y, clock(), yaw_of(m.pose.pose.orientation)), sd)
    node.create_subscription(PoseWithCovarianceStamped, f'{ns}/amcl_pose',
                             lambda m: d.amcl(m.pose.covariance[0], m.pose.covariance[7], clock()), rel)
    node.create_subscription(LaserScan, f'{ns}/scan', lambda m: (d.seen('scan', clock()), d.stamps(scan=m.header.stamp.sec + m.header.stamp.nanosec / 1e9, scan_frame=m.header.frame_id)), sd)
    node.create_subscription(OccupancyGrid, f'{ns}/local_costmap/costmap',
                             lambda m: d.seen('local_costmap', clock()), rel)
    from rclpy.qos import DurabilityPolicy
    mapq = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, f'{ns}/map',
                             lambda m: d.map_msg(sum(1 for v in m.data if v == 0), clock()), mapq)
    node.create_subscription(Twist, f'{ns}/cmd_vel',
                             lambda m: d.cmd(m.linear.x, m.angular.z, clock()), sd)
    node.create_subscription(Twist, f'{ns}/cmd_vel_nav', lambda m: d.cmd_nav(clock()), sd)

    seen_active, counted = set(), set()

    def on_status(m):
        # Flapping = goals CANCELED/ABORTED before finishing; a new goal after SUCCEEDED is normal exploring.
        active = False
        for s in m.status_list:
            gid = bytes(s.goal_info.goal_id.uuid).hex()
            if s.status in (1, 2):                       # ACCEPTED / EXECUTING
                seen_active.add(gid)
                active = True
            elif s.status in (5, 6) and gid in seen_active and gid not in counted:   # CANCELED / ABORTED
                counted.add(gid)
                d.goal(clock())
        d.set_goal_active(active, clock())
    node.create_subscription(GoalStatusArray, f'{ns}/navigate_to_pose/_action/status', on_status, rel)

    try:
        from nav2_msgs.msg import BehaviorTreeLog

        def on_bt(m):
            for e in m.event_log:
                d.bt_status(e.node_name, e.current_status, clock())
                if e.current_status == 'RUNNING' and e.previous_status != 'RUNNING':
                    d.recovery(e.node_name, clock())
        node.create_subscription(BehaviorTreeLog, f'{ns}/behavior_tree_log', on_bt, rel)
    except ImportError:    # nav2_msgs not installed: skip the recovery-loop check
        pass

    clients = {n: node.create_client(GetState, f'{ns}/{n}/get_state') for n in NAV2_NODES}

    def poll_lifecycle():
        for n, c in clients.items():
            if not c.service_is_ready():
                d.lifecycle[n] = None
                continue
            fut = c.call_async(GetState.Request())
            fut.add_done_callback(lambda f, n=n: d.lifecycle.__setitem__(n, f.result().current_state.label))
    node.create_timer(3.0, poll_lifecycle)

    def poll_cmd_subscribers():
        infos = node.get_subscriptions_info_by_topic(f'{ns}/cmd_vel')
        d.cmd_subscribers = sum(1 for i in infos if i.node_name != node.get_name())
    node.create_timer(1.0, poll_cmd_subscribers)

    param_cli = node.create_client(GetParameters, f'{ns}/local_costmap/local_costmap/get_parameters')
    names = ['inflation_layer.inflation_radius', 'robot_radius', 'footprint']

    def poll_params():
        if not param_cli.service_is_ready():
            return
        req = GetParameters.Request(names=names)

        def done(f):
            try:
                vals = f.result().values
            except Exception:
                return
            for key, v in zip(('inflation_radius', 'robot_radius', 'footprint'), vals):
                if v.type == 3:        # PARAMETER_DOUBLE
                    d.params[key] = v.double_value
                elif v.type == 4 and key == 'footprint' and v.string_value.strip('[] '):   # PARAMETER_STRING
                    r = inscribed_radius(v.string_value)
                    if r is not None:
                        d.params['robot_radius'] = r
        param_cli.call_async(req).add_done_callback(done)
    node.create_timer(10.0, poll_params)

    dwb_cli = node.create_client(GetParameters, f'{ns}/controller_server/get_parameters')
    dwb_keys = ('max_speed_xy', 'min_speed_xy', 'max_vel_theta', 'min_speed_theta')

    def poll_dwb():
        if not dwb_cli.service_is_ready():
            return

        def done(f):
            try:
                vals = f.result().values
            except Exception:
                return
            # a non-DWB controller has no such parameters (type 0): the check then stays quiet
            d.dwb = {k: v.double_value for k, v in zip(dwb_keys, vals) if v.type == 3}
        dwb_cli.call_async(GetParameters.Request(names=[f'FollowPath.{k}' for k in dwb_keys])).add_done_callback(done)
    node.create_timer(10.0, poll_dwb)

    # costmap filters (keepout, speed zones): find the CostmapFilterInfo topics, then check each mask has a publisher
    filter_subs = {}
    try:
        from nav2_msgs.msg import CostmapFilterInfo
    except ImportError:    # nav2_msgs not installed: skip the filter checks
        CostmapFilterInfo = None

    node.create_timer(2.0, lambda: d.speed_limits(*speed_limit_graph(node, ns), clock()))

    def discover_filters():
        if CostmapFilterInfo is None:
            return
        info_topics = [t for t, types in node.get_topic_names_and_types() if 'nav2_msgs/msg/CostmapFilterInfo' in types]
        d.filter_info_pubs = sum(len(node.get_publishers_info_by_topic(t)) for t in info_topics)
        for t in info_topics:
            if t not in filter_subs:
                def on_info(m, t=t):
                    mask = m.filter_mask_topic
                    mask = mask if mask.startswith('/') or not mask else f'{ns}/{mask}'
                    d.filter_info(t, mask, clock(), m.type, m.base, m.multiplier)
                    if mask and mask not in filter_subs:
                        filter_subs[mask] = node.create_subscription(
                            OccupancyGrid, mask, lambda g, mask=mask, t=t: d.filter_mask(mask, g.data, t), mapq)
                filter_subs[t] = node.create_subscription(CostmapFilterInfo, t, on_info, mapq)
        for info in d.filter_infos.values():
            if info['mask']:
                d.filter_pubs[info['mask']] = len(node.get_publishers_info_by_topic(info['mask']))
    node.create_timer(2.0, discover_filters)

    fparam_cli = node.create_client(GetParameters, f'{ns}/local_costmap/local_costmap/get_parameters')

    def poll_filters():
        if not fparam_cli.service_is_ready():
            return

        def done(f):
            try:
                v = f.result().values[0]
            except Exception:
                return
            if v.type == 9:        # PARAMETER_STRING_ARRAY
                d.filters_configured = list(v.string_array_value)
        fparam_cli.call_async(GetParameters.Request(names=['filters'])).add_done_callback(done)
    node.create_timer(10.0, poll_filters)

    diag_pub = node.create_publisher(DiagnosticArray, diagnostics_topic(args.ns), 10) if args.diagnostics else None
    lvl = {'OK': DiagnosticStatus.OK, 'WARN': DiagnosticStatus.WARN, 'ERROR': DiagnosticStatus.ERROR}

    def publish_diag(found):
        msg = DiagnosticArray()
        msg.header.stamp = node.get_clock().now().to_msg()
        for x in found:
            msg.status.append(DiagnosticStatus(level=lvl[x.level], name=f'nav2_doctor/{x.code}',
                                               message=x.msg, hardware_id=args.ns or 'nav2'))
        diag_pub.publish(msg)

    exit_code = [0]

    def report():
        found = d.diagnose(clock())
        if args.json:
            print(to_json(found, args.ns, time.time()), flush=True)
        else:
            print(f'\n--- nav2_doctor {time.strftime("%H:%M:%S")} ns="{args.ns}" ---')
            print(fmt(found, args.hints), flush=True)
        if diag_pub:
            publish_diag(found)
        return found
    start = clock()

    def tick():
        if args.once and clock() - start >= args.once:
            if args.learn:
                from nav2_doctor import baseline as bl
                learned = bl.learn(d.arrivals)
                bl.save(learned, args.baseline_file)
                print(f'learned {len(learned)} streams -> {args.baseline_file}: ' +
                      ', '.join(f'{k} {v["hz"]:g} Hz' for k, v in sorted(learned.items())), flush=True)
                raise SystemExit
            found = report()
            errors = [x for x in found if x.level == 'ERROR']
            exit_code[0] = 1 if errors else 0
            if args.ready is not None and not args.json:
                print(readiness_verdict(found), flush=True)
            raise SystemExit
        if not args.once:
            report()
    node.create_timer(args.once or args.period, tick)
    node.get_logger().info(f'watching {ns or "/"} - Ctrl+C to stop')
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return exit_code[0]


if __name__ == '__main__':
    raise SystemExit(main())
