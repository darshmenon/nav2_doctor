"""Run the real CLI against a synthetic ROS graph (no sim). Needs a sourced ROS 2 env."""
import os
import re
import subprocess
import sys
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from action_msgs.msg import GoalStatus, GoalStatusArray  # noqa: E402
from geometry_msgs.msg import Twist, TransformStamped  # noqa: E402
from lifecycle_msgs.srv import GetState  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), '..')
NODES = ['controller_server', 'planner_server', 'bt_navigator',
         'behavior_server', 'smoother_server', 'waypoint_follower']
_domain = [140]


class FakeNav2(Node):
    """cfg keys: ns, stop_odom_tf/stop_all_tf/stop_odom/stop_scan (seconds), inactive (node name),
    goal ('none'|'executing'|'flap'|'sequence'), cmd ('zero'|'move')"""

    def __init__(self, cfg):
        super().__init__('fake_nav2')
        self.cfg, self.t0 = cfg, time.monotonic()
        ns = ('/' + cfg['ns']) if cfg.get('ns') else ''
        self.tf_pub = self.create_publisher(TFMessage, f'{ns}/tf', 100)
        self.odom_pub = self.create_publisher(Odometry, f'{ns}/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, f'{ns}/scan', 10)
        self.cmd_pub = self.create_publisher(Twist, f'{ns}/cmd_vel', 10)
        if cfg.get('subscriber', True):   # stands in for the base driver
            self.create_subscription(Twist, f'{ns}/cmd_vel', lambda m: None, 10)
        self.st_pub = self.create_publisher(GoalStatusArray, f'{ns}/navigate_to_pose/_action/status', 10)
        for n in NODES:
            self.create_service(GetState, f'{ns}/{n}/get_state', lambda rq, rs, n=n: self._state(n, rs))
        self.n = 0
        self.create_timer(0.05, self.tick)
        self.create_timer(0.5, self.goals)

    def _state(self, n, rs):
        rs.current_state.label = 'inactive' if self.cfg.get('inactive') == n else 'active'
        return rs

    def alive(self, key):
        stop = self.cfg.get(key)
        return stop is None or time.monotonic() - self.t0 < stop

    def tf(self, p, c):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = p, c
        return t

    def tick(self):
        edges = []
        if self.alive('stop_all_tf'):
            edges.append(self.tf('map', 'odom'))
            if self.alive('stop_odom_tf'):
                edges.append(self.tf('odom', 'base_link'))
        if edges:
            self.tf_pub.publish(TFMessage(transforms=edges))
        if self.alive('stop_odom'):
            od = Odometry()
            if self.cfg.get('cmd') == 'move' and not self.cfg.get('frozen_odom'):
                od.pose.pose.position.x = 0.2 * (time.monotonic() - self.t0)   # follows the commanded 0.2 m/s
            self.odom_pub.publish(od)
        if self.alive('stop_scan'):
            self.scan_pub.publish(LaserScan())
        tw = Twist()
        if self.cfg.get('cmd') == 'move':
            tw.linear.x = 0.2
        self.cmd_pub.publish(tw)

    def goals(self):
        mode = self.cfg.get('goal', 'none')
        if mode == 'none':
            return
        def mk(uid, status):
            s = GoalStatus()
            s.status = status
            s.goal_info.goal_id.uuid = [uid % 256] + [0] * 15
            return s
        self.n += 1
        if mode == 'executing':
            lst = [mk(1, GoalStatus.STATUS_EXECUTING)]
        else:   # 'flap': previous goal CANCELED; 'sequence': previous goal SUCCEEDED (normal exploring)
            end = GoalStatus.STATUS_CANCELED if mode == 'flap' else GoalStatus.STATUS_SUCCEEDED
            lst = [mk(self.n - 1, end), mk(self.n, GoalStatus.STATUS_EXECUTING)] if self.n > 1 else [mk(self.n, GoalStatus.STATUS_EXECUTING)]
        self.st_pub.publish(GoalStatusArray(status_list=lst))


def run(cfg, args=(), secs=8):
    _domain[0] += 1
    env = dict(os.environ, ROS_DOMAIN_ID=str(_domain[0]), PYTHONPATH=ROOT + os.pathsep + os.environ.get('PYTHONPATH', ''))
    os.environ['ROS_DOMAIN_ID'] = str(_domain[0])
    rclpy.init()
    node = FakeNav2(cfg)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        p = subprocess.run([sys.executable, '-m', 'nav2_doctor.doctor', '--once', str(secs),
                            '--startup-grace', '0', *args],
                           capture_output=True, text=True, env=env, cwd=ROOT, timeout=secs + 30)
    finally:
        ex.shutdown(); th.join(2); node.destroy_node(); rclpy.shutdown()
    out = re.sub(r'\x1b\[[0-9;]*m', '', p.stdout)
    return set(re.findall(r'\[([A-Z_]+)\]', out)), p.returncode, out


def test_healthy():
    codes, rc, out = run({'cmd': 'move'})
    assert 'HEALTHY' in codes and rc == 0, out


def test_odom_tf_edge_dies():
    codes, rc, out = run({'stop_odom_tf': 4})
    assert 'TF_EDGE_STALE' in codes and rc == 1, out


def test_all_tf_dies():
    codes, rc, out = run({'stop_all_tf': 4})
    assert 'TF_FREEZE' in codes and rc == 1, out


def test_odom_dies():
    codes, rc, out = run({'stop_odom': 4})
    assert 'ODOM_STALE' in codes and rc == 1, out


def test_scan_dies():
    codes, rc, out = run({'stop_scan': 4})
    assert 'SCAN_STALE' in codes and rc == 1, out


def test_inactive_node():
    codes, rc, out = run({'inactive': 'controller_server'})
    assert 'NODE_INACTIVE' in codes and rc == 1, out


def test_goal_flapping():
    codes, rc, out = run({'goal': 'flap', 'cmd': 'move'})
    assert 'GOAL_FLAPPING' in codes, out


def test_normal_exploring_is_not_flapping():
    codes, rc, out = run({'goal': 'sequence', 'cmd': 'move'})
    assert 'GOAL_FLAPPING' not in codes and 'HEALTHY' in codes and rc == 0, out


def test_cmd_zero_during_goal():
    codes, rc, out = run({'goal': 'executing', 'cmd': 'zero'})
    assert 'CMD_ZERO' in codes and rc == 1, out


def test_moving_during_goal_is_healthy():
    codes, rc, out = run({'goal': 'executing', 'cmd': 'move'})
    assert 'HEALTHY' in codes and 'CMD_ZERO' not in codes and rc == 0, out


def test_namespace():
    codes, rc, out = run({'ns': 'robot1', 'cmd': 'move'}, args=('--ns', 'robot1'))
    assert 'HEALTHY' in codes and rc == 0, out


def test_wrong_namespace_sees_nothing():
    codes, rc, out = run({'ns': 'robot1'}, args=('--ns', 'robot2'))
    assert 'TF_NONE' in codes and rc == 1, out


def test_version_flag():
    p = subprocess.run([sys.executable, '-m', 'nav2_doctor.doctor', '--version'],
                       capture_output=True, text=True, cwd=ROOT)
    assert p.stdout.startswith('nav2_doctor ') and p.returncode == 0


def test_no_cmd_subscriber():
    codes, rc, out = run({'cmd': 'move', 'subscriber': False})
    assert 'NO_CMD_SUBSCRIBER' in codes and rc == 1, out


def test_cmd_without_motion():
    codes, rc, out = run({'cmd': 'move', 'frozen_odom': True}, secs=9)
    assert 'CMD_NO_MOTION' in codes and rc == 1, out
