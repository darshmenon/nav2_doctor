"""Run the real CLI against a synthetic ROS graph (no sim). Needs a sourced ROS 2 env."""
import json
import math
import os
import re
import subprocess
import sys
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from action_msgs.msg import GoalStatus, GoalStatusArray  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus  # noqa: E402
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
    goal ('none'|'executing'|'flap'|'sequence'), cmd ('zero'|'move'|'spin'),
    diag_topics (list of topics to listen on for DiagnosticArray)"""

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
        self.diag = {}
        for topic in cfg.get('diag_topics', []):
            self.diag[topic] = []
            self.create_subscription(DiagnosticArray, topic, lambda m, t=topic: self.diag[t].append(m), 10)
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
            if self.cfg.get('cmd') == 'spin':   # rotates in place at the commanded 0.5 rad/s, x/y stay put
                yaw = 0.5 * (time.monotonic() - self.t0)
                od.pose.pose.orientation.z, od.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.odom_pub.publish(od)
        if self.alive('stop_scan'):
            self.scan_pub.publish(LaserScan())
        tw = Twist()
        if self.cfg.get('cmd') == 'move':
            tw.linear.x = 0.2
        if self.cfg.get('cmd') == 'spin':
            tw.angular.z = 0.5
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
    run.diag = node.diag
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


def test_rotating_in_place_is_progress():
    args = ('--progress-time', '4')
    codes, rc, out = run({'goal': 'executing', 'cmd': 'spin'}, args=args, secs=9)
    assert 'NO_PROGRESS' not in codes and 'CMD_NO_MOTION' not in codes, out
    codes, rc, out = run({'goal': 'executing', 'cmd': 'spin'}, args=args + ('--progress-yaw', '100'), secs=9)
    assert 'NO_PROGRESS' in codes, out   # with the yaw rule effectively off, the old behaviour comes back


def test_json_output_is_one_parseable_object_per_report():
    _, rc, out = run({'cmd': 'move'}, args=('--json',))
    lines = [ln for ln in out.splitlines() if ln.startswith('{')]
    assert lines, out
    rep = json.loads(lines[-1])
    assert rep['ns'] == '' and isinstance(rep['findings'], list), rep
    assert {'level', 'code', 'msg'} <= set(rep['findings'][0]), rep
    assert any(f['code'] == 'HEALTHY' for f in rep['findings']), rep
    assert rc == 0


def test_json_output_carries_errors_and_exit_code():
    _, rc, out = run({'stop_scan': 3}, args=('--json',), secs=8)
    rep = json.loads([ln for ln in out.splitlines() if ln.startswith('{')][-1])
    assert any(f['code'] == 'SCAN_STALE' and f['level'] == 'ERROR' for f in rep['findings']), rep
    assert rc == 1


def test_diagnostics_published_on_global_topic():
    run({'cmd': 'move', 'diag_topics': ['/diagnostics']}, args=('--diagnostics',))
    msgs = run.diag['/diagnostics']
    assert msgs, 'nothing published on /diagnostics'
    st = msgs[-1].status
    assert st and all(x.name.startswith('nav2_doctor/') for x in st)
    assert any(x.name == 'nav2_doctor/HEALTHY' and x.level == DiagnosticStatus.OK for x in st)


def test_diagnostics_use_the_namespace_with_ns():
    run({'ns': 'robot1', 'cmd': 'move', 'diag_topics': ['/robot1/diagnostics', '/diagnostics']},
        args=('--ns', 'robot1', '--diagnostics'))
    assert run.diag['/robot1/diagnostics'], 'nothing on /robot1/diagnostics'
    assert not run.diag['/diagnostics'], 'a namespaced doctor must not publish on the global /diagnostics'
    assert run.diag['/robot1/diagnostics'][-1].status[0].hardware_id == 'robot1'


def test_diagnostics_carry_error_levels():
    run({'stop_scan': 3, 'diag_topics': ['/diagnostics']}, args=('--diagnostics',), secs=8)
    st = run.diag['/diagnostics'][-1].status
    assert any(x.name == 'nav2_doctor/SCAN_STALE' and x.level == DiagnosticStatus.ERROR for x in st)
