import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from nav2_doctor.doctor import Doctor


def codes(d, now):
    return {f.code for f in d.diagnose(now)}


def healthy(now=0.0):
    d = Doctor()
    d.tf('map', 'odom', now); d.tf('odom', 'base_link', now)
    d.seen('odom', now); d.seen('scan', now)
    for n in ('controller_server', 'planner_server'):
        d.lifecycle[n] = 'active'
    return d


def test_healthy():
    assert codes(healthy(), 1.0) == {'HEALTHY'}


def test_no_tf():
    assert 'TF_NONE' in codes(Doctor(), 1.0)


def test_tf_freeze():
    assert 'TF_FREEZE' in codes(healthy(), 10.0)


def test_single_edge_stale():
    d = healthy()
    d.tf('odom', 'base_link', 9.5)   # odom->base alive, map->odom died
    assert 'TF_EDGE_STALE' in codes(d, 10.0)


def test_inactive_node():
    d = healthy(100.0); d.lifecycle['controller_server'] = 'inactive'
    d.seen('odom', 100.0); d.started = 0.0
    assert 'NODE_INACTIVE' in codes(d, 100.5)


def test_inactive_during_startup_is_warn():
    d = healthy(); d.lifecycle['controller_server'] = 'unconfigured'
    c = codes(d, 1.0)
    assert 'NODE_STARTING' in c and 'NODE_INACTIVE' not in c


def test_flapping():
    d = healthy()
    for t in (0.1, 0.2, 0.3, 0.4):
        d.goal(t)
    assert 'GOAL_FLAPPING' in codes(d, 1.0)


def test_cmd_zero_with_active_goal():
    d = healthy(); d.goal_active = True; d.cmd(0, 0, 0.9)
    assert 'CMD_ZERO' in codes(d, 1.0)


def test_moving_ok():
    d = healthy(); d.goal_active = True; d.cmd(0.2, 0, 0.9)
    assert 'CMD_ZERO' not in codes(d, 1.0)


def test_versions_match():
    import re
    from nav2_doctor.doctor import __version__
    root = os.path.join(os.path.dirname(__file__), '..')
    pkg = re.search(r'<version>(.*?)</version>', open(os.path.join(root, 'package.xml')).read()).group(1)
    setup = re.search(r"version='(.*?)'", open(os.path.join(root, 'setup.py')).read()).group(1)
    assert __version__ == pkg == setup
