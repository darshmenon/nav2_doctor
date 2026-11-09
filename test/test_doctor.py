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


# ---- edge cases -------------------------------------------------------
from nav2_doctor.doctor import fmt, STALE_S, FLAP_WINDOW_S


def test_leading_slash_frames_normalised():
    d = Doctor(); d.tf('/map', '/odom', 0.0); d.tf('/odom', '/base_link', 0.0)
    assert ('map', 'odom') in d.tf_edges and 'NO_MAP_ODOM' not in codes(d, 0.5)


def test_namespaced_frames_count_as_map_odom():
    d = Doctor(); d.tf('robot1/map', 'robot1/odom', 0.0)
    assert 'NO_MAP_ODOM' not in codes(d, 0.5)


def test_missing_map_odom_warns():
    d = Doctor(); d.tf('odom', 'base_link', 0.0)
    assert 'NO_MAP_ODOM' in codes(d, 0.5)


def test_stale_boundary():
    d = healthy()
    assert 'TF_FREEZE' not in codes(d, STALE_S - 0.01)
    assert 'TF_FREEZE' in codes(d, STALE_S + 0.5)


def test_flapping_window_expires():
    d = healthy()
    for t in (0.0, 0.1, 0.2, 0.3):
        d.goal(t)
    d.goal(FLAP_WINDOW_S + 5)           # old goals fall out of window
    assert 'GOAL_FLAPPING' not in codes(d, FLAP_WINDOW_S + 5.5)


def test_three_goals_not_flapping():
    d = healthy()
    for t in (0.1, 0.2, 0.3):
        d.goal(t)
    assert 'GOAL_FLAPPING' not in codes(d, 1.0)


def test_no_goal_zero_cmd_is_fine():
    d = healthy(); d.cmd(0, 0, 0.9)
    assert codes(d, 1.0) == {'HEALTHY'}


def test_goal_active_without_cmd_vel():
    d = healthy(); d.goal_active = True
    assert 'NO_CMD_VEL' in codes(d, 1.0)


def test_cmd_vel_stops_while_goal_active():
    d = healthy(); d.goal_active = True
    d.cmd(0.3, 0, 0.0); d.cmd(0, 0, 5.0)
    d.tf('map', 'odom', 5.0); d.tf('odom', 'base_link', 5.0); d.seen('odom', 5.0); d.seen('scan', 5.0)
    assert 'CMD_ZERO' in codes(d, 5.5)


def test_angular_only_counts_as_moving():
    d = healthy(); d.goal_active = True; d.cmd(0.0, 0.5, 0.9)
    assert 'CMD_ZERO' not in codes(d, 1.0)


def test_unreachable_node_warns_not_errors():
    d = healthy(); d.lifecycle['bt_navigator'] = None
    c = {f.code: f.level for f in d.diagnose(1.0)}
    assert c['NODE_MISSING'] == 'WARN' and 'HEALTHY' in c


def test_active_nodes_are_fine():
    d = healthy(); d.lifecycle['bt_navigator'] = 'active'
    assert codes(d, 1.0) == {'HEALTHY'}


def test_costmap_stale_only_when_previously_seen():
    d = healthy()
    assert 'COSTMAP_STALE' not in codes(d, 1.0)
    d.seen('local_costmap', 0.0); d.tf('map', 'odom', 5.0); d.tf('odom', 'base_link', 5.0)
    d.seen('odom', 5.0); d.seen('scan', 5.0)
    assert 'COSTMAP_STALE' in codes(d, 5.5)


def test_scan_and_odom_none_warn():
    d = Doctor(); d.tf('map', 'odom', 0.0)
    c = codes(d, 0.5)
    assert {'ODOM_NONE', 'SCAN_NONE'} <= c


def test_healthy_has_no_errors_or_warns():
    assert [f.level for f in healthy().diagnose(0.5)] == ['OK']


def test_fmt_renders_every_finding():
    d = Doctor()
    out = fmt(d.diagnose(1.0))
    assert 'TF_NONE' in out and out.count('\n') == len(d.diagnose(1.0)) - 1


def test_recovery_clears_errors():
    d = healthy()
    assert 'TF_FREEZE' in codes(d, 10.0)
    d.tf('map', 'odom', 10.0); d.tf('odom', 'base_link', 10.0); d.seen('odom', 10.0); d.seen('scan', 10.0)
    assert codes(d, 10.2) == {'HEALTHY'}


def test_custom_startup_grace():
    d = healthy(); d.startup_s = 0.0; d.started = 0.0
    d.lifecycle['controller_server'] = 'inactive'
    assert 'NODE_INACTIVE' in codes(d, 1.0)


# ---- NO_PROGRESS, tunable thresholds, JSON ------------------------------
def _goal_run(moving, progress_s=15.0):
    d = Doctor(progress_s=progress_s)
    d.set_goal_active(True, 0.0)
    for i in range(0, 31):
        d.pose(i * 0.5 if moving else 1.0, 0.0, float(i))
    return d


def test_no_progress_when_goal_active_and_robot_still():
    assert 'NO_PROGRESS' in [x.code for x in _goal_run(False).diagnose(30.0)]


def test_progress_when_robot_moves():
    assert 'NO_PROGRESS' not in [x.code for x in _goal_run(True).diagnose(30.0)]


def test_no_progress_needs_time_and_active_goal():
    d = Doctor()
    d.set_goal_active(True, 0.0)
    d.pose(0, 0, 0.0); d.pose(0, 0, 5.0)
    assert 'NO_PROGRESS' not in [x.code for x in d.diagnose(5.0)]
    d2 = _goal_run(False)
    d2.set_goal_active(False, 30.0)
    assert 'NO_PROGRESS' not in [x.code for x in d2.diagnose(30.0)]


def test_custom_stale_threshold():
    d = Doctor(stale_s=10.0)
    d.tf('map', 'odom', 0.0); d.tf('odom', 'base_link', 0.0)
    d.pose(0, 0, 0.0); d.seen('scan', 0.0)
    assert 'ODOM_STALE' not in [x.code for x in d.diagnose(5.0)]
    assert 'ODOM_STALE' in [x.code for x in d.diagnose(11.0)]


def test_custom_flap_count():
    d = Doctor(flap_count=2)
    d.goal(0.0); d.goal(1.0)
    assert 'GOAL_FLAPPING' in [x.code for x in d.diagnose(1.0)]


def test_to_json_roundtrip():
    import json
    from nav2_doctor.doctor import to_json
    out = json.loads(to_json(Doctor().diagnose(1.0), 'r1', 123.0))
    assert out['ns'] == 'r1' and out['ok'] is False
    assert out['findings'][0]['code'] == 'TF_NONE'


def test_no_cmd_subscriber():
    d = healthy(); d.cmd(0.2, 0, 0.9); d.cmd_subscribers = 0
    assert 'NO_CMD_SUBSCRIBER' in codes(d, 1.0)
    d.cmd_subscribers = 1
    assert 'NO_CMD_SUBSCRIBER' not in codes(d, 1.0)


def test_cmd_no_motion():
    d = healthy()
    for i in range(0, 80):
        t = i * 0.1
        d.tf('map', 'odom', t); d.tf('odom', 'base_link', t); d.seen('scan', t)
        d.cmd(0.2, 0, t); d.pose(0.0, 0.0, t)
    assert 'CMD_NO_MOTION' in codes(d, 7.9)


def test_cmd_with_motion_ok():
    d = healthy()
    for i in range(0, 80):
        t = i * 0.1
        d.tf('map', 'odom', t); d.tf('odom', 'base_link', t); d.seen('scan', t)
        d.cmd(0.2, 0, t); d.pose(t * 0.2, 0.0, t)
    assert 'CMD_NO_MOTION' not in codes(d, 7.9)


def test_clock_skew():
    d = healthy(); d.tf('odom', 'base_link', 1.0, skew=5.0)
    assert 'CLOCK_SKEW' in codes(d, 1.0)
    d.tf('odom', 'base_link', 1.0, skew=0.01)
    assert 'CLOCK_SKEW' not in codes(d, 1.0)


def test_inflation_small():
    d = healthy(); d.params = {'inflation_radius': 0.1, 'robot_radius': 0.3}
    assert 'INFLATION_SMALL' in codes(d, 1.0)


def test_nothing_visible_hint():
    d = Doctor(); d.lifecycle = {'controller_server': None}
    assert 'NO_NAV2_VISIBLE' in codes(d, 1.0)



def test_every_code_has_hint():
    import re
    from nav2_doctor.doctor import HINTS
    src = open(os.path.join(os.path.dirname(__file__), '..', 'nav2_doctor', 'doctor.py')).read()
    for code in set(re.findall(r"Finding\('(?:ERROR|WARN)', f?'([A-Z_]+)'", src)):
        assert code in HINTS, code


def test_fmt_hints_and_json():
    import json
    from nav2_doctor.doctor import fmt, to_json
    f = Doctor().diagnose(1.0)
    assert '->' in fmt(f, hints=True) and '->' not in fmt(f)
    assert json.loads(to_json(f, '', 0))['findings'][0]['hint']


def test_amcl_uncertain():
    d = healthy(); d.amcl(0.9, 0.1, 0.5)
    assert 'AMCL_UNCERTAIN' in codes(d, 1.0)


def test_amcl_confident():
    d = healthy(); d.amcl(0.05, 0.05, 0.5)
    assert 'AMCL_UNCERTAIN' not in codes(d, 1.0)


def test_recovery_loop():
    d = healthy()
    for t, n in ((1, 'Spin'), (5, 'BackUp'), (9, 'Wait'), (10, 'ComputePathToPose')):
        d.recovery(n, t)
    assert 'RECOVERY_LOOP' in codes(d, 10.5)


def test_recovery_loop_expires():
    d = healthy(200.0)
    for t in (1, 5, 9):
        d.recovery('Spin', t)
    d.recovery('Spin', 200.0)   # prunes old entries
    assert 'RECOVERY_LOOP' not in codes(d, 200.5)


def test_inscribed_radius():
    from nav2_doctor.doctor import inscribed_radius
    assert abs(inscribed_radius('[[0.3,0.2],[0.3,-0.2],[-0.3,-0.2],[-0.3,0.2]]') - 0.2) < 1e-9
    assert inscribed_radius('[]') is None and inscribed_radius('junk') is None


def test_readiness_verdict():
    from nav2_doctor.doctor import Finding, readiness_verdict
    assert 'NOT READY' in readiness_verdict([Finding('ERROR', 'TF_EDGE_STALE', 'x'), Finding('WARN', 'W', 'y')])
    assert 'TF_EDGE_STALE' in readiness_verdict([Finding('ERROR', 'TF_EDGE_STALE', 'x')])
    v = readiness_verdict([Finding('WARN', 'RECOVERY_LOOP', 'y')])
    assert 'READY' in v and 'NOT' not in v and 'RECOVERY_LOOP' in v
    assert 'READY' in readiness_verdict([Finding('OK', 'HEALTHY', 'ok')])


def _idle(d, upto, cmd=False, goal=False):
    for i in range(0, int(upto * 2)):
        t = i * 0.5
        d.tf('map', 'odom', t); d.tf('odom', 'base_link', t); d.seen('scan', t); d.pose(0.0, 0.0, t)
        if cmd:
            d.cmd(0.2, 0, t)
        if goal:
            d.set_goal_active(True, t)


def test_idle_no_goals_flagged():
    d = healthy()
    _idle(d, 70)
    assert 'IDLE_NO_GOALS' in codes(d, 70)


def test_idle_no_goals_not_before_grace():
    d = healthy()
    _idle(d, 20)
    assert 'IDLE_NO_GOALS' not in codes(d, 20)


def test_idle_no_goals_cleared_by_goal_or_motion():
    d = healthy(); _idle(d, 70, goal=True)
    assert 'IDLE_NO_GOALS' not in codes(d, 70)
    d = healthy(); _idle(d, 70, cmd=True)
    assert 'IDLE_NO_GOALS' not in codes(d, 70)


def _map_run(free_at, secs=70, cmd=True):
    d = healthy()
    for i in range(0, secs * 2):
        t = i * 0.5
        d.tf('map', 'odom', t); d.tf('odom', 'base_link', t); d.seen('scan', t); d.pose(0.0, 0.0, t)
        if cmd:
            d.cmd(0.0, 0.5, t)
        if i % 4 == 0:
            d.map_msg(free_at(t), t)
    return d


def test_map_stall_flagged_when_map_frozen_while_driving():
    d = _map_run(lambda t: 1319)
    assert 'MAP_STALL' in codes(d, 69.5)


def test_map_stall_not_flagged_when_map_grows():
    d = _map_run(lambda t: 1319 + int(t * 200))
    assert 'MAP_STALL' not in codes(d, 69.5)


def test_map_stall_not_flagged_when_robot_idle():
    d = _map_run(lambda t: 1319, cmd=False)
    assert 'MAP_STALL' not in codes(d, 69.5)


def test_scan_tf_ahead_flagged():
    d = healthy()
    d.tf('map', 'odom', 1.0); d.tf('odom', 'base_link', 1.0); d.seen('scan', 1.0)
    d.stamps(scan=20.0, tf=18.8)
    assert 'SCAN_TF_AHEAD' in codes(d, 1.0)


def test_scan_tf_small_lead_ok():
    d = healthy()
    d.tf('map', 'odom', 1.0); d.tf('odom', 'base_link', 1.0); d.seen('scan', 1.0)
    d.stamps(scan=20.008, tf=20.0)
    assert 'SCAN_TF_AHEAD' not in codes(d, 1.0)
