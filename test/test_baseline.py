import json, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from nav2_doctor import baseline as bl
from nav2_doctor.doctor import Doctor


def stream(hz, secs, start=0.0, jitter=0.0):
    return [start + i / hz + (jitter if i % 7 == 0 else 0.0) for i in range(int(secs * hz))]


def test_learn_recovers_rate_and_ignores_a_few_stalls():
    t = stream(10, 30)
    t[40:] = [x + 3.0 for x in t[40:]]                      # one 3 s stall in the middle
    b = bl.learn({'scan': t})
    assert abs(b['scan']['hz'] - 10) < 0.2


def test_learn_skips_streams_with_too_few_messages():
    assert bl.learn({'map': [0.0, 5.0, 10.0]}) == {}


def test_check_is_quiet_at_normal_rate_and_names_a_drop_and_a_spike():
    base = {'scan': {'hz': 10.0, 'jitter': 0.0, 'n': 100}}
    assert bl.check(base, {'scan': stream(10, 20)}, 19.9) == []
    drop = bl.check(base, {'scan': stream(3, 20)}, 19.9)
    assert [c for _, c, _ in drop] == ['RATE_DROP'] and '3' in drop[0][2] and '10' in drop[0][2]
    assert [c for _, c, _ in bl.check(base, {'scan': stream(30, 20)}, 19.9)] == ['RATE_SPIKE']


def test_silent_stream_is_left_to_the_stale_checks():
    base = {'scan': {'hz': 10.0, 'jitter': 0.0, 'n': 100}}
    assert bl.check(base, {'scan': stream(10, 5)}, 60.0) == []      # no recent messages: not a rate finding


def test_save_and_load_round_trip(tmp_path):
    p = str(tmp_path / 'b.json')
    b = bl.learn({'scan': stream(10, 20)})
    bl.save(b, p)
    assert bl.load(p) == b and json.load(open(p))['version'] == 1


def test_doctor_records_arrivals_and_reports_a_drop_with_a_hint():
    from nav2_doctor.doctor import HINTS
    d = Doctor()
    for n in ('controller_server', 'planner_server', 'bt_navigator', 'behavior_server', 'smoother_server', 'waypoint_follower'):
        d.lifecycle[n] = 'active'
    for t in stream(10, 20):
        d.seen('scan', t)
    d.baseline = bl.learn(d.arrivals)
    d.arrivals['scan'] = stream(2, 10, start=20.0)                  # scan slows to 2 Hz
    d.last['scan'] = 29.5
    for e in (('map', 'odom'), ('odom', 'base_link')):
        d.tf(*e, 29.5)
    d.pose(0, 0, 29.5)
    found = {f.code for f in d.diagnose(29.6)}
    assert 'RATE_DROP' in found and 'SCAN_STALE' not in found
    assert HINTS['RATE_DROP'] and HINTS['RATE_SPIKE']


def test_arrivals_are_bounded():
    d = Doctor()
    for i in range(5000):
        d.seen('scan', i * 0.01)
    assert len(d.arrivals['scan']) <= 1500


def test_parse_expect_accepts_colon_and_equals_and_rejects_junk():
    import pytest
    assert bl.parse_expect(['scan:10', '/odom=20.5']) == {'scan': {'hz': 10.0, 'jitter': 0.0, 'n': 0},
                                                         'odom': {'hz': 20.5, 'jitter': 0.0, 'n': 0}}
    for bad in ('scan', 'scan:fast', 'scan:0', ':5'):
        with pytest.raises(ValueError):
            bl.parse_expect([bad])


def test_expected_rate_feeds_check():
    exp = bl.parse_expect(['scan:10'])
    assert bl.check(exp, {'scan': stream(10, 20)}, 19.9) == []
    assert bl.check(exp, {'scan': stream(3, 20)}, 19.9)[0][1] == 'RATE_DROP'


def test_load_rejects_bad_files_with_a_readable_error(tmp_path):
    import pytest
    cases = {'missing.json': None, 'empty.json': '', 'notjson.json': 'hello', 'nokey.json': '{"version": 1}',
             'nohz.json': '{"streams": {"scan": {"jitter": 1}}}', 'zero.json': '{"streams": {"scan": {"hz": 0}}}',
             'text.json': '{"streams": {"scan": {"hz": "fast"}}}', 'list.json': '[1, 2]'}
    for name, txt in cases.items():
        p = tmp_path / name
        if txt is not None:
            p.write_text(txt)
        with pytest.raises(ValueError):
            bl.load(str(p))
    good = tmp_path / 'good.json'
    bl.save({'scan': {'hz': 10.0, 'jitter': 0.0, 'n': 5}}, str(good))
    assert bl.load(str(good))['scan']['hz'] == 10.0
