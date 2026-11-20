import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'docs'))
import nav_sim  # simulate() is pure python; matplotlib is only needed by render()


def seen(rows, lo, hi):
    return {c for r in rows if lo <= r['t'] <= hi for _, c in r['findings']}


def test_scenario_findings_follow_the_faults():
    rows = nav_sim.simulate()
    assert not seen(rows, 0, 12)                                   # healthy before the first fault
    assert {'TF_FREEZE', 'ODOM_STALE', 'SCAN_STALE'} <= seen(rows, 12, 24)
    assert 'CMD_ZERO' in seen(rows, 40, 56)
    assert not seen(rows, 70, 90)                                  # recovers and finishes clean
