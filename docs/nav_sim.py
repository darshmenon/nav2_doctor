#!/usr/bin/env python3
"""A matplotlib simulation of a robot navigating a map while two faults are injected, diagnosed by the real
nav2_doctor `Doctor` class. The robot and the faults are simulated; the findings are what the tool computes.

    python3 docs/nav_sim.py            # writes docs/nav_sim.png and docs/nav_sim.gif
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from nav2_doctor.doctor import Doctor  # noqa: E402

DT = 0.1
SPEED = 0.45                    # m/s along the path
STALE_S, PROGRESS_S = 2.0, 8.0  # --stale 2 (default), --progress-time 8 (default is 15)
OBSTACLES = [(2.5, 0.0, 3.5, 4.2), (5.5, 3.0, 6.5, 8.0), (8.8, 2.5, 9.8, 5.0)]   # x0, y0, x1, y1
PATH = [(1.0, 1.0), (1.0, 5.2), (4.5, 5.2), (4.5, 1.5), (8.5, 1.5), (8.5, 6.5), (10.8, 6.5)]
MAP_W, MAP_H = 12.0, 8.0
FAULTS = [   # (start s, end s, name)
    (12.0, 24.0, 'bridge paused: TF, odom and scan stop'),
    (40.0, 56.0, 'controller stalled: cmd_vel is zero'),
]
T_END = 90.0
NODES = ('controller_server', 'planner_server', 'bt_navigator', 'behavior_server', 'smoother_server', 'waypoint_follower')


def fault_at(t):
    for a, b, name in FAULTS:
        if a <= t < b:
            return name
    return None


def along(path, dist):
    """Point and heading at `dist` metres along the polyline (clamped to its end)."""
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        if dist <= seg:
            f = dist / seg
            return x0 + f * (x1 - x0), y0 + f * (y1 - y0), math.atan2(y1 - y0, x1 - x0)
        dist -= seg
    (x0, y0), (x1, y1) = path[-2], path[-1]
    return x1, y1, math.atan2(y1 - y0, x1 - x0)


def simulate():
    """Run the scenario. Returns a list of per-step dicts: t, x, y, v, fault, findings [(level, code)]."""
    total = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(PATH, PATH[1:]))
    d = Doctor(stale_s=STALE_S, progress_s=PROGRESS_S)
    for n in NODES:
        d.lifecycle[n] = 'active'
    rows, dist, t = [], 0.0, 0.0
    d.set_goal_active(True, 0.0)
    while t <= T_END:
        fault = fault_at(t)
        arrived = dist >= total - 1e-6
        v = 0.0 if (fault or arrived) else SPEED
        dist = min(total, dist + v * DT)
        x, y, yaw = along(PATH, dist)
        if not (fault and fault.startswith('bridge')):     # a paused bridge publishes nothing
            d.tf('map', 'odom', t)
            d.tf('odom', 'base_link', t)
            d.pose(x, y, t, yaw)
            d.seen('scan', t)
        if not (fault and fault.startswith('bridge')):
            d.cmd(0.0 if (fault or arrived) else SPEED, 0.0, t)
        else:
            d.cmd(SPEED, 0.0, t)                            # the controller keeps commanding into the void
        if arrived:
            d.set_goal_active(False, t)
        found = [(f.level, f.code) for f in d.diagnose(t) if f.level in ('ERROR', 'WARN')]
        rows.append(dict(t=t, x=x, y=y, v=v, fault=fault, findings=found))
        t = round(t + DT, 6)
    return rows


def status(row):
    levels = {lv for lv, _ in row['findings']}
    return 'ERROR' if 'ERROR' in levels else 'WARN' if 'WARN' in levels else 'OK'


def render(rows, out_png, out_gif, every=10):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Rectangle

    COLOR = {'OK': '#2a9d57', 'WARN': '#e0a100', 'ERROR': '#d63a3a'}
    codes = []
    for r in rows:
        for lv, c in r['findings']:
            if c not in codes:
                codes.append(c)
    first = {}
    for r in rows:
        for lv, c in r['findings']:
            first.setdefault(c, r)

    fig = plt.figure(figsize=(12.5, 6.4), dpi=80)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.35, 1], height_ratios=[1.5, 1], hspace=0.34, wspace=0.16)
    ax = fig.add_subplot(gs[:, 0])
    tl = fig.add_subplot(gs[0, 1])
    sp = fig.add_subplot(gs[1, 1])
    fig.subplots_adjust(top=0.9)
    fig.suptitle('nav2_doctor on a simulated Nav2 run (simulated robot and faults, real Doctor logic)', fontsize=12)

    ax.set_xlim(0, MAP_W)
    ax.set_ylim(0, MAP_H)
    ax.set_aspect('equal')
    ax.set_title('trajectory coloured by the doctor\'s verdict', fontsize=10)
    for x0, y0, x1, y1 in OBSTACLES:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color='#555b66'))
    ax.plot(*zip(*PATH), '--', color='#9aa3b2', lw=1.2, label='planned path')
    ax.plot(*PATH[0], 's', color='#333', ms=8)
    ax.plot(*PATH[-1], '*', color='#e0a100', ms=16, mec='#333')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    groups = []                              # codes that first appear at the same spot share one label
    for code, r in sorted(first.items(), key=lambda kv: kv[1]['t']):
        for g in groups:
            if math.hypot(g['x'] - r['x'], g['y'] - r['y']) < 0.8:
                g['codes'].append((code, r['t']))
                break
        else:
            groups.append(dict(x=r['x'], y=r['y'], codes=[(code, r['t'])]))
    offsets = [(-1.0, 1.0), (0.9, -1.15), (0.9, 1.3)]
    for k, g in enumerate(groups):
        dx, dy = offsets[k % len(offsets)]
        ax.annotate('\n'.join(f'{c}  {t:.0f} s' for c, t in g['codes']), (g['x'], g['y']), xytext=(g['x'] + dx, g['y'] + dy),
                    fontsize=8, arrowprops=dict(arrowstyle='-', color='#777'),
                    bbox=dict(boxstyle='round,pad=0.25', fc='white', ec='#bbb', alpha=0.95), zorder=6)

    lane = {c: i for i, c in enumerate(codes)}
    tl.set_xlim(0, T_END)
    tl.set_ylim(-0.7, len(codes) - 0.3)
    tl.set_yticks(range(len(codes)))
    tl.set_yticklabels(codes, fontsize=7.5)
    tl.set_title('findings over time', fontsize=10)
    tl.invert_yaxis()
    for a, b, name in FAULTS:
        tl.axvspan(a, b, color='#ddd', alpha=0.6)
        sp.axvspan(a, b, color='#ddd', alpha=0.6)
    sp.set_xlim(0, T_END)
    sp.set_ylim(-0.05, SPEED * 1.35)
    sp.set_title('robot speed (grey = injected fault)', fontsize=10)
    sp.set_xlabel('simulated time (s)')
    sp.set_ylabel('m/s')
    for (a, b, name), y in zip(FAULTS, (SPEED * 1.2, SPEED * 1.2)):
        sp.text((a + b) / 2, y, name.split(':')[0], ha='center', fontsize=8.5, color='#444')

    speed_line = sp.plot([], [], color='#2a6fdb', lw=1.6)[0]
    arts = []                                # artists added while drawing, removed again before the animation
    robot, = ax.plot([], [], 'o', color='#1d3557', ms=9, mec='white', zorder=5)
    verdict = ax.text(0.98, 0.97, '', transform=ax.transAxes, va='top', ha='right', fontsize=10, fontweight='bold',
                      bbox=dict(boxstyle='round', fc='white', ec='#999'))
    tline = tl.axvline(0, color='#333', lw=1)
    drawn = {'i': 0}

    def draw_to(k):
        while drawn['i'] <= k:
            i = drawn['i']
            r = rows[i]
            if i > 0:
                p = rows[i - 1]
                arts.extend(ax.plot([p['x'], r['x']], [p['y'], r['y']], color=COLOR[status(r)], lw=3.2, solid_capstyle='round', zorder=3))
            if status(r) != 'OK':        # a stalled robot has no path length, so mark where it sits
                arts.extend(ax.plot([r['x']], [r['y']], 'o', color=COLOR[status(r)], ms=20, alpha=0.12, zorder=2))
            for lv, c in r['findings']:
                arts.extend(tl.plot([r['t'], r['t'] + DT], [lane[c], lane[c]], color=COLOR[lv], lw=7, solid_capstyle='butt'))
            drawn['i'] += 1
        r = rows[k]
        robot.set_data([r['x']], [r['y']])
        speed_line.set_data([x['t'] for x in rows[:k + 1]], [x['v'] for x in rows[:k + 1]])
        tline.set_xdata([r['t'], r['t']])
        st = status(r)
        what = ', '.join(c for _, c in r['findings']) or 'HEALTHY'
        verdict.set_text(f't = {r["t"]:.0f} s   {what}')
        verdict.set_color(COLOR[st])

    idx = list(range(0, len(rows), every)) + [len(rows) - 1]
    draw_to(len(rows) - 1)
    fig.savefig(out_png, bbox_inches='tight')
    for art in arts:
        art.remove()                       # reset the trails for the animation
    arts.clear()
    drawn['i'] = 0
    anim = FuncAnimation(fig, lambda n: draw_to(idx[n]), frames=len(idx), interval=100)
    anim.save(out_gif, writer=PillowWriter(fps=10))
    plt.close(fig)


if __name__ == '__main__':
    data = simulate()
    render(data, os.path.join(HERE, 'nav_sim.png'), os.path.join(HERE, 'nav_sim.gif'))
    seen = {}
    for r in data:
        for lv, c in r['findings']:
            seen.setdefault(c, (r['t'], lv))
    print('first appearance:', {c: f'{t:.1f}s {lv}' for c, (t, lv) in seen.items()})
