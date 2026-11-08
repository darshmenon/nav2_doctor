#!/usr/bin/env python3
"""Render docs/demo_raw.txt (a real nav2_doctor capture) into docs/demo.gif."""
import os
import re
import sys

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
raw = open(os.path.join(HERE, 'demo_raw.txt')).read()
FONT = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 15)
COL = {'31': (255, 95, 95), '33': (240, 200, 80), '32': (110, 220, 130), '0': (205, 205, 205)}
W, H, LH, ROWS = 980, 410, 19, 19
BG, FG, DIM = (22, 24, 30), (205, 205, 205), (120, 125, 135)

reports = [b for b in re.split(r'(?=^--- nav2_doctor)', raw, flags=re.M) if b.startswith('---')]


def segs(line):
    out, color = [], FG
    for part in re.split(r'(\x1b\[[0-9;]*m)', line):
        m = re.fullmatch(r'\x1b\[([0-9;]*)m', part)
        if m:
            code = m.group(1).split(';')[-1]
            color = COL.get(code, FG)
        elif part:
            out.append((part, color))
    return out


def frame(idx):
    img = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, W, 26], fill=(40, 43, 52))
    d.text((12, 5), 'nav2_doctor --period 1      (live capture: Gazebo bridge paused, then resumed)', font=FONT, fill=DIM)
    lines = [l for l in reports[idx].splitlines() if l.strip()]
    y = 36
    for ln in lines[:ROWS]:
        x = 14
        for text, color in segs(ln):
            d.text((x, y), text, font=FONT, fill=color)
            x += d.textlength(text, font=FONT)
        y += LH
    d.text((14, H - 24), f'report {idx + 1}/{len(reports)}', font=FONT, fill=DIM)
    return img


frames = [frame(i) for i in range(len(reports))]
dur = [700] * len(frames)
dur[-1] = 2500
frames[0].save(os.path.join(HERE, 'demo.gif'), save_all=True, append_images=frames[1:], duration=dur, loop=0, optimize=True)
print(f'{len(frames)} frames')
