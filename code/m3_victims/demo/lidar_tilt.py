#!/usr/bin/env python3
"""Widen the 3D LiDAR's downward view from -15 deg to -30 deg so people lying on the floor
are seen from ~2-3 m instead of only from ~6 m. Keeps the ring spacing by adding rings.
Backup: model.sdf.m3bak (first run only). Safe to run again (does nothing the second time)."""
import os, re, shutil, sys

path = os.path.expanduser(sys.argv[1])
NEW_MIN = -0.5236
txt = open(path).read()
changed = 0

def fix(m):
    global changed
    block = m.group(0)
    mn = re.search(r'<min_angle>\s*([-\d.eE]+)\s*</min_angle>', block)
    mx = re.search(r'<max_angle>\s*([-\d.eE]+)\s*</max_angle>', block)
    sm = re.search(r'<samples>\s*(\d+)\s*</samples>', block)
    if not (mn and mx and sm):
        return block
    old_min, old_max, n = float(mn.group(1)), float(mx.group(1)), int(sm.group(1))
    if old_min <= NEW_MIN + 0.01 or old_min > -0.1:     # already done, or not the tilt we expect
        return block
    new_n = max(n, round((n - 1) * (old_max - NEW_MIN) / (old_max - old_min)) + 1)
    block = block.replace(mn.group(0), f'<min_angle>{NEW_MIN}</min_angle>')
    block = block.replace(sm.group(0), f'<samples>{new_n}</samples>')
    print(f'3D LiDAR vertical view: {old_min:.3f}..{old_max:.3f} rad, {n} rings  ->  '
          f'{NEW_MIN:.3f}..{old_max:.3f} rad, {new_n} rings')
    changed += 1
    return block

new = re.sub(r'<vertical>.*?</vertical>', fix, txt, flags=re.S)
if changed:
    if not os.path.exists(path + '.m3bak'):
        shutil.copy(path, path + '.m3bak')
    open(path, 'w').write(new)
elif re.search(r'<min_angle>\s*-0\.52', txt):
    print('3D LiDAR already looks down to -30 deg')
else:
    print('!! 3D LiDAR <vertical> block not found or unexpected; victims lying down may be missed')
