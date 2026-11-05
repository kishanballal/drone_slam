#!/usr/bin/env python3
"""Create building_victims.sdf = building.sdf + 4 victims, without touching building.sdf.

Usage:
  python3 make_victims_world.py ~/PX4-Autopilot/Tools/simulation/gz/worlds/building.sdf
Writes building_victims.sdf next to it, with <world name="building_victims">
(PX4 needs the world name to match PX4_GZ_WORLD).
"""
import os
import re
import sys

# name, model, x, y, yaw  (Gazebo world ENU; all >= 0.6 m from walls and off the corridor-loop path)
VICTIMS = [
    ('victim_1', 'victim_sitting', -8.0, -6.5, 0.6),     # SW room, far corner
    ('victim_2', 'victim_lying', 3.0, -4.5, 0.0),        # SE room, middle
    ('victim_3', 'victim_sitting', -3.0, 7.2, -1.57),    # north corridor, against the outer wall side
    ('victim_4', 'victim_lying', 8.6, 5.5, 1.57),        # east corridor, along the outer wall
]


def main():
    src = os.path.expanduser(sys.argv[1])
    dst = os.path.join(os.path.dirname(src), 'building_victims.sdf')
    sdf = open(src).read()
    sdf, n = re.subn(r'<world\s+name\s*=\s*"[^"]*"', '<world name="building_victims"', sdf, count=1)
    if n != 1:
        sys.exit('ERROR: no <world name="..."> tag found in ' + src)
    if 'victim_1' in sdf:
        sys.exit('ERROR: source already contains victims')
    inc = '\n    <!-- M3 victims -->\n'
    for name, model, x, y, yaw in VICTIMS:
        inc += (f'    <include>\n      <uri>model://{model}</uri>\n      <name>{name}</name>\n'
                f'      <pose>{x} {y} 0 0 0 {yaw}</pose>\n    </include>\n')
    i = sdf.rfind('</world>')
    sdf = sdf[:i] + inc + '  ' + sdf[i:]
    open(dst, 'w').write(sdf)
    print('wrote', dst)
    for v in VICTIMS:
        print('  %-9s %-15s x=%5.1f y=%5.1f' % v[:4])


if __name__ == '__main__':
    main()
