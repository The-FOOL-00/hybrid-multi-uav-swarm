#!/usr/bin/env python3
"""Check for remaining non-ASCII chars in controller file."""
import sys
sys.stdout.reconfigure(encoding='utf-8')

filepath = r'd:\IFSP\hybrid-multi-uav-swarm\controllers\uav_swarm_controller\uav_swarm_controller.py'
content = open(filepath, 'r', encoding='utf-8').read()
lines = content.split('\n')

found = []
for i, line in enumerate(lines, 1):
    for c in line:
        if ord(c) > 127:
            found.append((i, f'U+{ord(c):04X}', repr(c), line.strip()[:80]))
            break

if found:
    for ln, code, ch, ctx in found[:50]:
        print(f'Line {ln}: {code} {ch}: {ctx}')
    print(f'\nTotal lines with non-ASCII: {len(found)}')
else:
    print('Clean! No non-ASCII chars found.')
