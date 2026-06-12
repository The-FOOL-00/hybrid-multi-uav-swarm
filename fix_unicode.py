#!/usr/bin/env python3
"""
Fix Unicode characters in uav_swarm_controller.py for Windows cp1252 compatibility.
Replaces arrows/box-drawing chars in print/string output with ASCII equivalents.
Also removes the unconditional stdout redirect added during debugging.
"""
import re

FILEPATH = r"d:\IFSP\hybrid-multi-uav-swarm\controllers\uav_swarm_controller\uav_swarm_controller.py"

# Read with UTF-8
with open(FILEPATH, "r", encoding="utf-8") as f:
    content = f.read()

original = content

# ── 1. Remove the unconditional stdout redirect (added during debugging) ──
# Replace:
#   sys.stdout = open(r"d:\IFSP\...\uav_debug.log", "w", buffering=1)
#   print("Redirected stdout to uav_debug.log unconditionally")
# With nothing (or a safe conditional version)
content = re.sub(
    r'sys\.stdout\s*=\s*open\(r?"[^"]*uav_debug\.log"[^\n]*\)\s*\n'
    r'print\("Redirected stdout[^\n]*"\)\s*\n',
    '',
    content
)

# ── 2. Replace Unicode box-drawing and arrow characters ──
REPLACEMENTS = [
    # Arrows
    ('\u2192', '->'),   # → RIGHT ARROW
    ('\u2190', '<-'),   # ← LEFT ARROW
    ('\u2191', '^'),    # ↑ UP ARROW
    ('\u2193', 'v'),    # ↓ DOWN ARROW
    ('\u279e', '->'),   # ➞
    ('\u27a1', '->'),   # ➡
    # Box drawing
    ('\u2551', '|'),    # ║
    ('\u2550', '='),    # ═
    ('\u2554', '+'),    # ╔
    ('\u2557', '+'),    # ╗
    ('\u255a', '+'),    # ╚
    ('\u255d', '+'),    # ╝
    ('\u2560', '+'),    # ╠
    ('\u2563', '+'),    # ╣
    ('\u2566', '+'),    # ╦
    ('\u2569', '+'),    # ╩
    ('\u256c', '+'),    # ╬
    # Check/cross marks
    ('\u2713', '(OK)'),   # ✓
    ('\u2714', '(OK)'),   # ✔
    ('\u2717', '(X)'),    # ✗
    ('\u2718', '(X)'),    # ✘
    ('\u2022', '*'),      # •  bullet
    ('\u25ba', '>'),      # ► play/arrow
    ('\u25c4', '<'),      # ◄
    # Misc symbols used in HUD
    ('\u2588', '#'),    # █ full block
    ('\u2584', '_'),    # ▄ lower half
    ('\u2580', '^'),    # ▀ upper half
    # Common unicode brackets/pipes used in debug banners
    ('\uff5c', '|'),    # ｜ fullwidth vertical line
]

for uni_char, replacement in REPLACEMENTS:
    content = content.replace(uni_char, replacement)

# ── 3. Replace \uXXXX escape literals in string literals ──
# e.g.  print(f"\u2551  Altitude") -> print(f"|  Altitude")
def replace_unicode_escapes(match):
    code = int(match.group(1), 16)
    c = chr(code)
    for uni_char, replacement in REPLACEMENTS:
        if c == uni_char:
            return replacement
    # For any other non-ASCII, return ASCII fallback
    if code > 127:
        return '?'
    return match.group(0)

content = re.sub(r'\\u([0-9a-fA-F]{4})', replace_unicode_escapes, content)

# ── 4. Report changed lines ──
orig_lines = original.split('\n')
new_lines = content.split('\n')
changed = 0
for i, (o, n) in enumerate(zip(orig_lines, new_lines), 1):
    if o != n:
        print(f"  Line {i:4d}: changed")
        changed += 1

print(f"\nTotal changed lines: {changed}")

# Write back as UTF-8 (safe for Webots, which reads UTF-8)
with open(FILEPATH, "w", encoding="utf-8") as f:
    f.write(content)

print("Done. File saved.")
