#!/usr/bin/env python3
"""
Fix ALL non-ASCII Unicode characters in uav_swarm_controller.py.
This is a comprehensive replacement that converts EVERY non-ASCII char.
"""
import re

FILEPATH = r"d:\IFSP\hybrid-multi-uav-swarm\controllers\uav_swarm_controller\uav_swarm_controller.py"

with open(FILEPATH, "r", encoding="utf-8") as f:
    content = f.read()

original = content

# Comprehensive Unicode-to-ASCII mapping
CHAR_MAP = {
    # Arrows
    '\u2192': '->',   # →
    '\u2190': '<-',   # ←
    '\u2191': '^',    # ↑
    '\u2193': 'v',    # ↓
    '\u279e': '->',
    '\u27a1': '->',
    # Em/en dashes (commonly used in docstrings and comments)
    '\u2014': ' - ',  # — em dash
    '\u2013': '-',    # – en dash
    '\u2012': '-',    # ‒ figure dash
    # Box drawing (all used in HUD banners)
    '\u2500': '-',    # ─ light horizontal
    '\u2501': '-',    # ━ heavy horizontal
    '\u2502': '|',    # │ light vertical
    '\u2503': '|',    # ┃ heavy vertical
    '\u2550': '=',    # ═
    '\u2551': '|',    # ║
    '\u2554': '+',    # ╔
    '\u2557': '+',    # ╗
    '\u255a': '+',    # ╚
    '\u255d': '+',    # ╝
    '\u2560': '+',    # ╠
    '\u2563': '+',    # ╣
    '\u2566': '+',    # ╦
    '\u2569': '+',    # ╩
    '\u256c': '+',    # ╬
    '\u250c': '+',    # ┌
    '\u2510': '+',    # ┐
    '\u2514': '+',    # └
    '\u2518': '+',    # ┘
    '\u253c': '+',    # ┼
    '\u252c': '+',    # ┬
    '\u2534': '+',    # ┴
    '\u251c': '+',    # ├
    '\u2524': '+',    # ┤
    # Check/cross marks
    '\u2713': '(OK)',  # ✓
    '\u2714': '(OK)',  # ✔
    '\u2717': '(X)',   # ✗
    '\u2718': '(X)',   # ✘
    '\u2022': '*',     # • bullet
    '\u25ba': '>',     # ►
    '\u25c4': '<',     # ◄
    '\u25b6': '>',     # ▶
    '\u25c0': '<',     # ◀
    # Block elements
    '\u2588': '#',    # █ full block
    '\u2584': '_',    # ▄
    '\u2580': '^',    # ▀
    '\u258c': '|',    # ▌
    '\u2590': '|',    # ▐
    # Math
    '\u2248': '~=',   # ≈ approx
    '\u00d7': 'x',    # × multiply
    '\u2212': '-',    # − minus sign
    # Greek letters in comments (pi)
    '\u03c0': 'pi',   # π
    '\u03b1': 'a',    # α
    '\u03b2': 'b',    # β
    '\u03b3': 'g',    # γ
    # Fullwidth
    '\uff5c': '|',    # ｜
    # Miscellaneous
    '\u00b0': 'deg',  # °
    '\u00b2': '2',    # ²
    '\u00b3': '3',    # ³
    '\u2032': "'",    # ′ prime
    '\u2033': "''",   # ″ double prime
    '\u2026': '...',  # … ellipsis
    '\u00e9': 'e',    # é
    '\u00e0': 'a',    # à
}

# Apply all replacements
for uni_char, replacement in CHAR_MAP.items():
    content = content.replace(uni_char, replacement)

# Also handle \uXXXX escape literals that appear as text in strings
def replace_literal_escape(match):
    code = int(match.group(1), 16)
    c = chr(code)
    if c in CHAR_MAP:
        return CHAR_MAP[c]
    if code > 127:
        return '?'
    return match.group(0)

content = re.sub(r'\\u([0-9a-fA-F]{4})', replace_literal_escape, content)

# Report
orig_lines = original.split('\n')
new_lines = content.split('\n')
changed = sum(1 for o, n in zip(orig_lines, new_lines) if o != n)
print(f"Changed {changed} lines.")

# Verify clean
remaining = [(i+1, c, line.strip()[:60]) for i, line in enumerate(new_lines) for c in line if ord(c) > 127]
if remaining:
    print(f"WARNING: {len(remaining)} non-ASCII chars remain:")
    for ln, c, ctx in remaining[:20]:
        print(f"  Line {ln}: U+{ord(c):04X} '{c}': {ctx}")
else:
    print("VERIFIED CLEAN: No non-ASCII chars remain.")

with open(FILEPATH, "w", encoding="utf-8") as f:
    f.write(content)

print("File saved.")
