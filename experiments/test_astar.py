import sys, types, io, math, importlib.util

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

ctrl_mod = types.ModuleType('controller')
ctrl_mod.Supervisor = object
sys.modules['controller'] = ctrl_mod

spec = importlib.util.spec_from_file_location(
    'uav_swarm_controller',
    r'd:\IFSP\hybrid-multi-uav-swarm\controllers\uav_swarm_controller\uav_swarm_controller.py'
)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

buildings = [
    {'name': 'office_nw2', 'x': -36.0, 'y': 48.0, 'r': 8.0},
    {'name': 'tower_c',    'x':  24.0, 'y': 46.0, 'r': 7.0},
    {'name': 'tower_b',    'x':  36.0, 'y': 48.0, 'r': 8.5},
    {'name': 'tower_a',    'x':  24.0, 'y': 58.0, 'r': 8.0},
    {'name': 'office_nw1', 'x': -26.0, 'y': 60.0, 'r': 11.0},
]
planner = mod.AStarPlanner(buildings, resolution=2.0, grid_margin=90.0, obstacle_inflation=2.0)
path = planner.plan((-40.0, 48.0), (50.0, 48.0))

print()
print(f'Path found: {len(path)} waypoints')
for i, (x, y) in enumerate(path):
    print(f'  WP[{i:02d}]  x={x:7.2f}  y={y:7.2f}')

# Verify no waypoint is inside an obstacle (using original, non-inflated radii)
ok = True
for i, (x, y) in enumerate(path):
    for b in buildings:
        d = math.hypot(x - b['x'], y - b['y'])
        if d < b['r']:
            bname = b['name']
            print(f'  [FAIL] WP[{i}] inside building {bname}! dist={d:.2f} r={b["r"]}')
            ok = False
if ok:
    print('  [PASS] All waypoints clear of all buildings.')
