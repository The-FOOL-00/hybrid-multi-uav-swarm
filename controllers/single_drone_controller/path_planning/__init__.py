"""
path_planning
=============
Phase 1 placeholder — future global path planners.

Phase 2 plan:
    - OccupancyGrid      : 2-D grid built from building AABB footprints
    - AStarPlanner       : classic A* on OccupancyGrid
    - RRTPlanner         : sampling-based Rapidly-exploring Random Tree (3-D)
    - PathSmoother       : Bézier / spline smoothing of raw waypoints

Usage (planned):
    from path_planning.astar import AStarPlanner
    from path_planning.grid  import OccupancyGrid

    grid    = OccupancyGrid(resolution=1.0, bounds=[(-100,-100),(100,100)])
    grid.add_obstacles(building_positions, footprint_radius=8.0)
    planner = AStarPlanner(grid)
    waypoints = planner.plan(start_xy=(-40,15), goal_xy=(40,-15))
"""
# Nothing implemented yet — Phase 1 uses direct vector navigation.
