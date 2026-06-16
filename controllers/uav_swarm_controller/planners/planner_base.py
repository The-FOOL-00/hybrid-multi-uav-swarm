import time
from abc import ABC, abstractmethod

class PlannerBase(ABC):
    """Abstract base class for all global path planners."""
    
    def __init__(self, world_config, grid_config):
        """
        Initialize planner with world and grid parameters.
        
        Args:
            world_config: Dict containing 'buildings' list
            grid_config: Dict containing 'resolution', 'grid_margin', 
                        'obstacle_inflation', 'hyperparams'
        """
        self.buildings = world_config.get("buildings", [])
        self.resolution = grid_config.get("resolution", 2.0)
        self.grid_margin = grid_config.get("grid_margin", 90.0)
        self.inflation = grid_config.get("obstacle_inflation", 2.0)
        self.hyperparams = grid_config.get("hyperparams", {})
        
        # Stats tracked for each planning call
        self.compute_time_ms = 0.0
        self.nodes_explored = 0
        self.path_length_m = 0.0
        self.smoothness_score = 1.0
    
    @abstractmethod
    def plan(self, start, goal, obstacles=None):
        """
        Compute path from start to goal avoiding obstacles.
        
        Args:
            start: Tuple (x, y) in world meters
            goal: Tuple (x, y) in world meters
            obstacles: Optional list (unused, for interface consistency)
        
        Returns:
            List of waypoints [(x, y), ...] in world meters (2D)
            Returns empty list if planning fails
        """
        pass
    
    def get_stats(self):
        """Return planner statistics dictionary."""
        return {
            "compute_time_ms": self.compute_time_ms,
            "nodes_explored": self.nodes_explored,
            "path_length_m": self.path_length_m,
            "smoothness_score": self.smoothness_score
        }
    
    def _compute_path_length(self, waypoints):
        """Helper: Compute total distance of path."""
        if len(waypoints) < 2:
            return 0.0
        total = 0.0
        for i in range(len(waypoints) - 1):
            x1, y1 = waypoints[i]
            x2, y2 = waypoints[i + 1]
            total += ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        return total
    
    def _compute_smoothness(self, waypoints):
        """
        Helper: Compute smoothness score.
        
        Returns value in [0.0, 1.0] where:
        - 1.0 = perfectly straight (no turns)
        - 0.0 = severe zigzag (180° turns)
        
        Formula: smoothness = 1.0 - mean(deviation_angles) / 180.0
        where deviation_angles are heading changes at each interior waypoint.
        """
        if len(waypoints) < 3:
            return 1.0  # Default to straight for short paths
        
        deviation_angles = []
        for i in range(1, len(waypoints) - 1):
            # Vectors from waypoint i-1 to i, and i to i+1
            x1, y1 = waypoints[i - 1]
            x2, y2 = waypoints[i]
            x3, y3 = waypoints[i + 1]
            
            # Direction vectors
            dx1, dy1 = x2 - x1, y2 - y1
            dx2, dy2 = x3 - x2, y3 - y2
            
            # Magnitude
            mag1 = (dx1 ** 2 + dy1 ** 2) ** 0.5
            mag2 = (dx2 ** 2 + dy2 ** 2) ** 0.5
            
            if mag1 > 0 and mag2 > 0:
                # Dot product and cross product for angle
                dot = dx1 * dx2 + dy1 * dy2
                cross = dx1 * dy2 - dy1 * dx2
                
                # Angle in degrees
                import math
                angle_rad = math.atan2(abs(cross), dot)
                angle_deg = math.degrees(angle_rad)
                deviation_angles.append(angle_deg)
        
        if not deviation_angles:
            return 1.0
        
        mean_deviation = sum(deviation_angles) / len(deviation_angles)
        smoothness = max(0.0, 1.0 - mean_deviation / 180.0)
        return round(smoothness, 3)
