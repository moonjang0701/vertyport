"""
3D Path Planning for UAM using A* algorithm
Considers terrain, obstacles, and risk zones
"""

import numpy as np
from typing import List, Tuple, Optional, Set
from dataclasses import dataclass
import heapq
from ..data_processing.dem_processor import DEMData


@dataclass
class Waypoint:
    """3D waypoint"""
    x: float
    y: float
    z: float
    
    def __hash__(self):
        return hash((round(self.x, 2), round(self.y, 2), round(self.z, 2)))
    
    def __eq__(self, other):
        return (round(self.x, 2) == round(other.x, 2) and 
                round(self.y, 2) == round(other.y, 2) and
                round(self.z, 2) == round(other.z, 2))
    
    def distance_to(self, other: 'Waypoint') -> float:
        """Calculate Euclidean distance to another waypoint"""
        return np.sqrt((self.x - other.x)**2 + 
                      (self.y - other.y)**2 + 
                      (self.z - other.z)**2)
    
    def to_tuple(self) -> Tuple[float, float, float]:
        """Convert to tuple"""
        return (self.x, self.y, self.z)


@dataclass
class FlightPath:
    """Complete flight path with waypoints"""
    waypoints: List[Waypoint]
    total_distance: float
    total_risk: float
    
    def __len__(self):
        return len(self.waypoints)
    
    def get_segment(self, idx: int) -> Tuple[Waypoint, Waypoint]:
        """Get path segment between two consecutive waypoints"""
        if idx >= len(self.waypoints) - 1:
            raise IndexError("Segment index out of range")
        return self.waypoints[idx], self.waypoints[idx + 1]


class PathPlanner3D:
    """3D A* path planner for UAM operations"""
    
    def __init__(self, dem_data: DEMData, risk_map: np.ndarray,
                 min_altitude: float = 50.0, max_altitude: float = 150.0,
                 safety_margin: float = 20.0):
        """
        Initialize 3D path planner
        
        Args:
            dem_data: Digital elevation model data
            risk_map: 2D risk map (0-1 values)
            min_altitude: Minimum flight altitude above ground (meters)
            max_altitude: Maximum flight altitude above ground (meters)
            safety_margin: Safety clearance above obstacles (meters)
        """
        self.dem_data = dem_data
        self.risk_map = risk_map
        self.min_altitude = min_altitude
        self.max_altitude = max_altitude
        self.safety_margin = safety_margin
        
        # Grid resolution for path planning
        self.grid_resolution = dem_data.resolution
        
    def plan_path(self, start: Tuple[float, float, float], 
                  goal: Tuple[float, float, float],
                  num_altitude_levels: int = 5) -> Optional[FlightPath]:
        """
        Plan 3D path from start to goal using A*
        
        Args:
            start: Start position (x, y, z)
            goal: Goal position (x, y, z)
            num_altitude_levels: Number of altitude levels to consider
            
        Returns:
            FlightPath object or None if no path found
        """
        start_wp = Waypoint(*start)
        goal_wp = Waypoint(*goal)
        
        # A* algorithm
        open_set = []
        heapq.heappush(open_set, (0, start_wp))
        
        came_from = {}
        g_score = {start_wp: 0}
        f_score = {start_wp: self._heuristic(start_wp, goal_wp)}
        
        closed_set: Set[Waypoint] = set()
        
        iterations = 0
        max_iterations = 10000
        
        while open_set and iterations < max_iterations:
            iterations += 1
            
            current_f, current = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
            
            # Check if we reached the goal
            if current.distance_to(goal_wp) < self.grid_resolution * 2:
                path = self._reconstruct_path(came_from, current)
                path.append(goal_wp)
                return self._create_flight_path(path)
            
            closed_set.add(current)
            
            # Explore neighbors
            neighbors = self._get_neighbors(current, num_altitude_levels)
            
            for neighbor in neighbors:
                if neighbor in closed_set:
                    continue
                
                # Calculate tentative g_score
                movement_cost = current.distance_to(neighbor)
                risk_cost = self._get_risk_cost(current, neighbor)
                tentative_g = g_score[current] + movement_cost + risk_cost * 50
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal_wp)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # No path found
        return None
    
    def _get_neighbors(self, waypoint: Waypoint, 
                      num_altitude_levels: int) -> List[Waypoint]:
        """Get valid neighboring waypoints"""
        neighbors = []
        
        # Define movement directions (8 horizontal + vertical)
        dx_dy_moves = [
            (1, 0), (-1, 0), (0, 1), (0, -1),  # Cardinal
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # Diagonal
        ]
        
        # Altitude changes
        dz_moves = [0, self.grid_resolution, -self.grid_resolution]
        
        for dx, dy in dx_dy_moves:
            for dz in dz_moves:
                new_x = waypoint.x + dx * self.grid_resolution
                new_y = waypoint.y + dy * self.grid_resolution
                new_z = waypoint.z + dz
                
                # Check if within bounds
                if not self._is_within_bounds(new_x, new_y):
                    continue
                
                # Check altitude constraints
                ground_elevation = self._get_ground_elevation(new_x, new_y)
                min_z = ground_elevation + self.min_altitude
                max_z = ground_elevation + self.max_altitude
                
                if new_z < min_z or new_z > max_z:
                    continue
                
                # Check collision with obstacles
                if not self._is_collision_free(new_x, new_y, new_z):
                    continue
                
                neighbors.append(Waypoint(new_x, new_y, new_z))
        
        return neighbors
    
    def _is_within_bounds(self, x: float, y: float) -> bool:
        """Check if position is within DEM bounds"""
        return (self.dem_data.bounds[0] <= x <= self.dem_data.bounds[2] and
                self.dem_data.bounds[1] <= y <= self.dem_data.bounds[3])
    
    def _get_ground_elevation(self, x: float, y: float) -> float:
        """Get ground elevation at position"""
        # Convert to grid indices
        x_idx = int((x - self.dem_data.bounds[0]) / self.grid_resolution)
        y_idx = int((y - self.dem_data.bounds[1]) / self.grid_resolution)
        
        # Clip to valid range
        x_idx = np.clip(x_idx, 0, self.dem_data.shape[1] - 1)
        y_idx = np.clip(y_idx, 0, self.dem_data.shape[0] - 1)
        
        return self.dem_data.elevation[y_idx, x_idx]
    
    def _is_collision_free(self, x: float, y: float, z: float) -> bool:
        """Check if position is collision-free"""
        ground_elevation = self._get_ground_elevation(x, y)
        
        # Check if too close to ground/obstacles
        if z < ground_elevation + self.safety_margin:
            return False
        
        return True
    
    def _get_risk_cost(self, waypoint1: Waypoint, waypoint2: Waypoint) -> float:
        """Calculate risk cost for moving between waypoints"""
        # Sample risk along the path segment
        num_samples = 5
        total_risk = 0.0
        
        for i in range(num_samples):
            t = i / (num_samples - 1)
            x = waypoint1.x + t * (waypoint2.x - waypoint1.x)
            y = waypoint1.y + t * (waypoint2.y - waypoint1.y)
            
            # Get risk value
            x_idx = int((x - self.dem_data.bounds[0]) / self.grid_resolution)
            y_idx = int((y - self.dem_data.bounds[1]) / self.grid_resolution)
            
            x_idx = np.clip(x_idx, 0, self.risk_map.shape[1] - 1)
            y_idx = np.clip(y_idx, 0, self.risk_map.shape[0] - 1)
            
            total_risk += self.risk_map[y_idx, x_idx]
        
        return total_risk / num_samples
    
    def _heuristic(self, waypoint: Waypoint, goal: Waypoint) -> float:
        """A* heuristic function (Euclidean distance)"""
        return waypoint.distance_to(goal)
    
    def _reconstruct_path(self, came_from: dict, 
                         current: Waypoint) -> List[Waypoint]:
        """Reconstruct path from A* came_from dict"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def _create_flight_path(self, waypoints: List[Waypoint]) -> FlightPath:
        """Create FlightPath object from waypoint list"""
        # Calculate total distance
        total_distance = 0.0
        for i in range(len(waypoints) - 1):
            total_distance += waypoints[i].distance_to(waypoints[i + 1])
        
        # Calculate total risk
        total_risk = 0.0
        for i in range(len(waypoints) - 1):
            total_risk += self._get_risk_cost(waypoints[i], waypoints[i + 1])
        
        return FlightPath(
            waypoints=waypoints,
            total_distance=total_distance,
            total_risk=total_risk
        )
    
    def smooth_path(self, path: FlightPath, smoothing_factor: float = 0.3) -> FlightPath:
        """
        Smooth flight path using moving average
        
        Args:
            path: Original flight path
            smoothing_factor: Smoothing intensity (0-1)
            
        Returns:
            Smoothed flight path
        """
        if len(path.waypoints) < 3:
            return path
        
        smoothed_waypoints = [path.waypoints[0]]  # Keep start point
        
        for i in range(1, len(path.waypoints) - 1):
            prev_wp = path.waypoints[i - 1]
            curr_wp = path.waypoints[i]
            next_wp = path.waypoints[i + 1]
            
            # Average with neighbors
            new_x = (1 - smoothing_factor) * curr_wp.x + \
                   smoothing_factor * (prev_wp.x + next_wp.x) / 2
            new_y = (1 - smoothing_factor) * curr_wp.y + \
                   smoothing_factor * (prev_wp.y + next_wp.y) / 2
            new_z = (1 - smoothing_factor) * curr_wp.z + \
                   smoothing_factor * (prev_wp.z + next_wp.z) / 2
            
            smoothed_waypoints.append(Waypoint(new_x, new_y, new_z))
        
        smoothed_waypoints.append(path.waypoints[-1])  # Keep end point
        
        return self._create_flight_path(smoothed_waypoints)
