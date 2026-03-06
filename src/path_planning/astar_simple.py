"""
Simple 3D A* Path Planner for UAM
출발지 → 도착지 경로 생성 (장애물 회피)
"""

import numpy as np
import heapq
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class PathResult:
    """Path planning result"""
    waypoints: List[Tuple[float, float, float]]  # (lat, lon, alt)
    total_distance: float  # meters
    total_risk: float  # 0-1
    avg_altitude: float  # meters
    computation_time: float  # seconds
    success: bool


class SimpleAStarPlanner:
    """
    Simple 3D A* path planner for UAM
    
    Uses:
    - DEM: Ground elevation
    - Obstacle map: Building heights
    - Risk map: Ground risk (0-1)
    """
    
    def __init__(self,
                 dem: np.ndarray,
                 obstacle_map: np.ndarray,
                 risk_map: np.ndarray,
                 bounds: dict,
                 min_altitude: float = 50.0,
                 max_altitude: float = 150.0,
                 safety_margin: float = 30.0):
        """
        Initialize A* planner
        
        Args:
            dem: Digital elevation model (meters)
            obstacle_map: Building heights (meters)
            risk_map: Ground risk values (0-1)
            bounds: {'lat_min', 'lat_max', 'lon_min', 'lon_max'}
            min_altitude: Minimum altitude above ground (m)
            max_altitude: Maximum altitude above ground (m)
            safety_margin: Safety clearance above obstacles (m)
        """
        self.dem = dem
        self.obstacle_map = obstacle_map
        self.risk_map = risk_map
        self.bounds = bounds
        
        self.min_altitude = min_altitude
        self.max_altitude = max_altitude
        self.safety_margin = safety_margin
        
        # Grid dimensions
        self.grid_height, self.grid_width = dem.shape
        
        # Coordinate conversion
        self.lat_step = (bounds['lat_max'] - bounds['lat_min']) / self.grid_height
        self.lon_step = (bounds['lon_max'] - bounds['lon_min']) / self.grid_width
        
        # Altitude discretization
        self.altitude_levels = 5  # Number of altitude levels
        self.altitude_step = (max_altitude - min_altitude) / (self.altitude_levels - 1)
    
    def plan_path(self,
                  start_lat: float, start_lon: float,
                  goal_lat: float, goal_lon: float,
                  max_iterations: int = 10000,
                  verbose: bool = False) -> PathResult:
        """
        Plan 3D path from start to goal
        
        Args:
            start_lat, start_lon: Start coordinates
            goal_lat, goal_lon: Goal coordinates
            max_iterations: Maximum A* iterations
            
        Returns:
            PathResult with waypoints and statistics
        """
        import time
        start_time = time.time()
        
        # Convert to grid coordinates
        start_i, start_j = self._latlon_to_grid(start_lat, start_lon)
        goal_i, goal_j = self._latlon_to_grid(goal_lat, goal_lon)
        
        if verbose:
            print(f"   Start grid: ({start_i}, {start_j})")
            print(f"   Goal grid: ({goal_i}, {goal_j})")
        
        # Check if start/goal are valid
        if not self._is_valid_grid(start_i, start_j):
            if verbose:
                print(f"   ❌ Start position invalid")
            return PathResult([], 0, 0, 0, 0, False)
        if not self._is_valid_grid(goal_i, goal_j):
            if verbose:
                print(f"   ❌ Goal position invalid")
            return PathResult([], 0, 0, 0, 0, False)
        
        # Get safe altitudes
        start_alt = self._get_safe_altitude(start_i, start_j)
        goal_alt = self._get_safe_altitude(goal_i, goal_j)
        
        if verbose:
            print(f"   Start altitude: {start_alt:.1f}m")
            print(f"   Goal altitude: {goal_alt:.1f}m")
            print(f"   Start ground: {self.dem[start_i, start_j]:.1f}m")
            print(f"   Start obstacle: {self.obstacle_map[start_i, start_j]:.1f}m")
            print(f"   Goal ground: {self.dem[goal_i, goal_j]:.1f}m")
            print(f"   Goal obstacle: {self.obstacle_map[goal_i, goal_j]:.1f}m")
        
        # Start and goal nodes: (i, j, alt_level)
        start_node = (start_i, start_j, self._altitude_to_level(start_alt))
        goal_node = (goal_i, goal_j, self._altitude_to_level(goal_alt))
        
        # A* algorithm
        open_set = []
        heapq.heappush(open_set, (0, start_node))
        
        came_from = {}
        g_score = {start_node: 0}
        f_score = {start_node: self._heuristic(start_node, goal_node)}
        
        closed_set = set()
        iterations = 0
        
        while open_set and iterations < max_iterations:
            iterations += 1
            
            if verbose and iterations % 1000 == 0:
                print(f"   Iteration {iterations}, open_set size: {len(open_set)}, closed: {len(closed_set)}")
            
            current_f, current = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
            
            # Goal check
            if self._is_goal(current, goal_node):
                # Reconstruct path
                path = self._reconstruct_path(came_from, current, goal_node)
                
                # Convert to lat/lon/alt
                waypoints = [
                    self._node_to_latlon(node) for node in path
                ]
                
                # Calculate statistics
                total_distance = self._calculate_distance(waypoints)
                total_risk = self._calculate_risk(waypoints)
                avg_altitude = np.mean([wp[2] for wp in waypoints])
                
                elapsed = time.time() - start_time
                
                return PathResult(
                    waypoints=waypoints,
                    total_distance=total_distance,
                    total_risk=total_risk,
                    avg_altitude=avg_altitude,
                    computation_time=elapsed,
                    success=True
                )
            
            closed_set.add(current)
            
            # Get neighbors
            neighbors = self._get_neighbors(current)
            
            for neighbor in neighbors:
                if neighbor in closed_set:
                    continue
                
                # Calculate cost
                move_cost = self._movement_cost(current, neighbor)
                risk_cost = self._get_risk_cost(current, neighbor)
                
                tentative_g = g_score[current] + move_cost + risk_cost * 100
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal_node)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # No path found
        elapsed = time.time() - start_time
        if verbose:
            print(f"   ❌ No path found after {iterations} iterations")
            print(f"   Closed set size: {len(closed_set)}")
        return PathResult([], 0, 0, 0, elapsed, False)
    
    def _latlon_to_grid(self, lat: float, lon: float) -> Tuple[int, int]:
        """Convert lat/lon to grid indices"""
        i = int((lat - self.bounds['lat_min']) / self.lat_step)
        j = int((lon - self.bounds['lon_min']) / self.lon_step)
        
        # Clamp to grid
        i = max(0, min(i, self.grid_height - 1))
        j = max(0, min(j, self.grid_width - 1))
        
        return i, j
    
    def _grid_to_latlon(self, i: int, j: int) -> Tuple[float, float]:
        """Convert grid indices to lat/lon"""
        lat = self.bounds['lat_min'] + i * self.lat_step
        lon = self.bounds['lon_min'] + j * self.lon_step
        return lat, lon
    
    def _altitude_to_level(self, alt: float) -> int:
        """Convert altitude to discrete level"""
        level = int((alt - self.min_altitude) / self.altitude_step)
        return max(0, min(level, self.altitude_levels - 1))
    
    def _level_to_altitude(self, level: int) -> float:
        """Convert level to altitude"""
        return self.min_altitude + level * self.altitude_step
    
    def _node_to_latlon(self, node: Tuple[int, int, int]) -> Tuple[float, float, float]:
        """Convert node (i, j, level) to (lat, lon, alt)"""
        i, j, level = node
        lat, lon = self._grid_to_latlon(i, j)
        alt = self._level_to_altitude(level)
        return (lat, lon, alt)
    
    def _is_valid_grid(self, i: int, j: int) -> bool:
        """Check if grid position is valid"""
        return 0 <= i < self.grid_height and 0 <= j < self.grid_width
    
    def _get_safe_altitude(self, i: int, j: int) -> float:
        """Get safe altitude for position"""
        ground = self.dem[i, j]
        obstacle = self.obstacle_map[i, j]
        
        # Altitude = max(ground, obstacle) + safety_margin + min_altitude
        min_safe = max(ground, obstacle) + self.safety_margin
        
        # Use middle of altitude range
        return min_safe + (self.max_altitude - self.min_altitude) / 2
    
    def _is_collision_free(self, i: int, j: int, alt: float) -> bool:
        """Check if position is collision-free"""
        if not self._is_valid_grid(i, j):
            return False
        
        ground = self.dem[i, j]
        obstacle = self.obstacle_map[i, j]
        
        # Check clearance
        required_alt = max(ground, obstacle) + self.safety_margin
        
        return alt >= required_alt
    
    def _get_neighbors(self, node: Tuple[int, int, int]) -> List[Tuple[int, int, int]]:
        """Get valid neighbors"""
        i, j, level = node
        neighbors = []
        
        # 8-directional horizontal movement
        for di in [-1, 0, 1]:
            for dj in [-1, 0, 1]:
                if di == 0 and dj == 0:
                    continue
                
                # Altitude changes: same, up, down
                for dlevel in [-1, 0, 1]:
                    new_i = i + di
                    new_j = j + dj
                    new_level = level + dlevel
                    
                    if not self._is_valid_grid(new_i, new_j):
                        continue
                    
                    if new_level < 0 or new_level >= self.altitude_levels:
                        continue
                    
                    # Check collision
                    new_alt = self._level_to_altitude(new_level)
                    if not self._is_collision_free(new_i, new_j, new_alt):
                        continue
                    
                    neighbors.append((new_i, new_j, new_level))
        
        return neighbors
    
    def _heuristic(self, node1: Tuple[int, int, int], 
                   node2: Tuple[int, int, int]) -> float:
        """A* heuristic (Euclidean distance)"""
        i1, j1, l1 = node1
        i2, j2, l2 = node2
        
        # Convert to meters (rough approximation)
        lat_dist = (i2 - i1) * self.lat_step * 111000  # 1 degree lat ≈ 111km
        lon_dist = (j2 - j1) * self.lon_step * 88000   # 1 degree lon ≈ 88km at Seoul
        alt_dist = (l2 - l1) * self.altitude_step
        
        return np.sqrt(lat_dist**2 + lon_dist**2 + alt_dist**2)
    
    def _movement_cost(self, node1: Tuple[int, int, int],
                      node2: Tuple[int, int, int]) -> float:
        """Cost of moving between nodes"""
        return self._heuristic(node1, node2)
    
    def _get_risk_cost(self, node1: Tuple[int, int, int],
                      node2: Tuple[int, int, int]) -> float:
        """Risk cost between nodes"""
        i1, j1, _ = node1
        i2, j2, _ = node2
        
        # Sample risk along the segment
        num_samples = 3
        total_risk = 0
        
        for k in range(num_samples):
            t = k / (num_samples - 1) if num_samples > 1 else 0.5
            i = int(i1 + t * (i2 - i1))
            j = int(j1 + t * (j2 - j1))
            
            if self._is_valid_grid(i, j):
                total_risk += self.risk_map[i, j]
        
        return total_risk / num_samples
    
    def _is_goal(self, node: Tuple[int, int, int],
                 goal: Tuple[int, int, int]) -> bool:
        """Check if reached goal"""
        i1, j1, _ = node
        i2, j2, _ = goal
        
        # Within 2 grid cells
        return abs(i1 - i2) <= 2 and abs(j1 - j2) <= 2
    
    def _reconstruct_path(self, came_from: dict,
                         current: Tuple[int, int, int],
                         goal: Tuple[int, int, int]) -> List[Tuple[int, int, int]]:
        """Reconstruct path from came_from dict"""
        path = [goal, current]
        
        while current in came_from:
            current = came_from[current]
            path.append(current)
        
        path.reverse()
        return path
    
    def _calculate_distance(self, waypoints: List[Tuple[float, float, float]]) -> float:
        """Calculate total path distance in meters"""
        total = 0
        
        for i in range(len(waypoints) - 1):
            lat1, lon1, alt1 = waypoints[i]
            lat2, lon2, alt2 = waypoints[i + 1]
            
            # Haversine distance (horizontal)
            dlat = (lat2 - lat1) * 111000
            dlon = (lon2 - lon1) * 88000
            dalt = alt2 - alt1
            
            total += np.sqrt(dlat**2 + dlon**2 + dalt**2)
        
        return total
    
    def _calculate_risk(self, waypoints: List[Tuple[float, float, float]]) -> float:
        """Calculate average risk along path"""
        total_risk = 0
        
        for lat, lon, alt in waypoints:
            i, j = self._latlon_to_grid(lat, lon)
            if self._is_valid_grid(i, j):
                total_risk += self.risk_map[i, j]
        
        return total_risk / len(waypoints) if waypoints else 0
