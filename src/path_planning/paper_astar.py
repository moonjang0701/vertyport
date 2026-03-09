"""
IEEE MAES Paper-Compliant A* Path Planning
논문 Algorithm 1 구현: 3D Discretized Airspace A* Path Planning

References:
- Paper: "A Holistic Design and Simulation of Advanced UTM Services for UAM"
- Section: Operation Plan Preparation & Optimisation (Algorithm 1)
"""

import numpy as np
import heapq
from typing import List, Tuple, Optional, Set
from dataclasses import dataclass


@dataclass
class PathSegment:
    """Path segment with detailed information"""
    start_pos: Tuple[float, float, float]  # (lat, lon, alt)
    end_pos: Tuple[float, float, float]
    start_cell: Tuple[int, int, int]  # (i, j, k) grid indices
    end_cell: Tuple[int, int, int]
    distance: float  # meters
    risk: float  # Risk value from Equation 1
    duration: float  # seconds


@dataclass
class FlightPlan:
    """Complete flight plan from A* algorithm"""
    waypoints: List[Tuple[float, float, float]]  # (lat, lon, alt)
    segments: List[PathSegment]
    cells_traversed: List[Tuple[int, int, int]]  # Grid cells used
    
    total_distance: float  # meters
    total_duration: float  # seconds
    avg_risk: float
    max_risk: float
    
    success: bool = True
    reroute_required: bool = False  # True if risk > threshold


class PaperCompliantAstar:
    """
    IEEE MAES 논문 Algorithm 1 구현
    
    Features:
    - 3D discretized airspace (uniform volume cells)
    - 10-direction connectivity (8 horizontal + 2 vertical)
    - Static obstacle avoidance (terrain, buildings, airspace restrictions)
    - Single optimal path generation (NOT multiple random paths)
    """
    
    def __init__(self,
                 dem: np.ndarray,
                 obstacle_map: np.ndarray,
                 risk_map: np.ndarray,
                 lat_min: float,
                 lat_max: float,
                 lon_min: float,
                 lon_max: float,
                 min_altitude: float = 50.0,
                 max_altitude: float = 150.0,
                 num_altitude_levels: int = 5,
                 safety_margin: float = 30.0):
        """
        Initialize paper-compliant A* planner
        
        Args:
            dem: Digital Elevation Model (ground height)
            obstacle_map: Building/obstacle heights
            risk_map: Pre-computed risk values per cell (from Equation 1)
            lat_min, lat_max, lon_min, lon_max: Geographic bounds
            min_altitude, max_altitude: Flight altitude range
            num_altitude_levels: Number of discrete altitude levels
            safety_margin: Minimum clearance above obstacles (meters)
        """
        self.dem = dem
        self.obstacle_map = obstacle_map
        self.risk_map = risk_map
        
        self.lat_min = lat_min
        self.lat_max = lat_max
        self.lon_min = lon_min
        self.lon_max = lon_max
        
        self.min_altitude = min_altitude
        self.max_altitude = max_altitude
        self.num_altitude_levels = num_altitude_levels
        self.safety_margin = safety_margin
        
        # Grid dimensions
        self.grid_h, self.grid_w = dem.shape
        self.lat_step = (lat_max - lat_min) / self.grid_h
        self.lon_step = (lon_max - lon_min) / self.grid_w
        self.alt_step = (max_altitude - min_altitude) / (num_altitude_levels - 1)
        
        # 10-direction connectivity (논문 기준)
        # 8 horizontal + 2 vertical
        self.directions = [
            # Horizontal movements (same altitude)
            (-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0),  # Cardinal
            (-1, -1, 0), (-1, 1, 0), (1, -1, 0), (1, 1, 0),  # Diagonal
            # Vertical movements
            (0, 0, -1), (0, 0, 1)  # Up/Down
        ]
    
    def latlon_to_grid(self, lat: float, lon: float) -> Tuple[int, int]:
        """Convert lat/lon to grid indices"""
        i = int((lat - self.lat_min) / self.lat_step)
        j = int((lon - self.lon_min) / self.lon_step)
        i = np.clip(i, 0, self.grid_h - 1)
        j = np.clip(j, 0, self.grid_w - 1)
        return i, j
    
    def grid_to_latlon(self, i: int, j: int) -> Tuple[float, float]:
        """Convert grid indices to lat/lon"""
        lat = self.lat_min + i * self.lat_step
        lon = self.lon_min + j * self.lon_step
        return lat, lon
    
    def altitude_to_level(self, altitude: float) -> int:
        """Convert altitude to discrete level"""
        level = int((altitude - self.min_altitude) / self.alt_step)
        return np.clip(level, 0, self.num_altitude_levels - 1)
    
    def level_to_altitude(self, level: int) -> float:
        """Convert discrete level to altitude"""
        return self.min_altitude + level * self.alt_step
    
    def is_valid_cell(self, i: int, j: int, k: int) -> bool:
        """Check if cell is valid (논문: terrain + obstacle check)"""
        # Boundary check
        if i < 0 or i >= self.grid_h or j < 0 or j >= self.grid_w:
            return False
        if k < 0 or k >= self.num_altitude_levels:
            return False
        
        # Altitude check: must be above ground + obstacles + safety margin
        altitude = self.level_to_altitude(k)
        ground_height = self.dem[i, j]
        obstacle_height = self.obstacle_map[i, j]
        required_altitude = ground_height + obstacle_height + self.safety_margin
        
        return altitude >= required_altitude
    
    def heuristic(self, cell1: Tuple[int, int, int], cell2: Tuple[int, int, int]) -> float:
        """A* heuristic (Euclidean distance)"""
        i1, j1, k1 = cell1
        i2, j2, k2 = cell2
        
        # Convert to physical coordinates
        lat1, lon1 = self.grid_to_latlon(i1, j1)
        lat2, lon2 = self.grid_to_latlon(i2, j2)
        alt1 = self.level_to_altitude(k1)
        alt2 = self.level_to_altitude(k2)
        
        # Distance in meters (approximate)
        d_lat = (lat2 - lat1) * 111000  # 1 deg lat ≈ 111 km
        d_lon = (lon2 - lon1) * 88000   # 1 deg lon ≈ 88 km (at Seoul latitude)
        d_alt = alt2 - alt1
        
        return np.sqrt(d_lat**2 + d_lon**2 + d_alt**2)
    
    def plan_path(self,
                  origin_lat: float,
                  origin_lon: float,
                  dest_lat: float,
                  dest_lon: float,
                  cruise_altitude: Optional[float] = None,
                  max_iterations: int = 50000,
                  verbose: bool = True) -> FlightPlan:
        """
        IEEE MAES Algorithm 1: 3D A* Path Planning
        
        Args:
            origin_lat, origin_lon: Start coordinates
            dest_lat, dest_lon: Destination coordinates
            cruise_altitude: Preferred cruise altitude (auto if None)
            max_iterations: Maximum A* iterations
            verbose: Print debug info
        
        Returns:
            FlightPlan with single optimal path
        """
        
        # Convert to grid
        start_i, start_j = self.latlon_to_grid(origin_lat, origin_lon)
        goal_i, goal_j = self.latlon_to_grid(dest_lat, dest_lon)
        
        # Determine altitude levels
        if cruise_altitude is None:
            # Auto: use middle altitude level
            start_k = self.num_altitude_levels // 2
            goal_k = self.num_altitude_levels // 2
        else:
            start_k = self.altitude_to_level(cruise_altitude)
            goal_k = self.altitude_to_level(cruise_altitude)
        
        # Find valid start altitude
        for k in range(start_k, self.num_altitude_levels):
            if self.is_valid_cell(start_i, start_j, k):
                start_k = k
                break
        
        # Find valid goal altitude
        for k in range(goal_k, self.num_altitude_levels):
            if self.is_valid_cell(goal_i, goal_j, k):
                goal_k = k
                break
        
        start_cell = (start_i, start_j, start_k)
        goal_cell = (goal_i, goal_j, goal_k)
        
        if verbose:
            print(f"🛫 A* Planning: ({origin_lat:.4f}, {origin_lon:.4f}) → ({dest_lat:.4f}, {dest_lon:.4f})")
            print(f"   Grid: {start_cell} → {goal_cell}")
            print(f"   Altitude: {self.level_to_altitude(start_k):.0f}m → {self.level_to_altitude(goal_k):.0f}m")
        
        # A* algorithm
        open_set = []
        heapq.heappush(open_set, (0.0, start_cell))
        
        came_from = {}
        g_score = {start_cell: 0.0}
        f_score = {start_cell: self.heuristic(start_cell, goal_cell)}
        
        closed_set: Set[Tuple[int, int, int]] = set()
        iterations = 0
        
        while open_set and iterations < max_iterations:
            iterations += 1
            
            _, current = heapq.heappop(open_set)
            
            if current == goal_cell:
                # Path found!
                if verbose:
                    print(f"✅ Path found in {iterations} iterations")
                
                # Reconstruct path
                path_cells = []
                cell = current
                while cell in came_from:
                    path_cells.append(cell)
                    cell = came_from[cell]
                path_cells.append(start_cell)
                path_cells.reverse()
                
                # Convert to FlightPlan
                return self._build_flight_plan(path_cells, verbose=verbose)
            
            closed_set.add(current)
            i, j, k = current
            
            # Explore neighbors (10 directions)
            for di, dj, dk in self.directions:
                neighbor = (i + di, j + dj, k + dk)
                
                if not self.is_valid_cell(*neighbor):
                    continue
                
                if neighbor in closed_set:
                    continue
                
                # Cost: distance + risk penalty
                ni, nj, nk = neighbor
                move_dist = self.heuristic(current, neighbor)
                risk_penalty = self.risk_map[ni, nj] * 100.0  # Weight risk heavily
                tentative_g = g_score[current] + move_dist + risk_penalty
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal_cell)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # No path found
        if verbose:
            print(f"❌ No path found after {iterations} iterations")
        
        return FlightPlan(
            waypoints=[],
            segments=[],
            cells_traversed=[],
            total_distance=0.0,
            total_duration=0.0,
            avg_risk=0.0,
            max_risk=0.0,
            success=False
        )
    
    def _build_flight_plan(self, path_cells: List[Tuple[int, int, int]], verbose: bool = False) -> FlightPlan:
        """Build FlightPlan from cell path"""
        waypoints = []
        segments = []
        risks = []
        
        cruise_speed = 50.0  # m/s (assumed aircraft speed)
        
        for idx, cell in enumerate(path_cells):
            i, j, k = cell
            lat, lon = self.grid_to_latlon(i, j)
            alt = self.level_to_altitude(k)
            waypoints.append((lat, lon, alt))
            
            # Create segment
            if idx > 0:
                prev_cell = path_cells[idx - 1]
                pi, pj, pk = prev_cell
                prev_lat, prev_lon = self.grid_to_latlon(pi, pj)
                prev_alt = self.level_to_altitude(pk)
                
                # Calculate distance
                d_lat = (lat - prev_lat) * 111000
                d_lon = (lon - prev_lon) * 88000
                d_alt = alt - prev_alt
                distance = np.sqrt(d_lat**2 + d_lon**2 + d_alt**2)
                duration = distance / cruise_speed
                
                # Average risk of segment
                segment_risk = (self.risk_map[i, j] + self.risk_map[pi, pj]) / 2.0
                risks.append(segment_risk)
                
                segment = PathSegment(
                    start_pos=(prev_lat, prev_lon, prev_alt),
                    end_pos=(lat, lon, alt),
                    start_cell=prev_cell,
                    end_cell=cell,
                    distance=distance,
                    risk=segment_risk,
                    duration=duration
                )
                segments.append(segment)
        
        total_distance = sum(seg.distance for seg in segments)
        total_duration = sum(seg.duration for seg in segments)
        avg_risk = np.mean(risks) if risks else 0.0
        max_risk = np.max(risks) if risks else 0.0
        
        if verbose:
            print(f"📊 Flight Plan:")
            print(f"   Waypoints: {len(waypoints)}")
            print(f"   Distance: {total_distance:.0f}m")
            print(f"   Duration: {total_duration:.1f}s")
            print(f"   Avg Risk: {avg_risk:.3f}")
            print(f"   Max Risk: {max_risk:.3f}")
        
        flight_plan = FlightPlan(
            waypoints=waypoints,
            segments=segments,
            cells_traversed=path_cells,
            total_distance=total_distance,
            total_duration=total_duration,
            avg_risk=avg_risk,
            max_risk=max_risk,
            success=True,
            reroute_required=False
        )
        
        return flight_plan
