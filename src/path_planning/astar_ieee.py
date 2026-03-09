"""
IEEE MAES Paper - Algorithm 1: A* Path Planning
3D Discretized Airspace with Real Building Obstacles

Based on Section III-B: Operation Plan Preparation & Optimisation
"""

import numpy as np
from typing import List, Tuple, Optional, Set
from dataclasses import dataclass
import heapq
from pathlib import Path
import sys

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from data_processing.building_loader import SeoulBuildingLoader
from data_processing.seoul_districts import SEOUL_DISTRICTS


@dataclass
class Waypoint3D:
    """3D waypoint in discretized airspace"""
    x: int  # Grid x index
    y: int  # Grid y index
    z: int  # Grid z index (altitude layer)
    lat: float  # Actual latitude
    lon: float  # Actual longitude
    alt: float  # Actual altitude (meters)
    
    def __hash__(self):
        return hash((self.x, self.y, self.z))
    
    def __eq__(self, other):
        return (self.x, self.y, self.z) == (other.x, other.y, other.z)


@dataclass
class FlightPlan:
    """Flight plan with waypoints and metadata"""
    waypoints: List[Waypoint3D]
    total_distance: float
    total_risk: float
    phases: List[str]  # ['takeoff', 'cruise', 'landing']


class DiscretizedAirspace:
    """
    3D Discretized Airspace
    IEEE MAES Section III-B
    """
    
    def __init__(self,
                 lat_min: float, lat_max: float,
                 lon_min: float, lon_max: float,
                 grid_size_horizontal: int = 50,  # Grid resolution
                 altitude_layers: List[float] = None,
                 min_separation: float = 50.0):  # meters
        """
        Initialize discretized airspace
        
        Args:
            lat_min, lat_max, lon_min, lon_max: Bounds
            grid_size_horizontal: Horizontal grid resolution
            altitude_layers: Altitude levels (meters AGL)
            min_separation: Minimum lateral separation (meters)
        """
        self.lat_min = lat_min
        self.lat_max = lat_max
        self.lon_min = lon_min
        self.lon_max = lon_max
        self.grid_size = grid_size_horizontal
        
        # Altitude layers (논문: nominal flight levels)
        if altitude_layers is None:
            # Default: 50m, 75m, 100m, 125m, 150m
            self.altitude_layers = [50.0, 75.0, 100.0, 125.0, 150.0]
        else:
            self.altitude_layers = altitude_layers
        
        self.num_layers = len(self.altitude_layers)
        self.min_separation = min_separation
        
        # Create grid
        self.lat_grid = np.linspace(lat_min, lat_max, grid_size_horizontal)
        self.lon_grid = np.linspace(lon_min, lon_max, grid_size_horizontal)
        
        # 10-directional connectivity (논문: 8 horizontal + 2 vertical)
        self.horizontal_directions = [
            (0, 1, 0),   # East
            (1, 0, 0),   # North
            (0, -1, 0),  # West
            (-1, 0, 0),  # South
            (1, 1, 0),   # NE
            (1, -1, 0),  # NW
            (-1, 1, 0),  # SE
            (-1, -1, 0), # SW
        ]
        
        self.vertical_directions = [
            (0, 0, 1),   # Up
            (0, 0, -1),  # Down
        ]
        
        self.all_directions = self.horizontal_directions + self.vertical_directions
        
        # Obstacle map (will be set later)
        self.obstacle_map = np.zeros((grid_size_horizontal, grid_size_horizontal))
        self.restricted_zones = set()  # Set of (x, y, z) tuples
        self.high_risk_zones = set()
        
    def set_obstacle_map(self, obstacle_map: np.ndarray):
        """Set building obstacle map"""
        # Resize if needed
        if obstacle_map.shape != (self.grid_size, self.grid_size):
            from scipy.ndimage import zoom
            scale = self.grid_size / obstacle_map.shape[0]
            self.obstacle_map = zoom(obstacle_map, scale, order=1)
        else:
            self.obstacle_map = obstacle_map
    
    def add_restricted_zone(self, lat: float, lon: float, radius: float, alt_range: Tuple[float, float]):
        """Add airspace restriction (논문: airspace restrictions)"""
        x, y = self.latlon_to_grid(lat, lon)
        z_min = self.altitude_to_layer(alt_range[0])
        z_max = self.altitude_to_layer(alt_range[1])
        
        # Add cells in radius
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                for z in range(z_min, z_max + 1):
                    if 0 <= x+dx < self.grid_size and 0 <= y+dy < self.grid_size:
                        self.restricted_zones.add((x+dx, y+dy, z))
    
    def add_high_risk_zone(self, lat: float, lon: float, radius: float):
        """Add high-risk zone (논문: high population density areas)"""
        x, y = self.latlon_to_grid(lat, lon)
        
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                for z in range(self.num_layers):
                    if 0 <= x+dx < self.grid_size and 0 <= y+dy < self.grid_size:
                        self.high_risk_zones.add((x+dx, y+dy, z))
    
    def latlon_to_grid(self, lat: float, lon: float) -> Tuple[int, int]:
        """Convert lat/lon to grid indices"""
        x = int(np.searchsorted(self.lat_grid, lat))
        y = int(np.searchsorted(self.lon_grid, lon))
        x = np.clip(x, 0, self.grid_size - 1)
        y = np.clip(y, 0, self.grid_size - 1)
        return x, y
    
    def altitude_to_layer(self, altitude: float) -> int:
        """Convert altitude to layer index"""
        for i, alt in enumerate(self.altitude_layers):
            if altitude <= alt:
                return i
        return self.num_layers - 1
    
    def grid_to_latlon(self, x: int, y: int) -> Tuple[float, float]:
        """Convert grid indices to lat/lon"""
        lat = self.lat_grid[x] if x < len(self.lat_grid) else self.lat_max
        lon = self.lon_grid[y] if y < len(self.lon_grid) else self.lon_max
        return lat, lon
    
    def is_valid_cell(self, x: int, y: int, z: int) -> bool:
        """Check if cell is valid (not obstacle or restricted)"""
        # Bounds check
        if not (0 <= x < self.grid_size and 0 <= y < self.grid_size and 0 <= z < self.num_layers):
            return False
        
        # Restricted zone check
        if (x, y, z) in self.restricted_zones:
            return False
        
        # Obstacle check (building clearance: 30m safety margin)
        altitude = self.altitude_layers[z]
        building_height = self.obstacle_map[x, y]
        if altitude < building_height + 30.0:  # 논문: h_clearance = 30m
            return False
        
        return True
    
    def get_neighbors(self, waypoint: Waypoint3D) -> List[Waypoint3D]:
        """Get valid neighboring cells (10-directional)"""
        neighbors = []
        
        for dx, dy, dz in self.all_directions:
            nx, ny, nz = waypoint.x + dx, waypoint.y + dy, waypoint.z + dz
            
            if self.is_valid_cell(nx, ny, nz):
                lat, lon = self.grid_to_latlon(nx, ny)
                alt = self.altitude_layers[nz]
                
                neighbors.append(Waypoint3D(nx, ny, nz, lat, lon, alt))
        
        return neighbors


class AStarPathPlanner:
    """
    A* Path Planning Algorithm
    IEEE MAES Algorithm 1
    """
    
    def __init__(self,
                 airspace: DiscretizedAirspace,
                 risk_map: np.ndarray,
                 w_d: float = 1.0,
                 w_r: float = 50.0,
                 w_e: float = 0.1):
        """
        Initialize A* planner
        
        Args:
            airspace: Discretized airspace
            risk_map: Risk assessment map
            w_d: Distance weight
            w_r: Risk weight (논문: 50.0)
            w_e: Elevation change weight
        """
        self.airspace = airspace
        self.risk_map = risk_map
        
        # Cost weights (논문 Section III-B)
        self.w_d = w_d
        self.w_r = w_r
        self.w_e = w_e
    
    def plan_flight(self,
                   origin: Tuple[float, float],  # (lat, lon)
                   destination: Tuple[float, float],
                   cruise_altitude: float = 100.0) -> Optional[FlightPlan]:
        """
        Plan complete flight with 3 phases:
        1. Vertical take-off
        2. Cruise (A* path)
        3. Vertical landing
        
        Args:
            origin: (lat, lon)
            destination: (lat, lon)
            cruise_altitude: Cruise altitude (meters)
            
        Returns:
            FlightPlan or None if no path found
        """
        print(f"\n{'='*80}")
        print(f"🛫 IEEE MAES A* Path Planning")
        print(f"{'='*80}")
        print(f"Origin: {origin}")
        print(f"Destination: {destination}")
        print(f"Cruise altitude: {cruise_altitude}m")
        
        # Convert to grid
        start_x, start_y = self.airspace.latlon_to_grid(*origin)
        goal_x, goal_y = self.airspace.latlon_to_grid(*destination)
        cruise_z = self.airspace.altitude_to_layer(cruise_altitude)
        
        print(f"\nGrid coordinates:")
        print(f"  Start: ({start_x}, {start_y})")
        print(f"  Goal: ({goal_x}, {goal_y})")
        print(f"  Cruise layer: {cruise_z} ({self.airspace.altitude_layers[cruise_z]}m)")
        
        # Phase 1: Vertical take-off (ground → cruise altitude)
        print(f"\n📈 Phase 1: Vertical Take-off")
        takeoff_waypoints = []
        for z in range(cruise_z + 1):
            lat, lon = self.airspace.grid_to_latlon(start_x, start_y)
            alt = self.airspace.altitude_layers[z] if z < self.airspace.num_layers else cruise_altitude
            takeoff_waypoints.append(Waypoint3D(start_x, start_y, z, lat, lon, alt))
        
        print(f"  ✅ {len(takeoff_waypoints)} waypoints")
        
        # Phase 2: Cruise (A* horizontal navigation)
        print(f"\n✈️  Phase 2: Cruise (A* Planning)")
        start_wp = Waypoint3D(
            start_x, start_y, cruise_z,
            *self.airspace.grid_to_latlon(start_x, start_y),
            self.airspace.altitude_layers[cruise_z]
        )
        
        goal_wp = Waypoint3D(
            goal_x, goal_y, cruise_z,
            *self.airspace.grid_to_latlon(goal_x, goal_y),
            self.airspace.altitude_layers[cruise_z]
        )
        
        cruise_waypoints = self._astar_search(start_wp, goal_wp)
        
        if cruise_waypoints is None:
            print(f"  ❌ No path found!")
            return None
        
        print(f"  ✅ {len(cruise_waypoints)} waypoints")
        
        # Phase 3: Vertical landing (cruise altitude → ground)
        print(f"\n📉 Phase 3: Vertical Landing")
        landing_waypoints = []
        for z in range(cruise_z, -1, -1):
            lat, lon = self.airspace.grid_to_latlon(goal_x, goal_y)
            alt = self.airspace.altitude_layers[z] if z < self.airspace.num_layers else 0
            landing_waypoints.append(Waypoint3D(goal_x, goal_y, z, lat, lon, alt))
        
        print(f"  ✅ {len(landing_waypoints)} waypoints")
        
        # Combine all phases
        all_waypoints = takeoff_waypoints + cruise_waypoints[1:] + landing_waypoints[1:]
        
        # Calculate metrics
        total_distance = self._calculate_path_distance(all_waypoints)
        total_risk = self._calculate_path_risk(all_waypoints)
        
        print(f"\n📊 Flight Plan Summary:")
        print(f"  Total waypoints: {len(all_waypoints)}")
        print(f"  Total distance: {total_distance:.1f}m")
        print(f"  Average risk: {total_risk:.4f}")
        print(f"{'='*80}\n")
        
        return FlightPlan(
            waypoints=all_waypoints,
            total_distance=total_distance,
            total_risk=total_risk,
            phases=['takeoff'] * len(takeoff_waypoints) +
                   ['cruise'] * len(cruise_waypoints) +
                   ['landing'] * len(landing_waypoints)
        )
    
    def _astar_search(self, start: Waypoint3D, goal: Waypoint3D) -> Optional[List[Waypoint3D]]:
        """A* search algorithm"""
        open_set = []
        heapq.heappush(open_set, (0, id(start), start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}
        
        visited = 0
        
        while open_set:
            _, _, current = heapq.heappop(open_set)
            visited += 1
            
            # Goal check
            if current.x == goal.x and current.y == goal.y:
                print(f"  🎯 Goal reached! (visited {visited} cells)")
                return self._reconstruct_path(came_from, current)
            
            # Explore neighbors
            for neighbor in self.airspace.get_neighbors(current):
                tentative_g = g_score[current] + self._cost(current, neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                    
                    heapq.heappush(open_set, (f_score[neighbor], id(neighbor), neighbor))
        
        print(f"  ❌ No path found after visiting {visited} cells")
        return None
    
    def _cost(self, from_wp: Waypoint3D, to_wp: Waypoint3D) -> float:
        """
        Segment cost function
        c(n1, n2) = w_d × distance + w_r × risk + w_e × elevation_change
        """
        # Distance
        distance = self._euclidean_distance(from_wp, to_wp)
        
        # Risk (average)
        risk = self._get_risk(to_wp.x, to_wp.y)
        
        # Elevation change
        elevation_change = abs(to_wp.alt - from_wp.alt)
        
        cost = self.w_d * distance + self.w_r * risk + self.w_e * elevation_change
        
        return cost
    
    def _heuristic(self, wp: Waypoint3D, goal: Waypoint3D) -> float:
        """
        Heuristic function h(n)
        Simple Euclidean distance (admissible)
        """
        return self._euclidean_distance(wp, goal)
    
    def _euclidean_distance(self, wp1: Waypoint3D, wp2: Waypoint3D) -> float:
        """Calculate 3D Euclidean distance"""
        # Approximate lat/lon to meters (rough)
        lat_m = (wp2.lat - wp1.lat) * 111000  # 1° ≈ 111km
        lon_m = (wp2.lon - wp1.lon) * 111000 * np.cos(np.radians(wp1.lat))
        alt_m = wp2.alt - wp1.alt
        
        return np.sqrt(lat_m**2 + lon_m**2 + alt_m**2)
    
    def _get_risk(self, x: int, y: int) -> float:
        """Get risk value at grid cell"""
        if x >= self.risk_map.shape[0] or y >= self.risk_map.shape[1]:
            return 0.5  # Default risk
        return self.risk_map[x, y]
    
    def _reconstruct_path(self, came_from: dict, current: Waypoint3D) -> List[Waypoint3D]:
        """Reconstruct path from came_from map"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return list(reversed(path))
    
    def _calculate_path_distance(self, waypoints: List[Waypoint3D]) -> float:
        """Calculate total path distance"""
        total = 0.0
        for i in range(len(waypoints) - 1):
            total += self._euclidean_distance(waypoints[i], waypoints[i+1])
        return total
    
    def _calculate_path_risk(self, waypoints: List[Waypoint3D]) -> float:
        """Calculate average path risk"""
        risks = [self._get_risk(wp.x, wp.y) for wp in waypoints]
        return np.mean(risks)


# Example usage and test
if __name__ == "__main__":
    print("="*80)
    print("IEEE MAES A* Path Planning - Gwanak-gu Test")
    print("="*80)
    
    # Load Gwanak-gu buildings
    print("\n📂 Loading building data...")
    loader = SeoulBuildingLoader()
    gdf = loader.load_district_buildings("11620")
    
    if gdf is None:
        print("❌ Failed to load buildings")
        sys.exit(1)
    
    # Get bounds
    bounds = gdf.total_bounds
    lat_min, lat_max = bounds[1], bounds[3]
    lon_min, lon_max = bounds[0], bounds[2]
    
    print(f"\n📍 Gwanak-gu bounds:")
    print(f"   Lat: {lat_min:.6f} ~ {lat_max:.6f}")
    print(f"   Lon: {lon_min:.6f} ~ {lon_max:.6f}")
    
    # Create discretized airspace
    print(f"\n🌐 Creating discretized airspace...")
    airspace = DiscretizedAirspace(
        lat_min, lat_max, lon_min, lon_max,
        grid_size_horizontal=50,
        altitude_layers=[50, 75, 100, 125, 150]
    )
    
    # Create obstacle map from buildings
    print(f"\n🏢 Creating obstacle map...")
    obstacle_map, metadata = loader.create_obstacle_map(gdf, resolution=50,
                                                        lat_min=lat_min, lat_max=lat_max,
                                                        lon_min=lon_min, lon_max=lon_max)
    
    print(f"   Max building height: {metadata['max_height']:.1f}m")
    print(f"   Cells with buildings: {metadata['cells_with_buildings']}")
    
    airspace.set_obstacle_map(obstacle_map)
    
    # Create risk map (simple: based on building density)
    risk_map = obstacle_map / max(obstacle_map.max(), 1.0)  # Normalize to [0,1]
    risk_map = np.clip(risk_map * 0.5, 0, 1)  # Scale down
    
    # Create A* planner
    planner = AStarPathPlanner(airspace, risk_map)
    
    # Plan flight (south to north across Gwanak-gu)
    origin = (lat_min + 0.01, (lon_min + lon_max) / 2)
    destination = (lat_max - 0.01, (lon_min + lon_max) / 2)
    
    flight_plan = planner.plan_flight(origin, destination, cruise_altitude=100.0)
    
    if flight_plan:
        print(f"\n✅ Flight plan generated successfully!")
        print(f"   Waypoints: {len(flight_plan.waypoints)}")
        print(f"   Distance: {flight_plan.total_distance:.1f}m")
        print(f"   Risk: {flight_plan.total_risk:.4f}")
