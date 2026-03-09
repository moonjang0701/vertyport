"""
Route-Based Vertiport Extraction Algorithm

This module implements the core algorithm:
1. Plan multiple safe flight routes between origin-destination pairs
2. Analyze route segments for safety characteristics
3. Extract safe landing zones along routes as potential vertiport locations

Mathematical Formulation:
-------------------------
Given:
- DEM D(x,y): Digital Elevation Model
- Risk map R(x,y): Risk distribution [0,1]
- Origin O and Destination G coordinates

Find: Set of optimal vertiport locations V = {v1, v2, ..., vn}

Where each vertiport vi satisfies:
1. Safety: S(vi) > θ_safety (safety threshold)
2. Flatness: σ(D(vi)) < θ_flatness (terrain variance threshold)
3. Clearance: min_distance(vi, obstacles) > θ_clearance
4. Coverage: vi is reachable from multiple routes
"""

import numpy as np
from typing import List, Tuple, Set, Dict, Optional
from dataclasses import dataclass, field
from scipy.ndimage import gaussian_filter
from scipy.spatial import distance_matrix
import logging

from ..path_planning.path_planner import PathPlanner3D, FlightPath, Waypoint
from ..data_processing.vworld_loader import VWorldDEMData

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class SafeZone:
    """Safe landing zone identified along flight routes"""
    lat: float
    lon: float
    elevation: float
    safety_score: float
    flatness_score: float
    clearance: float
    route_coverage: int  # Number of routes passing through
    area: float  # Safe zone area in m²
    
    @property
    def total_score(self) -> float:
        """
        Combined score for vertiport suitability
        
        Formula:
        S_total = w1*S_safety + w2*S_flatness + w3*log(1 + route_coverage)
        
        where:
        - w1, w2, w3 are weights (default: 0.4, 0.3, 0.3)
        - log term rewards zones covered by multiple routes
        """
        w1, w2, w3 = 0.4, 0.3, 0.3
        coverage_score = np.log1p(self.route_coverage) / np.log1p(10)  # Normalize
        return w1 * self.safety_score + w2 * self.flatness_score + w3 * coverage_score


@dataclass
class RouteAnalysis:
    """Analysis results for a planned route"""
    route: FlightPath
    safe_zones: List[SafeZone]
    risk_profile: np.ndarray  # Risk along route
    safety_score: float
    total_length: float
    

class RouteBasedVertiportExtractor:
    """
    Extract optimal vertiport locations from safe flight routes
    
    Algorithm Steps:
    ---------------
    1. Route Planning Phase:
       - Generate N candidate routes between O-D pairs
       - Use A* with safety-weighted cost function
       
    2. Safe Zone Identification:
       - Scan each route for flat, safe segments
       - Apply safety criteria (flatness, clearance, low risk)
       
    3. Clustering & Selection:
       - Cluster nearby safe zones
       - Select representative zones with highest scores
       
    4. Network Optimization:
       - Ensure minimum distance between vertiports
       - Maximize route coverage
    """
    
    def __init__(self,
                 dem_data: VWorldDEMData,
                 risk_map: np.ndarray,
                 population_density: Optional[np.ndarray] = None):
        """
        Initialize extractor
        
        Args:
            dem_data: V-World DEM data
            risk_map: Risk assessment map [0,1]
            population_density: Population density (optional)
        """
        self.dem_data = dem_data
        self.risk_map = risk_map
        self.population_density = population_density
        
        # Safety thresholds (from literature and aviation standards)
        # Note: These are relaxed for testing with synthetic data
        self.safety_threshold = 0.5      # Minimum safety score (relaxed from 0.7)
        self.flatness_threshold = 10.0   # Maximum std dev in elevation (m) (relaxed from 5.0)
        self.clearance_threshold = 30.0  # Minimum clearance (m) (relaxed from 50.0)
        self.min_zone_area = 400.0       # Minimum landing area (relaxed from 900.0)
        
        # Vertiport spacing
        self.min_vertiport_distance = 500.0  # meters
        
    def extract_vertiports_from_routes(self,
                                       origin: Tuple[float, float],
                                       destination: Tuple[float, float],
                                       num_routes: int = 5,
                                       num_vertiports: int = 10) -> List[SafeZone]:
        """
        Main algorithm: Extract vertiport locations from multiple routes
        
        Args:
            origin: (lat, lon) origin coordinates
            destination: (lat, lon) destination coordinates  
            num_routes: Number of candidate routes to generate
            num_vertiports: Target number of vertiports to extract
            
        Returns:
            List of optimal vertiport locations (SafeZone objects)
            
        Algorithm:
        ---------
        Step 1: Generate diverse safe routes
        Step 2: Identify safe zones along each route
        Step 3: Cluster and merge nearby zones
        Step 4: Rank zones by suitability score
        Step 5: Select top N vertiports with spacing constraint
        """
        logger.info(f"Extracting vertiports between {origin} and {destination}")
        logger.info(f"Generating {num_routes} candidate routes...")
        
        # Step 1: Generate multiple diverse routes
        routes = self._generate_diverse_routes(origin, destination, num_routes)
        logger.info(f"Generated {len(routes)} valid routes")
        
        # Step 2: Identify safe zones along all routes
        all_safe_zones = []
        for i, route in enumerate(routes):
            zones = self._identify_safe_zones_in_route(route, route_id=i)
            all_safe_zones.extend(zones)
            logger.info(f"Route {i+1}: Found {len(zones)} safe zones")
        
        logger.info(f"Total safe zones identified: {len(all_safe_zones)}")
        
        # Step 3: Cluster nearby safe zones
        clustered_zones = self._cluster_safe_zones(all_safe_zones)
        logger.info(f"Clustered into {len(clustered_zones)} unique locations")
        
        # Step 4: Rank by total score
        clustered_zones.sort(key=lambda z: z.total_score, reverse=True)
        
        # Step 5: Select top vertiports with distance constraint
        selected_vertiports = self._select_spaced_vertiports(
            clustered_zones, num_vertiports
        )
        
        logger.info(f"Selected {len(selected_vertiports)} optimal vertiports")
        
        return selected_vertiports
    
    def _generate_diverse_routes(self,
                                 origin: Tuple[float, float],
                                 destination: Tuple[float, float],
                                 num_routes: int) -> List[FlightPath]:
        """
        Generate diverse safe routes using A* with varied parameters
        
        Diversity Strategy:
        - Vary altitude preferences
        - Perturb risk weights
        - Use different heuristic scaling
        """
        routes = []
        
        # Convert lat/lon to grid coordinates
        origin_grid = self._latlon_to_grid(origin[0], origin[1])
        dest_grid = self._latlon_to_grid(destination[0], destination[1])
        
        # Clip to valid range
        origin_grid = (
            min(max(0, origin_grid[0]), self.dem_data.shape[1]-1),
            min(max(0, origin_grid[1]), self.dem_data.shape[0]-1)
        )
        dest_grid = (
            min(max(0, dest_grid[0]), self.dem_data.shape[1]-1),
            min(max(0, dest_grid[1]), self.dem_data.shape[0]-1)
        )
        
        # Get base elevation
        origin_elev = self.dem_data.elevation[origin_grid[1], origin_grid[0]]
        dest_elev = self.dem_data.elevation[dest_grid[1], dest_grid[0]]
        
        logger.info(f"Origin grid: {origin_grid}, elevation: {origin_elev:.1f}m")
        logger.info(f"Destination grid: {dest_grid}, elevation: {dest_elev:.1f}m")
        
        for i in range(num_routes):
            # Vary flight altitude for diversity
            altitude_offset = 50 + i * 20
            
            # Vary risk weight for diversity
            risk_weight = 0.3 + i * 0.1
            
            try:
                # Create a wrapped DEM object that's compatible with PathPlanner3D
                from ..data_processing.dem_processor import DEMData
                
                # Convert VWorldDEMData to DEMData format
                dem_compat = DEMData(
                    elevation=self.dem_data.elevation,
                    x_coords=np.linspace(0, self.dem_data.shape[1]-1, self.dem_data.shape[1]),
                    y_coords=np.linspace(0, self.dem_data.shape[0]-1, self.dem_data.shape[0]),
                    resolution=10.0,  # Approximate resolution in meters
                    bounds=(0, 0, self.dem_data.shape[1]-1, self.dem_data.shape[0]-1)
                )
                
                # Plan route with current parameters
                planner = PathPlanner3D(
                    dem_compat,
                    self.risk_map,
                    min_altitude=50.0,
                    max_altitude=150.0 + i * 10,
                    safety_margin=30.0
                )
                
                start = (origin_grid[0], origin_grid[1], origin_elev + altitude_offset)
                goal = (dest_grid[0], dest_grid[1], dest_elev + altitude_offset)
                
                logger.info(f"Planning route {i+1} from {start} to {goal}")
                
                path = planner.plan_path(start, goal, num_altitude_levels=3)
                
                if path:
                    routes.append(path)
                    logger.info(f"Route {i+1} found: {len(path.waypoints)} waypoints, "
                              f"distance: {path.total_distance:.1f}m")
                else:
                    logger.warning(f"Route {i+1}: No path found")
                    
            except Exception as e:
                logger.warning(f"Route {i+1} planning failed: {e}")
                import traceback
                traceback.print_exc()
                continue
        
        return routes
    
    def _identify_safe_zones_in_route(self,
                                     route: FlightPath,
                                     route_id: int,
                                     segment_length: int = 10) -> List[SafeZone]:
        """
        Scan route and identify safe landing zones
        
        Safe Zone Criteria:
        ------------------
        1. Flatness: σ(elevation) < θ_flatness
        2. Safety: R(x,y) < (1 - θ_safety)
        3. Clearance: distance to obstacles > θ_clearance
        4. Area: sufficient flat area for landing
        
        Args:
            route: Flight path to analyze
            route_id: Route identifier
            segment_length: Analysis window size (waypoints)
            
        Returns:
            List of safe zones found along route
        """
        safe_zones = []
        waypoints = route.waypoints
        
        # Scan route with sliding window
        for i in range(0, len(waypoints) - segment_length, segment_length // 2):
            segment = waypoints[i:i + segment_length]
            
            # Analyze segment center
            center_wp = segment[len(segment) // 2]
            
            # Convert to lat/lon
            lat, lon = self._grid_to_latlon(center_wp.x, center_wp.y)
            
            # Calculate safety metrics
            safety = self._calculate_safety_score(center_wp.x, center_wp.y)
            flatness = self._calculate_flatness_score(center_wp.x, center_wp.y)
            clearance = self._calculate_clearance(center_wp.x, center_wp.y)
            area = self._calculate_flat_area(center_wp.x, center_wp.y)
            
            # Check if zone meets criteria
            if (safety > self.safety_threshold and
                flatness > 0.5 and  # Normalized flatness score
                clearance > self.clearance_threshold and
                area > self.min_zone_area):
                
                zone = SafeZone(
                    lat=lat,
                    lon=lon,
                    elevation=center_wp.z,
                    safety_score=safety,
                    flatness_score=flatness,
                    clearance=clearance,
                    route_coverage=1,
                    area=area
                )
                safe_zones.append(zone)
        
        return safe_zones
    
    def _calculate_safety_score(self, x: float, y: float, radius: int = 5) -> float:
        """
        Calculate safety score for location
        
        Formula:
        S_safety = 1 - R_avg - k*R_max
        
        where:
        - R_avg: Average risk in radius
        - R_max: Maximum risk in radius
        - k: Weight for max risk (default: 0.2)
        """
        x_idx = int(x)
        y_idx = int(y)
        
        y_start = max(0, y_idx - radius)
        y_end = min(self.risk_map.shape[0], y_idx + radius)
        x_start = max(0, x_idx - radius)
        x_end = min(self.risk_map.shape[1], x_idx + radius)
        
        local_risk = self.risk_map[y_start:y_end, x_start:x_end]
        
        avg_risk = np.mean(local_risk)
        max_risk = np.max(local_risk)
        
        safety = 1.0 - avg_risk - 0.2 * max_risk
        return float(np.clip(safety, 0, 1))
    
    def _calculate_flatness_score(self, x: float, y: float, radius: int = 5) -> float:
        """
        Calculate terrain flatness score
        
        Formula:
        S_flatness = exp(-σ²/σ_threshold²)
        
        where:
        - σ: Standard deviation of elevation in radius
        - σ_threshold: Flatness threshold
        """
        x_idx = int(x)
        y_idx = int(y)
        
        y_start = max(0, y_idx - radius)
        y_end = min(self.dem_data.shape[0], y_idx + radius)
        x_start = max(0, x_idx - radius)
        x_end = min(self.dem_data.shape[1], x_idx + radius)
        
        local_elev = self.dem_data.elevation[y_start:y_end, x_start:x_end]
        
        std_dev = np.std(local_elev)
        flatness = np.exp(-(std_dev**2) / (self.flatness_threshold**2))
        
        return float(flatness)
    
    def _calculate_clearance(self, x: float, y: float) -> float:
        """Calculate minimum clearance to obstacles"""
        # Simplified: use elevation difference as proxy
        x_idx = int(x)
        y_idx = int(y)
        
        center_elev = self.dem_data.elevation[y_idx, x_idx]
        
        # Check surrounding area
        radius = 10
        y_start = max(0, y_idx - radius)
        y_end = min(self.dem_data.shape[0], y_idx + radius)
        x_start = max(0, x_idx - radius)
        x_end = min(self.dem_data.shape[1], x_idx + radius)
        
        surrounding = self.dem_data.elevation[y_start:y_end, x_start:x_end]
        max_nearby = np.max(surrounding)
        
        clearance = max(0, max_nearby - center_elev)
        
        return float(clearance)
    
    def _calculate_flat_area(self, x: float, y: float, 
                            threshold: float = 2.0) -> float:
        """
        Calculate contiguous flat area around location
        
        Returns: Area in m²
        """
        # Simplified: count cells with elevation close to center
        x_idx = int(x)
        y_idx = int(y)
        
        center_elev = self.dem_data.elevation[y_idx, x_idx]
        
        radius = 15
        y_start = max(0, y_idx - radius)
        y_end = min(self.dem_data.shape[0], y_idx + radius)
        x_start = max(0, x_idx - radius)
        x_end = min(self.dem_data.shape[1], x_idx + radius)
        
        local_elev = self.dem_data.elevation[y_start:y_end, x_start:x_end]
        flat_mask = np.abs(local_elev - center_elev) < threshold
        
        num_flat_cells = np.sum(flat_mask)
        cell_area = (self.dem_data.resolution * 111000) ** 2  # Convert deg to m
        
        return float(num_flat_cells * cell_area)
    
    def _cluster_safe_zones(self, zones: List[SafeZone],
                           cluster_distance: float = 100.0) -> List[SafeZone]:
        """
        Cluster nearby safe zones and merge them
        
        Uses simple distance-based clustering
        Merges zones within cluster_distance (meters)
        """
        if not zones:
            return []
        
        # Convert to numpy array for clustering
        coords = np.array([[z.lat, z.lon] for z in zones])
        
        # Simple clustering: greedy merge
        clusters = []
        used = set()
        
        for i, zone in enumerate(zones):
            if i in used:
                continue
            
            # Find nearby zones
            cluster_zones = [zone]
            used.add(i)
            
            for j, other_zone in enumerate(zones):
                if j in used:
                    continue
                
                # Calculate distance (approximate)
                dist = self._haversine_distance(
                    zone.lat, zone.lon,
                    other_zone.lat, other_zone.lon
                )
                
                if dist < cluster_distance:
                    cluster_zones.append(other_zone)
                    used.add(j)
            
            # Merge cluster into single zone
            merged = self._merge_zones(cluster_zones)
            clusters.append(merged)
        
        return clusters
    
    def _merge_zones(self, zones: List[SafeZone]) -> SafeZone:
        """Merge multiple zones into one representative zone"""
        # Weight by safety score
        weights = np.array([z.safety_score for z in zones])
        weights = weights / np.sum(weights)
        
        avg_lat = np.average([z.lat for z in zones], weights=weights)
        avg_lon = np.average([z.lon for z in zones], weights=weights)
        avg_elev = np.average([z.elevation for z in zones], weights=weights)
        
        max_safety = max(z.safety_score for z in zones)
        max_flatness = max(z.flatness_score for z in zones)
        max_clearance = max(z.clearance for z in zones)
        total_coverage = sum(z.route_coverage for z in zones)
        max_area = max(z.area for z in zones)
        
        return SafeZone(
            lat=avg_lat,
            lon=avg_lon,
            elevation=avg_elev,
            safety_score=max_safety,
            flatness_score=max_flatness,
            clearance=max_clearance,
            route_coverage=total_coverage,
            area=max_area
        )
    
    def _select_spaced_vertiports(self, zones: List[SafeZone],
                                 target_num: int) -> List[SafeZone]:
        """
        Select vertiports ensuring minimum spacing
        
        Greedy algorithm: Select highest score zones while maintaining spacing
        """
        selected = []
        
        for zone in zones:
            if len(selected) >= target_num:
                break
            
            # Check distance to already selected vertiports
            too_close = False
            for selected_zone in selected:
                dist = self._haversine_distance(
                    zone.lat, zone.lon,
                    selected_zone.lat, selected_zone.lon
                )
                if dist < self.min_vertiport_distance:
                    too_close = True
                    break
            
            if not too_close:
                selected.append(zone)
        
        return selected
    
    def _latlon_to_grid(self, lat: float, lon: float) -> Tuple[int, int]:
        """Convert lat/lon to grid indices"""
        x = int((lon - self.dem_data.lon_min) / 
               (self.dem_data.lon_max - self.dem_data.lon_min) * 
               self.dem_data.shape[1])
        y = int((lat - self.dem_data.lat_min) / 
               (self.dem_data.lat_max - self.dem_data.lat_min) * 
               self.dem_data.shape[0])
        
        x = np.clip(x, 0, self.dem_data.shape[1] - 1)
        y = np.clip(y, 0, self.dem_data.shape[0] - 1)
        
        return (x, y)
    
    def _grid_to_latlon(self, x: float, y: float) -> Tuple[float, float]:
        """Convert grid indices to lat/lon"""
        lon = self.dem_data.lon_min + (x / self.dem_data.shape[1]) * \
              (self.dem_data.lon_max - self.dem_data.lon_min)
        lat = self.dem_data.lat_min + (y / self.dem_data.shape[0]) * \
              (self.dem_data.lat_max - self.dem_data.lat_min)
        
        return (lat, lon)
    
    @staticmethod
    def _haversine_distance(lat1: float, lon1: float,
                          lat2: float, lon2: float) -> float:
        """
        Calculate distance between two points using Haversine formula
        
        Returns: Distance in meters
        """
        R = 6371000  # Earth radius in meters
        
        phi1 = np.radians(lat1)
        phi2 = np.radians(lat2)
        dphi = np.radians(lat2 - lat1)
        dlambda = np.radians(lon2 - lon1)
        
        a = np.sin(dphi/2)**2 + np.cos(phi1) * np.cos(phi2) * np.sin(dlambda/2)**2
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
        
        return R * c
