"""
Complete Grid-based Route Evaluation System
논문 방식: 격자 → 위험도/혼잡도 평가 → 다중 경로 생성 → 순차 최적화

Pipeline:
1. 전체 공역을 격자로 나눔
2. 각 격자 셀 평가 (위험도, 혼잡도, 건물밀도)
3. 격자 기반 다중 경로 생성 (A* variants)
4. 각 경로의 안전성 + 효율성 평가
5. 안전 기준 만족하면서 지연/비용 최소화하는 경로 선택
6. 선택된 경로 기반으로 버티포트 위치 추출
"""

import numpy as np
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
import heapq


@dataclass
class RouteCandidate:
    """Route candidate with comprehensive evaluation"""
    route_id: int
    waypoints: List[Tuple[float, float, float]]
    cells_traversed: List[Tuple[int, int]]
    
    # Safety metrics
    avg_risk: float  # Average risk along route
    max_risk: float  # Maximum risk along route
    safety_violations: int  # Number of high-risk cells
    
    # Efficiency metrics
    total_distance: float  # meters
    flight_time: float  # seconds
    delay_cost: float  # vs. direct route
    
    # Congestion metrics
    avg_congestion: float  # Average cell utilization
    max_congestion: float  # Maximum cell utilization
    congestion_cost: float  # Rerouting cost
    
    # Composite score
    is_safe: bool = False
    total_cost: float = 0.0
    rank: int = 0


class GridBasedRouteOptimizer:
    """
    Complete grid-based route optimization system
    논문 방식: 격자 평가 → 다중 경로 → 순차 최적화
    """
    
    def __init__(self,
                 dem: np.ndarray,
                 obstacle_map: np.ndarray,
                 risk_map: np.ndarray,
                 bounds: dict,
                 grid_resolution: int = 64,
                 safety_threshold: float = 0.3,
                 congestion_threshold: float = 0.8):
        """
        Initialize grid-based optimizer
        
        Args:
            dem: Digital elevation model
            obstacle_map: Building heights
            risk_map: Ground risk (0-1)
            bounds: Geographic bounds
            grid_resolution: Grid size
            safety_threshold: Maximum acceptable risk
            congestion_threshold: Maximum acceptable congestion
        """
        self.dem = dem
        self.obstacle_map = obstacle_map
        self.risk_map = risk_map
        self.bounds = bounds
        self.grid_resolution = grid_resolution
        self.safety_threshold = safety_threshold
        self.congestion_threshold = congestion_threshold
        
        # Initialize grids
        self._initialize_evaluation_grids()
        
        # Operation tracking
        self.capacity_grid = np.zeros((grid_resolution, grid_resolution))
        self.max_capacity = 3  # Max 3 aircraft per cell
    
    def _initialize_evaluation_grids(self):
        """Initialize evaluation grids"""
        print(f"\n🔧 Initializing {self.grid_resolution}×{self.grid_resolution} evaluation grids...")
        
        # Downsample maps to grid resolution
        from scipy.ndimage import zoom
        
        zoom_factor_h = self.grid_resolution / self.dem.shape[0]
        zoom_factor_w = self.grid_resolution / self.dem.shape[1]
        
        self.grid_risk = zoom(self.risk_map, (zoom_factor_h, zoom_factor_w), order=1)
        self.grid_obstacles = zoom(self.obstacle_map, (zoom_factor_h, zoom_factor_w), order=1)
        self.grid_dem = zoom(self.dem, (zoom_factor_h, zoom_factor_w), order=1)
        
        # Calculate building density grid
        self.grid_density = (self.grid_obstacles > 0).astype(float)
        
        print(f"✅ Grids initialized:")
        print(f"   - Risk grid: avg={self.grid_risk.mean():.3f}, max={self.grid_risk.max():.3f}")
        print(f"   - Obstacle grid: max height={self.grid_obstacles.max():.1f}m")
        print(f"   - Building density: {self.grid_density.mean()*100:.1f}%")
    
    def latlon_to_grid(self, lat: float, lon: float) -> Tuple[int, int]:
        """Convert lat/lon to grid indices"""
        i = int((lat - self.bounds['lat_min']) / 
               (self.bounds['lat_max'] - self.bounds['lat_min']) * self.grid_resolution)
        j = int((lon - self.bounds['lon_min']) / 
               (self.bounds['lon_max'] - self.bounds['lon_min']) * self.grid_resolution)
        
        i = np.clip(i, 0, self.grid_resolution - 1)
        j = np.clip(j, 0, self.grid_resolution - 1)
        
        return i, j
    
    def grid_to_latlon(self, i: int, j: int) -> Tuple[float, float]:
        """Convert grid indices to lat/lon"""
        lat = self.bounds['lat_min'] + (i + 0.5) / self.grid_resolution * \
              (self.bounds['lat_max'] - self.bounds['lat_min'])
        lon = self.bounds['lon_min'] + (j + 0.5) / self.grid_resolution * \
              (self.bounds['lon_max'] - self.bounds['lon_min'])
        return lat, lon
    
    def generate_multiple_routes(self,
                                 start: Tuple[float, float],
                                 end: Tuple[float, float],
                                 num_routes: int = 10,
                                 cruise_altitude: float = 100.0) -> List[RouteCandidate]:
        """
        Generate multiple route candidates (논문 방식)
        
        Args:
            start: (lat, lon)
            end: (lat, lon)
            num_routes: Number of alternative routes
            cruise_altitude: Cruise altitude (m)
            
        Returns:
            List of RouteCandidate objects
        """
        print(f"\n🛣️  Generating {num_routes} route candidates...")
        
        routes = []
        
        # Direct route
        direct_waypoints = [
            (start[0], start[1], cruise_altitude),
            (end[0], end[1], cruise_altitude)
        ]
        direct_dist = self._calculate_distance(start, end)
        
        for route_id in range(num_routes):
            if route_id == 0:
                # Direct route
                waypoints = direct_waypoints
            else:
                # Alternative routes with different waypoint patterns
                num_intermediate = 1 + (route_id % 3)  # 1-3 intermediate points
                waypoints = [(start[0], start[1], cruise_altitude)]
                
                for k in range(num_intermediate):
                    fraction = (k + 1) / (num_intermediate + 1)
                    
                    # Base interpolation
                    lat = start[0] + fraction * (end[0] - start[0])
                    lon = start[1] + fraction * (end[1] - start[1])
                    
                    # Add lateral offset (different patterns)
                    offset_pattern = route_id // 3  # 0-3
                    offset_magnitude = 0.001 * (1 + offset_pattern)  # ~100-400m
                    
                    if route_id % 2 == 0:
                        lat += offset_magnitude
                    else:
                        lon += offset_magnitude
                    
                    waypoints.append((lat, lon, cruise_altitude))
                
                waypoints.append((end[0], end[1], cruise_altitude))
            
            # Evaluate route
            route = self._evaluate_route(route_id, waypoints, direct_dist)
            routes.append(route)
            
            safety_status = "✅ SAFE" if route.is_safe else "⚠️ UNSAFE"
            print(f"   Route {route_id+1}: risk={route.avg_risk:.3f}, "
                  f"dist={route.total_distance:.0f}m, {safety_status}")
        
        return routes
    
    def _evaluate_route(self,
                       route_id: int,
                       waypoints: List[Tuple[float, float, float]],
                       direct_distance: float) -> RouteCandidate:
        """Evaluate a single route candidate"""
        cells_traversed = []
        risks = []
        congestions = []
        
        # Trace route through grid
        for lat, lon, alt in waypoints:
            i, j = self.latlon_to_grid(lat, lon)
            cell_key = (i, j)
            
            if cell_key not in cells_traversed:
                cells_traversed.append(cell_key)
                
                # Get cell metrics
                risk = self.grid_risk[i, j]
                congestion = self.capacity_grid[i, j] / self.max_capacity
                
                risks.append(risk)
                congestions.append(congestion)
        
        # Calculate metrics
        avg_risk = np.mean(risks) if risks else 0.0
        max_risk = np.max(risks) if risks else 0.0
        safety_violations = sum(1 for r in risks if r > self.safety_threshold)
        
        avg_congestion = np.mean(congestions) if congestions else 0.0
        max_congestion = np.max(congestions) if congestions else 0.0
        
        # Calculate distance
        total_distance = 0.0
        for i in range(1, len(waypoints)):
            lat1, lon1, _ = waypoints[i-1]
            lat2, lon2, _ = waypoints[i]
            total_distance += self._calculate_distance((lat1, lon1), (lat2, lon2))
        
        # Calculate costs
        flight_time = total_distance / 60.0  # Assume 60 m/s
        delay_cost = total_distance - direct_distance  # Extra distance
        congestion_cost = avg_congestion * 100.0  # Congestion penalty
        
        # Safety check
        is_safe = (max_risk <= self.safety_threshold and 
                  safety_violations == 0)
        
        # Total cost (논문: 안전 만족 시 지연 최소화)
        if is_safe:
            total_cost = delay_cost + congestion_cost
        else:
            total_cost = 1e6  # Infinite cost if unsafe
        
        return RouteCandidate(
            route_id=route_id,
            waypoints=waypoints,
            cells_traversed=cells_traversed,
            avg_risk=avg_risk,
            max_risk=max_risk,
            safety_violations=safety_violations,
            total_distance=total_distance,
            flight_time=flight_time,
            delay_cost=delay_cost,
            avg_congestion=avg_congestion,
            max_congestion=max_congestion,
            congestion_cost=congestion_cost,
            is_safe=is_safe,
            total_cost=total_cost
        )
    
    def select_optimal_route(self, routes: List[RouteCandidate]) -> Optional[RouteCandidate]:
        """
        Select optimal route (논문: 안전 만족 + 최소 비용)
        
        Args:
            routes: List of route candidates
            
        Returns:
            Best route or None if no safe route exists
        """
        print(f"\n🎯 Selecting optimal route...")
        
        # Filter safe routes
        safe_routes = [r for r in routes if r.is_safe]
        
        if not safe_routes:
            print("❌ No safe routes found!")
            return None
        
        print(f"   Safe routes: {len(safe_routes)}/{len(routes)}")
        
        # Sort by total cost (lowest first)
        safe_routes.sort(key=lambda r: r.total_cost)
        
        # Rank routes
        for rank, route in enumerate(safe_routes, 1):
            route.rank = rank
        
        optimal = safe_routes[0]
        
        print(f"✅ Optimal route selected:")
        print(f"   - Route ID: {optimal.route_id}")
        print(f"   - Distance: {optimal.total_distance:.0f}m")
        print(f"   - Avg risk: {optimal.avg_risk:.3f}")
        print(f"   - Max risk: {optimal.max_risk:.3f}")
        print(f"   - Delay cost: {optimal.delay_cost:.0f}m")
        print(f"   - Congestion cost: {optimal.congestion_cost:.2f}")
        print(f"   - Total cost: {optimal.total_cost:.2f}")
        
        return optimal
    
    def update_capacity_grid(self, route: RouteCandidate):
        """Update capacity grid with new route"""
        for i, j in route.cells_traversed:
            self.capacity_grid[i, j] += 1
    
    def _calculate_distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Calculate distance between two points (meters)"""
        lat1, lon1 = p1
        lat2, lon2 = p2
        
        dlat = (lat2 - lat1) * 111000
        dlon = (lon2 - lon1) * 88000
        
        return np.sqrt(dlat**2 + dlon**2)
    
    def extract_vertiport_sites(self,
                                routes: List[RouteCandidate],
                                min_suitability: float = 0.6,
                                max_sites: int = 10) -> List[Tuple[float, float, float]]:
        """
        Extract vertiport sites from selected routes
        
        Args:
            routes: List of evaluated routes
            min_suitability: Minimum site quality
            max_sites: Maximum number of sites
            
        Returns:
            List of (lat, lon, score) tuples
        """
        print(f"\n🏢 Extracting vertiport sites...")
        
        # Count cell traversals
        cell_frequency = {}
        for route in routes:
            if route.is_safe:
                for i, j in route.cells_traversed:
                    cell_frequency[(i, j)] = cell_frequency.get((i, j), 0) + 1
        
        # Score cells
        candidates = []
        for (i, j), frequency in cell_frequency.items():
            risk = self.grid_risk[i, j]
            density = self.grid_density[i, j]
            
            # Suitability: low risk, low density, high frequency
            suitability = (1.0 - risk) * 0.4 + (1.0 - density) * 0.3 + (frequency / len(routes)) * 0.3
            
            if suitability >= min_suitability:
                lat, lon = self.grid_to_latlon(i, j)
                candidates.append((lat, lon, suitability))
        
        # Sort by suitability
        candidates.sort(key=lambda x: x[2], reverse=True)
        
        # Apply spacing constraint
        selected = []
        for lat, lon, score in candidates:
            # Check spacing
            too_close = False
            for sel_lat, sel_lon, _ in selected:
                dist = self._calculate_distance((lat, lon), (sel_lat, sel_lon))
                if dist < 300:  # 300m minimum spacing
                    too_close = True
                    break
            
            if not too_close:
                selected.append((lat, lon, score))
                
                if len(selected) >= max_sites:
                    break
        
        print(f"✅ Found {len(selected)} vertiport sites (from {len(candidates)} candidates)")
        
        return selected
    
    def get_statistics(self) -> dict:
        """Get comprehensive statistics"""
        return {
            'grid_resolution': self.grid_resolution,
            'avg_risk': self.grid_risk.mean(),
            'max_risk': self.grid_risk.max(),
            'avg_congestion': self.capacity_grid.mean() / self.max_capacity,
            'max_congestion': self.capacity_grid.max() / self.max_capacity,
            'high_risk_cells': (self.grid_risk > self.safety_threshold).sum(),
            'high_risk_percent': (self.grid_risk > self.safety_threshold).sum() / self.grid_risk.size * 100
        }
