"""
Grid-based Vertiport Site Evaluation System
논문 방식: 전체 공역을 격자로 나누고 각 셀의 적합도를 종합 평가

논문 방식:
1. 전체 구역을 uniform grid로 분할
2. 각 grid cell의 안전도 평가:
   - 건물 밀도 (obstacle map)
   - 인구 밀도 (risk map)
   - 지형 평탄도 (DEM variation)
   - 접근성 (도로/교통)
3. 모든 가능한 경로 조합 탐색
4. 격자별 안전도 누적 평가
5. 최적 버티포트 위치 선정
"""

import numpy as np
from typing import List, Tuple, Dict, Set
from dataclasses import dataclass
from scipy.ndimage import uniform_filter
import heapq


@dataclass
class GridCell:
    """Single grid cell with comprehensive evaluation"""
    i: int  # Grid row
    j: int  # Grid column
    lat: float
    lon: float
    
    # Safety metrics
    risk_score: float  # 0-1, lower is better
    building_density: float  # 0-1
    terrain_flatness: float  # 0-1, higher is better
    accessibility: float  # 0-1, higher is better
    
    # Composite score
    suitability_score: float = 0.0  # Overall suitability
    
    def calculate_suitability(self, 
                            w_risk: float = 0.4,
                            w_density: float = 0.2,
                            w_flatness: float = 0.2,
                            w_access: float = 0.2):
        """Calculate composite suitability score"""
        self.suitability_score = (
            w_risk * (1.0 - self.risk_score) +  # Lower risk is better
            w_density * (1.0 - self.building_density) +  # Lower density is better
            w_flatness * self.terrain_flatness +  # Higher flatness is better
            w_access * self.accessibility  # Higher accessibility is better
        )
        return self.suitability_score


@dataclass
class VertiportSite:
    """Potential vertiport site"""
    cell: GridCell
    rank: int
    nearby_safe_cells: int  # Number of safe landing alternatives
    route_coverage: int  # Number of routes passing through


class GridBasedEvaluator:
    """
    Grid-based Comprehensive Vertiport Evaluation System
    논문 방식: 격자 기반 전체 평가
    """
    
    def __init__(self,
                 dem: np.ndarray,
                 obstacle_map: np.ndarray,
                 risk_map: np.ndarray,
                 bounds: dict,
                 grid_resolution: int = 64):  # 64x64 grid for evaluation
        """
        Initialize grid-based evaluator
        
        Args:
            dem: Digital elevation model
            obstacle_map: Building heights
            risk_map: Ground risk (0-1)
            bounds: Geographic bounds
            grid_resolution: Number of grid cells per dimension
        """
        self.dem = dem
        self.obstacle_map = obstacle_map
        self.risk_map = risk_map
        self.bounds = bounds
        self.grid_resolution = grid_resolution
        
        # Create evaluation grid
        self.grid_cells: Dict[Tuple[int, int], GridCell] = {}
        self._initialize_grid()
        
        # Route analysis
        self.routes_evaluated = 0
        self.cell_route_count: Dict[Tuple[int, int], int] = {}
    
    def _initialize_grid(self):
        """Initialize uniform evaluation grid"""
        lat_min, lat_max = self.bounds['lat_min'], self.bounds['lat_max']
        lon_min, lon_max = self.bounds['lon_min'], self.bounds['lon_max']
        
        lat_step = (lat_max - lat_min) / self.grid_resolution
        lon_step = (lon_max - lon_min) / self.grid_resolution
        
        # Original data dimensions
        orig_h, orig_w = self.dem.shape
        
        print(f"\n🔧 Initializing {self.grid_resolution}×{self.grid_resolution} evaluation grid...")
        
        for i in range(self.grid_resolution):
            for j in range(self.grid_resolution):
                # Grid cell center
                lat = lat_min + (i + 0.5) * lat_step
                lon = lon_min + (j + 0.5) * lon_step
                
                # Map to original data indices
                orig_i = int((lat - lat_min) / (lat_max - lat_min) * orig_h)
                orig_j = int((lon - lon_min) / (lon_max - lon_min) * orig_w)
                
                orig_i = np.clip(orig_i, 0, orig_h - 1)
                orig_j = np.clip(orig_j, 0, orig_w - 1)
                
                # Extract metrics from maps
                risk = self.risk_map[orig_i, orig_j]
                building_height = self.obstacle_map[orig_i, orig_j]
                
                # Calculate building density (local average)
                window_size = max(1, orig_h // self.grid_resolution)
                i_min = max(0, orig_i - window_size)
                i_max = min(orig_h, orig_i + window_size + 1)
                j_min = max(0, orig_j - window_size)
                j_max = min(orig_w, orig_j + window_size + 1)
                
                local_buildings = self.obstacle_map[i_min:i_max, j_min:j_max]
                building_density = (local_buildings > 0).sum() / local_buildings.size
                
                # Calculate terrain flatness (std of elevations)
                local_dem = self.dem[i_min:i_max, j_min:j_max]
                terrain_std = np.std(local_dem)
                terrain_flatness = np.exp(-terrain_std / 10.0)  # Lower std = flatter
                
                # Accessibility (simplified: inverse of risk + density)
                accessibility = 1.0 - (risk * 0.5 + building_density * 0.5)
                
                # Create cell
                cell = GridCell(
                    i=i, j=j,
                    lat=lat, lon=lon,
                    risk_score=risk,
                    building_density=building_density,
                    terrain_flatness=terrain_flatness,
                    accessibility=accessibility
                )
                
                # Calculate composite suitability
                cell.calculate_suitability()
                
                self.grid_cells[(i, j)] = cell
                self.cell_route_count[(i, j)] = 0
        
        print(f"✅ Grid initialized: {len(self.grid_cells)} cells")
    
    def evaluate_route(self, waypoints: List[Tuple[float, float, float]]) -> Tuple[float, List[Tuple[int, int]]]:
        """
        Evaluate a route through the grid
        
        Args:
            waypoints: Route waypoints (lat, lon, alt)
            
        Returns:
            (route_score, cells_traversed)
        """
        cells_traversed = []
        route_score = 0.0
        
        for lat, lon, alt in waypoints:
            # Find grid cell
            i = int((lat - self.bounds['lat_min']) / 
                   (self.bounds['lat_max'] - self.bounds['lat_min']) * self.grid_resolution)
            j = int((lon - self.bounds['lon_min']) / 
                   (self.bounds['lon_max'] - self.bounds['lon_min']) * self.grid_resolution)
            
            i = np.clip(i, 0, self.grid_resolution - 1)
            j = np.clip(j, 0, self.grid_resolution - 1)
            
            cell_key = (i, j)
            
            if cell_key not in cells_traversed:
                cells_traversed.append(cell_key)
                
                # Add cell's suitability to route score
                if cell_key in self.grid_cells:
                    route_score += self.grid_cells[cell_key].suitability_score
                    self.cell_route_count[cell_key] += 1
        
        # Average score
        route_score = route_score / len(cells_traversed) if cells_traversed else 0.0
        self.routes_evaluated += 1
        
        return route_score, cells_traversed
    
    def generate_multiple_routes(self,
                                 start: Tuple[float, float],
                                 end: Tuple[float, float],
                                 num_routes: int = 10) -> List[Tuple[float, List[Tuple[float, float, float]]]]:
        """
        Generate multiple alternative routes (논문 방식)
        
        Args:
            start: (lat, lon)
            end: (lat, lon)
            num_routes: Number of routes to generate
            
        Returns:
            List of (score, waypoints) tuples
        """
        routes = []
        
        print(f"\n🛣️  Generating {num_routes} alternative routes...")
        
        # Base altitude
        cruise_alt = 100.0
        
        for route_id in range(num_routes):
            # Generate route with variation
            if route_id == 0:
                # Direct route
                waypoints = [
                    (start[0], start[1], cruise_alt),
                    (end[0], end[1], cruise_alt)
                ]
            else:
                # Alternative routes with lateral offset
                offset_factor = (route_id % 5) - 2  # -2, -1, 0, 1, 2
                offset_lat = offset_factor * 0.002  # ~200m offset
                offset_lon = offset_factor * 0.002
                
                # Mid-point with offset
                mid_lat = (start[0] + end[0]) / 2 + offset_lat
                mid_lon = (start[1] + end[1]) / 2 + offset_lon
                
                waypoints = [
                    (start[0], start[1], cruise_alt),
                    (mid_lat, mid_lon, cruise_alt),
                    (end[0], end[1], cruise_alt)
                ]
            
            # Evaluate route
            score, cells = self.evaluate_route(waypoints)
            routes.append((score, waypoints))
            
            print(f"   Route {route_id+1}: score={score:.3f}, cells={len(cells)}")
        
        # Sort by score (highest first)
        routes.sort(key=lambda x: x[0], reverse=True)
        
        return routes
    
    def select_vertiport_sites(self, 
                               min_suitability: float = 0.6,
                               min_spacing_cells: int = 3,
                               max_sites: int = 10) -> List[VertiportSite]:
        """
        Select optimal vertiport sites from grid (논문 방식)
        
        Args:
            min_suitability: Minimum suitability threshold
            min_spacing_cells: Minimum spacing in grid cells
            max_sites: Maximum number of sites
            
        Returns:
            List of VertiportSite objects
        """
        print(f"\n🎯 Selecting vertiport sites from grid...")
        print(f"   Criteria: suitability≥{min_suitability}, spacing≥{min_spacing_cells} cells")
        
        # Sort cells by suitability
        candidates = []
        for (i, j), cell in self.grid_cells.items():
            if cell.suitability_score >= min_suitability:
                candidates.append(((i, j), cell))
        
        candidates.sort(key=lambda x: x[1].suitability_score, reverse=True)
        
        print(f"   Candidates: {len(candidates)} cells above threshold")
        
        # Select with spacing constraint
        selected_sites = []
        
        for (i, j), cell in candidates:
            # Check spacing with already selected sites
            too_close = False
            for site in selected_sites:
                di = abs(site.cell.i - i)
                dj = abs(site.cell.j - j)
                dist = np.sqrt(di**2 + dj**2)
                
                if dist < min_spacing_cells:
                    too_close = True
                    break
            
            if not too_close:
                # Count nearby safe cells
                nearby_safe = 0
                for di in range(-1, 2):
                    for dj in range(-1, 2):
                        ni, nj = i + di, j + dj
                        if (ni, nj) in self.grid_cells:
                            neighbor = self.grid_cells[(ni, nj)]
                            if neighbor.suitability_score >= min_suitability:
                                nearby_safe += 1
                
                # Get route coverage
                route_coverage = self.cell_route_count.get((i, j), 0)
                
                site = VertiportSite(
                    cell=cell,
                    rank=len(selected_sites) + 1,
                    nearby_safe_cells=nearby_safe,
                    route_coverage=route_coverage
                )
                
                selected_sites.append(site)
                
                if len(selected_sites) >= max_sites:
                    break
        
        print(f"✅ Selected {len(selected_sites)} sites")
        
        return selected_sites
    
    def get_grid_statistics(self) -> dict:
        """Get comprehensive grid statistics"""
        suitabilities = [cell.suitability_score for cell in self.grid_cells.values()]
        risks = [cell.risk_score for cell in self.grid_cells.values()]
        densities = [cell.building_density for cell in self.grid_cells.values()]
        flatnesses = [cell.terrain_flatness for cell in self.grid_cells.values()]
        
        high_suit_count = sum(1 for s in suitabilities if s >= 0.6)
        
        return {
            'total_cells': len(self.grid_cells),
            'avg_suitability': np.mean(suitabilities),
            'max_suitability': np.max(suitabilities),
            'min_suitability': np.min(suitabilities),
            'high_suitability_cells': high_suit_count,
            'high_suitability_percent': high_suit_count / len(self.grid_cells) * 100,
            'avg_risk': np.mean(risks),
            'avg_building_density': np.mean(densities),
            'avg_terrain_flatness': np.mean(flatnesses),
            'routes_evaluated': self.routes_evaluated
        }
    
    def get_grid_heatmap(self) -> np.ndarray:
        """Get 2D suitability heatmap"""
        heatmap = np.zeros((self.grid_resolution, self.grid_resolution))
        
        for (i, j), cell in self.grid_cells.items():
            heatmap[i, j] = cell.suitability_score
        
        return heatmap
