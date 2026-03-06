"""
Safe Landing Zone (SLZ) Extraction - IEEE MAES Paper
논문 기준 버티포트 추출

논문 방식:
1. Risk map에서 낮은 위험도 영역 식별
2. 경로 주변 500m 이내 SLZ 탐색
3. 가장 가까운 unoccupied 옵션 선택
4. 최소 간격 유지 (300m-500m)

평탄도, 면적 등은 논문에 없음 - 오직 Risk만 사용
"""

import numpy as np
from typing import List, Tuple, Dict
from dataclasses import dataclass
from scipy.spatial.distance import cdist


@dataclass
class SafeLandingZone:
    """Safe Landing Zone (SLZ) - 논문 기준"""
    lat: float
    lon: float
    risk_score: float  # 0-1, lower is better
    distance_to_path: float  # meters
    ground_elevation: float  # meters
    clearance_above_obstacles: float  # meters
    nearby_waypoints: int


class SLZExtractor:
    """
    Safe Landing Zone Extractor
    논문 방식: 오직 Risk Map만 사용하여 안전 착륙 지역 추출
    """
    
    def __init__(self,
                 dem: np.ndarray,
                 obstacle_map: np.ndarray,
                 risk_map: np.ndarray,
                 bounds: dict,
                 max_risk: float = 0.3,  # Maximum acceptable risk
                 min_spacing_m: float = 300.0):  # Minimum spacing between SLZs
        """
        Initialize SLZ extractor
        
        Args:
            dem: Digital elevation model
            obstacle_map: Building heights
            risk_map: Ground risk (0-1)
            bounds: {'lat_min', 'lat_max', 'lon_min', 'lon_max'}
            max_risk: Maximum acceptable risk threshold (논문에서 SORA 기준 1e-6 사용)
            min_spacing_m: Minimum spacing between SLZs (m)
        """
        self.dem = dem
        self.obstacle_map = obstacle_map
        self.risk_map = risk_map
        self.bounds = bounds
        
        self.max_risk = max_risk
        self.min_spacing_m = min_spacing_m
        
        # Grid parameters
        self.grid_height, self.grid_width = dem.shape
        self.lat_step = (bounds['lat_max'] - bounds['lat_min']) / self.grid_height
        self.lon_step = (bounds['lon_max'] - bounds['lon_min']) / self.grid_width
    
    def extract_from_path(self,
                         waypoints: List[Tuple[float, float, float]],
                         search_radius_m: float = 500.0,
                         max_slzs: int = 10) -> List[SafeLandingZone]:
        """
        Extract Safe Landing Zones along a flight path (논문 방식)
        
        Args:
            waypoints: List of (lat, lon, alt) waypoints
            search_radius_m: Search radius around each waypoint (m)
            max_slzs: Maximum number of SLZs to return
            
        Returns:
            List of SafeLandingZone objects, sorted by risk score (lowest first)
        """
        all_candidates = []
        
        # For each waypoint, search nearby area
        for i, (lat, lon, alt) in enumerate(waypoints):
            # Convert to grid
            grid_i, grid_j = self._latlon_to_grid(lat, lon)
            
            if grid_i is None or grid_j is None:
                continue
            
            # Search radius in grid cells (1 degree ≈ 111 km)
            search_radius_lat = search_radius_m / 111000.0
            search_radius_lon = search_radius_m / (111000.0 * np.cos(np.radians(lat)))
            
            search_cells_i = int(search_radius_lat / self.lat_step)
            search_cells_j = int(search_radius_lon / self.lon_step)
            
            # Search in a square region
            i_min = max(0, grid_i - search_cells_i)
            i_max = min(self.grid_height, grid_i + search_cells_i + 1)
            j_min = max(0, grid_j - search_cells_j)
            j_max = min(self.grid_width, grid_j + search_cells_j + 1)
            
            # Check each cell in search area
            for ci in range(i_min, i_max):
                for cj in range(j_min, j_max):
                    # Get risk at this location
                    risk = self.risk_map[ci, cj]
                    
                    # Check if risk is acceptable
                    if risk > self.max_risk:
                        continue
                    
                    # Get location data
                    cell_lat, cell_lon = self._grid_to_latlon(ci, cj)
                    elevation = self.dem[ci, cj]
                    clearance = self.obstacle_map[ci, cj]
                    
                    # Calculate distance to path
                    dist = self._haversine_distance(lat, lon, cell_lat, cell_lon)
                    
                    # Only include if within search radius
                    if dist > search_radius_m:
                        continue
                    
                    # Create candidate
                    slz = SafeLandingZone(
                        lat=cell_lat,
                        lon=cell_lon,
                        risk_score=risk,
                        distance_to_path=dist,
                        ground_elevation=elevation,
                        clearance_above_obstacles=clearance,
                        nearby_waypoints=1
                    )
                    
                    all_candidates.append(slz)
        
        if not all_candidates:
            return []
        
        # Sort by risk (lowest risk first)
        all_candidates.sort(key=lambda x: x.risk_score)
        
        # Apply spacing constraint
        selected_slzs = []
        for candidate in all_candidates:
            # Check distance to already selected SLZs
            too_close = False
            for selected in selected_slzs:
                dist = self._haversine_distance(
                    candidate.lat, candidate.lon,
                    selected.lat, selected.lon
                )
                if dist < self.min_spacing_m:
                    too_close = True
                    break
            
            if not too_close:
                selected_slzs.append(candidate)
                
                if len(selected_slzs) >= max_slzs:
                    break
        
        return selected_slzs
    
    def _latlon_to_grid(self, lat: float, lon: float) -> Tuple[int, int]:
        """Convert lat/lon to grid indices"""
        if not (self.bounds['lat_min'] <= lat <= self.bounds['lat_max']):
            return None, None
        if not (self.bounds['lon_min'] <= lon <= self.bounds['lon_max']):
            return None, None
        
        i = int((lat - self.bounds['lat_min']) / self.lat_step)
        j = int((lon - self.bounds['lon_min']) / self.lon_step)
        
        i = np.clip(i, 0, self.grid_height - 1)
        j = np.clip(j, 0, self.grid_width - 1)
        
        return i, j
    
    def _grid_to_latlon(self, i: int, j: int) -> Tuple[float, float]:
        """Convert grid indices to lat/lon"""
        lat = self.bounds['lat_min'] + (i + 0.5) * self.lat_step
        lon = self.bounds['lon_min'] + (j + 0.5) * self.lon_step
        return lat, lon
    
    def _haversine_distance(self, lat1: float, lon1: float, 
                           lat2: float, lon2: float) -> float:
        """Calculate distance between two points in meters"""
        R = 6371000  # Earth radius in meters
        
        dlat = np.radians(lat2 - lat1)
        dlon = np.radians(lon2 - lon1)
        
        a = (np.sin(dlat/2)**2 + 
             np.cos(np.radians(lat1)) * np.cos(np.radians(lat2)) * 
             np.sin(dlon/2)**2)
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
        
        return R * c
    
    def get_statistics(self, slzs: List[SafeLandingZone]) -> dict:
        """Get statistics of extracted SLZs"""
        if not slzs:
            return {
                'count': 0,
                'avg_risk': 0.0,
                'min_risk': 0.0,
                'max_risk': 0.0,
                'avg_distance_to_path': 0.0
            }
        
        risks = [slz.risk_score for slz in slzs]
        dists = [slz.distance_to_path for slz in slzs]
        
        return {
            'count': len(slzs),
            'avg_risk': np.mean(risks),
            'min_risk': np.min(risks),
            'max_risk': np.max(risks),
            'avg_distance_to_path': np.mean(dists),
            'min_distance': np.min(dists),
            'max_distance': np.max(dists)
        }
