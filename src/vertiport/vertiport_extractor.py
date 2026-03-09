"""
Route-based Vertiport Extraction
경로 기반 버티포트 후보 추출

논문 기준:
- Safety score: S_safety = 1 - R_avg - 0.2×R_max
- Flatness score: S_flatness = exp(-σ²/θ²), θ=5m
- Total score: S_total = 0.4×S_safety + 0.3×S_flatness + 0.3×coverage
- Minimum spacing: 500m
- Minimum area: 900m² (30m × 30m)
"""

import numpy as np
from typing import List, Tuple, Dict
from dataclasses import dataclass
from scipy.spatial.distance import cdist


@dataclass
class VertiportCandidate:
    """Vertiport candidate site"""
    lat: float
    lon: float
    safety_score: float
    flatness_score: float
    coverage_score: float
    total_score: float
    ground_elevation: float
    area_flat: float  # m²
    clearance: float  # m
    nearby_waypoints: int


class VertiportExtractor:
    """
    Extract vertiport candidates along flight paths
    경로를 따라 안전한 버티포트 후보 추출
    """
    
    def __init__(self,
                 dem: np.ndarray,
                 obstacle_map: np.ndarray,
                 risk_map: np.ndarray,
                 bounds: dict,
                 min_safety: float = 0.7,
                 min_flatness: float = 0.5,
                 min_area: float = 900.0,  # m²
                 min_clearance: float = 50.0,  # m
                 min_spacing: float = 500.0):  # m
        """
        Initialize vertiport extractor
        
        Args:
            dem: Digital elevation model
            obstacle_map: Building heights
            risk_map: Ground risk (0-1)
            bounds: {'lat_min', 'lat_max', 'lon_min', 'lon_max'}
            min_safety: Minimum safety score threshold
            min_flatness: Minimum flatness score threshold
            min_area: Minimum flat area (m²)
            min_clearance: Minimum clearance above obstacles (m)
            min_spacing: Minimum spacing between vertiports (m)
        """
        self.dem = dem
        self.obstacle_map = obstacle_map
        self.risk_map = risk_map
        self.bounds = bounds
        
        self.min_safety = min_safety
        self.min_flatness = min_flatness
        self.min_area = min_area
        self.min_clearance = min_clearance
        self.min_spacing = min_spacing
        
        # Grid parameters
        self.grid_height, self.grid_width = dem.shape
        self.lat_step = (bounds['lat_max'] - bounds['lat_min']) / self.grid_height
        self.lon_step = (bounds['lon_max'] - bounds['lon_min']) / self.grid_width
    
    def extract_from_path(self,
                         waypoints: List[Tuple[float, float, float]],
                         search_radius_m: float = 500.0,
                         max_candidates: int = 10) -> List[VertiportCandidate]:
        """
        Extract vertiport candidates along a flight path
        
        Args:
            waypoints: List of (lat, lon, alt) waypoints
            search_radius_m: Search radius around each waypoint (m)
            max_candidates: Maximum number of candidates to return
            
        Returns:
            List of VertiportCandidate objects, sorted by total score
        """
        candidates = []
        
        # For each waypoint, search nearby area
        for i, (lat, lon, alt) in enumerate(waypoints):
            # Convert to grid
            grid_i, grid_j = self._latlon_to_grid(lat, lon)
            
            # Search radius in grid cells
            # Approximate: 1 degree ≈ 111km, so radius in cells
            radius_lat = int(search_radius_m / 111000 / self.lat_step)
            radius_lon = int(search_radius_m / 88000 / self.lon_step)
            
            # Search area
            i_min = max(0, grid_i - radius_lat)
            i_max = min(self.grid_height, grid_i + radius_lat + 1)
            j_min = max(0, grid_j - radius_lon)
            j_max = min(self.grid_width, grid_j + radius_lon + 1)
            
            # Evaluate each cell in search area
            for ci in range(i_min, i_max):
                for cj in range(j_min, j_max):
                    # Evaluate site
                    site = self._evaluate_site(ci, cj, waypoints)
                    
                    if site is not None:
                        candidates.append(site)
        
        # Remove duplicates (close candidates)
        candidates = self._cluster_candidates(candidates)
        
        # Sort by total score
        candidates.sort(key=lambda x: x.total_score, reverse=True)
        
        # Apply minimum spacing constraint
        final_candidates = self._apply_spacing_constraint(candidates)
        
        return final_candidates[:max_candidates]
    
    def _evaluate_site(self,
                      i: int, j: int,
                      waypoints: List[Tuple[float, float, float]]) -> VertiportCandidate:
        """
        Evaluate a potential vertiport site
        
        Returns:
            VertiportCandidate or None if site doesn't meet criteria
        """
        # 1. Safety score
        safety_score = self._calculate_safety_score(i, j)
        
        if safety_score < self.min_safety:
            return None
        
        # 2. Flatness score
        flatness_score, std_dev = self._calculate_flatness_score(i, j)
        
        if flatness_score < self.min_flatness:
            return None
        
        # 3. Clearance check
        clearance = self._calculate_clearance(i, j)
        
        if clearance < self.min_clearance:
            return None
        
        # 4. Flat area
        flat_area = self._calculate_flat_area(i, j)
        
        if flat_area < self.min_area:
            return None
        
        # 5. Coverage score (how many waypoints nearby)
        coverage_score, nearby_count = self._calculate_coverage_score(i, j, waypoints)
        
        # 6. Total score
        total_score = (
            0.4 * safety_score +
            0.3 * flatness_score +
            0.3 * coverage_score
        )
        
        # Convert to lat/lon
        lat, lon = self._grid_to_latlon(i, j)
        
        return VertiportCandidate(
            lat=lat,
            lon=lon,
            safety_score=safety_score,
            flatness_score=flatness_score,
            coverage_score=coverage_score,
            total_score=total_score,
            ground_elevation=float(self.dem[i, j]),
            area_flat=flat_area,
            clearance=clearance,
            nearby_waypoints=nearby_count
        )
    
    def _calculate_safety_score(self, i: int, j: int, radius: int = 3) -> float:
        """
        S_safety = 1 - R_avg - α×R_max
        α = 0.2
        """
        # Get local risk in radius
        i_min = max(0, i - radius)
        i_max = min(self.grid_height, i + radius + 1)
        j_min = max(0, j - radius)
        j_max = min(self.grid_width, j + radius + 1)
        
        local_risk = self.risk_map[i_min:i_max, j_min:j_max]
        
        R_avg = np.mean(local_risk)
        R_max = np.max(local_risk)
        
        alpha = 0.2
        safety = 1.0 - R_avg - alpha * R_max
        
        return max(0, min(1, safety))
    
    def _calculate_flatness_score(self, i: int, j: int, radius: int = 3) -> Tuple[float, float]:
        """
        S_flatness = exp(-σ²/θ²)
        θ = 5m (flatness threshold)
        
        Returns:
            (flatness_score, std_dev)
        """
        # Get local elevation
        i_min = max(0, i - radius)
        i_max = min(self.grid_height, i + radius + 1)
        j_min = max(0, j - radius)
        j_max = min(self.grid_width, j + radius + 1)
        
        local_elevation = self.dem[i_min:i_max, j_min:j_max]
        
        sigma = np.std(local_elevation)
        theta = 5.0  # meters
        
        flatness = np.exp(-(sigma**2) / (theta**2))
        
        return flatness, sigma
    
    def _calculate_clearance(self, i: int, j: int) -> float:
        """
        Clearance = max(ground, obstacles) difference
        """
        ground = self.dem[i, j]
        obstacle = self.obstacle_map[i, j]
        
        clearance = max(ground, obstacle) - ground
        
        return float(clearance)
    
    def _calculate_flat_area(self, i: int, j: int, radius: int = 5, threshold: float = 2.0) -> float:
        """
        Calculate flat area around site
        
        Flat = cells where |elevation - center_elevation| < threshold
        
        Returns:
            Flat area in m²
        """
        center_elevation = self.dem[i, j]
        
        i_min = max(0, i - radius)
        i_max = min(self.grid_height, i + radius + 1)
        j_min = max(0, j - radius)
        j_max = min(self.grid_width, j + radius + 1)
        
        local_elevation = self.dem[i_min:i_max, j_min:j_max]
        
        # Count flat cells
        flat_mask = np.abs(local_elevation - center_elevation) < threshold
        flat_cells = np.sum(flat_mask)
        
        # Convert to m²
        # Approximate cell area
        cell_area_lat = self.lat_step * 111000  # meters
        cell_area_lon = self.lon_step * 88000   # meters
        cell_area = cell_area_lat * cell_area_lon  # m²
        
        flat_area = flat_cells * cell_area
        
        return flat_area
    
    def _calculate_coverage_score(self,
                                  i: int, j: int,
                                  waypoints: List[Tuple[float, float, float]],
                                  radius_m: float = 1000.0) -> Tuple[float, int]:
        """
        Coverage = how many waypoints are within radius
        
        Score = ln(1 + n_routes) / ln(11)
        (논문에서는 여러 경로를 고려하지만, 여기서는 하나의 경로의 waypoint 개수)
        
        Returns:
            (coverage_score, nearby_count)
        """
        site_lat, site_lon = self._grid_to_latlon(i, j)
        
        nearby_count = 0
        
        for wp_lat, wp_lon, wp_alt in waypoints:
            # Haversine distance (approximate)
            dlat = (wp_lat - site_lat) * 111000
            dlon = (wp_lon - site_lon) * 88000
            dist = np.sqrt(dlat**2 + dlon**2)
            
            if dist < radius_m:
                nearby_count += 1
        
        # Coverage score
        coverage = np.log(1 + nearby_count) / np.log(11)
        
        return coverage, nearby_count
    
    def _cluster_candidates(self,
                           candidates: List[VertiportCandidate],
                           merge_distance_m: float = 100.0) -> List[VertiportCandidate]:
        """
        Cluster nearby candidates and keep the best one
        """
        if len(candidates) == 0:
            return []
        
        # Convert to array
        positions = np.array([[c.lat, c.lon] for c in candidates])
        
        # Compute pairwise distances (approximate)
        # lat: 1 degree ≈ 111km
        # lon: 1 degree ≈ 88km (at Seoul latitude)
        distances = cdist(positions, positions, metric='euclidean')
        distances *= 111000  # Convert to meters (rough approximation)
        
        # Cluster using simple greedy approach
        clustered = []
        used = set()
        
        # Sort by total score
        sorted_candidates = sorted(candidates, key=lambda x: x.total_score, reverse=True)
        
        for idx, candidate in enumerate(sorted_candidates):
            if idx in used:
                continue
            
            # This is the best in its cluster
            clustered.append(candidate)
            used.add(idx)
            
            # Mark nearby candidates as used
            for j, other in enumerate(sorted_candidates):
                if j in used:
                    continue
                
                # Distance check
                dlat = (candidate.lat - other.lat) * 111000
                dlon = (candidate.lon - other.lon) * 88000
                dist = np.sqrt(dlat**2 + dlon**2)
                
                if dist < merge_distance_m:
                    used.add(j)
        
        return clustered
    
    def _apply_spacing_constraint(self,
                                  candidates: List[VertiportCandidate]) -> List[VertiportCandidate]:
        """
        Apply minimum spacing constraint
        Keep highest scoring candidates that are at least min_spacing apart
        """
        if len(candidates) == 0:
            return []
        
        selected = [candidates[0]]  # Start with best candidate
        
        for candidate in candidates[1:]:
            # Check distance to all selected
            min_dist = float('inf')
            
            for selected_cand in selected:
                dlat = (candidate.lat - selected_cand.lat) * 111000
                dlon = (candidate.lon - selected_cand.lon) * 88000
                dist = np.sqrt(dlat**2 + dlon**2)
                
                min_dist = min(min_dist, dist)
            
            # If far enough, add it
            if min_dist >= self.min_spacing:
                selected.append(candidate)
        
        return selected
    
    def _latlon_to_grid(self, lat: float, lon: float) -> Tuple[int, int]:
        """Convert lat/lon to grid indices"""
        i = int((lat - self.bounds['lat_min']) / self.lat_step)
        j = int((lon - self.bounds['lon_min']) / self.lon_step)
        
        i = max(0, min(i, self.grid_height - 1))
        j = max(0, min(j, self.grid_width - 1))
        
        return i, j
    
    def _grid_to_latlon(self, i: int, j: int) -> Tuple[float, float]:
        """Convert grid indices to lat/lon"""
        lat = self.bounds['lat_min'] + i * self.lat_step
        lon = self.bounds['lon_min'] + j * self.lon_step
        return lat, lon
