"""
Vertiport Location Optimization
Finds optimal locations for vertiports based on accessibility, safety, and connectivity
"""

import numpy as np
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
from scipy.spatial import distance_matrix
from ..data_processing.dem_processor import DEMData


@dataclass
class VertiportCandidate:
    """Candidate vertiport location"""
    x: float
    y: float
    z: float
    score: float
    accessibility_score: float
    safety_score: float
    connectivity_score: float
    coverage_area: float
    
    def to_dict(self) -> dict:
        return {
            'x': self.x,
            'y': self.y,
            'z': self.z,
            'score': self.score,
            'accessibility_score': self.accessibility_score,
            'safety_score': self.safety_score,
            'connectivity_score': self.connectivity_score,
            'coverage_area': self.coverage_area
        }


class VertiportOptimizer:
    """Optimize vertiport locations for UAM network"""
    
    def __init__(self, dem_data: DEMData, risk_map: np.ndarray,
                 population_density: Optional[np.ndarray] = None):
        """
        Initialize vertiport optimizer
        
        Args:
            dem_data: Digital elevation model
            risk_map: Risk assessment map
            population_density: Population density map (optional)
        """
        self.dem_data = dem_data
        self.risk_map = risk_map
        self.population_density = population_density
        
        # Default weights for scoring
        self.weights = {
            'accessibility': 0.35,
            'safety': 0.35,
            'connectivity': 0.30
        }
    
    def find_optimal_locations(self, num_vertiports: int = 5,
                              min_distance: float = 200.0,
                              search_resolution: int = 10) -> List[VertiportCandidate]:
        """
        Find optimal vertiport locations
        
        Args:
            num_vertiports: Number of vertiports to place
            min_distance: Minimum distance between vertiports (meters)
            search_resolution: Grid search resolution (higher = faster but less precise)
            
        Returns:
            List of optimal vertiport candidates
        """
        height, width = self.dem_data.shape
        
        # Generate candidate grid (subsample for efficiency)
        y_indices = np.arange(0, height, search_resolution)
        x_indices = np.arange(0, width, search_resolution)
        
        candidates = []
        
        # Evaluate each candidate location
        for y_idx in y_indices:
            for x_idx in x_indices:
                # Convert grid indices to coordinates
                x = self.dem_data.x_coords[min(x_idx, width-1)]
                y = self.dem_data.y_coords[min(y_idx, height-1)]
                z = self.dem_data.elevation[min(y_idx, height-1), min(x_idx, width-1)]
                
                # Calculate scores
                accessibility = self._calculate_accessibility_score(x_idx, y_idx)
                safety = self._calculate_safety_score(x_idx, y_idx)
                
                # Initial connectivity score (will be updated later)
                connectivity = 0.0
                
                # Overall score
                score = (self.weights['accessibility'] * accessibility +
                        self.weights['safety'] * safety)
                
                if score > 0.3:  # Only keep promising candidates
                    candidates.append(VertiportCandidate(
                        x=x, y=y, z=z,
                        score=score,
                        accessibility_score=accessibility,
                        safety_score=safety,
                        connectivity_score=connectivity,
                        coverage_area=0.0
                    ))
        
        # Sort by initial score
        candidates.sort(key=lambda c: c.score, reverse=True)
        
        # Select vertiports with minimum distance constraint
        selected = self._select_with_distance_constraint(
            candidates, num_vertiports, min_distance
        )
        
        # Update connectivity scores
        selected = self._update_connectivity_scores(selected)
        
        # Calculate coverage areas
        selected = self._calculate_coverage_areas(selected)
        
        # Recalculate final scores
        for vp in selected:
            vp.score = (self.weights['accessibility'] * vp.accessibility_score +
                       self.weights['safety'] * vp.safety_score +
                       self.weights['connectivity'] * vp.connectivity_score)
        
        # Sort by final score
        selected.sort(key=lambda c: c.score, reverse=True)
        
        return selected
    
    def _calculate_accessibility_score(self, x_idx: int, y_idx: int) -> float:
        """
        Calculate accessibility score based on population density
        Higher score = more accessible to population
        """
        if self.population_density is None:
            # Use inverse of terrain roughness as proxy
            window_size = 5
            y_start = max(0, y_idx - window_size)
            y_end = min(self.dem_data.shape[0], y_idx + window_size)
            x_start = max(0, x_idx - window_size)
            x_end = min(self.dem_data.shape[1], x_idx + window_size)
            
            local_terrain = self.dem_data.elevation[y_start:y_end, x_start:x_end]
            roughness = np.std(local_terrain)
            
            # Lower roughness = better accessibility
            score = 1.0 / (1.0 + roughness / 20.0)
        else:
            # Use population density in surrounding area
            window_size = 10
            y_start = max(0, y_idx - window_size)
            y_end = min(self.population_density.shape[0], y_idx + window_size)
            x_start = max(0, x_idx - window_size)
            x_end = min(self.population_density.shape[1], x_idx + window_size)
            
            local_density = self.population_density[y_start:y_end, x_start:x_end]
            avg_density = np.mean(local_density)
            
            # Normalize to 0-1
            max_density = np.max(self.population_density)
            score = avg_density / (max_density + 1e-10)
        
        return float(np.clip(score, 0, 1))
    
    def _calculate_safety_score(self, x_idx: int, y_idx: int) -> float:
        """
        Calculate safety score based on risk map
        Higher score = safer location
        """
        # Check surrounding area for safety
        window_size = 8
        y_start = max(0, y_idx - window_size)
        y_end = min(self.risk_map.shape[0], y_idx + window_size)
        x_start = max(0, x_idx - window_size)
        x_end = min(self.risk_map.shape[1], x_idx + window_size)
        
        local_risk = self.risk_map[y_start:y_end, x_start:x_end]
        avg_risk = np.mean(local_risk)
        
        # Lower risk = higher safety score
        safety_score = 1.0 - avg_risk
        
        # Check for flat terrain (important for vertiports)
        local_terrain = self.dem_data.elevation[y_start:y_end, x_start:x_end]
        terrain_variance = np.var(local_terrain)
        flatness_score = 1.0 / (1.0 + terrain_variance / 100.0)
        
        # Combine safety and flatness
        score = 0.7 * safety_score + 0.3 * flatness_score
        
        return float(np.clip(score, 0, 1))
    
    def _select_with_distance_constraint(self, candidates: List[VertiportCandidate],
                                        num_vertiports: int,
                                        min_distance: float) -> List[VertiportCandidate]:
        """
        Select vertiports ensuring minimum distance between them
        """
        selected = []
        
        for candidate in candidates:
            if len(selected) >= num_vertiports:
                break
            
            # Check distance to already selected vertiports
            too_close = False
            for selected_vp in selected:
                dist = np.sqrt((candidate.x - selected_vp.x)**2 + 
                             (candidate.y - selected_vp.y)**2)
                if dist < min_distance:
                    too_close = True
                    break
            
            if not too_close:
                selected.append(candidate)
        
        return selected
    
    def _update_connectivity_scores(self, 
                                   vertiports: List[VertiportCandidate]) -> List[VertiportCandidate]:
        """
        Update connectivity scores based on network topology
        """
        if len(vertiports) <= 1:
            return vertiports
        
        # Create distance matrix
        positions = np.array([[vp.x, vp.y] for vp in vertiports])
        dist_matrix = distance_matrix(positions, positions)
        
        # Calculate connectivity score for each vertiport
        for i, vp in enumerate(vertiports):
            # Average distance to other vertiports
            distances = dist_matrix[i]
            distances = distances[distances > 0]  # Exclude self
            
            if len(distances) > 0:
                avg_distance = np.mean(distances)
                # Optimal distance is around 500-1000m
                optimal_distance = 750.0
                connectivity = 1.0 - abs(avg_distance - optimal_distance) / optimal_distance
                connectivity = np.clip(connectivity, 0, 1)
            else:
                connectivity = 0.0
            
            vp.connectivity_score = float(connectivity)
        
        return vertiports
    
    def _calculate_coverage_areas(self, 
                                 vertiports: List[VertiportCandidate]) -> List[VertiportCandidate]:
        """
        Calculate coverage area for each vertiport using Voronoi-like approach
        """
        if not vertiports:
            return vertiports
        
        height, width = self.dem_data.shape
        
        # Create grid of points
        y_coords = np.arange(0, height, 5)
        x_coords = np.arange(0, width, 5)
        grid_y, grid_x = np.meshgrid(y_coords, x_coords, indexing='ij')
        
        # Convert to real coordinates
        grid_points = np.stack([
            np.interp(grid_x, np.arange(width), self.dem_data.x_coords),
            np.interp(grid_y, np.arange(height), self.dem_data.y_coords)
        ], axis=-1)
        
        # Vertiport positions
        vp_positions = np.array([[vp.x, vp.y] for vp in vertiports])
        
        # Assign each grid point to nearest vertiport
        coverage_counts = [0] * len(vertiports)
        
        for i in range(grid_points.shape[0]):
            for j in range(grid_points.shape[1]):
                point = grid_points[i, j]
                distances = np.sqrt(np.sum((vp_positions - point)**2, axis=1))
                nearest_vp_idx = np.argmin(distances)
                coverage_counts[nearest_vp_idx] += 1
        
        # Calculate coverage area (grid cells * resolution^2)
        cell_area = (self.dem_data.resolution * 5) ** 2  # 5 is subsampling factor
        
        for i, vp in enumerate(vertiports):
            vp.coverage_area = coverage_counts[i] * cell_area
        
        return vertiports
    
    def visualize_vertiport_network(self, vertiports: List[VertiportCandidate]) -> Dict:
        """
        Create data structure for network visualization
        
        Returns:
            Dictionary with nodes and edges for visualization
        """
        nodes = []
        for i, vp in enumerate(vertiports):
            nodes.append({
                'id': i,
                'x': vp.x,
                'y': vp.y,
                'z': vp.z,
                'score': vp.score,
                'label': f'VP-{i+1}'
            })
        
        # Create edges (connect all vertiports for visualization)
        edges = []
        for i in range(len(vertiports)):
            for j in range(i + 1, len(vertiports)):
                dist = np.sqrt((vertiports[i].x - vertiports[j].x)**2 + 
                             (vertiports[i].y - vertiports[j].y)**2)
                edges.append({
                    'source': i,
                    'target': j,
                    'distance': dist
                })
        
        return {'nodes': nodes, 'edges': edges}
