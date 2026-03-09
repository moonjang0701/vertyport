"""
DEM (Digital Elevation Model) Data Processor
Handles loading, processing, and analyzing terrain elevation data
"""

import numpy as np
from typing import Tuple, Optional, Dict
from dataclasses import dataclass
import cv2


@dataclass
class DEMData:
    """Data class for DEM information"""
    elevation: np.ndarray
    x_coords: np.ndarray
    y_coords: np.ndarray
    resolution: float
    bounds: Tuple[float, float, float, float]  # (min_x, min_y, max_x, max_y)
    
    @property
    def shape(self) -> Tuple[int, int]:
        return self.elevation.shape
    
    @property
    def min_elevation(self) -> float:
        return np.nanmin(self.elevation)
    
    @property
    def max_elevation(self) -> float:
        return np.nanmax(self.elevation)


class DEMProcessor:
    """Process and analyze DEM data for UAM path planning"""
    
    def __init__(self, resolution: float = 10.0):
        """
        Initialize DEM processor
        
        Args:
            resolution: Grid resolution in meters
        """
        self.resolution = resolution
        
    def load_from_array(self, elevation_data: np.ndarray, 
                       bounds: Optional[Tuple[float, float, float, float]] = None) -> DEMData:
        """
        Load DEM from numpy array
        
        Args:
            elevation_data: 2D array of elevation values
            bounds: Geographic bounds (min_x, min_y, max_x, max_y)
            
        Returns:
            DEMData object
        """
        height, width = elevation_data.shape
        
        if bounds is None:
            bounds = (0, 0, width * self.resolution, height * self.resolution)
        
        x_coords = np.linspace(bounds[0], bounds[2], width)
        y_coords = np.linspace(bounds[1], bounds[3], height)
        
        return DEMData(
            elevation=elevation_data,
            x_coords=x_coords,
            y_coords=y_coords,
            resolution=self.resolution,
            bounds=bounds
        )
    
    def create_synthetic_urban_dem(self, width: int = 100, height: int = 100,
                                   num_buildings: int = 20) -> DEMData:
        """
        Create synthetic urban DEM with buildings for testing
        
        Args:
            width: Grid width
            height: Grid height
            num_buildings: Number of buildings to generate
            
        Returns:
            DEMData object with synthetic urban terrain
        """
        # Base terrain with some variation
        elevation = np.random.randn(height, width) * 5 + 50
        
        # Add buildings as elevated rectangular areas
        for _ in range(num_buildings):
            building_height = np.random.uniform(20, 80)
            building_width = np.random.randint(3, 10)
            building_depth = np.random.randint(3, 10)
            
            x = np.random.randint(0, width - building_width)
            y = np.random.randint(0, height - building_depth)
            
            elevation[y:y+building_depth, x:x+building_width] += building_height
        
        # Smooth the elevation data
        elevation = cv2.GaussianBlur(elevation, (5, 5), 1.0)
        
        bounds = (0, 0, width * self.resolution, height * self.resolution)
        return self.load_from_array(elevation, bounds)
    
    def calculate_slope(self, dem_data: DEMData) -> np.ndarray:
        """
        Calculate terrain slope from DEM
        
        Args:
            dem_data: DEM data object
            
        Returns:
            Slope values in degrees
        """
        dz_dy, dz_dx = np.gradient(dem_data.elevation, dem_data.resolution)
        slope = np.arctan(np.sqrt(dz_dx**2 + dz_dy**2))
        return np.degrees(slope)
    
    def identify_obstacles(self, dem_data: DEMData, 
                         height_threshold: float = 30.0) -> np.ndarray:
        """
        Identify obstacles (buildings, structures) from DEM
        
        Args:
            dem_data: DEM data object
            height_threshold: Minimum height difference to consider as obstacle
            
        Returns:
            Binary mask where 1 indicates obstacle
        """
        # Use morphological operations to detect significant elevation changes
        smoothed = cv2.GaussianBlur(dem_data.elevation, (5, 5), 2.0)
        height_diff = dem_data.elevation - smoothed
        
        obstacles = (height_diff > height_threshold).astype(np.uint8)
        
        # Clean up noise with morphological operations
        kernel = np.ones((3, 3), np.uint8)
        obstacles = cv2.morphologyEx(obstacles, cv2.MORPH_CLOSE, kernel)
        obstacles = cv2.morphologyEx(obstacles, cv2.MORPH_OPEN, kernel)
        
        return obstacles
    
    def create_risk_map(self, dem_data: DEMData, 
                       population_density: Optional[np.ndarray] = None,
                       obstacle_mask: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Create risk map considering terrain, obstacles, and population
        
        Args:
            dem_data: DEM data object
            population_density: Population density grid (optional)
            obstacle_mask: Binary obstacle mask (optional)
            
        Returns:
            Risk values (0-1) where 1 is highest risk
        """
        height, width = dem_data.shape
        risk_map = np.zeros((height, width))
        
        # Add obstacle risk
        if obstacle_mask is not None:
            risk_map += obstacle_mask.astype(float) * 0.5
        
        # Add population density risk
        if population_density is not None:
            # Normalize population density to 0-1
            pop_normalized = (population_density - population_density.min()) / \
                           (population_density.max() - population_density.min() + 1e-10)
            risk_map += pop_normalized * 0.5
        
        # Clip to 0-1 range
        risk_map = np.clip(risk_map, 0, 1)
        
        return risk_map
    
    def get_elevation_at_point(self, dem_data: DEMData, x: float, y: float) -> float:
        """
        Get elevation at specific coordinate using bilinear interpolation
        
        Args:
            dem_data: DEM data object
            x: X coordinate
            y: Y coordinate
            
        Returns:
            Interpolated elevation value
        """
        # Convert coordinates to grid indices
        x_idx = np.interp(x, dem_data.x_coords, np.arange(len(dem_data.x_coords)))
        y_idx = np.interp(y, dem_data.y_coords, np.arange(len(dem_data.y_coords)))
        
        # Get surrounding grid points
        x0, x1 = int(np.floor(x_idx)), int(np.ceil(x_idx))
        y0, y1 = int(np.floor(y_idx)), int(np.ceil(y_idx))
        
        # Clip to valid range
        x0 = np.clip(x0, 0, dem_data.shape[1] - 1)
        x1 = np.clip(x1, 0, dem_data.shape[1] - 1)
        y0 = np.clip(y0, 0, dem_data.shape[0] - 1)
        y1 = np.clip(y1, 0, dem_data.shape[0] - 1)
        
        # Bilinear interpolation
        if x0 == x1 and y0 == y1:
            return dem_data.elevation[y0, x0]
        
        wx = x_idx - x0
        wy = y_idx - y0
        
        z00 = dem_data.elevation[y0, x0]
        z01 = dem_data.elevation[y1, x0]
        z10 = dem_data.elevation[y0, x1]
        z11 = dem_data.elevation[y1, x1]
        
        z = (1 - wx) * (1 - wy) * z00 + \
            (1 - wx) * wy * z01 + \
            wx * (1 - wy) * z10 + \
            wx * wy * z11
        
        return float(z)
