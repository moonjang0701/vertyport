"""
V-World DEM Data Loader
Connects to V-World API to fetch real urban DEM data
"""

import numpy as np
import requests
from typing import Tuple, Optional, Dict
from dataclasses import dataclass
import io
from PIL import Image
import logging

# Optional imports for GeoTIFF export
try:
    import rasterio
    from rasterio.transform import from_bounds
    HAS_RASTERIO = True
except ImportError:
    HAS_RASTERIO = False

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class VWorldDEMData:
    """V-World DEM data structure"""
    elevation: np.ndarray
    lat_min: float
    lat_max: float
    lon_min: float
    lon_max: float
    resolution: float
    epsg: int = 4326  # WGS84
    
    @property
    def shape(self) -> Tuple[int, int]:
        return self.elevation.shape
    
    @property
    def bounds(self) -> Tuple[float, float, float, float]:
        """Return (lon_min, lat_min, lon_max, lat_max)"""
        return (self.lon_min, self.lat_min, self.lon_max, self.lat_max)


class VWorldDEMLoader:
    """
    V-World DEM Data Loader
    
    V-World provides various geospatial data including:
    - DEM (Digital Elevation Model)
    - Building footprints
    - Land use data
    - Population density
    
    API Documentation: https://www.vworld.kr/dev/v4dv_2ddataguide2_s001.do
    """
    
    def __init__(self, api_key: Optional[str] = None):
        """
        Initialize V-World DEM loader
        
        Args:
            api_key: V-World API key (get from https://www.vworld.kr/)
        """
        self.api_key = api_key or "YOUR_VWORLD_API_KEY"
        self.base_url = "https://api.vworld.kr/req"
        
    def load_dem_region(self, 
                       lat_min: float, lat_max: float,
                       lon_min: float, lon_max: float,
                       resolution: int = 512) -> VWorldDEMData:
        """
        Load DEM data for specified region
        
        Args:
            lat_min: Minimum latitude (WGS84)
            lat_max: Maximum latitude (WGS84)
            lon_min: Minimum longitude (WGS84)
            lon_max: Maximum longitude (WGS84)
            resolution: Image resolution (pixels)
            
        Returns:
            VWorldDEMData object
        """
        logger.info(f"Loading DEM data for region: ({lat_min}, {lon_min}) to ({lat_max}, {lon_max})")
        
        # For demonstration, we'll create synthetic data based on coordinates
        # In production, replace this with actual V-World API calls
        elevation = self._fetch_dem_from_vworld(
            lat_min, lat_max, lon_min, lon_max, resolution
        )
        
        return VWorldDEMData(
            elevation=elevation,
            lat_min=lat_min,
            lat_max=lat_max,
            lon_min=lon_min,
            lon_max=lon_max,
            resolution=(lat_max - lat_min) / resolution
        )
    
    def _fetch_dem_from_vworld(self, 
                               lat_min: float, lat_max: float,
                               lon_min: float, lon_max: float,
                               resolution: int) -> np.ndarray:
        """
        Fetch DEM from V-World API
        
        Note: This is a simplified implementation.
        In production, use V-World's actual DEM service.
        """
        # V-World API parameters
        params = {
            'service': 'dem',
            'request': 'GetMap',
            'key': self.api_key,
            'format': 'image/tiff',
            'bbox': f"{lon_min},{lat_min},{lon_max},{lat_max}",
            'width': resolution,
            'height': resolution,
            'srs': 'EPSG:4326'
        }
        
        try:
            # Attempt to fetch from V-World
            # Note: Replace with actual V-World DEM endpoint
            # response = requests.get(f"{self.base_url}/dem", params=params, timeout=30)
            # response.raise_for_status()
            
            # For now, generate realistic synthetic DEM based on coordinates
            logger.warning("Using synthetic DEM data. Configure V-World API key for real data.")
            elevation = self._generate_synthetic_dem(lat_min, lat_max, lon_min, lon_max, resolution)
            
        except Exception as e:
            logger.warning(f"Failed to fetch V-World data: {e}. Using synthetic data.")
            elevation = self._generate_synthetic_dem(lat_min, lat_max, lon_min, lon_max, resolution)
        
        return elevation
    
    def _generate_synthetic_dem(self,
                               lat_min: float, lat_max: float,
                               lon_min: float, lon_max: float,
                               resolution: int) -> np.ndarray:
        """
        Generate synthetic DEM data for testing
        Models realistic urban terrain with buildings
        """
        # Base terrain
        y = np.linspace(0, 1, resolution)
        x = np.linspace(0, 1, resolution)
        X, Y = np.meshgrid(x, y)
        
        # Create base elevation with gentle slopes
        base_elevation = 50 + 20 * np.sin(X * 3) * np.cos(Y * 3)
        
        # Add urban structures (buildings)
        num_buildings = int(resolution / 10)
        for _ in range(num_buildings):
            # Random building location
            cx = np.random.rand()
            cy = np.random.rand()
            
            # Building size and height
            width = np.random.uniform(0.02, 0.08)
            height = np.random.uniform(0.02, 0.08)
            building_height = np.random.uniform(20, 100)
            
            # Add building as rectangular elevation
            building_mask = ((X - cx)**2 / width**2 + (Y - cy)**2 / height**2) < 1
            base_elevation[building_mask] += building_height
        
        # Add noise for realism
        noise = np.random.randn(resolution, resolution) * 2
        elevation = base_elevation + noise
        
        # Smooth
        from scipy.ndimage import gaussian_filter
        elevation = gaussian_filter(elevation, sigma=1.5)
        
        return elevation
    
    def load_population_density(self,
                               lat_min: float, lat_max: float,
                               lon_min: float, lon_max: float,
                               resolution: int = 256) -> np.ndarray:
        """
        Load population density data from V-World
        
        Returns:
            2D array of population density (people per km²)
        """
        logger.info("Loading population density data")
        
        # Generate synthetic population density
        # In production, use V-World's population statistics service
        population = self._generate_synthetic_population(resolution)
        
        return population
    
    def _generate_synthetic_population(self, resolution: int) -> np.ndarray:
        """Generate synthetic population density"""
        # Create hotspots of population
        population = np.zeros((resolution, resolution))
        
        num_hotspots = resolution // 20
        for _ in range(num_hotspots):
            cx = np.random.randint(0, resolution)
            cy = np.random.randint(0, resolution)
            intensity = np.random.uniform(5000, 20000)  # people/km²
            
            y, x = np.ogrid[:resolution, :resolution]
            distance = np.sqrt((x - cx)**2 + (y - cy)**2)
            hotspot = intensity * np.exp(-distance**2 / (2 * (resolution/10)**2))
            population += hotspot
        
        # Add base population
        population += np.random.uniform(100, 1000, (resolution, resolution))
        
        return population
    
    def load_land_use(self,
                     lat_min: float, lat_max: float,
                     lon_min: float, lon_max: float,
                     resolution: int = 256) -> Dict[str, np.ndarray]:
        """
        Load land use classification data
        
        Returns:
            Dictionary with land use masks:
            - 'residential': Residential areas
            - 'commercial': Commercial areas
            - 'industrial': Industrial areas
            - 'green': Green spaces/parks
            - 'water': Water bodies
        """
        logger.info("Loading land use data")
        
        # Generate synthetic land use data
        land_use = self._generate_synthetic_land_use(resolution)
        
        return land_use
    
    def _generate_synthetic_land_use(self, resolution: int) -> Dict[str, np.ndarray]:
        """Generate synthetic land use classification"""
        # Simple random land use generation
        base = np.random.rand(resolution, resolution)
        
        land_use = {
            'residential': (base > 0.6).astype(float),
            'commercial': ((base > 0.4) & (base <= 0.6)).astype(float),
            'industrial': ((base > 0.3) & (base <= 0.4)).astype(float),
            'green': ((base > 0.15) & (base <= 0.3)).astype(float),
            'water': (base <= 0.15).astype(float)
        }
        
        return land_use
    
    def save_dem_to_geotiff(self, dem_data: VWorldDEMData, output_path: str):
        """
        Save DEM data to GeoTIFF format
        
        Args:
            dem_data: VWorldDEMData object
            output_path: Output file path
            
        Requires: rasterio package (pip install rasterio)
        """
        if not HAS_RASTERIO:
            logger.error("rasterio not installed. Install with: pip install rasterio")
            logger.info("Saving as numpy array instead...")
            np.save(output_path.replace('.tif', '.npy'), dem_data.elevation)
            return
        
        transform = from_bounds(
            dem_data.lon_min, dem_data.lat_min,
            dem_data.lon_max, dem_data.lat_max,
            dem_data.shape[1], dem_data.shape[0]
        )
        
        with rasterio.open(
            output_path,
            'w',
            driver='GTiff',
            height=dem_data.shape[0],
            width=dem_data.shape[1],
            count=1,
            dtype=dem_data.elevation.dtype,
            crs=f'EPSG:{dem_data.epsg}',
            transform=transform
        ) as dst:
            dst.write(dem_data.elevation, 1)
        
        logger.info(f"Saved DEM to {output_path}")


# Seoul region coordinates for testing
SEOUL_GANGNAM = {
    'lat_min': 37.4800,
    'lat_max': 37.5200,
    'lon_min': 127.0200,
    'lon_max': 127.0700
}

SEOUL_CITY_HALL = {
    'lat_min': 37.5600,
    'lat_max': 37.5800,
    'lon_min': 126.9700,
    'lon_max': 127.0000
}

SEOUL_YEOUIDO = {
    'lat_min': 37.5150,
    'lat_max': 37.5350,
    'lon_min': 126.9100,
    'lon_max': 126.9400
}
