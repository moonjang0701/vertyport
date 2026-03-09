"""
Building Data Loader for Seoul Districts
서울시 구별 건물 SHP 파일 로더
"""

import geopandas as gpd
import numpy as np
from pathlib import Path
from typing import Optional, Dict, Tuple
import pandas as pd


class SeoulBuildingLoader:
    """서울시 구별 건물 데이터 로더"""
    
    def __init__(self, data_dir: str = "data/buildings"):
        """
        Initialize building loader
        
        Args:
            data_dir: Directory containing building SHP files
        """
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        
        # District name mapping (한글 → 영문 및 구 코드)
        self.district_map = {
            "관악구": {"eng": "gwanak", "code": "11620"},
            "강남구": {"eng": "gangnam", "code": "11680"},
            "서초구": {"eng": "seocho", "code": "11650"},
            "송파구": {"eng": "songpa", "code": "11710"},
            # Add more as needed
        }
    
    def load_district_buildings(self, 
                               district_name: str,
                               return_wgs84: bool = True) -> Optional[gpd.GeoDataFrame]:
        """
        Load building data for a specific district
        
        Args:
            district_name: 구 이름 (예: "관악구")
            return_wgs84: Return in WGS84 (lat/lon) coordinates
            
        Returns:
            GeoDataFrame with building data or None if not found
        """
        # Find SHP file - try multiple patterns
        shp_files = []
        
        # Pattern 1: Korean name in filename
        shp_files.extend(list(self.data_dir.glob(f"*{district_name}*.shp")))
        
        # Pattern 2: District code
        if district_name in self.district_map:
            district_code = self.district_map[district_name]["code"]
            shp_files.extend(list(self.data_dir.glob(f"*{district_code}*.shp")))
            
            # Pattern 3: English name
            eng_name = self.district_map[district_name]["eng"]
            shp_files.extend(list(self.data_dir.glob(f"*{eng_name}*.shp")))
        
        # Remove duplicates
        shp_files = list(set(shp_files))
        
        if not shp_files:
            print(f"⚠️  건물 데이터 없음: {district_name}")
            return None
        
        shp_file = shp_files[0]
        
        print(f"📂 로딩 중: {shp_file.name}")
        
        try:
            # Load shapefile
            gdf = gpd.read_file(shp_file, encoding='cp949')
            
            print(f"   ✅ {len(gdf):,}개 건물 로딩 완료")
            
            # Convert to WGS84 if needed
            if return_wgs84 and gdf.crs and gdf.crs.to_epsg() != 4326:
                print(f"   🔄 WGS84 변환 중...")
                gdf = gdf.to_crs(epsg=4326)
            
            return gdf
            
        except Exception as e:
            print(f"   ❌ 에러: {e}")
            return None
    
    def get_buildings_in_bounds(self,
                               gdf: gpd.GeoDataFrame,
                               lat_min: float, lat_max: float,
                               lon_min: float, lon_max: float) -> gpd.GeoDataFrame:
        """
        Filter buildings within bounds
        
        Args:
            gdf: GeoDataFrame with buildings
            lat_min, lat_max: Latitude range
            lon_min, lon_max: Longitude range
            
        Returns:
            Filtered GeoDataFrame
        """
        # Bounding box filter
        mask = (
            (gdf.geometry.bounds['miny'] >= lat_min) &
            (gdf.geometry.bounds['maxy'] <= lat_max) &
            (gdf.geometry.bounds['minx'] >= lon_min) &
            (gdf.geometry.bounds['maxx'] <= lon_max)
        )
        
        return gdf[mask]
    
    def extract_building_heights(self,
                                gdf: gpd.GeoDataFrame,
                                height_column: str = 'HEIGHT') -> np.ndarray:
        """
        Extract building heights as numpy array
        
        Args:
            gdf: GeoDataFrame with buildings
            height_column: Column name for height
            
        Returns:
            Array of building heights (meters)
        """
        if height_column not in gdf.columns:
            print(f"⚠️  높이 컬럼 없음: {height_column}")
            return np.zeros(len(gdf))
        
        heights = gdf[height_column].fillna(0).values
        
        # Filter out zero/invalid heights
        # If HEIGHT is 0, estimate from floor count
        if 'GRND_FLR' in gdf.columns:
            zero_mask = heights == 0
            floor_counts = gdf['GRND_FLR'].fillna(1).values
            # Assume 3m per floor
            estimated_heights = floor_counts * 3.0
            heights = np.where(zero_mask, estimated_heights, heights)
        
        return heights
    
    def create_obstacle_map(self,
                           gdf: gpd.GeoDataFrame,
                           resolution: int = 256,
                           lat_min: float = None,
                           lat_max: float = None,
                           lon_min: float = None,
                           lon_max: float = None) -> Tuple[np.ndarray, Dict]:
        """
        Create 2D obstacle map from buildings
        
        Args:
            gdf: GeoDataFrame with buildings (WGS84)
            resolution: Grid resolution
            lat_min, lat_max, lon_min, lon_max: Bounds (optional)
            
        Returns:
            (obstacle_map, metadata) where obstacle_map is max building height per cell
        """
        # Get bounds
        if lat_min is None:
            bounds = gdf.total_bounds
            lon_min, lat_min, lon_max, lat_max = bounds
        
        # Create grid
        lon_grid = np.linspace(lon_min, lon_max, resolution)
        lat_grid = np.linspace(lat_min, lat_max, resolution)
        
        # Initialize obstacle map
        obstacle_map = np.zeros((resolution, resolution))
        
        # Extract heights
        heights = self.extract_building_heights(gdf)
        
        # Rasterize buildings
        for idx, (geom, height) in enumerate(zip(gdf.geometry, heights)):
            if geom is None or height <= 0:
                continue
            
            # Get building centroid
            centroid = geom.centroid
            lon, lat = centroid.x, centroid.y
            
            # Find grid cell
            lon_idx = np.searchsorted(lon_grid, lon)
            lat_idx = np.searchsorted(lat_grid, lat)
            
            # Bound check
            if 0 <= lon_idx < resolution and 0 <= lat_idx < resolution:
                # Take maximum height in each cell
                obstacle_map[lat_idx, lon_idx] = max(
                    obstacle_map[lat_idx, lon_idx],
                    height
                )
        
        metadata = {
            'lat_min': lat_min,
            'lat_max': lat_max,
            'lon_min': lon_min,
            'lon_max': lon_max,
            'resolution': resolution,
            'max_height': float(np.max(obstacle_map)),
            'num_buildings': len(gdf),
            'cells_with_buildings': int(np.sum(obstacle_map > 0))
        }
        
        return obstacle_map, metadata
    
    def get_building_statistics(self, gdf: gpd.GeoDataFrame) -> Dict:
        """Get building statistics"""
        heights = self.extract_building_heights(gdf)
        
        stats = {
            'total_buildings': len(gdf),
            'height_min': float(np.min(heights)),
            'height_max': float(np.max(heights)),
            'height_mean': float(np.mean(heights)),
            'height_median': float(np.median(heights)),
            'height_std': float(np.std(heights))
        }
        
        # Floor statistics
        if 'GRND_FLR' in gdf.columns:
            floors = gdf['GRND_FLR'].fillna(0).values
            stats.update({
                'floor_min': int(np.min(floors)),
                'floor_max': int(np.max(floors)),
                'floor_mean': float(np.mean(floors))
            })
        
        return stats


# Example usage
if __name__ == "__main__":
    loader = SeoulBuildingLoader()
    
    print("="*80)
    print("서울시 건물 데이터 로더 테스트")
    print("="*80)
    
    # Load Gwanak-gu buildings
    gdf = loader.load_district_buildings("관악구")
    
    if gdf is not None:
        # Statistics
        stats = loader.get_building_statistics(gdf)
        
        print(f"\n📊 건물 통계:")
        print(f"   총 건물 수: {stats['total_buildings']:,}")
        print(f"   높이 범위: {stats['height_min']:.1f}m ~ {stats['height_max']:.1f}m")
        print(f"   평균 높이: {stats['height_mean']:.1f}m")
        
        if 'floor_max' in stats:
            print(f"   최대 층수: {stats['floor_max']}층")
        
        # Create obstacle map
        print(f"\n🗺️  장애물 맵 생성 중...")
        obstacle_map, metadata = loader.create_obstacle_map(gdf, resolution=256)
        
        print(f"   ✅ 완료:")
        print(f"      해상도: {metadata['resolution']}×{metadata['resolution']}")
        print(f"      최대 높이: {metadata['max_height']:.1f}m")
        print(f"      건물 있는 셀: {metadata['cells_with_buildings']:,}")
        
        # Test filtering
        print(f"\n🔍 경계 필터 테스트...")
        bounds = gdf.total_bounds
        mid_lat = (bounds[1] + bounds[3]) / 2
        mid_lon = (bounds[0] + bounds[2]) / 2
        delta = 0.01
        
        filtered = loader.get_buildings_in_bounds(
            gdf,
            mid_lat - delta, mid_lat + delta,
            mid_lon - delta, mid_lon + delta
        )
        
        print(f"   필터 범위: ±{delta}° from center")
        print(f"   필터링된 건물: {len(filtered):,}개")
