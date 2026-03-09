"""
Safety Map Visualization
실제 지도 위에 위험도 기반 안전/위험 구역 시각화

논문 기반:
- Green zone: Low risk (safe for flight)
- Yellow zone: Medium risk (caution)
- Red zone: High risk (geofence, avoid)
"""

import numpy as np
import plotly.graph_objects as go
from typing import Tuple, Optional, List


class SafetyMapVisualizer:
    """
    실제 지도 위에 위험도 맵 오버레이
    
    Features:
    - Color-coded risk zones (green → yellow → red)
    - Geofence visualization for high-risk areas
    - Flight path overlay
    - Interactive map with lat/lon coordinates
    """
    
    def __init__(self,
                 risk_map: np.ndarray,
                 lat_min: float,
                 lat_max: float,
                 lon_min: float,
                 lon_max: float,
                 risk_thresholds: Optional[Tuple[float, float, float]] = None):
        """
        Initialize safety map visualizer
        
        Args:
            risk_map: 2D risk values
            lat_min, lat_max, lon_min, lon_max: Geographic bounds
            risk_thresholds: (low, medium, high) thresholds
                             default: (0.1, 0.3, 1.0)
        """
        self.risk_map = risk_map
        self.lat_min = lat_min
        self.lat_max = lat_max
        self.lon_min = lon_min
        self.lon_max = lon_max
        
        if risk_thresholds is None:
            # 논문 기준: SORA threshold ≈ 1e-6, but relaxed for visualization
            self.threshold_low = 0.1
            self.threshold_medium = 0.3
            self.threshold_high = 1.0
        else:
            self.threshold_low, self.threshold_medium, self.threshold_high = risk_thresholds
        
        self.grid_h, self.grid_w = risk_map.shape
        self.lat_step = (lat_max - lat_min) / self.grid_h
        self.lon_step = (lon_max - lon_min) / self.grid_w
    
    def create_safety_map(self, show_grid: bool = False) -> go.Figure:
        """
        Create safety map with color-coded risk zones
        
        Returns:
            Plotly Figure with geographic coordinates
        """
        
        # Create lat/lon coordinate arrays
        lats = np.linspace(self.lat_min, self.lat_max, self.grid_h)
        lons = np.linspace(self.lon_min, self.lon_max, self.grid_w)
        
        # Create custom colorscale (green → yellow → red)
        # Based on risk thresholds
        colorscale = [
            [0.0, 'rgb(0, 255, 0)'],      # Green (safe)
            [0.33, 'rgb(144, 238, 144)'], # Light green
            [0.5, 'rgb(255, 255, 0)'],    # Yellow (caution)
            [0.66, 'rgb(255, 165, 0)'],   # Orange
            [0.83, 'rgb(255, 69, 0)'],    # Red-orange
            [1.0, 'rgb(255, 0, 0)']       # Red (dangerous)
        ]
        
        fig = go.Figure()
        
        # Add risk heatmap
        fig.add_trace(go.Heatmap(
            z=self.risk_map,
            x=lons,
            y=lats,
            colorscale=colorscale,
            zmin=0.0,
            zmax=self.threshold_high,
            colorbar=dict(
                title="위험도<br>Risk Level",
                tickvals=[0, self.threshold_low, self.threshold_medium, self.threshold_high],
                ticktext=['Safe<br>0.0', f'Low<br>{self.threshold_low}',
                         f'Medium<br>{self.threshold_medium}', f'High<br>{self.threshold_high}'],
                len=0.7,
                thickness=20
            ),
            hovertemplate='<b>위치</b><br>' +
                         'Lat: %{y:.4f}<br>' +
                         'Lon: %{x:.4f}<br>' +
                         'Risk: %{z:.3f}<br>' +
                         '<extra></extra>',
            opacity=0.7
        ))
        
        fig.update_layout(
            title="관악구 UAM 안전 구역 맵 (실제 건물 DEM 기반)",
            xaxis_title="경도 (Longitude)",
            yaxis_title="위도 (Latitude)",
            height=700,
            width=900,
            xaxis=dict(
                showgrid=show_grid,
                zeroline=False
            ),
            yaxis=dict(
                showgrid=show_grid,
                zeroline=False,
                scaleanchor="x",
                scaleratio=1
            )
        )
        
        return fig
    
    def add_flight_path(self, fig: go.Figure, waypoints: List[Tuple[float, float, float]],
                       path_color: str = 'white', path_width: int = 4) -> go.Figure:
        """
        Add flight path to safety map
        
        Args:
            fig: Existing Plotly figure
            waypoints: List of (lat, lon, alt) tuples
            path_color: Path line color
            path_width: Path line width
        
        Returns:
            Updated figure
        """
        if not waypoints:
            return fig
        
        lats = [wp[0] for wp in waypoints]
        lons = [wp[1] for wp in waypoints]
        alts = [wp[2] for wp in waypoints]
        
        # Add path line
        fig.add_trace(go.Scatter(
            x=lons,
            y=lats,
            mode='lines+markers',
            name='비행 경로',
            line=dict(color=path_color, width=path_width),
            marker=dict(size=8, color=path_color, symbol='circle',
                       line=dict(color='black', width=1)),
            hovertemplate='<b>경로 웨이포인트</b><br>' +
                         'Lat: %{y:.4f}<br>' +
                         'Lon: %{x:.4f}<br>' +
                         'Alt: %{customdata:.0f}m<br>' +
                         '<extra></extra>',
            customdata=alts
        ))
        
        # Mark start and end
        fig.add_trace(go.Scatter(
            x=[lons[0]],
            y=[lats[0]],
            mode='markers',
            name='출발지',
            marker=dict(size=15, color='lime', symbol='star',
                       line=dict(color='black', width=2)),
            hovertemplate='<b>출발지</b><br>' +
                         'Lat: %{y:.4f}<br>' +
                         'Lon: %{x:.4f}<br>' +
                         '<extra></extra>'
        ))
        
        fig.add_trace(go.Scatter(
            x=[lons[-1]],
            y=[lats[-1]],
            mode='markers',
            name='도착지',
            marker=dict(size=15, color='cyan', symbol='star',
                       line=dict(color='black', width=2)),
            hovertemplate='<b>도착지</b><br>' +
                         'Lat: %{y:.4f}<br>' +
                         'Lon: %{x:.4f}<br>' +
                         '<extra></extra>'
        ))
        
        return fig
    
    def add_geofence(self, fig: go.Figure, geofence_cells: List[Tuple[int, int]],
                    fence_color: str = 'yellow') -> go.Figure:
        """
        Add geofence markers for high-risk zones
        
        Args:
            fig: Existing figure
            geofence_cells: List of (i, j) grid cells to fence
            fence_color: Fence boundary color
        
        Returns:
            Updated figure
        """
        if not geofence_cells:
            return fig
        
        lats = []
        lons = []
        risks = []
        
        for i, j in geofence_cells:
            lat = self.lat_min + i * self.lat_step
            lon = self.lon_min + j * self.lon_step
            lats.append(lat)
            lons.append(lon)
            risks.append(self.risk_map[i, j])
        
        fig.add_trace(go.Scatter(
            x=lons,
            y=lats,
            mode='markers',
            name='Geofence (고위험)',
            marker=dict(size=10, color=fence_color, symbol='x',
                       line=dict(color='black', width=2)),
            hovertemplate='<b>고위험 구역</b><br>' +
                         'Lat: %{y:.4f}<br>' +
                         'Lon: %{x:.4f}<br>' +
                         'Risk: %{customdata:.3f}<br>' +
                         '<extra></extra>',
            customdata=risks
        ))
        
        return fig
    
    def get_geofence_cells(self, threshold: Optional[float] = None) -> List[Tuple[int, int]]:
        """
        Identify cells that need geofencing (high risk)
        
        Args:
            threshold: Risk threshold (default: self.threshold_medium)
        
        Returns:
            List of (i, j) grid indices
        """
        if threshold is None:
            threshold = self.threshold_medium
        
        geofence_cells = []
        for i in range(self.grid_h):
            for j in range(self.grid_w):
                if self.risk_map[i, j] > threshold:
                    geofence_cells.append((i, j))
        
        return geofence_cells
    
    def get_statistics(self) -> dict:
        """Get safety zone statistics"""
        total_cells = self.grid_h * self.grid_w
        
        safe_cells = np.sum(self.risk_map <= self.threshold_low)
        caution_cells = np.sum((self.risk_map > self.threshold_low) & 
                               (self.risk_map <= self.threshold_medium))
        danger_cells = np.sum(self.risk_map > self.threshold_medium)
        
        return {
            'total_cells': total_cells,
            'safe_cells': int(safe_cells),
            'safe_percent': (safe_cells / total_cells) * 100,
            'caution_cells': int(caution_cells),
            'caution_percent': (caution_cells / total_cells) * 100,
            'danger_cells': int(danger_cells),
            'danger_percent': (danger_cells / total_cells) * 100,
            'avg_risk': float(np.mean(self.risk_map)),
            'max_risk': float(np.max(self.risk_map)),
            'min_risk': float(np.min(self.risk_map))
        }
