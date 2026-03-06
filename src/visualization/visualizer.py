"""
3D Visualization for UAM path planning and vertiport optimization
"""

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from typing import List, Optional, Dict
from ..data_processing.dem_processor import DEMData
from ..path_planning.path_planner import FlightPath, Waypoint
from ..vertiport_optimization.optimizer import VertiportCandidate


class Visualizer3D:
    """3D visualization for UAM simulation"""
    
    def __init__(self, dem_data: DEMData):
        """
        Initialize visualizer
        
        Args:
            dem_data: Digital elevation model data
        """
        self.dem_data = dem_data
        
    def create_terrain_surface(self, opacity: float = 0.8, 
                               colorscale: str = 'Viridis') -> go.Surface:
        """
        Create 3D terrain surface
        
        Args:
            opacity: Surface opacity (0-1)
            colorscale: Plotly colorscale name
            
        Returns:
            Plotly Surface object
        """
        x_grid, y_grid = np.meshgrid(
            self.dem_data.x_coords,
            self.dem_data.y_coords
        )
        
        surface = go.Surface(
            x=x_grid,
            y=y_grid,
            z=self.dem_data.elevation,
            colorscale=colorscale,
            opacity=opacity,
            name='Terrain',
            showscale=True,
            colorbar=dict(title='Elevation (m)', x=1.15)
        )
        
        return surface
    
    def create_risk_overlay(self, risk_map: np.ndarray,
                          height_offset: float = 5.0) -> go.Surface:
        """
        Create risk map overlay on terrain
        
        Args:
            risk_map: 2D risk map
            height_offset: Height above terrain to display overlay
            
        Returns:
            Plotly Surface object
        """
        x_grid, y_grid = np.meshgrid(
            self.dem_data.x_coords,
            self.dem_data.y_coords
        )
        
        z_grid = self.dem_data.elevation + height_offset
        
        # Create risk overlay
        risk_surface = go.Surface(
            x=x_grid,
            y=y_grid,
            z=z_grid,
            surfacecolor=risk_map,
            colorscale='Reds',
            opacity=0.5,
            name='Risk Map',
            showscale=True,
            colorbar=dict(title='Risk Level', x=1.0)
        )
        
        return risk_surface
    
    def create_flight_path_trace(self, path: FlightPath,
                                color: str = 'blue',
                                name: str = 'Flight Path',
                                width: int = 5) -> go.Scatter3d:
        """
        Create 3D flight path trace
        
        Args:
            path: FlightPath object
            color: Line color
            name: Trace name
            width: Line width
            
        Returns:
            Plotly Scatter3d object
        """
        x = [wp.x for wp in path.waypoints]
        y = [wp.y for wp in path.waypoints]
        z = [wp.z for wp in path.waypoints]
        
        trace = go.Scatter3d(
            x=x, y=y, z=z,
            mode='lines+markers',
            line=dict(color=color, width=width),
            marker=dict(size=3, color=color),
            name=name,
            hovertemplate='<b>Waypoint</b><br>' +
                         'X: %{x:.1f}m<br>' +
                         'Y: %{y:.1f}m<br>' +
                         'Z: %{z:.1f}m<br>' +
                         '<extra></extra>'
        )
        
        return trace
    
    def create_vertiport_markers(self, vertiports: List[VertiportCandidate],
                                size: int = 15) -> go.Scatter3d:
        """
        Create vertiport location markers
        
        Args:
            vertiports: List of vertiport candidates
            size: Marker size
            
        Returns:
            Plotly Scatter3d object
        """
        x = [vp.x for vp in vertiports]
        y = [vp.y for vp in vertiports]
        z = [vp.z + 10 for vp in vertiports]  # Slightly above ground
        
        colors = [vp.score for vp in vertiports]
        
        labels = [f'<b>VP-{i+1}</b><br>' +
                 f'Score: {vp.score:.2f}<br>' +
                 f'Safety: {vp.safety_score:.2f}<br>' +
                 f'Access: {vp.accessibility_score:.2f}<br>' +
                 f'Connect: {vp.connectivity_score:.2f}<br>' +
                 f'Coverage: {vp.coverage_area:.0f}m²'
                 for i, vp in enumerate(vertiports)]
        
        markers = go.Scatter3d(
            x=x, y=y, z=z,
            mode='markers+text',
            marker=dict(
                size=size,
                color=colors,
                colorscale='Viridis',
                showscale=True,
                colorbar=dict(title='Score', x=0.85),
                line=dict(color='white', width=2),
                symbol='diamond'
            ),
            text=[f'VP-{i+1}' for i in range(len(vertiports))],
            textposition='top center',
            textfont=dict(size=10, color='white'),
            name='Vertiports',
            hovertext=labels,
            hoverinfo='text'
        )
        
        return markers
    
    def create_vertiport_connections(self, vertiports: List[VertiportCandidate],
                                    max_distance: float = 1000.0) -> List[go.Scatter3d]:
        """
        Create connection lines between vertiports
        
        Args:
            vertiports: List of vertiport candidates
            max_distance: Maximum distance to draw connections
            
        Returns:
            List of Plotly Scatter3d objects
        """
        traces = []
        
        for i in range(len(vertiports)):
            for j in range(i + 1, len(vertiports)):
                vp1 = vertiports[i]
                vp2 = vertiports[j]
                
                dist = np.sqrt((vp1.x - vp2.x)**2 + (vp1.y - vp2.y)**2)
                
                if dist <= max_distance:
                    trace = go.Scatter3d(
                        x=[vp1.x, vp2.x],
                        y=[vp1.y, vp2.y],
                        z=[vp1.z + 10, vp2.z + 10],
                        mode='lines',
                        line=dict(color='yellow', width=2, dash='dash'),
                        showlegend=False,
                        hoverinfo='skip'
                    )
                    traces.append(trace)
        
        return traces
    
    def create_complete_visualization(self, 
                                     risk_map: Optional[np.ndarray] = None,
                                     paths: Optional[List[FlightPath]] = None,
                                     vertiports: Optional[List[VertiportCandidate]] = None,
                                     title: str = 'UAM Path Planning & Vertiport Optimization') -> go.Figure:
        """
        Create complete visualization with all elements
        
        Args:
            risk_map: Risk map overlay (optional)
            paths: Flight paths to display (optional)
            vertiports: Vertiport locations (optional)
            title: Figure title
            
        Returns:
            Plotly Figure object
        """
        fig = go.Figure()
        
        # Add terrain
        fig.add_trace(self.create_terrain_surface())
        
        # Add risk map overlay
        if risk_map is not None:
            fig.add_trace(self.create_risk_overlay(risk_map))
        
        # Add flight paths
        if paths is not None:
            colors = ['blue', 'green', 'purple', 'orange', 'cyan']
            for i, path in enumerate(paths):
                color = colors[i % len(colors)]
                fig.add_trace(self.create_flight_path_trace(
                    path, color=color, name=f'Path {i+1}'
                ))
        
        # Add vertiports
        if vertiports is not None:
            fig.add_trace(self.create_vertiport_markers(vertiports))
            
            # Add connections
            connection_traces = self.create_vertiport_connections(vertiports)
            for trace in connection_traces:
                fig.add_trace(trace)
        
        # Update layout
        fig.update_layout(
            title=dict(text=title, x=0.5, xanchor='center'),
            scene=dict(
                xaxis_title='X (m)',
                yaxis_title='Y (m)',
                zaxis_title='Altitude (m)',
                aspectmode='manual',
                aspectratio=dict(x=1, y=1, z=0.5),
                camera=dict(
                    eye=dict(x=1.5, y=1.5, z=1.2)
                )
            ),
            width=1200,
            height=800,
            showlegend=True,
            legend=dict(x=0.02, y=0.98),
            hovermode='closest'
        )
        
        return fig
    
    def create_2d_risk_heatmap(self, risk_map: np.ndarray,
                              vertiports: Optional[List[VertiportCandidate]] = None) -> go.Figure:
        """
        Create 2D risk heatmap view
        
        Args:
            risk_map: Risk map data
            vertiports: Vertiport locations (optional)
            
        Returns:
            Plotly Figure object
        """
        fig = go.Figure()
        
        # Add risk heatmap
        fig.add_trace(go.Heatmap(
            x=self.dem_data.x_coords,
            y=self.dem_data.y_coords,
            z=risk_map,
            colorscale='Reds',
            colorbar=dict(title='Risk Level')
        ))
        
        # Add vertiport markers
        if vertiports is not None:
            x = [vp.x for vp in vertiports]
            y = [vp.y for vp in vertiports]
            
            fig.add_trace(go.Scatter(
                x=x, y=y,
                mode='markers+text',
                marker=dict(size=15, color='green', symbol='diamond',
                          line=dict(color='white', width=2)),
                text=[f'VP-{i+1}' for i in range(len(vertiports))],
                textposition='top center',
                name='Vertiports'
            ))
        
        fig.update_layout(
            title='Risk Map and Vertiport Locations',
            xaxis_title='X (m)',
            yaxis_title='Y (m)',
            width=900,
            height=700
        )
        
        return fig
    
    def create_elevation_profile(self, path: FlightPath) -> go.Figure:
        """
        Create elevation profile along flight path
        
        Args:
            path: Flight path
            
        Returns:
            Plotly Figure object
        """
        # Calculate cumulative distance
        distances = [0]
        for i in range(1, len(path.waypoints)):
            dist = path.waypoints[i-1].distance_to(path.waypoints[i])
            distances.append(distances[-1] + dist)
        
        altitudes = [wp.z for wp in path.waypoints]
        
        fig = go.Figure()
        
        fig.add_trace(go.Scatter(
            x=distances,
            y=altitudes,
            mode='lines+markers',
            line=dict(color='blue', width=3),
            marker=dict(size=6),
            name='Flight Path'
        ))
        
        fig.update_layout(
            title='Flight Path Elevation Profile',
            xaxis_title='Distance (m)',
            yaxis_title='Altitude (m)',
            width=1000,
            height=400,
            showlegend=True
        )
        
        return fig
