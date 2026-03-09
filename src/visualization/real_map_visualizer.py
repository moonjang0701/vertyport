"""
Real Map Visualization using Folium and Plotly
Displays UAM routes and vertiports on actual geographic maps
"""

import folium
from folium import plugins
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np
from typing import List, Tuple, Optional
import branca.colormap as cm

from ..data_processing.vworld_loader import VWorldDEMData
from ..path_planning.path_planner import FlightPath, Waypoint
from ..vertiport_optimization.route_based_extractor import SafeZone


class RealMapVisualizer:
    """Visualize UAM system on real geographic maps"""
    
    def __init__(self, dem_data: VWorldDEMData):
        """
        Initialize visualizer with DEM data
        
        Args:
            dem_data: V-World DEM data with geographic coordinates
        """
        self.dem_data = dem_data
        self.center_lat = (dem_data.lat_min + dem_data.lat_max) / 2
        self.center_lon = (dem_data.lon_min + dem_data.lon_max) / 2
    
    def create_folium_map(self,
                         routes: Optional[List[FlightPath]] = None,
                         vertiports: Optional[List[SafeZone]] = None,
                         risk_map: Optional[np.ndarray] = None,
                         show_3d_buildings: bool = True) -> folium.Map:
        """
        Create interactive Folium map with routes and vertiports
        
        Args:
            routes: List of flight paths to display
            vertiports: List of vertiport locations
            risk_map: Risk heatmap overlay
            show_3d_buildings: Enable 3D building view
            
        Returns:
            Folium Map object
        """
        # Create base map
        m = folium.Map(
            location=[self.center_lat, self.center_lon],
            zoom_start=14,
            tiles=None,
            control_scale=True
        )
        
        # Add multiple tile layers
        folium.TileLayer('OpenStreetMap', name='OpenStreetMap').add_to(m)
        folium.TileLayer('CartoDB positron', name='Light Map').add_to(m)
        folium.TileLayer(
            tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            attr='Esri',
            name='Satellite',
            overlay=False,
            control=True
        ).add_to(m)
        
        # Add V-World base map (Korean) - using double braces for z,y,x
        folium.TileLayer(
            tiles='http://api.vworld.kr/req/wmts/1.0.0/YOUR_VWORLD_API_KEY/Base/{{z}}/{{y}}/{{x}}.png',
            attr='V-World',
            name='V-World Base',
            overlay=False,
            control=True
        ).add_to(m)
        
        # Add risk heatmap
        if risk_map is not None:
            self._add_risk_heatmap(m, risk_map)
        
        # Add DEM elevation overlay
        self._add_elevation_overlay(m)
        
        # Add flight routes
        if routes is not None:
            self._add_routes_to_folium(m, routes)
        
        # Add vertiports
        if vertiports is not None:
            self._add_vertiports_to_folium(m, vertiports)
        
        # Add layer control
        folium.LayerControl(position='topright', collapsed=False).add_to(m)
        
        # Add fullscreen button
        plugins.Fullscreen(position='topleft').add_to(m)
        
        # Add measure control
        plugins.MeasureControl(position='bottomleft').add_to(m)
        
        # Add minimap
        minimap = plugins.MiniMap(toggle_display=True)
        m.add_child(minimap)
        
        return m
    
    def _add_elevation_overlay(self, m: folium.Map):
        """Add DEM elevation as colored overlay"""
        # Create color map
        elevation = self.dem_data.elevation
        min_elev = np.nanmin(elevation)
        max_elev = np.nanmax(elevation)
        
        # Normalize elevation for coloring
        elev_norm = (elevation - min_elev) / (max_elev - min_elev)
        
        # Create image overlay
        colormap = cm.LinearColormap(
            colors=['green', 'yellow', 'orange', 'red'],
            vmin=min_elev,
            vmax=max_elev,
            caption='Elevation (m)'
        )
        
        # Add to map as ImageOverlay
        # Note: Folium's ImageOverlay expects RGBA format
        from PIL import Image
        import matplotlib.cm as mpl_cm
        
        # Convert elevation to RGBA
        rgba = mpl_cm.terrain(elev_norm)
        rgba[:, :, 3] = 0.5  # Set transparency
        rgba = (rgba * 255).astype(np.uint8)
        
        img = Image.fromarray(rgba, mode='RGBA')
        
        # Save to temporary buffer
        import io
        import base64
        buffer = io.BytesIO()
        img.save(buffer, format='PNG')
        buffer.seek(0)
        img_str = base64.b64encode(buffer.read()).decode()
        
        # Add as ImageOverlay
        folium.raster_layers.ImageOverlay(
            image=f'data:image/png;base64,{img_str}',
            bounds=[[self.dem_data.lat_min, self.dem_data.lon_min],
                   [self.dem_data.lat_max, self.dem_data.lon_max]],
            opacity=0.5,
            name='Elevation',
            overlay=True,
            control=True
        ).add_to(m)
        
        colormap.add_to(m)
    
    def _add_risk_heatmap(self, m: folium.Map, risk_map: np.ndarray):
        """Add risk as heatmap overlay"""
        # Create heatmap data
        heat_data = []
        
        step = max(1, risk_map.shape[0] // 50)  # Subsample for performance
        
        for i in range(0, risk_map.shape[0], step):
            for j in range(0, risk_map.shape[1], step):
                risk = risk_map[i, j]
                if risk > 0.3:  # Only show significant risk
                    lat = self.dem_data.lat_min + (i / risk_map.shape[0]) * \
                          (self.dem_data.lat_max - self.dem_data.lat_min)
                    lon = self.dem_data.lon_min + (j / risk_map.shape[1]) * \
                          (self.dem_data.lon_max - self.dem_data.lon_min)
                    heat_data.append([lat, lon, float(risk)])
        
        # Add heatmap layer
        plugins.HeatMap(
            heat_data,
            name='Risk Heatmap',
            min_opacity=0.3,
            max_val=1.0,
            radius=15,
            blur=20,
            gradient={
                0.0: 'green',
                0.5: 'yellow',
                0.7: 'orange',
                1.0: 'red'
            },
            overlay=True,
            control=True
        ).add_to(m)
    
    def _add_routes_to_folium(self, m: folium.Map, routes: List[FlightPath]):
        """Add flight routes to map"""
        colors = ['blue', 'green', 'purple', 'orange', 'red', 'cyan', 'magenta']
        
        for idx, route in enumerate(routes):
            color = colors[idx % len(colors)]
            
            # Extract coordinates
            coords = []
            for wp in route.waypoints:
                lat, lon = self._grid_to_latlon(wp.x, wp.y)
                coords.append([lat, lon])
            
            # Add route line
            folium.PolyLine(
                coords,
                color=color,
                weight=3,
                opacity=0.8,
                popup=f'Route {idx+1}<br>Distance: {route.total_distance:.0f}m<br>Risk: {route.total_risk:.3f}',
                tooltip=f'Route {idx+1}'
            ).add_to(m)
            
            # Add start/end markers
            folium.CircleMarker(
                coords[0],
                radius=8,
                color=color,
                fill=True,
                fill_color='white',
                popup=f'Route {idx+1} Start',
                tooltip='Start'
            ).add_to(m)
            
            folium.CircleMarker(
                coords[-1],
                radius=8,
                color=color,
                fill=True,
                fill_color='black',
                popup=f'Route {idx+1} End',
                tooltip='End'
            ).add_to(m)
    
    def _add_vertiports_to_folium(self, m: folium.Map, vertiports: List[SafeZone]):
        """Add vertiport markers to map"""
        # Create feature group for vertiports
        vp_group = folium.FeatureGroup(name='Vertiports', show=True)
        
        for idx, vp in enumerate(vertiports):
            # Create custom icon
            icon_html = f'''
            <div style="
                background-color: #28a745;
                border: 3px solid white;
                border-radius: 50%;
                width: 30px;
                height: 30px;
                display: flex;
                align-items: center;
                justify-content: center;
                font-weight: bold;
                color: white;
                font-size: 12px;
                box-shadow: 0 2px 5px rgba(0,0,0,0.3);
            ">
                {idx+1}
            </div>
            '''
            
            # Popup HTML
            popup_html = f'''
            <div style="font-family: Arial; font-size: 12px;">
                <h4 style="margin: 0 0 10px 0; color: #28a745;">🚁 Vertiport VP-{idx+1}</h4>
                <table style="width: 100%;">
                    <tr><td><b>Location:</b></td><td>{vp.lat:.5f}°N, {vp.lon:.5f}°E</td></tr>
                    <tr><td><b>Elevation:</b></td><td>{vp.elevation:.1f} m</td></tr>
                    <tr><td><b>Total Score:</b></td><td>{vp.total_score:.3f}</td></tr>
                    <tr><td><b>Safety:</b></td><td>{vp.safety_score:.3f}</td></tr>
                    <tr><td><b>Flatness:</b></td><td>{vp.flatness_score:.3f}</td></tr>
                    <tr><td><b>Clearance:</b></td><td>{vp.clearance:.1f} m</td></tr>
                    <tr><td><b>Coverage:</b></td><td>{vp.route_coverage} routes</td></tr>
                    <tr><td><b>Area:</b></td><td>{vp.area:.0f} m²</td></tr>
                </table>
            </div>
            '''
            
            # Add marker
            folium.Marker(
                location=[vp.lat, vp.lon],
                popup=folium.Popup(popup_html, max_width=300),
                tooltip=f'VP-{idx+1} (Score: {vp.total_score:.2f})',
                icon=folium.DivIcon(html=icon_html)
            ).add_to(vp_group)
            
            # Add safe zone circle
            folium.Circle(
                location=[vp.lat, vp.lon],
                radius=np.sqrt(vp.area / np.pi),  # Radius from area
                color='green',
                fill=True,
                fill_color='green',
                fill_opacity=0.2,
                weight=2,
                popup=f'Safe Zone {idx+1}'
            ).add_to(vp_group)
        
        vp_group.add_to(m)
    
    def create_3d_plotly_map(self,
                            routes: Optional[List[FlightPath]] = None,
                            vertiports: Optional[List[SafeZone]] = None) -> go.Figure:
        """
        Create 3D Plotly visualization with geographic coordinates
        
        Shows terrain elevation and flight paths in 3D space
        """
        fig = go.Figure()
        
        # Create lat/lon grids
        lats = np.linspace(self.dem_data.lat_min, self.dem_data.lat_max, 
                          self.dem_data.shape[0])
        lons = np.linspace(self.dem_data.lon_min, self.dem_data.lon_max,
                          self.dem_data.shape[1])
        
        lon_grid, lat_grid = np.meshgrid(lons, lats)
        
        # Add terrain surface
        fig.add_trace(go.Surface(
            x=lon_grid,
            y=lat_grid,
            z=self.dem_data.elevation,
            colorscale='Viridis',
            opacity=0.7,
            name='Terrain',
            showscale=True,
            colorbar=dict(title='Elevation (m)', x=1.15)
        ))
        
        # Add routes
        if routes is not None:
            colors = ['blue', 'red', 'green', 'purple', 'orange']
            for idx, route in enumerate(routes):
                lats_route = []
                lons_route = []
                elevs_route = []
                
                for wp in route.waypoints:
                    lat, lon = self._grid_to_latlon(wp.x, wp.y)
                    lats_route.append(lat)
                    lons_route.append(lon)
                    elevs_route.append(wp.z)
                
                fig.add_trace(go.Scatter3d(
                    x=lons_route,
                    y=lats_route,
                    z=elevs_route,
                    mode='lines+markers',
                    line=dict(color=colors[idx % len(colors)], width=4),
                    marker=dict(size=3),
                    name=f'Route {idx+1}'
                ))
        
        # Add vertiports
        if vertiports is not None:
            lats_vp = [vp.lat for vp in vertiports]
            lons_vp = [vp.lon for vp in vertiports]
            elevs_vp = [vp.elevation + 10 for vp in vertiports]  # Slightly above ground
            scores = [vp.total_score for vp in vertiports]
            
            fig.add_trace(go.Scatter3d(
                x=lons_vp,
                y=lats_vp,
                z=elevs_vp,
                mode='markers+text',
                marker=dict(
                    size=15,
                    color=scores,
                    colorscale='Greens',
                    showscale=True,
                    colorbar=dict(title='Score', x=0.85),
                    line=dict(color='white', width=2),
                    symbol='diamond'
                ),
                text=[f'VP-{i+1}' for i in range(len(vertiports))],
                textposition='top center',
                name='Vertiports'
            ))
        
        # Update layout
        fig.update_layout(
            title='UAM System - Geographic 3D View',
            scene=dict(
                xaxis_title='Longitude',
                yaxis_title='Latitude',
                zaxis_title='Elevation (m)',
                aspectmode='manual',
                aspectratio=dict(x=1, y=1, z=0.3)
            ),
            width=1200,
            height=800
        )
        
        return fig
    
    def _grid_to_latlon(self, x: float, y: float) -> Tuple[float, float]:
        """Convert grid coordinates to lat/lon"""
        lon = self.dem_data.lon_min + (x / self.dem_data.shape[1]) * \
              (self.dem_data.lon_max - self.dem_data.lon_min)
        lat = self.dem_data.lat_min + (y / self.dem_data.shape[0]) * \
              (self.dem_data.lat_max - self.dem_data.lat_min)
        return (lat, lon)
    
    def save_map(self, m: folium.Map, filename: str):
        """Save Folium map to HTML file"""
        m.save(filename)
        print(f"Map saved to {filename}")
