"""
Streamlit Web Interface for UAM Vertiport Optimization System
"""

import streamlit as st
import numpy as np
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent))

from src.data_processing.dem_processor import DEMProcessor, DEMData
from src.path_planning.path_planner import PathPlanner3D, Waypoint
from src.vertiport_optimization.optimizer import VertiportOptimizer
from src.visualization.visualizer import Visualizer3D


# Page configuration
st.set_page_config(
    page_title="UAM Vertiport Optimizer",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded"
)

# Custom CSS
st.markdown("""
<style>
    .main-header {
        font-size: 2.5rem;
        font-weight: bold;
        color: #1f77b4;
        text-align: center;
        margin-bottom: 1rem;
    }
    .sub-header {
        font-size: 1.2rem;
        color: #666;
        text-align: center;
        margin-bottom: 2rem;
    }
    .metric-card {
        background-color: #f0f2f6;
        padding: 1rem;
        border-radius: 0.5rem;
        margin: 0.5rem 0;
    }
</style>
""", unsafe_allow_html=True)


def initialize_session_state():
    """Initialize session state variables"""
    if 'dem_data' not in st.session_state:
        st.session_state.dem_data = None
    if 'risk_map' not in st.session_state:
        st.session_state.risk_map = None
    if 'vertiports' not in st.session_state:
        st.session_state.vertiports = None
    if 'paths' not in st.session_state:
        st.session_state.paths = None


def generate_synthetic_data(width, height, num_buildings, population_enabled):
    """Generate synthetic urban DEM data"""
    with st.spinner('🏗️ Generating synthetic urban environment...'):
        processor = DEMProcessor(resolution=10.0)
        dem_data = processor.create_synthetic_urban_dem(
            width=width,
            height=height,
            num_buildings=num_buildings
        )
        
        # Create risk map
        obstacles = processor.identify_obstacles(dem_data, height_threshold=20.0)
        
        if population_enabled:
            # Generate synthetic population density
            population = np.random.rand(height, width) * 100
            population = np.maximum(population - 50, 0)  # Sparse population
        else:
            population = None
        
        risk_map = processor.create_risk_map(dem_data, population, obstacles)
        
        return dem_data, risk_map


def main():
    """Main application"""
    initialize_session_state()
    
    # Header
    st.markdown('<div class="main-header">🚁 UAM Vertiport Optimization System</div>', 
                unsafe_allow_html=True)
    st.markdown('<div class="sub-header">Based on IEEE MAES - Advanced UTM Services for Urban Air Mobility</div>', 
                unsafe_allow_html=True)
    
    # Sidebar
    with st.sidebar:
        st.header("⚙️ Configuration")
        
        st.subheader("1️⃣ Terrain Settings")
        terrain_width = st.slider("Terrain Width (cells)", 50, 200, 100)
        terrain_height = st.slider("Terrain Height (cells)", 50, 200, 100)
        num_buildings = st.slider("Number of Buildings", 10, 50, 20)
        population_enabled = st.checkbox("Enable Population Density", value=True)
        
        if st.button("🌍 Generate Terrain", use_container_width=True):
            dem_data, risk_map = generate_synthetic_data(
                terrain_width, terrain_height, num_buildings, population_enabled
            )
            st.session_state.dem_data = dem_data
            st.session_state.risk_map = risk_map
            st.success("✅ Terrain generated successfully!")
        
        st.divider()
        
        if st.session_state.dem_data is not None:
            st.subheader("2️⃣ Vertiport Optimization")
            num_vertiports = st.slider("Number of Vertiports", 3, 10, 5)
            min_distance = st.slider("Minimum Distance (m)", 100, 500, 200)
            
            # Weight adjustment
            st.write("**Score Weights:**")
            weight_accessibility = st.slider("Accessibility", 0.0, 1.0, 0.35, 0.05)
            weight_safety = st.slider("Safety", 0.0, 1.0, 0.35, 0.05)
            weight_connectivity = st.slider("Connectivity", 0.0, 1.0, 0.30, 0.05)
            
            # Normalize weights
            total_weight = weight_accessibility + weight_safety + weight_connectivity
            if total_weight > 0:
                weight_accessibility /= total_weight
                weight_safety /= total_weight
                weight_connectivity /= total_weight
            
            if st.button("🎯 Optimize Vertiport Locations", use_container_width=True):
                with st.spinner('🔍 Finding optimal locations...'):
                    optimizer = VertiportOptimizer(
                        st.session_state.dem_data,
                        st.session_state.risk_map
                    )
                    optimizer.weights = {
                        'accessibility': weight_accessibility,
                        'safety': weight_safety,
                        'connectivity': weight_connectivity
                    }
                    
                    vertiports = optimizer.find_optimal_locations(
                        num_vertiports=num_vertiports,
                        min_distance=min_distance,
                        search_resolution=5
                    )
                    st.session_state.vertiports = vertiports
                    st.success(f"✅ Found {len(vertiports)} optimal locations!")
            
            st.divider()
            
            if st.session_state.vertiports is not None:
                st.subheader("3️⃣ Path Planning")
                st.write("Select start and goal vertiports:")
                
                vp_options = [f"VP-{i+1}" for i in range(len(st.session_state.vertiports))]
                
                start_vp_idx = st.selectbox("Start Vertiport", range(len(vp_options)), 
                                           format_func=lambda x: vp_options[x])
                goal_vp_idx = st.selectbox("Goal Vertiport", range(len(vp_options)), 
                                          format_func=lambda x: vp_options[x],
                                          index=min(1, len(vp_options)-1))
                
                min_altitude = st.slider("Min Flight Altitude (m)", 30, 100, 50)
                max_altitude = st.slider("Max Flight Altitude (m)", 100, 200, 150)
                
                if st.button("🛩️ Plan Flight Path", use_container_width=True):
                    if start_vp_idx != goal_vp_idx:
                        with st.spinner('🛤️ Planning optimal path...'):
                            start_vp = st.session_state.vertiports[start_vp_idx]
                            goal_vp = st.session_state.vertiports[goal_vp_idx]
                            
                            planner = PathPlanner3D(
                                st.session_state.dem_data,
                                st.session_state.risk_map,
                                min_altitude=min_altitude,
                                max_altitude=max_altitude
                            )
                            
                            path = planner.plan_path(
                                (start_vp.x, start_vp.y, start_vp.z + min_altitude),
                                (goal_vp.x, goal_vp.y, goal_vp.z + min_altitude),
                                num_altitude_levels=5
                            )
                            
                            if path:
                                # Smooth the path
                                path = planner.smooth_path(path, smoothing_factor=0.3)
                                st.session_state.paths = [path]
                                st.success(f"✅ Path planned! Distance: {path.total_distance:.1f}m")
                            else:
                                st.error("❌ No valid path found!")
                    else:
                        st.warning("⚠️ Start and goal must be different!")
    
    # Main content area
    if st.session_state.dem_data is None:
        st.info("👈 Start by generating terrain data in the sidebar")
        
        # Show example image/description
        col1, col2, col3 = st.columns(3)
        
        with col1:
            st.markdown("### 🏙️ Urban Terrain")
            st.write("Generate synthetic urban environment with buildings and terrain variations")
        
        with col2:
            st.markdown("### 🎯 Vertiport Optimization")
            st.write("Find optimal locations based on accessibility, safety, and connectivity")
        
        with col3:
            st.markdown("### 🛩️ Path Planning")
            st.write("Plan safe 3D flight paths avoiding obstacles and high-risk areas")
        
    else:
        # Create tabs for different views
        tab1, tab2, tab3, tab4 = st.tabs([
            "📊 Overview", "🗺️ 3D Visualization", "📈 Risk Analysis", "📋 Vertiport Details"
        ])
        
        with tab1:
            st.header("System Overview")
            
            col1, col2, col3, col4 = st.columns(4)
            
            with col1:
                st.metric(
                    "Terrain Size",
                    f"{st.session_state.dem_data.shape[1]} × {st.session_state.dem_data.shape[0]}"
                )
            
            with col2:
                st.metric(
                    "Elevation Range",
                    f"{st.session_state.dem_data.min_elevation:.1f} - {st.session_state.dem_data.max_elevation:.1f}m"
                )
            
            with col3:
                if st.session_state.vertiports:
                    st.metric("Vertiports", len(st.session_state.vertiports))
                else:
                    st.metric("Vertiports", "Not optimized")
            
            with col4:
                if st.session_state.paths:
                    st.metric("Flight Paths", len(st.session_state.paths))
                else:
                    st.metric("Flight Paths", "Not planned")
            
            st.divider()
            
            # Quick stats
            if st.session_state.vertiports:
                st.subheader("Vertiport Network Statistics")
                
                col1, col2 = st.columns(2)
                
                with col1:
                    avg_score = np.mean([vp.score for vp in st.session_state.vertiports])
                    avg_safety = np.mean([vp.safety_score for vp in st.session_state.vertiports])
                    avg_access = np.mean([vp.accessibility_score for vp in st.session_state.vertiports])
                    
                    st.metric("Average Overall Score", f"{avg_score:.3f}")
                    st.metric("Average Safety Score", f"{avg_safety:.3f}")
                    st.metric("Average Accessibility Score", f"{avg_access:.3f}")
                
                with col2:
                    avg_connectivity = np.mean([vp.connectivity_score for vp in st.session_state.vertiports])
                    total_coverage = sum([vp.coverage_area for vp in st.session_state.vertiports])
                    
                    st.metric("Average Connectivity Score", f"{avg_connectivity:.3f}")
                    st.metric("Total Coverage Area", f"{total_coverage/1e6:.2f} km²")
        
        with tab2:
            st.header("3D Visualization")
            
            if st.session_state.dem_data:
                visualizer = Visualizer3D(st.session_state.dem_data)
                
                # Create visualization
                fig = visualizer.create_complete_visualization(
                    risk_map=st.session_state.risk_map,
                    paths=st.session_state.paths,
                    vertiports=st.session_state.vertiports,
                    title="UAM Path Planning & Vertiport Optimization (Fig 3 Style)"
                )
                
                st.plotly_chart(fig, use_container_width=True)
                
                # Elevation profile
                if st.session_state.paths:
                    st.subheader("Flight Path Elevation Profile")
                    for i, path in enumerate(st.session_state.paths):
                        profile_fig = visualizer.create_elevation_profile(path)
                        st.plotly_chart(profile_fig, use_container_width=True)
        
        with tab3:
            st.header("Risk Analysis")
            
            if st.session_state.risk_map is not None:
                visualizer = Visualizer3D(st.session_state.dem_data)
                
                # 2D heatmap
                heatmap_fig = visualizer.create_2d_risk_heatmap(
                    st.session_state.risk_map,
                    st.session_state.vertiports
                )
                st.plotly_chart(heatmap_fig, use_container_width=True)
                
                # Risk statistics
                col1, col2, col3 = st.columns(3)
                
                with col1:
                    st.metric("Average Risk", f"{np.mean(st.session_state.risk_map):.3f}")
                
                with col2:
                    st.metric("Maximum Risk", f"{np.max(st.session_state.risk_map):.3f}")
                
                with col3:
                    high_risk_percentage = (st.session_state.risk_map > 0.7).sum() / st.session_state.risk_map.size * 100
                    st.metric("High Risk Area", f"{high_risk_percentage:.1f}%")
        
        with tab4:
            st.header("Vertiport Details")
            
            if st.session_state.vertiports:
                for i, vp in enumerate(st.session_state.vertiports):
                    with st.expander(f"🚁 Vertiport VP-{i+1} (Score: {vp.score:.3f})"):
                        col1, col2 = st.columns(2)
                        
                        with col1:
                            st.write("**Location:**")
                            st.write(f"- X: {vp.x:.1f} m")
                            st.write(f"- Y: {vp.y:.1f} m")
                            st.write(f"- Z: {vp.z:.1f} m")
                            st.write(f"- Coverage: {vp.coverage_area/1000:.1f} km²")
                        
                        with col2:
                            st.write("**Scores:**")
                            st.write(f"- Overall: {vp.score:.3f}")
                            st.write(f"- Safety: {vp.safety_score:.3f}")
                            st.write(f"- Accessibility: {vp.accessibility_score:.3f}")
                            st.write(f"- Connectivity: {vp.connectivity_score:.3f}")
            else:
                st.info("No vertiports optimized yet. Use the sidebar to optimize locations.")
    
    # Footer
    st.divider()
    st.markdown("""
    <div style='text-align: center; color: #666; padding: 1rem;'>
        <p><strong>UAM Vertiport Optimization System</strong></p>
        <p>Based on: IEEE MAES - A Holistic Design and Simulation of Advanced UTM Services for Urban Air Mobility</p>
        <p>Fig 3: Operation trajectories generated in 3D designated airspace with constraints applied</p>
    </div>
    """, unsafe_allow_html=True)


if __name__ == "__main__":
    main()
