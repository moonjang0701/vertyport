"""
Simple test script to verify the UAM system
"""

import sys
import numpy as np
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent))

from src.data_processing.dem_processor import DEMProcessor
from src.path_planning.path_planner import PathPlanner3D
from src.vertiport_optimization.optimizer import VertiportOptimizer
from src.visualization.visualizer import Visualizer3D


def test_system():
    """Test the complete UAM system"""
    print("🚁 UAM Vertiport Optimization System Test")
    print("=" * 60)
    
    # 1. Generate synthetic terrain
    print("\n1️⃣ Generating synthetic urban terrain...")
    processor = DEMProcessor(resolution=10.0)
    dem_data = processor.create_synthetic_urban_dem(
        width=100, height=100, num_buildings=20
    )
    print(f"   ✅ Terrain generated: {dem_data.shape}")
    print(f"   📏 Elevation range: {dem_data.min_elevation:.1f}m - {dem_data.max_elevation:.1f}m")
    
    # 2. Create risk map
    print("\n2️⃣ Creating risk map...")
    obstacles = processor.identify_obstacles(dem_data, height_threshold=20.0)
    population = np.random.rand(100, 100) * 100
    risk_map = processor.create_risk_map(dem_data, population, obstacles)
    print(f"   ✅ Risk map created")
    print(f"   ⚠️  Average risk: {np.mean(risk_map):.3f}")
    print(f"   🔴 High risk areas: {(risk_map > 0.7).sum() / risk_map.size * 100:.1f}%")
    
    # 3. Optimize vertiport locations
    print("\n3️⃣ Optimizing vertiport locations...")
    optimizer = VertiportOptimizer(dem_data, risk_map, population)
    vertiports = optimizer.find_optimal_locations(
        num_vertiports=5,
        min_distance=200.0,
        search_resolution=5
    )
    print(f"   ✅ Found {len(vertiports)} optimal locations")
    
    for i, vp in enumerate(vertiports):
        print(f"   🚁 VP-{i+1}: Score={vp.score:.3f}, "
              f"Safety={vp.safety_score:.3f}, "
              f"Access={vp.accessibility_score:.3f}, "
              f"Connect={vp.connectivity_score:.3f}")
    
    # 4. Plan flight path
    print("\n4️⃣ Planning flight path...")
    if len(vertiports) >= 2:
        planner = PathPlanner3D(
            dem_data, risk_map,
            min_altitude=50.0,
            max_altitude=150.0
        )
        
        start_vp = vertiports[0]
        goal_vp = vertiports[1]
        
        path = planner.plan_path(
            (start_vp.x, start_vp.y, start_vp.z + 50),
            (goal_vp.x, goal_vp.y, goal_vp.z + 50),
            num_altitude_levels=5
        )
        
        if path:
            print(f"   ✅ Path planned successfully")
            print(f"   📏 Total distance: {path.total_distance:.1f}m")
            print(f"   ⚠️  Total risk: {path.total_risk:.3f}")
            print(f"   📍 Waypoints: {len(path.waypoints)}")
            
            # Smooth path
            smoothed_path = planner.smooth_path(path)
            print(f"   🎨 Path smoothed")
        else:
            print(f"   ❌ No valid path found")
    
    # 5. Create visualizations
    print("\n5️⃣ Creating visualizations...")
    visualizer = Visualizer3D(dem_data)
    
    # Create complete visualization
    fig = visualizer.create_complete_visualization(
        risk_map=risk_map,
        paths=[smoothed_path] if path else None,
        vertiports=vertiports,
        title="UAM System Test - Fig 3 Style"
    )
    
    # Save as HTML
    output_file = "test_visualization.html"
    fig.write_html(output_file)
    print(f"   ✅ Visualization saved to {output_file}")
    
    print("\n" + "=" * 60)
    print("✅ All tests completed successfully!")
    print(f"📊 Open {output_file} in your browser to view results")
    
    return dem_data, risk_map, vertiports, smoothed_path if path else None


if __name__ == "__main__":
    test_system()
