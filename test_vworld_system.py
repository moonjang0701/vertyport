"""
Comprehensive Test Script for V-World Based UAM System
Tests the complete pipeline with real map visualization
"""

import sys
import numpy as np
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from src.data_processing.vworld_loader import VWorldDEMLoader, SEOUL_GANGNAM
from src.data_processing.dem_processor import DEMProcessor
from src.vertiport_optimization.route_based_extractor import RouteBasedVertiportExtractor
from src.visualization.real_map_visualizer import RealMapVisualizer
from src.evaluation.performance_evaluator import PerformanceEvaluator


def main():
    """Run comprehensive system test"""
    print("="*80)
    print("🚁 UAM VERTIPORT OPTIMIZATION SYSTEM - COMPREHENSIVE TEST")
    print("Based on V-World DEM Data with Real Map Visualization")
    print("="*80)
    
    start_time = time.time()
    
    # Step 1: Load V-World DEM Data
    print("\n📡 Step 1: Loading V-World DEM Data...")
    print("-" * 80)
    
    loader = VWorldDEMLoader(api_key="YOUR_API_KEY")
    
    # Test with Seoul Gangnam area
    dem_data = loader.load_dem_region(
        lat_min=SEOUL_GANGNAM['lat_min'],
        lat_max=SEOUL_GANGNAM['lat_max'],
        lon_min=SEOUL_GANGNAM['lon_min'],
        lon_max=SEOUL_GANGNAM['lon_max'],
        resolution=256
    )
    
    print(f"✅ DEM Data Loaded:")
    print(f"   • Region: Seoul Gangnam")
    print(f"   • Bounds: ({dem_data.lat_min:.4f}, {dem_data.lon_min:.4f}) to "
          f"({dem_data.lat_max:.4f}, {dem_data.lon_max:.4f})")
    print(f"   • Shape: {dem_data.shape}")
    print(f"   • Elevation range: {np.min(dem_data.elevation):.1f}m - "
          f"{np.max(dem_data.elevation):.1f}m")
    
    # Step 2: Load Population and Create Risk Map
    print("\n🗺️  Step 2: Creating Risk Map...")
    print("-" * 80)
    
    population = loader.load_population_density(
        dem_data.lat_min, dem_data.lat_max,
        dem_data.lon_min, dem_data.lon_max,
        resolution=256
    )
    
    processor = DEMProcessor(resolution=dem_data.resolution)
    obstacles = processor.identify_obstacles(dem_data, height_threshold=20.0)
    risk_map = processor.create_risk_map(dem_data, population, obstacles)
    
    print(f"✅ Risk Map Created:")
    print(f"   • Average risk: {np.mean(risk_map):.3f}")
    print(f"   • Maximum risk: {np.max(risk_map):.3f}")
    print(f"   • High-risk areas: {(risk_map > 0.7).sum() / risk_map.size * 100:.1f}%")
    
    # Step 3: Extract Vertiports from Routes
    print("\n🎯 Step 3: Route-Based Vertiport Extraction...")
    print("-" * 80)
    
    # Define origin and destination (Gangnam Station to COEX)
    origin = (37.4979, 127.0276)  # Gangnam Station
    destination = (37.5126, 127.0588)  # COEX
    
    print(f"   • Origin: Gangnam Station {origin}")
    print(f"   • Destination: COEX {destination}")
    print(f"   • Distance: ~{RouteBasedVertiportExtractor._haversine_distance(*origin, *destination):.0f}m")
    
    extractor = RouteBasedVertiportExtractor(
        dem_data=dem_data,
        risk_map=risk_map,
        population_density=population
    )
    
    route_start = time.time()
    vertiports = extractor.extract_vertiports_from_routes(
        origin=origin,
        destination=destination,
        num_routes=5,
        num_vertiports=8
    )
    route_time = time.time() - route_start
    
    print(f"\n✅ Vertiport Extraction Complete:")
    print(f"   • Extracted {len(vertiports)} optimal vertiports")
    print(f"   • Computation time: {route_time:.2f}s")
    
    print("\n📍 Top 5 Vertiports:")
    for i, vp in enumerate(vertiports[:5]):
        print(f"   VP-{i+1}: Score={vp.total_score:.3f}, "
              f"Safety={vp.safety_score:.3f}, "
              f"Flatness={vp.flatness_score:.3f}, "
              f"Coverage={vp.route_coverage} routes")
    
    # Step 4: Performance Evaluation
    print("\n📊 Step 4: Performance Evaluation...")
    print("-" * 80)
    
    evaluator = PerformanceEvaluator(dem_data, risk_map)
    
    # Note: For full evaluation, we need to regenerate routes
    # For this demo, we'll evaluate the vertiport network
    
    vp_metrics = evaluator.evaluate_vertiport_network(vertiports)
    
    print(f"\n✅ Vertiport Network Metrics:")
    print(f"   • Number of vertiports: {vp_metrics.num_vertiports}")
    print(f"   • Average safety score: {vp_metrics.average_safety_score:.3f}")
    print(f"   • Average flatness score: {vp_metrics.average_flatness_score:.3f}")
    print(f"   • Average total score: {vp_metrics.average_total_score:.3f}")
    print(f"   • Minimum spacing: {vp_metrics.min_spacing:.1f}m")
    print(f"   • Average spacing: {vp_metrics.average_spacing:.1f}m")
    print(f"   • Coverage area: {vp_metrics.coverage_area:.3f} km²")
    print(f"   • Network connectivity: {vp_metrics.network_connectivity:.3f}")
    print(f"   • Spatial uniformity: {vp_metrics.spatial_uniformity:.3f}")
    
    # Step 5: Create Visualizations
    print("\n🎨 Step 5: Creating Visualizations...")
    print("-" * 80)
    
    visualizer = RealMapVisualizer(dem_data)
    
    # Create Folium map
    print("   • Creating interactive Folium map...")
    folium_map = visualizer.create_folium_map(
        routes=None,  # Would include routes if available
        vertiports=vertiports,
        risk_map=risk_map,
        show_3d_buildings=True
    )
    
    folium_output = "vworld_uam_map.html"
    visualizer.save_map(folium_map, folium_output)
    print(f"   ✅ Folium map saved: {folium_output}")
    
    # Create 3D Plotly visualization
    print("   • Creating 3D Plotly visualization...")
    plotly_fig = visualizer.create_3d_plotly_map(
        routes=None,
        vertiports=vertiports
    )
    
    plotly_output = "vworld_uam_3d.html"
    plotly_fig.write_html(plotly_output)
    print(f"   ✅ 3D visualization saved: {plotly_output}")
    
    # Step 6: Summary
    total_time = time.time() - start_time
    
    print("\n" + "="*80)
    print("✅ TEST COMPLETED SUCCESSFULLY!")
    print("="*80)
    
    print(f"\n📈 Summary Statistics:")
    print(f"   • Total execution time: {total_time:.2f}s")
    print(f"   • DEM resolution: {dem_data.shape[0]}×{dem_data.shape[1]}")
    print(f"   • Area covered: ~{vp_metrics.coverage_area:.2f} km²")
    print(f"   • Vertiports extracted: {len(vertiports)}")
    print(f"   • Average vertiport score: {vp_metrics.average_total_score:.3f}")
    
    print(f"\n📁 Output Files:")
    print(f"   • Interactive map: {folium_output}")
    print(f"   • 3D visualization: {plotly_output}")
    
    print(f"\n💡 Next Steps:")
    print(f"   1. Open {folium_output} in your browser to explore the interactive map")
    print(f"   2. Open {plotly_output} to view the 3D terrain and vertiport network")
    print(f"   3. Run the Streamlit app for full system demonstration:")
    print(f"      streamlit run app_vworld.py")
    
    print("\n" + "="*80)
    
    return dem_data, risk_map, vertiports


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
