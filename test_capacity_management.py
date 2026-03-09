"""
Test Dynamic Capacity Management (Equation 2)
관악구에서 다중 UAM 운항 시뮬레이션 및 혼잡도 관리
"""

import numpy as np
import sys
from pathlib import Path

src_path = Path(__file__).parent / "src"
sys.path.insert(0, str(src_path))

from capacity.dynamic_capacity_manager import (
    DynamicCapacityManager,
    FlightOperation
)

def main():
    print("=" * 80)
    print("Dynamic Capacity Management Test - IEEE MAES Equation 2")
    print("=" * 80)
    
    # Define Gwanak-gu bounds
    bounds = {
        'lat_min': 37.46,
        'lat_max': 37.49,
        'lon_min': 126.93,
        'lon_max': 126.98
    }
    
    print(f"\n📍 Area: Gwanak-gu")
    print(f"   Bounds: lat {bounds['lat_min']:.2f}-{bounds['lat_max']:.2f}, "
          f"lon {bounds['lon_min']:.2f}-{bounds['lon_max']:.2f}")
    
    # Initialize capacity manager
    print("\n" + "=" * 80)
    print("🏗️  Initializing Airspace Grid")
    print("=" * 80)
    
    manager = DynamicCapacityManager(
        bounds=bounds,
        cell_size_m=500.0,  # 500m × 500m cells
        altitude_layers=5,  # 5 altitude levels (50-250m)
        base_altitude=50.0,
        altitude_step=50.0,
        max_capacity_per_cell=3  # Max 3 aircraft per cell
    )
    
    print(f"✅ Airspace grid created:")
    print(f"   - Grid size: {manager.grid_x} × {manager.grid_y} × {manager.altitude_layers}")
    print(f"   - Total cells: {manager.grid_x * manager.grid_y * manager.altitude_layers}")
    print(f"   - Cell size: {manager.cell_size_m}m × {manager.cell_size_m}m")
    print(f"   - Altitude range: {manager.base_altitude}m - {manager.base_altitude + manager.altitude_layers * manager.altitude_step}m")
    print(f"   - Max capacity per cell: {manager.max_capacity}")
    
    # Create sample flight operations
    print("\n" + "=" * 80)
    print("✈️  Creating Flight Operations")
    print("=" * 80)
    
    # Operation 1: West to East (across district)
    op1_waypoints = [
        (37.475, 126.935, 100.0),
        (37.477, 126.945, 100.0),
        (37.478, 126.955, 100.0),
        (37.480, 126.965, 100.0)
    ]
    
    op1 = FlightOperation(
        operation_id=1,
        waypoints=op1_waypoints,
        submission_time=0.0,  # First to submit
        original_duration=180.0  # 3 minutes
    )
    
    # Operation 2: South to North (parallel path)
    op2_waypoints = [
        (37.465, 126.945, 100.0),
        (37.470, 126.948, 100.0),
        (37.475, 126.950, 100.0),
        (37.480, 126.952, 100.0)
    ]
    
    op2 = FlightOperation(
        operation_id=2,
        waypoints=op2_waypoints,
        submission_time=10.0,  # 10 seconds later
        original_duration=200.0
    )
    
    # Operation 3: Diagonal crossing (potential conflict)
    op3_waypoints = [
        (37.465, 126.965, 100.0),
        (37.472, 126.955, 100.0),
        (37.478, 126.945, 100.0),
        (37.485, 126.935, 100.0)
    ]
    
    op3 = FlightOperation(
        operation_id=3,
        waypoints=op3_waypoints,
        submission_time=20.0,
        original_duration=220.0
    )
    
    # Operation 4: Similar to op1 (creates congestion)
    op4_waypoints = [
        (37.476, 126.936, 100.0),
        (37.477, 126.946, 100.0),
        (37.478, 126.956, 100.0),
        (37.479, 126.966, 100.0)
    ]
    
    op4 = FlightOperation(
        operation_id=4,
        waypoints=op4_waypoints,
        submission_time=5.0,
        original_duration=190.0
    )
    
    # Operation 5: Another parallel (increase load)
    op5_waypoints = [
        (37.477, 126.937, 100.0),
        (37.478, 126.947, 100.0),
        (37.479, 126.957, 100.0),
        (37.480, 126.967, 100.0)
    ]
    
    op5 = FlightOperation(
        operation_id=5,
        waypoints=op5_waypoints,
        submission_time=12.0,
        original_duration=185.0
    )
    
    operations = [op1, op2, op3, op4, op5]
    
    print(f"Created {len(operations)} flight operations:")
    for op in operations:
        print(f"   - Operation {op.operation_id}: {len(op.waypoints)} waypoints, "
              f"duration {op.original_duration:.0f}s, submitted at t={op.submission_time:.0f}s")
    
    # Add operations to airspace
    print("\n" + "=" * 80)
    print("📊 Adding Operations to Airspace")
    print("=" * 80)
    
    for op in operations:
        manager.add_operation(op)
        cells = manager.get_trajectory_cells(op.waypoints)
        print(f"   Operation {op.operation_id}: passes through {len(cells)} cells")
    
    # Get initial statistics
    stats_before = manager.get_statistics()
    
    print(f"\n📈 Initial Airspace Statistics:")
    print(f"   - Total cells: {stats_before['total_cells']}")
    print(f"   - Occupied cells: {stats_before['occupied_cells']} ({stats_before['occupancy_rate']*100:.1f}%)")
    print(f"   - Congested cells: {stats_before['full_cells']} ({stats_before['congestion_rate']*100:.1f}%)")
    print(f"   - Avg utilization: {stats_before['avg_utilization']*100:.1f}%")
    print(f"   - Max utilization: {stats_before['max_utilization']*100:.1f}%")
    print(f"   - Total operations: {stats_before['total_operations']}")
    
    # Identify hotspots
    print("\n" + "=" * 80)
    print("🔥 Identifying Congestion Hotspots")
    print("=" * 80)
    
    hotspots = manager.identify_hotspots(threshold=0.8)
    
    if hotspots:
        print(f"⚠️  Found {len(hotspots)} congested cells (>80% capacity):")
        for i, cell in enumerate(hotspots[:10], 1):  # Show first 10
            x, y, z = cell
            utilization = manager.airspace[cell].get_utilization()
            print(f"   {i}. Cell ({x},{y},{z}): {utilization*100:.0f}% utilization")
    else:
        print("✅ No congestion detected")
    
    # Optimize airspace
    print("\n" + "=" * 80)
    print("🎯 Optimizing Airspace (Equation 2)")
    print("=" * 80)
    
    assignments = manager.optimize_airspace()
    
    rerouted_count = sum(1 for traj_id in assignments.values() if traj_id > 0)
    
    print(f"\n✅ Optimization complete:")
    print(f"   - Total operations: {len(operations)}")
    print(f"   - Rerouted: {rerouted_count}")
    print(f"   - Kept original: {len(operations) - rerouted_count}")
    
    print(f"\n📋 Assignment Results:")
    for op_id, traj_id in assignments.items():
        if traj_id == 0:
            print(f"   - Operation {op_id}: Original route ✓")
        else:
            print(f"   - Operation {op_id}: Alternative route {traj_id} 🔄")
    
    # Calculate rerouting cost
    print("\n" + "=" * 80)
    print("💰 Rerouting Cost Analysis (Equation 2)")
    print("=" * 80)
    
    # Create alternatives dict for cost calculation
    alternatives_dict = {}
    for op in operations:
        if assignments[op.operation_id] > 0:
            # Generate alternative for cost calculation
            alt_waypoints = manager.generate_alternative_trajectory(
                op.waypoints,
                hotspots,
                offset_m=1000.0
            )
            
            if alt_waypoints:
                from capacity.dynamic_capacity_manager import AlternativeTrajectory
                
                original_dist = manager._calculate_path_length(op.waypoints)
                alt_dist = manager._calculate_path_length(alt_waypoints)
                additional_duration = (alt_dist - original_dist) / 60.0
                
                priority = 1.0 / (1.0 + op.submission_time)
                cost = priority * additional_duration
                
                alt_traj = AlternativeTrajectory(
                    trajectory_id=1,
                    waypoints=alt_waypoints,
                    additional_duration=additional_duration,
                    cost=cost
                )
                
                alternatives_dict[op.operation_id] = [alt_traj]
                op.selected_trajectory = 1
    
    # Calculate total cost
    if alternatives_dict:
        total_cost = manager.calculate_rerouting_cost(operations, alternatives_dict)
        
        print(f"Total Rerouting Cost (C_r): {total_cost:.2f}")
        print(f"\nCost Breakdown:")
        for op in operations:
            if op.operation_id in alternatives_dict:
                alt = alternatives_dict[op.operation_id][0]
                v_f = 1.0 / (1.0 + op.submission_time)
                print(f"   - Op {op.operation_id}: v_f={v_f:.3f}, d_f^k={alt.additional_duration:.1f}s, cost={alt.cost:.2f}")
    else:
        print("No rerouting needed - airspace is not congested")
    
    print("\n" + "=" * 80)
    print("✅ Dynamic Capacity Management Test Complete")
    print("=" * 80)
    
    print("\n💡 Summary:")
    print(f"   - Airspace: {manager.grid_x}×{manager.grid_y}×{manager.altitude_layers} cells")
    print(f"   - Operations: {len(operations)}")
    print(f"   - Hotspots: {len(hotspots)}")
    print(f"   - Rerouted: {rerouted_count}/{len(operations)}")
    if alternatives_dict:
        print(f"   - Total Cost: {total_cost:.2f}")

if __name__ == "__main__":
    main()
