"""
Dynamic Capacity Management - IEEE MAES Paper (Equation 2)
공역 혼잡도 관리 및 우회 경로 생성

Equation 2: C_r = Σ_{f∈F} Σ_{k∈K_f} v_f × d^k_f × z^k_f
- d^k_f: Additional flight duration for alternative trajectory k
- v_f: Operation priority (submission time)
- z^k_f: Boolean (1 if trajectory k is selected)
"""

import numpy as np
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
from collections import defaultdict
import heapq


@dataclass
class AirspaceCell:
    """3D airspace grid cell"""
    x: int  # Grid index x
    y: int  # Grid index y
    z: int  # Altitude level
    capacity: int = 1  # Max simultaneous operations
    current_load: int = 0  # Current traffic count
    
    def is_full(self) -> bool:
        return self.current_load >= self.capacity
    
    def get_utilization(self) -> float:
        return self.current_load / self.capacity if self.capacity > 0 else 0.0


@dataclass
class FlightOperation:
    """UAM flight operation"""
    operation_id: int
    waypoints: List[Tuple[float, float, float]]  # (lat, lon, alt)
    submission_time: float  # seconds (priority)
    original_duration: float  # seconds
    selected_trajectory: int = 0  # 0 = original


@dataclass
class AlternativeTrajectory:
    """Alternative route for congestion avoidance"""
    trajectory_id: int
    waypoints: List[Tuple[float, float, float]]
    additional_duration: float  # seconds (d^k_f)
    cost: float  # v_f × d^k_f


class DynamicCapacityManager:
    """
    Dynamic Capacity Management System
    논문 Equation 2 구현: 혼잡 완화 및 우회 경로 최적화
    """
    
    def __init__(self,
                 bounds: dict,
                 cell_size_m: float = 500.0,  # 500m × 500m cells
                 altitude_layers: int = 5,  # 50m per layer
                 base_altitude: float = 50.0,
                 altitude_step: float = 50.0,
                 max_capacity_per_cell: int = 3):  # Max 3 aircraft per cell
        """
        Initialize capacity manager
        
        Args:
            bounds: Geographic bounds {'lat_min', 'lat_max', 'lon_min', 'lon_max'}
            cell_size_m: Airspace cell size (meters)
            altitude_layers: Number of altitude levels
            base_altitude: Minimum altitude (m)
            altitude_step: Altitude increment per layer (m)
            max_capacity_per_cell: Maximum operations per cell
        """
        self.bounds = bounds
        self.cell_size_m = cell_size_m
        self.altitude_layers = altitude_layers
        self.base_altitude = base_altitude
        self.altitude_step = altitude_step
        self.max_capacity = max_capacity_per_cell
        
        # Calculate grid dimensions
        lat_range = bounds['lat_max'] - bounds['lat_min']
        lon_range = bounds['lon_max'] - bounds['lon_min']
        
        # Convert to meters (approximate)
        lat_m = lat_range * 111000  # 1 degree ≈ 111km
        lon_m = lon_range * 88000  # 1 degree ≈ 88km at mid-latitude
        
        self.grid_x = int(np.ceil(lon_m / cell_size_m))
        self.grid_y = int(np.ceil(lat_m / cell_size_m))
        
        # Initialize 3D airspace grid
        self.airspace = {}
        for x in range(self.grid_x):
            for y in range(self.grid_y):
                for z in range(altitude_layers):
                    cell_key = (x, y, z)
                    self.airspace[cell_key] = AirspaceCell(
                        x=x, y=y, z=z, 
                        capacity=max_capacity_per_cell
                    )
        
        # Flight operations
        self.operations: List[FlightOperation] = []
        self.cell_occupancy: Dict[Tuple, List[int]] = defaultdict(list)
    
    def latlon_to_grid(self, lat: float, lon: float, alt: float) -> Tuple[int, int, int]:
        """Convert lat/lon/alt to grid indices"""
        # Normalize to [0, 1]
        x_norm = (lon - self.bounds['lon_min']) / (self.bounds['lon_max'] - self.bounds['lon_min'])
        y_norm = (lat - self.bounds['lat_min']) / (self.bounds['lat_max'] - self.bounds['lat_min'])
        
        # Convert to grid indices
        x = int(x_norm * self.grid_x)
        y = int(y_norm * self.grid_y)
        z = int((alt - self.base_altitude) / self.altitude_step)
        
        # Clamp
        x = np.clip(x, 0, self.grid_x - 1)
        y = np.clip(y, 0, self.grid_y - 1)
        z = np.clip(z, 0, self.altitude_layers - 1)
        
        return x, y, z
    
    def get_trajectory_cells(self, waypoints: List[Tuple[float, float, float]]) -> List[Tuple[int, int, int]]:
        """Get all airspace cells intersected by trajectory"""
        cells = []
        
        for wp in waypoints:
            lat, lon, alt = wp
            cell = self.latlon_to_grid(lat, lon, alt)
            if cell not in cells:
                cells.append(cell)
        
        return cells
    
    def add_operation(self, operation: FlightOperation):
        """Add flight operation and update airspace occupancy"""
        self.operations.append(operation)
        op_id = operation.operation_id
        
        # Get trajectory cells
        cells = self.get_trajectory_cells(operation.waypoints)
        
        # Update occupancy
        for cell_key in cells:
            if cell_key in self.airspace:
                self.airspace[cell_key].current_load += 1
                self.cell_occupancy[cell_key].append(op_id)
    
    def identify_hotspots(self, threshold: float = 0.8) -> List[Tuple[int, int, int]]:
        """
        Identify congested airspace cells (hotspots)
        
        Args:
            threshold: Utilization threshold (0.8 = 80% capacity)
            
        Returns:
            List of congested cell keys
        """
        hotspots = []
        
        for cell_key, cell in self.airspace.items():
            if cell.get_utilization() >= threshold:
                hotspots.append(cell_key)
        
        return hotspots
    
    def calculate_rerouting_cost(self, 
                                 operations: List[FlightOperation],
                                 alternatives: Dict[int, List[AlternativeTrajectory]]) -> float:
        """
        Calculate total rerouting cost (Equation 2)
        C_r = Σ_{f∈F} Σ_{k∈K_f} v_f × d^k_f × z^k_f
        
        Args:
            operations: List of flight operations
            alternatives: Dict mapping operation_id to alternative trajectories
            
        Returns:
            Total rerouting cost
        """
        total_cost = 0.0
        
        for op in operations:
            op_id = op.operation_id
            
            if op_id in alternatives and op.selected_trajectory > 0:
                # Get selected alternative
                alt_traj = alternatives[op_id][op.selected_trajectory - 1]
                
                # Priority (earlier submission = higher priority = lower v_f)
                v_f = 1.0 / (1.0 + op.submission_time)
                
                # Additional duration
                d_f_k = alt_traj.additional_duration
                
                # z^k_f = 1 (selected)
                z_f_k = 1
                
                # Add to cost
                total_cost += v_f * d_f_k * z_f_k
        
        return total_cost
    
    def generate_alternative_trajectory(self,
                                       original: List[Tuple[float, float, float]],
                                       hotspot_cells: List[Tuple[int, int, int]],
                                       offset_m: float = 1000.0) -> Optional[List[Tuple[float, float, float]]]:
        """
        Generate alternative trajectory avoiding hotspots
        
        Args:
            original: Original waypoints
            hotspot_cells: Congested cells to avoid
            offset_m: Lateral offset distance (meters)
            
        Returns:
            Alternative waypoints or None
        """
        if len(original) < 2:
            return None
        
        # Keep first and last waypoints (논문: 첫/마지막 셀 제외)
        start = original[0]
        end = original[-1]
        
        # Create offset path (simple lateral shift)
        alternative = [start]
        
        # Offset middle waypoints
        for i in range(1, len(original) - 1):
            lat, lon, alt = original[i]
            
            # Simple offset (perpendicular to path direction)
            # In real implementation, use proper path planning
            lat_offset = offset_m / 111000.0  # Convert to degrees
            lon_offset = offset_m / 88000.0
            
            new_lat = lat + lat_offset
            new_lon = lon + lon_offset
            
            alternative.append((new_lat, new_lon, alt))
        
        alternative.append(end)
        
        return alternative
    
    def optimize_airspace(self) -> Dict[int, int]:
        """
        Optimize airspace utilization by assigning alternative trajectories
        
        Returns:
            Dict mapping operation_id to selected trajectory (0=original, 1+=alternative)
        """
        # Identify hotspots
        hotspots = self.identify_hotspots(threshold=0.8)
        
        if not hotspots:
            # No congestion
            return {op.operation_id: 0 for op in self.operations}
        
        print(f"⚠️  Identified {len(hotspots)} congested cells")
        
        # Generate alternatives for operations passing through hotspots
        alternatives = {}
        
        for op in self.operations:
            cells = self.get_trajectory_cells(op.waypoints)
            
            # Check if passes through hotspot
            if any(cell in hotspots for cell in cells):
                # Generate alternatives
                alt_waypoints = self.generate_alternative_trajectory(
                    op.waypoints, hotspots, offset_m=1000.0
                )
                
                if alt_waypoints:
                    # Calculate additional duration (simplified)
                    original_dist = self._calculate_path_length(op.waypoints)
                    alt_dist = self._calculate_path_length(alt_waypoints)
                    additional_duration = (alt_dist - original_dist) / 60.0  # Assume 60 m/s
                    
                    priority = 1.0 / (1.0 + op.submission_time)
                    cost = priority * additional_duration
                    
                    alt_traj = AlternativeTrajectory(
                        trajectory_id=1,
                        waypoints=alt_waypoints,
                        additional_duration=additional_duration,
                        cost=cost
                    )
                    
                    alternatives[op.operation_id] = [alt_traj]
        
        # Simple greedy selection: assign alternatives to minimize total cost
        assignments = {}
        
        for op in self.operations:
            if op.operation_id in alternatives:
                # Select alternative if cost is acceptable
                alt = alternatives[op.operation_id][0]
                if alt.cost < 100.0:  # Cost threshold
                    assignments[op.operation_id] = 1
                else:
                    assignments[op.operation_id] = 0
            else:
                assignments[op.operation_id] = 0
        
        return assignments
    
    def _calculate_path_length(self, waypoints: List[Tuple[float, float, float]]) -> float:
        """Calculate total path length in meters"""
        total = 0.0
        
        for i in range(1, len(waypoints)):
            lat1, lon1, _ = waypoints[i-1]
            lat2, lon2, _ = waypoints[i]
            
            dlat = (lat2 - lat1) * 111000
            dlon = (lon2 - lon1) * 88000
            
            total += np.sqrt(dlat**2 + dlon**2)
        
        return total
    
    def get_statistics(self) -> dict:
        """Get airspace utilization statistics"""
        total_cells = len(self.airspace)
        occupied_cells = sum(1 for cell in self.airspace.values() if cell.current_load > 0)
        full_cells = sum(1 for cell in self.airspace.values() if cell.is_full())
        
        utilizations = [cell.get_utilization() for cell in self.airspace.values()]
        avg_utilization = np.mean(utilizations) if utilizations else 0.0
        max_utilization = np.max(utilizations) if utilizations else 0.0
        
        return {
            'total_cells': total_cells,
            'occupied_cells': occupied_cells,
            'full_cells': full_cells,
            'occupancy_rate': occupied_cells / total_cells if total_cells > 0 else 0.0,
            'congestion_rate': full_cells / total_cells if total_cells > 0 else 0.0,
            'avg_utilization': avg_utilization,
            'max_utilization': max_utilization,
            'total_operations': len(self.operations)
        }
