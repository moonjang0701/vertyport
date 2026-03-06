"""
Performance Evaluation System
Comprehensive metrics and analysis for UAM system
"""

import numpy as np
from typing import List, Dict, Tuple
from dataclasses import dataclass, asdict
import json
from datetime import datetime

from ..path_planning.path_planner import FlightPath
from ..vertiport_optimization.route_based_extractor import SafeZone
from ..data_processing.vworld_loader import VWorldDEMData


@dataclass
class RouteMetrics:
    """Metrics for a single route"""
    route_id: int
    total_length: float  # meters
    straight_line_distance: float  # meters
    efficiency: float  # ratio
    average_risk: float  # [0,1]
    max_risk: float  # [0,1]
    route_safety: float  # [0,1]
    altitude_variance: float  # meters
    num_waypoints: int
    computation_time: float  # seconds


@dataclass
class VertiportMetrics:
    """Metrics for vertiport network"""
    num_vertiports: int
    average_safety_score: float
    average_flatness_score: float
    average_total_score: float
    min_spacing: float  # meters
    average_spacing: float  # meters
    coverage_area: float  # km²
    network_connectivity: float  # [0,1]
    spatial_uniformity: float  # [0,1]


@dataclass
class SystemMetrics:
    """Overall system performance metrics"""
    num_routes: int
    num_vertiports: int
    average_route_safety: float
    average_route_efficiency: float
    network_coverage: float  # percentage
    integrated_performance_index: float  # IPI
    total_computation_time: float  # seconds
    timestamp: str


class PerformanceEvaluator:
    """Evaluate and analyze UAM system performance"""
    
    def __init__(self, dem_data: VWorldDEMData, risk_map: np.ndarray):
        """
        Initialize evaluator
        
        Args:
            dem_data: DEM data for reference
            risk_map: Risk assessment map
        """
        self.dem_data = dem_data
        self.risk_map = risk_map
        
    def evaluate_route(self,
                      route: FlightPath,
                      route_id: int,
                      origin: Tuple[float, float],
                      destination: Tuple[float, float],
                      computation_time: float = 0.0) -> RouteMetrics:
        """
        Evaluate a single route
        
        Args:
            route: Flight path to evaluate
            route_id: Route identifier
            origin: (lat, lon) origin
            destination: (lat, lon) destination
            computation_time: Time taken to plan route
            
        Returns:
            RouteMetrics object
        """
        # Calculate straight line distance
        straight_dist = self._haversine_distance(
            origin[0], origin[1],
            destination[0], destination[1]
        )
        
        # Calculate actual route length
        total_length = route.total_distance
        
        # Efficiency
        efficiency = straight_dist / total_length if total_length > 0 else 0
        
        # Risk analysis
        risks = []
        for wp in route.waypoints:
            x_idx = int(wp.x) % self.risk_map.shape[1]
            y_idx = int(wp.y) % self.risk_map.shape[0]
            risks.append(self.risk_map[y_idx, x_idx])
        
        avg_risk = np.mean(risks)
        max_risk = np.max(risks)
        route_safety = 1.0 - avg_risk
        
        # Altitude variance
        altitudes = [wp.z for wp in route.waypoints]
        altitude_var = np.std(altitudes)
        
        return RouteMetrics(
            route_id=route_id,
            total_length=total_length,
            straight_line_distance=straight_dist,
            efficiency=efficiency,
            average_risk=avg_risk,
            max_risk=max_risk,
            route_safety=route_safety,
            altitude_variance=altitude_var,
            num_waypoints=len(route.waypoints),
            computation_time=computation_time
        )
    
    def evaluate_vertiport_network(self, vertiports: List[SafeZone]) -> VertiportMetrics:
        """
        Evaluate vertiport network
        
        Args:
            vertiports: List of vertiport locations
            
        Returns:
            VertiportMetrics object
        """
        if not vertiports:
            return VertiportMetrics(
                num_vertiports=0,
                average_safety_score=0,
                average_flatness_score=0,
                average_total_score=0,
                min_spacing=0,
                average_spacing=0,
                coverage_area=0,
                network_connectivity=0,
                spatial_uniformity=0
            )
        
        # Average scores
        avg_safety = np.mean([vp.safety_score for vp in vertiports])
        avg_flatness = np.mean([vp.flatness_score for vp in vertiports])
        avg_total = np.mean([vp.total_score for vp in vertiports])
        
        # Spacing analysis
        spacings = []
        for i, vp1 in enumerate(vertiports):
            for j, vp2 in enumerate(vertiports):
                if i < j:
                    dist = self._haversine_distance(
                        vp1.lat, vp1.lon,
                        vp2.lat, vp2.lon
                    )
                    spacings.append(dist)
        
        min_spacing = np.min(spacings) if spacings else 0
        avg_spacing = np.mean(spacings) if spacings else 0
        
        # Coverage area (sum of individual coverage areas)
        total_coverage = sum(vp.area for vp in vertiports) / 1e6  # Convert to km²
        
        # Network connectivity
        connectivity = self._calculate_connectivity(vertiports)
        
        # Spatial uniformity (using Voronoi cell variance)
        uniformity = self._calculate_uniformity(vertiports)
        
        return VertiportMetrics(
            num_vertiports=len(vertiports),
            average_safety_score=avg_safety,
            average_flatness_score=avg_flatness,
            average_total_score=avg_total,
            min_spacing=min_spacing,
            average_spacing=avg_spacing,
            coverage_area=total_coverage,
            network_connectivity=connectivity,
            spatial_uniformity=uniformity
        )
    
    def evaluate_system(self,
                       routes: List[FlightPath],
                       vertiports: List[SafeZone],
                       route_metrics_list: List[RouteMetrics],
                       vertiport_metrics: VertiportMetrics,
                       total_time: float) -> SystemMetrics:
        """
        Evaluate overall system performance
        
        Args:
            routes: All planned routes
            vertiports: All vertiports
            route_metrics_list: Pre-calculated route metrics
            vertiport_metrics: Pre-calculated vertiport metrics
            total_time: Total computation time
            
        Returns:
            SystemMetrics object
        """
        # Average route metrics
        avg_route_safety = np.mean([rm.route_safety for rm in route_metrics_list])
        avg_route_efficiency = np.mean([rm.efficiency for rm in route_metrics_list])
        
        # Network coverage (percentage of area covered)
        total_area = self._calculate_total_area()
        coverage_percentage = (vertiport_metrics.coverage_area / total_area) * 100
        
        # Integrated Performance Index (IPI)
        w_s, w_e, w_c = 0.5, 0.2, 0.3
        ipi = (w_s * avg_route_safety + 
               w_e * avg_route_efficiency + 
               w_c * (coverage_percentage / 100))
        
        return SystemMetrics(
            num_routes=len(routes),
            num_vertiports=len(vertiports),
            average_route_safety=avg_route_safety,
            average_route_efficiency=avg_route_efficiency,
            network_coverage=coverage_percentage,
            integrated_performance_index=ipi,
            total_computation_time=total_time,
            timestamp=datetime.now().isoformat()
        )
    
    def _calculate_connectivity(self, vertiports: List[SafeZone],
                               max_distance: float = 2000.0) -> float:
        """
        Calculate network connectivity
        
        Connectivity = 2E / (M(M-1))
        where E = number of connected pairs, M = number of vertiports
        """
        if len(vertiports) < 2:
            return 0.0
        
        num_connections = 0
        total_pairs = len(vertiports) * (len(vertiports) - 1) / 2
        
        for i, vp1 in enumerate(vertiports):
            for j, vp2 in enumerate(vertiports):
                if i < j:
                    dist = self._haversine_distance(
                        vp1.lat, vp1.lon,
                        vp2.lat, vp2.lon
                    )
                    if dist < max_distance:
                        num_connections += 1
        
        connectivity = (2 * num_connections) / (len(vertiports) * (len(vertiports) - 1))
        return connectivity
    
    def _calculate_uniformity(self, vertiports: List[SafeZone]) -> float:
        """
        Calculate spatial uniformity using coverage area variance
        
        Uniformity = 1 - σ(areas) / mean(areas)
        """
        if len(vertiports) < 2:
            return 1.0
        
        areas = [vp.area for vp in vertiports]
        mean_area = np.mean(areas)
        std_area = np.std(areas)
        
        if mean_area == 0:
            return 0.0
        
        uniformity = 1.0 - (std_area / mean_area)
        return max(0.0, uniformity)
    
    def _calculate_total_area(self) -> float:
        """Calculate total area of the region in km²"""
        # Approximate using Haversine
        width = self._haversine_distance(
            self.dem_data.lat_min, self.dem_data.lon_min,
            self.dem_data.lat_min, self.dem_data.lon_max
        )
        height = self._haversine_distance(
            self.dem_data.lat_min, self.dem_data.lon_min,
            self.dem_data.lat_max, self.dem_data.lon_min
        )
        
        area_m2 = width * height
        area_km2 = area_m2 / 1e6
        
        return area_km2
    
    @staticmethod
    def _haversine_distance(lat1: float, lon1: float,
                          lat2: float, lon2: float) -> float:
        """Calculate distance in meters"""
        R = 6371000  # Earth radius in meters
        
        phi1 = np.radians(lat1)
        phi2 = np.radians(lat2)
        dphi = np.radians(lat2 - lat1)
        dlambda = np.radians(lon2 - lon1)
        
        a = np.sin(dphi/2)**2 + np.cos(phi1) * np.cos(phi2) * np.sin(dlambda/2)**2
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
        
        return R * c
    
    def generate_report(self,
                       route_metrics: List[RouteMetrics],
                       vertiport_metrics: VertiportMetrics,
                       system_metrics: SystemMetrics,
                       output_file: str = "evaluation_report.json"):
        """
        Generate comprehensive evaluation report
        
        Args:
            route_metrics: List of route metrics
            vertiport_metrics: Vertiport network metrics
            system_metrics: System-level metrics
            output_file: Output JSON file path
        """
        report = {
            'system_metrics': asdict(system_metrics),
            'vertiport_metrics': asdict(vertiport_metrics),
            'route_metrics': [asdict(rm) for rm in route_metrics],
            'summary': {
                'routes': {
                    'total': len(route_metrics),
                    'avg_length_km': np.mean([rm.total_length for rm in route_metrics]) / 1000,
                    'avg_efficiency': np.mean([rm.efficiency for rm in route_metrics]),
                    'avg_safety': np.mean([rm.route_safety for rm in route_metrics])
                },
                'vertiports': {
                    'total': vertiport_metrics.num_vertiports,
                    'avg_score': vertiport_metrics.average_total_score,
                    'avg_spacing_m': vertiport_metrics.average_spacing,
                    'coverage_km2': vertiport_metrics.coverage_area
                },
                'performance': {
                    'ipi': system_metrics.integrated_performance_index,
                    'coverage_percent': system_metrics.network_coverage,
                    'computation_time_s': system_metrics.total_computation_time
                }
            }
        }
        
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        
        print(f"Evaluation report saved to {output_file}")
        return report
    
    def print_summary(self,
                     route_metrics: List[RouteMetrics],
                     vertiport_metrics: VertiportMetrics,
                     system_metrics: SystemMetrics):
        """Print evaluation summary to console"""
        print("\n" + "="*70)
        print("📊 UAM SYSTEM PERFORMANCE EVALUATION SUMMARY")
        print("="*70)
        
        print("\n🛩️  ROUTE PERFORMANCE:")
        print(f"  • Number of routes: {len(route_metrics)}")
        print(f"  • Average length: {np.mean([rm.total_length for rm in route_metrics]):.1f} m")
        print(f"  • Average efficiency: {np.mean([rm.efficiency for rm in route_metrics]):.2%}")
        print(f"  • Average safety: {np.mean([rm.route_safety for rm in route_metrics]):.3f}")
        print(f"  • Average risk: {np.mean([rm.average_risk for rm in route_metrics]):.3f}")
        
        print("\n🚁 VERTIPORT NETWORK:")
        print(f"  • Number of vertiports: {vertiport_metrics.num_vertiports}")
        print(f"  • Average total score: {vertiport_metrics.average_total_score:.3f}")
        print(f"  • Average safety score: {vertiport_metrics.average_safety_score:.3f}")
        print(f"  • Min spacing: {vertiport_metrics.min_spacing:.1f} m")
        print(f"  • Avg spacing: {vertiport_metrics.average_spacing:.1f} m")
        print(f"  • Coverage area: {vertiport_metrics.coverage_area:.2f} km²")
        print(f"  • Network connectivity: {vertiport_metrics.network_connectivity:.3f}")
        print(f"  • Spatial uniformity: {vertiport_metrics.spatial_uniformity:.3f}")
        
        print("\n🎯 SYSTEM PERFORMANCE:")
        print(f"  • Integrated Performance Index: {system_metrics.integrated_performance_index:.3f}")
        print(f"  • Network coverage: {system_metrics.network_coverage:.1f}%")
        print(f"  • Total computation time: {system_metrics.total_computation_time:.2f} s")
        
        print("\n" + "="*70)
