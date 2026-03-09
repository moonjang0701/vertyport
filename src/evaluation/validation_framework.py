"""
Comprehensive Validation Framework
Implements the test methodology with real benchmarks
"""

import numpy as np
from typing import List, Dict, Tuple
from dataclasses import dataclass
from scipy import stats
import json
import time

from ..vertiport_optimization.route_based_extractor import SafeZone


# Real heliport locations in Seoul (from public data)
KNOWN_HELIPORTS = {
    'seoul': [
        {'name': 'Samsung Medical Center', 'lat': 37.4881, 'lon': 127.0857},
        {'name': 'Gangnam Severance Hospital', 'lat': 37.5172, 'lon': 127.0473},
        {'name': 'Seoul St. Mary Hospital', 'lat': 37.5020, 'lon': 127.0037},
        {'name': 'Yeouido Heliport', 'lat': 37.5285, 'lon': 126.9245},
        {'name': 'Seoul City Hall', 'lat': 37.5663, 'lon': 126.9779},
    ]
}

# Literature-based thresholds
STANDARDS = {
    'FAA': {
        'min_landing_area': 900,    # m² (14 CFR Part 77)
        'min_clearance': 50,         # m
        'max_slope': 3,              # degrees
    },
    'EASA': {
        'min_safety_distance': 100,  # m
        'wind_coverage': 0.95,
    },
    'ICAO_Annex_14': {
        'fato_length': 30,           # m
        'safety_area_factor': 1.5,   # × rotor diameter
    }
}

SUCCESS_CRITERIA = {
    'precision@5': {'min': 0.6, 'target': 0.8, 'excellent': 0.9},
    'avg_distance': {'max': 1000, 'target': 500, 'excellent': 200},
    'coverage_ratio': {'min': 0.7, 'target': 0.85, 'excellent': 0.95},
    'route_safety': {'min': 0.75, 'target': 0.85, 'excellent': 0.95},
    'computation_time': {'max': 300, 'target': 60, 'excellent': 10}
}


@dataclass
class ValidationResult:
    """Validation results for a method"""
    method_name: str
    precision_at_k: float
    avg_distance_to_heliport: float
    coverage_ratio: float
    route_safety: float
    computation_time: float
    
    def meets_criteria(self, metric: str) -> str:
        """Check if metric meets success criteria"""
        value = getattr(self, metric)
        criteria = SUCCESS_CRITERIA.get(metric, {})
        
        if metric in ['avg_distance', 'computation_time']:
            # Lower is better
            if value <= criteria.get('excellent', float('inf')):
                return 'excellent'
            elif value <= criteria.get('target', float('inf')):
                return 'target'
            elif value <= criteria.get('max', float('inf')):
                return 'minimum'
            else:
                return 'fail'
        else:
            # Higher is better
            if value >= criteria.get('excellent', 0):
                return 'excellent'
            elif value >= criteria.get('target', 0):
                return 'target'
            elif value >= criteria.get('min', 0):
                return 'minimum'
            else:
                return 'fail'


class ValidationFramework:
    """Framework for validating vertiport extraction methods"""
    
    def __init__(self, region: str = 'seoul'):
        """
        Initialize validation framework
        
        Args:
            region: Region name (must be in KNOWN_HELIPORTS)
        """
        self.region = region
        self.ground_truth = KNOWN_HELIPORTS.get(region, [])
        
    def precision_at_k(self, predicted: List[SafeZone], k: int = 5,
                      radius: float = 500.0) -> float:
        """
        Calculate Precision@K
        
        How many of the top-K predictions are near real heliports?
        
        Args:
            predicted: List of predicted vertiport locations
            k: Number of top predictions to consider
            radius: Distance threshold (meters)
            
        Returns:
            Precision value [0, 1]
        """
        if not predicted or not self.ground_truth:
            return 0.0
        
        top_k = predicted[:k]
        matches = 0
        
        for pred in top_k:
            for heliport in self.ground_truth:
                dist = self._haversine_distance(
                    pred.lat, pred.lon,
                    heliport['lat'], heliport['lon']
                )
                if dist <= radius:
                    matches += 1
                    break  # Count each prediction only once
        
        return matches / k
    
    def average_distance_to_nearest_heliport(self, 
                                            predicted: List[SafeZone]) -> float:
        """
        Calculate average distance from predictions to nearest real heliport
        
        Args:
            predicted: List of predicted vertiport locations
            
        Returns:
            Average distance in meters
        """
        if not predicted or not self.ground_truth:
            return float('inf')
        
        distances = []
        for pred in predicted:
            min_dist = float('inf')
            for heliport in self.ground_truth:
                dist = self._haversine_distance(
                    pred.lat, pred.lon,
                    heliport['lat'], heliport['lon']
                )
                min_dist = min(min_dist, dist)
            distances.append(min_dist)
        
        return np.mean(distances)
    
    def coverage_ratio(self, predicted: List[SafeZone],
                      radius: float = 1000.0) -> float:
        """
        Calculate coverage ratio
        
        What fraction of real heliports are covered by predictions?
        
        Args:
            predicted: List of predicted vertiport locations
            radius: Coverage radius (meters)
            
        Returns:
            Coverage ratio [0, 1]
        """
        if not predicted or not self.ground_truth:
            return 0.0
        
        covered = 0
        for heliport in self.ground_truth:
            for pred in predicted:
                dist = self._haversine_distance(
                    pred.lat, pred.lon,
                    heliport['lat'], heliport['lon']
                )
                if dist <= radius:
                    covered += 1
                    break  # Count each heliport only once
        
        return covered / len(self.ground_truth)
    
    def validate_method(self, predicted: List[SafeZone],
                       route_safety: float,
                       computation_time: float,
                       method_name: str) -> ValidationResult:
        """
        Validate a vertiport extraction method
        
        Args:
            predicted: Predicted vertiport locations
            route_safety: Route safety score [0, 1]
            computation_time: Time taken (seconds)
            method_name: Name of the method
            
        Returns:
            ValidationResult object
        """
        precision = self.precision_at_k(predicted, k=5)
        avg_dist = self.average_distance_to_nearest_heliport(predicted)
        coverage = self.coverage_ratio(predicted)
        
        return ValidationResult(
            method_name=method_name,
            precision_at_k=precision,
            avg_distance_to_heliport=avg_dist,
            coverage_ratio=coverage,
            route_safety=route_safety,
            computation_time=computation_time
        )
    
    def compare_methods(self, results: List[ValidationResult]) -> Dict:
        """
        Compare multiple methods statistically
        
        Args:
            results: List of validation results from different methods
            
        Returns:
            Statistical comparison results
        """
        if len(results) < 2:
            return {}
        
        comparison = {
            'methods': [r.method_name for r in results],
            'metrics': {}
        }
        
        # Compare each metric
        metrics = ['precision_at_k', 'avg_distance_to_heliport', 
                  'coverage_ratio', 'route_safety', 'computation_time']
        
        for metric in metrics:
            values = [getattr(r, metric) for r in results]
            
            # Find best
            if metric in ['avg_distance_to_heliport', 'computation_time']:
                best_idx = np.argmin(values)
            else:
                best_idx = np.argmax(values)
            
            comparison['metrics'][metric] = {
                'values': values,
                'best_method': results[best_idx].method_name,
                'best_value': values[best_idx]
            }
        
        return comparison
    
    def statistical_significance(self, our_results: List[float],
                                baseline_results: List[float]) -> Dict:
        """
        Test statistical significance using t-test
        
        Args:
            our_results: Results from our method (multiple runs)
            baseline_results: Results from baseline method
            
        Returns:
            Statistical test results
        """
        t_stat, p_value = stats.ttest_ind(our_results, baseline_results)
        
        # Cohen's d (effect size)
        mean_diff = np.mean(our_results) - np.mean(baseline_results)
        pooled_std = np.sqrt(
            (np.std(our_results)**2 + np.std(baseline_results)**2) / 2
        )
        cohen_d = mean_diff / pooled_std if pooled_std > 0 else 0
        
        # Interpret effect size
        if abs(cohen_d) < 0.2:
            effect_size = 'negligible'
        elif abs(cohen_d) < 0.5:
            effect_size = 'small'
        elif abs(cohen_d) < 0.8:
            effect_size = 'medium'
        else:
            effect_size = 'large'
        
        return {
            't_statistic': t_stat,
            'p_value': p_value,
            'significant': p_value < 0.05,
            'cohens_d': cohen_d,
            'effect_size': effect_size,
            'our_mean': np.mean(our_results),
            'baseline_mean': np.mean(baseline_results),
            'improvement': mean_diff
        }
    
    def generate_report(self, results: List[ValidationResult],
                       output_file: str = 'validation_report.json'):
        """
        Generate comprehensive validation report
        
        Args:
            results: List of validation results
            output_file: Output JSON file path
        """
        report = {
            'region': self.region,
            'ground_truth_count': len(self.ground_truth),
            'standards': STANDARDS,
            'success_criteria': SUCCESS_CRITERIA,
            'results': []
        }
        
        for result in results:
            result_dict = {
                'method': result.method_name,
                'metrics': {
                    'precision@5': {
                        'value': result.precision_at_k,
                        'status': result.meets_criteria('precision@5')
                    },
                    'avg_distance': {
                        'value': result.avg_distance_to_heliport,
                        'status': result.meets_criteria('avg_distance')
                    },
                    'coverage': {
                        'value': result.coverage_ratio,
                        'status': result.meets_criteria('coverage_ratio')
                    },
                    'route_safety': {
                        'value': result.route_safety,
                        'status': result.meets_criteria('route_safety')
                    },
                    'computation_time': {
                        'value': result.computation_time,
                        'status': result.meets_criteria('computation_time')
                    }
                }
            }
            report['results'].append(result_dict)
        
        # Add comparison
        report['comparison'] = self.compare_methods(results)
        
        # Save report
        with open(output_file, 'w') as f:
            json.dump(report, f, indent=2)
        
        return report
    
    def print_report(self, results: List[ValidationResult]):
        """Print validation report to console"""
        print("\n" + "="*80)
        print("📊 VALIDATION REPORT")
        print("="*80)
        
        print(f"\n🗺️  Region: {self.region.upper()}")
        print(f"📍 Ground Truth Heliports: {len(self.ground_truth)}")
        
        print("\n" + "-"*80)
        print("Method Comparison:")
        print("-"*80)
        
        # Header
        print(f"{'Method':<20} {'Prec@5':<10} {'AvgDist':<10} {'Cover':<10} {'Safety':<10} {'Time':<10}")
        print("-"*80)
        
        # Results
        for r in results:
            print(f"{r.method_name:<20} "
                  f"{r.precision_at_k:<10.3f} "
                  f"{r.avg_distance_to_heliport:<10.0f} "
                  f"{r.coverage_ratio:<10.3f} "
                  f"{r.route_safety:<10.3f} "
                  f"{r.computation_time:<10.1f}")
        
        print("="*80)
        
        # Success criteria assessment
        print("\n✅ Success Criteria Assessment:")
        for r in results:
            print(f"\n{r.method_name}:")
            for metric in ['precision@5', 'avg_distance', 'coverage_ratio', 
                          'route_safety', 'computation_time']:
                status = r.meets_criteria(metric)
                emoji = {'excellent': '⭐', 'target': '✓', 'minimum': '○', 'fail': '✗'}
                print(f"  {emoji.get(status, '?')} {metric}: {status}")
    
    @staticmethod
    def _haversine_distance(lat1: float, lon1: float,
                          lat2: float, lon2: float) -> float:
        """Calculate distance in meters"""
        R = 6371000  # Earth radius
        phi1, phi2 = np.radians(lat1), np.radians(lat2)
        dphi = np.radians(lat2 - lat1)
        dlambda = np.radians(lon2 - lon1)
        
        a = np.sin(dphi/2)**2 + np.cos(phi1) * np.cos(phi2) * np.sin(dlambda/2)**2
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
        
        return R * c
