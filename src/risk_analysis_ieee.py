"""
IEEE MAES Paper - Risk Analysis Assistance Implementation
Equation 1: R = P_CR × P_IM|CR × P_FA|IM
"""

import numpy as np
from typing import Tuple, Dict
from dataclasses import dataclass


@dataclass
class AircraftParams:
    """Aircraft performance parameters"""
    size: float  # m² (cross-sectional area)
    weight: float  # kg
    cruise_speed: float  # m/s
    max_glide_ratio: float  # dimensionless (typical eVTOL: 3-5)


@dataclass
class EnvironmentalParams:
    """Environmental factors"""
    wind_speed: float  # m/s
    wind_direction: float  # degrees (0-360)
    temperature: float  # Celsius
    visibility: float  # meters


@dataclass
class GroundContext:
    """Ground context data"""
    population_density: float  # people per km²
    road_traffic_density: float  # vehicles per km²
    building_density: float  # buildings per km²
    protected_areas: bool  # schools, hospitals, etc.


class RiskAnalysisIEEE:
    """
    IEEE MAES Paper Implementation
    Risk Analysis Assistance Service (Section III-C)
    """
    
    def __init__(self, risk_threshold: float = 1e-6):
        """
        Initialize risk analysis
        
        Args:
            risk_threshold: Acceptable risk level (SORA-based)
                           1e-6 = 1 fatality per 1 million flight hours
        """
        self.risk_threshold = risk_threshold
        
        # SORA-based catastrophic failure rates (per flight hour)
        self.P_CR_BASE = {
            "low": 1e-4,      # Well-maintained, certified systems
            "medium": 5e-4,   # Standard systems
            "high": 1e-3      # Experimental or degraded systems
        }
    
    def calculate_P_CR(self, 
                       aircraft: AircraftParams,
                       environment: EnvironmentalParams,
                       segment_duration: float,
                       reliability_class: str = "medium") -> float:
        """
        Calculate P_CR: Probability of Catastrophic failure
        
        Based on:
        - Aircraft reliability class
        - Environmental conditions
        - Flight duration
        
        Args:
            aircraft: Aircraft parameters
            environment: Environmental conditions
            segment_duration: Segment flight time (hours)
            reliability_class: "low", "medium", or "high" risk
        
        Returns:
            P_CR value
        """
        # Base failure rate
        base_rate = self.P_CR_BASE[reliability_class]
        
        # Environmental adjustment factors
        wind_factor = 1.0 + (environment.wind_speed / 15.0) * 0.2  # 15 m/s nominal
        temp_factor = 1.0 + abs(environment.temperature - 20) / 40.0 * 0.1  # 20°C optimal
        vis_factor = 1.0 if environment.visibility > 5000 else 1.5  # Low visibility penalty
        
        # Aircraft-specific adjustments
        weight_factor = 1.0 + (aircraft.weight / 500.0) * 0.1  # 500 kg nominal
        
        # Combine factors
        environmental_multiplier = wind_factor * temp_factor * vis_factor
        aircraft_multiplier = weight_factor
        
        # Total P_CR for segment
        P_CR = base_rate * segment_duration * environmental_multiplier * aircraft_multiplier
        
        return min(P_CR, 0.1)  # Cap at 10%
    
    def calculate_P_IM_given_CR(self,
                                aircraft: AircraftParams,
                                environment: EnvironmentalParams,
                                altitude: float,
                                ground_context: GroundContext) -> float:
        """
        Calculate P_IM|CR: Conditional probability of Impact given catastrophic failure
        
        Considers:
        - Glide capability (altitude, glide ratio)
        - Population density (more people = higher impact probability)
        - Environmental conditions
        
        Args:
            aircraft: Aircraft parameters
            environment: Environmental conditions
            altitude: Flight altitude (meters AGL)
            ground_context: Ground information
        
        Returns:
            P_IM|CR value
        """
        # Glide distance estimation
        glide_distance = altitude * aircraft.max_glide_ratio  # meters
        
        # Impact area (ellipse due to wind)
        wind_effect = 1.0 + environment.wind_speed / 10.0  # Wind increases dispersion
        impact_area_km2 = (glide_distance / 1000.0) ** 2 * np.pi * wind_effect  # km²
        
        # Ground occupancy
        pop_density_normalized = min(ground_context.population_density / 10000.0, 1.0)  # 10k/km² = 1.0
        road_density_normalized = min(ground_context.road_traffic_density / 1000.0, 1.0)
        building_density_normalized = min(ground_context.building_density / 500.0, 1.0)
        
        ground_occupancy = (
            0.5 * pop_density_normalized +
            0.3 * road_density_normalized +
            0.2 * building_density_normalized
        )
        
        # Protected area penalty
        protected_penalty = 2.0 if ground_context.protected_areas else 1.0
        
        # Base impact probability (controllability loss)
        base_impact_prob = 0.8  # 80% impact if catastrophic failure
        
        # Altitude safety factor (higher altitude = more control options)
        altitude_factor = 1.0 - min(altitude / 150.0, 0.5)  # Up to 50% reduction at 150m
        
        # Calculate conditional probability
        P_IM_CR = (
            base_impact_prob *
            altitude_factor *
            ground_occupancy *
            protected_penalty
        )
        
        return min(P_IM_CR, 0.95)  # Cap at 95%
    
    def calculate_P_FA_given_IM(self,
                                aircraft: AircraftParams,
                                ground_context: GroundContext) -> float:
        """
        Calculate P_FA|IM: Probability of FAtality given impact
        
        Considers:
        - Aircraft kinetic energy (mass, speed)
        - Population density
        - Protected areas
        
        Args:
            aircraft: Aircraft parameters
            ground_context: Ground information
        
        Returns:
            P_FA|IM value
        """
        # Kinetic energy at impact (simplified)
        # KE = 0.5 * m * v² (Joules)
        kinetic_energy = 0.5 * aircraft.weight * (aircraft.cruise_speed ** 2)
        
        # Energy factor (normalized to typical eVTOL)
        # Joby S4: ~400kg, 50 m/s → ~500 kJ
        energy_factor = min(kinetic_energy / 500000.0, 2.0)  # Cap at 2x
        
        # Population exposure
        pop_density_normalized = min(ground_context.population_density / 10000.0, 1.0)
        
        # Base fatality probability given impact
        # Based on aviation accident statistics
        base_fatality_prob = 0.1  # 10% for small aircraft
        
        # Protected area critical penalty
        protected_multiplier = 5.0 if ground_context.protected_areas else 1.0
        
        # Calculate P_FA|IM
        P_FA_IM = (
            base_fatality_prob *
            energy_factor *
            (0.5 + 0.5 * pop_density_normalized) *  # Minimum 50% even in empty areas
            protected_multiplier
        )
        
        return min(P_FA_IM, 0.8)  # Cap at 80%
    
    def calculate_segment_risk(self,
                               aircraft: AircraftParams,
                               environment: EnvironmentalParams,
                               altitude: float,
                               ground_context: GroundContext,
                               segment_duration: float,
                               reliability_class: str = "medium") -> Dict[str, float]:
        """
        Calculate total risk for a flight segment using Equation 1
        
        R = P_CR × P_IM|CR × P_FA|IM
        
        Args:
            aircraft: Aircraft parameters
            environment: Environmental conditions
            altitude: Flight altitude (meters AGL)
            ground_context: Ground information
            segment_duration: Flight time (hours)
            reliability_class: Aircraft reliability
        
        Returns:
            Dictionary with risk components and total risk
        """
        # Calculate individual probabilities
        P_CR = self.calculate_P_CR(
            aircraft, environment, segment_duration, reliability_class
        )
        
        P_IM_CR = self.calculate_P_IM_given_CR(
            aircraft, environment, altitude, ground_context
        )
        
        P_FA_IM = self.calculate_P_FA_given_IM(
            aircraft, ground_context
        )
        
        # Equation 1: R = P_CR × P_IM|CR × P_FA|IM
        R = P_CR * P_IM_CR * P_FA_IM
        
        return {
            "P_CR": P_CR,
            "P_IM|CR": P_IM_CR,
            "P_FA|IM": P_FA_IM,
            "R_total": R,
            "exceeds_threshold": R > self.risk_threshold,
            "risk_level": self._classify_risk(R)
        }
    
    def _classify_risk(self, risk: float) -> str:
        """Classify risk level"""
        if risk < 1e-7:
            return "VERY_LOW"
        elif risk < 1e-6:
            return "LOW"
        elif risk < 1e-5:
            return "MEDIUM"
        elif risk < 1e-4:
            return "HIGH"
        else:
            return "VERY_HIGH"
    
    def evaluate_flight_plan(self,
                            segments: list,
                            aircraft: AircraftParams,
                            environment: EnvironmentalParams) -> Tuple[bool, list]:
        """
        Evaluate entire flight plan
        
        Args:
            segments: List of (altitude, ground_context, duration) tuples
            aircraft: Aircraft parameters
            environment: Environmental conditions
        
        Returns:
            (approved, high_risk_segments)
        """
        high_risk_segments = []
        
        for idx, (altitude, ground_context, duration) in enumerate(segments):
            risk_result = self.calculate_segment_risk(
                aircraft, environment, altitude, ground_context, duration
            )
            
            if risk_result["exceeds_threshold"]:
                high_risk_segments.append({
                    "segment_id": idx,
                    "risk": risk_result["R_total"],
                    "risk_level": risk_result["risk_level"],
                    "details": risk_result
                })
        
        approved = len(high_risk_segments) == 0
        
        return approved, high_risk_segments


# Example usage
if __name__ == "__main__":
    # Define aircraft (typical eVTOL)
    aircraft = AircraftParams(
        size=20.0,  # m²
        weight=450.0,  # kg (Joby S4-like)
        cruise_speed=50.0,  # m/s (~180 km/h)
        max_glide_ratio=4.0
    )
    
    # Environmental conditions
    environment = EnvironmentalParams(
        wind_speed=5.0,  # m/s
        wind_direction=90.0,  # East
        temperature=20.0,  # °C
        visibility=10000.0  # meters
    )
    
    # Ground context: Dense urban area
    ground_dense = GroundContext(
        population_density=8000.0,  # people/km²
        road_traffic_density=500.0,  # vehicles/km²
        building_density=300.0,  # buildings/km²
        protected_areas=False
    )
    
    # Ground context: Protected area (school)
    ground_protected = GroundContext(
        population_density=2000.0,
        road_traffic_density=100.0,
        building_density=50.0,
        protected_areas=True
    )
    
    # Create risk analyzer
    analyzer = RiskAnalysisIEEE(risk_threshold=1e-6)
    
    # Test segment: 100m altitude, 60 seconds flight time
    print("="*80)
    print("DENSE URBAN AREA")
    print("="*80)
    result_dense = analyzer.calculate_segment_risk(
        aircraft=aircraft,
        environment=environment,
        altitude=100.0,
        ground_context=ground_dense,
        segment_duration=60.0 / 3600.0,  # Convert seconds to hours
        reliability_class="medium"
    )
    
    print(f"P_CR (Catastrophic failure): {result_dense['P_CR']:.2e}")
    print(f"P_IM|CR (Impact given failure): {result_dense['P_IM|CR']:.4f}")
    print(f"P_FA|IM (Fatality given impact): {result_dense['P_FA|IM']:.4f}")
    print(f"R_total: {result_dense['R_total']:.2e}")
    print(f"Risk level: {result_dense['risk_level']}")
    print(f"Exceeds threshold: {result_dense['exceeds_threshold']}")
    
    print("\n" + "="*80)
    print("PROTECTED AREA (SCHOOL)")
    print("="*80)
    result_protected = analyzer.calculate_segment_risk(
        aircraft=aircraft,
        environment=environment,
        altitude=100.0,
        ground_context=ground_protected,
        segment_duration=60.0 / 3600.0,
        reliability_class="medium"
    )
    
    print(f"P_CR: {result_protected['P_CR']:.2e}")
    print(f"P_IM|CR: {result_protected['P_IM|CR']:.4f}")
    print(f"P_FA|IM: {result_protected['P_FA|IM']:.4f}")
    print(f"R_total: {result_protected['R_total']:.2e}")
    print(f"Risk level: {result_protected['risk_level']}")
    print(f"Exceeds threshold: {result_protected['exceeds_threshold']}")
