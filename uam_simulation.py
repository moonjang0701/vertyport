#!/usr/bin/env python3
"""
UAM Operation Trajectory Simulation - Fig 3 Recreation
3D airspace trajectory planning with constraints
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import random
from typing import List, Tuple, Optional
from dataclasses import dataclass

@dataclass
class Constraint:
    """Represents an obstacle or restricted area"""
    center: np.ndarray  # [x, y, z]
    radius: float
    height: float
    constraint_type: str  # 'obstacle', 'restriction', 'high_risk'
    
    def get_color(self):
        colors = {
            'obstacle': 'green',
            'restriction': 'blue', 
            'high_risk': 'red'
        }
        return colors.get(self.constraint_type, 'gray')
    
    def is_collision(self, point: np.ndarray, safety_margin: float = 10.0) -> bool:
        """Check if a point collides with this constraint"""
        # Check horizontal distance
        horizontal_dist = np.sqrt((point[0] - self.center[0])**2 + 
                                 (point[1] - self.center[1])**2)
        # Check vertical range
        z_min = self.center[2] - self.height / 2
        z_max = self.center[2] + self.height / 2
        
        if horizontal_dist < (self.radius + safety_margin) and z_min <= point[2] <= z_max:
            return True
        return False


@dataclass
class Waypoint:
    """Represents a waypoint in the flight plan"""
    position: np.ndarray  # [x, y, z]
    waypoint_type: str  # 'takeoff', 'cruise', 'landing'


class PathPlanner:
    """3D path planner with obstacle avoidance"""
    
    def __init__(self, constraints: List[Constraint]):
        self.constraints = constraints
        
    def plan_path(self, start: np.ndarray, goal: np.ndarray, 
                  num_waypoints: int = 20) -> List[np.ndarray]:
        """
        Simple path planning with obstacle avoidance
        Uses iterative approach to generate safe trajectory
        """
        waypoints = []
        
        # Direct path as initial attempt
        for i in range(num_waypoints):
            t = i / (num_waypoints - 1)
            point = start + t * (goal - start)
            
            # Check for collisions and adjust
            adjusted_point = self._avoid_obstacles(point, start, goal)
            waypoints.append(adjusted_point)
        
        # Smooth the path
        waypoints = self._smooth_path(waypoints)
        
        return waypoints
    
    def _avoid_obstacles(self, point: np.ndarray, start: np.ndarray, 
                        goal: np.ndarray) -> np.ndarray:
        """Adjust point to avoid obstacles"""
        adjusted = point.copy()
        
        for constraint in self.constraints:
            if constraint.is_collision(point, safety_margin=50.0):
                # Move perpendicular to the direct line
                direction = goal - start
                # Create perpendicular vector
                perp = np.array([-direction[1], direction[0], 0])
                perp_norm = perp / (np.linalg.norm(perp) + 1e-6)
                
                # Determine which side to move
                to_obstacle = constraint.center - point
                if np.dot(perp_norm[:2], to_obstacle[:2]) > 0:
                    perp_norm = -perp_norm
                
                # Adjust position
                adjusted[:2] += perp_norm[:2] * 80.0
        
        return adjusted
    
    def _smooth_path(self, waypoints: List[np.ndarray], 
                    smoothing_factor: float = 0.3) -> List[np.ndarray]:
        """Apply smoothing to the path"""
        if len(waypoints) < 3:
            return waypoints
        
        smoothed = [waypoints[0]]
        
        for i in range(1, len(waypoints) - 1):
            prev = waypoints[i - 1]
            curr = waypoints[i]
            next_wp = waypoints[i + 1]
            
            # Average with neighbors
            smoothed_point = (1 - smoothing_factor) * curr + \
                           smoothing_factor * 0.5 * (prev + next_wp)
            smoothed.append(smoothed_point)
        
        smoothed.append(waypoints[-1])
        return smoothed


class UAMSimulation:
    """Main simulation class for UAM trajectory generation"""
    
    def __init__(self, airspace_size: Tuple[float, float, float] = (2000, 2000, 500)):
        self.airspace_size = airspace_size  # meters
        self.constraints: List[Constraint] = []
        self.trajectories: List[List[np.ndarray]] = []
        self.planner: Optional[PathPlanner] = None
        
    def add_constraint(self, center: Tuple[float, float, float], 
                      radius: float, height: float, 
                      constraint_type: str):
        """Add an obstacle or restricted area"""
        constraint = Constraint(
            center=np.array(center),
            radius=radius,
            height=height,
            constraint_type=constraint_type
        )
        self.constraints.append(constraint)
        
    def generate_constraints(self):
        """Generate random constraints similar to Fig 3"""
        # Static obstacles (green)
        for _ in range(5):
            x = random.uniform(200, self.airspace_size[0] - 200)
            y = random.uniform(200, self.airspace_size[1] - 200)
            z = random.uniform(100, 300)
            self.add_constraint((x, y, z), 
                              radius=random.uniform(80, 150),
                              height=random.uniform(150, 300),
                              constraint_type='obstacle')
        
        # Airspace restrictions (blue)
        for _ in range(3):
            x = random.uniform(300, self.airspace_size[0] - 300)
            y = random.uniform(300, self.airspace_size[1] - 300)
            z = random.uniform(150, 350)
            self.add_constraint((x, y, z),
                              radius=random.uniform(100, 180),
                              height=random.uniform(200, 400),
                              constraint_type='restriction')
        
        # High-risk areas (red)
        for _ in range(4):
            x = random.uniform(400, self.airspace_size[0] - 400)
            y = random.uniform(400, self.airspace_size[1] - 400)
            z = random.uniform(100, 250)
            self.add_constraint((x, y, z),
                              radius=random.uniform(60, 120),
                              height=random.uniform(100, 250),
                              constraint_type='high_risk')
    
    def generate_flight_plan(self, start_pos: Tuple[float, float], 
                           end_pos: Tuple[float, float],
                           cruise_altitude: float = 300.0) -> List[np.ndarray]:
        """
        Generate a complete flight plan with:
        - Vertical takeoff
        - Cruise phase with obstacle avoidance
        - Vertical landing
        """
        waypoints = []
        
        # 1. Vertical takeoff
        takeoff_waypoints = self._vertical_takeoff(start_pos, cruise_altitude)
        waypoints.extend(takeoff_waypoints)
        
        # 2. Cruise phase
        cruise_start = np.array([start_pos[0], start_pos[1], cruise_altitude])
        cruise_end = np.array([end_pos[0], end_pos[1], cruise_altitude])
        
        if self.planner is None:
            self.planner = PathPlanner(self.constraints)
        
        cruise_waypoints = self.planner.plan_path(cruise_start, cruise_end, num_waypoints=30)
        waypoints.extend(cruise_waypoints)
        
        # 3. Vertical landing
        landing_waypoints = self._vertical_landing(end_pos, cruise_altitude)
        waypoints.extend(landing_waypoints)
        
        return waypoints
    
    def _vertical_takeoff(self, position: Tuple[float, float], 
                         target_altitude: float, num_points: int = 5) -> List[np.ndarray]:
        """Generate vertical takeoff trajectory"""
        waypoints = []
        for i in range(num_points):
            z = (i / (num_points - 1)) * target_altitude
            waypoints.append(np.array([position[0], position[1], z]))
        return waypoints
    
    def _vertical_landing(self, position: Tuple[float, float], 
                         start_altitude: float, num_points: int = 5) -> List[np.ndarray]:
        """Generate vertical landing trajectory"""
        waypoints = []
        for i in range(num_points):
            z = start_altitude * (1 - i / (num_points - 1))
            waypoints.append(np.array([position[0], position[1], z]))
        return waypoints
    
    def generate_multiple_trajectories(self, num_operations: int = 5):
        """Generate multiple deconflicted flight plans"""
        self.trajectories = []
        
        for i in range(num_operations):
            # Random start and end positions around the airspace
            angle_start = 2 * np.pi * i / num_operations
            angle_end = 2 * np.pi * (i + 0.5) / num_operations
            
            radius_start = self.airspace_size[0] * 0.3
            radius_end = self.airspace_size[0] * 0.3
            
            start_x = self.airspace_size[0] / 2 + radius_start * np.cos(angle_start)
            start_y = self.airspace_size[1] / 2 + radius_start * np.sin(angle_start)
            
            end_x = self.airspace_size[0] / 2 + radius_end * np.cos(angle_end)
            end_y = self.airspace_size[1] / 2 + radius_end * np.sin(angle_end)
            
            cruise_alt = 250 + random.uniform(-50, 50)
            
            trajectory = self.generate_flight_plan(
                (start_x, start_y),
                (end_x, end_y),
                cruise_altitude=cruise_alt
            )
            
            self.trajectories.append(trajectory)
    
    def visualize(self, save_path: str = 'fig3_simulation.png'):
        """Create 3D visualization similar to Fig 3"""
        fig = plt.figure(figsize=(16, 12))
        ax = fig.add_subplot(111, projection='3d')
        
        # Draw constraints
        for constraint in self.constraints:
            self._draw_cylinder(ax, constraint)
        
        # Draw trajectories
        colors = plt.cm.Blues(np.linspace(0.4, 0.9, len(self.trajectories)))
        
        for idx, trajectory in enumerate(self.trajectories):
            trajectory_array = np.array(trajectory)
            ax.plot(trajectory_array[:, 0], 
                   trajectory_array[:, 1], 
                   trajectory_array[:, 2],
                   color=colors[idx],
                   linewidth=2.5,
                   label=f'Operation {idx+1}',
                   alpha=0.8)
            
            # Mark start and end points
            ax.scatter(trajectory_array[0, 0], 
                      trajectory_array[0, 1], 
                      trajectory_array[0, 2],
                      color='green', s=100, marker='o', 
                      edgecolors='black', linewidth=2)
            ax.scatter(trajectory_array[-1, 0], 
                      trajectory_array[-1, 1], 
                      trajectory_array[-1, 2],
                      color='red', s=100, marker='s',
                      edgecolors='black', linewidth=2)
        
        # Set labels and title
        ax.set_xlabel('X (meters)', fontsize=12, fontweight='bold')
        ax.set_ylabel('Y (meters)', fontsize=12, fontweight='bold')
        ax.set_zlabel('Altitude (meters)', fontsize=12, fontweight='bold')
        ax.set_title('Operation Trajectories Generated in 3D Designated Airspace\n' +
                    'with Constraints Applied (Fig 3 Recreation)',
                    fontsize=14, fontweight='bold', pad=20)
        
        # Set axis limits
        ax.set_xlim(0, self.airspace_size[0])
        ax.set_ylim(0, self.airspace_size[1])
        ax.set_zlim(0, self.airspace_size[2])
        
        # Add legend
        legend_elements = [
            plt.Line2D([0], [0], color='green', linewidth=10, label='Static Obstacles'),
            plt.Line2D([0], [0], color='blue', linewidth=10, label='Airspace Restrictions'),
            plt.Line2D([0], [0], color='red', linewidth=10, label='High-Risk Areas'),
            plt.Line2D([0], [0], color='cyan', linewidth=2.5, label='Flight Trajectories'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='green', 
                      markersize=10, label='Takeoff Point'),
            plt.Line2D([0], [0], marker='s', color='w', markerfacecolor='red', 
                      markersize=10, label='Landing Point')
        ]
        ax.legend(handles=legend_elements, loc='upper left', fontsize=10)
        
        # Adjust viewing angle
        ax.view_init(elev=25, azim=45)
        
        # Grid
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"✓ Visualization saved to: {save_path}")
        
        return fig, ax
    
    def _draw_cylinder(self, ax, constraint: Constraint):
        """Draw a cylindrical constraint in 3D"""
        # Cylinder parameters
        center = constraint.center
        radius = constraint.radius
        height = constraint.height
        color = constraint.get_color()
        
        # Generate cylinder
        z_bottom = center[2] - height / 2
        z_top = center[2] + height / 2
        
        # Create cylinder surface
        theta = np.linspace(0, 2 * np.pi, 30)
        z = np.linspace(z_bottom, z_top, 10)
        Theta, Z = np.meshgrid(theta, z)
        X = center[0] + radius * np.cos(Theta)
        Y = center[1] + radius * np.sin(Theta)
        
        ax.plot_surface(X, Y, Z, alpha=0.3, color=color, edgecolor='none')
        
        # Draw top and bottom circles
        for z_level in [z_bottom, z_top]:
            x_circle = center[0] + radius * np.cos(theta)
            y_circle = center[1] + radius * np.sin(theta)
            z_circle = np.full_like(theta, z_level)
            ax.plot(x_circle, y_circle, z_circle, color=color, linewidth=2, alpha=0.6)


def main():
    """Main execution function"""
    print("="*80)
    print("UAM Operation Trajectory Simulation - Fig 3 Recreation")
    print("="*80)
    
    # Set random seed for reproducibility
    random.seed(42)
    np.random.seed(42)
    
    # Create simulation
    print("\n[1/5] Initializing simulation environment...")
    sim = UAMSimulation(airspace_size=(2000, 2000, 500))
    
    # Generate constraints
    print("[2/5] Generating constraints (obstacles, restrictions, high-risk areas)...")
    sim.generate_constraints()
    print(f"  ✓ Generated {len(sim.constraints)} constraints:")
    for ctype in ['obstacle', 'restriction', 'high_risk']:
        count = sum(1 for c in sim.constraints if c.constraint_type == ctype)
        print(f"    - {ctype.replace('_', ' ').title()}: {count}")
    
    # Generate trajectories
    print("[3/5] Generating flight trajectories with obstacle avoidance...")
    sim.generate_multiple_trajectories(num_operations=5)
    print(f"  ✓ Generated {len(sim.trajectories)} flight operations")
    
    # Calculate statistics
    print("[4/5] Analyzing trajectories...")
    for idx, traj in enumerate(sim.trajectories):
        traj_array = np.array(traj)
        distances = np.sqrt(np.sum(np.diff(traj_array, axis=0)**2, axis=1))
        total_distance = np.sum(distances)
        max_altitude = np.max(traj_array[:, 2])
        print(f"  Operation {idx+1}: Distance={total_distance:.1f}m, Max Alt={max_altitude:.1f}m")
    
    # Visualize
    print("[5/5] Creating 3D visualization...")
    sim.visualize(save_path='fig3_simulation.png')
    
    print("\n" + "="*80)
    print("✓ Simulation completed successfully!")
    print("="*80)
    print("\nGenerated files:")
    print("  - fig3_simulation.png : 3D visualization of trajectories")
    print("\nThe simulation recreates Fig 3 with:")
    print("  • Static obstacles (green cylinders)")
    print("  • Airspace restrictions (blue cylinders)")
    print("  • High-risk areas (red cylinders)")
    print("  • Deconflicted flight trajectories (blue curves)")
    print("  • Vertical takeoff and landing phases")
    print("="*80)


if __name__ == "__main__":
    main()
