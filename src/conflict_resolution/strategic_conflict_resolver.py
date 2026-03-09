"""
IEEE MAES Algorithm 2: Strategic Conflict Resolution (FCFS)
논문 방식: First-Come First-Served with time-space deconfliction

Reference:
- Paper Section: Strategic Conflict Resolution
- Algorithm 2: FCFS scheduling with sliding time windows
- Equation 3: Delay cost minimization
"""

import numpy as np
from typing import List, Tuple, Dict, Set, Optional
from dataclasses import dataclass
from collections import defaultdict


@dataclass
class FlightOperation:
    """Single flight operation"""
    operation_id: int
    waypoints: List[Tuple[int, int, int]]  # Grid cells (i, j, k)
    submission_time: float  # seconds (earlier = higher priority)
    departure_time: float  # scheduled departure
    arrival_time: float  # scheduled arrival
    priority: float = 1.0  # λ_l in Equation 3
    
    # Result after scheduling
    actual_departure: float = 0.0
    delay: float = 0.0
    assigned_slots: List[Tuple[int, int, int, float]] = None  # (cell, time)


@dataclass
class ConflictResolutionResult:
    """Result of conflict resolution"""
    operations: List[FlightOperation]
    total_delay: float
    delay_cost: float  # Equation 3
    conflicts_detected: int
    conflicts_resolved: int
    
    # Statistics
    max_delay: float
    avg_delay: float
    on_time_percent: float


class StrategyConflictResolver:
    """
    IEEE MAES Algorithm 2: FCFS Strategic Conflict Resolution
    
    Features:
    - Time-space deconfliction (one operation per cell per time slot)
    - Sliding time window (separation = sep)
    - FCFS priority (earlier submission = higher priority)
    - Delay cost calculation (Equation 3)
    """
    
    def __init__(self,
                 grid_shape: Tuple[int, int, int],
                 time_step: float = 1.0,  # seconds
                 separation: float = 10.0,  # seconds (t±sep in Algorithm 2)
                 max_delay: float = 300.0):  # Maximum acceptable delay (seconds)
        """
        Initialize conflict resolver
        
        Args:
            grid_shape: (height, width, altitude_levels)
            time_step: Time discretization (seconds)
            separation: Minimum time separation between operations in same cell
            max_delay: Maximum delay allowed (seconds)
        """
        self.grid_shape = grid_shape
        self.time_step = time_step
        self.separation = separation
        self.max_delay = max_delay
        
        # Occupancy tracking: cell -> set of occupied time slots
        self.occupancy: Dict[Tuple[int, int, int], Set[float]] = defaultdict(set)
        
        # Statistics
        self.conflicts_detected = 0
        self.conflicts_resolved = 0
    
    def is_blocked(self, cell: Tuple[int, int, int], time: float) -> bool:
        """
        Check if cell is blocked at given time
        
        Paper: Line 3 in Algorithm 2
        "if (e,t) or (be,t) is blocked then"
        """
        if cell not in self.occupancy:
            return False
        
        # Check time window [t - sep, t + sep]
        for occupied_time in self.occupancy[cell]:
            if abs(occupied_time - time) < self.separation:
                return True
        
        return False
    
    def occupy_cell(self, cell: Tuple[int, int, int], time: float):
        """
        Mark cell as occupied at time t and t±sep
        
        Paper: Lines 11-12 in Algorithm 2
        "occupancy(e,t), (e, t±sep)"
        """
        # Occupy the exact time
        self.occupancy[cell].add(time)
        
        # Also mark separation buffer
        for t_offset in np.arange(-self.separation, self.separation + self.time_step, self.time_step):
            if t_offset != 0:
                self.occupancy[cell].add(time + t_offset)
    
    def schedule_operation(self, operation: FlightOperation, verbose: bool = False) -> bool:
        """
        Schedule single operation using FCFS
        
        Paper: Algorithm 2, lines 1-15
        
        Returns:
            True if successfully scheduled, False if exceeds max_delay
        """
        if verbose:
            print(f"\n📋 Scheduling Operation {operation.operation_id}")
            print(f"   Submission: {operation.submission_time:.1f}s")
            print(f"   Scheduled departure: {operation.departure_time:.1f}s")
            print(f"   Path: {len(operation.waypoints)} waypoints")
        
        delay = 0.0
        operation.assigned_slots = []
        
        # Calculate waypoint timing
        waypoint_times = []
        total_segments = len(operation.waypoints) - 1
        if total_segments > 0:
            segment_duration = (operation.arrival_time - operation.departure_time) / total_segments
            for idx in range(len(operation.waypoints)):
                waypoint_times.append(operation.departure_time + idx * segment_duration)
        else:
            waypoint_times = [operation.departure_time]
        
        # Try to schedule each waypoint
        scheduled = False
        max_attempts = int(self.max_delay / self.time_step)
        
        for attempt in range(max_attempts):
            current_delay = delay
            blocked = False
            temp_slots = []
            
            # Check all waypoints with current delay
            for wp_idx, (cell, scheduled_time) in enumerate(zip(operation.waypoints, waypoint_times)):
                actual_time = scheduled_time + current_delay
                
                if self.is_blocked(cell, actual_time):
                    # Line 3-5: if blocked, delay by 1 time unit
                    blocked = True
                    self.conflicts_detected += 1
                    break
                
                temp_slots.append((cell, actual_time))
            
            if not blocked:
                # Successfully scheduled! Occupy all cells
                for cell, time in temp_slots:
                    self.occupy_cell(cell, time)
                    operation.assigned_slots.append((cell, time))
                
                operation.actual_departure = operation.departure_time + current_delay
                operation.delay = current_delay
                scheduled = True
                
                if current_delay > 0:
                    self.conflicts_resolved += 1
                
                if verbose:
                    if current_delay > 0:
                        print(f"   ✅ Scheduled with delay: {current_delay:.1f}s")
                    else:
                        print(f"   ✅ Scheduled on time")
                
                break
            else:
                # Delay by 1 time unit (Line 4)
                delay += self.time_step
        
        if not scheduled:
            if verbose:
                print(f"   ❌ Failed to schedule (exceeded max delay {self.max_delay}s)")
        
        return scheduled
    
    def resolve_conflicts(self,
                         operations: List[FlightOperation],
                         verbose: bool = True) -> ConflictResolutionResult:
        """
        Resolve conflicts for multiple operations using FCFS
        
        Paper: Algorithm 2
        - Sort operations by submission time (FCFS)
        - Schedule each operation sequentially
        - Calculate delay cost (Equation 3)
        
        Args:
            operations: List of flight operations
            verbose: Print progress
        
        Returns:
            ConflictResolutionResult with statistics
        """
        if verbose:
            print(f"\n{'='*60}")
            print(f"🚦 Strategic Conflict Resolution (Algorithm 2 - FCFS)")
            print(f"{'='*60}")
            print(f"Operations: {len(operations)}")
            print(f"Grid: {self.grid_shape}")
            print(f"Separation: {self.separation}s")
        
        # Reset occupancy
        self.occupancy.clear()
        self.conflicts_detected = 0
        self.conflicts_resolved = 0
        
        # Sort by submission time (FCFS) - Line 1: for f ∈ F do
        sorted_ops = sorted(operations, key=lambda op: op.submission_time)
        
        # Schedule each operation
        scheduled_ops = []
        failed_ops = []
        
        for op in sorted_ops:
            if self.schedule_operation(op, verbose=verbose):
                scheduled_ops.append(op)
            else:
                failed_ops.append(op)
        
        # Calculate delay cost (Equation 3)
        total_delay = sum(op.delay for op in scheduled_ops)
        
        # Simplified Equation 3: C_delay = Σ λ × delay
        delay_cost = sum(op.priority * op.delay for op in scheduled_ops)
        
        # Statistics
        delays = [op.delay for op in scheduled_ops]
        max_delay = max(delays) if delays else 0.0
        avg_delay = np.mean(delays) if delays else 0.0
        on_time_count = sum(1 for op in scheduled_ops if op.delay == 0)
        on_time_percent = (on_time_count / len(operations)) * 100 if operations else 0.0
        
        if verbose:
            print(f"\n{'='*60}")
            print(f"📊 Resolution Results")
            print(f"{'='*60}")
            print(f"✅ Scheduled: {len(scheduled_ops)}/{len(operations)}")
            print(f"❌ Failed: {len(failed_ops)}")
            print(f"🔍 Conflicts detected: {self.conflicts_detected}")
            print(f"✔️  Conflicts resolved: {self.conflicts_resolved}")
            print(f"⏱️  Total delay: {total_delay:.1f}s")
            print(f"💰 Delay cost (Eq. 3): {delay_cost:.2f}")
            print(f"📈 Max delay: {max_delay:.1f}s")
            print(f"📊 Avg delay: {avg_delay:.1f}s")
            print(f"⏰ On-time: {on_time_percent:.1f}%")
            
            if failed_ops:
                print(f"\n⚠️  Failed operations:")
                for op in failed_ops:
                    print(f"   - Op {op.operation_id}: exceeded max delay")
        
        result = ConflictResolutionResult(
            operations=scheduled_ops,
            total_delay=total_delay,
            delay_cost=delay_cost,
            conflicts_detected=self.conflicts_detected,
            conflicts_resolved=self.conflicts_resolved,
            max_delay=max_delay,
            avg_delay=avg_delay,
            on_time_percent=on_time_percent
        )
        
        return result
    
    def get_occupancy_heatmap(self, time_range: Tuple[float, float], altitude_level: int = 0) -> np.ndarray:
        """
        Generate occupancy heatmap for visualization
        
        Args:
            time_range: (start_time, end_time)
            altitude_level: Which altitude level to visualize
        
        Returns:
            2D array of occupancy counts
        """
        h, w, _ = self.grid_shape
        heatmap = np.zeros((h, w), dtype=int)
        
        start_time, end_time = time_range
        
        for cell, times in self.occupancy.items():
            i, j, k = cell
            if k == altitude_level:
                # Count how many times this cell is occupied in time range
                count = sum(1 for t in times if start_time <= t <= end_time)
                heatmap[i, j] += count
        
        return heatmap
