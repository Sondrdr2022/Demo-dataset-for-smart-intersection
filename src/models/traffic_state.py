"""
Traffic State and Lane Models

This module defines the core data structures for representing traffic states,
lane conditions, and intersection models used throughout the system.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
from enum import Enum
import time
import uuid

class TrafficState(Enum):
    """Traffic state classifications"""
    SMOOTH = "SMOOTH"      # Good traffic flow (+1 to +5 points)
    CONGESTED = "CONGESTED"  # Bad traffic flow (-1 to -5 points)
    CRITICAL = "CRITICAL"   # Severe congestion (requires immediate action)

class VehicleType(Enum):
    """Vehicle type classifications with priority weights"""
    CAR = ("car", 1)
    BUS = ("bus", 2) 
    TRUCK = ("truck", 2)
    EMERGENCY = ("emergency", 3)
    
    def __init__(self, type_name: str, weight: int):
        self.type_name = type_name
        self.weight = weight

@dataclass
class TrafficParameters:
    """
    Core traffic parameters for Status_t = f(l, tđ, m, v, g)
    """
    l: float = 0.0      # Queue length (number of vehicles)
    td: float = 0.0     # Average waiting time (seconds) 
    m: float = 0.0      # Vehicle density (0-1 normalized)
    v: float = 0.0      # Average speed (m/s)
    g: float = 0.0      # Vehicle flow rate (vehicles/hour)
    t: float = 0.0      # Current time calculation
    c: int = 0          # Current light cycle
    ambulance_detected: bool = False  # Emergency vehicle detection flag
    raw_count: int = 0  # Raw vehicle count
    
    def normalize_parameters(self, max_values: Dict[str, float]) -> Dict[str, float]:
        """Normalize parameters for status calculation"""
        normalized = {}
        for param in ['l', 'td', 'm', 'v', 'g']:
            max_val = max_values.get(param, 1.0)
            if max_val > 0:
                if param == 'v':  # Higher speed is better, so invert
                    normalized[param] = 1.0 - min(getattr(self, param) / max_val, 1.0)
                else:  # Lower values are better for queue, waiting time, density
                    normalized[param] = min(getattr(self, param) / max_val, 1.0)
            else:
                normalized[param] = 0.0
        return normalized

@dataclass 
class LaneState:
    """
    Represents the state of a single lane with scoring system
    """
    lane_id: str
    edge_id: str = ""
    detector_id: Optional[str] = None
    
    # Current traffic parameters
    parameters: TrafficParameters = field(default_factory=TrafficParameters)
    
    # Lane scoring system
    lane_score: float = 0.0  # Accumulated score (+5 to -5 range)
    current_state: TrafficState = TrafficState.SMOOTH
    last_state_change: float = 0.0
    state_duration: float = 0.0
    
    # Historical data for trend analysis
    score_history: List[Tuple[float, float]] = field(default_factory=list)  # (time, score)
    parameter_history: List[Tuple[float, TrafficParameters]] = field(default_factory=list)
    
    def update_parameters(self, new_params: TrafficParameters):
        """Update traffic parameters and maintain history"""
        current_time = time.time()
        
        # Store historical data
        self.parameter_history.append((current_time, self.parameters))
        if len(self.parameter_history) > 100:  # Keep last 100 entries
            self.parameter_history.pop(0)
            
        # Update current parameters
        self.parameters = new_params
        self.parameters.t = current_time
        
    def calculate_status_t(self, weights: Dict[str, float], max_values: Dict[str, float]) -> float:
        """
        Calculate Status_t = f(l, tđ, m, v, g) using normalized parameters
        """
        normalized = self.parameters.normalize_parameters(max_values)
        
        status = sum(weights[param] * normalized[param] for param in weights.keys())
        return min(max(status, 0.0), 1.0)  # Clamp between 0 and 1
    
    def update_lane_score(self, status_t: float, threshold: float):
        """
        Update lane score based on current status with continuous rewards
        GOOD traffic: +1 to +5 points
        BAD traffic: -1 to -5 points
        State changes maintain score and continue with new state
        """
        current_time = time.time()
        
        # Determine current state
        if status_t < threshold * 0.5:
            new_state = TrafficState.SMOOTH
            # Reward based on how smooth (lower status is better)
            reward = 1 + int(4 * (1 - status_t / (threshold * 0.5)))  # +1 to +5
        elif status_t > threshold:
            if status_t > 0.8:  # Critical threshold
                new_state = TrafficState.CRITICAL
                penalty = -3 - int(2 * (status_t - 0.8) / 0.2)  # -3 to -5
            else:
                new_state = TrafficState.CONGESTED
                penalty = -1 - int(2 * (status_t - threshold) / (0.8 - threshold))  # -1 to -3
            reward = penalty
        else:
            # Neutral zone - small positive reward
            new_state = TrafficState.SMOOTH
            reward = 1
        
        # Update state tracking
        if new_state != self.current_state:
            self.last_state_change = current_time
            self.state_duration = 0.0
            self.current_state = new_state
        else:
            self.state_duration = current_time - self.last_state_change
        
        # Apply reward to lane score
        self.lane_score += reward
        
        # Clamp lane score to reasonable bounds
        self.lane_score = max(-50, min(50, self.lane_score))
        
        # Store score history
        self.score_history.append((current_time, self.lane_score))
        if len(self.score_history) > 200:  # Keep last 200 entries
            self.score_history.pop(0)
        
        return reward

@dataclass
class ApproachState:
    """
    Represents an approach (group of lanes from same direction) to intersection
    """
    approach_id: str
    direction: str  # "north", "south", "east", "west"
    lanes: Dict[str, LaneState] = field(default_factory=dict)
    
    # Approach-level metrics
    total_score: float = 0.0
    avg_status: float = 0.0
    priority_level: float = 0.0
    
    # Left turn handling
    has_left_turn: bool = False
    left_turn_lanes: List[str] = field(default_factory=list)
    left_turn_blocked_time: float = 0.0
    
    def update_approach_metrics(self):
        """Calculate approach-level metrics from lane states"""
        if not self.lanes:
            return
        
        # Aggregate lane scores
        total_lanes = len(self.lanes)
        self.total_score = sum(lane.lane_score for lane in self.lanes.values())
        
        # Calculate average status
        lane_statuses = []
        for lane in self.lanes.values():
            # Use recent parameter history to estimate status
            if lane.parameter_history:
                # Simple status estimation based on recent parameters
                recent_params = lane.parameter_history[-1][1]
                estimated_status = (recent_params.l * 0.3 + recent_params.td * 0.3 + 
                                  recent_params.m * 0.2 + (1 - recent_params.v/50) * 0.2)
                lane_statuses.append(min(estimated_status, 1.0))
        
        self.avg_status = sum(lane_statuses) / len(lane_statuses) if lane_statuses else 0.0
        
        # Calculate priority level (higher for more congested approaches)
        self.priority_level = self.avg_status * (1 + abs(self.total_score) / 10.0)

@dataclass
class IntersectionState:
    """
    Complete intersection state with all approaches and traffic lights
    """
    intersection_id: str
    traffic_light_ids: List[str] = field(default_factory=list)
    approaches: Dict[str, ApproachState] = field(default_factory=dict)
    
    # Current traffic light state
    current_phase: int = 0
    phase_start_time: float = 0.0
    phase_duration: float = 0.0
    cycle_count: int = 0
    
    # Emergency handling
    emergency_detected: bool = False
    emergency_approach: Optional[str] = None
    emergency_override_active: bool = False
    emergency_start_time: float = 0.0
    
    # System performance metrics
    total_waiting_time: float = 0.0
    total_vehicles_served: int = 0
    average_delay: float = 0.0
    
    def detect_emergency_vehicles(self) -> bool:
        """
        Detect emergency vehicles in any approach
        Sets emergency_detected flag and identifies approach
        """
        self.emergency_detected = False
        self.emergency_approach = None
        
        for approach_id, approach in self.approaches.items():
            for lane in approach.lanes.values():
                if lane.parameters.ambulance_detected:
                    self.emergency_detected = True
                    self.emergency_approach = approach_id
                    if not self.emergency_override_active:
                        self.emergency_start_time = time.time()
                    return True
        
        # Reset emergency state if no vehicles detected
        if self.emergency_override_active and not self.emergency_detected:
            self.emergency_override_active = False
            
        return False
    
    def get_priority_approach(self) -> Optional[str]:
        """
        Determine which approach should get priority based on scoring system
        Returns approach_id with highest priority or emergency approach
        """
        if self.emergency_detected and self.emergency_approach:
            return self.emergency_approach
        
        if not self.approaches:
            return None
        
        # Find approach with highest priority level
        max_priority = -1
        priority_approach = None
        
        for approach_id, approach in self.approaches.items():
            approach.update_approach_metrics()
            if approach.priority_level > max_priority:
                max_priority = approach.priority_level
                priority_approach = approach_id
        
        return priority_approach
    
    def should_extend_phase(self, max_waiting_time: int = 120) -> bool:
        """
        Determine if current phase should be extended
        Considers emergency vehicles and maximum waiting time constraint
        """
        current_time = time.time()
        
        # Emergency override - extend until vehicle passes
        if self.emergency_override_active:
            return True
        
        # Check 2-minute maximum waiting time constraint
        if self.phase_duration > max_waiting_time:
            return False
        
        # Check if any approach is critically congested
        for approach in self.approaches.values():
            if approach.avg_status > 0.8:  # Critical threshold
                # Don't extend if this approach has been waiting too long
                if approach_id := self.get_priority_approach():
                    if approach_id != self._get_current_green_approach():
                        return False
        
        return True
    
    def _get_current_green_approach(self) -> Optional[str]:
        """Helper to determine which approach currently has green light"""
        # This would need to be implemented based on phase mapping
        # For now, return None as placeholder
        return None