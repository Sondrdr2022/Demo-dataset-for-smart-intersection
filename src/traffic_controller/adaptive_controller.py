"""
Adaptive Traffic Light Controller

This module implements the dynamic traffic light controller with 
adaptive timing based on real-time traffic conditions and scoring system.
"""

import time
import math
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from enum import Enum

from ..models.traffic_state import IntersectionState, ApproachState, TrafficState
from ..simulation.sumo_interface import SUMOInterface
from ..simulation.parameter_monitor import ParameterMonitor
from ..traffic_controller.scoring_system import CycleAggregator, DirectionScoringSystem
from ..utils.config_manager import config

class ControlAction(Enum):
    """Traffic light control actions"""
    MAINTAIN_CURRENT = "MAINTAIN"
    CHANGE_PHASE = "CHANGE"
    EXTEND_GREEN = "EXTEND"
    EMERGENCY_OVERRIDE = "EMERGENCY"
    OPTIMIZE_TIMING = "OPTIMIZE"

@dataclass
class PhaseChange:
    """Represents a phase change decision"""
    from_phase: int
    to_phase: int
    reason: str
    duration: Optional[float] = None
    emergency: bool = False

@dataclass 
class TimingAdjustment:
    """Represents a timing adjustment"""
    approach_id: str
    current_green_time: float
    new_green_time: float
    adjustment: float
    reason: str
    urgency: str

class AdaptiveController:
    """
    Main adaptive traffic light controller implementing:
    - Dynamic green/red light timing based on real-time conditions
    - Lane state management with real-time updates
    - Priority vehicle handling (ambulances/emergency)
    - Smart left turn priority
    - 2-minute maximum waiting time constraint
    """
    
    def __init__(self, sumo_interface: SUMOInterface, parameter_monitor: ParameterMonitor):
        self.sumo = sumo_interface
        self.monitor = parameter_monitor
        self.cycle_aggregator = CycleAggregator()
        self.direction_scorer = DirectionScoringSystem()
        
        # Control state
        self.is_active = False
        self.control_interval = config.traffic.eval_interval
        self.last_control_time = 0.0
        
        # Timing constraints
        self.min_green = config.traffic.min_green_time
        self.max_green = config.traffic.max_green_time
        self.yellow_time = config.traffic.yellow_time
        self.max_waiting_time = config.traffic.max_waiting_time
        
        # Phase management
        self.intersection_phases: Dict[str, Dict[str, Any]] = {}
        self.phase_changes: List[PhaseChange] = []
        self.timing_adjustments: List[TimingAdjustment] = []
        
        # Emergency handling
        self.emergency_active = False
        self.emergency_start_time = 0.0
        self.emergency_intersection = None
        
        # Performance tracking
        self.control_cycles = 0
        self.total_adjustments = 0
        self.emergency_events = 0
    
    def start_control(self) -> bool:
        """Start adaptive traffic light control"""
        try:
            if not self.sumo.is_running:
                print("❌ Cannot start control: SUMO simulation not running")
                return False
            
            if not self.monitor.monitoring_active:
                print("❌ Cannot start control: Parameter monitoring not active")
                return False
            
            # Initialize intersection phase tracking
            self._initialize_phase_tracking()
            
            self.is_active = True
            self.last_control_time = time.time()
            
            print("✓ Adaptive traffic light control started")
            return True
            
        except Exception as e:
            print(f"❌ Failed to start adaptive control: {e}")
            return False
    
    def stop_control(self):
        """Stop adaptive traffic light control"""
        self.is_active = False
        
        print(f"✓ Adaptive control stopped")
        print(f"  Control cycles: {self.control_cycles}")
        print(f"  Total adjustments: {self.total_adjustments}")
        print(f"  Emergency events: {self.emergency_events}")
    
    def update_control(self) -> bool:
        """
        Main control update loop - should be called regularly
        Implements the dynamic adjustment logic
        """
        if not self.is_active:
            return False
        
        current_time = time.time()
        
        # Check if it's time for control update
        if current_time - self.last_control_time < self.control_interval:
            return True
        
        try:
            # Update all traffic parameters first
            self.monitor.update_all_parameters()
            
            # Check for emergency vehicles (highest priority)
            emergency_status = self._check_emergency_vehicles()
            
            if emergency_status['emergency_detected']:
                self._handle_emergency_priority(emergency_status)
                return True
            
            # Process each intersection
            for tl_id, intersection in self.monitor.intersection_states.items():
                self._process_intersection_control(tl_id, intersection)
            
            self.control_cycles += 1
            self.last_control_time = current_time
            
            return True
            
        except Exception as e:
            print(f"❌ Error in control update: {e}")
            return False
    
    def _initialize_phase_tracking(self):
        """Initialize phase tracking for all intersections"""
        intersection_data = self.sumo.get_intersection_data()
        
        for tl_id in intersection_data.keys():
            tl_state = self.sumo.get_traffic_light_state(tl_id)
            
            self.intersection_phases[tl_id] = {
                'current_phase': tl_state.get('current_phase', 0),
                'phase_start_time': time.time(),
                'green_time': self.min_green,
                'cycle_time': config.traffic.min_cycle_time,
                'last_adjustment': 0.0,
                'approaches': {},
                'emergency_override': False
            }
            
            # Initialize approach timing
            if tl_id in self.monitor.intersection_states:
                intersection = self.monitor.intersection_states[tl_id]
                for approach_id in intersection.approaches:
                    self.intersection_phases[tl_id]['approaches'][approach_id] = {
                        'green_time': self.min_green,
                        'last_green_start': 0.0,
                        'total_waiting_time': 0.0,
                        'priority_level': 0.0
                    }
    
    def _check_emergency_vehicles(self) -> Dict[str, Any]:
        """Check for emergency vehicles across all intersections"""
        emergency_status = {
            'emergency_detected': False,
            'intersections': {},
            'total_emergencies': 0
        }
        
        for tl_id, intersection in self.monitor.intersection_states.items():
            intersection.detect_emergency_vehicles()
            
            emergency_status['intersections'][tl_id] = {
                'emergency_detected': intersection.emergency_detected,
                'emergency_approach': intersection.emergency_approach,
                'override_active': intersection.emergency_override_active
            }
            
            if intersection.emergency_detected:
                emergency_status['emergency_detected'] = True
                emergency_status['total_emergencies'] += 1
        
        return emergency_status
    
    def _handle_emergency_priority(self, emergency_status: Dict[str, Any]):
        """
        Handle emergency vehicle priority - absolute priority system
        Maintain green light until vehicle completely passes
        """
        for tl_id, status in emergency_status['intersections'].items():
            if status['emergency_detected']:
                intersection = self.monitor.intersection_states[tl_id]
                
                if not intersection.emergency_override_active:
                    # Start emergency override
                    intersection.emergency_override_active = True
                    intersection.emergency_start_time = time.time()
                    self.emergency_events += 1
                    
                    print(f"🚨 EMERGENCY OVERRIDE: {tl_id} approach {status['emergency_approach']}")
                
                # Implement absolute priority
                self._implement_emergency_override(tl_id, intersection, status['emergency_approach'])
    
    def _implement_emergency_override(self, tl_id: str, intersection: IntersectionState, emergency_approach: str):
        """
        Implement emergency override with absolute priority
        """
        try:
            # Find the appropriate phase for emergency approach
            emergency_phase = self._get_emergency_phase(tl_id, emergency_approach)
            
            if emergency_phase is not None:
                current_phase = intersection.current_phase
                
                if current_phase != emergency_phase:
                    # Change to emergency phase immediately
                    self.sumo.set_traffic_light_phase(tl_id, emergency_phase)
                    
                    phase_change = PhaseChange(
                        from_phase=current_phase,
                        to_phase=emergency_phase,
                        reason=f"Emergency vehicle priority - approach {emergency_approach}",
                        emergency=True
                    )
                    self.phase_changes.append(phase_change)
                    
                    print(f"🚨 Emergency phase change: {tl_id} phase {current_phase} → {emergency_phase}")
                
                # Extend green time indefinitely until vehicle passes
                extended_duration = 120.0  # Maximum 2 minutes
                self.sumo.set_traffic_light_phase(tl_id, emergency_phase, extended_duration)
                
                # Update intersection state
                intersection.current_phase = emergency_phase
                intersection.phase_start_time = time.time()
                
        except Exception as e:
            print(f"❌ Error implementing emergency override for {tl_id}: {e}")
    
    def _get_emergency_phase(self, tl_id: str, emergency_approach: str) -> Optional[int]:
        """
        Determine the appropriate phase for emergency approach
        This is a simplified implementation - real implementation would need
        to map approaches to traffic light phases based on intersection geometry
        """
        # Simplified mapping - in real implementation this would be based on
        # actual traffic light program and intersection layout
        approach_to_phase = {
            'north': 0,
            'south': 0, 
            'east': 1,
            'west': 1
        }
        
        # Try to match approach direction
        for direction, phase in approach_to_phase.items():
            if direction in emergency_approach.lower():
                return phase
        
        # Default to phase 0
        return 0
    
    def _process_intersection_control(self, tl_id: str, intersection: IntersectionState):
        """
        Process control logic for a single intersection
        """
        # Skip if emergency override is active
        if intersection.emergency_override_active:
            return
        
        # Get current phase information
        phase_info = self.intersection_phases.get(tl_id, {})
        current_time = time.time()
        
        # Calculate cycle scores and get timing recommendations
        cycle_results = self.cycle_aggregator.aggregate_cycle_scores(intersection)
        recommendations = self.cycle_aggregator.get_timing_recommendations(intersection)
        
        # Check for phase change conditions
        phase_change_needed = self._evaluate_phase_change_conditions(
            tl_id, intersection, phase_info, recommendations
        )
        
        if phase_change_needed:
            self._implement_phase_change(tl_id, intersection, recommendations)
        else:
            # Apply timing adjustments for current phase
            self._apply_timing_adjustments(tl_id, intersection, recommendations)
        
        # Check maximum waiting time constraint (2-minute rule)
        self._enforce_max_waiting_constraint(tl_id, intersection)
        
        # Update phase tracking
        self._update_phase_tracking(tl_id, intersection, phase_info)
    
    def _evaluate_phase_change_conditions(self, tl_id: str, intersection: IntersectionState, 
                                        phase_info: Dict[str, Any], 
                                        recommendations: Dict[str, Any]) -> bool:
        """
        Evaluate whether a phase change is needed
        """
        current_time = time.time()
        phase_duration = current_time - intersection.phase_start_time
        
        # Minimum phase duration enforcement
        if phase_duration < self.min_green:
            return False
        
        # Maximum waiting time constraint
        if phase_duration > self.max_waiting_time:
            return True
        
        # Priority-based phase change
        priority_approach = recommendations.get('priority_approach')
        
        if priority_approach:
            current_green_approach = self._get_current_green_approach(tl_id, intersection)
            
            # Change if priority approach is not currently green and has high priority
            if (priority_approach != current_green_approach and 
                recommendations['recommendations'].get(priority_approach, {}).get('urgency') == 'HIGH'):
                return True
        
        # Left turn priority conditions
        if self._check_left_turn_priority_conditions(intersection):
            return True
        
        return False
    
    def _implement_phase_change(self, tl_id: str, intersection: IntersectionState, 
                              recommendations: Dict[str, Any]):
        """
        Implement phase change based on recommendations
        """
        try:
            priority_approach = recommendations.get('priority_approach')
            
            if priority_approach:
                new_phase = self._get_phase_for_approach(tl_id, priority_approach)
                current_phase = intersection.current_phase
                
                if new_phase is not None and new_phase != current_phase:
                    # Implement phase change with yellow transition
                    self._safe_phase_change(tl_id, current_phase, new_phase, priority_approach)
                    
                    # Record phase change
                    phase_change = PhaseChange(
                        from_phase=current_phase,
                        to_phase=new_phase,
                        reason=f"Priority to {priority_approach}",
                        emergency=False
                    )
                    self.phase_changes.append(phase_change)
                    
                    # Update intersection state
                    intersection.current_phase = new_phase
                    intersection.phase_start_time = time.time()
                    
                    print(f"⚡ Phase change: {tl_id} {current_phase} → {new_phase} (priority: {priority_approach})")
                    
        except Exception as e:
            print(f"❌ Error implementing phase change for {tl_id}: {e}")
    
    def _safe_phase_change(self, tl_id: str, from_phase: int, to_phase: int, reason: str):
        """
        Implement safe phase change with yellow transition
        """
        # Insert yellow phase for safety
        yellow_phase = from_phase + 1 if from_phase % 2 == 0 else from_phase  # Simplified logic
        
        # Set yellow phase first
        self.sumo.set_traffic_light_phase(tl_id, yellow_phase, self.yellow_time)
        
        # Schedule green phase after yellow
        # Note: In a real implementation, this would need proper scheduling
        # For now, we'll set the target phase immediately with yellow duration
        time.sleep(0.1)  # Small delay to ensure yellow is set
        self.sumo.set_traffic_light_phase(tl_id, to_phase)
    
    def _apply_timing_adjustments(self, tl_id: str, intersection: IntersectionState,
                                recommendations: Dict[str, Any]):
        """
        Apply timing adjustments for current green phase
        """
        current_green_approach = self._get_current_green_approach(tl_id, intersection)
        
        if current_green_approach and current_green_approach in recommendations['recommendations']:
            adjustment_data = recommendations['recommendations'][current_green_approach]
            green_time_change = adjustment_data.get('green_time_change', 0)
            
            if abs(green_time_change) >= 2:  # Only apply significant adjustments
                phase_info = self.intersection_phases[tl_id]
                current_green = phase_info['green_time']
                new_green = max(self.min_green, min(self.max_green, current_green + green_time_change))
                
                if new_green != current_green:
                    # Apply timing adjustment
                    remaining_time = new_green
                    self.sumo.set_traffic_light_phase(tl_id, intersection.current_phase, remaining_time)
                    
                    # Record adjustment
                    adjustment = TimingAdjustment(
                        approach_id=current_green_approach,
                        current_green_time=current_green,
                        new_green_time=new_green,
                        adjustment=green_time_change,
                        reason=adjustment_data.get('reason', 'Adaptive timing'),
                        urgency=adjustment_data.get('urgency', 'MEDIUM')
                    )
                    self.timing_adjustments.append(adjustment)
                    
                    # Update phase info
                    phase_info['green_time'] = new_green
                    self.total_adjustments += 1
                    
                    print(f"⏱️  Timing adjustment: {tl_id} {current_green_approach} "
                          f"{current_green}s → {new_green}s ({green_time_change:+d}s)")
    
    def _enforce_max_waiting_constraint(self, tl_id: str, intersection: IntersectionState):
        """
        Enforce 2-minute maximum waiting time constraint
        """
        current_time = time.time()
        phase_duration = current_time - intersection.phase_start_time
        
        if phase_duration > self.max_waiting_time:
            # Force phase change due to maximum waiting time
            next_phase = (intersection.current_phase + 1) % 4  # Simplified cycle
            
            print(f"⏰ MAX WAIT EXCEEDED: {tl_id} forcing phase change after {phase_duration:.1f}s")
            
            self._safe_phase_change(tl_id, intersection.current_phase, next_phase, 
                                  "Maximum waiting time exceeded")
            
            # Record forced change
            phase_change = PhaseChange(
                from_phase=intersection.current_phase,
                to_phase=next_phase,
                reason="Maximum waiting time constraint (2 minutes)",
                emergency=False
            )
            self.phase_changes.append(phase_change)
            
            # Update intersection state
            intersection.current_phase = next_phase
            intersection.phase_start_time = current_time
    
    def _check_left_turn_priority_conditions(self, intersection: IntersectionState) -> bool:
        """
        Check smart left turn priority conditions
        If left turn blocked by other directions with no vehicles or red lights,
        automatically switch left turn to green
        """
        if not config.priority.left_turn_smart_priority:
            return False
        
        current_time = time.time()
        
        for approach in intersection.approaches.values():
            if approach.has_left_turn and approach.left_turn_blocked_time > config.priority.left_turn_max_wait:
                # Check if other directions are clear
                other_approaches_clear = True
                
                for other_approach in intersection.approaches.values():
                    if other_approach != approach:
                        if other_approach.avg_status > 0.1:  # Has vehicles waiting
                            other_approaches_clear = False
                            break
                
                if other_approaches_clear:
                    print(f"🔄 Left turn priority triggered for {approach.approach_id}")
                    return True
        
        return False
    
    def _get_current_green_approach(self, tl_id: str, intersection: IntersectionState) -> Optional[str]:
        """
        Determine which approach currently has green light
        This is a simplified implementation
        """
        # Simplified mapping based on phase
        phase_to_approach = {
            0: 'north_south',
            1: 'east_west',
            2: 'north_south',
            3: 'east_west'
        }
        
        phase_approach = phase_to_approach.get(intersection.current_phase)
        
        # Find matching approach
        for approach_id in intersection.approaches:
            if phase_approach and phase_approach in approach_id.lower():
                return approach_id
        
        return None
    
    def _get_phase_for_approach(self, tl_id: str, approach_id: str) -> Optional[int]:
        """
        Get appropriate phase for given approach
        This is a simplified implementation
        """
        # Simplified mapping
        if any(direction in approach_id.lower() for direction in ['north', 'south']):
            return 0
        elif any(direction in approach_id.lower() for direction in ['east', 'west']):
            return 2
        else:
            return 0
    
    def _update_phase_tracking(self, tl_id: str, intersection: IntersectionState, 
                             phase_info: Dict[str, Any]):
        """
        Update phase tracking information
        """
        current_time = time.time()
        
        # Update approach waiting times
        for approach_id, approach in intersection.approaches.items():
            if approach_id in phase_info.get('approaches', {}):
                approach_info = phase_info['approaches'][approach_id]
                
                # Update total waiting time if not currently green
                current_green = self._get_current_green_approach(tl_id, intersection)
                if approach_id != current_green:
                    approach_info['total_waiting_time'] += self.control_interval
                else:
                    approach_info['last_green_start'] = current_time
                    approach_info['total_waiting_time'] = 0.0
    
    def get_control_status(self) -> Dict[str, Any]:
        """Get current control system status"""
        return {
            'active': self.is_active,
            'control_cycles': self.control_cycles,
            'total_adjustments': self.total_adjustments,
            'emergency_events': self.emergency_events,
            'emergency_active': self.emergency_active,
            'last_control_time': self.last_control_time,
            'intersections_controlled': len(self.intersection_phases),
            'recent_phase_changes': self.phase_changes[-10:] if self.phase_changes else [],
            'recent_timing_adjustments': self.timing_adjustments[-10:] if self.timing_adjustments else []
        }