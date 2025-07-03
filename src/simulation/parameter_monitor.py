"""
Parameter Monitor Module

This module handles real-time monitoring and collection of traffic parameters
implementing the comprehensive parameter tracking system specified in requirements.
"""

import time
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass, field
from collections import defaultdict, deque

from ..models.traffic_state import TrafficParameters, LaneState, ApproachState, IntersectionState
from ..simulation.sumo_interface import SUMOInterface
from ..utils.config_manager import config

@dataclass
class MonitoringSession:
    """Represents a single monitoring session with historical data"""
    session_id: str
    start_time: float
    parameters_log: List[Tuple[float, str, TrafficParameters]] = field(default_factory=list)
    events_log: List[Tuple[float, str, str, Any]] = field(default_factory=list)  # time, event_type, lane_id, data
    
class ParameterMonitor:
    """
    Real-time traffic parameter monitoring system
    Implements comprehensive tracking of: t, l, tđ, c, m, v, g, ambulance_detected
    """
    
    def __init__(self, sumo_interface: SUMOInterface):
        self.sumo = sumo_interface
        self.monitoring_active = False
        self.last_update_time = 0.0
        
        # Intersection and lane tracking
        self.intersection_states: Dict[str, IntersectionState] = {}
        self.lane_states: Dict[str, LaneState] = {}
        self.approach_states: Dict[str, ApproachState] = {}
        
        # Historical data and statistics
        self.parameter_history: Dict[str, deque] = defaultdict(lambda: deque(maxlen=1000))
        self.max_values: Dict[str, float] = {
            'l': 20.0,    # Max queue length
            'td': 300.0,  # Max waiting time (5 minutes)
            'm': 1.0,     # Max density (normalized)
            'v': 50.0,    # Max speed (50 m/s)
            'g': 3600.0   # Max flow (vehicles/hour)
        }
        
        # Monitoring session
        self.current_session = None
        
        # Performance tracking
        self.update_count = 0
        self.total_update_time = 0.0
        
    def start_monitoring(self, session_id: str = None) -> bool:
        """
        Start parameter monitoring session
        """
        try:
            if not self.sumo.is_running:
                print("❌ Cannot start monitoring: SUMO simulation not running")
                return False
            
            session_id = session_id or f"session_{int(time.time())}"
            self.current_session = MonitoringSession(
                session_id=session_id,
                start_time=time.time()
            )
            
            # Initialize intersection structures
            self._initialize_intersection_states()
            
            self.monitoring_active = True
            self.last_update_time = time.time()
            
            print(f"✓ Parameter monitoring started: {session_id}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to start monitoring: {e}")
            return False
    
    def stop_monitoring(self):
        """Stop parameter monitoring and save session data"""
        if self.monitoring_active:
            self.monitoring_active = False
            
            if self.current_session:
                duration = time.time() - self.current_session.start_time
                print(f"✓ Monitoring stopped. Session: {self.current_session.session_id}")
                print(f"  Duration: {duration:.1f}s, Updates: {self.update_count}")
                print(f"  Avg update time: {self.total_update_time/self.update_count*1000:.2f}ms")
    
    def update_all_parameters(self) -> bool:
        """
        Update all traffic parameters for all monitored lanes and intersections
        This is the main monitoring loop function
        """
        if not self.monitoring_active or not self.sumo.is_running:
            return False
        
        start_time = time.time()
        
        try:
            current_sim_time = self.sumo.get_simulation_time()
            
            # Update all lane states
            updated_lanes = self._update_lane_parameters()
            
            # Update approach states
            self._update_approach_states()
            
            # Update intersection states
            self._update_intersection_states()
            
            # Check for emergency vehicles
            self._check_emergency_vehicles()
            
            # Update historical data
            self._update_historical_data(current_sim_time)
            
            # Performance tracking
            self.update_count += 1
            update_time = time.time() - start_time
            self.total_update_time += update_time
            self.last_update_time = time.time()
            
            return True
            
        except Exception as e:
            print(f"❌ Error updating parameters: {e}")
            return False
    
    def _initialize_intersection_states(self):
        """Initialize intersection, approach, and lane state objects"""
        intersection_data = self.sumo.get_intersection_data()
        
        for tl_id, tl_data in intersection_data.items():
            # Create intersection state
            intersection = IntersectionState(
                intersection_id=tl_id,
                traffic_light_ids=[tl_id]
            )
            
            # Create approach and lane states
            for approach_id, approach_data in tl_data['approaches'].items():
                # Create approach state
                approach = ApproachState(
                    approach_id=f"{tl_id}_{approach_id}",
                    direction=self._determine_direction(approach_id)
                )
                
                # Create lane states for this approach
                for lane_id in approach_data['lanes']:
                    detector_id = tl_data['lane_to_detector'].get(lane_id)
                    
                    lane_state = LaneState(
                        lane_id=lane_id,
                        edge_id=approach_id,
                        detector_id=detector_id
                    )
                    
                    self.lane_states[lane_id] = lane_state
                    approach.lanes[lane_id] = lane_state
                
                self.approach_states[approach.approach_id] = approach
                intersection.approaches[approach.approach_id] = approach
            
            self.intersection_states[tl_id] = intersection
        
        print(f"Initialized monitoring for {len(self.intersection_states)} intersections, "
              f"{len(self.approach_states)} approaches, {len(self.lane_states)} lanes")
    
    def _determine_direction(self, approach_id: str) -> str:
        """Determine cardinal direction from approach ID"""
        approach_lower = approach_id.lower()
        if any(x in approach_lower for x in ['north', 'n', 'top']):
            return 'north'
        elif any(x in approach_lower for x in ['south', 's', 'bottom']):
            return 'south'
        elif any(x in approach_lower for x in ['east', 'e', 'right']):
            return 'east'
        elif any(x in approach_lower for x in ['west', 'w', 'left']):
            return 'west'
        else:
            return f'direction_{approach_id}'
    
    def _update_lane_parameters(self) -> List[str]:
        """Update traffic parameters for all monitored lanes"""
        updated_lanes = []
        
        for lane_id, lane_state in self.lane_states.items():
            try:
                # Get fresh parameters from SUMO
                new_params = self.sumo.get_lane_traffic_parameters(
                    lane_id, lane_state.detector_id
                )
                
                # Update lane state
                lane_state.update_parameters(new_params)
                
                # Calculate status and update score
                status_t = lane_state.calculate_status_t(
                    config.traffic.parameter_weights,
                    self.max_values
                )
                
                # Update lane score with continuous rewards
                reward = lane_state.update_lane_score(
                    status_t, 
                    config.traffic.status_threshold
                )
                
                # Log significant events
                if abs(reward) >= 3:  # Significant reward/penalty
                    self._log_event('significant_score_change', lane_id, {
                        'reward': reward,
                        'status_t': status_t,
                        'lane_score': lane_state.lane_score,
                        'state': lane_state.current_state.value
                    })
                
                updated_lanes.append(lane_id)
                
                # Update max values for normalization
                self._update_max_values(new_params)
                
            except Exception as e:
                print(f"Warning: Error updating lane {lane_id}: {e}")
        
        return updated_lanes
    
    def _update_approach_states(self):
        """Update approach-level metrics and priorities"""
        for approach_id, approach in self.approach_states.items():
            try:
                approach.update_approach_metrics()
                
                # Check for left turn management
                if approach.has_left_turn:
                    self._check_left_turn_priority(approach)
                
            except Exception as e:
                print(f"Warning: Error updating approach {approach_id}: {e}")
    
    def _update_intersection_states(self):
        """Update intersection-level states and control decisions"""
        for tl_id, intersection in self.intersection_states.items():
            try:
                # Update traffic light state information
                tl_state = self.sumo.get_traffic_light_state(tl_id)
                if tl_state:
                    current_time = time.time()
                    
                    # Check for phase changes
                    if intersection.current_phase != tl_state.get('current_phase', 0):
                        intersection.phase_start_time = current_time
                        intersection.current_phase = tl_state.get('current_phase', 0)
                        intersection.cycle_count += 1
                    
                    # Update phase duration
                    intersection.phase_duration = current_time - intersection.phase_start_time
                
                # Detect emergency vehicles
                intersection.detect_emergency_vehicles()
                
                # Update performance metrics
                self._update_intersection_performance(intersection)
                
            except Exception as e:
                print(f"Warning: Error updating intersection {tl_id}: {e}")
    
    def _check_emergency_vehicles(self):
        """Check for emergency vehicles across all intersections"""
        for intersection in self.intersection_states.values():
            if intersection.detect_emergency_vehicles():
                self._log_event('emergency_detected', intersection.intersection_id, {
                    'approach': intersection.emergency_approach,
                    'override_active': intersection.emergency_override_active
                })
    
    def _check_left_turn_priority(self, approach: ApproachState):
        """Check and manage smart left turn priority"""
        if not approach.has_left_turn:
            return
        
        current_time = time.time()
        
        # Check if left turn has been blocked too long
        if approach.left_turn_blocked_time > config.priority.left_turn_max_wait:
            self._log_event('left_turn_priority_needed', approach.approach_id, {
                'blocked_time': approach.left_turn_blocked_time,
                'lanes': approach.left_turn_lanes
            })
    
    def _update_max_values(self, params: TrafficParameters):
        """Update maximum values for parameter normalization"""
        self.max_values['l'] = max(self.max_values['l'], params.l)
        self.max_values['td'] = max(self.max_values['td'], params.td)
        self.max_values['m'] = max(self.max_values['m'], params.m)
        self.max_values['v'] = max(self.max_values['v'], params.v)
        self.max_values['g'] = max(self.max_values['g'], params.g)
    
    def _update_historical_data(self, sim_time: float):
        """Update historical parameter data"""
        if not self.current_session:
            return
        
        # Store parameters for all lanes
        for lane_id, lane_state in self.lane_states.items():
            self.parameter_history[lane_id].append((sim_time, lane_state.parameters))
            
            # Add to session log
            self.current_session.parameters_log.append((
                sim_time, lane_id, lane_state.parameters
            ))
    
    def _update_intersection_performance(self, intersection: IntersectionState):
        """Update intersection performance metrics"""
        total_waiting = 0.0
        total_vehicles = 0
        
        for approach in intersection.approaches.values():
            for lane in approach.lanes.values():
                total_waiting += lane.parameters.td * lane.parameters.raw_count
                total_vehicles += lane.parameters.raw_count
        
        intersection.total_waiting_time = total_waiting
        intersection.total_vehicles_served = total_vehicles
        intersection.average_delay = total_waiting / total_vehicles if total_vehicles > 0 else 0.0
    
    def _log_event(self, event_type: str, lane_id: str, data: Any):
        """Log significant events during monitoring"""
        if self.current_session:
            self.current_session.events_log.append((
                time.time(), event_type, lane_id, data
            ))
    
    def get_lane_state(self, lane_id: str) -> Optional[LaneState]:
        """Get current state of specific lane"""
        return self.lane_states.get(lane_id)
    
    def get_approach_state(self, approach_id: str) -> Optional[ApproachState]:
        """Get current state of specific approach"""
        return self.approach_states.get(approach_id)
    
    def get_intersection_state(self, intersection_id: str) -> Optional[IntersectionState]:
        """Get current state of specific intersection"""
        return self.intersection_states.get(intersection_id)
    
    def get_emergency_status(self) -> Dict[str, Any]:
        """Get emergency vehicle status across all intersections"""
        emergency_status = {}
        
        for tl_id, intersection in self.intersection_states.items():
            emergency_status[tl_id] = {
                'emergency_detected': intersection.emergency_detected,
                'emergency_approach': intersection.emergency_approach,
                'override_active': intersection.emergency_override_active,
                'emergency_start_time': intersection.emergency_start_time
            }
        
        return emergency_status
    
    def get_performance_summary(self) -> Dict[str, Any]:
        """Get performance summary for all intersections"""
        summary = {
            'total_intersections': len(self.intersection_states),
            'total_approaches': len(self.approach_states),
            'total_lanes': len(self.lane_states),
            'monitoring_duration': time.time() - (self.current_session.start_time if self.current_session else time.time()),
            'update_frequency': self.update_count / max((time.time() - self.last_update_time), 1),
            'intersections': {}
        }
        
        for tl_id, intersection in self.intersection_states.items():
            summary['intersections'][tl_id] = {
                'total_waiting_time': intersection.total_waiting_time,
                'total_vehicles_served': intersection.total_vehicles_served,
                'average_delay': intersection.average_delay,
                'emergency_detected': intersection.emergency_detected,
                'cycle_count': intersection.cycle_count,
                'current_phase': intersection.current_phase,
                'phase_duration': intersection.phase_duration
            }
        
        return summary