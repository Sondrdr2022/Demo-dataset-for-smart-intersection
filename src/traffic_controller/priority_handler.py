"""
Priority Handler Module

This module implements priority vehicle handling and smart left turn priority
as specified in the requirements.
"""

import time
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass
from enum import Enum

from ..models.traffic_state import IntersectionState, ApproachState, VehicleType
from ..simulation.sumo_interface import SUMOInterface
from ..utils.config_manager import config

class PriorityLevel(Enum):
    """Priority levels for different situations"""
    EMERGENCY_ABSOLUTE = "EMERGENCY_ABSOLUTE"  # Highest - ambulance, fire, police
    LEFT_TURN_SMART = "LEFT_TURN_SMART"       # Medium - smart left turn optimization
    NORMAL = "NORMAL"                         # Lowest - normal traffic flow

@dataclass
class EmergencyEvent:
    """Represents an emergency vehicle event"""
    vehicle_id: str
    vehicle_type: str
    detection_time: float
    approach_id: str
    intersection_id: str
    priority_start_time: float
    completed: bool = False
    completion_time: Optional[float] = None

@dataclass
class LeftTurnEvent:
    """Represents a left turn priority event"""
    approach_id: str
    intersection_id: str
    start_time: float
    blocked_duration: float
    lanes_involved: List[str]
    other_approaches_status: Dict[str, float]
    granted: bool = False
    grant_time: Optional[float] = None

class EmergencyVehiclePriorityHandler:
    """
    Handles emergency vehicle priority with absolute priority system:
    - When ambulance_detected = True, maintain green light until vehicle completely passes
    - System override - temporarily suspend normal adjustments
    """
    
    def __init__(self, sumo_interface: SUMOInterface):
        self.sumo = sumo_interface
        self.emergency_events: List[EmergencyEvent] = []
        self.active_emergencies: Dict[str, EmergencyEvent] = {}  # intersection_id -> event
        
        # Configuration
        self.detection_range = config.priority.emergency_detection_range
        self.vehicle_types = config.priority.emergency_vehicle_types
        self.absolute_priority = config.priority.emergency_absolute_priority
        
        # State tracking
        self.total_emergency_events = 0
        self.total_priority_time = 0.0
        
    def detect_emergency_vehicles(self, intersection: IntersectionState) -> Dict[str, Any]:
        """
        Detect emergency vehicles in intersection approaches
        Returns detection status and vehicle information
        """
        detection_result = {
            'emergency_detected': False,
            'vehicles': [],
            'approaches_with_emergency': [],
            'highest_priority_approach': None,
            'total_emergency_vehicles': 0
        }
        
        emergency_vehicles = []
        
        for approach_id, approach in intersection.approaches.items():
            approach_has_emergency = False
            
            for lane_id, lane_state in approach.lanes.items():
                if lane_state.parameters.ambulance_detected:
                    # Get detailed vehicle information
                    emergency_vehicles_in_lane = self._get_emergency_vehicles_in_lane(lane_id)
                    
                    for vehicle_info in emergency_vehicles_in_lane:
                        emergency_vehicles.append({
                            'vehicle_id': vehicle_info['id'],
                            'vehicle_type': vehicle_info['type'],
                            'lane_id': lane_id,
                            'approach_id': approach_id,
                            'position': vehicle_info['position'],
                            'speed': vehicle_info['speed'],
                            'distance_to_intersection': vehicle_info['distance_to_intersection'],
                            'priority_weight': self._get_priority_weight(vehicle_info['type'])
                        })
                        approach_has_emergency = True
            
            if approach_has_emergency:
                detection_result['approaches_with_emergency'].append(approach_id)
        
        if emergency_vehicles:
            detection_result['emergency_detected'] = True
            detection_result['vehicles'] = emergency_vehicles
            detection_result['total_emergency_vehicles'] = len(emergency_vehicles)
            
            # Find highest priority approach (closest emergency vehicle)
            closest_vehicle = min(emergency_vehicles, 
                                key=lambda v: v['distance_to_intersection'])
            detection_result['highest_priority_approach'] = closest_vehicle['approach_id']
        
        return detection_result
    
    def handle_emergency_priority(self, intersection: IntersectionState, 
                                detection_result: Dict[str, Any]) -> Dict[str, Any]:
        """
        Handle emergency vehicle priority with absolute priority system
        """
        intersection_id = intersection.intersection_id
        current_time = time.time()
        
        if not detection_result['emergency_detected']:
            # Check if we need to complete any active emergency
            if intersection_id in self.active_emergencies:
                return self._complete_emergency_priority(intersection_id)
            return {'action': 'NO_EMERGENCY', 'active': False}
        
        # Emergency detected - implement absolute priority
        priority_approach = detection_result['highest_priority_approach']
        
        # Check if this is a new emergency event
        if intersection_id not in self.active_emergencies:
            # Start new emergency priority
            return self._start_emergency_priority(
                intersection, priority_approach, detection_result, current_time
            )
        else:
            # Continue existing emergency priority
            return self._continue_emergency_priority(
                intersection, priority_approach, detection_result, current_time
            )
    
    def _start_emergency_priority(self, intersection: IntersectionState, 
                                priority_approach: str, detection_result: Dict[str, Any],
                                current_time: float) -> Dict[str, Any]:
        """Start emergency priority for intersection"""
        intersection_id = intersection.intersection_id
        
        # Create emergency event
        emergency_vehicle = detection_result['vehicles'][0]  # Use closest vehicle
        event = EmergencyEvent(
            vehicle_id=emergency_vehicle['vehicle_id'],
            vehicle_type=emergency_vehicle['vehicle_type'],
            detection_time=current_time,
            approach_id=priority_approach,
            intersection_id=intersection_id,
            priority_start_time=current_time
        )
        
        # Activate emergency priority
        self.active_emergencies[intersection_id] = event
        self.emergency_events.append(event)
        self.total_emergency_events += 1
        
        # Set intersection emergency state
        intersection.emergency_detected = True
        intersection.emergency_approach = priority_approach
        intersection.emergency_override_active = True
        intersection.emergency_start_time = current_time
        
        print(f"🚨 EMERGENCY PRIORITY STARTED: {intersection_id}")
        print(f"   Vehicle: {emergency_vehicle['vehicle_id']} ({emergency_vehicle['vehicle_type']})")
        print(f"   Approach: {priority_approach}")
        print(f"   Distance: {emergency_vehicle['distance_to_intersection']:.1f}m")
        
        return {
            'action': 'START_EMERGENCY_PRIORITY',
            'active': True,
            'approach': priority_approach,
            'vehicle_id': emergency_vehicle['vehicle_id'],
            'override_normal_control': True,
            'maintain_green_until_passed': True
        }
    
    def _continue_emergency_priority(self, intersection: IntersectionState,
                                   priority_approach: str, detection_result: Dict[str, Any],
                                   current_time: float) -> Dict[str, Any]:
        """Continue existing emergency priority"""
        intersection_id = intersection.intersection_id
        event = self.active_emergencies[intersection_id]
        
        # Check if emergency vehicle has completely passed
        if self._has_emergency_vehicle_passed(intersection, event, detection_result):
            return self._complete_emergency_priority(intersection_id)
        
        # Continue maintaining priority
        duration = current_time - event.priority_start_time
        
        return {
            'action': 'CONTINUE_EMERGENCY_PRIORITY',
            'active': True,
            'approach': priority_approach,
            'vehicle_id': event.vehicle_id,
            'duration': duration,
            'override_normal_control': True,
            'maintain_green_until_passed': True
        }
    
    def _complete_emergency_priority(self, intersection_id: str) -> Dict[str, Any]:
        """Complete emergency priority when vehicle has passed"""
        if intersection_id not in self.active_emergencies:
            return {'action': 'NO_EMERGENCY', 'active': False}
        
        event = self.active_emergencies[intersection_id]
        current_time = time.time()
        
        # Complete the event
        event.completed = True
        event.completion_time = current_time
        priority_duration = current_time - event.priority_start_time
        self.total_priority_time += priority_duration
        
        # Remove from active emergencies
        del self.active_emergencies[intersection_id]
        
        print(f"✅ EMERGENCY PRIORITY COMPLETED: {intersection_id}")
        print(f"   Duration: {priority_duration:.1f}s")
        print(f"   Vehicle: {event.vehicle_id}")
        
        return {
            'action': 'COMPLETE_EMERGENCY_PRIORITY',
            'active': False,
            'duration': priority_duration,
            'vehicle_id': event.vehicle_id,
            'resume_normal_control': True
        }
    
    def _has_emergency_vehicle_passed(self, intersection: IntersectionState,
                                    event: EmergencyEvent, 
                                    detection_result: Dict[str, Any]) -> bool:
        """
        Check if emergency vehicle has completely passed the intersection
        """
        # Check if the original emergency vehicle is still detected
        for vehicle in detection_result['vehicles']:
            if vehicle['vehicle_id'] == event.vehicle_id:
                # Vehicle still in detection range
                return False
        
        # Original vehicle not detected - check if it's in the intersection or past it
        try:
            # Try to get vehicle position from SUMO
            vehicle_info = self.sumo.get_all_vehicle_info()
            if event.vehicle_id in vehicle_info:
                vehicle_data = vehicle_info[event.vehicle_id]
                lane_id = vehicle_data['lane']
                
                # If vehicle is in internal lane (:) or past intersection, it's completing passage
                if ':' in lane_id or self._is_vehicle_past_intersection(vehicle_data, intersection):
                    return True
            else:
                # Vehicle no longer in simulation - has passed completely
                return True
                
        except Exception as e:
            print(f"Warning: Error checking emergency vehicle passage: {e}")
            # Fallback: assume passed after reasonable time
            if time.time() - event.priority_start_time > 60:  # 1 minute timeout
                return True
        
        return False
    
    def _is_vehicle_past_intersection(self, vehicle_data: Dict[str, Any], 
                                    intersection: IntersectionState) -> bool:
        """Check if vehicle has passed the intersection"""
        # Simplified check - in real implementation would need intersection geometry
        lane_id = vehicle_data['lane']
        
        # Check if vehicle is on an exit lane (not an approach lane)
        for approach in intersection.approaches.values():
            if lane_id in [lane.lane_id for lane in approach.lanes.values()]:
                return False  # Still on approach lane
        
        return True  # Not on any approach lane - likely past intersection
    
    def _get_emergency_vehicles_in_lane(self, lane_id: str) -> List[Dict[str, Any]]:
        """Get detailed information about emergency vehicles in a specific lane"""
        emergency_vehicles = []
        
        try:
            vehicle_ids = self.sumo.sumo.lane.getLastStepVehicleIDs(lane_id) if self.sumo.is_running else []
            
            for vehicle_id in vehicle_ids:
                try:
                    vehicle_type = self.sumo.sumo.vehicle.getTypeID(vehicle_id)
                    vehicle_class = self.sumo.sumo.vehicle.getVehicleClass(vehicle_id)
                    
                    # Check if it's an emergency vehicle
                    is_emergency = (
                        vehicle_class in self.vehicle_types or
                        any(emer_type in vehicle_type.lower() for emer_type in self.vehicle_types)
                    )
                    
                    if is_emergency:
                        position = self.sumo.sumo.vehicle.getLanePosition(vehicle_id)
                        speed = self.sumo.sumo.vehicle.getSpeed(vehicle_id)
                        lane_length = self.sumo.sumo.lane.getLength(lane_id)
                        
                        emergency_vehicles.append({
                            'id': vehicle_id,
                            'type': vehicle_type,
                            'class': vehicle_class,
                            'position': position,
                            'speed': speed,
                            'distance_to_intersection': lane_length - position
                        })
                        
                except Exception:
                    continue
                    
        except Exception as e:
            print(f"Warning: Error getting emergency vehicles in lane {lane_id}: {e}")
        
        return emergency_vehicles
    
    def _get_priority_weight(self, vehicle_type: str) -> int:
        """Get priority weight for vehicle type"""
        for vtype in VehicleType:
            if vtype.type_name in vehicle_type.lower():
                return vtype.weight
        return 3  # Default high priority for emergency vehicles

class SmartLeftTurnPriorityHandler:
    """
    Handles smart left turn priority:
    - If left turn blocked by other directions with no vehicles or red lights,
    - automatically switch left turn to green
    - Flexible control to reduce left turn waiting time
    """
    
    def __init__(self):
        self.left_turn_events: List[LeftTurnEvent] = []
        self.active_left_turns: Dict[str, LeftTurnEvent] = {}
        
        # Configuration
        self.max_wait_time = config.priority.left_turn_max_wait
        self.smart_priority_enabled = config.priority.left_turn_smart_priority
        
        # Thresholds for "clear" conditions
        self.clear_threshold = 0.1  # Status below this is considered "clear"
        self.min_wait_before_priority = 30  # Minimum wait before considering priority
        
    def evaluate_left_turn_priority(self, intersection: IntersectionState) -> Dict[str, Any]:
        """
        Evaluate left turn priority conditions for all approaches
        """
        if not self.smart_priority_enabled:
            return {'priority_needed': False, 'approaches': []}
        
        current_time = time.time()
        priority_recommendations = []
        
        for approach_id, approach in intersection.approaches.items():
            if approach.has_left_turn:
                priority_result = self._evaluate_approach_left_turn(
                    approach, intersection, current_time
                )
                
                if priority_result['priority_recommended']:
                    priority_recommendations.append(priority_result)
        
        return {
            'priority_needed': len(priority_recommendations) > 0,
            'approaches': priority_recommendations,
            'total_approaches_needing_priority': len(priority_recommendations)
        }
    
    def _evaluate_approach_left_turn(self, approach: ApproachState, 
                                   intersection: IntersectionState,
                                   current_time: float) -> Dict[str, Any]:
        """Evaluate left turn priority for a specific approach"""
        approach_id = approach.approach_id
        
        # Check if left turn has been waiting long enough
        if approach.left_turn_blocked_time < self.min_wait_before_priority:
            return {
                'approach_id': approach_id,
                'priority_recommended': False,
                'reason': 'Insufficient wait time',
                'wait_time': approach.left_turn_blocked_time
            }
        
        # Check if other approaches are clear (conditional green logic)
        other_approaches_status = self._evaluate_other_approaches(approach, intersection)
        
        all_others_clear = all(
            status < self.clear_threshold 
            for status in other_approaches_status.values()
        )
        
        if not all_others_clear:
            return {
                'approach_id': approach_id,
                'priority_recommended': False,
                'reason': 'Other approaches not clear',
                'wait_time': approach.left_turn_blocked_time,
                'other_approaches_status': other_approaches_status
            }
        
        # Left turn priority is recommended
        return {
            'approach_id': approach_id,
            'priority_recommended': True,
            'reason': 'All other approaches clear, long wait time',
            'wait_time': approach.left_turn_blocked_time,
            'other_approaches_status': other_approaches_status,
            'left_turn_lanes': approach.left_turn_lanes,
            'urgency': 'HIGH' if approach.left_turn_blocked_time > self.max_wait_time else 'MEDIUM'
        }
    
    def _evaluate_other_approaches(self, current_approach: ApproachState, 
                                 intersection: IntersectionState) -> Dict[str, float]:
        """Evaluate traffic status of other approaches"""
        other_statuses = {}
        
        for approach_id, approach in intersection.approaches.items():
            if approach_id != current_approach.approach_id:
                # Use average status of all lanes in approach
                if approach.lanes:
                    lane_statuses = []
                    for lane in approach.lanes.values():
                        # Simple status calculation based on queue and waiting time
                        lane_status = min(
                            (lane.parameters.l / 10.0) + (lane.parameters.td / 100.0),
                            1.0
                        )
                        lane_statuses.append(lane_status)
                    
                    other_statuses[approach_id] = sum(lane_statuses) / len(lane_statuses)
                else:
                    other_statuses[approach_id] = 0.0
        
        return other_statuses
    
    def implement_left_turn_priority(self, intersection: IntersectionState,
                                   priority_result: Dict[str, Any]) -> Dict[str, Any]:
        """
        Implement left turn priority for recommended approaches
        """
        if not priority_result['priority_needed']:
            return {'action': 'NO_LEFT_TURN_PRIORITY', 'implementations': []}
        
        implementations = []
        current_time = time.time()
        
        for approach_data in priority_result['approaches']:
            approach_id = approach_data['approach_id']
            
            # Create left turn event if not already active
            if approach_id not in self.active_left_turns:
                event = LeftTurnEvent(
                    approach_id=approach_id,
                    intersection_id=intersection.intersection_id,
                    start_time=current_time,
                    blocked_duration=approach_data['wait_time'],
                    lanes_involved=approach_data.get('left_turn_lanes', []),
                    other_approaches_status=approach_data['other_approaches_status']
                )
                
                self.active_left_turns[approach_id] = event
                self.left_turn_events.append(event)
            
            # Grant priority
            event = self.active_left_turns[approach_id]
            if not event.granted:
                event.granted = True
                event.grant_time = current_time
                
                implementations.append({
                    'approach_id': approach_id,
                    'action': 'GRANT_LEFT_TURN_PRIORITY',
                    'wait_time': approach_data['wait_time'],
                    'urgency': approach_data['urgency'],
                    'lanes': approach_data.get('left_turn_lanes', [])
                })
                
                print(f"🔄 LEFT TURN PRIORITY GRANTED: {approach_id}")
                print(f"   Wait time: {approach_data['wait_time']:.1f}s")
                print(f"   Urgency: {approach_data['urgency']}")
        
        return {
            'action': 'IMPLEMENT_LEFT_TURN_PRIORITY',
            'implementations': implementations,
            'total_granted': len(implementations)
        }
    
    def complete_left_turn_priority(self, approach_id: str) -> Dict[str, Any]:
        """Complete left turn priority when vehicles have passed"""
        if approach_id not in self.active_left_turns:
            return {'action': 'NO_ACTIVE_LEFT_TURN', 'approach_id': approach_id}
        
        event = self.active_left_turns[approach_id]
        current_time = time.time()
        
        priority_duration = current_time - (event.grant_time or event.start_time)
        
        # Remove from active left turns
        del self.active_left_turns[approach_id]
        
        print(f"✅ LEFT TURN PRIORITY COMPLETED: {approach_id}")
        print(f"   Total duration: {priority_duration:.1f}s")
        
        return {
            'action': 'COMPLETE_LEFT_TURN_PRIORITY',
            'approach_id': approach_id,
            'duration': priority_duration
        }

class PriorityManager:
    """
    Main priority management system coordinating emergency and left turn priorities
    """
    
    def __init__(self, sumo_interface: SUMOInterface):
        self.emergency_handler = EmergencyVehiclePriorityHandler(sumo_interface)
        self.left_turn_handler = SmartLeftTurnPriorityHandler()
        
        # Overall priority state
        self.active_priorities: Dict[str, Dict[str, Any]] = {}
        
    def evaluate_all_priorities(self, intersection: IntersectionState) -> Dict[str, Any]:
        """
        Evaluate all priority conditions for an intersection
        Emergency vehicles have absolute priority over left turns
        """
        intersection_id = intersection.intersection_id
        current_time = time.time()
        
        # 1. Check emergency vehicle priority (highest priority)
        emergency_detection = self.emergency_handler.detect_emergency_vehicles(intersection)
        
        if emergency_detection['emergency_detected']:
            emergency_result = self.emergency_handler.handle_emergency_priority(
                intersection, emergency_detection
            )
            
            # Emergency takes absolute priority
            self.active_priorities[intersection_id] = {
                'type': PriorityLevel.EMERGENCY_ABSOLUTE,
                'emergency': emergency_result,
                'left_turn': {'active': False},
                'timestamp': current_time
            }
            
            return {
                'priority_active': True,
                'priority_type': PriorityLevel.EMERGENCY_ABSOLUTE,
                'emergency': emergency_result,
                'left_turn': {'active': False},
                'override_normal_control': True
            }
        
        # 2. Check left turn priority (if no emergency)
        left_turn_evaluation = self.left_turn_handler.evaluate_left_turn_priority(intersection)
        
        if left_turn_evaluation['priority_needed']:
            left_turn_result = self.left_turn_handler.implement_left_turn_priority(
                intersection, left_turn_evaluation
            )
            
            self.active_priorities[intersection_id] = {
                'type': PriorityLevel.LEFT_TURN_SMART,
                'emergency': {'active': False},
                'left_turn': left_turn_result,
                'timestamp': current_time
            }
            
            return {
                'priority_active': True,
                'priority_type': PriorityLevel.LEFT_TURN_SMART,
                'emergency': {'active': False},
                'left_turn': left_turn_result,
                'override_normal_control': False  # Left turn can work with normal control
            }
        
        # 3. No priorities active
        if intersection_id in self.active_priorities:
            del self.active_priorities[intersection_id]
        
        return {
            'priority_active': False,
            'priority_type': PriorityLevel.NORMAL,
            'emergency': {'active': False},
            'left_turn': {'active': False},
            'override_normal_control': False
        }
    
    def get_priority_status(self) -> Dict[str, Any]:
        """Get overall priority system status"""
        return {
            'active_priorities': len(self.active_priorities),
            'total_emergency_events': self.emergency_handler.total_emergency_events,
            'total_left_turn_events': len(self.left_turn_handler.left_turn_events),
            'emergency_time': self.emergency_handler.total_priority_time,
            'active_emergencies': len(self.emergency_handler.active_emergencies),
            'active_left_turns': len(self.left_turn_handler.active_left_turns)
        }