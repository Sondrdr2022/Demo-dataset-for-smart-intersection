"""
SUMO Interface Module

This module provides a comprehensive interface to SUMO traffic simulation
with TraCI integration for real-time traffic monitoring and control.
"""

import os
import sys
import time
import subprocess
from typing import Dict, List, Optional, Tuple, Any
import traci
from dataclasses import dataclass

from ..utils.config_manager import config
from ..models.traffic_state import TrafficParameters, VehicleType

class SUMOInterface:
    """
    Main interface for SUMO simulation with TraCI integration
    Handles simulation startup, traffic monitoring, and control
    """
    
    def __init__(self, config_file: Optional[str] = None, use_gui: bool = True):
        self.config_file = config_file or config.sumo.config_file
        self.use_gui = use_gui
        self.is_running = False
        self.simulation_step = 0
        self.start_time = 0.0
        
        # Traffic monitoring data
        self.intersection_data = {}
        self.detector_lane_mapping = {}
        
    def start_simulation(self) -> bool:
        """
        Start SUMO simulation with TraCI
        Returns True if successful, False otherwise
        """
        try:
            # Determine SUMO binary
            binary_name = "sumo-gui" if self.use_gui else "sumo"
            sumo_binary = os.path.join(config.sumo.sumo_home, 'bin', binary_name)
            
            # Check if config file exists
            if not os.path.exists(self.config_file):
                print(f"Warning: Config file {self.config_file} not found. Using default.")
                self.config_file = "dataset.sumocfg"  # Fallback to existing file
            
            # SUMO command arguments
            sumo_cmd = [
                sumo_binary,
                '-c', self.config_file,
                '--step-length', str(config.sumo.step_length),
                '--waiting-time-memory', '1000',  # Remember waiting times
                '--time-to-teleport', '-1'  # Disable teleporting
            ]
            
            print(f"Starting SUMO simulation with: {' '.join(sumo_cmd)}")
            traci.start(sumo_cmd)
            
            self.is_running = True
            self.start_time = time.time()
            self.simulation_step = 0
            
            # Initialize intersection detection
            self._detect_intersection_structure()
            
            print("✓ SUMO simulation started successfully")
            return True
            
        except Exception as e:
            print(f"❌ Failed to start SUMO simulation: {e}")
            return False
    
    def stop_simulation(self):
        """Stop SUMO simulation and clean up"""
        try:
            if self.is_running:
                traci.close()
                self.is_running = False
                print("✓ SUMO simulation stopped")
        except Exception as e:
            print(f"Warning: Error stopping SUMO: {e}")
    
    def step(self) -> bool:
        """
        Execute one simulation step
        Returns True if simulation continues, False if ended
        """
        try:
            if not self.is_running:
                return False
                
            traci.simulationStep()
            self.simulation_step += 1
            
            # Check if simulation ended
            if traci.simulation.getMinExpectedNumber() <= 0:
                return False
                
            return True
            
        except Exception as e:
            print(f"❌ Error in simulation step: {e}")
            return False
    
    def get_simulation_time(self) -> float:
        """Get current simulation time in seconds"""
        try:
            return traci.simulation.getTime()
        except:
            return 0.0
    
    def _detect_intersection_structure(self):
        """
        Auto-detect intersection structure, traffic lights, and detectors
        """
        try:
            # Get all traffic lights
            tl_ids = traci.trafficlight.getIDList()
            
            # Get all detectors
            detector_ids = []
            try:
                detector_ids = traci.lanearea.getIDList()
            except:
                try:
                    detector_ids = traci.inductionloop.getIDList()
                except:
                    print("No detectors found in simulation")
            
            # Get all lanes
            lane_ids = traci.lane.getIDList()
            
            print(f"Found {len(tl_ids)} traffic lights, {len(detector_ids)} detectors, {len(lane_ids)} lanes")
            
            # Process each traffic light
            for tl_id in tl_ids:
                controlled_links = traci.trafficlight.getControlledLinks(tl_id)
                controlled_lanes = traci.trafficlight.getControlledLanes(tl_id)
                
                # Group lanes by approach (edge)
                approaches = {}
                lane_to_detector = {}
                
                # Map detectors to lanes
                for detector_id in detector_ids:
                    try:
                        if hasattr(traci, 'lanearea'):
                            detector_lane = traci.lanearea.getLaneID(detector_id)
                        else:
                            detector_lane = traci.inductionloop.getLaneID(detector_id)
                            
                        if detector_lane in controlled_lanes:
                            lane_to_detector[detector_lane] = detector_id
                    except:
                        continue
                
                # Group controlled lanes into approaches
                for lane_id in set(controlled_lanes):
                    if ':' not in lane_id:  # Skip internal lanes
                        # Extract edge ID (approach direction)
                        edge_id = lane_id.split('_')[0] if '_' in lane_id else lane_id.split('#')[0]
                        
                        if edge_id not in approaches:
                            approaches[edge_id] = {
                                'lanes': [],
                                'detectors': [],
                                'lane_count': 0
                            }
                        
                        approaches[edge_id]['lanes'].append(lane_id)
                        approaches[edge_id]['lane_count'] += 1
                        
                        if lane_id in lane_to_detector:
                            approaches[edge_id]['detectors'].append(lane_to_detector[lane_id])
                
                # Store intersection data
                self.intersection_data[tl_id] = {
                    'approaches': approaches,
                    'controlled_links': controlled_links,
                    'controlled_lanes': controlled_lanes,
                    'lane_to_detector': lane_to_detector,
                    'total_approaches': len(approaches),
                    'total_lanes': len(set(controlled_lanes))
                }
                
                print(f"Traffic light {tl_id}: {len(approaches)} approaches, {len(set(controlled_lanes))} lanes")
                
        except Exception as e:
            print(f"❌ Error detecting intersection structure: {e}")
    
    def get_lane_traffic_parameters(self, lane_id: str, detector_id: Optional[str] = None) -> TrafficParameters:
        """
        Collect comprehensive traffic parameters for a lane
        Implements the parameter tracking system: t, l, tđ, c, m, v, g, ambulance_detected
        """
        params = TrafficParameters()
        
        try:
            # Current time
            params.t = self.get_simulation_time()
            params.c = self.simulation_step // 100  # Cycle approximation
            
            if detector_id and self._detector_exists(detector_id):
                # Use detector data if available
                params = self._get_detector_parameters(detector_id, params)
            else:
                # Use lane data directly
                params = self._get_lane_parameters(lane_id, params)
            
            # Check for emergency vehicles
            params.ambulance_detected = self._detect_emergency_vehicles_in_lane(lane_id)
            
        except Exception as e:
            print(f"Warning: Error getting parameters for lane {lane_id}: {e}")
        
        return params
    
    def _detector_exists(self, detector_id: str) -> bool:
        """Check if detector exists in simulation"""
        try:
            if hasattr(traci, 'lanearea'):
                return detector_id in traci.lanearea.getIDList()
            else:
                return detector_id in traci.inductionloop.getIDList()
        except:
            return False
    
    def _get_detector_parameters(self, detector_id: str, params: TrafficParameters) -> TrafficParameters:
        """Get traffic parameters from detector"""
        try:
            if hasattr(traci, 'lanearea'):
                # Lane area detector
                params.l = traci.lanearea.getJamLengthVehicle(detector_id)
                params.raw_count = traci.lanearea.getLastStepVehicleNumber(detector_id)
                params.v = traci.lanearea.getLastStepMeanSpeed(detector_id)
                params.m = traci.lanearea.getLastStepOccupancy(detector_id) / 100.0
                
                # Calculate waiting time from vehicles on detector
                vehicle_ids = traci.lanearea.getLastStepVehicleIDs(detector_id)
                if vehicle_ids:
                    wait_times = []
                    for vid in vehicle_ids:
                        try:
                            wait_times.append(traci.vehicle.getAccumulatedWaitingTime(vid))
                        except:
                            pass
                    params.td = sum(wait_times) / len(wait_times) if wait_times else 0.0
                
                # Flow rate (vehicles/hour)
                params.g = params.raw_count * 3600.0
                
            else:
                # Induction loop detector (simpler data)
                params.raw_count = traci.inductionloop.getLastStepVehicleNumber(detector_id)
                params.v = traci.inductionloop.getLastStepMeanSpeed(detector_id)
                params.l = params.raw_count  # Approximation
                params.g = params.raw_count * 3600.0
                
        except Exception as e:
            print(f"Warning: Error reading detector {detector_id}: {e}")
        
        return params
    
    def _get_lane_parameters(self, lane_id: str, params: TrafficParameters) -> TrafficParameters:
        """Get traffic parameters directly from lane"""
        try:
            # Get vehicles on lane
            vehicle_ids = traci.lane.getLastStepVehicleIDs(lane_id)
            params.raw_count = len(vehicle_ids)
            params.l = params.raw_count  # Simple queue length approximation
            
            if vehicle_ids:
                # Calculate average speed
                speeds = []
                wait_times = []
                
                for vid in vehicle_ids:
                    try:
                        speed = traci.vehicle.getSpeed(vid)
                        wait_time = traci.vehicle.getAccumulatedWaitingTime(vid)
                        speeds.append(speed)
                        wait_times.append(wait_time)
                    except:
                        pass
                
                params.v = sum(speeds) / len(speeds) if speeds else 0.0
                params.td = sum(wait_times) / len(wait_times) if wait_times else 0.0
            
            # Density approximation
            lane_length = traci.lane.getLength(lane_id)
            if lane_length > 0:
                params.m = min(params.raw_count * 5.0 / lane_length, 1.0)  # Assume 5m per vehicle
            
            # Flow rate
            params.g = params.raw_count * 3600.0 if self.simulation_step > 0 else 0.0
            
        except Exception as e:
            print(f"Warning: Error reading lane {lane_id}: {e}")
        
        return params
    
    def _detect_emergency_vehicles_in_lane(self, lane_id: str) -> bool:
        """
        Detect emergency vehicles in specific lane
        """
        try:
            vehicle_ids = traci.lane.getLastStepVehicleIDs(lane_id)
            
            for vid in vehicle_ids:
                try:
                    vtype = traci.vehicle.getVehicleClass(vid)
                    if vtype in config.priority.emergency_vehicle_types:
                        return True
                    
                    # Also check by vehicle type ID
                    type_id = traci.vehicle.getTypeID(vid)
                    if any(emer_type in type_id.lower() for emer_type in config.priority.emergency_vehicle_types):
                        return True
                        
                except:
                    pass
                    
        except Exception as e:
            print(f"Warning: Error detecting emergency vehicles in {lane_id}: {e}")
        
        return False
    
    def get_traffic_light_state(self, tl_id: str) -> Dict[str, Any]:
        """Get current traffic light state information"""
        try:
            return {
                'current_phase': traci.trafficlight.getPhase(tl_id),
                'phase_duration': traci.trafficlight.getPhaseDuration(tl_id),
                'next_switch': traci.trafficlight.getNextSwitch(tl_id),
                'program': traci.trafficlight.getProgram(tl_id),
                'red_yellow_green_state': traci.trafficlight.getRedYellowGreenState(tl_id)
            }
        except Exception as e:
            print(f"Warning: Error getting traffic light state for {tl_id}: {e}")
            return {}
    
    def set_traffic_light_phase(self, tl_id: str, phase: int, duration: Optional[float] = None):
        """Set traffic light phase and optionally duration"""
        try:
            traci.trafficlight.setPhase(tl_id, phase)
            if duration is not None:
                traci.trafficlight.setPhaseDuration(tl_id, duration)
        except Exception as e:
            print(f"Warning: Error setting traffic light phase for {tl_id}: {e}")
    
    def get_intersection_data(self) -> Dict[str, Any]:
        """Get detected intersection structure data"""
        return self.intersection_data
    
    def get_vehicle_count(self) -> int:
        """Get total number of vehicles in simulation"""
        try:
            return traci.vehicle.getIDCount()
        except:
            return 0
    
    def get_all_vehicle_info(self) -> Dict[str, Dict[str, Any]]:
        """Get information about all vehicles in simulation"""
        vehicles = {}
        try:
            for vid in traci.vehicle.getIDList():
                try:
                    vehicles[vid] = {
                        'position': traci.vehicle.getPosition(vid),
                        'speed': traci.vehicle.getSpeed(vid),
                        'lane': traci.vehicle.getLaneID(vid),
                        'type': traci.vehicle.getTypeID(vid),
                        'waiting_time': traci.vehicle.getAccumulatedWaitingTime(vid),
                        'route': traci.vehicle.getRoute(vid)
                    }
                except:
                    pass
        except Exception as e:
            print(f"Warning: Error getting vehicle info: {e}")
        
        return vehicles