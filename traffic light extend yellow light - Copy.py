import os
import sys
import traci
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

# Add SUMO to Python path
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare SUMO_HOME environment variable")

# IMPROVED CONFIGURATION TO MINIMIZE EMERGENCY BRAKING
MIN_GREEN_TIME = 45          # Increased from 40 to 45 seconds for more stability
MAX_GREEN_TIME = 110         # Increased from 100 to 110 seconds
EVAL_INTERVAL = 25           # Increased from 20 to 25 seconds for less frequent evaluation
YELLOW_TIME = 12             # Increased from 10 to 12 seconds for better reaction time
CLEARANCE_TIME = 6           # Increased from 5 to 6 seconds for complete junction clearing
COOLDOWN_PERIOD = 25         # Increased from 20 to 25 seconds
THRESHOLD = 0.7              # Keep threshold unchanged
JUNCTION_CLEARING_DISTANCE = 70  # Increased from 60 to 70 meters for larger safety zone
SAFETY_SPEED_BUFFER = 1.5    # Reduced from 2 to 1.5 for better detection
YELLOW_TIME_BUFFER = 4       # Increased from 3 to 4 seconds for additional safety
MAX_DECELERATION = 2.5       # Reduced from 3.0 to 2.5 m/s² for gentler braking
MIN_PHASE_GAP = 60           # Increased from 50 to 60 (minimum 6 seconds between phase changes)
AGGRESSIVE_DETECTION_DISTANCE = 80  # New: Distance for aggressive vehicle detection
SPEED_THRESHOLD_HIGH = 15    # New: High speed threshold for extended safety checks

last_phase_change_step = -MIN_PHASE_GAP

def start_sumo():
    """Start SUMO"""
    sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
    sumoCmd = [sumoBinary, '-c', r"C:\Users\Admin\Downloads\sumo test\New folder\dataset.sumocfg", '--step-length', '0.1']
    traci.start(sumoCmd)

def get_lane_metrics(detector_id):
    """Get metrics for specific lane"""
    metrics = {}
    
    # Queue length (number of vehicles)
    metrics['queue_length'] = traci.lanearea.getJamLengthVehicle(detector_id)
    
    # Waiting time (maximum waiting time of vehicles on detector)
    vehicles = traci.lanearea.getLastStepVehicleIDs(detector_id)
    wait_times = [traci.vehicle.getWaitingTime(veh) for veh in vehicles] if vehicles else [0]
    metrics['waiting_time'] = max(wait_times) if wait_times else 0
    
    # Lane density (vehicles per lane length)
    occupancy = traci.lanearea.getLastStepOccupancy(detector_id) / 100.0
    metrics['density'] = occupancy
    
    # Average speed
    metrics['avg_speed'] = traci.lanearea.getLastStepMeanSpeed(detector_id)
    
    # Flow rate (vehicles/hour)
    metrics['flow_rate'] = traci.lanearea.getLastStepVehicleNumber(detector_id) * 3600
    
    return metrics

def calculate_status(metrics_list):
    """Calculate status based on multiple metrics from different lanes"""
    weights = {
        'queue_length': 0.3,
        'waiting_time': 0.4,
        'density': 0.2,
        'avg_speed': 0.0,
        'flow_rate': 0.1
    }
    
    status_components = []
    
    for metrics in metrics_list:
        lane_score = (
            weights['queue_length'] * min(metrics['queue_length'] / 10, 1) +
            weights['waiting_time'] * min(metrics['waiting_time'] / 60, 1) +
            weights['density'] * metrics['density'] +
            weights['flow_rate'] * (metrics['flow_rate'] / 1800)
        )
        status_components.append(lane_score)
    
    total_status = sum(status_components)
    max_component = max(status_components) if status_components else 0
    
    adjusted_status = 0.6 * total_status + 0.4 * max_component
    
    return adjusted_status, status_components

def can_stop_safely(speed, distance):
    """Enhanced safety check for whether vehicle can stop safely"""
    if speed < 0.5:  # Very low speed vehicles
        return True
    
    # Enhanced reaction time for different speeds
    if speed > 15:  # High speed vehicles need more reaction time
        t_reaction = 1.5
    elif speed > 10:
        t_reaction = 1.2
    else:
        t_reaction = 1.0
    
    # Calculate minimum stopping distance with enhanced safety buffer
    min_stopping_distance = (speed ** 2) / (2 * MAX_DECELERATION) + speed * t_reaction
    safety_buffer = 1.5 if speed > 12 else 1.3  # Dynamic safety buffer
    min_stopping_distance *= safety_buffer
    
    # Time-based check
    stopping_time = speed / MAX_DECELERATION + t_reaction
    time_to_intersection = distance / max(speed, 0.1)
    
    # Additional check for high-speed vehicles
    if speed > SPEED_THRESHOLD_HIGH and distance < AGGRESSIVE_DETECTION_DISTANCE:
        return False
    
    return distance > min_stopping_distance and time_to_intersection > stopping_time * 1.2

def is_safe_to_change_phase(direction_detectors, junction_id='E3'):
    """Enhanced safety check to ensure vehicles can safely pass through intersection"""
    global last_phase_change_step
    current_step = traci.simulation.getTime() * 10  # Convert to simulation steps
    
    # Enforce minimum gap between phase changes
    if current_step - last_phase_change_step < MIN_PHASE_GAP:
        return False
    
    # Check vehicles approaching the intersection with enhanced detection
    for detector_id in direction_detectors:
        vehicles = traci.lanearea.getLastStepVehicleIDs(detector_id)
        for vehicle in vehicles:
            try:
                speed = traci.vehicle.getSpeed(vehicle)
                
                # Enhanced detection for slow-moving vehicles
                if speed > SAFETY_SPEED_BUFFER:
                    distance = traci.vehicle.getLanePosition(vehicle)
                    lane_length = traci.lane.getLength(traci.vehicle.getLaneID(vehicle))
                    remaining_distance = lane_length - distance
                    
                    # Enhanced danger zone detection
                    danger_zone_distance = 45 if speed > 12 else 35
                    critical_speed = 8 if speed > 15 else 6
                    
                    if remaining_distance < danger_zone_distance and speed > critical_speed:
                        print(f"[{vehicle}] Too close to intersection with high speed -> UNSAFE")
                        return False
                    
                    # Enhanced safety stopping check
                    if not can_stop_safely(speed, remaining_distance):
                        print(f"[{vehicle}] Cannot stop safely in {remaining_distance:.2f}m at {speed:.2f} m/s")
                        return False
                    
                    # Enhanced time-based calculation with dynamic buffer
                    speed_factor = 1.4 if speed > 12 else 1.3
                    time_to_intersection = (remaining_distance / max(speed, 1.0)) * speed_factor
                    
                    # Enhanced yellow time check with dynamic buffer
                    yellow_buffer = YELLOW_TIME_BUFFER + (1 if speed > 12 else 0)
                    if 0 < time_to_intersection < YELLOW_TIME + yellow_buffer:
                        return False
                    
                    # Enhanced decision zone check for fast vehicles
                    decision_distance = 70 if speed > 15 else 60
                    if speed > 12 and remaining_distance < decision_distance:
                        return False
                        
            except:
                continue
    
    # Check vehicles currently in junction
    vehicles_in_junction = get_vehicles_in_junction(junction_id)
    if vehicles_in_junction:
        return False
    
    # Update last phase change time
    last_phase_change_step = current_step
    return True

def get_vehicles_in_junction(junction_id):
    """Get all vehicles currently in or near the intersection with expanded detection zone"""
    junction_vehicles = []
    all_vehicles = traci.vehicle.getIDList()
    
    for veh_id in all_vehicles:
        try:
            road_id = traci.vehicle.getRoadID(veh_id)
            
            # Check if vehicle is on internal lane (inside intersection)
            if road_id.startswith(':'):
                junction_vehicles.append(veh_id)
                continue
            
            # Enhanced proximity check
            lane_id = traci.vehicle.getLaneID(veh_id)
            lane_length = traci.lane.getLength(lane_id)
            position = traci.vehicle.getLanePosition(veh_id)
            speed = traci.vehicle.getSpeed(veh_id)
            
            # Enhanced calculation with speed-dependent detection zone
            detection_distance = JUNCTION_CLEARING_DISTANCE
            if speed > 12:
                detection_distance += 15  # Extended zone for fast vehicles
            
            estimated_time_to_junction = (lane_length - position) / max(speed, 1.0)
            
            # Enhanced conditions for junction vicinity detection
            near_junction = (position > lane_length - detection_distance or 
                           (speed > 3 and estimated_time_to_junction < 5))
            
            if near_junction:
                next_links = traci.lane.getLinks(lane_id)
                for link in next_links:
                    if junction_id in link[0]:
                        junction_vehicles.append(veh_id)
                        break
        except:
            continue
            
    return junction_vehicles

def count_emergency_braking_events():
    """Count emergency braking events in current step"""
    count = 0
    for veh_id in traci.vehicle.getIDList():
        try:
            # Check for emergency braking (deceleration > 4.0 m/s^2)
            if traci.vehicle.getAcceleration(veh_id) < -4.0:  # Reduced threshold for earlier detection
                count += 1
        except:
            continue
    return count

def wait_for_junction_clearing(junction_id='E3', max_wait=8):  # Increased from 7 to 8 seconds
    """Wait for vehicles to completely clear the junction"""
    wait_steps = 0
    cleared = False
    
    while wait_steps < max_wait * 10:
        traci.simulationStep()
        vehicles_in_junction = get_vehicles_in_junction(junction_id)
        
        if not vehicles_in_junction:
            # Additional wait to ensure complete clearing
            additional_wait = 10  # 1 second additional wait
            for _ in range(additional_wait):
                traci.simulationStep()
                wait_steps += 1
            cleared = True
            break
        wait_steps += 1
    
    if not cleared:
        print("Warning: Junction could not be completely cleared after maximum wait time!")
    
    return wait_steps

def plot_traffic_status(status_data, threshold):
    """Plot traffic status graph compared to threshold - UNCHANGED"""
    plt.figure(figsize=(15, 10))
    
    # Plot traffic status vs threshold
    for direction, color in zip(['North', 'South', 'East', 'West'], ['blue', 'green', 'red', 'orange']):
        label = {'North': 'Hướng Bắc', 'South': 'Hướng Nam', 
                 'East': 'Hướng Đông', 'West': 'Hướng Tây'}[direction]
        plt.plot(status_data['time'], status_data[direction], label=label, color=color, linewidth=2)
    
    # Add threshold line
    plt.axhline(y=threshold, color='r', linestyle='--', linewidth=2, 
                label=f'Ngưỡng ({threshold})')
    
    # Add colored zones for GOOD/BAD status
    max_status = 0
    for d in ['North', 'South', 'East', 'West']:
        if status_data[d]:
            max_status = max(max_status, max(status_data[d]))
    
    plt.fill_between(status_data['time'], threshold, max_status * 1.1, 
                    color='green', alpha=0.2, label='Vùng Trạng Thái TỐT')
    plt.fill_between(status_data['time'], 0, threshold,
                    color='red', alpha=0.2, label='Vùng Trạng Thái XẤU')
    
    plt.xlabel('Thời Gian (giây)', fontsize=12)
    plt.ylabel('Giá Trị Trạng Thái', fontsize=12)
    plt.title('Trạng Thái Giao Thông Theo Hướng So Với Ngưỡng (Cải Tiến Chống Phanh Khẩn Cấp)', fontsize=16)
    plt.legend(fontsize=12)
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('trang_thai_giao_thong_cai_tien.png', dpi=150)
    print("Đã lưu biểu đồ dưới dạng 'trang_thai_giao_thong_cai_tien.png'")
    plt.show()

def run_simulation():
    """Run simulation with improvements to reduce emergency braking"""
    # Define detector groups by direction
    detector_groups = {
        'North': ['E1-3-1', 'E1-3-2'],
        'East': ['E3-4-1', 'E3-4-2'],
        'South': ['E5-3-1', 'E5-3-2'],
        'West': ['E3-2-1', 'E3-2-2']
    }
    
    # Traffic light phases
    direction_to_phase = {
        'North': 0, 
        'South': 0,
        'East': 2, 
        'West': 2
    }
    
    # Tracking metrics over time
    metrics_history = defaultdict(list)
    current_phase = 0
    phase_duration = 0
    cooldown_timer = 0
    
    # Data for visualization
    status_data = {'time': [], 'North': [], 'South': [], 'East': [], 'West': []}
    
    # Main simulation loop
    step = 0
    in_transition = False
    transition_stage = 0
    next_green_phase = None
    emergency_braking_events = 0
    data_collection_interval = 50
    
    print("Starting simulation with enhanced emergency braking prevention...")
    
    while step < 5000:
        traci.simulationStep()
        
        # Check emergency braking events
        if step % 10 == 0:
            new_events = count_emergency_braking_events()
            if new_events > 0:
                print(f"Time {step/10:.1f}s: Detected {new_events} new emergency braking events")
            emergency_braking_events += new_events
        
        # Get all metrics from detectors
        all_metrics = {}
        for direction, detectors in detector_groups.items():
            direction_metrics = [get_lane_metrics(detector) for detector in detectors]
            status, components = calculate_status(direction_metrics)
            all_metrics[direction] = {
                'status': status,
                'components': components,
                'is_good': status >= THRESHOLD,
                'metrics': direction_metrics
            }
            
            metrics_history[direction].append(all_metrics[direction])
        
        # Collect data for visualization periodically
        if step % data_collection_interval == 0:
            time_sec = step / 10.0
            status_data['time'].append(time_sec)
            for direction, data in all_metrics.items():
                status_data[direction].append(data['status'])
        
        # Increment phase duration counter
        phase_duration += 1
        
        # Handle cooldown timer after phase change
        if cooldown_timer > 0:
            cooldown_timer -= 1
            step += 1
            continue
        
        # Handle transition phases (yellow and all-red)
        if in_transition:
            if transition_stage == 1 and phase_duration >= YELLOW_TIME:
                # After yellow, move to all-red stage
                transition_stage = 2
                phase_duration = 0
                print(f"Time {step/10:.1f}s: Moving to all-red stage")
                
            elif transition_stage == 2 and phase_duration >= CLEARANCE_TIME:
                # After all-red stage, wait for junction to clear
                print(f"Time {step/10:.1f}s: Waiting for junction to clear completely")
                wait_steps = wait_for_junction_clearing()
                
                # Set next green phase
                in_transition = False
                transition_stage = 0
                current_phase = next_green_phase
                traci.trafficlight.setPhase('E3', current_phase)
                phase_duration = 0
                cooldown_timer = COOLDOWN_PERIOD
                print(f"Time {(step+wait_steps)/10:.1f}s: Changed to new green phase (phase {current_phase})")
                
                step += wait_steps
            step += 1
            continue
        
        # Evaluate phase regularly (every EVAL_INTERVAL seconds)
        if phase_duration % EVAL_INTERVAL == 0:
            current_directions = [dir for dir, phase in direction_to_phase.items() if phase == current_phase]
            opposing_directions = [dir for dir, phase in direction_to_phase.items() if phase != current_phase]
            
            current_detectors = []
            for direction in current_directions:
                current_detectors.extend(detector_groups[direction])
            
            # Check if current phase is still good or minimum time reached
            current_good = any(all_metrics[dir]['is_good'] for dir in current_directions)
            opposing_need = any(all_metrics[dir]['status'] > THRESHOLD * 1.3 for dir in opposing_directions)  # Increased threshold
            
            # Enhanced logic for phase switching with safety checks
            should_change = (not current_good or opposing_need) and phase_duration >= MIN_GREEN_TIME
            
            if should_change:
                # Enhanced safety check before phase switching
                if is_safe_to_change_phase(current_detectors):
                    # Start yellow phase
                    in_transition = True
                    transition_stage = 1
                    if current_phase == 0:
                        traci.trafficlight.setPhase('E3', 1)  # N-S yellow
                        next_green_phase = 2  # Target will be E-W green
                        print(f"Time {step/10:.1f}s: Starting N-S yellow (poor status)")
                    else:
                        traci.trafficlight.setPhase('E3', 3)  # E-W yellow
                        next_green_phase = 0  # Target will be N-S green
                        print(f"Time {step/10:.1f}s: Starting E-W yellow (poor status)")
                    phase_duration = 0
                else:
                    print(f"Time {step/10:.1f}s: Delaying phase change - not safe")
            
            # Force change if maximum green time reached with enhanced safety
            elif phase_duration >= MAX_GREEN_TIME:
                if is_safe_to_change_phase(current_detectors):
                    in_transition = True
                    transition_stage = 1
                    if current_phase == 0:
                        traci.trafficlight.setPhase('E3', 1)
                        next_green_phase = 2
                        print(f"Time {step/10:.1f}s: Starting N-S yellow (max time reached)")
                    else:
                        traci.trafficlight.setPhase('E3', 3)
                        next_green_phase = 0
                        print(f"Time {step/10:.1f}s: Starting E-W yellow (max time reached)")
                    phase_duration = 0
                else:
                    # If not safe even at max time, extend slightly and check again
                    phase_duration = MAX_GREEN_TIME - 20  # Increased extension
                    print(f"Time {step/10:.1f}s: Extending green phase further - unsafe to change")
        
        # Print status every 30 seconds
        if step % 300 == 0:
            print(f"Time: {step / 10} seconds")
            for direction, data in all_metrics.items():
                direction_name = {'North': 'Bắc', 'South': 'Nam', 'East': 'Đông', 'West': 'Tây'}[direction]
                status = "TỐT" if data['is_good'] else "XẤU"
                print(f"Direction {direction_name}: Status = {data['status']:.2f} ({status})")
            print(f"Emergency braking events so far: {emergency_braking_events}")
            print("-" * 40)
        
        step += 1
    
    # Simulation completed
    print("Simulation completed")
    
    # Calculate and display summary statistics
    print("\nSummary Statistics:")
    avg_waiting_times = {}
    avg_queue_lengths = {}
    
    for direction, history in metrics_history.items():
        avg_waiting_times[direction] = np.mean([np.mean([m['waiting_time'] for m in h['metrics']]) 
                                               for h in history])
        avg_queue_lengths[direction] = np.mean([np.mean([m['queue_length'] for m in h['metrics']]) 
                                              for h in history])
    
    total_avg_wait = np.mean(list(avg_waiting_times.values()))
    total_avg_queue = np.mean(list(avg_queue_lengths.values()))
    
    print(f"Overall average waiting time: {total_avg_wait:.2f} seconds")
    print(f"Overall average queue length: {total_avg_queue:.2f} vehicles")
    print(f"Total emergency braking events: {emergency_braking_events}")
    
    direction_names = {'North': 'Bắc', 'South': 'Nam', 'East': 'Đông', 'West': 'Tây'}
    for direction, avg_wait in avg_waiting_times.items():
        dir_name = direction_names[direction]
        print(f"Direction {dir_name}: Avg Wait = {avg_wait:.2f}s, Avg Queue = {avg_queue_lengths[direction]:.2f}")
    
    # Create graph - UNCHANGED functionality
    plot_traffic_status(status_data, THRESHOLD)

    traci.close()

if __name__ == "__main__":
    start_sumo()
    run_simulation()