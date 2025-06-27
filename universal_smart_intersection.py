import os
import sys
import traci
import numpy as np

# ====== SUMO PATH SETUP ======
# This section ensures that the SUMO tools are available in the Python path.
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Vui lòng khai báo biến môi trường 'SUMO_HOME'")

# ====== CONFIGURATION ======
# Define minimum and maximum values for signal phases and cycles.
MIN_GREEN_TIME = 15         # Minimum green phase duration (seconds)
MAX_GREEN_TIME = 120        # Maximum green phase duration (seconds)
MIN_CYCLE_TIME = 60         # Minimum total signal cycle time (seconds)
MAX_CYCLE_TIME = 200        # Maximum total signal cycle time (seconds)
YELLOW_TIME = 3             # Yellow phase duration (seconds)
ALL_RED_TIME = 2            # All-red phase duration (seconds)
EVAL_INTERVAL = 2           # Traffic evaluation interval (seconds)

# Thresholds for signal adjustment logic
STATUS_THRESHOLD = 0.4
CRITICAL_THRESHOLD = 0.8

# Weights for each traffic parameter in status calculation
WEIGHTS = {'l': 0.25, 'td': 0.20, 'm': 0.20, 'v': 0.15, 'g': 0.20}

def start_sumo():
    """
    Start the SUMO-GUI simulation using the specified configuration file.
    If the file does not exist, exit with an error.
    """
    sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
    sumo_config_path = r"C:\Users\Admin\Downloads\sumo test\New folder\20 node\20e.sumocfg"
    if not os.path.exists(sumo_config_path):
        print(f"Lỗi: Không tìm thấy file cấu hình SUMO tại '{sumo_config_path}'")
        sys.exit(1)
    sumoCmd = [sumoBinary, '-c', sumo_config_path, '--step-length', '0.1']
    traci.start(sumoCmd)

def auto_detect_intersection_structure():
    """
    Automatically detect all traffic lights and their approaches in the SUMO network.
    Returns a dictionary with per-traffic-light information.
    """
    tl_ids = traci.trafficlight.getIDList()
    detector_ids = traci.lanearea.getIDList()
    lane_ids = traci.lane.getIDList()
    intersection_data = {}

    for tl_id in tl_ids:
        controlled_links = traci.trafficlight.getControlledLinks(tl_id)
        controlled_lanes = traci.trafficlight.getControlledLanes(tl_id)
        approaches = {}
        lane_to_detector = {}

        # Map detectors to lanes if detectors exist in the SUMO network
        for detector_id in detector_ids:
            try:
                detector_lane = traci.lanearea.getLaneID(detector_id)
                if detector_lane in controlled_lanes:
                    lane_to_detector[detector_lane] = detector_id
            except:
                continue

        # Group controlled lanes into approaches by edge
        for lane_id in set(controlled_lanes):
            if ':' not in lane_id:
                edge_id = lane_id.split('_')[0] if '_' in lane_id else lane_id.split('#')[0]
                if edge_id not in approaches:
                    approaches[edge_id] = {'lanes': [], 'detectors': [], 'lane_count': 0}
                approaches[edge_id]['lanes'].append(lane_id)
                approaches[edge_id]['lane_count'] += 1
                if lane_id in lane_to_detector:
                    approaches[edge_id]['detectors'].append(lane_to_detector[lane_id])

        intersection_data[tl_id] = {
            'approaches': approaches,
            'controlled_links': controlled_links,
            'controlled_lanes': controlled_lanes,
            'lane_to_detector': lane_to_detector,
            'total_approaches': len(approaches),
            'total_lanes': len(set(controlled_lanes))
        }
    return intersection_data

def get_traffic_parameters(lane_id, detector_id=None, api_data=None):
    """
    Collect traffic parameters for a given lane (and detector, if available).
    If api_data is provided (i.e., in real-time or API mode), use it directly.
    Otherwise, gather parameters from SUMO simulation using detectors (if present) or lane data.
    Returns a dictionary with:
      - l: queue length (vehicles)
      - td: mean waiting time (seconds)
      - m: density (0-1)
      - v: mean speed (m/s)
      - g: flow (vehicles/hour)
      - raw_count: vehicle count
    """
    if api_data is not None:
        # Real-time/API mode: expect api_data as a dict keyed by lane_id
        return api_data.get(lane_id, {'l': 0, 'td': 0, 'm': 0, 'v': 0, 'g': 0, 'raw_count': 0})
    try:
        if detector_id and detector_id in traci.lanearea.getIDList():
            # Use detector data if available
            l = traci.lanearea.getJamLengthVehicle(detector_id)
            m = traci.lanearea.getLastStepOccupancy(detector_id) / 100.0
            vehicle_ids = traci.lanearea.getLastStepVehicleIDs(detector_id)
            vehicle_count = len(vehicle_ids)
            wait_times = []
            velocities = []
            for veh_id in vehicle_ids:
                try:
                    wait_times.append(traci.vehicle.getWaitingTime(veh_id))
                    velocities.append(traci.vehicle.getSpeed(veh_id))
                except:
                    continue
            td = np.mean(wait_times) if wait_times else 0.0
            v = np.mean(velocities) if velocities else 0.0
            g = vehicle_count * 3600 / EVAL_INTERVAL if vehicle_count > 0 else 0.0
        else:
            # Fallback: use full lane data
            vehicle_ids = traci.lane.getLastStepVehicleIDs(lane_id)
            vehicle_count = len(vehicle_ids)
            l = vehicle_count
            m = traci.lane.getLastStepOccupancy(lane_id) / 100.0
            wait_times = []
            velocities = []
            for veh_id in vehicle_ids:
                try:
                    wait_times.append(traci.vehicle.getWaitingTime(veh_id))
                    velocities.append(traci.vehicle.getSpeed(veh_id))
                except:
                    continue
            td = np.mean(wait_times) if wait_times else 0.0
            v = np.mean(velocities) if velocities else 0.0
            g = vehicle_count * 3600 / EVAL_INTERVAL if vehicle_count > 0 else 0.0
        return {'l': l, 'td': td, 'm': m, 'v': v, 'g': g, 'raw_count': vehicle_count}
    except Exception as e:
        print(f"⚠️  Lỗi khi lấy tham số từ {lane_id}: {e}")
        return {'l': 0, 'td': 0, 'm': 0, 'v': 0, 'g': 0, 'raw_count': 0}

def calculate_status_t(traffic_params, max_values=None):
    """
    Calculate Status_t traffic status index for a lane or approach.
    Status_t = w_l * (l/l_max) + w_td * (td/td_max) + w_m * m +
               w_v * (1 - v/v_max) + w_g * (1 - g/g_max)

    Where:
      - l = queue length (vehicles)
      - td = mean waiting time (seconds)
      - m = density (0-1)
      - v = mean speed (m/s)
      - g = flow (vehicles/hour)
    The result is a weighted sum, higher means more congestion.
    """
    # Default max values for normalization, can be tuned as needed
    if max_values is None:
        max_values = {'l_max': 20, 'td_max': 120, 'v_max': 15, 'g_max': 1800}
    l_norm = min(traffic_params['l'] / max_values['l_max'], 1.0)
    td_norm = min(traffic_params['td'] / max_values['td_max'], 1.0)
    m_norm = traffic_params['m']
    v_norm = min(traffic_params['v'] / max_values['v_max'], 1.0)
    g_norm = min(traffic_params['g'] / max_values['g_max'], 1.0)
    status_t = (
        WEIGHTS['l'] * l_norm +
        WEIGHTS['td'] * td_norm +
        WEIGHTS['m'] * m_norm +
        WEIGHTS['v'] * (1.0 - v_norm) +  # Lower speed = higher status_t (worse)
        WEIGHTS['g'] * (1.0 - g_norm)    # Lower flow = higher status_t (worse)
    )
    return status_t

def adaptive_phase_timing(approaches_status, current_green_time, current_cycle_time):
    """
    Adjust phase timing based on approach status.
    Returns a dictionary with new green and cycle time.
    The approach with the highest Status_t is prioritized for longer green.
    """
    max_status = 0
    priority_approach = None
    for approach_name, data in approaches_status.items():
        if data['status'] > max_status:
            max_status = data['status']
            priority_approach = approach_name
    adjustment = {
        'green_time_change': 0,
        'cycle_time_change': 0,
        'priority_approach': priority_approach,
        'reason': ''
    }
    # Apply adjustments based on congestion severity
    if max_status >= CRITICAL_THRESHOLD:
        adjustment['green_time_change'] = +15
        adjustment['cycle_time_change'] = +10
        adjustment['reason'] = f"CRITICAL: {priority_approach} Status_t = {max_status:.2f}"
    elif max_status >= STATUS_THRESHOLD:
        adjustment['green_time_change'] = +8
        adjustment['cycle_time_change'] = +5
        adjustment['reason'] = f"CONGESTED: {priority_approach} Status_t = {max_status:.2f}"
    elif max_status < STATUS_THRESHOLD * 0.5:
        adjustment['green_time_change'] = -5
        adjustment['cycle_time_change'] = -3
        adjustment['reason'] = f"FREE: can optimize time"
    # Clamp to min/max limits
    new_green_time = max(MIN_GREEN_TIME, min(MAX_GREEN_TIME, current_green_time + adjustment['green_time_change']))
    new_cycle_time = max(MIN_CYCLE_TIME, min(MAX_CYCLE_TIME, current_cycle_time + adjustment['cycle_time_change']))
    adjustment['new_green_time'] = new_green_time
    adjustment['new_cycle_time'] = new_cycle_time
    return adjustment

def run_adaptive_simulation():
    """
    Main simulation loop:
    - Detect intersections and initialize state.
    - At each evaluation interval, collect traffic stats, calculate status, and adjust phase times if needed.
    - Print adjustments for each traffic light.
    """
    intersection_data = auto_detect_intersection_structure()
    if not intersection_data:
        print("❌ Không phát hiện được đèn giao thông nào!")
        return
    tl_ids = list(intersection_data.keys())
    # Initialize per-traffic-light state
    tl_states = {}
    for tl_id in tl_ids:
        tl_states[tl_id] = {
            'current_green_time': 30,
            'current_cycle_time': 90,
            'last_adjustment_time': 0
        }
    step = 0
    try:
        while step < 100000:  # 1 hour of simulation
            traci.simulationStep()
            current_time = step / 10.0
            for tl_id in tl_ids:
                state = tl_states[tl_id]
                # Only evaluate and adjust at the defined interval
                if step % (EVAL_INTERVAL * 10) == 0:
                    overall_status, approaches_status = evaluate_intersection_status(
                        intersection_data, tl_id)
                    # Only adjust every 60 seconds
                    if current_time - state['last_adjustment_time'] >= 60:
                        adjustment = adaptive_phase_timing(
                            approaches_status,
                            state['current_green_time'],
                            state['current_cycle_time']
                        )
                        # If any adjustment is needed, apply and print info
                        if adjustment['green_time_change'] != 0 or adjustment['cycle_time_change'] != 0:
                            state['current_green_time'] = adjustment['new_green_time']
                            state['current_cycle_time'] = adjustment['new_cycle_time']
                            state['last_adjustment_time'] = current_time
                            print(f"[{tl_id}] {adjustment['reason']}: Green={state['current_green_time']}s, Cycle={state['current_cycle_time']}s")
            step += 1
    except Exception as e:
        print(f"❌ Lỗi trong simulation: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            traci.close()
        except:
            pass

def evaluate_intersection_status(intersection_data, tl_id):
    """
    For a traffic light, evaluate the status (congestion level) of each approach and overall.
    Returns:
      - overall_status: weighted average status for the intersection
      - approaches_status: dict with per-approach status
    """
    approaches_status = {}
    overall_status = 0.0
    total_weight = 0.0
    intersection = intersection_data[tl_id]
    for approach_name, approach_data in intersection['approaches'].items():
        approach_statuses = []
        for i, lane_id in enumerate(approach_data['lanes']):
            detector_id = None
            if i < len(approach_data['detectors']):
                detector_id = approach_data['detectors'][i]
            traffic_params = get_traffic_parameters(lane_id, detector_id)
            lane_status = calculate_status_t(traffic_params)
            approach_statuses.append(lane_status)
        if approach_statuses:
            approach_status = np.mean(approach_statuses)
            approaches_status[approach_name] = {
                'status': approach_status,
                'lane_count': len(approach_statuses),
                'lane_statuses': approach_statuses
            }
            weight = len(approach_statuses)
            overall_status += approach_status * weight
            total_weight += weight
    if total_weight > 0:
        overall_status = overall_status / total_weight
    return overall_status, approaches_status

if __name__ == "__main__":
    # Start the SUMO simulation and run the adaptive control loop
    start_sumo()
    run_adaptive_simulation()