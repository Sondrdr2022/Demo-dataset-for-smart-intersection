import os
import sys
import traci
import numpy as np
import matplotlib.pyplot as plt
from collections import deque, defaultdict
from datetime import datetime

# ====== SUMO PATH SETUP ======
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Vui lòng khai báo biến môi trường 'SUMO_HOME'")

# CONFIGURATION
MIN_GREEN_TIME = 12
MAX_GREEN_TIME = 120
YELLOW_TIME = 4
ALL_RED_TIME = 2
EVAL_INTERVAL = 2
COOLDOWN_PERIOD = 2
DECISION_THRESHOLD = 0.4
EMERGENCY_THRESHOLD = 0.8

LANE_SCORE_WEIGHTS = {'w1': 0.40, 'w2': 0.20, 'w3': 0.30, 'w4': 0.10}
STATUS_WEIGHTS = {'w1': 0.6, 'w2': 0.4}

def start_sumo(sumo_config_path):
    sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
    if not os.path.exists(sumo_config_path):
        print(f"Lỗi: Không tìm thấy file cấu hình SUMO tại '{sumo_config_path}'")
        sys.exit(1)
    sumoCmd = [sumoBinary, '-c', sumo_config_path, '--step-length', '0.1']
    traci.start(sumoCmd)

def auto_detect_intersection_structure():
    """Detects all traffic lights, their controlled lanes, and possible detectors for each lane."""
    tl_ids = traci.trafficlight.getIDList()
    detector_ids = traci.lanearea.getIDList() if hasattr(traci, "lanearea") else []
    intersection_data = {}
    for tl_id in tl_ids:
        controlled_links = traci.trafficlight.getControlledLinks(tl_id)
        controlled_lanes = traci.trafficlight.getControlledLanes(tl_id)
        approaches = defaultdict(lambda: {'lanes': [], 'detectors': [], 'lane_count': 0})
        lane_to_detector = {}
        # Map detectors to lanes
        for detector_id in detector_ids:
            try:
                detector_lane = traci.lanearea.getLaneID(detector_id)
                if detector_lane in controlled_lanes:
                    lane_to_detector[detector_lane] = detector_id
            except:
                continue
        # Group controlled lanes into approaches by edge (entry)
        for lane_id in set(controlled_lanes):
            if ':' in lane_id:
                continue
            edge_id = lane_id.split('_')[0] if '_' in lane_id else lane_id.split('#')[0]
            approaches[edge_id]['lanes'].append(lane_id)
            approaches[edge_id]['lane_count'] += 1
            if lane_id in lane_to_detector:
                approaches[edge_id]['detectors'].append(lane_to_detector[lane_id])
        intersection_data[tl_id] = {
            'approaches': dict(approaches),
            'controlled_links': controlled_links,
            'controlled_lanes': controlled_lanes,
            'lane_to_detector': lane_to_detector,
            'total_approaches': len(approaches),
            'total_lanes': len(set(controlled_lanes))
        }
    return intersection_data

def get_lane_metrics(lane_id, detector_id=None):
    """Returns metrics from detector or lane."""
    try:
        if detector_id and detector_id in traci.lanearea.getIDList():
            vehicle_ids = traci.lanearea.getLastStepVehicleIDs(detector_id)
            vehicle_count = len(vehicle_ids)
            Q = traci.lanearea.getJamLengthVehicle(detector_id)
            W = max([traci.vehicle.getWaitingTime(veh) for veh in vehicle_ids]) if vehicle_ids else 0
            D = traci.lanearea.getLastStepOccupancy(detector_id) / 100.0
            F = vehicle_count * 360
        else:
            vehicles = traci.lane.getLastStepVehicleIDs(lane_id)
            vehicle_count = len(vehicles)
            Q = vehicle_count
            W = max([traci.vehicle.getWaitingTime(veh) for veh in vehicles]) if vehicles else 0
            D = traci.lane.getLastStepOccupancy(lane_id) / 100.0
            F = vehicle_count * 360
        return {'Q': Q, 'W': W, 'D': D, 'F': F, 'raw_count': vehicle_count}
    except Exception as e:
        print(f"⚠️  Không thể lấy metrics từ {lane_id}/{detector_id}: {e}")
        return {'Q': 0, 'W': 0, 'D': 0, 'F': 0, 'raw_count': 0}

def calculate_lane_score(metrics):
    """Lane_Score = w1·Q + w2·W + w3·D + w4·F (normalized)"""
    if not metrics:
        return 0.0
    Q_normalized = min((metrics['Q'] / 8.0)**2, 1.0)
    W_normalized = min((metrics['W'] / 60.0)**1.5, 1.0)
    D_normalized = metrics['D']
    F_normalized = min(metrics['F'] / 1800.0, 1.0)
    lane_score = (
        LANE_SCORE_WEIGHTS['w1'] * Q_normalized +
        LANE_SCORE_WEIGHTS['w2'] * W_normalized +
        LANE_SCORE_WEIGHTS['w3'] * D_normalized +
        LANE_SCORE_WEIGHTS['w4'] * F_normalized
    )
    return lane_score

def calculate_direction_status(lane_scores):
    """status = w1·Σlane_score + w2·max(lane_score)"""
    if not lane_scores:
        return 0.0
    sum_lane_scores = sum(lane_scores)
    max_lane_score = max(lane_scores)
    status = (
        STATUS_WEIGHTS['w1'] * sum_lane_scores +
        STATUS_WEIGHTS['w2'] * max_lane_score
    )
    return status

def aggregate_approach_statuses(intersection_data, tl_id):
    """Returns status for each approach (by entry edge)."""
    approaches = intersection_data[tl_id]['approaches']
    approach_statuses = {}
    for approach_name, approach_data in approaches.items():
        lane_scores = []
        for i, lane_id in enumerate(approach_data['lanes']):
            detector_id = approach_data['detectors'][i] if i < len(approach_data['detectors']) else None
            metrics = get_lane_metrics(lane_id, detector_id)
            score = calculate_lane_score(metrics)
            lane_scores.append(score)
        status = calculate_direction_status(lane_scores)
        approach_statuses[approach_name] = {'status': status, 'lane_scores': lane_scores}
    return approach_statuses

def create_approach_states(intersection_data, tl_id):
    """Dynamically create states for each approach: only that approach green, rest red."""
    n_lights = len(traci.trafficlight.getRedYellowGreenState(tl_id))
    approaches = intersection_data[tl_id]['approaches']
    controlled_links = traci.trafficlight.getControlledLinks(tl_id)
    approach_states = {}
    for approach_name, approach_data in approaches.items():
        state = ['r'] * n_lights
        lane_indices = []
        target_lanes = set(approach_data['lanes'])
        for link_i, links in enumerate(controlled_links):
            for link in links:
                from_lane, to_lane, via_lane = link
                if from_lane in target_lanes:
                    lane_indices.append(link_i)
        for idx in lane_indices:
            if idx < n_lights:
                state[idx] = 'G'
        green_state = ''.join(state)
        yellow_state = ''.join(['y' if c == 'G' else 'r' for c in state])
        red_state = 'r' * n_lights
        approach_states[approach_name] = {'G': green_state, 'y': yellow_state, 'r': red_state}
    return approach_states

def safe_set_traffic_state(tl_id, approach, approach_states, color):
    try:
        state_string = approach_states[approach][color]
        traci.trafficlight.setRedYellowGreenState(tl_id, state_string)
        return True
    except Exception as e:
        print(f"❌ Error setting state: {e}")
        return False

def intelligent_phase_decision(approach_statuses, current_approach, phase_duration, last_green_times, current_time):
    """Decides whether to rotate green phase to another approach (edge) based on priorities."""
    best_approach = max(approach_statuses, key=lambda k: approach_statuses[k]['status'])
    best_priority = approach_statuses[best_approach]['status']
    current_priority = approach_statuses[current_approach]['status']
    should_change = False
    reason = "Maintain current"
    next_approach = current_approach
    # Starvation logic: switch if any approach waited too long
    starved = [app for app in approach_statuses if (current_time - last_green_times.get(app, 0)) > 120]
    if starved and phase_duration >= MIN_GREEN_TIME:
        next_approach = max(starved, key=lambda app: current_time - last_green_times.get(app, 0))
        should_change = True
        reason = "STARVATION"
        return should_change, next_approach, reason
    # Emergency
    if best_priority >= EMERGENCY_THRESHOLD and best_approach != current_approach and phase_duration >= MIN_GREEN_TIME:
        should_change = True
        next_approach = best_approach
        reason = "EMERGENCY"
        return should_change, next_approach, reason
    # Switch due to priority difference
    if phase_duration >= MIN_GREEN_TIME:
        if best_priority > current_priority + 0.2 and best_priority > DECISION_THRESHOLD:
            should_change = True
            next_approach = best_approach
            reason = "Higher priority"
        elif phase_duration >= MAX_GREEN_TIME:
            should_change = True
            next_approach = best_approach
            reason = "Max green time exceeded"
        elif current_priority < DECISION_THRESHOLD and best_priority > DECISION_THRESHOLD:
            should_change = True
            next_approach = best_approach
            reason = "Current below threshold"
    return should_change, next_approach, reason

def run_adaptive_simulation(sumo_config_path):
    start_sumo(sumo_config_path)
    intersection_data = auto_detect_intersection_structure()
    if not intersection_data:
        print("❌ Không phát hiện được đèn giao thông nào!")
        return
    tl_ids = list(intersection_data.keys())
    print(f"Phát hiện {len(tl_ids)} đèn giao thông: {tl_ids}")
    try:
        for tl_id in tl_ids:
            print(f"\n=== ADAPTIVE CONTROL FOR TRAFFIC LIGHT: {tl_id} ===")
            approaches = intersection_data[tl_id]['approaches']
            approach_states = create_approach_states(intersection_data, tl_id)
            approach_names = list(approaches.keys())
            if not approach_names:
                print(f"Đèn {tl_id} không có hướng nào hợp lệ.")
                continue
            current_approach = approach_names[0]
            safe_set_traffic_state(tl_id, current_approach, approach_states, 'G')
            print(f"🟢 Bắt đầu với approach: {current_approach}")
            phase_start_time = 0
            step = 0
            last_green_times = {app: 0 for app in approach_names}
            cooldown_end = 0
            while step < 36000:  # 1 hour at step-length 0.1s
                traci.simulationStep()
                current_time = step / 10.0
                phase_duration = current_time - phase_start_time
                in_cooldown = current_time < cooldown_end
                if not in_cooldown and step % (EVAL_INTERVAL * 10) == 0:
                    approach_statuses = aggregate_approach_statuses(intersection_data, tl_id)
                    should_change, next_approach, reason = intelligent_phase_decision(
                        approach_statuses, current_approach, phase_duration, last_green_times, current_time)
                    if should_change and next_approach != current_approach:
                        # Yellow
                        safe_set_traffic_state(tl_id, current_approach, approach_states, 'y')
                        for _ in range(YELLOW_TIME * 10):
                            traci.simulationStep()
                        # All-red
                        safe_set_traffic_state(tl_id, current_approach, approach_states, 'r')
                        for _ in range(ALL_RED_TIME * 10):
                            traci.simulationStep()
                        # Next green
                        safe_set_traffic_state(tl_id, next_approach, approach_states, 'G')
                        print(f"🔄 CHANGE: {current_approach} -> {next_approach} at {current_time:.1f}s | {reason}")
                        last_green_times[next_approach] = current_time
                        cooldown_end = current_time + COOLDOWN_PERIOD
                        current_approach = next_approach
                        phase_start_time = current_time
                step += 1
            print(f"=== KẾT THÚC MÔ PHỎNG ĐÈN: {tl_id} ===")
    except Exception as e:
        print(f"❌ Lỗi trong simulation: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            traci.close()
        except:
            pass

if __name__ == "__main__":
    sumo_config_path = r"C:\Users\Admin\Downloads\sumo test\New folder\20 node\20e.sumocfg"
    run_adaptive_simulation(sumo_config_path)