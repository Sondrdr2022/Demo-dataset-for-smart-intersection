import os
import sys
import traci
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict, deque
from datetime import datetime

# ====== SUMO PATH SETUP ======
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please set the 'SUMO_HOME' environment variable.")

# ====== CONFIGURATION ======
MIN_GREEN_TIME = 12
DYNAMIC_MAX_GREEN_TIME = 180
MAX_GREEN_TIME = 90
YELLOW_TIME = 4
ALL_RED_TIME = 2
EVAL_INTERVAL = 2
COOLDOWN_PERIOD = 3
DECISION_THRESHOLD = 0.35
EMERGENCY_THRESHOLD = 0.95
CONGESTION_THRESHOLD = 0.55
MAX_WAIT_TIME = 120

LANE_SCORE_WEIGHTS = {'w1': 0.55, 'w2': 0.3, 'w3': 0.1, 'w4': 0.05}
STATUS_WEIGHTS = {'w1': 0.8, 'w2': 0.2}

def start_sumo(sumo_config_path):
    sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
    if not os.path.exists(sumo_config_path):
        print(f"Error: SUMO config file not found at '{sumo_config_path}'")
        sys.exit(1)
    sumoCmd = [sumoBinary, '-c', sumo_config_path, '--step-length', '0.1']
    traci.start(sumoCmd)

def auto_detect_intersection_structure():
    tl_ids = traci.trafficlight.getIDList()
    lane_ids = traci.lane.getIDList()
    intersection_data = {}
    for tl_id in tl_ids:
        controlled_links = traci.trafficlight.getControlledLinks(tl_id)
        controlled_lanes = traci.trafficlight.getControlledLanes(tl_id)
        approaches = {}
        for lane_id in set(controlled_lanes):
            if ':' not in lane_id:
                edge_id = lane_id.split('_')[0] if '_' in lane_id else lane_id.split('#')[0]
                if edge_id not in approaches:
                    approaches[edge_id] = {'lanes': [], 'lane_count': 0}
                approaches[edge_id]['lanes'].append(lane_id)
                approaches[edge_id]['lane_count'] += 1
        intersection_data[tl_id] = {
            'approaches': approaches,
            'controlled_links': controlled_links,
            'controlled_lanes': controlled_lanes,
            'total_approaches': len(approaches),
            'total_lanes': len(set(controlled_lanes))
        }
    return intersection_data

def get_lane_metrics(lane_id):
    """Get metrics for a lane using TraCI APIs."""
    try:
        vehicles = traci.lane.getLastStepVehicleIDs(lane_id)
        vehicle_count = len(vehicles)
        Q = vehicle_count
        W = max([traci.vehicle.getWaitingTime(veh) for veh in vehicles]) if vehicles else 0
        D = traci.lane.getLastStepOccupancy(lane_id) / 100.0
        F = vehicle_count * 360
        # Downstream congestion detection
        downstream = traci.lane.getLinks(lane_id)
        downstream_free = all(traci.lane.getLastStepOccupancy(link[0]) < 80 for link in downstream) if downstream else True
        congestion_factor = 1.5 if not downstream_free else 1.0
        return {
            'Q': Q * congestion_factor,
            'W': W,
            'D': D,
            'F': F,
            'raw_count': vehicle_count,
            'congestion': congestion_factor > 1.0
        }
    except Exception as e:
        print(f"⚠️  Cannot get metrics from {lane_id}: {e}")
        return {'Q': 0, 'W': 0, 'D': 0, 'F': 0, 'raw_count': 0, 'congestion': False}

def calculate_lane_score(metrics):
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
    if not lane_scores:
        return 0.0
    sum_lane_scores = sum(lane_scores)
    max_lane_score = max(lane_scores)
    status = (
        STATUS_WEIGHTS['w1'] * max_lane_score +
        STATUS_WEIGHTS['w2'] * sum_lane_scores
    )
    return status

def aggregate_approach_priorities(intersection_data, tl_id):
    approaches = intersection_data[tl_id]['approaches']
    approach_statuses = {}
    for approach_name, approach_data in approaches.items():
        lane_scores = []
        congestion_detected = False
        for lane_id in approach_data['lanes']:
            metrics = get_lane_metrics(lane_id)
            score = calculate_lane_score(metrics)
            lane_scores.append(score)
            congestion_detected = congestion_detected or metrics['congestion']
        status = calculate_direction_status(lane_scores)
        approach_statuses[approach_name] = {
            'status': status,
            'lane_scores': lane_scores,
            'congestion': congestion_detected
        }
    return approach_statuses

def intelligent_phase_decision(approach_statuses, current_approach, phase_duration, last_green_times, current_time):
    emergency_approaches = [
        app for app, data in approach_statuses.items() 
        if data['status'] >= EMERGENCY_THRESHOLD
    ]
    congested_approaches = [
        app for app, data in approach_statuses.items()
        if data['congestion']
    ]
    starved_approaches = [
        app for app in approach_statuses
        if current_time - last_green_times[app] > MAX_WAIT_TIME
    ]

    if emergency_approaches:
        best_approach = max(emergency_approaches, key=lambda x: approach_statuses[x]['status'])
        if best_approach != current_approach:
            return True, best_approach, "EMERGENCY", best_approach, approach_statuses[best_approach]['status']
    if congested_approaches:
        best_approach = max(congested_approaches, key=lambda x: approach_statuses[x]['status'])
        if approach_statuses[best_approach]['status'] > 0.8 or approach_statuses[best_approach]['congestion']:
            if phase_duration < DYNAMIC_MAX_GREEN_TIME:
                return False, current_approach, "EXTEND GREEN FOR CONGESTION", best_approach, approach_statuses[best_approach]['status']
        if best_approach != current_approach and phase_duration >= MIN_GREEN_TIME:
            return True, best_approach, "CONGESTION", best_approach, approach_statuses[best_approach]['status']
    if starved_approaches:
        best_approach = max(starved_approaches, key=lambda x: current_time - last_green_times[x])
        if phase_duration >= MIN_GREEN_TIME:
            return True, best_approach, "STARVATION", best_approach, approach_statuses[best_approach]['status']

    best_approach = max(approach_statuses, key=lambda k: approach_statuses[k]['status'])
    best_priority = approach_statuses[best_approach]['status']
    current_priority = approach_statuses[current_approach]['status']
    should_change = False
    reason = "Maintain current"
    next_approach = current_approach
    if phase_duration >= MIN_GREEN_TIME:
        if best_priority >= CONGESTION_THRESHOLD and best_priority > current_priority + 0.15:
            should_change = True
            next_approach = best_approach
            reason = f"High priority difference ({best_priority-current_priority:.2f})"
        elif phase_duration >= MAX_GREEN_TIME:
            should_change = True
            next_approach = best_approach
            reason = "Max green time exceeded"
        elif current_priority < DECISION_THRESHOLD and best_priority > DECISION_THRESHOLD:
            should_change = True
            next_approach = best_approach
            reason = "Current below threshold"
    return should_change, next_approach, reason, best_approach, best_priority

def create_approach_states(intersection_data, tl_id):
    n_lights = len(traci.trafficlight.getRedYellowGreenState(tl_id))
    approaches = intersection_data[tl_id]['approaches']
    approach_states = {}
    approach_idxmap = {}
    all_idx = list(range(n_lights))
    for i, (approach_name, approach_data) in enumerate(approaches.items()):
        state = ['r'] * n_lights
        lane_indices = []
        for link_i, links in enumerate(traci.trafficlight.getControlledLinks(tl_id)):
            for link in links:
                from_lane, to_lane, via_lane = link
                if from_lane in approach_data['lanes']:
                    lane_indices.append(link_i)
        for idx in lane_indices:
            if idx < n_lights:
                state[idx] = 'G'
        if not lane_indices and i < n_lights:
            state[i] = 'G'
        green_state = ''.join(state)
        yellow_state = ''.join(['y' if c == 'G' else 'r' for c in state])
        red_state = 'r' * n_lights
        approach_states[approach_name] = {'G': green_state, 'y': yellow_state, 'r': red_state}
        approach_idxmap[approach_name] = lane_indices
    return approach_states

def plot_congestion_graph(tracking_data, tl_id):
    try:
        plt.style.use('default')
        fig, ax = plt.subplots(figsize=(16, 8))
        time = tracking_data['time']
        statuses = tracking_data['approach_statuses']
        for approach_name, status_list in statuses.items():
            ax.plot(time, status_list, linewidth=2, label=f'Approach {approach_name}')
        threshold = 1.0
        ax.axhline(y=threshold, color='red', linestyle='--', linewidth=2.5, label=f'Congestion Threshold ({threshold:.1f})')
        max_y = 0
        for status_list in statuses.values():
            if status_list:
                max_y = max(max_y, max(status_list))
        ax.set_ylim(bottom=0, top=max(max_y * 1.1, 2.0))
        ax.fill_between(time, threshold, ax.get_ylim()[1], color='red', alpha=0.2, label='Congested')
        ax.fill_between(time, 0, threshold, color='green', alpha=0.2, label='Clear')
        ax.set_title(f'Traffic Congestion Index over Time (Light {tl_id})', fontsize=15)
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Congestion Index', fontsize=12)
        if time:
            ax.set_xlim(left=0, right=max(time))
        ax.grid(True, which='both', linestyle='-', linewidth=0.5, alpha=0.4)
        ax.legend(loc='lower right', fontsize=12)
        plt.tight_layout(rect=[0, 0.01, 1, 0.97])
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'traffic_congestion_status_{tl_id}_{timestamp}.png'
        plt.savefig(filename, dpi=150)
        print(f"📊 Saved congestion plot: '{filename}'")
        plt.show()
    except Exception as e:
        print(f"❌ Error plotting congestion: {e}")
        import traceback
        traceback.print_exc()

def safe_set_traffic_state(tl_id, approach_name, approach_states, color):
    try:
        state_string = approach_states[approach_name][color]
        traci.trafficlight.setRedYellowGreenState(tl_id, state_string)
        return True
    except Exception as e:
        print(f"❌ Error setting traffic state: {e}")
        return False

def run_adaptive_simulation(sumo_config_path):
    start_sumo(sumo_config_path)
    intersection_data = auto_detect_intersection_structure()
    if not intersection_data:
        print("❌ No traffic lights detected!")
        return
    tl_ids = list(intersection_data.keys())
    print(f"Detected {len(tl_ids)} traffic lights: {tl_ids}")
    try:
        for tl_id in tl_ids:
            print(f"\n=== ADAPTIVE CONTROL FOR TRAFFIC LIGHT: {tl_id} ===")
            approaches = intersection_data[tl_id]['approaches']
            approach_states = create_approach_states(intersection_data, tl_id)
            approach_names = list(approaches.keys())
            if not approach_names:
                print(f"Light {tl_id} has no valid approaches.")
                continue
            current_approach = approach_names[0]
            safe_set_traffic_state(tl_id, current_approach, approach_states, 'G')
            print(f"🟢 Starting with approach: {current_approach}")
            phase_start_time = 0
            step = 0
            last_green_times = {app: 0 for app in approach_names}
            congestion_history = {app: deque(maxlen=10) for app in approach_names}
            cooldown_end = 0

            tracking_data = {
                'time': [],
                'approach_statuses': {aname: [] for aname in approach_names},
                'current_approach': [],
                'state_changes': []
            }
            while step < 36000:  # 1 hour at step-length 0.1s
                traci.simulationStep()
                current_time = step / 10.0
                phase_duration = current_time - phase_start_time
                in_cooldown = current_time < cooldown_end
                if not in_cooldown and step % (EVAL_INTERVAL * 10) == 0:
                    approach_statuses = aggregate_approach_priorities(intersection_data, tl_id)
                    for aname in approach_names:
                        tracking_data['approach_statuses'][aname].append(
                            approach_statuses.get(aname, {'status':0})['status']
                        )
                        congestion_history[aname].append(
                            1 if approach_statuses[aname]['congestion'] else 0
                        )
                    tracking_data['time'].append(current_time)
                    tracking_data['current_approach'].append(current_approach)
                    should_change, next_approach, reason, best_approach, best_priority = intelligent_phase_decision(
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
                        if safe_set_traffic_state(tl_id, next_approach, approach_states, 'G'):
                            print(f"🔄 CHANGE: {current_approach} -> {next_approach} at {current_time:.1f}s | {reason}")
                            tracking_data['state_changes'].append({
                                'time': current_time,
                                'from': current_approach,
                                'to': next_approach,
                                'reason': reason,
                                'target_approach': best_approach,
                                'target_priority': best_priority
                            })
                            last_green_times[next_approach] = current_time
                            cooldown_end = current_time + COOLDOWN_PERIOD
                            current_approach = next_approach
                            phase_start_time = current_time
                step += 1
            print(f"=== END SIMULATION FOR LIGHT: {tl_id} ===")
            if tracking_data['time']:
                plot_congestion_graph(tracking_data, tl_id)
    except Exception as e:
        print(f"❌ Error in simulation: {e}")
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