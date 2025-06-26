import os
import sys
import traci
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
import time

# Thêm SUMO vào đường dẫn Python
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Vui lòng khai báo biến môi trường 'SUMO_HOME'")

# Cấu hình với hệ thống scoring mới và lane-specific control
MIN_GREEN_TIME = 40  # Thời gian xanh tối thiểu
MAX_GREEN_TIME = 150   # Thời gian xanh tối đa
EVAL_INTERVAL = 5     # Đánh giá điều kiện giao thông mỗi bao lâu
YELLOW_TIME = 10       # Thời gian đèn vàng
COOLDOWN_PERIOD = 2   # Thời gian làm mát sau khi thay đổi pha

# Trọng số cho Lane Score equation: Lane_Score = w1·Q + w2·W + w3·D + w4·F
LANE_SCORE_WEIGHTS = {
    'w1': 0.40,  # Trọng số cho hàng đợi (Q)
    'w2': 0.20,  # Trọng số cho thời gian chờ (W)
    'w3': 0.30,  # Trọng số cho mật độ (D)
    'w4': 0.10   # Trọng số cho lưu lượng (F)
}

# Trọng số cho Status equation: status = w1·Σlane_score + w2·max(lane_score)
STATUS_WEIGHTS = {
    'w1': 0.6,   # Trọng số cho tổng điểm làn
    'w2': 0.4    # Trọng số cho điểm làn cao nhất
}

# Ngưỡng quyết định
DECISION_THRESHOLD = 0.4  # Ngưỡng để chuyển pha
EMERGENCY_THRESHOLD = 0.8 # Ngưỡng khẩn cấp

def start_sumo():
    """Khởi động SUMO"""
    sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
    # Make sure to update this path to your actual .sumocfg file
    sumo_config_path = r"C:\Users\Admin\Downloads\sumo test\New folder\dataset.sumocfg"
    if not os.path.exists(sumo_config_path):
        print(f"Lỗi: Không tìm thấy file cấu hình SUMO tại '{sumo_config_path}'")
        print("Vui lòng cập nhật biến 'sumo_config_path' trong hàm start_sumo().")
        sys.exit(1)
    sumoCmd = [sumoBinary, '-c', sumo_config_path, '--step-length', '0.1']
    traci.start(sumoCmd)

def setup_traffic_light_program():
    """Thiết lập traffic light program với lane-specific control"""
    try:
        # Chuyển về program adaptive_1 để có control tốt hơn
        traci.trafficlight.setProgram('E3', 'adaptive_1')
        print("Đã chuyển sang program adaptive_1")
        
        # Lấy thông tin về controlled links để hiểu mapping
        controlled_links = traci.trafficlight.getControlledLinks('E3')
        print(f"Số lượng controlled links: {len(controlled_links)}")
        
        # Phân tích mapping của signal positions với lanes
        print("=== TRAFFIC LIGHT SIGNAL MAPPING ===")
        for i, link_list in enumerate(controlled_links):
            if link_list:  # Nếu có links
                for link in link_list:
                    from_lane, to_lane, via_lane = link
                    print(f"Signal {i:2d}: {from_lane} -> {to_lane} (via {via_lane})")
        
        return True
        
    except Exception as e:
        print(f"Lỗi khi setup traffic light: {e}")
        return False

def create_lane_specific_states():
    """
    Tạo các state strings cho lane-specific control
    Áp dụng logic North-South cho tất cả các hướng
    """
    
    # Movement type mapping based on signal analysis
    # Applying NS logic to all directions for simplified control
    movement_groups = {
        'All_straight_left': [0, 2, 6, 7, 8, 11, 12, 13, 16, 17, 18],     # All straight+left movements
        'All_straight_right': [1, 4, 5, 9, 10, 14, 15, 19, 20, 21],       # All straight+right movements
        
        # Traditional groupings for reference
        'NS_all': [6, 7, 8, 9, 10, 16, 17, 18, 19, 20, 21],              # All North-South movements
        'EW_all': [0, 1, 2, 3, 4, 5, 11, 12, 13, 14, 15]                 # All East-West movements
    }
    
    def create_state(active_groups):
        """Create a 22-character state string with specified groups active"""
        state = ['r'] * 22  # Start with all red
        for group in active_groups:
            if group in movement_groups:
                for signal_pos in movement_groups[group]:
                    if signal_pos < 22:  # Safety check
                        state[signal_pos] = 'G'
        return ''.join(state)
    
    def create_yellow_state(active_groups):
        """Create yellow transition state"""
        state = ['r'] * 22  # Start with all red
        for group in active_groups:
            if group in movement_groups:
                for signal_pos in movement_groups[group]:
                    if signal_pos < 22:  # Safety check
                        state[signal_pos] = 'y'
        return ''.join(state)
    
    states = {
        # Primary Lane-Specific States
        'all_straight_left_only': create_state(['All_straight_left']),
        'all_straight_left_yellow': create_yellow_state(['All_straight_left']),
        
        'all_straight_right_only': create_state(['All_straight_right']),
        'all_straight_right_yellow': create_yellow_state(['All_straight_right']),
        
        # Traditional directional states for fallback
        'NS_traditional': create_state(['NS_all']),
        'NS_traditional_yellow': create_yellow_state(['NS_all']),
        
        'EW_traditional': create_state(['EW_all']),
        'EW_traditional_yellow': create_yellow_state(['EW_all']),
        
        # All RED (transition state)
        'all_red': 'r' * 22,
    }
    
    # Validate all states are 22 characters
    for name, state in states.items():
        if len(state) != 22:
            print(f"❌ State '{name}' has wrong length: {len(state)} (should be 22)")
        else:
            print(f"✅ State '{name}': {state}")
    
    return states

def get_lane_raw_metrics(detector_id):
    """Lấy các metrics thô từ detector hoặc lane"""
    try:
        # Thử lấy từ lane area detector trước
        available_detectors = traci.lanearea.getIDList()
        if detector_id in available_detectors:
            Q = traci.lanearea.getJamLengthVehicle(detector_id)
            D = traci.lanearea.getLastStepOccupancy(detector_id) / 100.0
            vehicle_count = traci.lanearea.getLastStepVehicleNumber(detector_id)
            F = vehicle_count * 360
            
            vehicles = traci.lanearea.getLastStepVehicleIDs(detector_id)
            W = 0
            if vehicles:
                wait_times = []
                for veh in vehicles:
                    try:
                        wait_times.append(traci.vehicle.getWaitingTime(veh))
                    except:
                        continue
                W = max(wait_times) if wait_times else 0
        else:
            # Fallback: lấy từ lane ID trực tiếp
            lane_id = detector_id
            vehicles = traci.lane.getLastStepVehicleIDs(lane_id)
            vehicle_count = len(vehicles)
            
            wait_times = []
            for veh in vehicles:
                try:
                    wait_times.append(traci.vehicle.getWaitingTime(veh))
                except:
                    continue
            
            Q = vehicle_count  # Approximation
            W = max(wait_times) if wait_times else 0
            D = traci.lane.getLastStepOccupancy(lane_id) / 100.0
            F = vehicle_count * 360
        
        return {
            'Q': Q,      # Queue length (vehicles)
            'W': W,      # Waiting time (seconds)
            'D': D,      # Density (0-1)
            'F': F,      # Flow rate (vehicles/hour)
            'raw_count': vehicle_count if 'vehicle_count' in locals() else 0
        }
    
    except Exception as e:
        print(f"⚠️  Không thể lấy metrics từ {detector_id}: {e}")
        return {
            'Q': 0, 'W': 0, 'D': 0, 'F': 0, 'raw_count': 0
        }

def calculate_lane_score(metrics):
    """
    Tính Lane Score theo công thức:
    Lane_Score = w1·Q + w2·W + w3·D + w4·F
    """
    if not metrics:
        return 0.0
    
    # Chuẩn hóa các tham số về thang điểm 0-1
    Q_normalized = min((metrics['Q'] / 8.0)**2, 1.0)         # 8 xe = 100%
    W_normalized = min((metrics['W'] / 60.0)**1.5, 1.0)      # 60 giây = 100%
    D_normalized = metrics['D']                         # Đã là 0-1
    F_normalized = min(metrics['F'] / 1800.0, 1.0)     # 1800 xe/giờ = 100%
    
    # Áp dụng công thức Lane Score
    lane_score = (
        LANE_SCORE_WEIGHTS['w1'] * Q_normalized +
        LANE_SCORE_WEIGHTS['w2'] * W_normalized +
        LANE_SCORE_WEIGHTS['w3'] * D_normalized +
        LANE_SCORE_WEIGHTS['w4'] * F_normalized
    )
    
    return lane_score

def calculate_direction_status(lane_scores):
    """
    Tính Status cho một hướng theo công thức:
    status = w1·Σlane_score + w2·max(lane_score)
    """
    if not lane_scores:
        return 0.0
    
    # Tính tổng điểm các làn
    sum_lane_scores = sum(lane_scores)
    
    # Tìm điểm cao nhất
    max_lane_score = max(lane_scores)
    
    # Áp dụng công thức Status
    status = (
        STATUS_WEIGHTS['w1'] * sum_lane_scores +
        STATUS_WEIGHTS['w2'] * max_lane_score
    )
    
    return status

def analyze_all_directions_lane_specific():
    """Phân tích điều kiện cho tất cả các hướng với lane-specific logic"""
    
    # Cấu hình detector cho tất cả các hướng
    # Áp dụng logic NS cho tất cả
    lane_detectors = {
        'North': {
            'straight_left': 'E1-3_0',   # Lane 0: straight + left
            'straight_right': 'E1-3_1'   # Lane 1: straight + right
        },
        'South': {
            'straight_left': 'E5-3_0',   # Lane 0: straight + left  
            'straight_right': 'E5-3_1'   # Lane 1: straight + right
        },
        'East': {
            'straight_left': 'E3-4_0',   # Lane 0: straight + left
            'straight_right': 'E3-4_1'   # Lane 1: straight + right
        },
        'West': {
            'straight_left': 'E3-2_0',   # Lane 0: straight + left
            'straight_right': 'E3-2_1'   # Lane 1: straight + right
        }
    }
    
    lane_analysis = {}
    
    for direction, lanes in lane_detectors.items():
        direction_data = {}
        
        for lane_type, detector_id in lanes.items():
            # Lấy metrics từ lane
            metrics = get_lane_raw_metrics(detector_id)
            
            # Tính lane score
            score = calculate_lane_score(metrics)
            
            direction_data[lane_type] = {
                'metrics': metrics,
                'score': score,
                'detector_id': detector_id
            }
        
        lane_analysis[direction] = direction_data
    
    return lane_analysis

def aggregate_global_lane_priorities(lane_analysis):
    """Tổng hợp priorities cho các nhóm làn toàn cục (áp dụng logic NS cho tất cả)"""
    
    group_priorities = {
        'all_straight_left': 0.0,     # Tất cả làn thẳng + rẽ trái
        'all_straight_right': 0.0,    # Tất cả làn thẳng + rẽ phải
        'traditional_NS': 0.0,        # Traditional North-South
        'traditional_EW': 0.0         # Traditional East-West
    }
    
    # Tổng hợp tất cả straight+left lanes
    all_sl_scores = []
    all_sr_scores = []
    ns_scores = []
    ew_scores = []
    
    for direction, data in lane_analysis.items():
        if 'straight_left' in data:
            all_sl_scores.append(data['straight_left']['score'])
        if 'straight_right' in data:
            all_sr_scores.append(data['straight_right']['score'])
        
        # Traditional grouping
        direction_scores = []
        if 'straight_left' in data:
            direction_scores.append(data['straight_left']['score'])
        if 'straight_right' in data:
            direction_scores.append(data['straight_right']['score'])
        
        if direction in ['North', 'South']:
            ns_scores.extend(direction_scores)
        elif direction in ['East', 'West']:
            ew_scores.extend(direction_scores)
    
    # Tính priorities
    group_priorities['all_straight_left'] = calculate_direction_status(all_sl_scores)
    group_priorities['all_straight_right'] = calculate_direction_status(all_sr_scores)
    group_priorities['traditional_NS'] = calculate_direction_status(ns_scores)
    group_priorities['traditional_EW'] = calculate_direction_status(ew_scores)
    
    return group_priorities

def intelligent_global_decision(group_priorities, current_state_type, phase_duration):
    """Quyết định thông minh cho global lane control"""
    
    # Map state types to group names
    state_to_group = {
        'all_straight_left_only': 'all_straight_left',
        'all_straight_right_only': 'all_straight_right',
        'NS_traditional': 'traditional_NS',
        'EW_traditional': 'traditional_EW'
    }
    
    current_group = state_to_group.get(current_state_type, 'all_straight_left')
    
    # Tìm nhóm có priority cao nhất
    best_group = max(group_priorities, key=group_priorities.get)
    best_priority = group_priorities[best_group]
    current_priority = group_priorities.get(current_group, 0.0)
    
    # Logic quyết định
    should_change = False
    next_state_type = current_state_type
    reason = "Maintain current state"
    
    # Map group back to state type
    group_to_state = {v: k for k, v in state_to_group.items()}
    
    # --- NEW: Logic to terminate green phase early if traffic has cleared ---
    # This check runs before the MIN_GREEN_TIME check.
    # It allows ending a green phase after a shorter duration (e.g., 15s) if its priority has become very low.
    if phase_duration > 15 and current_priority < (DECISION_THRESHOLD * 0.3):
        if best_priority > (current_priority + 0.1): # And another group needs service
            should_change = True
            next_state_type = group_to_state.get(best_group, current_state_type)
            reason = f"Early termination: current priority ({current_priority:.3f}) is very low."
            # Return immediately since we've made a decision
            return should_change, next_state_type, reason, best_group, best_priority

    # Kiểm tra điều kiện chuyển đổi (Original logic)
    if phase_duration >= MIN_GREEN_TIME:
        
        # Điều kiện 1: Priority khẩn cấp
        if best_priority >= EMERGENCY_THRESHOLD:
            should_change = True
            next_state_type = group_to_state.get(best_group, current_state_type)
            reason = f"Emergency priority: {best_group} ({best_priority:.3f})"
        
        # Điều kiện 2: Nhóm khác có priority cao hơn đáng kể
        elif (best_group != current_group and 
              best_priority > current_priority + 0.15 and
              best_priority >= DECISION_THRESHOLD):
            should_change = True
            next_state_type = group_to_state.get(best_group, current_state_type)
            reason = f"Higher priority: {best_group} ({best_priority:.3f}) vs current ({current_priority:.3f})"
        
        # Điều kiện 3: Quá thời gian tối đa
        elif phase_duration >= MAX_GREEN_TIME:
            should_change = True
            next_state_type = group_to_state.get(best_group, current_state_type)
            reason = "Maximum green time exceeded"
        
        # Điều kiện 4: Priority hiện tại quá thấp
        elif (current_priority < DECISION_THRESHOLD * 0.5 and 
              best_priority > DECISION_THRESHOLD):
            should_change = True
            next_state_type = group_to_state.get(best_group, current_state_type)
            reason = f"Current priority too low: {current_priority:.3f}"
    
    return should_change, next_state_type, reason, best_group, best_priority
def safe_set_traffic_state(tl_id, state_type, states):
    """Đặt trạng thái traffic light một cách an toàn"""
    try:
        if state_type in states:
            state_string = states[state_type]
            if len(state_string) != 22:
                print(f"❌ State string length mismatch: {len(state_string)} != 22")
                return False
            
            traci.trafficlight.setRedYellowGreenState(tl_id, state_string)
            print(f"✅ Set state: {state_type}")
            return True
        else:
            print(f"❌ State type '{state_type}' không tồn tại")
            return False
    except Exception as e:
        print(f"❌ Lỗi khi đặt state: {e}")
        return False

def run_global_lane_simulation():
    """Chạy mô phỏng với global lane-specific control"""
    
    print("=== KHỞI TẠO GLOBAL LANE-SPECIFIC TRAFFIC CONTROL ===")
    print("Applied NS logic to ALL directions for unified lane control")
    
    # Setup traffic light program
    if not setup_traffic_light_program():
        print("Không thể setup traffic light program")
        return
    
    # Tạo lane-specific states
    states = create_lane_specific_states()
    print(f"Đã tạo {len(states)} state configurations")
    
    # Khởi tạo biến
    current_state_type = 'all_straight_left_only'
    phase_start_time = 0
    step = 0
    
    # Dữ liệu tracking (MODIFIED to include direction_statuses for the new graph)
    tracking_data = {
        'time': [],
        'direction_statuses': {
            'North': [], 'South': [], 'East': [], 'West': []
        },
        'group_priorities': {
            'all_straight_left': [],
            'all_straight_right': [], 
            'traditional_NS': [],
            'traditional_EW': []
        },
        'current_state': [],
        'state_changes': [],
        'lane_details': []
    }
    
    # Đặt state đầu tiên
    safe_set_traffic_state('E3', current_state_type, states)
    print(f"🟢 Bắt đầu với state: {current_state_type}")
    print("🎯 This allows ALL straight+left lanes (N,S,E,W) to be green while ALL straight+right lanes are red")
    
    print("\n=== BẮT ĐẦU GLOBAL LANE-SPECIFIC SIMULATION ===")
    
    try:
        while step < 30000:  
            traci.simulationStep()
            current_time = step / 10.0
            phase_duration = current_time - phase_start_time
            
            # Phân tích định kỳ
            if step % (EVAL_INTERVAL * 10) == 0:
                
                # Phân tích all directions với lane-specific logic
                lane_analysis = analyze_all_directions_lane_specific()
                
                # Tổng hợp global priorities
                group_priorities = aggregate_global_lane_priorities(lane_analysis)
                
                # --- MODIFICATION START ---
                # Lưu tracking data
                tracking_data['time'].append(current_time)
                
                # Calculate and store status for each individual direction for the new graph
                for direction, lanes_data in lane_analysis.items():
                    lane_scores = [info['score'] for info in lanes_data.values()]
                    status = calculate_direction_status(lane_scores)
                    if direction in tracking_data['direction_statuses']:
                        tracking_data['direction_statuses'][direction].append(status)

                # Store other data for analysis and decision making
                tracking_data['current_state'].append(current_state_type)
                for group, priority in group_priorities.items():
                    tracking_data['group_priorities'][group].append(priority)
                
                tracking_data['lane_details'].append({
                    'time': current_time,
                    'analysis': lane_analysis,
                    'priorities': group_priorities
                })
                # --- MODIFICATION END ---

                # In thông tin chi tiết định kỳ
                if step % 600 == 0:  # Mỗi 60 giây
                    print(f"\n--- 📊 GLOBAL LANE-SPECIFIC STATUS: {current_time:.1f}s ---")
                    print(f"🚦 Current State: {current_state_type}")
                    print(f"⏱️  State Duration: {phase_duration:.1f}s")
                    print("🎯 Global Lane Group Priorities:")
                    
                    for group, priority in group_priorities.items():
                        indicator = "🟢 ACTIVE" if group in current_state_type else ""
                        urgency = ""
                        if priority >= EMERGENCY_THRESHOLD:
                            urgency = "🚨 EMERGENCY"
                        elif priority >= DECISION_THRESHOLD:
                            urgency = "⚠️  HIGH"
                        print(f"  {group:20s}: {priority:.3f} {urgency} {indicator}")
                    
                    # Chi tiết theo hướng và làn
                    print("📍 Direction & Lane Details:")
                    for direction, data in lane_analysis.items():
                        print(f"  {direction}:")
                        for lane_type, info in data.items():
                            metrics = info['metrics']
                            score = info['score']
                            print(f"    {lane_type:15s}: Score={score:.3f} | Q={metrics['Q']:.1f} W={metrics['W']:.1f}s D={metrics['D']:.2f} F={metrics['F']:.0f}")
                
                # Quyết định global lane-specific
                should_change, next_state_type, reason, best_group, best_priority = intelligent_global_decision(
                    group_priorities, current_state_type, phase_duration
                )
                
                # Thực hiện chuyển đổi state nếu cần
                if should_change and next_state_type != current_state_type:
                    # 1. Chuyển qua YELLOW cho pha hiện tại
                    # NOTE: The yellow state should be based on the *current* state, not the next one.
                    current_yellow_state_type = current_state_type.replace('_only', '_yellow').replace('_traditional', '_traditional_yellow')
                    if current_yellow_state_type in states:
                        safe_set_traffic_state('E3', current_yellow_state_type, states)
                        print(f"🟡 YELLOW TRANSITION: from {current_state_type} to yellow")
                        
                        # Đợi yellow time
                        for _ in range(YELLOW_TIME * 10):
                            traci.simulationStep()
                            # FIX: Do not increment step here
                    
                    # 2. (NEW) Chuyển sang ALL-RED để dọn dẹp giao lộ
                    all_red_clearance_time = 2  # Duration in seconds for the all-red phase
                    safe_set_traffic_state('E3', 'all_red', states)
                    print(f"🔴 ALL-RED CLEARANCE for {all_red_clearance_time} seconds.")
                    for _ in range(all_red_clearance_time * 10):
                        traci.simulationStep()
                        # FIX: Do not increment step here

                    # 3. Chuyển sang GREEN state mới
                    if safe_set_traffic_state('E3', next_state_type, states):
                        print(f"🔄 GLOBAL LANE CHANGE: {current_state_type} -> {next_state_type}")
                        print(f"   📋 Reason: {reason}")
                        print(f"   🎯 Target: {best_group} (priority: {best_priority:.3f})")
                        
                        # Mô tả hiệu ứng của state change
                        if 'straight_left' in next_state_type:
                            print("   🚦 Effect: ALL directions' straight+left lanes GREEN, straight+right lanes RED")
                        elif 'straight_right' in next_state_type:
                            print("   🚦 Effect: ALL directions' straight+right lanes GREEN, straight+left lanes RED")
                        elif 'NS_traditional' in next_state_type:
                            print("   🚦 Effect: North-South ALL lanes GREEN, East-West ALL lanes RED")
                        elif 'EW_traditional' in next_state_type:
                            print("   🚦 Effect: East-West ALL lanes GREEN, North-South ALL lanes RED")
                        
                        # Lưu thông tin state change
                        tracking_data['state_changes'].append({
                            'time': current_time,
                            'from': current_state_type,
                            'to': next_state_type,
                            'reason': reason,
                            'target_group': best_group,
                            'target_priority': best_priority
                        })
                        
                        current_state_type = next_state_type
                        phase_start_time = current_time
            
            step += 1
    
    except Exception as e:
        print(f"❌ Lỗi trong simulation: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\n=== 🏁 KẾT THÚC GLOBAL LANE-SPECIFIC SIMULATION ===")
        print(f"📊 Tổng số lần chuyển state: {len(tracking_data['state_changes'])}")
        
        # Summary statistics
        print("\n=== 📊 SUMMARY STATISTICS ===")
        if tracking_data['time']:
            print(f"📈 Simulation Duration: {tracking_data['time'][-1]:.1f} seconds")
            print(f"🔄 Average Time Between Changes: {tracking_data['time'][-1] / max(len(tracking_data['state_changes']), 1):.1f} seconds")
            
        # Call the new plotting function if data was collected
        if tracking_data['time']:
            plot_congestion_graph(tracking_data)
        
        try:
            traci.close()
        except:
            pass

def plot_congestion_graph(tracking_data):
    """
    Vẽ biểu đồ Trạng Thái Giao Thông: Chỉ Số Tắc Nghẽn Theo Thời Gian.
    This function is designed to replicate the style of the user-provided image.
    """
    try:
        plt.style.use('default')
        fig, ax = plt.subplots(figsize=(16, 8))

        time = tracking_data['time']
        statuses = tracking_data['direction_statuses']
        
        # Plot data for each direction
        ax.plot(time, statuses['North'], color='blue', linewidth=2, label='Hướng Bắc')
        ax.plot(time, statuses['South'], color='green', linewidth=2, label='Hướng Nam')
        ax.plot(time, statuses['East'], color='red', linewidth=2, label='Hướng Đông')
        ax.plot(time, statuses['West'], color='orange', linewidth=2, label='Hướng Tây')

        # Congestion threshold line
        CONGESTION_THRESHOLD = 1.0
        ax.axhline(y=CONGESTION_THRESHOLD, color='red', linestyle='--', linewidth=2.5, label=f'Ngưỡng Tắc Nghẽn ({CONGESTION_THRESHOLD:.1f})')

        # Find the max value from all statuses to set a proper upper y-limit
        max_y = 0
        for direction in statuses:
            if statuses[direction]:
                max_y = max(max_y, max(statuses[direction]))
        # Set y-axis limit with some padding, ensuring it's at least 3.0 like the image
        ax.set_ylim(bottom=0, top=max(max_y * 1.1, 2.0))

        # Shaded regions, using the y-limit we just set
        ax.fill_between(time, CONGESTION_THRESHOLD, ax.get_ylim()[1], color='red', alpha=0.2, label='Vùng TẮC NGHẼN (Xấu)')
        ax.fill_between(time, 0, CONGESTION_THRESHOLD, color='green', alpha=0.2, label='Vùng THÔNG THOÁNG (Tốt)')
        
        # Titles and labels, styled to match the image
        ax.set_title('Trạng Thái Giao Thông: Chỉ Số Tắc Nghẽn Theo Thời Gian\n(Thấp = Tốt, Cao = Tắc nghẽn)', fontsize=15)
        ax.set_xlabel('Thời Gian (giây)', fontsize=12)
        ax.set_ylabel('Chỉ Số Tắc Nghẽn', fontsize=12)

        # X-axis limit
        if time:
            ax.set_xlim(left=0, right=max(time))

        # Grid style
        ax.grid(True, which='both', linestyle='-', linewidth=0.5, alpha=0.4)

        # Explanation text box in the top-left
        explanation_text = (
            '  GIẢI THÍCH:\n'
            ' • Đường dưới ngưỡng = Giao thông thông thoáng\n'
            ' • Đường trên ngưỡng = Giao thông tắc nghẽn'
        )
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.9, edgecolor='black', lw=0.5)
        ax.text(0.015, 0.97, explanation_text, transform=ax.transAxes, fontsize=11,
                verticalalignment='top', bbox=props)

        # Create and order the legend to match the image
        handles, labels = ax.get_legend_handles_labels()
        # Desired order: 4 direction lines, threshold line, good zone, bad zone
        try:
            order = [labels.index('Hướng Bắc'), labels.index('Hướng Nam'), labels.index('Hướng Đông'), labels.index('Hướng Tây'), 
                     labels.index(f'Ngưỡng Tắc Nghẽn ({CONGESTION_THRESHOLD:.1f})'), 
                     labels.index('Vùng THÔNG THOÁNG (Tốt)'), labels.index('Vùng TẮC NGHẼN (Xấu)')]
            ax.legend([handles[idx] for idx in order], [labels[idx] for idx in order], loc='lower right', fontsize=12)
        except (ValueError, IndexError): # Fallback if a label isn't found
            ax.legend(loc='lower right', fontsize=12)

        plt.tight_layout(rect=[0, 0.01, 1, 0.97]) # Adjust layout to prevent elements from overlapping
        
        # Save and show the plot
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'traffic_congestion_status_{timestamp}.png'
        plt.savefig(filename, dpi=150)
        print(f"📊 Đã lưu biểu đồ trạng thái tắc nghẽn: '{filename}'")
        
        plt.show()

    except Exception as e:
        print(f"❌ Lỗi khi vẽ biểu đồ tắc nghẽn: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    start_sumo()
    run_global_lane_simulation()