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
MAX_GREEN_TIME = 110   # Thời gian xanh tối đa
EVAL_INTERVAL = 8     # Đánh giá điều kiện giao thông mỗi bao lâu
YELLOW_TIME = 5       # Thời gian đèn vàng
COOLDOWN_PERIOD = 3   # Thời gian làm mát sau khi thay đổi pha

# Trọng số cho Lane Score equation: Lane_Score = w1·Q + w2·W + w3·D + w4·F
LANE_SCORE_WEIGHTS = {
    'w1': 0.35,  # Trọng số cho hàng đợi (Q)
    'w2': 0.40,  # Trọng số cho thời gian chờ (W)
    'w3': 0.15,  # Trọng số cho mật độ (D)
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
    sumoCmd = [sumoBinary, '-c', r"C:\Users\Admin\Downloads\sumo test\New folder\dataset.sumocfg", '--step-length', '0.1']
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
    Q_normalized = min(metrics['Q'] / 8.0, 1.0)        # 8 xe = 100%
    W_normalized = min(metrics['W'] / 60.0, 1.0)       # 60 giây = 100%
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
    
    # Kiểm tra điều kiện chuyển đổi
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
    
    # Dữ liệu tracking (simplified - no plotting)
    tracking_data = {
        'time': [],
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
        while step < 15000:  # Chạy 25 phút
            traci.simulationStep()
            current_time = step / 10.0
            phase_duration = current_time - phase_start_time
            
            # Phân tích định kỳ
            if step % (EVAL_INTERVAL * 10) == 0:
                
                # Phân tích all directions với lane-specific logic
                lane_analysis = analyze_all_directions_lane_specific()
                
                # Tổng hợp global priorities
                group_priorities = aggregate_global_lane_priorities(lane_analysis)
                
                # Lưu tracking data
                tracking_data['time'].append(current_time)
                tracking_data['current_state'].append(current_state_type)
                for group, priority in group_priorities.items():
                    tracking_data['group_priorities'][group].append(priority)
                
                # Lưu chi tiết lane analysis
                tracking_data['lane_details'].append({
                    'time': current_time,
                    'analysis': lane_analysis,
                    'priorities': group_priorities
                })
                
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
                    # Chuyển qua yellow trước
                    yellow_state = next_state_type.replace('_only', '_yellow').replace('_traditional', '_traditional_yellow')
                    if yellow_state in states:
                        safe_set_traffic_state('E3', yellow_state, states)
                        print(f"🟡 YELLOW TRANSITION: {yellow_state}")
                        
                        # Đợi yellow time
                        for _ in range(YELLOW_TIME * 10):
                            traci.simulationStep()
                            step += 1
                    
                    # Chuyển sang green state mới
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
        
        # Phân tích hiệu quả (text only - no graphs)
        if tracking_data['state_changes']:
            print("\n=== 📈 PHÂN TÍCH GLOBAL LANE CHANGES ===")
            for change in tracking_data['state_changes']:
                print(f"⏰ T={change['time']:.1f}s: {change['from']} -> {change['to']}")
                print(f"   📝 {change['reason']}")
                print(f"   🎯 Target group: {change['target_group']} (priority: {change['target_priority']:.3f})")
        
        # Summary statistics
        print("\n=== 📊 SUMMARY STATISTICS ===")
        if tracking_data['time']:
            print(f"📈 Simulation Duration: {tracking_data['time'][-1]:.1f} seconds")
            print(f"🔄 Average Time Between Changes: {tracking_data['time'][-1] / max(len(tracking_data['state_changes']), 1):.1f} seconds")
            
            # Count state type usage
            state_usage = {}
            for state in tracking_data['current_state']:
                state_usage[state] = state_usage.get(state, 0) + 1
            
            print("🚦 State Usage Distribution:")
            for state, count in state_usage.items():
                percentage = (count / len(tracking_data['current_state'])) * 100
                print(f"   {state}: {percentage:.1f}% ({count} intervals)")
        
        if tracking_data['time']:
            plot_global_lane_analysis(tracking_data)
        
        try:
            traci.close()
        except:
            pass

def plot_global_lane_analysis(tracking_data):
    """Enhanced visualization for global lane-specific analysis"""
    try:
        fig, axes = plt.subplots(2, 2, figsize=(20, 16))
        
        # Biểu đồ 1: Group Priorities theo thời gian (Enhanced)
        ax1 = axes[0, 0]
        colors = ['blue', 'lightblue', 'red', 'pink']
        labels = ['All Straight+Left', 'All Straight+Right', 'Traditional NS', 'Traditional EW']
        
        for i, (group, color, label) in enumerate(zip(tracking_data['group_priorities'].keys(), colors, labels)):
            if tracking_data['group_priorities'][group]:
                ax1.plot(tracking_data['time'], tracking_data['group_priorities'][group], 
                        label=label, color=color, linewidth=2.5)
        
        # Enhanced threshold lines
        ax1.axhline(y=DECISION_THRESHOLD, color='orange', linestyle='--', 
                   linewidth=2, label=f'Decision Threshold ({DECISION_THRESHOLD})')
        ax1.axhline(y=EMERGENCY_THRESHOLD, color='red', linestyle='--', 
                   linewidth=2, label=f'Emergency Threshold ({EMERGENCY_THRESHOLD})')
        
        # Additional threshold line for preemptive action
        PREEMPTIVE_THRESHOLD = 0.3
        ax1.axhline(y=PREEMPTIVE_THRESHOLD, color='green', linestyle=':', 
                   linewidth=1, label=f'Preemptive Threshold ({PREEMPTIVE_THRESHOLD})')
        
        # Enhanced fill areas
        if tracking_data['time']:
            max_priority = 1.0
            if any(tracking_data['group_priorities'][group] for group in tracking_data['group_priorities']):
                max_priority = max([max(tracking_data['group_priorities'][group]) 
                                  for group in tracking_data['group_priorities'] 
                                  if tracking_data['group_priorities'][group]])
            
            ax1.fill_between(tracking_data['time'], 0, PREEMPTIVE_THRESHOLD, 
                            color='green', alpha=0.2, label='Excellent Zone')
            ax1.fill_between(tracking_data['time'], PREEMPTIVE_THRESHOLD, DECISION_THRESHOLD, 
                            color='yellow', alpha=0.2, label='Acceptable Zone')
            ax1.fill_between(tracking_data['time'], DECISION_THRESHOLD, EMERGENCY_THRESHOLD, 
                            color='orange', alpha=0.2, label='Warning Zone')
            ax1.fill_between(tracking_data['time'], EMERGENCY_THRESHOLD, max_priority * 1.1, 
                            color='red', alpha=0.2, label='Critical Zone')
        
        ax1.set_xlabel('Thời Gian (giây)', fontsize=12)
        ax1.set_ylabel('Lane Group Priority', fontsize=12)
        ax1.set_title('Global Lane-Specific Priority Analysis (Enhanced)', fontsize=14)
        ax1.legend(fontsize=10, loc='upper right')
        ax1.grid(True, alpha=0.3)
        
        # Biểu đồ 2: Enhanced State timeline with better visualization
        ax2 = axes[0, 1]
        if tracking_data['current_state']:
            state_types = list(set(tracking_data['current_state']))
            # Enhanced color mapping
            state_color_map = {
                'all_straight_left_only': '#2E8B57',      # Sea Green
                'all_straight_right_only': '#4169E1',     # Royal Blue
                'NS_traditional': '#FF6347',              # Tomato
                'EW_traditional': '#FFD700',              # Gold
                'all_red': '#DC143C'                      # Crimson
            }
            
            # Add default colors for any unmapped states
            default_colors = plt.cm.Set3(np.linspace(0, 1, len(state_types)))
            for i, state in enumerate(state_types):
                if state not in state_color_map:
                    state_color_map[state] = default_colors[i]
            
            for i in range(len(tracking_data['time']) - 1):
                duration = tracking_data['time'][i+1] - tracking_data['time'][i]
                state = tracking_data['current_state'][i]
                color = state_color_map.get(state, 'gray')
                
                ax2.barh(0, duration, left=tracking_data['time'][i], height=0.8, 
                        color=color, alpha=0.8, edgecolor='black', linewidth=0.5)
                
                # Enhanced state labeling
                if duration > 15:  # Only label if duration > 15s
                    if 'straight_left' in state:
                        short_name = 'SL Global'
                    elif 'straight_right' in state:
                        short_name = 'SR Global'
                    elif 'NS_traditional' in state:
                        short_name = 'NS Trad'
                    elif 'EW_traditional' in state:
                        short_name = 'EW Trad'
                    else:
                        short_name = state.replace('_', ' ')[:8]
                        
                    ax2.text(tracking_data['time'][i] + duration/2, 0, short_name, 
                            ha='center', va='center', fontsize=9, fontweight='bold',
                            bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8))
        
        ax2.set_xlabel('Thời Gian (giây)', fontsize=12)
        ax2.set_ylabel('Traffic Light States', fontsize=12)
        ax2.set_title('Global Lane-Specific State Timeline', fontsize=14)
        ax2.set_ylim(-0.5, 0.5)
        ax2.grid(True, alpha=0.3)
        
        # Biểu đồ 3: Enhanced State change events with annotations
        ax3 = axes[1, 0]
        if tracking_data.get('state_changes'):
            change_times = [change['time'] for change in tracking_data['state_changes']]
            change_types = [change['to'] for change in tracking_data['state_changes']]
            change_priorities = [change.get('target_priority', 0) for change in tracking_data['state_changes']]
            
            # Color code by priority level
            colors = []
            for priority in change_priorities:
                if priority >= EMERGENCY_THRESHOLD:
                    colors.append('red')
                elif priority >= DECISION_THRESHOLD:
                    colors.append('orange')
                else:
                    colors.append('blue')
            
            scatter = ax3.scatter(change_times, range(len(change_times)), 
                       c=colors, s=120, alpha=0.7, marker='o', edgecolors='black')
            
            # Enhanced annotations
            for i, (time, state_type, priority) in enumerate(zip(change_times, change_types, change_priorities)):
                if 'straight_left' in state_type:
                    short_name = 'SL Global'
                elif 'straight_right' in state_type:
                    short_name = 'SR Global'
                elif 'NS_traditional' in state_type:
                    short_name = 'NS Trad'
                elif 'EW_traditional' in state_type:
                    short_name = 'EW Trad'
                else:
                    short_name = state_type.replace('_only', '').replace('_', ' ')
                
                # Priority indicator
                if priority >= EMERGENCY_THRESHOLD:
                    priority_icon = '🚨'
                elif priority >= DECISION_THRESHOLD:
                    priority_icon = '⚠️'
                else:
                    priority_icon = '✅'
                
                annotation_text = f"{priority_icon} {short_name}\n({priority:.2f})"
                
                ax3.annotate(annotation_text, (time, i), xytext=(15, 0), 
                           textcoords='offset points', fontsize=8, 
                           bbox=dict(boxstyle='round,pad=0.4', facecolor='yellow', alpha=0.8),
                           ha='left')
        
        ax3.set_xlabel('Thời Gian (giây)', fontsize=12)
        ax3.set_ylabel('State Change Events', fontsize=12)
        ax3.set_title('Global Lane-Specific State Changes (Priority Coded)', fontsize=14)
        ax3.grid(True, alpha=0.3)
        
        # Biểu đồ 4: Enhanced statistics with performance metrics
        ax4 = axes[1, 1]
        if tracking_data['lane_details']:
            # Enhanced statistics subplot
            directions = ['North', 'South', 'East', 'West']
            lane_types = ['straight_left', 'straight_right']
            
            # Calculate comprehensive statistics
            avg_scores = np.zeros((len(directions), len(lane_types)))
            max_scores = np.zeros((len(directions), len(lane_types)))
            
            for detail in tracking_data['lane_details']:
                analysis = detail['analysis']
                for i, direction in enumerate(directions):
                    if direction in analysis:
                        for j, lane_type in enumerate(lane_types):
                            if lane_type in analysis[direction]:
                                score = analysis[direction][lane_type]['score']
                                avg_scores[i, j] += score
                                max_scores[i, j] = max(max_scores[i, j], score)
            
            if len(tracking_data['lane_details']) > 0:
                avg_scores /= len(tracking_data['lane_details'])
            
            # Create enhanced heatmap
            im = ax4.imshow(avg_scores, cmap='RdYlGn_r', aspect='auto', vmin=0, vmax=1)
            ax4.set_xticks(range(len(lane_types)))
            ax4.set_xticklabels(['Straight+Left', 'Straight+Right'], fontsize=11)
            ax4.set_yticks(range(len(directions)))
            ax4.set_yticklabels(directions, fontsize=11)
            ax4.set_title('Average Lane Performance Heatmap\n(Lower = Better)', fontsize=14)
            
            # Enhanced text annotations with performance indicators
            for i in range(len(directions)):
                for j in range(len(lane_types)):
                    score = avg_scores[i, j]
                    max_score = max_scores[i, j]
                    
                    # Performance indicator
                    if score <= PREEMPTIVE_THRESHOLD:
                        perf_icon = '✅'
                        text_color = 'white'
                    elif score <= DECISION_THRESHOLD:
                        perf_icon = '🟡'
                        text_color = 'black'
                    else:
                        perf_icon = '🔴'
                        text_color = 'white'
                    
                    ax4.text(j, i, f'{perf_icon}\n{score:.2f}\n(Max: {max_score:.2f})', 
                           ha="center", va="center", color=text_color, fontweight='bold',
                           fontsize=9)
            
            # Add colorbar with custom labels
            cbar = plt.colorbar(im, ax=ax4)
            cbar.set_label('Performance Index (Lower = Better)', fontsize=11)
            cbar.set_ticks([0, PREEMPTIVE_THRESHOLD, DECISION_THRESHOLD, EMERGENCY_THRESHOLD, 1.0])
            cbar.set_ticklabels(['Excellent', 'Good', 'Warning', 'Critical', 'Severe'])
        
        plt.tight_layout()
        
        # Enhanced filename with timestamp
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'global_lane_analysis_enhanced_{timestamp}.png'
        
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"📊 Đã lưu biểu đồ phân tích global lane (enhanced): '{filename}'")
        
        # Additional summary plot if there are state changes
        if tracking_data.get('state_changes'):
            plt.figure(figsize=(12, 6))
            
            # State change frequency analysis
            state_change_counts = {}
            for change in tracking_data['state_changes']:
                state = change['to']
                state_change_counts[state] = state_change_counts.get(state, 0) + 1
            
            if state_change_counts:
                states = list(state_change_counts.keys())
                counts = list(state_change_counts.values())
                
                # Enhanced bar chart
                bars = plt.bar(range(len(states)), counts, 
                              color=['green' if 'straight_left' in s else 'blue' if 'straight_right' in s 
                                    else 'red' if 'NS' in s else 'orange' for s in states],
                              alpha=0.7, edgecolor='black')
                
                # Add value labels on bars
                for bar, count in zip(bars, counts):
                    plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1, 
                            str(count), ha='center', va='bottom', fontweight='bold')
                
                plt.xticks(range(len(states)), 
                          [s.replace('_only', '').replace('_', '\n') for s in states], 
                          rotation=45, ha='right')
                plt.ylabel('Number of Activations', fontsize=12)
                plt.title('State Activation Frequency Analysis', fontsize=14)
                plt.grid(True, alpha=0.3, axis='y')
                
                plt.tight_layout()
                summary_filename = f'state_frequency_analysis_{timestamp}.png'
                plt.savefig(summary_filename, dpi=150, bbox_inches='tight')
                print(f"📊 Đã lưu biểu đồ tần suất state: '{summary_filename}'")
        
        plt.show()
        
    except Exception as e:
        print(f"❌ Lỗi khi vẽ biểu đồ: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    start_sumo()
    run_global_lane_simulation()