import os
import sys
import traci
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

# Thêm SUMO vào đường dẫn Python
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Vui lòng khai báo biến môi trường 'SUMO_HOME'")

# CẤU HÌNH TỐI ƯU KẾT HỢP - GIẢM THIỂU PHANH KHẨN CẤP TỐI ĐA
MIN_GREEN_TIME = 50          # Tăng từ 45 lên 50 giây để ổn định hơn
MAX_GREEN_TIME = 120         # Tăng từ 110 lên 120 giây
EVAL_INTERVAL = 30           # Tăng từ 25 lên 30 giây để giảm tần suất đánh giá
YELLOW_TIME = 15             # Tăng từ 12 lên 15 giây cho thời gian phản ứng tối ưu
CLEARANCE_TIME = 8           # Tăng từ 6 lên 8 giây để đảm bảo giao lộ hoàn toàn trống
COOLDOWN_PERIOD = 30         # Tăng từ 25 lên 30 giây
THRESHOLD = 0.7              # Giữ ngưỡng không đổi
JUNCTION_CLEARING_DISTANCE = 80  # Tăng từ 70 lên 80 mét cho vùng an toàn mở rộng
SAFETY_SPEED_BUFFER = 1.0    # Giảm từ 1.5 xuống 1.0 để phát hiện tốt hơn
YELLOW_TIME_BUFFER = 5       # Tăng từ 4 lên 5 giây cho an toàn bổ sung
MAX_DECELERATION = 2.0       # Giảm từ 2.5 xuống 2.0 m/s² cho phanh nhẹ nhàng hơn
MIN_PHASE_GAP = 80           # Tăng từ 60 lên 80 (minimum 8 giây giữa các lần chuyển pha)
AGGRESSIVE_DETECTION_DISTANCE = 100  # Tăng từ 80 lên 100 mét
SPEED_THRESHOLD_HIGH = 12    # Giảm từ 15 xuống 12 m/s để phát hiện sớm hơn
SPEED_THRESHOLD_MEDIUM = 8   # Mới: Ngưỡng tốc độ trung bình
DECISION_ZONE_MULTIPLIER = 1.5  # Mới: Hệ số mở rộng vùng quyết định

last_phase_change_step = -MIN_PHASE_GAP

def start_sumo():
    """Khởi động SUMO"""
    sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
    sumoCmd = [sumoBinary, '-c', r"C:\Users\Admin\Downloads\sumo test\New folder\dataset.sumocfg", '--step-length', '0.1']
    traci.start(sumoCmd)

def get_lane_metrics(detector_id):
    """Lấy các chỉ số cho làn đường cụ thể - Cải tiến"""
    metrics = {}
    
    try:
        # Độ dài hàng đợi (số xe)
        metrics['queue_length'] = traci.lanearea.getJamLengthVehicle(detector_id)
        
        # Thời gian chờ (thời gian chờ tối đa của các xe trên bộ dò)
        vehicles = traci.lanearea.getLastStepVehicleIDs(detector_id)
        wait_times = [traci.vehicle.getWaitingTime(veh) for veh in vehicles] if vehicles else [0]
        metrics['waiting_time'] = max(wait_times) if wait_times else 0
        
        # Mật độ làn đường (xe trên chiều dài làn)
        occupancy = traci.lanearea.getLastStepOccupancy(detector_id) / 100.0
        metrics['density'] = occupancy
        
        # Tốc độ trung bình
        metrics['avg_speed'] = traci.lanearea.getLastStepMeanSpeed(detector_id)
        
        # Lưu lượng xe (xe/giờ)
        metrics['flow_rate'] = traci.lanearea.getLastStepVehicleNumber(detector_id) * 3600
        
        # Mới: Số xe dừng
        metrics['stopped_vehicles'] = len([v for v in vehicles if traci.vehicle.getSpeed(v) < 1.0]) if vehicles else 0
        
    except Exception as e:
        # Trả về giá trị mặc định nếu có lỗi
        metrics = {
            'queue_length': 0,
            'waiting_time': 0,
            'density': 0,
            'avg_speed': 0,
            'flow_rate': 0,
            'stopped_vehicles': 0
        }
    
    return metrics

def calculate_status(metrics_list):
    """Tính toán trạng thái dựa trên nhiều chỉ số - Cải tiến trọng số"""
    # Trọng số được điều chỉnh để tối ưu hóa
    weights = {
        'queue_length': 0.25,      # Giảm từ 0.3
        'waiting_time': 0.35,      # Giảm từ 0.4
        'density': 0.2,           # Giữ nguyên
        'avg_speed': 0.05,        # Tăng từ 0.0
        'flow_rate': 0.1,         # Giữ nguyên
        'stopped_vehicles': 0.05   # Mới thêm
    }
    
    status_components = []
    
    for metrics in metrics_list:
        # Tính điểm cho mỗi làn đường với công thức cải tiến
        lane_score = (
            weights['queue_length'] * min(metrics['queue_length'] / 12, 1) +  # Tăng mẫu số
            weights['waiting_time'] * min(metrics['waiting_time'] / 80, 1) +  # Tăng mẫu số
            weights['density'] * metrics['density'] +
            weights['avg_speed'] * (1 - min(metrics['avg_speed'] / 15, 1)) +  # Đảo ngược logic
            weights['flow_rate'] * (metrics['flow_rate'] / 2000) +            # Tăng mẫu số
            weights['stopped_vehicles'] * min(metrics['stopped_vehicles'] / 8, 1)  # Mới
        )
        status_components.append(lane_score)
    
    # Tính trạng thái tổng hợp với thuật toán cải tiến
    if not status_components:
        return 0, []
    
    total_status = sum(status_components)
    max_component = max(status_components)
    avg_component = np.mean(status_components)
    
    # Công thức cải tiến với trọng số phân tán
    adjusted_status = 0.5 * total_status + 0.3 * max_component + 0.2 * avg_component
    
    return adjusted_status, status_components

def can_stop_safely(speed, distance):
    """Kiểm tra an toàn dừng xe - Cải tiến toàn diện"""
    if speed < 0.5:  # Xe di chuyển rất chậm
        return True
    
    # Thời gian phản ứng động theo tốc độ và điều kiện
    if speed > SPEED_THRESHOLD_HIGH:      # > 12 m/s
        t_reaction = 2.0      # Tăng thời gian phản ứng cho xe nhanh
    elif speed > SPEED_THRESHOLD_MEDIUM:  # > 8 m/s
        t_reaction = 1.5
    elif speed > 5:
        t_reaction = 1.2
    else:
        t_reaction = 1.0
    
    # Tính khoảng cách dừng tối thiểu với công thức cải tiến
    min_stopping_distance = (speed ** 2) / (2 * MAX_DECELERATION) + speed * t_reaction
    
    # Hệ số an toàn động theo tốc độ
    if speed > SPEED_THRESHOLD_HIGH:
        safety_buffer = 2.0      # Buffer lớn hơn cho xe nhanh
    elif speed > SPEED_THRESHOLD_MEDIUM:
        safety_buffer = 1.7
    elif speed > 5:
        safety_buffer = 1.5
    else:
        safety_buffer = 1.3
    
    min_stopping_distance *= safety_buffer
    
    # Kiểm tra thời gian
    stopping_time = speed / MAX_DECELERATION + t_reaction
    time_to_intersection = distance / max(speed, 0.1)
    
    # Kiểm tra bổ sung cho xe tốc độ cao
    if speed > SPEED_THRESHOLD_HIGH and distance < AGGRESSIVE_DETECTION_DISTANCE:
        return False
    
    # Kiểm tra vùng quyết định mở rộng
    decision_zone = 50 * DECISION_ZONE_MULTIPLIER  # 75 mét
    if speed > SPEED_THRESHOLD_MEDIUM and distance < decision_zone:
        return distance > min_stopping_distance * 1.2  # Thêm 20% buffer
    
    return distance > min_stopping_distance and time_to_intersection > stopping_time * 1.3

def is_safe_to_change_phase(direction_detectors, junction_id='E3'):
    """Kiểm tra an toàn chuyển pha - Tối ưu hóa toàn diện"""
    global last_phase_change_step
    current_step = traci.simulation.getTime() * 10
    
    # Bắt buộc khoảng cách tối thiểu giữa các lần chuyển pha
    if current_step - last_phase_change_step < MIN_PHASE_GAP:
        print(f"Phase change blocked: Too soon (gap: {current_step - last_phase_change_step})")
        return False
    
    # Kiểm tra xe đang tiến gần với thuật toán nâng cao
    for detector_id in direction_detectors:
        vehicles = traci.lanearea.getLastStepVehicleIDs(detector_id)
        for vehicle in vehicles:
            try:
                speed = traci.vehicle.getSpeed(vehicle)
                
                # Phát hiện cả xe chậm với ngưỡng thấp hơn
                if speed > SAFETY_SPEED_BUFFER:
                    distance = traci.vehicle.getLanePosition(vehicle)
                    lane_length = traci.lane.getLength(traci.vehicle.getLaneID(vehicle))
                    remaining_distance = lane_length - distance
                    
                    # Vùng nguy hiểm động theo tốc độ
                    if speed > SPEED_THRESHOLD_HIGH:
                        danger_zone = 60
                        critical_speed = 10
                    elif speed > SPEED_THRESHOLD_MEDIUM:
                        danger_zone = 50
                        critical_speed = 8
                    else:
                        danger_zone = 40
                        critical_speed = 6
                    
                    if remaining_distance < danger_zone and speed > critical_speed:
                        print(f"[{vehicle}] In danger zone: {remaining_distance:.1f}m at {speed:.1f} m/s")
                        return False
                    
                    # Kiểm tra khả năng dừng an toàn nâng cao
                    if not can_stop_safely(speed, remaining_distance):
                        print(f"[{vehicle}] Cannot stop safely: {remaining_distance:.1f}m at {speed:.1f} m/s")
                        return False
                    
                    # Tính toán thời gian với hệ số an toàn động
                    if speed > SPEED_THRESHOLD_HIGH:
                        speed_factor = 1.6
                    elif speed > SPEED_THRESHOLD_MEDIUM:
                        speed_factor = 1.4
                    else:
                        speed_factor = 1.3
                    
                    time_to_intersection = (remaining_distance / max(speed, 1.0)) * speed_factor
                    
                    # Buffer thời gian đèn vàng động
                    if speed > SPEED_THRESHOLD_HIGH:
                        yellow_buffer = YELLOW_TIME_BUFFER + 2
                    elif speed > SPEED_THRESHOLD_MEDIUM:
                        yellow_buffer = YELLOW_TIME_BUFFER + 1
                    else:
                        yellow_buffer = YELLOW_TIME_BUFFER
                    
                    if 0 < time_to_intersection < YELLOW_TIME + yellow_buffer:
                        print(f"[{vehicle}] In yellow time conflict zone")
                        return False
                    
                    # Vùng quyết định mở rộng cho xe nhanh
                    if speed > SPEED_THRESHOLD_HIGH:
                        decision_distance = 90
                    elif speed > SPEED_THRESHOLD_MEDIUM:
                        decision_distance = 75
                    else:
                        decision_distance = 60
                    
                    if speed > SPEED_THRESHOLD_MEDIUM and remaining_distance < decision_distance:
                        print(f"[{vehicle}] In extended decision zone")
                        return False
                        
            except Exception as e:
                continue
    
    # Kiểm tra xe trong giao lộ
    vehicles_in_junction = get_vehicles_in_junction(junction_id)
    if vehicles_in_junction:
        print(f"Junction not clear: {len(vehicles_in_junction)} vehicles")
        return False
    
    # Cập nhật thời gian chuyển pha cuối
    last_phase_change_step = current_step
    return True

def get_vehicles_in_junction(junction_id):
    """Lấy xe trong giao lộ - Mở rộng vùng phát hiện"""
    junction_vehicles = []
    all_vehicles = traci.vehicle.getIDList()
    
    for veh_id in all_vehicles:
        try:
            road_id = traci.vehicle.getRoadID(veh_id)
            
            # Xe đang ở trong giao lộ
            if road_id.startswith(':'):
                junction_vehicles.append(veh_id)
                continue
            
            # Kiểm tra xe gần giao lộ với thuật toán nâng cao
            lane_id = traci.vehicle.getLaneID(veh_id)
            lane_length = traci.lane.getLength(lane_id)
            position = traci.vehicle.getLanePosition(veh_id)
            speed = traci.vehicle.getSpeed(veh_id)
            
            # Vùng phát hiện động theo tốc độ
            if speed > SPEED_THRESHOLD_HIGH:
                detection_distance = JUNCTION_CLEARING_DISTANCE + 25
            elif speed > SPEED_THRESHOLD_MEDIUM:
                detection_distance = JUNCTION_CLEARING_DISTANCE + 15
            else:
                detection_distance = JUNCTION_CLEARING_DISTANCE
            
            estimated_time_to_junction = (lane_length - position) / max(speed, 1.0)
            
            # Điều kiện phát hiện gần giao lộ cải tiến
            near_junction = (
                position > lane_length - detection_distance or 
                (speed > 2 and estimated_time_to_junction < 6) or
                (speed > SPEED_THRESHOLD_MEDIUM and estimated_time_to_junction < 8)
            )
            
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
    """Đếm sự kiện phanh khẩn cấp - Cải tiến ngưỡng"""
    count = 0
    for veh_id in traci.vehicle.getIDList():
        try:
            # Ngưỡng phanh khẩn cấp nghiêm ngặt hơn
            acceleration = traci.vehicle.getAcceleration(veh_id)
            speed = traci.vehicle.getSpeed(veh_id)
            
            # Phân loại theo tốc độ để có ngưỡng phù hợp
            if speed > 10:
                emergency_threshold = -3.5  # Nghiêm ngặt hơn cho xe nhanh
            elif speed > 5:
                emergency_threshold = -4.0
            else:
                emergency_threshold = -4.5
            
            if acceleration < emergency_threshold:
                count += 1
        except:
            continue
    return count

def wait_for_junction_clearing(junction_id='E3', max_wait=10):  # Tăng lên 10 giây
    """Chờ giao lộ trống hoàn toàn - Cải tiến"""
    wait_steps = 0
    cleared = False
    consecutive_clear_steps = 0
    required_clear_steps = 20  # Yêu cầu 2 giây liên tiếp trống
    
    while wait_steps < max_wait * 10:
        traci.simulationStep()
        vehicles_in_junction = get_vehicles_in_junction(junction_id)
        
        if not vehicles_in_junction:
            consecutive_clear_steps += 1
            if consecutive_clear_steps >= required_clear_steps:
                cleared = True
                break
        else:
            consecutive_clear_steps = 0  # Reset nếu có xe
        
        wait_steps += 1
    
    if not cleared:
        print("Cảnh báo: Giao lộ không thể trống hoàn toàn sau thời gian chờ tối đa!")
    else:
        print(f"Giao lộ đã trống sau {wait_steps/10:.1f} giây")
    
    return wait_steps

def plot_traffic_status(status_data, threshold):
    """Vẽ biểu đồ trạng thái giao thông - Giữ nguyên với tiêu đề cập nhật"""
    plt.figure(figsize=(15, 10))
    
    # Vẽ biểu đồ trạng thái giao thông vs ngưỡng
    for direction, color in zip(['North', 'South', 'East', 'West'], ['blue', 'green', 'red', 'orange']):
        label = {'North': 'Hướng Bắc', 'South': 'Hướng Nam', 
                 'East': 'Hướng Đông', 'West': 'Hướng Tây'}[direction]
        plt.plot(status_data['time'], status_data[direction], label=label, color=color, linewidth=2)
    
    # Thêm đường ngưỡng
    plt.axhline(y=threshold, color='r', linestyle='--', linewidth=2, 
                label=f'Ngưỡng ({threshold})')
    
    # Thêm vùng màu để thể hiện trạng thái TỐT/XẤU
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
    plt.title('Hệ Thống Điều Khiển Đèn Giao Thông Tối Ưu (Kết Hợp Tất Cả Cải Tiến)', fontsize=16)
    plt.legend(fontsize=12)
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('he_thong_dieu_khien_toi_uu.png', dpi=150)
    print("Đã lưu biểu đồ dưới dạng 'he_thong_dieu_khien_toi_uu.png'")
    plt.show()

def run_simulation():
    """Chạy mô phỏng với tất cả cải tiến được kết hợp"""
    # Xác định nhóm bộ dò theo hướng
    detector_groups = {
        'North': ['E1-3-1', 'E1-3-2'],
        'East': ['E3-4-1', 'E3-4-2'],
        'South': ['E5-3-1', 'E5-3-2'],
        'West': ['E3-2-1', 'E3-2-2']
    }
    
    # Các pha của đèn giao thông
    direction_to_phase = {
        'North': 0, 
        'South': 0,
        'East': 2, 
        'West': 2
    }
    
    # Theo dõi số liệu
    metrics_history = defaultdict(list)
    current_phase = 0
    phase_duration = 0
    cooldown_timer = 0
    
    # Dữ liệu cho trực quan hóa
    status_data = {'time': [], 'North': [], 'South': [], 'East': [], 'West': []}
    
    # Vòng lặp mô phỏng chính
    step = 0
    in_transition = False
    transition_stage = 0
    next_green_phase = None
    emergency_braking_events = 0
    data_collection_interval = 50
    
    print("Bắt đầu mô phỏng với hệ thống điều khiển đèn giao thông tối ưu...")
    print(f"Cấu hình: GREEN({MIN_GREEN_TIME}-{MAX_GREEN_TIME}s), YELLOW({YELLOW_TIME}s), EVAL({EVAL_INTERVAL}s)")
    
    while step < 10000:
        traci.simulationStep()
        
        # Kiểm tra sự kiện phanh khẩn cấp
        if step % 10 == 0:
            new_events = count_emergency_braking_events()
            if new_events > 0:
                print(f"⚠️  Thời điểm {step/10:.1f}s: Phát hiện {new_events} sự kiện phanh khẩn cấp")
            emergency_braking_events += new_events
        
        # Lấy tất cả các chỉ số từ bộ dò
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
        
        # Thu thập dữ liệu cho trực quan hóa
        if step % data_collection_interval == 0:
            time_sec = step / 10.0
            status_data['time'].append(time_sec)
            for direction, data in all_metrics.items():
                status_data[direction].append(data['status'])
        
        # Tăng bộ đếm thời gian pha
        phase_duration += 1
        
        # Xử lý thời gian làm mát
        if cooldown_timer > 0:
            cooldown_timer -= 1
            step += 1
            continue
        
        # Xử lý các pha chuyển tiếp
        if in_transition:
            if transition_stage == 1 and phase_duration >= YELLOW_TIME:
                # Chuyển sang giai đoạn đèn đỏ toàn phần
                transition_stage = 2
                phase_duration = 0
                print(f"🔴 Thời điểm {step/10:.1f}s: Giai đoạn đèn đỏ toàn phần")
                
            elif transition_stage == 2 and phase_duration >= CLEARANCE_TIME:
                # Đợi xe đi qua hết giao lộ
                print(f"⏳ Thời điểm {step/10:.1f}s: Đang chờ giao lộ trống...")
                wait_steps = wait_for_junction_clearing()
                
                # Đặt pha xanh tiếp theo
                in_transition = False
                transition_stage = 0
                current_phase = next_green_phase
                traci.trafficlight.setPhase('E3', current_phase)
                phase_duration = 0
                cooldown_timer = COOLDOWN_PERIOD
                print(f"🟢 Thời điểm {(step+wait_steps)/10:.1f}s: Pha xanh mới (pha {current_phase})")
                
                step += wait_steps
            step += 1
            continue
        
        # Đánh giá pha định kỳ
        if phase_duration % EVAL_INTERVAL == 0:
            current_directions = [dir for dir, phase in direction_to_phase.items() if phase == current_phase]
            opposing_directions = [dir for dir, phase in direction_to_phase.items() if phase != current_phase]
            
            current_detectors = []
            for direction in current_directions:
                current_detectors.extend(detector_groups[direction])
            
            # Logic chuyển đổi pha cải tiến
            current_good = any(all_metrics[dir]['is_good'] for dir in current_directions)
            opposing_need = any(all_metrics[dir]['status'] > THRESHOLD * 1.4 for dir in opposing_directions)  # Tăng ngưỡng
            
            should_change = (not current_good or opposing_need) and phase_duration >= MIN_GREEN_TIME
            
            if should_change:
                if is_safe_to_change_phase(current_detectors):
                    # Bắt đầu pha vàng
                    in_transition = True
                    transition_stage = 1
                    if current_phase == 0:
                        traci.trafficlight.setPhase('E3', 1)  # N-S vàng
                        next_green_phase = 2
                        print(f"🟡 Thời điểm {step/10:.1f}s: Đèn vàng N-S (trạng thái xấu)")
                    else:
                        traci.trafficlight.setPhase('E3', 3)  # E-W vàng
                        next_green_phase = 0
                        print(f"🟡 Thời điểm {step/10:.1f}s: Đèn vàng E-W (trạng thái xấu)")
                    phase_duration = 0
                else:
                    print(f"⏸️  Thời điểm {step/10:.1f}s: Hoãn chuyển pha - không an toàn")
            
            # Bắt buộc chuyển đổi khi đạt thời gian tối đa
            elif phase_duration >= MAX_GREEN_TIME:
                if is_safe_to_change_phase(current_detectors):
                    in_transition = True
                    transition_stage = 1
                    if current_phase == 0:
                        traci.trafficlight.setPhase('E3', 1)
                        next_green_phase = 2
                        print(f"🟡 Thời điểm {step/10:.1f}s: Đèn vàng N-S (thời gian tối đa)")
                    else:
                        traci.trafficlight.setPhase('E3', 3)
                        next_green_phase = 0
                        print(f"🟡 Thời điểm {step/10:.1f}s: Đèn vàng E-W (thời gian tối đa)")
                    phase_duration = 0
                else:
                    # Kéo dài thêm nếu không an toàn
                    phase_duration = MAX_GREEN_TIME - 25
                    print(f"⏰ Thời điểm {step/10:.1f}s: Kéo dài pha xanh thêm - chưa an toàn")
        
        # In trạng thái định kỳ
        if step % 300 == 0:
            print(f"\n📊 Thời gian: {step / 10} giây")
            for direction, data in all_metrics.items():
                direction_name = {'North': 'Bắc', 'South': 'Nam', 'East': 'Đông', 'West': 'Tây'}[direction]
                status = "TỐT ✅" if data['is_good'] else "XẤU ❌"
                print(f"   Hướng {direction_name}: {data['status']:.2f} ({status})")
            print(f"   Phanh khẩn cấp: {emergency_braking_events} sự kiện")
            print("-" * 50)
        
        step += 1
    
    # Kết thúc mô phỏng
    print("\n🏁 Mô phỏng hoàn thành!")
    
    # Thống kê tổng hợp
    print("\n📈 Thống kê tổng hợp:")
    avg_waiting_times = {}
    avg_queue_lengths = {}
    
    for direction, history in metrics_history.items():
        avg_waiting_times[direction] = np.mean([np.mean([m['waiting_time'] for m in h['metrics']]) 
                                               for h in history])
        avg_queue_lengths[direction] = np.mean([np.mean([m['queue_length'] for m in h['metrics']]) 
                                              for h in history])
    
    total_avg_wait = np.mean(list(avg_waiting_times.values()))
    total_avg_queue = np.mean(list(avg_queue_lengths.values()))
    
    print(f"⏱️  Thời gian chờ trung bình: {total_avg_wait:.2f} giây")
    print(f"🚗 Độ dài hàng đợi trung bình: {total_avg_queue:.2f} xe")
    print(f"⚠️  Tổng sự kiện phanh khẩn cấp: {emergency_braking_events}")
    
    direction_names = {'North': 'Bắc', 'South': 'Nam', 'East': 'Đông', 'West': 'Tây'}
    for direction, avg_wait in avg_waiting_times.items():
        dir_name = direction_names[direction]
        print(f"   {dir_name}: Chờ {avg_wait:.1f}s, Hàng đợi {avg_queue_lengths[direction]:.1f}")
    
    # Tạo biểu đồ
    plot_traffic_status(status_data, THRESHOLD)

    traci.close()

if __name__ == "__main__":
    start_sumo()
    run_simulation()