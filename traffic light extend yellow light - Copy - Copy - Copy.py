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

# CẤU HÌNH TỐI ƯU CHO RẼ TRÁI
MIN_GREEN_TIME = 45          # Giảm xuống để linh hoạt hơn
MAX_GREEN_TIME = 110         # Giảm xuống để tăng tần suất đánh giá
EVAL_INTERVAL = 20           # Giảm xuống để phản ứng nhanh hơn
YELLOW_TIME = 12             
CLEARANCE_TIME = 6           
COOLDOWN_PERIOD = 20         # Giảm thời gian cooldown
THRESHOLD = 0.6              # Giảm ngưỡng để dễ kích hoạt chuyển pha hơn
JUNCTION_CLEARING_DISTANCE = 70  
SAFETY_SPEED_BUFFER = 1.0    
YELLOW_TIME_BUFFER = 4       
MAX_DECELERATION = 2.5       
MIN_PHASE_GAP = 60           
AGGRESSIVE_DETECTION_DISTANCE = 80  
SPEED_THRESHOLD_HIGH = 15    
SPEED_THRESHOLD_MEDIUM = 8   
DECISION_ZONE_MULTIPLIER = 1.3

# CẤU HÌNH MỚI CHO RẼ TRÁI - ĐÃ ĐIỀU CHỈNH
LEFT_TURN_PRIORITY_MULTIPLIER = 5.0    # Tăng ưu tiên cho rẽ trái
LEFT_TURN_MIN_VEHICLES = 1             # Giảm xuống 1 để kích hoạt ngay cả khi chỉ có 1 xe
LEFT_TURN_MAX_WAIT = 45                # Giảm thời gian chờ tối đa
LEFT_TURN_PHASE_DURATION = 35          # Tăng thời lượng pha rẽ trái
PROTECTED_LEFT_TURN_ENABLED = True     # Giữ nguyên
LEFT_TURN_DETECTION_DISTANCE = 150     # Thêm mới - khoảng cách phát hiện xe rẽ trái
LEFT_TURN_PHASE_EXTENSION = 5          # Thêm mới - mở rộng thời gian nếu còn xe
MAX_LEFT_TURN_VEHICLES_PER_CYCLE = 5   # Thêm mới - số lượng xe tối đa trên mỗi chu kỳ

last_phase_change_step = -MIN_PHASE_GAP

def start_sumo():
    """Khởi động SUMO"""
    sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
    sumoCmd = [sumoBinary, '-c', r"C:\Users\Admin\Downloads\sumo test\New folder\dataset.sumocfg", '--step-length', '0.1']
    traci.start(sumoCmd)

def get_lane_metrics(detector_id):
    """Lấy các chỉ số cho làn đường cụ thể - Cải tiến cho phát hiện rẽ trái tốt hơn"""
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
        
        # Số xe dừng
        metrics['stopped_vehicles'] = len([v for v in vehicles if traci.vehicle.getSpeed(v) < 1.0]) if vehicles else 0
        
        # CẢI TIẾN: Phát hiện xe rẽ trái - cải thiện thuật toán phát hiện
        left_turn_vehicles = []
        left_turn_waiting_time = 0
        
        for vehicle in vehicles:
            try:
                # Sử dụng nhiều phương pháp kết hợp để phát hiện xe rẽ trái
                is_left_turn = False
                
                # 1. Kiểm tra tín hiệu rẽ
                signals = traci.vehicle.getSignals(vehicle)
                if signals & 4:  # Bit 2 được bật (rẽ trái)
                    is_left_turn = True
                
                # 2. Kiểm tra bằng route
                if not is_left_turn:
                    route = traci.vehicle.getRoute(vehicle)
                    current_edge = traci.vehicle.getRoadID(vehicle)
                    if current_edge in route:
                        current_idx = route.index(current_edge)
                        if current_idx < len(route) - 1:
                            next_edge = route[current_idx + 1]
                            if is_left_turn_movement(current_edge, next_edge):
                                is_left_turn = True
                
                # 3. Kiểm tra thông qua ID lane nếu có quy ước đặt tên
                if not is_left_turn:
                    lane_id = traci.vehicle.getLaneID(vehicle)
                    if "_LEFT" in lane_id.upper() or "_LT" in lane_id.upper() or lane_id.endswith("_2"):
                        is_left_turn = True
                
                # 4. Kiểm tra vị trí xe và hướng di chuyển kế tiếp
                if not is_left_turn and detector_id:
                    detector_lane = detector_id.split("-")[0]
                    # Thêm các logic phát hiện dựa trên đặc điểm cụ thể của mô phỏng
                    if detector_lane == "E1" and "E4" in route:
                        is_left_turn = True
                    elif detector_lane == "E5" and "E2" in route:
                        is_left_turn = True
                    elif detector_lane == "-E2" and "-E5" in route:
                        is_left_turn = True
                    elif detector_lane == "-E4" and "-E1" in route:
                        is_left_turn = True
                
                if is_left_turn:
                    left_turn_vehicles.append(vehicle)
                    wait_time = traci.vehicle.getWaitingTime(vehicle)
                    left_turn_waiting_time = max(left_turn_waiting_time, wait_time)
            except:
                continue
        
        metrics['left_turn_vehicles'] = len(left_turn_vehicles)
        metrics['left_turn_waiting_time'] = left_turn_waiting_time
        metrics['left_turn_vehicle_ids'] = left_turn_vehicles  # Lưu ID các xe rẽ trái
        
    except Exception as e:
        print(f"Lỗi khi lấy chỉ số từ detector {detector_id}: {str(e)}")
        # Trả về giá trị mặc định nếu có lỗi
        metrics = {
            'queue_length': 0,
            'waiting_time': 0,
            'density': 0,
            'avg_speed': 0,
            'flow_rate': 0,
            'stopped_vehicles': 0,
            'left_turn_vehicles': 0,
            'left_turn_waiting_time': 0,
            'left_turn_vehicle_ids': []
        }
    
    return metrics

def is_left_turn_movement(current_edge, next_edge):
    """Phát hiện chuyển động rẽ trái dựa trên tên edge - ĐÃ CẢI TIẾN"""
    # Mapping hướng di chuyển dựa trên tên edge
    edge_directions = {
        'E1': 'south_to_center',  # Từ Nam lên
        'E2': 'center_to_west',   # Đi Tây
        'E3': 'center',           # Tâm giao lộ
        'E4': 'center_to_east',   # Đi Đông  
        'E5': 'north_to_center',  # Từ Bắc xuống
        '-E1': 'center_to_south', # Đi Nam
        '-E2': 'west_to_center',  # Từ Tây vào
        '-E4': 'east_to_center',  # Từ Đông vào
        '-E5': 'center_to_north'  # Đi Bắc
    }
    
    # Logic rẽ trái đơn giản - MỞ RỘNG DANH SÁCH
    left_turn_patterns = [
        ('E1', 'E4'),     # Nam -> Đông (rẽ trái)
        ('E1', '-E2'),    # Nam -> Tây (rẽ trái) 
        ('-E2', 'E1'),    # Tây -> Nam (rẽ trái)
        ('-E2', '-E5'),   # Tây -> Bắc (rẽ trái)
        ('-E4', '-E1'),   # Đông -> Nam (rẽ trái)
        ('-E4', 'E5'),    # Đông -> Bắc (rẽ trái)
        ('E5', '-E4'),    # Bắc -> Đông (rẽ trái)
        ('E5', 'E2'),     # Bắc -> Tây (rẽ trái)
        # Thêm các mẫu có thể thiếu
        ('E1', '-E5'),    # Nam -> Bắc (qua giao lộ)
        ('E5', '-E1')     # Bắc -> Nam (qua giao lộ)
    ]
    
    return (current_edge, next_edge) in left_turn_patterns

def calculate_status(metrics_list, is_left_turn_priority=False):
    """Tính toán trạng thái với ưu tiên rẽ trái - ĐÃ CẢI TIẾN"""
    # Trọng số cơ bản
    weights = {
        'queue_length': 0.25,
        'waiting_time': 0.35,
        'density': 0.2,
        'avg_speed': 0.05,
        'flow_rate': 0.1,
        'stopped_vehicles': 0.05
    }
    
    # Trọng số đặc biệt cho rẽ trái - TĂNG TRỌNG SỐ
    left_turn_weights = {
        'left_turn_vehicles': 0.4,        # Tăng trọng số
        'left_turn_waiting_time': 0.5     # Tăng trọng số
    }
    
    status_components = []
    
    for metrics in metrics_list:
        # Tính điểm cơ bản
        lane_score = (
            weights['queue_length'] * min(metrics['queue_length'] / 10, 1) +
            weights['waiting_time'] * min(metrics['waiting_time'] / 60, 1) +
            weights['density'] * metrics['density'] +
            weights['avg_speed'] * (1 - min(metrics['avg_speed'] / 15, 1)) +
            weights['flow_rate'] * (metrics['flow_rate'] / 1800) +
            weights['stopped_vehicles'] * min(metrics['stopped_vehicles'] / 6, 1)
        )
        
        # Thêm điểm ưu tiên rẽ trái
        if metrics['left_turn_vehicles'] > 0:
            # ĐÃ CẢI THIỆN: Ngay cả khi chỉ có 1 xe rẽ trái, vẫn tăng điểm đáng kể
            left_turn_score = (
                left_turn_weights['left_turn_vehicles'] +  # Luôn cộng điểm cơ bản
                left_turn_weights['left_turn_waiting_time'] * min(metrics['left_turn_waiting_time'] / LEFT_TURN_MAX_WAIT, 1)
            )
            
            # Thêm hệ số dựa trên thời gian chờ
            if metrics['left_turn_waiting_time'] > LEFT_TURN_MAX_WAIT * 0.5:
                left_turn_score *= 1.5  # Tăng 50% điểm khi thời gian chờ vượt quá 50% ngưỡng
            
            # Nhân với hệ số ưu tiên
            if is_left_turn_priority:
                left_turn_score *= LEFT_TURN_PRIORITY_MULTIPLIER
            
            lane_score += left_turn_score
        
        status_components.append(lane_score)
    
    if not status_components:
        return 0, []
    
    total_status = sum(status_components)
    max_component = max(status_components)
    avg_component = np.mean(status_components)
    
    # Công thức tính toán cuối
    adjusted_status = 0.4 * total_status + 0.4 * max_component + 0.2 * avg_component
    
    return adjusted_status, status_components

def needs_left_turn_phase(detector_groups, all_metrics):
    """Kiểm tra xem có cần pha rẽ trái không - ĐÃ CẢI TIẾN"""
    if not PROTECTED_LEFT_TURN_ENABLED:
        return False, None
    
    left_turn_demand = {}
    
    for direction, detectors in detector_groups.items():
        total_left_turn_vehicles = 0
        max_left_turn_waiting = 0
        left_turn_vehicle_ids = []
        
        for detector in detectors:
            metrics = get_lane_metrics(detector)
            total_left_turn_vehicles += metrics['left_turn_vehicles']
            max_left_turn_waiting = max(max_left_turn_waiting, metrics['left_turn_waiting_time'])
            left_turn_vehicle_ids.extend(metrics.get('left_turn_vehicle_ids', []))
        
        # ĐÃ CẢI THIỆN: Công thức tính độ ưu tiên
        # Ngay cả khi chỉ có 1 xe rẽ trái với thời gian chờ dài, vẫn ưu tiên cao
        priority_score = total_left_turn_vehicles * 10
        if max_left_turn_waiting > 0:
            # Trọng số tăng theo thời gian chờ theo hàm mũ
            priority_score += min(max_left_turn_waiting * 0.5, LEFT_TURN_MAX_WAIT) ** 1.5
        
        left_turn_demand[direction] = {
            'vehicles': total_left_turn_vehicles,
            'max_waiting': max_left_turn_waiting,
            'priority_score': priority_score,
            'vehicle_ids': left_turn_vehicle_ids
        }
    
    # Tìm hướng cần pha rẽ trái nhất
    best_direction = None
    best_score = 0
    
    for direction, demand in left_turn_demand.items():
        # ĐÃ CẢI THIỆN: Điều kiện ưu tiên
        # Nếu có ít nhất 1 xe rẽ trái HOẶC thời gian chờ cao
        if ((demand['vehicles'] >= LEFT_TURN_MIN_VEHICLES) or 
            (demand['max_waiting'] >= LEFT_TURN_MAX_WAIT * 0.75)) and demand['priority_score'] > best_score:
            best_direction = direction
            best_score = demand['priority_score']
    
    # Log thông tin cho debug
    if best_direction:
        demand = left_turn_demand[best_direction]
        print(f"🚨 Phát hiện nhu cầu rẽ trái ở hướng {best_direction}: {demand['vehicles']} xe, chờ {demand['max_waiting']:.1f}s, điểm {best_score:.1f}")
    
    return best_direction is not None, best_direction

def get_left_turn_phase(direction):
    """Lấy pha rẽ trái cho hướng cụ thể - ĐÃ CẢI TIẾN"""
    # Mapping pha rẽ trái
    left_turn_phases = {
        'North': 4,  # Pha rẽ trái từ Bắc
        'South': 4,  # Pha rẽ trái từ Nam  
        'East': 6,   # Pha rẽ trái từ Đông
        'West': 6    # Pha rẽ trái từ Tây
    }
    
    # Log để debug
    print(f"🔄 Chọn pha rẽ trái {left_turn_phases.get(direction, 4)} cho hướng {direction}")
    return left_turn_phases.get(direction, 4)

def can_stop_safely(speed, distance):
    """Kiểm tra an toàn dừng xe"""
    if speed < 0.5:
        return True
    
    if speed > SPEED_THRESHOLD_HIGH:
        t_reaction = 2.0
    elif speed > SPEED_THRESHOLD_MEDIUM:
        t_reaction = 1.5
    elif speed > 5:
        t_reaction = 1.2
    else:
        t_reaction = 1.0
    
    min_stopping_distance = (speed ** 2) / (2 * MAX_DECELERATION) + speed * t_reaction
    
    if speed > SPEED_THRESHOLD_HIGH:
        safety_buffer = 2.0
    elif speed > SPEED_THRESHOLD_MEDIUM:
        safety_buffer = 1.7
    elif speed > 5:
        safety_buffer = 1.5
    else:
        safety_buffer = 1.3
    
    min_stopping_distance *= safety_buffer
    
    stopping_time = speed / MAX_DECELERATION + t_reaction
    time_to_intersection = distance / max(speed, 0.1)
    
    if speed > SPEED_THRESHOLD_HIGH and distance < AGGRESSIVE_DETECTION_DISTANCE:
        return False
    
    decision_zone = 50 * DECISION_ZONE_MULTIPLIER
    if speed > SPEED_THRESHOLD_MEDIUM and distance < decision_zone:
        return distance > min_stopping_distance * 1.2
    
    return distance > min_stopping_distance and time_to_intersection > stopping_time * 1.3

def is_safe_to_change_phase(direction_detectors, junction_id='E3'):
    """Kiểm tra an toàn chuyển pha"""
    global last_phase_change_step
    current_step = traci.simulation.getTime() * 10
    
    if current_step - last_phase_change_step < MIN_PHASE_GAP:
        return False
    
    for detector_id in direction_detectors:
        vehicles = traci.lanearea.getLastStepVehicleIDs(detector_id)
        for vehicle in vehicles:
            try:
                speed = traci.vehicle.getSpeed(vehicle)
                
                if speed > SAFETY_SPEED_BUFFER:
                    distance = traci.vehicle.getLanePosition(vehicle)
                    lane_length = traci.lane.getLength(traci.vehicle.getLaneID(vehicle))
                    remaining_distance = lane_length - distance
                    
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
                        return False
                    
                    if not can_stop_safely(speed, remaining_distance):
                        return False
                        
            except Exception as e:
                continue
    
    vehicles_in_junction = get_vehicles_in_junction(junction_id)
    if vehicles_in_junction:
        return False
    
    last_phase_change_step = current_step
    return True

def get_vehicles_in_junction(junction_id):
    """Lấy xe trong giao lộ"""
    junction_vehicles = []
    all_vehicles = traci.vehicle.getIDList()
    
    for veh_id in all_vehicles:
        try:
            road_id = traci.vehicle.getRoadID(veh_id)
            
            if road_id.startswith(':'):
                junction_vehicles.append(veh_id)
                continue
            
            lane_id = traci.vehicle.getLaneID(veh_id)
            lane_length = traci.lane.getLength(lane_id)
            position = traci.vehicle.getLanePosition(veh_id)
            speed = traci.vehicle.getSpeed(veh_id)
            
            if speed > SPEED_THRESHOLD_HIGH:
                detection_distance = JUNCTION_CLEARING_DISTANCE + 25
            elif speed > SPEED_THRESHOLD_MEDIUM:
                detection_distance = JUNCTION_CLEARING_DISTANCE + 15
            else:
                detection_distance = JUNCTION_CLEARING_DISTANCE
            
            estimated_time_to_junction = (lane_length - position) / max(speed, 1.0)
            
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
    """Đếm sự kiện phanh khẩn cấp"""
    count = 0
    for veh_id in traci.vehicle.getIDList():
        try:
            acceleration = traci.vehicle.getAcceleration(veh_id)
            speed = traci.vehicle.getSpeed(veh_id)
            
            if speed > 10:
                emergency_threshold = -3.5
            elif speed > 5:
                emergency_threshold = -4.0
            else:
                emergency_threshold = -4.5
            
            if acceleration < emergency_threshold:
                count += 1
        except:
            continue
    return count

def wait_for_junction_clearing(junction_id='E3', max_wait=8):
    """Chờ giao lộ trống hoàn toàn"""
    wait_steps = 0
    cleared = False
    consecutive_clear_steps = 0
    required_clear_steps = 15
    
    while wait_steps < max_wait * 10:
        traci.simulationStep()
        vehicles_in_junction = get_vehicles_in_junction(junction_id)
        
        if not vehicles_in_junction:
            consecutive_clear_steps += 1
            if consecutive_clear_steps >= required_clear_steps:
                cleared = True
                break
        else:
            consecutive_clear_steps = 0
        
        wait_steps += 1
    
    return wait_steps

def plot_traffic_status(status_data, threshold):
    """Vẽ biểu đồ trạng thái giao thông"""
    plt.figure(figsize=(15, 10))
    
    for direction, color in zip(['North', 'South', 'East', 'West'], ['blue', 'green', 'red', 'orange']):
        label = {'North': 'Hướng Bắc', 'South': 'Hướng Nam', 
                 'East': 'Hướng Đông', 'West': 'Hướng Tây'}[direction]
        plt.plot(status_data['time'], status_data[direction], label=label, color=color, linewidth=2)
    
    plt.axhline(y=threshold, color='r', linestyle='--', linewidth=2, 
                label=f'Ngưỡng ({threshold})')
    
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
    plt.title('Hệ Thống Điều Khiển Đèn Giao Thông Tối Ưu (Cải Tiến Cho Rẽ Trái)', fontsize=16)
    plt.legend(fontsize=12)
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('he_thong_dieu_khien_re_trai_toi_uu.png', dpi=150)
    print("Đã lưu biểu đồ dưới dạng 'he_thong_dieu_khien_re_trai_toi_uu.png'")
    plt.show()

def process_left_turn_vehicles(direction, left_turn_vehicles, max_process=MAX_LEFT_TURN_VEHICLES_PER_CYCLE):
    """CHỨC NĂNG MỚI: Xử lý xe rẽ trái để đảm bảo nhiều xe di chuyển cùng lúc"""
    print(f"👉 Xử lý {len(left_turn_vehicles)} xe rẽ trái cho hướng {direction}")
    
    # Sắp xếp xe theo thời gian chờ, ưu tiên xe chờ lâu nhất
    vehicles_with_wait_time = []
    for veh_id in left_turn_vehicles:
        try:
            wait_time = traci.vehicle.getWaitingTime(veh_id)
            vehicles_with_wait_time.append((veh_id, wait_time))
        except:
            continue
    
    # Sắp xếp giảm dần theo thời gian chờ
    vehicles_with_wait_time.sort(key=lambda x: x[1], reverse=True)
    
    # Xử lý đến max_process xe
    count = 0
    for veh_id, wait_time in vehicles_with_wait_time[:max_process]:
        try:
            # Tăng ưu tiên cho xe này để đảm bảo nó khởi hành
            traci.vehicle.setSpeedFactor(veh_id, 1.2)  # Tăng 20% tốc độ tối đa
            traci.vehicle.setSpeed(veh_id, -1)  # Đặt lại tốc độ để kích hoạt lại xe
            
            # Thêm: Đánh dấu xe để không bị dừng bởi các xe khác
            traci.vehicle.setParameter(veh_id, "junctionModel.ignoreIDs", "all")
            traci.vehicle.setParameter(veh_id, "junctionModel.ignoreTypes", "all")
            
            # Thêm: Giảm thời gian phản ứng
            traci.vehicle.setParameter(veh_id, "junctionModel.sigma", "0.1")  # Giảm độ không chắc chắn
            
            # Tăng gia tốc tối đa để xe di chuyển nhanh hơn
            default_accel = traci.vehicle.getAccel(veh_id)
            traci.vehicle.setAccel(veh_id, default_accel * 1.3)  # Tăng 30% gia tốc
            
            # Đặt xe vào chế độ ưu tiên tại giao lộ
            try:
                traci.vehicle.setParameter(veh_id, "has.priority", "true")
                traci.vehicle.setParameter(veh_id, "junctionModel.impatience", "1.0")
            except:
                pass
            
            count += 1
            print(f"   ✅ Xử lý xe {veh_id} (đã chờ {wait_time:.1f}s)")
        except Exception as e:
            print(f"   ❌ Lỗi xử lý xe {veh_id}: {str(e)}")
    
    return count

def run_simulation():
    """Chạy mô phỏng với cải tiến rẽ trái"""
    # Xác định nhóm bộ dò theo hướng
    detector_groups = {
        'North': ['E1-3-1', 'E1-3-2'],
        'East': ['E3-4-1', 'E3-4-2'],
        'South': ['E5-3-1', 'E5-3-2'],
        'West': ['E3-2-1', 'E3-2-2']
    }
    
    # Các pha của đèn giao thông (mở rộng với pha rẽ trái)
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
    left_turn_phase_active = False
    left_turn_phase_duration = 0
    last_left_turn_direction = None
    active_left_turn_vehicles = []
    processed_left_turn_count = 0
    
    # Dữ liệu cho trực quan hóa
    status_data = {'time': [], 'North': [], 'South': [], 'East': [], 'West': []}
    
    # Vòng lặp mô phỏng chính
    step = 0
    in_transition = False
    transition_stage = 0
    next_green_phase = None
    emergency_braking_events = 0
    data_collection_interval = 50
    left_turn_activations = 0
    
    print("Bắt đầu mô phỏng với hệ thống điều khiển tối ưu cho rẽ trái...")
    print(f"Cấu hình: GREEN({MIN_GREEN_TIME}-{MAX_GREEN_TIME}s), LEFT_TURN({LEFT_TURN_PHASE_DURATION}s)")
    
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
            # Đánh dấu ưu tiên rẽ trái cho hướng hiện tại
            is_left_priority = left_turn_phase_active and direction == last_left_turn_direction
            status, components = calculate_status(direction_metrics, is_left_priority)
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
        if left_turn_phase_active:
            left_turn_phase_duration += 1
        
        # Xử lý thời gian làm mát
        if cooldown_timer > 0:
            cooldown_timer -= 1
            step += 1
            continue
        
        # PHẦN CẢI TIẾN: Xử lý pha rẽ trái đang hoạt động
        if left_turn_phase_active:
            # Giai đoạn đầu tiên: Xử lý và kích hoạt các xe rẽ trái
            if left_turn_phase_duration == 1:
                # Thu thập tất cả xe rẽ trái trong hướng hiện tại
                all_left_turn_vehicles = []
                
                for detector in detector_groups[last_left_turn_direction]:
                    metrics = get_lane_metrics(detector)
                    all_left_turn_vehicles.extend(metrics.get('left_turn_vehicle_ids', []))
                
                # Lưu danh sách các xe đang xử lý
                active_left_turn_vehicles = all_left_turn_vehicles
                
                # Kích hoạt xử lý xe rẽ trái - ĐIỂM QUAN TRỌNG
                processed_left_turn_count = process_left_turn_vehicles(
                    last_left_turn_direction, 
                    active_left_turn_vehicles
                )
                
                # Nếu không có xe nào được xử lý, kết thúc pha sớm
                if processed_left_turn_count == 0:
                    left_turn_phase_active = False
                    left_turn_phase_duration = 0
                    traci.trafficlight.setPhase('E3', current_phase)
                    cooldown_timer = COOLDOWN_PERIOD // 4
                    print(f"🔄 Thời điểm {step/10:.1f}s: Không phát hiện xe rẽ trái, kết thúc pha sớm")
            
            # Mỗi 5 giây, kiểm tra còn xe nào đang chờ không
            elif left_turn_phase_duration % 50 == 0 and left_turn_phase_duration < LEFT_TURN_PHASE_DURATION:
                # Kiểm tra các xe đang xử lý
                still_waiting_count = 0
                for veh_id in active_left_turn_vehicles:
                    try:
                        if traci.vehicle.getWaitingTime(veh_id) > 0.5:
                            still_waiting_count += 1
                    except:
                        # Xe có thể đã ra khỏi mô phỏng
                        pass
                
                # Thu thập thêm xe mới (nếu có)
                new_left_turn_vehicles = []
                for detector in detector_groups[last_left_turn_direction]:
                    metrics = get_lane_metrics(detector)
                    for veh_id in metrics.get('left_turn_vehicle_ids', []):
                        if veh_id not in active_left_turn_vehicles:
                            new_left_turn_vehicles.append(veh_id)
                
                # Nếu không còn xe nào đang chờ và không có xe mới
                if still_waiting_count == 0 and not new_left_turn_vehicles:
                    left_turn_phase_duration = LEFT_TURN_PHASE_DURATION  # Kết thúc pha sớm
                    print(f"🔄 Thời điểm {step/10:.1f}s: Tất cả xe rẽ trái đã di chuyển, kết thúc pha sớm")
                elif new_left_turn_vehicles:
                    # Xử lý thêm xe mới
                    new_processed = process_left_turn_vehicles(last_left_turn_direction, new_left_turn_vehicles)
                    active_left_turn_vehicles.extend(new_left_turn_vehicles)
                    processed_left_turn_count += new_processed
                    print(f"🔄 Thời điểm {step/10:.1f}s: Phát hiện và xử lý thêm {new_processed} xe rẽ trái mới")
            
            # Kết thúc pha rẽ trái sau khi đã hoàn thành thời gian
            if left_turn_phase_duration >= LEFT_TURN_PHASE_DURATION:
                # Kiểm tra có cần kéo dài thêm không
                need_extension = False
                
                # Nếu có nhiều hơn 3 xe và đã xử lý ít nhất 2 xe
                if len(active_left_turn_vehicles) > 3 and processed_left_turn_count >= 2:
                    # Kiểm tra còn xe nào vẫn đang di chuyển trong giao lộ
                    vehicles_in_junction = get_vehicles_in_junction('E3')
                    left_turn_vehicles_in_junction = [veh for veh in vehicles_in_junction if veh in active_left_turn_vehicles]
                    
                    if left_turn_vehicles_in_junction:
                        need_extension = True
                        left_turn_phase_duration -= LEFT_TURN_PHASE_EXTENSION  # Kéo dài thêm
                        print(f"🕒 Thời điểm {step/10:.1f}s: Kéo dài pha rẽ trái thêm {LEFT_TURN_PHASE_EXTENSION/10}s để các xe hoàn tất")
                
                # Kết thúc pha nếu không cần kéo dài
                if not need_extension:
                    left_turn_phase_active = False
                    left_turn_phase_duration = 0
                    traci.trafficlight.setPhase('E3', current_phase)
                    cooldown_timer = COOLDOWN_PERIOD // 2  # Cooldown ngắn hơn
                    print(f"🔄 Thời điểm {step/10:.1f}s: Kết thúc pha rẽ trái, quay về pha {current_phase}")
            step += 1
            continue
        
        # Xử lý các pha chuyển tiếp
        if in_transition:
            if transition_stage == 1 and phase_duration >= YELLOW_TIME:
                transition_stage = 2
                phase_duration = 0
                print(f"🔴 Thời điểm {step/10:.1f}s: Giai đoạn đèn đỏ toàn phần")
                
            elif transition_stage == 2 and phase_duration >= CLEARANCE_TIME:
                print(f"⏳ Thời điểm {step/10:.1f}s: Đang chờ giao lộ trống...")
                wait_steps = wait_for_junction_clearing()
                
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
        
        # Kiểm tra nhu cầu pha rẽ trái
        needs_left_turn, left_turn_direction = needs_left_turn_phase(detector_groups, all_metrics)
        
        if needs_left_turn and not left_turn_phase_active and phase_duration >= MIN_GREEN_TIME // 2:
            # Kích hoạt pha rẽ trái
            left_turn_phase = get_left_turn_phase(left_turn_direction)
            current_detectors = []
            for direction in [left_turn_direction]:
                current_detectors.extend(detector_groups[direction])
            
            if is_safe_to_change_phase(current_detectors):
                left_turn_phase_active = True
                left_turn_phase_duration = 0
                last_left_turn_direction = left_turn_direction
                left_turn_activations += 1
                traci.trafficlight.setPhase('E3', left_turn_phase)
                print(f"↩️  Thời điểm {step/10:.1f}s: Kích hoạt pha rẽ trái cho hướng {left_turn_direction} (pha {left_turn_phase})")
                step += 1
                continue
        
        # Đánh giá pha định kỳ (logic gốc với điều chỉnh)
        if phase_duration % EVAL_INTERVAL == 0:
            current_directions = [dir for dir, phase in direction_to_phase.items() if phase == current_phase]
            opposing_directions = [dir for dir, phase in direction_to_phase.items() if phase != current_phase]
            
            current_detectors = []
            for direction in current_directions:
                current_detectors.extend(detector_groups[direction])
            
            # Logic chuyển đổi pha với điều chỉnh cho rẽ trái
            current_good = any(all_metrics[dir]['is_good'] for dir in current_directions)
            opposing_need = any(all_metrics[dir]['status'] > THRESHOLD * 1.2 for dir in opposing_directions)
            
            should_change = (not current_good or opposing_need) and phase_duration >= MIN_GREEN_TIME
            
            if should_change:
                if is_safe_to_change_phase(current_detectors):
                    in_transition = True
                    transition_stage = 1
                    if current_phase == 0:
                        traci.trafficlight.setPhase('E3', 1)
                        next_green_phase = 2
                        print(f"🟡 Thời điểm {step/10:.1f}s: Đèn vàng N-S")
                    else:
                        traci.trafficlight.setPhase('E3', 3)
                        next_green_phase = 0
                        print(f"🟡 Thời điểm {step/10:.1f}s: Đèn vàng E-W")
                    phase_duration = 0
                else:
                    print(f"⏸️  Thời điểm {step/10:.1f}s: Hoãn chuyển pha - không an toàn")
            
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
                    phase_duration = MAX_GREEN_TIME - 20
                    print(f"⏰ Thời điểm {step/10:.1f}s: Kéo dài pha xanh thêm - chưa an toàn")
        
        # In trạng thái định kỳ
        if step % 400 == 0:
            print(f"\n📊 Thời gian: {step / 10} giây")
            for direction, data in all_metrics.items():
                direction_name = {'North': 'Bắc', 'South': 'Nam', 'East': 'Đông', 'West': 'Tây'}[direction]
                status = "TỐT ✅" if data['is_good'] else "XẤU ❌"
                
                # Đếm xe rẽ trái trong hướng này
                left_turn_count = sum(m['left_turn_vehicles'] for m in data['metrics'])
                left_turn_info = f" (Rẽ trái: {left_turn_count})" if left_turn_count > 0 else ""
                
                print(f"   Hướng {direction_name}: {data['status']:.2f} ({status}){left_turn_info}")
            print(f"   Phanh khẩn cấp: {emergency_braking_events}")
            print(f"   Pha rẽ trái đã kích hoạt: {left_turn_activations} lần")
            print("-" * 60)
        
        step += 1
    
    # Kết thúc mô phỏng
    print("\n🏁 Mô phỏng hoàn thành!")
    
    # Thống kê tổng hợp
    print("\n📈 Thống kê tổng hợp (Có cải tiến rẽ trái):")
    avg_waiting_times = {}
    avg_queue_lengths = {}
    total_left_turn_vehicles = 0
    
    for direction, history in metrics_history.items():
        avg_waiting_times[direction] = np.mean([np.mean([m['waiting_time'] for m in h['metrics']]) 
                                               for h in history])
        avg_queue_lengths[direction] = np.mean([np.mean([m['queue_length'] for m in h['metrics']]) 
                                              for h in history])
        
        # Thống kê rẽ trái
        total_left_turn_vehicles += sum([sum([m['left_turn_vehicles'] for m in h['metrics']]) 
                                        for h in history])
    
    total_avg_wait = np.mean(list(avg_waiting_times.values()))
    total_avg_queue = np.mean(list(avg_queue_lengths.values()))
    
    print(f"⏱️  Thời gian chờ trung bình: {total_avg_wait:.2f} giây")
    print(f"🚗 Độ dài hàng đợi trung bình: {total_avg_queue:.2f} xe")
    print(f"⚠️  Tổng sự kiện phanh khẩn cấp: {emergency_braking_events}")
    print(f"↩️  Tổng pha rẽ trái được kích hoạt: {left_turn_activations} lần")
    print(f"🚙 Tổng xe rẽ trái được phục vụ: {total_left_turn_vehicles} xe")
    
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