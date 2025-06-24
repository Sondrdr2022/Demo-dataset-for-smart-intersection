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

# Cấu hình - CẢI TIẾN TỐI ƯU ĐỂ GIẢM THIỂU PHANH KHẨN CẤP
MIN_GREEN_TIME = 40  # Tăng từ 35 lên 40 giây để giảm tần suất chuyển đổi đèn
MAX_GREEN_TIME = 100  # Tăng từ 95 lên 100 giây
EVAL_INTERVAL = 20   # Tăng từ 15 lên 20 giây để giảm tần suất đánh giá hơn nữa
YELLOW_TIME = 10     # Tăng từ 7 lên 10 giây để cho xe có nhiều thời gian phản ứng hơn rất nhiều
CLEARANCE_TIME = 5   # Tăng từ 3 lên 5 giây để đảm bảo giao lộ hoàn toàn trống
COOLDOWN_PERIOD = 20 # Tăng từ 15 lên 20 giây
THRESHOLD = 0.7      # Giữ ngưỡng trạng thái "TỐT" không đổi
JUNCTION_CLEARING_DISTANCE = 60  # Tăng từ 40 lên 50 mét để mở rộng vùng an toàn hơn nữa
SAFETY_SPEED_BUFFER = 2          # Giảm từ 3 xuống 2 để phát hiện thêm nhiều xe hơn
YELLOW_TIME_BUFFER = 3           # Thời gian đệm bổ sung cho việc tính toán an toàn đèn vàng
MAX_DECELERATION = 3.0           # Giảm tốc tối đa an toàn (m/s²) để tránh phanh khẩn cấp

def start_sumo():
    """Khởi động SUMO"""
    sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
    sumoCmd = [sumoBinary, '-c', r"C:\Users\Admin\Downloads\sumo test\New folder\dataset.sumocfg", '--step-length', '0.1']
    traci.start(sumoCmd)

def get_lane_metrics(detector_id):
    """Lấy các chỉ số cho làn đường cụ thể"""
    metrics = {}
    
    # Độ dài hàng đợi (số xe)
    metrics['queue_length'] = traci.lanearea.getJamLengthVehicle(detector_id)
    
    # Thời gian chờ (thời gian chờ tối đa của các xe trên bộ dò)
    vehicles = traci.lanearea.getLastStepVehicleIDs(detector_id)
    wait_times = [traci.vehicle.getWaitingTime(veh) for veh in vehicles] if vehicles else [0]
    metrics['waiting_time'] = max(wait_times) if wait_times else 0
    
    # Mật độ làn đường (xe trên chiều dài làn)
    occupancy = traci.lanearea.getLastStepOccupancy(detector_id) / 100.0  # Chuyển đổi từ phần trăm
    metrics['density'] = occupancy
    
    # Tốc độ trung bình
    metrics['avg_speed'] = traci.lanearea.getLastStepMeanSpeed(detector_id)
    
    # Lưu lượng xe (xe/giờ)
    metrics['flow_rate'] = traci.lanearea.getLastStepVehicleNumber(detector_id) * 3600  # xe/giờ
    
    return metrics

def calculate_status(metrics_list):
    """Tính toán trạng thái dựa trên nhiều chỉ số từ các làn đường khác nhau"""
    # Trọng số cho các tham số khác nhau
    weights = {
        'queue_length': 0.3,
        'waiting_time': 0.4,
        'density': 0.2,
        'avg_speed': 0.0,  # Không sử dụng trực tiếp trong tính toán
        'flow_rate': 0.1
    }
    
    # Chuẩn hóa và kết hợp các chỉ số
    status_components = []
    
    for metrics in metrics_list:
        # Tính điểm cho mỗi làn đường
        lane_score = (
            weights['queue_length'] * min(metrics['queue_length'] / 10, 1) +
            weights['waiting_time'] * min(metrics['waiting_time'] / 60, 1) +
            weights['density'] * metrics['density'] +
            weights['flow_rate'] * (metrics['flow_rate'] / 1800)  # Chuẩn hóa theo lưu lượng tối đa
        )
        status_components.append(lane_score)
    
    # Tính trạng thái tổng hợp
    total_status = sum(status_components)
    max_component = max(status_components) if status_components else 0
    
    # Đưa trọng số cao hơn cho thành phần tối đa
    adjusted_status = 0.6 * total_status + 0.4 * max_component
    
    return adjusted_status, status_components

def can_stop_safely(speed, distance):
    """Kiểm tra xem một xe có thể dừng an toàn không, tránh phanh khẩn cấp"""
    # Dựa trên công thức khoảng cách dừng: d = v²/(2*a) + v*t_reaction
    # t_reaction = 1.0 giây (thời gian phản ứng người lái)
    t_reaction = 1.0
    # Tính toán khoảng cách dừng tối thiểu cần thiết
    min_stopping_distance = (speed**2)/(2*MAX_DECELERATION) + speed*t_reaction
    
    # Thêm 20% khoảng cách đệm an toàn
    min_stopping_distance *= 1.2
    
    return distance > min_stopping_distance

def is_safe_to_change_phase(direction_detectors, junction_id='E3'):
    """Kiểm tra an toàn để đảm bảo xe có thể đi qua giao lộ - Cải tiến tối ưu"""
    # Kiểm tra các xe đang tiến gần đến giao lộ
    for detector_id in direction_detectors:
        vehicles = traci.lanearea.getLastStepVehicleIDs(detector_id)
        for vehicle in vehicles:
            try:
                # Kiểm tra xe trong vùng nguy hiểm (đang tiến gần giao lộ)
                speed = traci.vehicle.getSpeed(vehicle)
                if speed > SAFETY_SPEED_BUFFER:  # Phát hiện cả xe di chuyển chậm
                    distance = traci.vehicle.getLanePosition(vehicle)
                    lane_length = traci.lane.getLength(traci.vehicle.getLaneID(vehicle))
                    remaining_distance = lane_length - distance
                    
                    # Nếu xe quá gần giao lộ và đang di chuyển
                    if remaining_distance < 35 and speed > 6:  # Tăng khoảng cách an toàn, giảm ngưỡng tốc độ
                        return False
                    
                    # Nếu xe không thể dừng an toàn với mức giảm tốc bình thường
                    if not can_stop_safely(speed, remaining_distance):
                        return False
                    
                    # Tính toán thời gian cần thiết để đi qua giao lộ với lượng đệm lớn hơn
                    # Thêm 30% thời gian đệm để đảm bảo an toàn
                    time_to_intersection = (remaining_distance / max(speed, 1.0)) * 1.3
                    
                    # Nếu xe sẽ đến giao lộ trong thời gian đèn vàng + thời gian đệm bổ sung
                    if 0 < time_to_intersection < YELLOW_TIME + YELLOW_TIME_BUFFER:
                        return False
                        
                    # Nếu xe đang di chuyển nhanh và gần đến giao lộ (vùng quyết định)
                    if speed > 12 and remaining_distance < 60:  # Xe nhanh cần khoảng cách lớn hơn
                        return False
            except:
                # Bỏ qua nếu xe biến mất hoặc có lỗi khác
                continue
    
    # Kiểm tra xe hiện đang ở trong giao lộ
    vehicles_in_junction = get_vehicles_in_junction(junction_id)
    if vehicles_in_junction:
        return False
        
    return True

def get_vehicles_in_junction(junction_id):
    """Lấy tất cả các xe hiện đang ở trong hoặc gần giao lộ - Mở rộng vùng kiểm tra"""
    junction_vehicles = []
    
    # Lấy tất cả các xe trong mô phỏng
    all_vehicles = traci.vehicle.getIDList()
    
    for veh_id in all_vehicles:
        try:
            # Lấy vị trí xe và ID đường
            road_id = traci.vehicle.getRoadID(veh_id)
            
            # Kiểm tra xem xe có đang ở trên làn đường nội bộ (bên trong giao lộ)
            if road_id.startswith(':'):
                junction_vehicles.append(veh_id)
                continue
            
            # Kiểm tra xem xe có rất gần giao lộ
            lane_id = traci.vehicle.getLaneID(veh_id)
            lane_length = traci.lane.getLength(lane_id)
            position = traci.vehicle.getLanePosition(veh_id)
            speed = traci.vehicle.getSpeed(veh_id)
            
            # Kiểm tra dựa trên một tổ hợp các yếu tố:
            # 1. Vị trí xe so với cuối làn đường
            # 2. Thời gian ước tính đến giao lộ
            # 3. Tốc độ hiện tại
            estimated_time_to_junction = (lane_length - position) / max(speed, 1.0)
            
            if position > lane_length - JUNCTION_CLEARING_DISTANCE or (speed > 5 and estimated_time_to_junction < 4):
                next_links = traci.lane.getLinks(lane_id)
                for link in next_links:
                    if junction_id in link[0]:  # Nếu làn đường tiếp theo dẫn đến giao lộ của chúng ta
                        junction_vehicles.append(veh_id)
                        break
        except:
            # Bỏ qua nếu xe biến mất hoặc lỗi khác
            continue
            
    return junction_vehicles

def count_emergency_braking_events():
    """Đếm sự kiện phanh khẩn cấp trong bước hiện tại"""
    count = 0
    for veh_id in traci.vehicle.getIDList():
        try:
            # Kiểm tra xe đang phanh khẩn cấp (giảm tốc > 4.5 m/s^2)
            if traci.vehicle.getAcceleration(veh_id) < -4.5:
                count += 1
        except:
            continue
    return count

def wait_for_junction_clearing(junction_id='E3', max_wait=7):  # Tăng từ 5 lên 7 giây
    """Chờ cho xe đi qua hết giao lộ"""
    wait_steps = 0
    cleared = False
    while wait_steps < max_wait * 10:  # Chuyển đổi sang bước mô phỏng (0.1s mỗi bước)
        traci.simulationStep()
        if not get_vehicles_in_junction(junction_id):
            cleared = True
            break
        wait_steps += 1
    
    # Nếu sau thời gian chờ tối đa mà vẫn chưa trống, vẫn tiếp tục
    if not cleared:
        print("Cảnh báo: Giao lộ không thể trống hoàn toàn sau thời gian chờ tối đa!")
    
    return wait_steps

def plot_traffic_status(status_data, threshold):
    """Vẽ biểu đồ trạng thái giao thông so với ngưỡng"""
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
        if status_data[d]:  # Kiểm tra danh sách không rỗng
            max_status = max(max_status, max(status_data[d]))
    
    plt.fill_between(status_data['time'], threshold, max_status * 1.1, 
                    color='green', alpha=0.2, label='Vùng Trạng Thái TỐT')
    plt.fill_between(status_data['time'], 0, threshold,
                    color='red', alpha=0.2, label='Vùng Trạng Thái XẤU')
    
    plt.xlabel('Thời Gian (giây)', fontsize=12)
    plt.ylabel('Giá Trị Trạng Thái', fontsize=12)
    plt.title('Trạng Thái Giao Thông Theo Hướng So Với Ngưỡng (Đèn Vàng Kéo Dài)', fontsize=16)
    plt.legend(fontsize=12)
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('trang_thai_giao_thong_den_vang_keodai.png', dpi=150)
    print("Đã lưu biểu đồ dưới dạng 'trang_thai_giao_thong_den_vang_keodai.png'")
    plt.show()

def run_simulation():
    """Chạy mô phỏng với cải tiến để giảm phanh khẩn cấp"""
    # Xác định nhóm bộ dò theo hướng
    detector_groups = {
        'North': ['E1-3-1', 'E1-3-2'],  # Hướng Bắc
        'East': ['E3-4-1', 'E3-4-2'],   # Hướng Đông
        'South': ['E5-3-1', 'E5-3-2'],  # Hướng Nam
        'West': ['E3-2-1', 'E3-2-2']    # Hướng Tây
    }
    
    # Các pha của đèn giao thông - ĐIỀU CHỈNH CHO PHA MẶC ĐỊNH 0-3
    # Pha 0: N-S xanh, E-W đỏ
    # Pha 1: N-S vàng, E-W đỏ
    # Pha 2: N-S đỏ, E-W xanh
    # Pha 3: N-S đỏ, E-W vàng
    direction_to_phase = {
        'North': 0, 
        'South': 0,
        'East': 2, 
        'West': 2
    }
    
    # Theo dõi số liệu theo thời gian
    metrics_history = defaultdict(list)
    current_phase = 0
    phase_duration = 0
    cooldown_timer = 0
    
    # Dữ liệu cho trực quan hóa
    status_data = {'time': [], 'North': [], 'South': [], 'East': [], 'West': []}
    
    # Vòng lặp mô phỏng chính
    step = 0
    in_transition = False
    next_green_phase = None
    emergency_braking_events = 0
    data_collection_interval = 50  # Thu thập dữ liệu mỗi 5 giây (50 * 0.1s bước)
    
    # Theo dõi giai đoạn chuyển đổi đèn giao thông
    transition_stage = 0  # 0: normal, 1: yellow, 2: all-red
    
    print("Bắt đầu mô phỏng với đèn vàng kéo dài để giảm thiểu phanh khẩn cấp...")
    while step < 5000:  
        traci.simulationStep()
        
        # Kiểm tra sự kiện phanh khẩn cấp trong bước này
        if step % 10 == 0:  # Kiểm tra định kỳ để tiết kiệm xử lý
            new_events = count_emergency_braking_events()
            if new_events > 0:
                print(f"Thời điểm {step/10:.1f}s: Phát hiện {new_events} sự kiện phanh khẩn cấp mới")
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
            
            # Lưu lịch sử số liệu
            metrics_history[direction].append(all_metrics[direction])
        
        # Thu thập dữ liệu cho trực quan hóa theo định kỳ
        if step % data_collection_interval == 0:
            # Lưu dữ liệu trạng thái
            time_sec = step / 10.0  # Chuyển đổi sang giây
            status_data['time'].append(time_sec)
            for direction, data in all_metrics.items():
                status_data[direction].append(data['status'])
        
        # Tăng bộ đếm thời gian pha
        phase_duration += 1
        
        # Xử lý thời gian làm mát sau khi thay đổi pha
        if cooldown_timer > 0:
            cooldown_timer -= 1
            step += 1
            continue
        
        # Xử lý các pha chuyển tiếp (đèn vàng và đèn đỏ toàn phần)
        if in_transition:
            if transition_stage == 1 and phase_duration >= YELLOW_TIME:
                # Sau đèn vàng, chuyển sang giai đoạn đèn đỏ toàn phần
                transition_stage = 2
                phase_duration = 0
                print(f"Thời điểm {step/10:.1f}s: Chuyển sang giai đoạn đèn đỏ toàn phần")
                # Trong thực tế, bạn sẽ đặt tất cả các đèn thành đỏ ở đây
                # nhưng vì giới hạn của program hiện tại, chúng ta sẽ mô phỏng bằng cách đợi
                
            elif transition_stage == 2 and phase_duration >= CLEARANCE_TIME:
                # Sau giai đoạn đèn đỏ toàn phần, đợi xe đi qua hết giao lộ
                print(f"Thời điểm {step/10:.1f}s: Đợi xe đi qua hết giao lộ")
                wait_steps = wait_for_junction_clearing()
                
                # Đặt pha xanh tiếp theo
                in_transition = False
                transition_stage = 0
                current_phase = next_green_phase
                traci.trafficlight.setPhase('E3', current_phase)
                phase_duration = 0
                cooldown_timer = COOLDOWN_PERIOD  # Thêm thời gian làm mát sau khi thay đổi pha
                print(f"Thời điểm {(step+wait_steps)/10:.1f}s: Chuyển sang pha xanh mới (pha {current_phase})")
                
                # Cập nhật bộ đếm bước để tính đến wait_for_junction_clearing
                step += wait_steps
            step += 1
            continue
        
        # Đánh giá pha thường xuyên (mỗi EVAL_INTERVAL giây)
        if phase_duration % EVAL_INTERVAL == 0:
            current_directions = [dir for dir, phase in direction_to_phase.items() if phase == current_phase]
            opposing_directions = [dir for dir, phase in direction_to_phase.items() if phase != current_phase]
            
            # Lấy tất cả các bộ dò cho hướng hiện tại
            current_detectors = []
            for direction in current_directions:
                current_detectors.extend(detector_groups[direction])
            
            # Kiểm tra xem pha hiện tại có còn tốt hay đã đạt thời gian tối thiểu
            current_good = any(all_metrics[dir]['is_good'] for dir in current_directions)
            opposing_need = any(all_metrics[dir]['status'] > THRESHOLD * 1.2 for dir in opposing_directions)
            
            # Logic cho việc chuyển đổi pha với kiểm tra an toàn
            if (not current_good or opposing_need) and phase_duration >= MIN_GREEN_TIME:
                # Kiểm tra an toàn trước khi chuyển đổi pha
                if is_safe_to_change_phase(current_detectors):
                    # Bắt đầu chuyển sang pha vàng
                    in_transition = True
                    transition_stage = 1  # Bắt đầu với đèn vàng
                    if current_phase == 0:
                        traci.trafficlight.setPhase('E3', 1)  # N-S vàng
                        next_green_phase = 2  # Mục tiêu sẽ là E-W xanh
                        print(f"Thời điểm {step/10:.1f}s: Bắt đầu đèn vàng N-S (trạng thái không tốt)")
                    else:
                        traci.trafficlight.setPhase('E3', 3)  # E-W vàng
                        next_green_phase = 0  # Mục tiêu sẽ là N-S xanh
                        print(f"Thời điểm {step/10:.1f}s: Bắt đầu đèn vàng E-W (trạng thái không tốt)")
                    phase_duration = 0
                
            # Bắt buộc chuyển đổi nếu đã đạt thời gian xanh tối đa
            elif phase_duration >= MAX_GREEN_TIME:
                # Ngay cả ở thời gian tối đa, vẫn thực hiện kiểm tra an toàn
                if is_safe_to_change_phase(current_detectors):
                    # Bắt đầu chuyển sang pha vàng
                    in_transition = True
                    transition_stage = 1  # Bắt đầu với đèn vàng
                    if current_phase == 0:
                        traci.trafficlight.setPhase('E3', 1)  # N-S vàng
                        next_green_phase = 2  # Mục tiêu sẽ là E-W xanh
                        print(f"Thời điểm {step/10:.1f}s: Bắt đầu đèn vàng N-S (đạt thời gian tối đa)")
                    else:
                        traci.trafficlight.setPhase('E3', 3)  # E-W vàng
                        next_green_phase = 0  # Mục tiêu sẽ là N-S xanh
                        print(f"Thời điểm {step/10:.1f}s: Bắt đầu đèn vàng E-W (đạt thời gian tối đa)")
                    phase_duration = 0
                else:
                    # Nếu không an toàn, kéo dài thêm một chút và kiểm tra lại
                    phase_duration = MAX_GREEN_TIME - 15  # Tăng từ 10 lên 15 giây
                    print(f"Thời điểm {step/10:.1f}s: Kéo dài pha xanh thêm vì không an toàn để chuyển đổi")
                
        # In trạng thái mỗi 30 giây
        if step % 300 == 0:
            print(f"Thời gian: {step / 10} giây")
            for direction, data in all_metrics.items():
                direction_name = {'North': 'Bắc', 'South': 'Nam', 'East': 'Đông', 'West': 'Tây'}[direction]
                status = "TỐT" if data['is_good'] else "XẤU"
                print(f"Hướng {direction_name}: Trạng thái = {data['status']:.2f} ({status})")
            print(f"Sự kiện phanh khẩn cấp đến nay: {emergency_braking_events}")
            print("-" * 40)
                
        step += 1
    
    # Mô phỏng hoàn thành
    print("Mô phỏng hoàn thành")
    
    # Tính toán và hiển thị thống kê tổng hợp
    print("\nThống kê tổng hợp:")
    avg_waiting_times = {}
    avg_queue_lengths = {}
    
    for direction, history in metrics_history.items():
        avg_waiting_times[direction] = np.mean([np.mean([m['waiting_time'] for m in h['metrics']]) 
                                               for h in history])
        avg_queue_lengths[direction] = np.mean([np.mean([m['queue_length'] for m in h['metrics']]) 
                                              for h in history])
    
    total_avg_wait = np.mean(list(avg_waiting_times.values()))
    total_avg_queue = np.mean(list(avg_queue_lengths.values()))
    
    print(f"Thời gian chờ trung bình tổng thể: {total_avg_wait:.2f} giây")
    print(f"Độ dài hàng đợi trung bình tổng thể: {total_avg_queue:.2f} xe")
    print(f"Tổng số sự kiện phanh khẩn cấp: {emergency_braking_events}")
    
    direction_names = {'North': 'Bắc', 'South': 'Nam', 'East': 'Đông', 'West': 'Tây'}
    for direction, avg_wait in avg_waiting_times.items():
        dir_name = direction_names[direction]
        print(f"Hướng {dir_name}: Chờ TB = {avg_wait:.2f}s, Hàng đợi TB = {avg_queue_lengths[direction]:.2f}")
    
    # Tạo biểu đồ
    plot_traffic_status(status_data, THRESHOLD)

    traci.close()

if __name__ == "__main__":
    start_sumo()
    run_simulation()
    