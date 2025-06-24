#!/usr/bin/env python
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

# Cấu hình
THRESHOLD = 0.7      # Ngưỡng cho trạng thái được coi là "TỐT"
SIMULATION_TIME = 3600  # Thời gian mô phỏng (giây)
DATA_COLLECTION_INTERVAL = 50  # Thu thập dữ liệu mỗi 5 giây (50 * 0.1s)

def start_sumo():
    """Khởi động SUMO với cấu hình mặc định"""
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
    plt.title('Trạng Thái Giao Thông Theo Hướng So Với Ngưỡng (Đèn Giao Thông Mặc Định)', fontsize=16)
    plt.legend(fontsize=12)
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('trang_thai_giao_thong_mac_dinh.png', dpi=150)
    print("Đã lưu biểu đồ dưới dạng 'trang_thai_giao_thong_mac_dinh.png'")
    plt.show()

def run_monitoring():
    """Chạy giám sát giao thông (không điều khiển đèn giao thông)"""
    # Xác định nhóm bộ dò theo hướng
    detector_groups = {
        'North': ['E1-3-1', 'E1-3-2'],  # Hướng Bắc
        'East': ['E3-4-1', 'E3-4-2'],   # Hướng Đông
        'South': ['E5-3-1', 'E5-3-2'],  # Hướng Nam
        'West': ['E3-2-1', 'E3-2-2']    # Hướng Tây
    }
    
    # Dữ liệu cho trực quan hóa
    status_data = {'time': [], 'North': [], 'South': [], 'East': [], 'West': []}
    
    # Vòng lặp mô phỏng chính
    step = 0
    emergency_braking_events = 0
    
    print("Bắt đầu mô phỏng với đèn giao thông mặc định...")
    print("Thu thập dữ liệu trạng thái giao thông...")
    
    while step < SIMULATION_TIME * 10:  # Nhân 10 vì mỗi bước là 0.1s
        traci.simulationStep()
        
        # Kiểm tra sự kiện phanh khẩn cấp
        if step % 10 == 0:
            emergency_braking_events += count_emergency_braking_events()
        
        # Thu thập dữ liệu theo định kỳ
        if step % DATA_COLLECTION_INTERVAL == 0:
            # Lấy dữ liệu từ tất cả các bộ dò
            all_metrics = {}
            for direction, detectors in detector_groups.items():
                direction_metrics = [get_lane_metrics(detector) for detector in detectors]
                status, components = calculate_status(direction_metrics)
                all_metrics[direction] = {
                    'status': status,
                    'is_good': status >= THRESHOLD
                }
            
            # Lưu dữ liệu trạng thái
            time_sec = step / 10.0  # Chuyển đổi sang giây
            status_data['time'].append(time_sec)
            
            for direction, data in all_metrics.items():
                status_data[direction].append(data['status'])
        
        # In trạng thái mỗi 60 giây
        if step % 600 == 0:
            time_sec = step / 10.0
            if step > 0:  # Bỏ qua lần đầu tiên
                print(f"Thời gian: {time_sec:.1f} giây")
                for direction in ['North', 'South', 'East', 'West']:
                    direction_name = {'North': 'Bắc', 'South': 'Nam', 'East': 'Đông', 'West': 'Tây'}[direction]
                    if status_data[direction]:  # Kiểm tra có dữ liệu không
                        status_value = status_data[direction][-1]  # Lấy giá trị mới nhất
                        status_text = "TỐT" if status_value >= THRESHOLD else "XẤU"
                        print(f"Hướng {direction_name}: Trạng thái = {status_value:.2f} ({status_text})")
                print(f"Sự kiện phanh khẩn cấp đến nay: {emergency_braking_events}")
                print("-" * 40)
                
        step += 1
    
    # Mô phỏng hoàn thành
    print("\nMô phỏng hoàn thành!")
    print(f"Tổng số sự kiện phanh khẩn cấp: {emergency_braking_events}")
    
    # Tạo biểu đồ
    plot_traffic_status(status_data, THRESHOLD)
    
    traci.close()

if __name__ == "__main__":
    start_sumo()
    run_monitoring()