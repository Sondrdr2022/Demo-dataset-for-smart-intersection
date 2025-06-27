import os
import sys
import traci
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
import time
import xml.etree.ElementTree as ET

# Thêm SUMO vào đường dẫn Python
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Vui lòng khai báo biến môi trường 'SUMO_HOME'")

# ========== CẤU HÌNH THÔNG SỐ HỆ THỐNG ==========
# Thông số điều khiển đèn giao thông
MIN_GREEN_TIME = 15        # Thời gian xanh tối thiểu (giây)
MAX_GREEN_TIME = 120       # Thời gian xanh tối đa (giây)
MIN_CYCLE_TIME = 60        # Chu kỳ tối thiểu (giây)
MAX_CYCLE_TIME = 200       # Chu kỳ tối đa (giây)
YELLOW_TIME = 3            # Thời gian đèn vàng (giây)
ALL_RED_TIME = 2           # Thời gian tất cả đèn đỏ (giây)
EVAL_INTERVAL = 2          # Đánh giá điều kiện giao thông mỗi n giây

# Ngưỡng đánh giá
STATUS_THRESHOLD = 1.0     # Ngưỡng đánh giá (Status_t < 1.0 = GOOD, >= 1.0 = BAD)
CRITICAL_THRESHOLD = 1.5   # Ngưỡng nguy hiểm cần can thiệp ngay

# Trọng số cho công thức Status_t
WEIGHTS = {
    'l': 0.25,    # Độ dài hàng đợi (queue length)
    'td': 0.20,   # Thời gian đợi (waiting time) 
    'm': 0.20,    # Mật độ phương tiện (density)
    'v': 0.15,    # Vận tốc trung bình (velocity)
    'g': 0.20     # Lưu lượng xe (flow)
}

def start_sumo():
    """Khởi động SUMO với cấu hình tự động"""
    sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
    # Cập nhật đường dẫn đến file cấu hình của bạn
    sumo_config_path = r"C:\Users\Admin\Downloads\sumo test\New folder\20 node\20e.sumocfg"
    
    if not os.path.exists(sumo_config_path):
        print(f"Lỗi: Không tìm thấy file cấu hình SUMO tại '{sumo_config_path}'")
        print("Vui lòng cập nhật biến 'sumo_config_path' trong hàm start_sumo().")
        sys.exit(1)
    
    sumoCmd = [sumoBinary, '-c', sumo_config_path, '--step-length', '0.1']
    traci.start(sumoCmd)

def auto_detect_intersection_structure():
    """Tự động phát hiện cấu trúc nút giao thông"""
    print("🔍 Đang phát hiện cấu trúc nút giao thông...")
    
    # Lấy danh sách tất cả traffic lights
    tl_ids = traci.trafficlight.getIDList()
    print(f"📍 Tìm thấy {len(tl_ids)} đèn giao thông: {tl_ids}")
    
    # Lấy danh sách tất cả lane area detectors (E2)
    detector_ids = traci.lanearea.getIDList()
    print(f"🔍 Tìm thấy {len(detector_ids)} lane area detectors: {detector_ids}")
    
    # Lấy danh sách tất cả lanes
    lane_ids = traci.lane.getIDList()
    internal_lanes = [l for l in lane_ids if ':' in l]  # Internal lanes (trong giao lộ)
    approach_lanes = [l for l in lane_ids if ':' not in l]  # Approach lanes (đến giao lộ)
    
    print(f"🛣️  Tìm thấy {len(approach_lanes)} làn tiếp cận, {len(internal_lanes)} làn nội bộ")
    
    # Phân tích cấu trúc cho từng đèn giao thông
    intersection_data = {}
    
    for tl_id in tl_ids:
        print(f"\n--- Phân tích đèn giao thông: {tl_id} ---")
        
        # Lấy thông tin controlled links
        controlled_links = traci.trafficlight.getControlledLinks(tl_id)
        controlled_lanes = traci.trafficlight.getControlledLanes(tl_id)
        
        print(f"🎮 Kiểm soát {len(controlled_links)} signal phases")
        print(f"🛣️  Kiểm soát {len(set(controlled_lanes))} lanes")
        
        # Phân tích các approach (hướng tiếp cận)
        approaches = {}
        lane_to_detector = {}
        
        # Tìm detectors cho từng lane
        for detector_id in detector_ids:
            try:
                detector_lane = traci.lanearea.getLaneID(detector_id)
                if detector_lane in controlled_lanes:
                    lane_to_detector[detector_lane] = detector_id
                    print(f"🔗 Detector {detector_id} -> Lane {detector_lane}")
            except:
                continue
        
        # Nhóm lanes theo hướng tiếp cận
        for lane_id in set(controlled_lanes):
            if ':' not in lane_id:  # Chỉ xét approach lanes
                # Xác định hướng dựa trên tên lane hoặc vị trí
                edge_id = lane_id.split('_')[0] if '_' in lane_id else lane_id.split('#')[0]
                
                if edge_id not in approaches:
                    approaches[edge_id] = {
                        'lanes': [],
                        'detectors': [],
                        'lane_count': 0
                    }
                
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
        
        print(f"📊 Kết quả phân tích:")
        print(f"   - Số hướng tiếp cận: {len(approaches)}")
        print(f"   - Tổng số làn: {len(set(controlled_lanes))}")
        for approach_name, approach_data in approaches.items():
            print(f"   - {approach_name}: {approach_data['lane_count']} làn, {len(approach_data['detectors'])} detectors")
    
    return intersection_data

def get_traffic_parameters(lane_id, detector_id=None):
    """
    Thu thập các tham số giao thông theo yêu cầu:
    l: độ dài hàng đợi, tđ: thời gian đợi, m: mật độ, v: vận tốc TB, g: lưu lượng
    """
    try:
        # Ưu tiên sử dụng detector nếu có
        if detector_id and detector_id in traci.lanearea.getIDList():
            # Sử dụng lane area detector (E2)
            l = traci.lanearea.getJamLengthVehicle(detector_id)  # Số xe xếp hàng
            m = traci.lanearea.getLastStepOccupancy(detector_id) / 100.0  # Mật độ (0-1)
            vehicle_ids = traci.lanearea.getLastStepVehicleIDs(detector_id)
            vehicle_count = len(vehicle_ids)
            
            # Tính thời gian đợi trung bình
            if vehicle_ids:
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
            else:
                td = 0.0
                v = 0.0
            
            # Lưu lượng (xe/giây -> xe/giờ)
            g = vehicle_count * 3600 / EVAL_INTERVAL if vehicle_count > 0 else 0.0
            
        else:
            # Fallback: sử dụng lane data trực tiếp
            vehicle_ids = traci.lane.getLastStepVehicleIDs(lane_id)
            vehicle_count = len(vehicle_ids)
            
            l = vehicle_count  # Approximation
            m = traci.lane.getLastStepOccupancy(lane_id) / 100.0
            
            if vehicle_ids:
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
            else:
                td = 0.0
                v = 0.0
            
            g = vehicle_count * 3600 / EVAL_INTERVAL if vehicle_count > 0 else 0.0
        
        return {
            'l': l,        # Độ dài hàng đợi (vehicles)
            'td': td,      # Thời gian đợi trung bình (seconds)
            'm': m,        # Mật độ (0-1)
            'v': v,        # Vận tốc trung bình (m/s)
            'g': g,        # Lưu lượng (vehicles/hour)
            'raw_count': vehicle_count
        }
        
    except Exception as e:
        print(f"⚠️  Lỗi khi lấy tham số từ {lane_id}: {e}")
        return {'l': 0, 'td': 0, 'm': 0, 'v': 0, 'g': 0, 'raw_count': 0}

def calculate_status_t(traffic_params, max_values=None):
    """
    Tính Status_t theo công thức tối ưu để tránh vấn đề tổng nhỏ vẫn < 1
    
    Status_t = w_l * (l/l_max) + w_td * (td/td_max) + w_m * m + 
               w_v * (1 - v/v_max) + w_g * (1 - g/g_max)
    
    Trong đó:
    - l, td, m: càng cao càng tệ (normalize về 0-1)
    - v, g: càng cao càng tốt (dùng 1 - normalized để đảo ngược)
    """
    
    # Giá trị max để normalize (có thể điều chỉnh theo thực tế)
    if max_values is None:
        max_values = {
            'l_max': 20,      # 20 xe xếp hàng = tắc nghẽn hoàn toàn
            'td_max': 120,    # 120 giây chờ = tắc nghẽn hoàn toàn  
            'v_max': 15,      # 15 m/s = tốc độ tối đa trong thành phố
            'g_max': 1800     # 1800 xe/giờ = lưu lượng tối đa cho 1 làn
        }
    
    # Normalize các tham số
    l_norm = min(traffic_params['l'] / max_values['l_max'], 1.0)
    td_norm = min(traffic_params['td'] / max_values['td_max'], 1.0)
    m_norm = traffic_params['m']  # Đã là 0-1
    v_norm = min(traffic_params['v'] / max_values['v_max'], 1.0)
    g_norm = min(traffic_params['g'] / max_values['g_max'], 1.0)
    
    # Tính Status_t
    status_t = (
        WEIGHTS['l'] * l_norm +
        WEIGHTS['td'] * td_norm +
        WEIGHTS['m'] * m_norm +
        WEIGHTS['v'] * (1.0 - v_norm) +  # Đảo ngược: tốc độ thấp = tệ
        WEIGHTS['g'] * (1.0 - g_norm)    # Đảo ngược: lưu lượng thấp = tệ
    )
    
    return status_t

def evaluate_intersection_status(intersection_data, tl_id):
    """Đánh giá trạng thái toàn bộ nút giao thông"""
    
    approaches_status = {}
    overall_status = 0.0
    total_weight = 0.0
    
    intersection = intersection_data[tl_id]
    
    for approach_name, approach_data in intersection['approaches'].items():
        approach_statuses = []
        
        # Đánh giá từng làn trong approach
        for i, lane_id in enumerate(approach_data['lanes']):
            detector_id = None
            if i < len(approach_data['detectors']):
                detector_id = approach_data['detectors'][i]
            
            # Lấy tham số giao thông
            traffic_params = get_traffic_parameters(lane_id, detector_id)
            
            # Tính Status_t cho làn này
            lane_status = calculate_status_t(traffic_params)
            approach_statuses.append(lane_status)
        
        # Trạng thái approach = trung bình trọng số các làn
        if approach_statuses:
            approach_status = np.mean(approach_statuses)
            approaches_status[approach_name] = {
                'status': approach_status,
                'lane_count': len(approach_statuses),
                'lane_statuses': approach_statuses
            }
            
            # Tích lũy vào trạng thái tổng thể (trọng số theo số làn)
            weight = len(approach_statuses)
            overall_status += approach_status * weight
            total_weight += weight
    
    # Tính trạng thái tổng thể
    if total_weight > 0:
        overall_status = overall_status / total_weight
    
    return overall_status, approaches_status

def adaptive_phase_timing(approaches_status, current_green_time, current_cycle_time):
    """
    Điều chỉnh thời gian pha dựa trên trạng thái giao thông
    
    Mục tiêu tối ưu:
    1. Tăng lưu lượng thoát (g↑) cho các hướng có Status_t cao
    2. Giảm thời gian đợi (td↓) tổng thể 
    3. Cải thiện vận tốc trung bình (v↑)
    """
    
    # Tìm approach có Status_t cao nhất (cần ưu tiên)
    max_status = 0
    priority_approach = None
    
    for approach_name, data in approaches_status.items():
        if data['status'] > max_status:
            max_status = data['status']
            priority_approach = approach_name
    
    # Quy tắc điều chỉnh
    adjustment = {
        'green_time_change': 0,
        'cycle_time_change': 0,
        'priority_approach': priority_approach,
        'reason': ''
    }
    
    if max_status >= CRITICAL_THRESHOLD:
        # Tình huống nguy hiểm: cần can thiệp mạnh
        adjustment['green_time_change'] = +15  # Tăng 15s xanh cho hướng ưu tiên
        adjustment['cycle_time_change'] = +10  # Tăng chu kỳ 10s
        adjustment['reason'] = f"KHẨN CẤP: {priority_approach} có Status_t = {max_status:.2f}"
        
    elif max_status >= STATUS_THRESHOLD:
        # Tình huống tắc nghẽn: cần điều chỉnh
        adjustment['green_time_change'] = +8   # Tăng 8s xanh
        adjustment['cycle_time_change'] = +5   # Tăng chu kỳ 5s  
        adjustment['reason'] = f"TẮC NGHẼN: {priority_approach} có Status_t = {max_status:.2f}"
        
    elif max_status < STATUS_THRESHOLD * 0.5:
        # Tình huống thông thoáng: có thể giảm thời gian
        adjustment['green_time_change'] = -5   # Giảm 5s xanh
        adjustment['cycle_time_change'] = -3   # Giảm chu kỳ 3s
        adjustment['reason'] = f"THÔNG THOÁNG: Có thể tối ưu thời gian"
    
    # Áp dụng giới hạn
    new_green_time = max(MIN_GREEN_TIME, 
                        min(MAX_GREEN_TIME, 
                            current_green_time + adjustment['green_time_change']))
    
    new_cycle_time = max(MIN_CYCLE_TIME,
                        min(MAX_CYCLE_TIME,
                            current_cycle_time + adjustment['cycle_time_change']))
    
    adjustment['new_green_time'] = new_green_time
    adjustment['new_cycle_time'] = new_cycle_time
    
    return adjustment

def create_adaptive_traffic_program(tl_id, intersection_data, green_time=30):
    """Tạo chương trình đèn giao thông thích ứng"""
    
    intersection = intersection_data[tl_id]
    num_approaches = len(intersection['approaches'])
    
    # Tạo các phase dựa trên số hướng tiếp cận
    if num_approaches == 4:
        # Giao lộ 4 hướng: NS và EW
        phases = [
            {'duration': green_time, 'state': 'GGGrrrrr', 'name': 'NS_green'},
            {'duration': YELLOW_TIME, 'state': 'yyyrrrrr', 'name': 'NS_yellow'},
            {'duration': ALL_RED_TIME, 'state': 'rrrrrrrr', 'name': 'all_red_1'},
            {'duration': green_time, 'state': 'rrrGGGrr', 'name': 'EW_green'},
            {'duration': YELLOW_TIME, 'state': 'rrryyyrr', 'name': 'EW_yellow'},
            {'duration': ALL_RED_TIME, 'state': 'rrrrrrrr', 'name': 'all_red_2'}
        ]
    elif num_approaches == 3:
        # Giao lộ 3 hướng (T-junction)
        phases = [
            {'duration': green_time, 'state': 'GGrrrr', 'name': 'main_green'},
            {'duration': YELLOW_TIME, 'state': 'yyrrrr', 'name': 'main_yellow'},
            {'duration': ALL_RED_TIME, 'state': 'rrrrrr', 'name': 'all_red_1'},
            {'duration': green_time//2, 'state': 'rrGGrr', 'name': 'side_green'},
            {'duration': YELLOW_TIME, 'state': 'rryyrr', 'name': 'side_yellow'},
            {'duration': ALL_RED_TIME, 'state': 'rrrrrr', 'name': 'all_red_2'}
        ]
    else:
        # Fallback: sử dụng program mặc định
        return None
    
    return phases

def run_adaptive_simulation():
    """Chạy mô phỏng với hệ thống đèn giao thông thích ứng"""
    
    print("🚦 === HỆ THỐNG ĐÈN GIAO THÔNG THÍCH ỨNG === 🚦")
    print("🎯 Mục tiêu: Tối ưu lưu lượng thoát và giảm thời gian đợi")
    
    # Tự động phát hiện cấu trúc giao lộ
    intersection_data = auto_detect_intersection_structure()
    
    if not intersection_data:
        print("❌ Không phát hiện được đèn giao thông nào!")
        return
    
    # Khởi tạo dữ liệu tracking
    tracking_data = {
        'time': [],
        'overall_status': [],
        'approaches_status': {},
        'green_times': [],
        'cycle_times': [],
        'adjustments': []
    }
    
    # Biến điều khiển
    step = 0
    current_green_time = 30  # Thời gian xanh hiện tại
    current_cycle_time = 90  # Chu kỳ hiện tại
    last_adjustment_time = 0
    
    # Lấy đèn giao thông chính để điều khiển
    main_tl_id = list(intersection_data.keys())[0]
    print(f"🎮 Điều khiển đèn giao thông chính: {main_tl_id}")
    
    try:
        while step < 36000:  # 1 giờ mô phỏng
            traci.simulationStep()
            current_time = step / 10.0
            
            # Đánh giá định kỳ
            if step % (EVAL_INTERVAL * 10) == 0:
                
                # Đánh giá trạng thái giao lộ
                overall_status, approaches_status = evaluate_intersection_status(
                    intersection_data, main_tl_id
                )
                
                # Lưu dữ liệu tracking
                tracking_data['time'].append(current_time)
                tracking_data['overall_status'].append(overall_status)
                tracking_data['green_times'].append(current_green_time)
                tracking_data['cycle_times'].append(current_cycle_time)
                
                # Lưu trạng thái từng approach
                for approach_name, data in approaches_status.items():
                    if approach_name not in tracking_data['approaches_status']:
                        tracking_data['approaches_status'][approach_name] = []
                    tracking_data['approaches_status'][approach_name].append(data['status'])
                
                # Đánh giá tình trạng giao thông
                if overall_status < STATUS_THRESHOLD:
                    traffic_condition = "🟢 GOOD (Thông thoáng)"
                elif overall_status < CRITICAL_THRESHOLD:
                    traffic_condition = "🟡 BAD (Tắc nghẽn nhẹ)"
                else:
                    traffic_condition = "🔴 CRITICAL (Tắc nghẽn nghiêm trọng)"
                
                # In thông tin định kỳ (mỗi 30 giây)
                if step % 300 == 0:
                    print(f"\n⏰ Thời gian: {current_time:.1f}s")
                    print(f"📊 Status_t tổng thể: {overall_status:.3f}")
                    print(f"🚦 Tình trạng: {traffic_condition}")
                    print(f"⚡ Thời gian xanh hiện tại: {current_green_time}s")
                    print(f"🔄 Chu kỳ hiện tại: {current_cycle_time}s")
                    
                    print("📍 Trạng thái từng hướng:")
                    for approach_name, data in approaches_status.items():
                        status_color = "🟢" if data['status'] < STATUS_THRESHOLD else "🔴"
                        print(f"   {approach_name}: {data['status']:.3f} {status_color}")
                
                # Điều chỉnh thời gian đèn (mỗi 60 giây)
                if current_time - last_adjustment_time >= 60:
                    
                    adjustment = adaptive_phase_timing(
                        approaches_status, current_green_time, current_cycle_time
                    )
                    
                    if adjustment['green_time_change'] != 0 or adjustment['cycle_time_change'] != 0:
                        
                        current_green_time = adjustment['new_green_time']
                        current_cycle_time = adjustment['new_cycle_time']
                        
                        print(f"\n🔧 ĐIỀU CHỈNH THỜI GIAN ĐÈN:")
                        print(f"   📋 Lý do: {adjustment['reason']}")
                        print(f"   🎯 Ưu tiên: {adjustment['priority_approach']}")
                        print(f"   ⚡ Thời gian xanh: {current_green_time}s ({adjustment['green_time_change']:+d}s)")
                        print(f"   🔄 Chu kỳ: {current_cycle_time}s ({adjustment['cycle_time_change']:+d}s)")
                        
                        tracking_data['adjustments'].append({
                            'time': current_time,
                            'adjustment': adjustment
                        })
                        
                        last_adjustment_time = current_time
                        
                        # Áp dụng thay đổi (giả lập)
                        # Trong thực tế, cần cập nhật traffic light program
                        # traci.trafficlight.setProgram() và setPhaseDefinition()
            
            step += 1
    
    except Exception as e:
        print(f"❌ Lỗi trong simulation: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\n🏁 === KẾT THÚC MÔ PHỎNG ===")
        print(f"📊 Thời gian mô phỏng: {tracking_data['time'][-1]:.1f} giây")
        print(f"🔧 Số lần điều chỉnh: {len(tracking_data['adjustments'])}")
        
        # Thống kê kết quả
        if tracking_data['overall_status']:
            avg_status = np.mean(tracking_data['overall_status'])
            good_ratio = len([s for s in tracking_data['overall_status'] if s < STATUS_THRESHOLD]) / len(tracking_data['overall_status'])
            
            print(f"📈 Status_t trung bình: {avg_status:.3f}")
            print(f"✅ Tỷ lệ thời gian GOOD: {good_ratio*100:.1f}%")
        
        # Vẽ biểu đồ kết quả
        plot_adaptive_results(tracking_data)
        
        try:
            traci.close()
        except:
            pass

def plot_adaptive_results(tracking_data):
    """Vẽ biểu đồ kết quả hệ thống thích ứng"""
    
    try:
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        
        time = tracking_data['time']
        
        # Biểu đồ 1: Status_t tổng thể
        ax1.plot(time, tracking_data['overall_status'], 'b-', linewidth=2, label='Status_t')
        ax1.axhline(y=STATUS_THRESHOLD, color='orange', linestyle='--', linewidth=2, label=f'Ngưỡng BAD ({STATUS_THRESHOLD})')
        ax1.axhline(y=CRITICAL_THRESHOLD, color='red', linestyle='--', linewidth=2, label=f'Ngưỡng CRITICAL ({CRITICAL_THRESHOLD})')
        ax1.fill_between(time, 0, STATUS_THRESHOLD, color='green', alpha=0.2, label='Vùng GOOD')
        ax1.fill_between(time, STATUS_THRESHOLD, CRITICAL_THRESHOLD, color='orange', alpha=0.2, label='Vùng BAD')
        ax1.fill_between(time, CRITICAL_THRESHOLD, max(tracking_data['overall_status'])*1.1, color='red', alpha=0.2, label='Vùng CRITICAL')
        
        ax1.set_title('Trạng Thái Tổng Thể Nút Giao Thông (Status_t)', fontsize=14)
        ax1.set_xlabel('Thời gian (giây)')
        ax1.set_ylabel('Status_t')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Biểu đồ 2: Trạng thái từng hướng
        colors = ['blue', 'green', 'red', 'orange', 'purple']
        for i, (approach_name, status_data) in enumerate(tracking_data['approaches_status'].items()):
            if status_data:  # Kiểm tra có dữ liệu
                ax2.plot(time[:len(status_data)], status_data, 
                        color=colors[i % len(colors)], linewidth=2, label=f'Hướng {approach_name}')
        
        ax2.axhline(y=STATUS_THRESHOLD, color='red', linestyle='--', alpha=0.7)
        ax2.set_title('Trạng Thái Từng Hướng Tiếp Cận', fontsize=14)
        ax2.set_xlabel('Thời gian (giây)')
        ax2.set_ylabel('Status_t')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Biểu đồ 3: Thời gian đèn xanh
        ax3.plot(time, tracking_data['green_times'], 'g-', linewidth=2, label='Thời gian xanh')
        ax3.axhline(y=MIN_GREEN_TIME, color='red', linestyle=':', alpha=0.7, label=f'Min ({MIN_GREEN_TIME}s)')
        ax3.axhline(y=MAX_GREEN_TIME, color='red', linestyle=':', alpha=0.7, label=f'Max ({MAX_GREEN_TIME}s)')
        
        # Đánh dấu các điểm điều chỉnh
        for adj in tracking_data['adjustments']:
            ax3.axvline(x=adj['time'], color='orange', linestyle='--', alpha=0.7)
        
        ax3.set_title('Thời Gian Đèn Xanh Thích Ứng', fontsize=14)
        ax3.set_xlabel('Thời gian (giây)')
        ax3.set_ylabel('Thời gian xanh (giây)')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # Biểu đồ 4: Chu kỳ đèn
        ax4.plot(time, tracking_data['cycle_times'], 'm-', linewidth=2, label='Chu kỳ')
        ax4.axhline(y=MIN_CYCLE_TIME, color='red', linestyle=':', alpha=0.7, label=f'Min ({MIN_CYCLE_TIME}s)')
        ax4.axhline(y=MAX_CYCLE_TIME, color='red', linestyle=':', alpha=0.7, label=f'Max ({MAX_CYCLE_TIME}s)')
        
        # Đánh dấu các điểm điều chỉnh
        for adj in tracking_data['adjustments']:
            ax4.axvline(x=adj['time'], color='orange', linestyle='--', alpha=0.7)
        
        ax4.set_title('Chu Kỳ Đèn Giao Thông Thích Ứng', fontsize=14)
        ax4.set_xlabel('Thời gian (giây)')
        ax4.set_ylabel('Chu kỳ (giây)')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Lưu biểu đồ
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'adaptive_traffic_results_{timestamp}.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"📊 Đã lưu biểu đồ kết quả: '{filename}'")
        
        plt.show()
        
    except Exception as e:
        print(f"❌ Lỗi khi vẽ biểu đồ: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    start_sumo()
    run_adaptive_simulation()