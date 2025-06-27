import traci

def get_lane_status(lane_id, l_max, tđ_max, m_max):
    # Lấy số xe dừng, chiều dài hàng đợi, thời gian đợi trung bình, mật độ
    l = traci.lane.getLastStepHaltingNumber(lane_id)  # số xe đang dừng trên làn
    tđ = traci.lane.getWaitingTime(lane_id)  # thời gian chờ trung bình
    m = traci.lane.getLastStepVehicleNumber(lane_id) / (traci.lane.getLength(lane_id)/1000)  # mật độ xe/km

    # Chuẩn hóa các giá trị, tránh chia cho 0
    l_ratio = l / l_max if l_max else 0
    tđ_ratio = tđ / tđ_max if tđ_max else 0
    m_ratio = m / m_max if m_max else 0

    # Tính chỉ số trạng thái (có thể hiệu chỉnh trọng số)
    Status_t = 0.4 * l_ratio + 0.3 * tđ_ratio + 0.3 * m_ratio
    return Status_t

def main():
    traci.start(["sumo-gui", "-c", r"C:\Users\Admin\Downloads\sumo test\New folder\20 node\20e.sumocfg"])
    tls_ids = traci.trafficlight.getIDList()  # Lấy danh sách đèn giao thông

    # Tham số max giả định (hoặc lấy từ cấu hình/làn)
    l_max, tđ_max, m_max = 10, 60, 30  # Tuỳ chỉnh theo thực tế mô phỏng

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        for tls_id in tls_ids:
            controlled_lanes = traci.trafficlight.getControlledLanes(tls_id)
            status_list = []
            for lane_id in controlled_lanes:
                status = get_lane_status(lane_id, l_max, tđ_max, m_max)
                status_list.append(status)
            
            max_status = max(status_list)
            if max_status >= 1:
                print(f"BAD: {tls_id} congested, adjusting light...")
                # Điều chỉnh: tăng thời gian xanh làn bị tắc
                # Ví dụ: traci.trafficlight.setPhaseDuration(tls_id, new_duration)
            else:
                print(f"GOOD: {tls_id} is clear.")

    traci.close()

if __name__ == "__main__":
    main()