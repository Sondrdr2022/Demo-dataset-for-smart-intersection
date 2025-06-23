import os
import sys

# Kiểm tra biến môi trường SUMO_HOME và nạp đường dẫn tools
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

import traci
import sumolib
import matplotlib.pyplot as plt

# CẤU HÌNH
SUMO_BINARY = "sumo-gui"  # Hoặc "sumo"
SUMO_CFG = r"C:\Users\Admin\Downloads\sumo test\New folder\dataset.sumocfg"
E2_DETECTORS = ["E3-2-2", "E3-2-1", "E1-3-1", "E1-3-2", "E3-4-1", "E3-4-2", "E5-3-1", "E5-3-2"]

# Trọng số mặc định: chỉ dùng giá trị 1 cho tất cả loại xe
WEIGHT_MAP = {
    "car": 1,
    "bus": 1,
    "truck": 1,
    "emergency": 1,
    # Các xe khác cũng = 1
}

# Trọng số cho công thức status t
W1 = 1  # queue
W2 = 1  # waiting time
W3 = 1  # density

LIMIT = 10
N_STEP = 5

def get_detector_status(det_id):
    v_ids = traci.lanearea.getLastStepVehicleIDs(det_id)
    queue = traci.lanearea.getLastStepHaltingNumber(det_id)
    waiting_time = 0
    total_weight = 0
    for vid in v_ids:
        vtype = traci.vehicle.getTypeID(vid)
        weight = WEIGHT_MAP.get(vtype, 1)
        total_weight += weight
        waiting_time += traci.vehicle.getAccumulatedWaitingTime(vid) * weight
    density = traci.lanearea.getLastStepOccupancy(det_id) / 100
    return total_weight, waiting_time, queue, density

def main():
    import collections

    sumo_cmd = [SUMO_BINARY, "-c", SUMO_CFG, "--start"]
    traci.start(sumo_cmd)

    status_t_list = collections.deque(maxlen=N_STEP)
    status_t_all_steps = []
    steps = []

    try:
        step = 0
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()

            total_weight = 0
            total_waiting = 0
            total_queue = 0
            total_density = 0
            for det in E2_DETECTORS:
                w, wt, q, d = get_detector_status(det)
                total_weight += w
                total_waiting += wt
                total_queue += q
                total_density += d

            n = len(E2_DETECTORS)
            avg_weight = total_weight / n if n else 0
            avg_waiting = total_waiting / n if n else 0
            avg_queue = total_queue / n if n else 0
            avg_density = total_density / n if n else 0

            status_t = W1 * avg_queue + W2 * avg_waiting + W3 * avg_density
            status_t_list.append(status_t)
            status_t_all_steps.append(status_t)
            steps.append(step)

            print(f"Step {step}: status_t={status_t:.2f}")

            if len(status_t_list) == N_STEP:
                avg_status_t = sum(status_t_list) / N_STEP
                if avg_status_t >= LIMIT:
                    print(f"Trạng thái: GOOD (avg_status_t={avg_status_t:.2f})")
                else:
                    print(f"Trạng thái: BAD (avg_status_t={avg_status_t:.2f})")
            step += 1

        traci.close()

        # Sau khi mô phỏng xong, vẽ biểu đồ
        plt.figure(figsize=(10, 5))
        plt.plot(steps, status_t_all_steps, label="status t (default)")
        plt.axhline(y=LIMIT, color='r', linestyle='--', label=f'LIMIT = {LIMIT}')
        plt.xlabel('Step')
        plt.ylabel('Status t')
        plt.title('Status t (chạy mặc định, không ưu tiên loại xe)')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    except Exception as e:
        traci.close()
        raise e

if __name__ == "__main__":
    main()