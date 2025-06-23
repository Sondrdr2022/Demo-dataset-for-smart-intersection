import os
import sys
import numpy as np
import random
import matplotlib.pyplot as plt
from datetime import datetime

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
    SUMO_BINARY = os.path.join(os.environ['SUMO_HOME'], 'bin', 'sumo-gui')
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

import traci

# Cấu hình
SUMO_CFG = r"C:\Users\Admin\Downloads\sumo test\New folder\dataset.sumocfg"
TLS_ID = "E3"
LANES = ["E3-2-2", "E3-2-1", "E1-3-1", "E1-3-2", "E3-4-1", "E3-4-2", "E5-3-1", "E5-3-2"]

# Tham số pha đèn
GREEN_PHASE_1 = 0
YELLOW_PHASE_1 = 1
GREEN_PHASE_2 = 2
YELLOW_PHASE_2 = 3
YELLOW_DURATION = 4  # Tăng từ 3 lên 4 để xe có thời gian phản ứng
MIN_PHASE_DURATION = 15  # Thời gian tối thiểu một pha xanh

# Q-learning params
alpha = 0.2  # Tốc độ học
gamma = 0.95  # Hệ số giảm reward tương lai
epsilon = 0.3  # Xác suất khám phá
epsilon_decay = 0.95  # Tốc độ giảm epsilon sau mỗi episode
epsilon_min = 0.05  # Giá trị tối thiểu của epsilon
EPISODES = 30
MAX_STEP = 5000

# Weights cho reward function
QUEUE_WEIGHT = 2.0
WAITING_WEIGHT = 1.0
SPEED_WEIGHT = 0.5

# Phân loại xe theo trọng số
WEIGHT_MAP = {
    "car": 1,
    "bus": 2,
    "truck": 2,
    "emergency": 3,
}

# Lấy trạng thái hiện tại của hệ thống
def get_state():
    # Lấy thông tin từng làn và lưu theo hướng
    north_queue = traci.lanearea.getLastStepHaltingNumber("E3-2-1") + traci.lanearea.getLastStepHaltingNumber("E3-2-2")
    south_queue = traci.lanearea.getLastStepHaltingNumber("E5-3-1") + traci.lanearea.getLastStepHaltingNumber("E5-3-2") 
    east_queue = traci.lanearea.getLastStepHaltingNumber("E3-4-1") + traci.lanearea.getLastStepHaltingNumber("E3-4-2")
    west_queue = traci.lanearea.getLastStepHaltingNumber("E1-3-1") + traci.lanearea.getLastStepHaltingNumber("E1-3-2")
    
    # Tính queue theo hướng (Bắc-Nam vs Đông-Tây)
    ns_queue = north_queue + south_queue  # Bắc-Nam
    ew_queue = east_queue + west_queue    # Đông-Tây
    
    # Lấy thêm mật độ
    total_density = sum(traci.lanearea.getLastStepOccupancy(det) for det in LANES) / len(LANES)
    
    # Discretize các giá trị
    ns_queue_level = min(int(ns_queue // 5), 4)  # 0-4
    ew_queue_level = min(int(ew_queue // 5), 4)  # 0-4
    density_level = min(int(total_density * 5), 4)  # 0-4
    
    # Thêm thông tin pha đèn hiện tại vào trạng thái
    current_phase = traci.trafficlight.getPhase(TLS_ID)
    is_ns_green = 1 if current_phase == GREEN_PHASE_1 else 0
    
    return (ns_queue_level, ew_queue_level, density_level, is_ns_green)

# Hàm tính reward tổng hợp
def get_reward():
    # 1. Tổng số xe đang chờ (queue)
    total_queue = sum(traci.lanearea.getLastStepHaltingNumber(det) for det in LANES)
    
    # 2. Tổng thời gian chờ
    total_waiting_time = 0
    for lane in LANES:
        vehicle_ids = traci.lanearea.getLastStepVehicleIDs(lane)
        for vid in vehicle_ids:
            try:
                veh_type = traci.vehicle.getTypeID(vid)
                weight = WEIGHT_MAP.get(veh_type, 1)
                waiting_time = traci.vehicle.getAccumulatedWaitingTime(vid)
                total_waiting_time += waiting_time * weight
            except:
                pass  # Xe có thể đã biến mất
    
    # 3. Tốc độ trung bình
    avg_speeds = []
    for lane in LANES:
        vehicle_ids = traci.lanearea.getLastStepVehicleIDs(lane)
        if vehicle_ids:
            speeds = [traci.vehicle.getSpeed(vid) for vid in vehicle_ids if traci.vehicle.getSpeed(vid) > 0.1]
            if speeds:
                avg_speeds.append(sum(speeds) / len(speeds))
    avg_speed = sum(avg_speeds) / len(avg_speeds) if avg_speeds else 0
    
    # Tính reward tổng hợp (âm)
    reward = -(
        QUEUE_WEIGHT * total_queue + 
        WAITING_WEIGHT * (total_waiting_time / 100) - 
        SPEED_WEIGHT * avg_speed
    )
    
    return reward

# Chọn hành động dựa trên Q-table và chính sách epsilon-greedy
def choose_action(state, Q_table, n_actions, epsilon):
    if np.random.rand() < epsilon:
        return np.random.randint(n_actions)
    else:
        return np.argmax(Q_table[state])

# Hàm chính
def main():
    # Tạo thư mục để lưu kết quả
    results_dir = "q_learning_results"
    os.makedirs(results_dir, exist_ok=True)
    
    # Kích thước không gian trạng thái và hành động
    state_space = (5, 5, 5, 2)  # ns_queue, ew_queue, density, is_ns_green
    n_actions = 2  # 0: xanh NS, 1: xanh EW
    
    # Khởi tạo hoặc tải Q-table
    qtable_file = os.path.join(results_dir, "qtable_traffic.npy")
    try:
        Q = np.load(qtable_file)
        print(f"Loaded previous Q-table from {qtable_file}")
    except:
        Q = np.zeros(state_space + (n_actions,))
        print("Created new Q-table")
    
    # Tracking metrics
    episode_rewards = []
    episode_avg_queues = []
    episode_avg_waiting_times = []
    
    # Training loop
    current_epsilon = epsilon
    for episode in range(EPISODES):
        print(f"\nEpisode {episode+1}/{EPISODES} (epsilon: {current_epsilon:.3f})")
        
        # Khởi tạo SUMO
        sumo_cmd = [SUMO_BINARY, "-c", SUMO_CFG, "--start"]
        traci.start(sumo_cmd)
        
        # Tracking metrics cho episode hiện tại
        step = 0
        episode_reward = 0
        queues = []
        waiting_times = []
        
        # Khởi tạo đèn giao thông
        traci.trafficlight.setPhase(TLS_ID, GREEN_PHASE_1)
        state = get_state()
        
        # Các biến theo dõi trạng thái đèn
        phase_duration = 0
        in_yellow = False
        yellow_timer = 0
        current_phase = GREEN_PHASE_1
        action = 0  # Mặc định bắt đầu với NS (action 0)
        
        # Simulation loop
        try:
            while traci.simulation.getMinExpectedNumber() > 0 and step < MAX_STEP:
                # Chỉ chọn action mới khi không trong trạng thái vàng và đủ thời gian pha tối thiểu
                if not in_yellow and phase_duration >= MIN_PHASE_DURATION:
                    action = choose_action(state, Q, n_actions, current_epsilon)
                    
                    # Kiểm tra xem có cần chuyển pha không
                    target_phase = GREEN_PHASE_1 if action == 0 else GREEN_PHASE_2
                    if target_phase != current_phase:
                        # Cần chuyển pha => bật đèn vàng
                        yellow_phase = YELLOW_PHASE_1 if current_phase == GREEN_PHASE_1 else YELLOW_PHASE_2
                        traci.trafficlight.setPhase(TLS_ID, yellow_phase)
                        in_yellow = True
                        yellow_timer = 0
                
                # Xử lý đèn vàng nếu đang trong trạng thái chuyển pha
                if in_yellow:
                    yellow_timer += 1
                    if yellow_timer >= YELLOW_DURATION:
                        # Hết thời gian vàng, chuyển sang pha xanh mới
                        current_phase = GREEN_PHASE_1 if action == 0 else GREEN_PHASE_2
                        traci.trafficlight.setPhase(TLS_ID, current_phase)
                        in_yellow = False
                        phase_duration = 0
                else:
                    # Tăng thời gian đã ở pha hiện tại
                    phase_duration += 1
                
                # Thực hiện một bước mô phỏng
                traci.simulationStep()
                
                # Lấy trạng thái mới và reward
                next_state = get_state()
                reward = get_reward()
                episode_reward += reward
                
                # Thu thập metrics
                total_queue = sum(traci.lanearea.getLastStepHaltingNumber(det) for det in LANES)
                queues.append(total_queue)
                
                total_waiting = 0
                for lane in LANES:
                    for vid in traci.lanearea.getLastStepVehicleIDs(lane):
                        try:
                            total_waiting += traci.vehicle.getAccumulatedWaitingTime(vid)
                        except:
                            pass
                waiting_times.append(total_waiting)
                
                # Q-learning update (chỉ khi đã chọn action mới và không phải đèn vàng)
                if not in_yellow and phase_duration == 1:  # Vừa chuyển phase xong
                    Q[state + (action,)] = Q[state + (action,)] + alpha * (
                        reward + gamma * np.max(Q[next_state]) - Q[state + (action,)]
                    )
                
                # Cập nhật trạng thái
                state = next_state
                step += 1
                
                if step % 100 == 0:
                    print(f"Step {step}, Queue: {total_queue}, Reward: {reward:.2f}")
            
            # Kết thúc episode, thu thập metrics
            if queues:
                episode_avg_queues.append(sum(queues) / len(queues))
            if waiting_times:
                episode_avg_waiting_times.append(sum(waiting_times) / len(waiting_times))
            episode_rewards.append(episode_reward)
            
            print(f"Episode {episode+1} completed. Steps: {step}, Avg Queue: {episode_avg_queues[-1]:.2f}, Total Reward: {episode_reward:.2f}")
            
        except Exception as e:
            print(f"Error in episode {episode+1}: {e}")
        finally:
            traci.close()
        
        # Decay epsilon
        current_epsilon = max(epsilon_min, current_epsilon * epsilon_decay)
        
        # Lưu Q-table sau mỗi episode
        np.save(qtable_file, Q)
        print(f"Saved Q-table to {qtable_file}")
    
    # Vẽ biểu đồ kết quả
    plt.figure(figsize=(15, 5))
    
    # Plot rewards
    plt.subplot(1, 3, 1)
    plt.plot(range(1, EPISODES+1), episode_rewards)
    plt.xlabel('Episode')
    plt.ylabel('Total Reward')
    plt.title('Reward per Episode')
    plt.grid(True)
    
    # Plot average queue
    plt.subplot(1, 3, 2)
    plt.plot(range(1, EPISODES+1), episode_avg_queues)
    plt.xlabel('Episode')
    plt.ylabel('Average Queue Length')
    plt.title('Average Queue per Episode')
    plt.grid(True)
    
    # Plot average waiting time
    plt.subplot(1, 3, 3)
    plt.plot(range(1, EPISODES+1), episode_avg_waiting_times)
    plt.xlabel('Episode')
    plt.ylabel('Average Waiting Time')
    plt.title('Average Waiting Time per Episode')
    plt.grid(True)
    
    plt.tight_layout()
    
    # Lưu biểu đồ
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    plot_file = os.path.join(results_dir, f"results_{timestamp}.png")
    plt.savefig(plot_file)
    plt.show()
    
    print(f"Training completed. Results saved to {plot_file}")
    print("Final Q-table statistics:")
    print(f" - Non-zero entries: {np.count_nonzero(Q)}")
    print(f" - Max Q-value: {np.max(Q)}")
    print(f" - Min Q-value: {np.min(Q)}")
    print(f" - Avg Q-value: {np.mean(Q[Q != 0])}")

if __name__ == "__main__":
    main()