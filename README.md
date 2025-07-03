# Smart Traffic Intersection System with Dynamic Traffic Light Control

A comprehensive intelligent traffic intersection model with adaptive traffic light system based on dynamic parameters, chain scoring system, and special priority weights combined with Reinforcement Learning that learns directly from simulation.

## 🚦 Project Overview

This system implements an advanced traffic management solution featuring:

- **Dynamic Traffic Light Controller**: Self-adjusting green/red light timing based on real-time traffic conditions
- **Lane State Management**: Real-time updates of lane conditions and dynamic adjustments  
- **Reinforcement Learning Integration**: RL agent that learns from each simulation run
- **Data Logging System**: Automatic logging after each simulation to create expanding training dataset
- **Priority Vehicle Handling**: Special handling for ambulances and emergency vehicles
- **Smart Left Turn Priority**: Intelligent left turn management

## 📁 Project Structure

```
smart-traffic-intersection/
├── src/
│   ├── traffic_controller/
│   │   ├── adaptive_controller.py     # Dynamic traffic light controller
│   │   ├── scoring_system.py         # Lane scoring and status evaluation
│   │   └── priority_handler.py       # Emergency and left turn priority
│   ├── rl_agent/
│   │   ├── q_learning_agent.py       # Q-learning implementation
│   │   └── training_manager.py       # Automatic training pipeline
│   ├── simulation/
│   │   ├── sumo_interface.py         # SUMO TraCI integration
│   │   ├── parameter_monitor.py      # Real-time traffic monitoring
│   │   └── data_logger.py            # Automatic data logging
│   ├── models/
│   │   └── traffic_state.py          # Traffic state representations
│   └── utils/
│       └── config_manager.py         # Configuration management
├── data/
│   ├── sumo_configs/                 # SUMO simulation files
│   ├── training_logs/                # Training data logs
│   └── models/                       # Saved RL models
├── tests/
│   └── test_smart_traffic.py         # Comprehensive test suite
├── smart_traffic_main.py             # Main system entry point
├── requirements.txt                  # Python dependencies
└── README.md                         # This file
```

## 🔧 Installation and Setup

### Prerequisites

1. **SUMO Traffic Simulator**: Install from [SUMO website](https://sumo.dlr.de/)
2. **Python 3.8+**: Required for the system
3. **Required Python packages**: Listed in requirements.txt

### Installation Steps

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd Demo-dataset-for-smart-intersection
   ```

2. **Install Python dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Set SUMO_HOME environment variable**:
   ```bash
   export SUMO_HOME="/path/to/sumo"  # Linux/Mac
   set SUMO_HOME="C:\path\to\sumo"   # Windows
   ```

4. **Run setup script**:
   ```bash
   chmod +x setup.sh
   ./setup.sh
   ```

## 🚀 Usage

### Basic Usage

```python
from smart_traffic_main import SmartTrafficIntersectionSystem

# Initialize the system
system = SmartTrafficIntersectionSystem(use_gui=True)

# Start the system
system.start_system()

# Run continuous simulation with learning
system.run_continuous_simulation(num_episodes=10)

# Stop the system
system.stop_system()
```

### Command Line Usage

```bash
# Run the complete system
python smart_traffic_main.py

# Run tests
python tests/test_smart_traffic.py
```

## 🧠 Core Features

### 1. Dynamic Traffic Light Controller

- **Self-adjusting timing**: Green/red light timing adapts based on real-time conditions
- **Phase management**: Safe phase transitions with yellow buffers
- **Maximum waiting constraint**: Enforces 2-minute maximum waiting time rule
- **Performance tracking**: Monitors all control decisions and adjustments

### 2. Comprehensive Parameter Tracking

The system tracks all required parameters:
- `t`: Current time calculation
- `l`: Queue length per lane  
- `tđ`: Average waiting time
- `c`: Current light cycle
- `m`: Vehicle density
- `v`: Average speed
- `g`: Vehicle flow through intersection
- `ambulance_detected`: Priority vehicle detection flag

### 3. Chain Scoring System

**Lane Score Tracking**: Each lane maintains a `lane_score` variable with continuous rewards:
- **Smooth traffic (GOOD)**: +1 to +5 points based on performance
- **Congested traffic (BAD)**: -1 to -5 points based on severity  
- **State changes GOOD↔BAD**: Maintain score and continue with new state

**Status Function**: `Status_t = f(l, tđ, m, v, g)` compared against configurable thresholds

### 4. Dynamic Time Distribution

- **Cycle-based Adjustments**: After each cycle, aggregates lane scores
- **Adaptive Timing**: Increases green time for low-score (congested) lanes
- **Proportional Reduction**: Slightly reduces time for smooth-flowing lanes
- **Percentage-based Adjustment**: Modifications based on total score ratios

### 5. Priority Mechanisms

#### Emergency Vehicle Priority
- **Absolute Priority**: When `ambulance_detected = True`, maintains green light until vehicle completely passes
- **System Override**: Temporarily suspends normal adjustments for emergency priority
- **Complete Passage Tracking**: Monitors vehicles through intersection

#### Smart Left Turn Priority  
- **Conditional Green**: If left turn blocked by other directions with no vehicles, automatically switches left turn to green
- **Flexible Control**: Reduces left turn waiting time through intelligent management

### 6. Reinforcement Learning Integration

#### Automatic Data Logging
- **Post-simulation Logging**: After each SUMO simulation run, logs data to CSV/XML files
- **Data Structure**: 
  ```
  | episode | time | lane_id | state | action | reward | next_state | ambulance_detected |
  ```
- **Cumulative Dataset**: Continuously appends to existing files for self-expanding dataset

#### Direct Agent Training
- **Post-logging Training**: After each data logging session:
  - Agent reloads entire log file
  - Uses Q-learning to update policy  
  - Updates control model for next simulation iteration
- **Automatic Retraining**: Background training thread processes new data

## 📊 Configuration

The system uses a comprehensive configuration system in `src/utils/config_manager.py`:

```python
# Traffic Parameters
config.traffic.min_green_time = 15        # Minimum green phase
config.traffic.max_green_time = 120       # Maximum green phase
config.traffic.status_threshold = 0.4     # Normal congestion threshold
config.traffic.critical_threshold = 0.8   # Critical congestion threshold
config.traffic.max_waiting_time = 120     # 2-minute maximum waiting

# Scoring Configuration  
config.scoring.smooth_traffic_max = 5     # Maximum reward for smooth traffic
config.scoring.congested_traffic_min = -5 # Maximum penalty for congestion

# RL Configuration
config.rl.alpha = 0.2                     # Learning rate
config.rl.gamma = 0.95                    # Discount factor
config.rl.epsilon = 0.3                   # Exploration rate

# Priority Configuration
config.priority.emergency_vehicle_types = ["emergency", "ambulance", "fire", "police"]
config.priority.left_turn_max_wait = 60   # Maximum left turn wait time
```

## 📈 Performance Metrics

The system tracks comprehensive performance metrics:

- **Traffic Flow Efficiency**: Average waiting time, queue lengths, vehicle throughput
- **Emergency Response**: Emergency events handled, response times
- **Learning Progress**: RL agent performance, training episodes, Q-value evolution
- **Control Effectiveness**: Phase changes, timing adjustments, priority activations

## 🧪 Testing

Run the comprehensive test suite:

```bash
python tests/test_smart_traffic.py
```

Tests cover:
- Traffic state models and parameter tracking
- Scoring system functionality  
- Q-learning agent behavior
- Data logging and persistence
- Emergency priority handling
- System integration

## 📋 Example Output

```
🚦 Smart Traffic Intersection System with Dynamic Control and RL
======================================================================

🚀 Starting Smart Traffic Intersection System...
✅ All systems started successfully

📊 System Status:
   SUMO simulation: ✓ Running
   Parameter monitoring: ✓ Active  
   Adaptive control: ✓ Active
   RL training: ✓ Ready
   Priority handling: ✓ Ready

🎯 Episode 1 - Smart Traffic Intersection Simulation
🚨 EMERGENCY OVERRIDE: tl1 approach north_approach  
⚡ Phase change: tl1 0 → 2 (priority: south_approach)
⏱️  Timing adjustment: tl1 south_approach 30s → 38s (+8s)
📈 Q-Learning training completed:
   Updates: 156
   Avg reward: 2.34
   Training time: 0.15s

✅ Episode 1 completed:
   Duration: 245.3s
   Steps: 2453
   Vehicles served: 234
   Emergency events: 3
   Average waiting time: 28.4s
```

## 🔬 Technical Implementation

### Architecture Highlights

- **Modular Design**: Separate components for scoring, control, and learning
- **Real-time Processing**: Continuous monitoring and adaptive responses
- **Safety First**: Yellow transitions and minimum phase durations
- **Priority Hierarchy**: Emergency > Left Turn > Normal traffic flow
- **Data Persistence**: Automatic logging and model saving

### Key Algorithms

1. **Lane Scoring**: `Lane_Score = w1·Q + w2·W + w3·D + w4·F`
2. **Status Function**: `Status_t = f(l, tđ, m, v, g)`  
3. **Q-Learning Update**: `Q(s,a) = Q(s,a) + α[r + γ·max(Q(s',a')) - Q(s,a)]`
4. **Dynamic Timing**: Proportional adjustment based on score ratios

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/improvement`)
3. Commit changes (`git commit -am 'Add new feature'`)
4. Push to branch (`git push origin feature/improvement`)  
5. Create a Pull Request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🙏 Acknowledgments

- SUMO Traffic Simulation Suite
- TensorFlow/PyTorch for RL frameworks
- The traffic engineering community for domain expertise

## 📞 Support

For questions, issues, or contributions:
- Create an issue in the repository
- Contact the development team
- Check the documentation and test suite for examples

---

**Smart Traffic Intersection System** - Advancing urban traffic management through intelligent control and machine learning.