"""
Smart Traffic Intersection System Configuration Manager

This module handles all configuration settings for the smart traffic intersection system
including SUMO paths, traffic parameters, RL settings, and thresholds.
"""

import os
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional

@dataclass
class SUMOConfig:
    """SUMO simulation configuration"""
    sumo_home: str
    config_file: str = "dataset.sumocfg"
    binary_name: str = "sumo-gui"
    step_length: float = 0.1
    
    def __post_init__(self):
        if not self.sumo_home:
            raise ValueError("SUMO_HOME environment variable must be set")
        
        self.binary_path = os.path.join(self.sumo_home, 'bin', self.binary_name)
        tools_path = os.path.join(self.sumo_home, 'tools')
        if tools_path not in sys.path:
            sys.path.append(tools_path)

@dataclass 
class TrafficParameters:
    """Traffic monitoring and control parameters"""
    # Timing constraints
    min_green_time: int = 15
    max_green_time: int = 120
    min_cycle_time: int = 60
    max_cycle_time: int = 200
    yellow_time: int = 3
    all_red_time: int = 2
    eval_interval: int = 2
    max_waiting_time: int = 120  # 2 minutes constraint
    
    # Status thresholds
    status_threshold: float = 0.4
    critical_threshold: float = 0.8
    
    # Parameter weights for Status_t = f(l, tđ, m, v, g)
    parameter_weights: Dict[str, float] = None
    
    def __post_init__(self):
        if self.parameter_weights is None:
            self.parameter_weights = {
                'l': 0.25,    # Queue length
                'td': 0.20,   # Average waiting time  
                'm': 0.20,    # Vehicle density
                'v': 0.15,    # Average speed
                'g': 0.20     # Vehicle flow
            }

@dataclass
class ScoringConfig:
    """Lane scoring system configuration"""
    # Continuous state rewards
    smooth_traffic_min: int = 1     # GOOD state minimum reward
    smooth_traffic_max: int = 5     # GOOD state maximum reward
    congested_traffic_min: int = -5  # BAD state minimum penalty
    congested_traffic_max: int = -1  # BAD state maximum penalty
    
    # Lane score weights: Lane_Score = w1·Q + w2·W + w3·D + w4·F
    lane_score_weights: Dict[str, float] = None
    
    def __post_init__(self):
        if self.lane_score_weights is None:
            self.lane_score_weights = {
                'w1': 0.3,  # Queue length weight
                'w2': 0.3,  # Waiting time weight  
                'w3': 0.2,  # Density weight
                'w4': 0.2   # Flow weight
            }

@dataclass
class RLConfig:
    """Reinforcement Learning configuration"""
    # Q-Learning parameters
    alpha: float = 0.2          # Learning rate
    gamma: float = 0.95         # Discount factor
    epsilon: float = 0.3        # Exploration rate
    epsilon_decay: float = 0.95 # Epsilon decay per episode
    epsilon_min: float = 0.05   # Minimum epsilon
    
    # Training parameters
    episodes: int = 100
    max_steps: int = 5000
    
    # State space dimensions (discretized)
    state_space_dims: tuple = (10, 10, 10, 2)  # (queue_ns, queue_ew, density, current_phase)
    n_actions: int = 2  # Number of possible actions
    
    # Reward function weights
    queue_weight: float = 2.0
    waiting_weight: float = 1.0
    speed_weight: float = 0.5
    emergency_weight: float = 10.0  # High priority for emergency vehicles

@dataclass
class PriorityConfig:
    """Priority vehicle and special handling configuration"""
    # Emergency vehicle settings
    emergency_vehicle_types: List[str] = None
    emergency_detection_range: float = 100.0  # Detection range in meters
    emergency_absolute_priority: bool = True  # Maintain green until completely passed
    
    # Left turn priority settings
    left_turn_smart_priority: bool = True
    left_turn_max_wait: int = 60  # Maximum wait time before forced green
    
    def __post_init__(self):
        if self.emergency_vehicle_types is None:
            self.emergency_vehicle_types = ["emergency", "ambulance", "fire", "police"]

class ConfigManager:
    """Main configuration manager for the smart traffic intersection system"""
    
    def __init__(self, config_file: Optional[str] = None):
        self.sumo = self._init_sumo_config()
        self.traffic = TrafficParameters()
        self.scoring = ScoringConfig()
        self.rl = RLConfig()
        self.priority = PriorityConfig()
        
        # Data paths
        self.data_dir = "data"
        self.training_logs_dir = os.path.join(self.data_dir, "training_logs")
        self.models_dir = os.path.join(self.data_dir, "models")
        self.sumo_configs_dir = os.path.join(self.data_dir, "sumo_configs")
        
        # Ensure directories exist
        self._create_directories()
    
    def _init_sumo_config(self) -> SUMOConfig:
        """Initialize SUMO configuration from environment"""
        sumo_home = os.environ.get('SUMO_HOME')
        if not sumo_home:
            # Try to find SUMO in common locations
            possible_paths = [
                '/usr/share/sumo',
                '/opt/sumo',
                '/usr/local/sumo',
                'C:/Program Files (x86)/Eclipse/Sumo',
                'C:/sumo'
            ]
            for path in possible_paths:
                if os.path.exists(path):
                    sumo_home = path
                    os.environ['SUMO_HOME'] = path
                    break
        
        if not sumo_home:
            print("Warning: SUMO_HOME not found. Using current directory config.")
            sumo_home = "."
            
        return SUMOConfig(sumo_home=sumo_home)
    
    def _create_directories(self):
        """Create necessary directories if they don't exist"""
        dirs = [
            self.data_dir,
            self.training_logs_dir, 
            self.models_dir,
            self.sumo_configs_dir
        ]
        
        for dir_path in dirs:
            os.makedirs(dir_path, exist_ok=True)
    
    def get_log_filename(self, episode: int) -> str:
        """Generate log filename for training data"""
        return os.path.join(self.training_logs_dir, f"training_episode_{episode:04d}.csv")
    
    def get_model_filename(self, model_type: str = "qtable") -> str:
        """Generate model filename"""
        return os.path.join(self.models_dir, f"{model_type}_latest.npy")

# Global configuration instance
config = ConfigManager()