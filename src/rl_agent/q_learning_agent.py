"""
Q-Learning Agent Module

This module implements Q-Learning agent with automatic retraining
and policy updates as specified in requirements.
"""

import numpy as np
import pandas as pd
import pickle
import os
import time
from typing import Dict, List, Tuple, Optional, Any
from collections import defaultdict, deque
import random

from ..utils.config_manager import config
from ..simulation.data_logger import DataLogger

class QLearningAgent:
    """
    Q-Learning agent that learns from traffic intersection simulation data
    Implements automatic reloading of log files and policy updates
    """
    
    def __init__(self, state_space_dims: Tuple[int, ...] = None, n_actions: int = None):
        # Configuration
        self.state_space_dims = state_space_dims or config.rl.state_space_dims
        self.n_actions = n_actions or config.rl.n_actions
        self.alpha = config.rl.alpha
        self.gamma = config.rl.gamma
        self.epsilon = config.rl.epsilon
        self.epsilon_decay = config.rl.epsilon_decay
        self.epsilon_min = config.rl.epsilon_min
        
        # Q-table initialization
        self.q_table = np.zeros(self.state_space_dims + (self.n_actions,))
        
        # Experience replay buffer
        self.experience_buffer = deque(maxlen=10000)
        
        # Training tracking
        self.training_episodes = 0
        self.total_training_steps = 0
        self.last_training_time = 0.0
        
        # Performance metrics
        self.episode_rewards = []
        self.episode_q_values = []
        self.training_losses = []
        
        # Model persistence
        self.model_save_path = config.get_model_filename("qtable")
        
        # State encoding/decoding
        self.state_encoder = StateEncoder(self.state_space_dims)
        
        # Load existing model if available
        self._load_model()
    
    def get_action(self, state: np.ndarray, training: bool = True) -> int:
        """
        Get action using epsilon-greedy policy
        """
        if training and np.random.random() < self.epsilon:
            return np.random.randint(self.n_actions)
        
        # Encode state to discrete indices
        state_indices = self.state_encoder.encode_state(state)
        
        # Get Q-values for this state
        q_values = self.q_table[state_indices]
        
        # Return action with highest Q-value
        return np.argmax(q_values)
    
    def update_q_value(self, state: np.ndarray, action: int, reward: float, 
                      next_state: np.ndarray, done: bool = False):
        """
        Update Q-value using Q-learning update rule
        Q(s,a) = Q(s,a) + α[r + γ·max(Q(s',a')) - Q(s,a)]
        """
        # Encode states
        state_indices = self.state_encoder.encode_state(state)
        next_state_indices = self.state_encoder.encode_state(next_state)
        
        # Current Q-value
        current_q = self.q_table[state_indices + (action,)]
        
        # Next state max Q-value
        if done:
            next_max_q = 0.0
        else:
            next_max_q = np.max(self.q_table[next_state_indices])
        
        # Q-learning update
        target_q = reward + self.gamma * next_max_q
        new_q = current_q + self.alpha * (target_q - current_q)
        
        # Update Q-table
        self.q_table[state_indices + (action,)] = new_q
        
        # Store experience for analysis
        self.experience_buffer.append({
            'state': state.copy(),
            'action': action,
            'reward': reward,
            'next_state': next_state.copy(),
            'done': done,
            'q_value': new_q,
            'timestamp': time.time()
        })
        
        self.total_training_steps += 1
    
    def train_from_episode_data(self, episode_data: pd.DataFrame) -> Dict[str, Any]:
        """
        Train agent from complete episode data
        This implements the post-logging training pipeline
        """
        if episode_data.empty:
            return {'success': False, 'reason': 'No data provided'}
        
        training_start_time = time.time()
        initial_epsilon = self.epsilon
        
        # Group by lane for sequential training
        training_stats = {
            'total_updates': 0,
            'avg_reward': 0.0,
            'avg_q_value': 0.0,
            'lanes_processed': 0,
            'training_time': 0.0
        }
        
        total_reward = 0.0
        total_q_value = 0.0
        
        # Process data by lane to maintain temporal sequence
        for lane_id in episode_data['lane_id'].unique():
            lane_data = episode_data[episode_data['lane_id'] == lane_id].sort_values('time')
            
            # Train on sequential experiences
            for i in range(len(lane_data) - 1):
                current_row = lane_data.iloc[i]
                next_row = lane_data.iloc[i + 1]
                
                # Extract training data
                state = np.array(current_row['state'])
                action = int(current_row['action'])
                reward = float(current_row['reward'])
                next_state = np.array(next_row['state'])
                
                # Update Q-value
                self.update_q_value(state, action, reward, next_state)
                
                # Track statistics
                training_stats['total_updates'] += 1
                total_reward += reward
                
                # Get current Q-value for tracking
                state_indices = self.state_encoder.encode_state(state)
                q_value = self.q_table[state_indices + (action,)]
                total_q_value += q_value
            
            training_stats['lanes_processed'] += 1
        
        # Calculate averages
        if training_stats['total_updates'] > 0:
            training_stats['avg_reward'] = total_reward / training_stats['total_updates']
            training_stats['avg_q_value'] = total_q_value / training_stats['total_updates']
        
        # Update epsilon (exploration decay)
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
        
        # Training completion
        training_stats['training_time'] = time.time() - training_start_time
        training_stats['epsilon_change'] = initial_epsilon - self.epsilon
        training_stats['success'] = True
        
        self.training_episodes += 1
        self.last_training_time = time.time()
        
        # Store performance metrics
        self.episode_rewards.append(training_stats['avg_reward'])
        self.episode_q_values.append(training_stats['avg_q_value'])
        
        print(f"📈 Q-Learning training completed:")
        print(f"   Updates: {training_stats['total_updates']}")
        print(f"   Avg reward: {training_stats['avg_reward']:.3f}")
        print(f"   Avg Q-value: {training_stats['avg_q_value']:.3f}")
        print(f"   Epsilon: {self.epsilon:.3f}")
        print(f"   Training time: {training_stats['training_time']:.2f}s")
        
        return training_stats
    
    def reload_and_retrain(self, data_logger: DataLogger, 
                          episodes: Optional[List[int]] = None) -> Dict[str, Any]:
        """
        Reload entire log file and retrain agent
        This implements the automatic retraining system
        """
        print("🔄 Reloading training data and retraining agent...")
        
        try:
            # Load training data
            training_data = data_logger.load_training_data(episodes)
            
            if training_data.empty:
                return {'success': False, 'reason': 'No training data available'}
            
            # Reset Q-table for fresh training (optional - can be incremental)
            # self.q_table = np.zeros(self.state_space_dims + (self.n_actions,))
            
            # Train on all available data
            retraining_stats = {
                'total_episodes': len(training_data['episode'].unique()),
                'total_records': len(training_data),
                'episode_results': []
            }
            
            # Train episode by episode
            for episode in sorted(training_data['episode'].unique()):
                episode_data = training_data[training_data['episode'] == episode]
                episode_result = self.train_from_episode_data(episode_data)
                retraining_stats['episode_results'].append(episode_result)
            
            # Save updated model
            self._save_model()
            
            # Calculate overall statistics
            successful_episodes = [r for r in retraining_stats['episode_results'] if r.get('success', False)]
            
            if successful_episodes:
                retraining_stats['avg_episode_reward'] = np.mean([r['avg_reward'] for r in successful_episodes])
                retraining_stats['avg_episode_q_value'] = np.mean([r['avg_q_value'] for r in successful_episodes])
                retraining_stats['total_updates'] = sum(r['total_updates'] for r in successful_episodes)
            
            retraining_stats['success'] = len(successful_episodes) > 0
            
            print(f"✅ Agent retraining completed:")
            print(f"   Episodes processed: {retraining_stats['total_episodes']}")
            print(f"   Total records: {retraining_stats['total_records']}")
            print(f"   Successful episodes: {len(successful_episodes)}")
            
            return retraining_stats
            
        except Exception as e:
            print(f"❌ Error during retraining: {e}")
            return {'success': False, 'reason': str(e)}
    
    def _save_model(self):
        """Save Q-table and agent state"""
        try:
            model_data = {
                'q_table': self.q_table,
                'epsilon': self.epsilon,
                'training_episodes': self.training_episodes,
                'total_training_steps': self.total_training_steps,
                'episode_rewards': self.episode_rewards,
                'episode_q_values': self.episode_q_values,
                'state_space_dims': self.state_space_dims,
                'n_actions': self.n_actions,
                'timestamp': time.time()
            }
            
            with open(self.model_save_path, 'wb') as f:
                pickle.dump(model_data, f)
                
            print(f"💾 Model saved to {self.model_save_path}")
            
        except Exception as e:
            print(f"❌ Error saving model: {e}")
    
    def _load_model(self):
        """Load existing Q-table and agent state"""
        try:
            if os.path.exists(self.model_save_path):
                with open(self.model_save_path, 'rb') as f:
                    model_data = pickle.load(f)
                
                self.q_table = model_data['q_table']
                self.epsilon = model_data.get('epsilon', self.epsilon)
                self.training_episodes = model_data.get('training_episodes', 0)
                self.total_training_steps = model_data.get('total_training_steps', 0)
                self.episode_rewards = model_data.get('episode_rewards', [])
                self.episode_q_values = model_data.get('episode_q_values', [])
                
                print(f"📂 Model loaded from {self.model_save_path}")
                print(f"   Training episodes: {self.training_episodes}")
                print(f"   Total steps: {self.total_training_steps}")
                print(f"   Current epsilon: {self.epsilon:.3f}")
            
        except Exception as e:
            print(f"⚠️  Could not load existing model: {e}")
    
    def get_policy_summary(self) -> Dict[str, Any]:
        """Get summary of current policy"""
        return {
            'training_episodes': self.training_episodes,
            'total_training_steps': self.total_training_steps,
            'current_epsilon': self.epsilon,
            'q_table_shape': self.q_table.shape,
            'non_zero_q_values': np.count_nonzero(self.q_table),
            'max_q_value': np.max(self.q_table),
            'min_q_value': np.min(self.q_table),
            'avg_q_value': np.mean(self.q_table[self.q_table != 0]),
            'recent_avg_reward': np.mean(self.episode_rewards[-10:]) if self.episode_rewards else 0.0,
            'recent_avg_q_value': np.mean(self.episode_q_values[-10:]) if self.episode_q_values else 0.0
        }

class StateEncoder:
    """
    Encodes continuous states to discrete state space for Q-table
    """
    
    def __init__(self, state_space_dims: Tuple[int, ...]):
        self.state_space_dims = state_space_dims
        self.n_features = len(state_space_dims)
        
        # Define ranges for each state feature
        self.feature_ranges = {
            0: (0, 20),    # Queue length (0-20 vehicles)
            1: (0, 300),   # Waiting time (0-300 seconds)
            2: (0, 1),     # Density (0-1 normalized)
            3: (0, 50),    # Speed (0-50 m/s)
            4: (0, 1),     # Emergency flag (0-1)
        }
    
    def encode_state(self, state: np.ndarray) -> Tuple[int, ...]:
        """
        Encode continuous state vector to discrete indices
        """
        if len(state) != self.n_features:
            # Pad or truncate state to match expected dimensions
            if len(state) < self.n_features:
                state = np.pad(state, (0, self.n_features - len(state)), 'constant')
            else:
                state = state[:self.n_features]
        
        indices = []
        
        for i, value in enumerate(state):
            if i < len(self.state_space_dims):
                # Get range for this feature
                min_val, max_val = self.feature_ranges.get(i, (0, 1))
                
                # Normalize to [0, 1]
                normalized = (value - min_val) / (max_val - min_val)
                normalized = max(0, min(1, normalized))  # Clamp to [0, 1]
                
                # Discretize to state space dimension
                discrete_index = int(normalized * (self.state_space_dims[i] - 1))
                discrete_index = max(0, min(self.state_space_dims[i] - 1, discrete_index))
                
                indices.append(discrete_index)
        
        return tuple(indices)
    
    def decode_state(self, indices: Tuple[int, ...]) -> np.ndarray:
        """
        Decode discrete indices back to continuous state (for analysis)
        """
        state = np.zeros(len(indices))
        
        for i, index in enumerate(indices):
            if i < len(self.feature_ranges):
                min_val, max_val = self.feature_ranges[i]
                
                # Convert index back to normalized value
                normalized = index / (self.state_space_dims[i] - 1)
                
                # Convert back to original range
                state[i] = min_val + normalized * (max_val - min_val)
        
        return state

class RewardFunction:
    """
    Comprehensive reward function for traffic intersection control
    """
    
    def __init__(self):
        self.queue_weight = config.rl.queue_weight
        self.waiting_weight = config.rl.waiting_weight
        self.speed_weight = config.rl.speed_weight
        self.emergency_weight = config.rl.emergency_weight
        
    def calculate_reward(self, current_state: Dict[str, Any], 
                        action: int, next_state: Dict[str, Any]) -> float:
        """
        Calculate comprehensive reward based on traffic state improvements
        """
        # Base reward components
        queue_improvement = self._calculate_queue_reward(current_state, next_state)
        waiting_improvement = self._calculate_waiting_reward(current_state, next_state)
        speed_improvement = self._calculate_speed_reward(current_state, next_state)
        
        # Emergency vehicle priority reward
        emergency_reward = self._calculate_emergency_reward(current_state, next_state, action)
        
        # Total reward
        total_reward = (
            self.queue_weight * queue_improvement +
            self.waiting_weight * waiting_improvement +
            self.speed_weight * speed_improvement +
            self.emergency_weight * emergency_reward
        )
        
        return total_reward
    
    def _calculate_queue_reward(self, current_state: Dict[str, Any], 
                              next_state: Dict[str, Any]) -> float:
        """Calculate reward based on queue length changes"""
        current_queue = current_state.get('queue_length', 0)
        next_queue = next_state.get('queue_length', 0)
        
        # Reward for reducing queue length
        improvement = current_queue - next_queue
        return improvement  # Positive reward for queue reduction
    
    def _calculate_waiting_reward(self, current_state: Dict[str, Any], 
                                next_state: Dict[str, Any]) -> float:
        """Calculate reward based on waiting time changes"""
        current_waiting = current_state.get('waiting_time', 0)
        next_waiting = next_state.get('waiting_time', 0)
        
        # Reward for reducing waiting time
        improvement = current_waiting - next_waiting
        return improvement / 10.0  # Scale down waiting time reward
    
    def _calculate_speed_reward(self, current_state: Dict[str, Any], 
                              next_state: Dict[str, Any]) -> float:
        """Calculate reward based on speed improvements"""
        current_speed = current_state.get('avg_speed', 0)
        next_speed = next_state.get('avg_speed', 0)
        
        # Reward for increasing average speed
        improvement = next_speed - current_speed
        return improvement / 10.0  # Scale speed reward
    
    def _calculate_emergency_reward(self, current_state: Dict[str, Any], 
                                  next_state: Dict[str, Any], action: int) -> float:
        """Calculate reward for emergency vehicle handling"""
        current_emergency = current_state.get('ambulance_detected', False)
        next_emergency = next_state.get('ambulance_detected', False)
        
        if current_emergency:
            # High reward for maintaining emergency priority
            if action == 1:  # Assuming action 1 is "maintain green"
                return 10.0
            else:
                return -5.0  # Penalty for not maintaining emergency priority
        
        return 0.0  # No emergency vehicle present