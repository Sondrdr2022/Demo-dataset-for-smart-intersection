"""
Training Manager Module

This module orchestrates the complete training pipeline with automatic
post-simulation training and agent updates as specified in requirements.
"""

import time
import os
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass
import threading
import queue

from ..simulation.data_logger import DataLogger
from ..rl_agent.q_learning_agent import QLearningAgent, RewardFunction
from ..simulation.parameter_monitor import ParameterMonitor
from ..models.traffic_state import IntersectionState, LaneState
from ..utils.config_manager import config

@dataclass
class TrainingSession:
    """Represents a complete training session"""
    session_id: str
    start_time: float
    episodes_completed: int
    total_training_time: float
    agent_performance: Dict[str, Any]
    data_logged: bool
    training_successful: bool

class TrainingManager:
    """
    Main training manager implementing the complete RL training pipeline:
    1. Post-simulation logging
    2. Agent reloading of entire log file  
    3. Q-learning/Deep Q Network policy updates
    4. Control model updates for next simulation iteration
    """
    
    def __init__(self, data_logger: DataLogger, parameter_monitor: ParameterMonitor):
        self.data_logger = data_logger
        self.parameter_monitor = parameter_monitor
        
        # Initialize RL agent
        self.agent = QLearningAgent()
        self.reward_function = RewardFunction()
        
        # Training configuration
        self.auto_training_enabled = True
        self.training_after_episodes = 1  # Train after every episode
        self.min_training_data_threshold = 10  # Minimum records needed for training
        
        # Training state
        self.current_episode = 0
        self.training_sessions: List[TrainingSession] = []
        self.last_training_time = 0.0
        
        # Performance tracking
        self.episode_performances: List[Dict[str, Any]] = []
        self.training_queue = queue.Queue()
        self.training_thread = None
        self.training_active = False
        
        # State management for RL integration
        self.current_states: Dict[str, Any] = {}  # lane_id -> state
        self.last_actions: Dict[str, int] = {}    # lane_id -> action
        self.step_count = 0
        
    def start_training_episode(self, episode_number: int) -> bool:
        """
        Start a new training episode with data logging
        """
        try:
            self.current_episode = episode_number
            self.step_count = 0
            
            # Start data logging for this episode
            if not self.data_logger.start_episode(episode_number):
                return False
            
            # Reset RL state tracking
            self.current_states.clear()
            self.last_actions.clear()
            
            print(f"🎯 Training episode {episode_number} started")
            return True
            
        except Exception as e:
            print(f"❌ Error starting training episode: {e}")
            return False
    
    def process_rl_step(self, intersection_state: IntersectionState) -> Dict[str, int]:
        """
        Process one RL step: get actions from agent and log experience
        This integrates RL decisions with traffic controller
        """
        try:
            actions = {}
            
            # Process each lane in the intersection
            for approach_id, approach in intersection_state.approaches.items():
                for lane_id, lane_state in approach.lanes.items():
                    
                    # Get current state representation
                    current_state = self._encode_lane_state(lane_state, intersection_state)
                    
                    # Get action from RL agent
                    action = self.agent.get_action(current_state, training=True)
                    actions[lane_id] = action
                    
                    # If we have previous state, calculate reward and update Q-value
                    if lane_id in self.current_states and lane_id in self.last_actions:
                        reward = self._calculate_step_reward(
                            lane_id, self.current_states[lane_id], current_state, 
                            lane_state, intersection_state
                        )
                        
                        # Update Q-value
                        self.agent.update_q_value(
                            self.current_states[lane_id],
                            self.last_actions[lane_id],
                            reward,
                            current_state
                        )
                        
                        # Log training record
                        self.data_logger.log_training_record(
                            lane_id=lane_id,
                            state=self.current_states[lane_id].tolist(),
                            action=self.last_actions[lane_id],
                            reward=reward,
                            next_state=current_state.tolist(),
                            ambulance_detected=lane_state.parameters.ambulance_detected,
                            intersection_id=intersection_state.intersection_id,
                            approach_id=approach_id,
                            phase=intersection_state.current_phase,
                            lane_score=lane_state.lane_score,
                            status_t=self._calculate_status_t(lane_state)
                        )
                    
                    # Update state tracking
                    self.current_states[lane_id] = current_state
                    self.last_actions[lane_id] = action
            
            self.step_count += 1
            return actions
            
        except Exception as e:
            print(f"❌ Error in RL step processing: {e}")
            return {}
    
    def end_training_episode(self, final_metrics: Dict[str, Any] = None) -> bool:
        """
        End training episode and trigger automatic training pipeline
        """
        try:
            # Log final intersection state
            for intersection in self.parameter_monitor.intersection_states.values():
                self.data_logger.log_intersection_state(intersection.intersection_id, intersection)
            
            # End data logging
            if not self.data_logger.end_episode(final_metrics):
                return False
            
            # Trigger automatic training if enabled
            if self.auto_training_enabled:
                self._trigger_automatic_training()
            
            # Store episode performance
            episode_performance = self._calculate_episode_performance(final_metrics)
            self.episode_performances.append(episode_performance)
            
            print(f"✅ Training episode {self.current_episode} completed")
            return True
            
        except Exception as e:
            print(f"❌ Error ending training episode: {e}")
            return False
    
    def _trigger_automatic_training(self):
        """
        Trigger automatic training pipeline in background thread
        """
        if self.current_episode % self.training_after_episodes == 0:
            # Check if we have enough training data
            dataset_info = self.data_logger.get_dataset_info()
            
            if dataset_info['total_training_records'] >= self.min_training_data_threshold:
                print("🔄 Triggering automatic post-simulation training...")
                
                # Add training task to queue
                training_task = {
                    'type': 'full_retrain',
                    'episode': self.current_episode,
                    'timestamp': time.time()
                }
                self.training_queue.put(training_task)
                
                # Start training thread if not already running
                if not self.training_active:
                    self._start_training_thread()
            else:
                print(f"⏳ Insufficient training data ({dataset_info['total_training_records']} < {self.min_training_data_threshold})")
    
    def _start_training_thread(self):
        """Start background training thread"""
        if self.training_thread is None or not self.training_thread.is_alive():
            self.training_thread = threading.Thread(target=self._training_worker, daemon=True)
            self.training_active = True
            self.training_thread.start()
            print("🧵 Training thread started")
    
    def _training_worker(self):
        """
        Background worker for automatic training
        Implements: "After each data logging session, agent reloads entire log file,
        uses Q-learning to update policy, updates control model for next simulation"
        """
        while self.training_active:
            try:
                # Get training task (blocking with timeout)
                task = self.training_queue.get(timeout=5.0)
                
                if task['type'] == 'full_retrain':
                    self._execute_full_retraining(task)
                elif task['type'] == 'stop':
                    break
                
                self.training_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"❌ Error in training worker: {e}")
        
        self.training_active = False
        print("🧵 Training thread stopped")
    
    def _execute_full_retraining(self, task: Dict[str, Any]):
        """
        Execute full retraining of RL agent
        """
        session_id = f"session_{task['episode']}_{int(task['timestamp'])}"
        session_start_time = time.time()
        
        print(f"🎓 Starting full retraining session: {session_id}")
        
        try:
            # Step 1: Agent reloads entire log file
            retraining_result = self.agent.reload_and_retrain(self.data_logger)
            
            if retraining_result.get('success', False):
                # Step 2: Update control model for next simulation iteration
                self._update_control_model(retraining_result)
                
                # Step 3: Record training session
                session = TrainingSession(
                    session_id=session_id,
                    start_time=session_start_time,
                    episodes_completed=retraining_result.get('total_episodes', 0),
                    total_training_time=time.time() - session_start_time,
                    agent_performance=self.agent.get_policy_summary(),
                    data_logged=True,
                    training_successful=True
                )
                
                self.training_sessions.append(session)
                self.last_training_time = time.time()
                
                print(f"✅ Full retraining completed successfully")
                print(f"   Episodes: {retraining_result.get('total_episodes', 0)}")
                print(f"   Records: {retraining_result.get('total_records', 0)}")
                print(f"   Training time: {session.total_training_time:.2f}s")
                
            else:
                print(f"❌ Retraining failed: {retraining_result.get('reason', 'Unknown error')}")
                
        except Exception as e:
            print(f"❌ Error during full retraining: {e}")
    
    def _update_control_model(self, retraining_result: Dict[str, Any]):
        """
        Update control model parameters based on training results
        """
        try:
            # Update exploration rate based on training progress
            if retraining_result.get('total_episodes', 0) > 10:
                # Reduce exploration as we get more data
                self.agent.epsilon = max(
                    self.agent.epsilon_min,
                    self.agent.epsilon * 0.95
                )
            
            # Log control model updates
            print(f"🔧 Control model updated:")
            print(f"   New epsilon: {self.agent.epsilon:.3f}")
            print(f"   Q-table non-zero values: {retraining_result.get('total_updates', 0)}")
            
        except Exception as e:
            print(f"❌ Error updating control model: {e}")
    
    def _encode_lane_state(self, lane_state: LaneState, intersection_state: IntersectionState) -> np.ndarray:
        """
        Encode lane state into format suitable for RL agent
        """
        import numpy as np
        
        params = lane_state.parameters
        
        # Core state features: [queue_length, waiting_time, density, speed, emergency_flag]
        state = np.array([
            params.l,                    # Queue length
            params.td,                   # Waiting time
            params.m,                    # Density
            params.v,                    # Speed
            float(params.ambulance_detected)  # Emergency flag
        ])
        
        return state
    
    def _calculate_step_reward(self, lane_id: str, prev_state: np.ndarray, 
                             current_state: np.ndarray, lane_state: LaneState,
                             intersection_state: IntersectionState) -> float:
        """
        Calculate reward for RL step based on traffic improvements
        """
        # Convert states to dict format for reward function
        prev_state_dict = {
            'queue_length': prev_state[0],
            'waiting_time': prev_state[1],
            'density': prev_state[2],
            'avg_speed': prev_state[3],
            'ambulance_detected': bool(prev_state[4])
        }
        
        current_state_dict = {
            'queue_length': current_state[0],
            'waiting_time': current_state[1], 
            'density': current_state[2],
            'avg_speed': current_state[3],
            'ambulance_detected': bool(current_state[4])
        }
        
        # Get last action for this lane
        last_action = self.last_actions.get(lane_id, 0)
        
        # Calculate reward using reward function
        reward = self.reward_function.calculate_reward(
            prev_state_dict, last_action, current_state_dict
        )
        
        # Add lane score component
        score_reward = lane_state.lane_score * 0.1  # Scale lane score
        
        return reward + score_reward
    
    def _calculate_status_t(self, lane_state: LaneState) -> float:
        """Calculate status_t for logging"""
        params = lane_state.parameters
        
        # Simplified status calculation
        return min(
            (params.l / 20.0) * 0.25 +
            (params.td / 300.0) * 0.20 +
            params.m * 0.20 +
            (1 - params.v / 50.0) * 0.15 +
            (1 - params.g / 3600.0) * 0.20,
            1.0
        )
    
    def _calculate_episode_performance(self, final_metrics: Dict[str, Any] = None) -> Dict[str, Any]:
        """Calculate comprehensive episode performance metrics"""
        agent_summary = self.agent.get_policy_summary()
        
        performance = {
            'episode': self.current_episode,
            'timestamp': time.time(),
            'steps': self.step_count,
            'agent_epsilon': agent_summary['current_epsilon'],
            'avg_reward': agent_summary['recent_avg_reward'],
            'q_value_progress': agent_summary['recent_avg_q_value'],
            'training_steps': agent_summary['total_training_steps']
        }
        
        # Add final metrics if provided
        if final_metrics:
            performance.update({
                'total_vehicles': final_metrics.get('total_vehicles', 0),
                'avg_waiting_time': final_metrics.get('avg_waiting_time', 0.0),
                'emergency_events': final_metrics.get('emergency_events', 0)
            })
        
        return performance
    
    def get_rl_action_for_control(self, lane_state: LaneState, 
                                intersection_state: IntersectionState) -> int:
        """
        Get RL action recommendation for traffic control
        This integrates RL decisions with the main traffic controller
        """
        try:
            # Encode current state
            state = self._encode_lane_state(lane_state, intersection_state)
            
            # Get action from trained agent (no exploration in deployment)
            action = self.agent.get_action(state, training=False)
            
            return action
            
        except Exception as e:
            print(f"❌ Error getting RL action: {e}")
            return 0  # Default action
    
    def stop_training(self):
        """Stop automatic training system"""
        if self.training_active:
            # Signal training thread to stop
            self.training_queue.put({'type': 'stop'})
            
            # Wait for thread to finish
            if self.training_thread and self.training_thread.is_alive():
                self.training_thread.join(timeout=10.0)
            
            self.training_active = False
            print("🛑 Training system stopped")
    
    def get_training_status(self) -> Dict[str, Any]:
        """Get comprehensive training system status"""
        return {
            'current_episode': self.current_episode,
            'auto_training_enabled': self.auto_training_enabled,
            'training_active': self.training_active,
            'last_training_time': self.last_training_time,
            'total_training_sessions': len(self.training_sessions),
            'agent_summary': self.agent.get_policy_summary(),
            'dataset_info': self.data_logger.get_dataset_info(),
            'recent_episodes_performance': self.episode_performances[-5:] if self.episode_performances else [],
            'training_queue_size': self.training_queue.qsize()
        }
    
    def manual_training_trigger(self, episodes: Optional[List[int]] = None) -> Dict[str, Any]:
        """
        Manually trigger training on specific episodes
        """
        print("🎯 Manual training trigger activated")
        
        try:
            if episodes:
                # Train on specific episodes
                training_data = self.data_logger.load_training_data(episodes)
                if not training_data.empty:
                    result = self.agent.train_from_episode_data(training_data)
                    self.agent._save_model()
                    return result
                else:
                    return {'success': False, 'reason': 'No data for specified episodes'}
            else:
                # Full retraining
                result = self.agent.reload_and_retrain(self.data_logger)
                return result
                
        except Exception as e:
            return {'success': False, 'reason': str(e)}