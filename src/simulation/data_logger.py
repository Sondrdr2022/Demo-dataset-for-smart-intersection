"""
Data Logger Module

This module implements automatic data logging system for creating 
expanding training datasets as specified in requirements.
"""

import os
import csv
import time
import json
import pandas as pd
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, asdict
from datetime import datetime

from ..models.traffic_state import IntersectionState, LaneState, TrafficParameters
from ..utils.config_manager import config

@dataclass
class TrainingRecord:
    """
    Single training record with structure:
    episode | time | lane_id | state | action | reward | next_state | ambulance_detected
    """
    episode: int
    time: float
    lane_id: str
    state: List[float]  # Encoded state vector
    action: int
    reward: float
    next_state: List[float]  # Encoded next state vector
    ambulance_detected: bool
    intersection_id: str = ""
    approach_id: str = ""
    phase: int = 0
    lane_score: float = 0.0
    status_t: float = 0.0

@dataclass
class EpisodeMetrics:
    """Episode-level performance metrics"""
    episode: int
    start_time: float
    end_time: float
    duration: float
    total_vehicles: int
    avg_waiting_time: float
    total_waiting_time: float
    avg_queue_length: float
    max_queue_length: float
    emergency_events: int
    phase_changes: int
    timing_adjustments: int
    total_reward: float
    avg_reward: float

class DataLogger:
    """
    Automatic data logging system that logs data after each simulation run
    Creates self-expanding dataset for reinforcement learning training
    """
    
    def __init__(self, output_dir: str = None):
        self.output_dir = output_dir or config.training_logs_dir
        self.current_episode = 0
        self.episode_start_time = 0.0
        
        # Data collection
        self.training_records: List[TrainingRecord] = []
        self.episode_metrics: List[EpisodeMetrics] = []
        
        # File paths
        self.training_csv_path = os.path.join(self.output_dir, "training_data.csv")
        self.metrics_csv_path = os.path.join(self.output_dir, "episode_metrics.csv")
        self.episodes_json_path = os.path.join(self.output_dir, "episodes_detailed.json")
        
        # Session tracking
        self.current_session_data = {
            'episode': 0,
            'records': [],
            'metrics': {},
            'start_time': 0.0
        }
        
        # Ensure output directory exists
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Initialize CSV files if they don't exist
        self._initialize_csv_files()
        
    def start_episode(self, episode_number: int) -> bool:
        """Start logging for a new episode"""
        try:
            self.current_episode = episode_number
            self.episode_start_time = time.time()
            
            # Reset session data
            self.current_session_data = {
                'episode': episode_number,
                'records': [],
                'metrics': {},
                'start_time': self.episode_start_time,
                'intersection_states': {},
                'performance_data': {}
            }
            
            print(f"📊 Data logging started for episode {episode_number}")
            return True
            
        except Exception as e:
            print(f"❌ Error starting episode logging: {e}")
            return False
    
    def log_training_record(self, lane_id: str, state: List[float], action: int,
                          reward: float, next_state: List[float], 
                          ambulance_detected: bool, **kwargs) -> bool:
        """
        Log a single training record during simulation
        """
        try:
            record = TrainingRecord(
                episode=self.current_episode,
                time=time.time() - self.episode_start_time,
                lane_id=lane_id,
                state=state,
                action=action,
                reward=reward,
                next_state=next_state,
                ambulance_detected=ambulance_detected,
                intersection_id=kwargs.get('intersection_id', ''),
                approach_id=kwargs.get('approach_id', ''),
                phase=kwargs.get('phase', 0),
                lane_score=kwargs.get('lane_score', 0.0),
                status_t=kwargs.get('status_t', 0.0)
            )
            
            # Add to current session
            self.current_session_data['records'].append(record)
            
            return True
            
        except Exception as e:
            print(f"❌ Error logging training record: {e}")
            return False
    
    def log_intersection_state(self, intersection_id: str, intersection_state: IntersectionState):
        """Log complete intersection state for analysis"""
        try:
            state_data = {
                'timestamp': time.time() - self.episode_start_time,
                'current_phase': intersection_state.current_phase,
                'phase_duration': intersection_state.phase_duration,
                'cycle_count': intersection_state.cycle_count,
                'emergency_detected': intersection_state.emergency_detected,
                'emergency_approach': intersection_state.emergency_approach,
                'total_waiting_time': intersection_state.total_waiting_time,
                'total_vehicles_served': intersection_state.total_vehicles_served,
                'average_delay': intersection_state.average_delay,
                'approaches': {}
            }
            
            # Log approach-level data
            for approach_id, approach in intersection_state.approaches.items():
                approach_data = {
                    'direction': approach.direction,
                    'total_score': approach.total_score,
                    'avg_status': approach.avg_status,
                    'priority_level': approach.priority_level,
                    'has_left_turn': approach.has_left_turn,
                    'left_turn_blocked_time': approach.left_turn_blocked_time,
                    'lanes': {}
                }
                
                # Log lane-level data
                for lane_id, lane in approach.lanes.items():
                    lane_data = {
                        'lane_score': lane.lane_score,
                        'current_state': lane.current_state.value,
                        'state_duration': lane.state_duration,
                        'parameters': asdict(lane.parameters)
                    }
                    approach_data['lanes'][lane_id] = lane_data
                
                state_data['approaches'][approach_id] = approach_data
            
            # Store in session data
            if intersection_id not in self.current_session_data['intersection_states']:
                self.current_session_data['intersection_states'][intersection_id] = []
            
            self.current_session_data['intersection_states'][intersection_id].append(state_data)
            
        except Exception as e:
            print(f"❌ Error logging intersection state: {e}")
    
    def end_episode(self, final_metrics: Dict[str, Any] = None) -> bool:
        """
        End episode logging and save all data to files
        This implements the automatic logging after each SUMO simulation run
        """
        try:
            episode_end_time = time.time()
            episode_duration = episode_end_time - self.episode_start_time
            
            # Calculate episode metrics
            metrics = self._calculate_episode_metrics(episode_duration, final_metrics)
            
            # Save training records to CSV (appending to existing file)
            self._append_training_records_to_csv()
            
            # Save episode metrics
            self._append_episode_metrics_to_csv(metrics)
            
            # Save detailed episode data to JSON
            self._save_episode_details_to_json()
            
            # Update cumulative dataset
            self._update_cumulative_dataset()
            
            print(f"✅ Episode {self.current_episode} data logged successfully")
            print(f"   Duration: {episode_duration:.1f}s")
            print(f"   Training records: {len(self.current_session_data['records'])}")
            print(f"   Total reward: {metrics.total_reward:.2f}")
            
            return True
            
        except Exception as e:
            print(f"❌ Error ending episode logging: {e}")
            return False
    
    def _initialize_csv_files(self):
        """Initialize CSV files with headers if they don't exist"""
        # Training data CSV header
        training_header = [
            'episode', 'time', 'lane_id', 'state', 'action', 'reward', 
            'next_state', 'ambulance_detected', 'intersection_id', 
            'approach_id', 'phase', 'lane_score', 'status_t'
        ]
        
        if not os.path.exists(self.training_csv_path):
            with open(self.training_csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(training_header)
        
        # Episode metrics CSV header
        metrics_header = [
            'episode', 'start_time', 'end_time', 'duration', 'total_vehicles',
            'avg_waiting_time', 'total_waiting_time', 'avg_queue_length', 
            'max_queue_length', 'emergency_events', 'phase_changes',
            'timing_adjustments', 'total_reward', 'avg_reward'
        ]
        
        if not os.path.exists(self.metrics_csv_path):
            with open(self.metrics_csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(metrics_header)
    
    def _calculate_episode_metrics(self, duration: float, final_metrics: Dict[str, Any] = None) -> EpisodeMetrics:
        """Calculate comprehensive episode metrics"""
        records = self.current_session_data['records']
        
        # Basic calculations
        total_reward = sum(record.reward for record in records)
        avg_reward = total_reward / len(records) if records else 0.0
        
        # Extract metrics from final_metrics if provided
        if final_metrics:
            total_vehicles = final_metrics.get('total_vehicles', 0)
            avg_waiting_time = final_metrics.get('avg_waiting_time', 0.0)
            total_waiting_time = final_metrics.get('total_waiting_time', 0.0)
            avg_queue_length = final_metrics.get('avg_queue_length', 0.0)
            max_queue_length = final_metrics.get('max_queue_length', 0.0)
            emergency_events = final_metrics.get('emergency_events', 0)
            phase_changes = final_metrics.get('phase_changes', 0)
            timing_adjustments = final_metrics.get('timing_adjustments', 0)
        else:
            # Calculate from logged data
            total_vehicles = 0
            total_waiting_time = 0.0
            queue_lengths = []
            emergency_events = sum(1 for record in records if record.ambulance_detected)
            phase_changes = 0  # Would need to track this
            timing_adjustments = 0  # Would need to track this
            
            # Estimate from intersection states
            for intersection_states in self.current_session_data.get('intersection_states', {}).values():
                if intersection_states:
                    latest_state = intersection_states[-1]
                    for approach_data in latest_state.get('approaches', {}).values():
                        for lane_data in approach_data.get('lanes', {}).values():
                            params = lane_data.get('parameters', {})
                            total_vehicles += params.get('raw_count', 0)
                            total_waiting_time += params.get('td', 0.0) * params.get('raw_count', 0)
                            queue_lengths.append(params.get('l', 0.0))
            
            avg_waiting_time = total_waiting_time / max(total_vehicles, 1)
            avg_queue_length = sum(queue_lengths) / max(len(queue_lengths), 1)
            max_queue_length = max(queue_lengths) if queue_lengths else 0.0
        
        return EpisodeMetrics(
            episode=self.current_episode,
            start_time=self.episode_start_time,
            end_time=time.time(),
            duration=duration,
            total_vehicles=total_vehicles,
            avg_waiting_time=avg_waiting_time,
            total_waiting_time=total_waiting_time,
            avg_queue_length=avg_queue_length,
            max_queue_length=max_queue_length,
            emergency_events=emergency_events,
            phase_changes=phase_changes,
            timing_adjustments=timing_adjustments,
            total_reward=total_reward,
            avg_reward=avg_reward
        )
    
    def _append_training_records_to_csv(self):
        """Append training records to CSV file"""
        with open(self.training_csv_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            
            for record in self.current_session_data['records']:
                # Convert state and next_state lists to JSON strings
                state_str = json.dumps(record.state)
                next_state_str = json.dumps(record.next_state)
                
                row = [
                    record.episode, record.time, record.lane_id, state_str,
                    record.action, record.reward, next_state_str, 
                    record.ambulance_detected, record.intersection_id,
                    record.approach_id, record.phase, record.lane_score, 
                    record.status_t
                ]
                writer.writerow(row)
    
    def _append_episode_metrics_to_csv(self, metrics: EpisodeMetrics):
        """Append episode metrics to CSV file"""
        with open(self.metrics_csv_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            
            row = [
                metrics.episode, metrics.start_time, metrics.end_time,
                metrics.duration, metrics.total_vehicles, metrics.avg_waiting_time,
                metrics.total_waiting_time, metrics.avg_queue_length,
                metrics.max_queue_length, metrics.emergency_events,
                metrics.phase_changes, metrics.timing_adjustments,
                metrics.total_reward, metrics.avg_reward
            ]
            writer.writerow(row)
    
    def _save_episode_details_to_json(self):
        """Save detailed episode data to JSON file"""
        episode_data = {
            'episode': self.current_episode,
            'timestamp': datetime.now().isoformat(),
            'duration': time.time() - self.episode_start_time,
            'training_records_count': len(self.current_session_data['records']),
            'intersection_states': self.current_session_data['intersection_states'],
            'performance_data': self.current_session_data.get('performance_data', {})
        }
        
        # Load existing data
        episodes_data = []
        if os.path.exists(self.episodes_json_path):
            try:
                with open(self.episodes_json_path, 'r', encoding='utf-8') as f:
                    episodes_data = json.load(f)
            except:
                episodes_data = []
        
        # Append new episode
        episodes_data.append(episode_data)
        
        # Keep only last 100 episodes to manage file size
        if len(episodes_data) > 100:
            episodes_data = episodes_data[-100:]
        
        # Save back to file
        with open(self.episodes_json_path, 'w', encoding='utf-8') as f:
            json.dump(episodes_data, f, indent=2)
    
    def _update_cumulative_dataset(self):
        """Update cumulative dataset statistics"""
        try:
            # Create/update dataset summary
            summary_path = os.path.join(self.output_dir, "dataset_summary.json")
            
            summary = {
                'last_updated': datetime.now().isoformat(),
                'total_episodes': self.current_episode,
                'total_training_records': self._count_total_training_records(),
                'dataset_files': {
                    'training_data': self.training_csv_path,
                    'episode_metrics': self.metrics_csv_path,
                    'detailed_episodes': self.episodes_json_path
                },
                'latest_episode_metrics': asdict(self.episode_metrics[-1]) if self.episode_metrics else {}
            }
            
            with open(summary_path, 'w', encoding='utf-8') as f:
                json.dump(summary, f, indent=2)
                
        except Exception as e:
            print(f"Warning: Error updating cumulative dataset: {e}")
    
    def _count_total_training_records(self) -> int:
        """Count total training records in CSV file"""
        try:
            if os.path.exists(self.training_csv_path):
                with open(self.training_csv_path, 'r', encoding='utf-8') as f:
                    return sum(1 for line in f) - 1  # Subtract header
            return 0
        except:
            return 0
    
    def load_training_data(self, episodes: Optional[List[int]] = None) -> pd.DataFrame:
        """
        Load training data from CSV file
        This is used by RL agents to reload data for training
        """
        try:
            if not os.path.exists(self.training_csv_path):
                return pd.DataFrame()
            
            df = pd.read_csv(self.training_csv_path)
            
            # Filter by episodes if specified
            if episodes:
                df = df[df['episode'].isin(episodes)]
            
            # Parse JSON strings back to lists
            df['state'] = df['state'].apply(lambda x: json.loads(x) if isinstance(x, str) else x)
            df['next_state'] = df['next_state'].apply(lambda x: json.loads(x) if isinstance(x, str) else x)
            
            return df
            
        except Exception as e:
            print(f"❌ Error loading training data: {e}")
            return pd.DataFrame()
    
    def load_episode_metrics(self, episodes: Optional[List[int]] = None) -> pd.DataFrame:
        """Load episode metrics from CSV file"""
        try:
            if not os.path.exists(self.metrics_csv_path):
                return pd.DataFrame()
            
            df = pd.read_csv(self.metrics_csv_path)
            
            # Filter by episodes if specified
            if episodes:
                df = df[df['episode'].isin(episodes)]
            
            return df
            
        except Exception as e:
            print(f"❌ Error loading episode metrics: {e}")
            return pd.DataFrame()
    
    def get_dataset_info(self) -> Dict[str, Any]:
        """Get comprehensive dataset information"""
        info = {
            'total_episodes': 0,
            'total_training_records': 0,
            'latest_episode': 0,
            'dataset_size_mb': 0.0,
            'files': {}
        }
        
        try:
            # Count episodes and records
            if os.path.exists(self.metrics_csv_path):
                metrics_df = pd.read_csv(self.metrics_csv_path)
                info['total_episodes'] = len(metrics_df)
                if not metrics_df.empty:
                    info['latest_episode'] = metrics_df['episode'].max()
            
            info['total_training_records'] = self._count_total_training_records()
            
            # File sizes
            for file_path in [self.training_csv_path, self.metrics_csv_path, self.episodes_json_path]:
                if os.path.exists(file_path):
                    size_mb = os.path.getsize(file_path) / (1024 * 1024)
                    info['files'][os.path.basename(file_path)] = f"{size_mb:.2f} MB"
                    info['dataset_size_mb'] += size_mb
            
            info['dataset_size_mb'] = round(info['dataset_size_mb'], 2)
            
        except Exception as e:
            print(f"Warning: Error getting dataset info: {e}")
        
        return info