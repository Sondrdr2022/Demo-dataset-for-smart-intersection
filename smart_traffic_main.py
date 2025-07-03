"""
Main Smart Traffic Intersection System

This module integrates all components to create the complete smart traffic
intersection system with dynamic control, RL learning, and priority handling.
"""

import time
import sys
import os
from typing import Dict, List, Optional, Any
import signal

# Add project root to path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from src.simulation.sumo_interface import SUMOInterface
from src.simulation.parameter_monitor import ParameterMonitor
from src.simulation.data_logger import DataLogger
from src.traffic_controller.adaptive_controller import AdaptiveController
from src.traffic_controller.priority_handler import PriorityManager
from src.rl_agent.training_manager import TrainingManager
from src.utils.config_manager import config

class SmartTrafficIntersectionSystem:
    """
    Main system class that orchestrates the complete smart traffic intersection
    with dynamic control, reinforcement learning, and priority handling
    """
    
    def __init__(self, config_file: str = None, use_gui: bool = True):
        print("🚦 Initializing Smart Traffic Intersection System...")
        
        # Core components
        self.sumo = SUMOInterface(config_file, use_gui)
        self.parameter_monitor = ParameterMonitor(self.sumo)
        self.data_logger = DataLogger()
        
        # Control components
        self.adaptive_controller = AdaptiveController(self.sumo, self.parameter_monitor)
        self.priority_manager = PriorityManager(self.sumo)
        
        # RL components
        self.training_manager = TrainingManager(self.data_logger, self.parameter_monitor)
        
        # System state
        self.system_running = False
        self.current_episode = 0
        self.simulation_start_time = 0.0
        self.total_simulation_time = 0.0
        
        # Performance tracking
        self.system_metrics = {
            'episodes_completed': 0,
            'total_vehicles_served': 0,
            'total_emergency_events': 0,
            'avg_waiting_time': 0.0,
            'control_adjustments': 0,
            'rl_training_sessions': 0
        }
        
        # Set up signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        print("✅ Smart Traffic Intersection System initialized")
    
    def start_system(self) -> bool:
        """
        Start the complete smart traffic intersection system
        """
        try:
            print("\n🚀 Starting Smart Traffic Intersection System...")
            
            # 1. Start SUMO simulation
            if not self.sumo.start_simulation():
                return False
            
            # 2. Start parameter monitoring
            if not self.parameter_monitor.start_monitoring():
                return False
            
            # 3. Start adaptive traffic control
            if not self.adaptive_controller.start_control():
                return False
            
            self.system_running = True
            self.simulation_start_time = time.time()
            
            print("✅ All systems started successfully")
            print("\n📊 System Status:")
            print(f"   SUMO simulation: ✓ Running")
            print(f"   Parameter monitoring: ✓ Active")
            print(f"   Adaptive control: ✓ Active")
            print(f"   RL training: ✓ Ready")
            print(f"   Priority handling: ✓ Ready")
            
            return True
            
        except Exception as e:
            print(f"❌ Error starting system: {e}")
            return False
    
    def run_episode(self, episode_number: int, max_steps: int = 5000) -> Dict[str, Any]:
        """
        Run a complete episode with all smart traffic features
        """
        if not self.system_running:
            return {'success': False, 'reason': 'System not running'}
        
        self.current_episode = episode_number
        episode_start_time = time.time()
        
        print(f"\n🎯 Episode {episode_number} - Smart Traffic Intersection Simulation")
        
        try:
            # Start episode logging and RL training
            if not self.training_manager.start_training_episode(episode_number):
                return {'success': False, 'reason': 'Failed to start episode training'}
            
            # Episode statistics
            episode_stats = {
                'steps': 0,
                'vehicles_served': 0,
                'emergency_events': 0,
                'phase_changes': 0,
                'timing_adjustments': 0,
                'priority_activations': 0,
                'total_waiting_time': 0.0,
                'avg_waiting_time': 0.0,
                'max_queue_length': 0.0
            }
            
            # Main simulation loop
            step = 0
            while step < max_steps and self.system_running:
                
                # Execute SUMO simulation step
                if not self.sumo.step():
                    print("📋 Simulation ended naturally")
                    break
                
                # Update all traffic parameters
                self.parameter_monitor.update_all_parameters()
                
                # Process all intersections
                for intersection_id, intersection in self.parameter_monitor.intersection_states.items():
                    
                    # 1. Check for priority situations (emergency vehicles, left turns)
                    priority_result = self.priority_manager.evaluate_all_priorities(intersection)
                    
                    if priority_result['priority_active']:
                        episode_stats['priority_activations'] += 1
                        
                        if priority_result['override_normal_control']:
                            # Emergency override - skip normal control
                            continue
                    
                    # 2. Get RL actions for intersection
                    rl_actions = self.training_manager.process_rl_step(intersection)
                    
                    # 3. Apply adaptive traffic control
                    self.adaptive_controller.update_control()
                    
                    # 4. Update episode statistics
                    self._update_episode_stats(episode_stats, intersection)
                
                step += 1
                
                # Progress reporting
                if step % 500 == 0:
                    elapsed_time = time.time() - episode_start_time
                    vehicles_count = self.sumo.get_vehicle_count()
                    print(f"   Step {step}: {vehicles_count} vehicles, {elapsed_time:.1f}s elapsed")
            
            # Episode completion
            episode_duration = time.time() - episode_start_time
            episode_stats['steps'] = step
            episode_stats['duration'] = episode_duration
            
            # Calculate final metrics
            final_metrics = self._calculate_final_episode_metrics(episode_stats)
            
            # End episode training and trigger automatic RL training
            self.training_manager.end_training_episode(final_metrics)
            
            # Update system metrics
            self._update_system_metrics(final_metrics)
            
            print(f"✅ Episode {episode_number} completed:")
            print(f"   Duration: {episode_duration:.1f}s")
            print(f"   Steps: {step}")
            print(f"   Vehicles served: {final_metrics['total_vehicles']}")
            print(f"   Emergency events: {final_metrics['emergency_events']}")
            print(f"   Average waiting time: {final_metrics['avg_waiting_time']:.2f}s")
            
            return {
                'success': True,
                'episode': episode_number,
                'metrics': final_metrics,
                'stats': episode_stats
            }
            
        except Exception as e:
            print(f"❌ Error in episode {episode_number}: {e}")
            return {'success': False, 'reason': str(e)}
    
    def run_continuous_simulation(self, num_episodes: int = 10, max_steps_per_episode: int = 5000):
        """
        Run continuous simulation with multiple episodes for RL learning
        """
        print(f"\n🔄 Starting continuous simulation: {num_episodes} episodes")
        
        episode_results = []
        
        for episode in range(1, num_episodes + 1):
            print(f"\n{'='*60}")
            
            # Run episode
            result = self.run_episode(episode, max_steps_per_episode)
            episode_results.append(result)
            
            if not result['success']:
                print(f"❌ Episode {episode} failed: {result.get('reason', 'Unknown error')}")
                break
            
            # Show progress
            progress = episode / num_episodes * 100
            print(f"📈 Progress: {progress:.1f}% ({episode}/{num_episodes} episodes)")
            
            # Brief pause between episodes
            time.sleep(1)
        
        # Summary
        successful_episodes = [r for r in episode_results if r['success']]
        
        print(f"\n{'='*60}")
        print("🏁 CONTINUOUS SIMULATION COMPLETED")
        print(f"   Total episodes: {num_episodes}")
        print(f"   Successful episodes: {len(successful_episodes)}")
        print(f"   Success rate: {len(successful_episodes)/num_episodes*100:.1f}%")
        
        if successful_episodes:
            avg_waiting_time = sum(r['metrics']['avg_waiting_time'] for r in successful_episodes) / len(successful_episodes)
            total_vehicles = sum(r['metrics']['total_vehicles'] for r in successful_episodes)
            total_emergencies = sum(r['metrics']['emergency_events'] for r in successful_episodes)
            
            print(f"   Average waiting time: {avg_waiting_time:.2f}s")
            print(f"   Total vehicles served: {total_vehicles}")
            print(f"   Total emergency events: {total_emergencies}")
        
        # Show RL training progress
        training_status = self.training_manager.get_training_status()
        print(f"\n🧠 RL TRAINING PROGRESS:")
        print(f"   Training sessions: {training_status['total_training_sessions']}")
        print(f"   Agent epsilon: {training_status['agent_summary']['current_epsilon']:.3f}")
        print(f"   Training steps: {training_status['agent_summary']['total_training_steps']}")
        print(f"   Recent avg reward: {training_status['agent_summary']['recent_avg_reward']:.3f}")
        
        return episode_results
    
    def _update_episode_stats(self, stats: Dict[str, Any], intersection: Any):
        """Update episode statistics from intersection state"""
        try:
            # Vehicle count
            stats['vehicles_served'] = max(stats['vehicles_served'], intersection.total_vehicles_served)
            
            # Emergency events
            if intersection.emergency_detected:
                stats['emergency_events'] += 1
            
            # Waiting time
            stats['total_waiting_time'] = max(stats['total_waiting_time'], intersection.total_waiting_time)
            
            # Queue lengths
            for approach in intersection.approaches.values():
                for lane in approach.lanes.values():
                    stats['max_queue_length'] = max(stats['max_queue_length'], lane.parameters.l)
            
        except Exception as e:
            print(f"Warning: Error updating episode stats: {e}")
    
    def _calculate_final_episode_metrics(self, episode_stats: Dict[str, Any]) -> Dict[str, Any]:
        """Calculate final metrics for episode"""
        total_vehicles = episode_stats['vehicles_served']
        total_waiting = episode_stats['total_waiting_time']
        
        metrics = {
            'total_vehicles': total_vehicles,
            'total_waiting_time': total_waiting,
            'avg_waiting_time': total_waiting / max(total_vehicles, 1),
            'max_queue_length': episode_stats['max_queue_length'],
            'emergency_events': episode_stats['emergency_events'],
            'phase_changes': episode_stats['phase_changes'],
            'timing_adjustments': episode_stats['timing_adjustments'],
            'priority_activations': episode_stats['priority_activations'],
            'episode_duration': episode_stats.get('duration', 0.0),
            'simulation_steps': episode_stats['steps']
        }
        
        return metrics
    
    def _update_system_metrics(self, episode_metrics: Dict[str, Any]):
        """Update overall system performance metrics"""
        self.system_metrics['episodes_completed'] += 1
        self.system_metrics['total_vehicles_served'] += episode_metrics['total_vehicles']
        self.system_metrics['total_emergency_events'] += episode_metrics['emergency_events']
        
        # Update running average of waiting time
        prev_avg = self.system_metrics['avg_waiting_time']
        episodes = self.system_metrics['episodes_completed']
        new_avg = (prev_avg * (episodes - 1) + episode_metrics['avg_waiting_time']) / episodes
        self.system_metrics['avg_waiting_time'] = new_avg
        
        self.system_metrics['control_adjustments'] += episode_metrics['timing_adjustments']
        
        # Update RL training sessions count
        training_status = self.training_manager.get_training_status()
        self.system_metrics['rl_training_sessions'] = training_status['total_training_sessions']
    
    def get_system_status(self) -> Dict[str, Any]:
        """Get comprehensive system status"""
        status = {
            'system_running': self.system_running,
            'current_episode': self.current_episode,
            'uptime': time.time() - self.simulation_start_time if self.system_running else 0.0,
            'system_metrics': self.system_metrics.copy()
        }
        
        if self.system_running:
            status.update({
                'sumo_status': {'running': self.sumo.is_running, 'vehicles': self.sumo.get_vehicle_count()},
                'monitoring_status': self.parameter_monitor.get_performance_summary(),
                'control_status': self.adaptive_controller.get_control_status(),
                'priority_status': self.priority_manager.get_priority_status(),
                'training_status': self.training_manager.get_training_status()
            })
        
        return status
    
    def stop_system(self):
        """Gracefully stop the entire system"""
        if not self.system_running:
            return
        
        print("\n🛑 Stopping Smart Traffic Intersection System...")
        
        try:
            # Stop training manager
            self.training_manager.stop_training()
            
            # Stop adaptive controller
            self.adaptive_controller.stop_control()
            
            # Stop parameter monitoring
            self.parameter_monitor.stop_monitoring()
            
            # Stop SUMO simulation
            self.sumo.stop_simulation()
            
            self.system_running = False
            self.total_simulation_time += time.time() - self.simulation_start_time
            
            print("✅ System stopped gracefully")
            print(f"📊 Final Statistics:")
            print(f"   Total runtime: {self.total_simulation_time:.1f}s")
            print(f"   Episodes completed: {self.system_metrics['episodes_completed']}")
            print(f"   Total vehicles served: {self.system_metrics['total_vehicles_served']}")
            print(f"   Emergency events handled: {self.system_metrics['total_emergency_events']}")
            print(f"   Average waiting time: {self.system_metrics['avg_waiting_time']:.2f}s")
            
        except Exception as e:
            print(f"❌ Error stopping system: {e}")
    
    def _signal_handler(self, signum, frame):
        """Handle system signals for graceful shutdown"""
        print(f"\n⚠️  Received signal {signum}, shutting down gracefully...")
        self.stop_system()
        sys.exit(0)

def main():
    """Main entry point for the smart traffic intersection system"""
    print("🚦 Smart Traffic Intersection System with Dynamic Control and RL")
    print("=" * 70)
    
    # Initialize system
    system = SmartTrafficIntersectionSystem(use_gui=False)  # Set to True for GUI
    
    try:
        # Start system
        if not system.start_system():
            print("❌ Failed to start system")
            return
        
        # Run continuous simulation for learning
        system.run_continuous_simulation(num_episodes=5, max_steps_per_episode=3000)
        
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
    finally:
        # Ensure clean shutdown
        system.stop_system()

if __name__ == "__main__":
    main()