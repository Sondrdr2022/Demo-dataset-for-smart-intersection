"""
Test suite for the Smart Traffic Intersection System

This module provides basic tests to validate system functionality
"""

import sys
import os
import unittest
from unittest.mock import Mock, patch
import numpy as np

# Add project root to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from src.utils.config_manager import config
from src.models.traffic_state import TrafficParameters, LaneState, ApproachState, IntersectionState
from src.traffic_controller.scoring_system import LaneScoringSystem, StatusFunction
from src.rl_agent.q_learning_agent import QLearningAgent, StateEncoder
from src.simulation.data_logger import DataLogger, TrainingRecord

class TestTrafficState(unittest.TestCase):
    """Test traffic state models"""
    
    def test_traffic_parameters_creation(self):
        """Test TrafficParameters creation and normalization"""
        params = TrafficParameters(l=5.0, td=30.0, m=0.3, v=25.0, g=1800.0)
        
        self.assertEqual(params.l, 5.0)
        self.assertEqual(params.td, 30.0)
        self.assertEqual(params.m, 0.3)
        self.assertEqual(params.v, 25.0)
        self.assertEqual(params.g, 1800.0)
        
        # Test normalization
        max_values = {'l': 20.0, 'td': 300.0, 'm': 1.0, 'v': 50.0, 'g': 3600.0}
        normalized = params.normalize_parameters(max_values)
        
        self.assertAlmostEqual(normalized['l'], 0.25)  # 5/20
        self.assertAlmostEqual(normalized['td'], 0.1)   # 30/300
        self.assertAlmostEqual(normalized['m'], 0.3)    # 0.3/1.0
        self.assertAlmostEqual(normalized['v'], 0.5)    # 1 - 25/50
        self.assertAlmostEqual(normalized['g'], 0.5)    # 1 - 1800/3600
    
    def test_lane_state_scoring(self):
        """Test lane state scoring functionality"""
        lane = LaneState(lane_id="test_lane")
        params = TrafficParameters(l=10.0, td=60.0, m=0.5, v=15.0)
        lane.update_parameters(params)
        
        self.assertEqual(lane.lane_id, "test_lane")
        self.assertEqual(lane.parameters.l, 10.0)

class TestScoringSystem(unittest.TestCase):
    """Test traffic scoring system"""
    
    def setUp(self):
        self.scoring_system = LaneScoringSystem()
        self.status_function = StatusFunction()
    
    def test_lane_scoring_calculation(self):
        """Test lane score calculation"""
        lane = LaneState(lane_id="test_lane")
        params = TrafficParameters(l=5.0, td=30.0, m=0.3, v=25.0, g=1800.0)
        lane.update_parameters(params)
        
        result = self.scoring_system.calculate_lane_score(lane)
        
        self.assertIsNotNone(result.score)
        self.assertIsNotNone(result.reward)
        self.assertIn('Q', result.components)
        self.assertIn('W', result.components)
        self.assertIn('D', result.components)
        self.assertIn('F', result.components)
    
    def test_status_function_calculation(self):
        """Test Status_t function calculation"""
        params = TrafficParameters(l=8.0, td=45.0, m=0.4, v=20.0, g=2400.0)
        max_values = {'l': 20.0, 'td': 300.0, 'm': 1.0, 'v': 50.0, 'g': 3600.0}
        
        status_t = self.status_function.calculate_status_t(params, max_values)
        
        self.assertGreaterEqual(status_t, 0.0)
        self.assertLessEqual(status_t, 1.0)
        
        # Test evaluation
        evaluation = self.status_function.evaluate_status(status_t)
        self.assertIn('level', evaluation)
        self.assertIn('priority', evaluation)
        self.assertIn('recommended_action', evaluation)

class TestQLearningAgent(unittest.TestCase):
    """Test Q-learning agent functionality"""
    
    def setUp(self):
        self.agent = QLearningAgent(state_space_dims=(5, 5, 5, 2), n_actions=2)
        self.state_encoder = StateEncoder((5, 5, 5, 2))
    
    def test_agent_initialization(self):
        """Test agent initialization"""
        self.assertEqual(self.agent.state_space_dims, (5, 5, 5, 2))
        self.assertEqual(self.agent.n_actions, 2)
        self.assertEqual(self.agent.q_table.shape, (5, 5, 5, 2, 2))
    
    def test_state_encoding(self):
        """Test state encoding functionality"""
        state = np.array([10.0, 150.0, 0.5, 25.0, 1.0])
        encoded = self.state_encoder.encode_state(state)
        
        self.assertEqual(len(encoded), 4)  # Should match state_space_dims without last dimension
        self.assertTrue(all(0 <= idx < dim for idx, dim in zip(encoded, self.agent.state_space_dims[:-1])))
    
    def test_action_selection(self):
        """Test action selection"""
        state = np.array([5.0, 75.0, 0.3, 15.0, 0.0])
        action = self.agent.get_action(state, training=False)
        
        self.assertIn(action, [0, 1])
    
    def test_q_value_update(self):
        """Test Q-value update"""
        state = np.array([5.0, 75.0, 0.3, 15.0, 0.0])
        next_state = np.array([4.0, 70.0, 0.25, 18.0, 0.0])
        action = 1
        reward = 2.5
        
        # Get initial Q-value
        state_indices = self.state_encoder.encode_state(state)
        initial_q = self.agent.q_table[state_indices + (action,)]
        
        # Update Q-value
        self.agent.update_q_value(state, action, reward, next_state)
        
        # Check that Q-value changed
        updated_q = self.agent.q_table[state_indices + (action,)]
        self.assertNotEqual(initial_q, updated_q)

class TestDataLogger(unittest.TestCase):
    """Test data logging functionality"""
    
    def setUp(self):
        # Use temporary directory for testing
        import tempfile
        self.temp_dir = tempfile.mkdtemp()
        self.data_logger = DataLogger(output_dir=self.temp_dir)
    
    def test_training_record_creation(self):
        """Test training record creation"""
        record = TrainingRecord(
            episode=1,
            time=10.5,
            lane_id="test_lane",
            state=[5.0, 30.0, 0.3, 25.0, 0.0],
            action=1,
            reward=2.5,
            next_state=[4.0, 25.0, 0.25, 28.0, 0.0],
            ambulance_detected=False
        )
        
        self.assertEqual(record.episode, 1)
        self.assertEqual(record.lane_id, "test_lane")
        self.assertEqual(record.action, 1)
        self.assertEqual(record.reward, 2.5)
        self.assertFalse(record.ambulance_detected)
    
    def test_episode_logging(self):
        """Test episode logging workflow"""
        # Start episode
        success = self.data_logger.start_episode(1)
        self.assertTrue(success)
        
        # Log some training records
        self.data_logger.log_training_record(
            lane_id="lane1",
            state=[5.0, 30.0, 0.3, 25.0, 0.0],
            action=1,
            reward=2.5,
            next_state=[4.0, 25.0, 0.25, 28.0, 0.0],
            ambulance_detected=False
        )
        
        # End episode
        success = self.data_logger.end_episode()
        self.assertTrue(success)
    
    def tearDown(self):
        # Clean up temporary directory
        import shutil
        shutil.rmtree(self.temp_dir, ignore_errors=True)

class TestEmergencyPriority(unittest.TestCase):
    """Test emergency vehicle priority handling"""
    
    def test_emergency_detection(self):
        """Test emergency vehicle detection"""
        intersection = IntersectionState(intersection_id="test_intersection")
        
        # Test no emergency initially
        self.assertFalse(intersection.emergency_detected)
        
        # Simulate emergency detection
        intersection.emergency_detected = True
        intersection.emergency_approach = "north_approach"
        
        self.assertTrue(intersection.emergency_detected)
        self.assertEqual(intersection.emergency_approach, "north_approach")

class TestSystemIntegration(unittest.TestCase):
    """Test system integration functionality"""
    
    @patch('src.simulation.sumo_interface.traci')
    def test_system_initialization(self, mock_traci):
        """Test system initialization without actual SUMO"""
        # Mock SUMO functions
        mock_traci.start.return_value = None
        mock_traci.trafficlight.getIDList.return_value = ["tl1"]
        mock_traci.lanearea.getIDList.return_value = []
        mock_traci.lane.getIDList.return_value = ["lane1", "lane2"]
        
        from src.simulation.sumo_interface import SUMOInterface
        
        sumo = SUMOInterface()
        self.assertIsNotNone(sumo)

def run_tests():
    """Run all tests"""
    print("🧪 Running Smart Traffic Intersection System Tests")
    print("=" * 60)
    
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add test cases
    test_cases = [
        TestTrafficState,
        TestScoringSystem,
        TestQLearningAgent,
        TestDataLogger,
        TestEmergencyPriority,
        TestSystemIntegration
    ]
    
    for test_case in test_cases:
        tests = unittest.TestLoader().loadTestsFromTestCase(test_case)
        test_suite.addTests(tests)
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    # Print summary
    print("\n" + "=" * 60)
    print("🧪 TEST SUMMARY")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    
    if result.failures:
        print("\n❌ FAILURES:")
        for test, traceback in result.failures:
            print(f"  {test}: {traceback}")
    
    if result.errors:
        print("\n❌ ERRORS:")
        for test, traceback in result.errors:
            print(f"  {test}: {traceback}")
    
    success = len(result.failures) == 0 and len(result.errors) == 0
    print(f"\n{'✅ ALL TESTS PASSED' if success else '❌ SOME TESTS FAILED'}")
    
    return success

if __name__ == "__main__":
    run_tests()