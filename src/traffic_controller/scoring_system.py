"""
Scoring System Module

This module implements the comprehensive scoring system for traffic lanes
with continuous state rewards and status evaluation functions.
"""

from typing import Dict, List, Tuple, Optional, Any
from enum import Enum
import math
import numpy as np

from ..models.traffic_state import TrafficParameters, LaneState, ApproachState, TrafficState
from ..utils.config_manager import config

class ScoringResult:
    """Results of scoring calculation"""
    def __init__(self, score: float, state: TrafficState, reward: int, components: Dict[str, float]):
        self.score = score
        self.state = state
        self.reward = reward
        self.components = components

class LaneScoringSystem:
    """
    Lane scoring system implementing:
    Lane_Score = w1·Q + w2·W + w3·D + w4·F (normalized)
    
    With continuous state rewards:
    - Smooth traffic (GOOD): +1 to +5 points
    - Congested traffic (BAD): -1 to -5 points  
    - State changes GOOD↔BAD: maintain score and continue with new state
    """
    
    def __init__(self):
        self.weights = config.scoring.lane_score_weights
        self.smooth_min = config.scoring.smooth_traffic_min
        self.smooth_max = config.scoring.smooth_traffic_max
        self.congested_min = config.scoring.congested_traffic_min
        self.congested_max = config.scoring.congested_traffic_max
        
        # Normalization factors (will be updated based on observed maximums)
        self.max_values = {
            'Q': 20.0,    # Max queue length
            'W': 300.0,   # Max waiting time (5 minutes)
            'D': 1.0,     # Max density (already normalized)
            'F': 3600.0   # Max flow rate (vehicles/hour)
        }
    
    def calculate_lane_score(self, lane_state: LaneState) -> ScoringResult:
        """
        Calculate comprehensive lane score using Lane_Score = w1·Q + w2·W + w3·D + w4·F
        """
        params = lane_state.parameters
        
        # Normalize components
        Q_norm = min(params.l / self.max_values['Q'], 1.0)
        W_norm = min(params.td / self.max_values['W'], 1.0)
        D_norm = params.m  # Already normalized 0-1
        F_norm = 1.0 - min(params.g / self.max_values['F'], 1.0)  # Invert - higher flow is better
        
        # Calculate weighted score
        components = {
            'Q': Q_norm,
            'W': W_norm, 
            'D': D_norm,
            'F': F_norm
        }
        
        weighted_score = (
            self.weights['w1'] * Q_norm +
            self.weights['w2'] * W_norm +
            self.weights['w3'] * D_norm +
            self.weights['w4'] * F_norm
        )
        
        # Determine state and reward
        state, reward = self._determine_state_and_reward(weighted_score)
        
        # Update lane score with continuous rewards
        lane_state.lane_score += reward
        
        # Clamp lane score to reasonable bounds
        lane_state.lane_score = max(-50, min(50, lane_state.lane_score))
        
        return ScoringResult(weighted_score, state, reward, components)
    
    def _determine_state_and_reward(self, score: float) -> Tuple[TrafficState, int]:
        """
        Determine traffic state and continuous reward based on score
        """
        if score < 0.3:  # Very smooth traffic
            reward = self.smooth_max  # +5
            state = TrafficState.SMOOTH
        elif score < 0.5:  # Moderately smooth
            reward = self.smooth_max - 1  # +4
            state = TrafficState.SMOOTH
        elif score < 0.6:  # Slightly smooth
            reward = self.smooth_min + 1  # +2
            state = TrafficState.SMOOTH
        elif score < 0.7:  # Neutral/slightly congested
            reward = self.smooth_min  # +1
            state = TrafficState.SMOOTH
        elif score < 0.8:  # Moderately congested  
            reward = self.congested_max  # -1
            state = TrafficState.CONGESTED
        elif score < 0.9:  # Heavily congested
            reward = self.congested_max - 1  # -2
            state = TrafficState.CONGESTED
        else:  # Critically congested
            reward = self.congested_min  # -5
            state = TrafficState.CRITICAL
        
        return state, reward
    
    def update_max_values(self, params: TrafficParameters):
        """Update maximum values for better normalization"""
        self.max_values['Q'] = max(self.max_values['Q'], params.l)
        self.max_values['W'] = max(self.max_values['W'], params.td)
        self.max_values['D'] = max(self.max_values['D'], params.m) 
        self.max_values['F'] = max(self.max_values['F'], params.g)

class StatusFunction:
    """
    Implements Status_t = f(l, tđ, m, v, g) function with threshold comparisons
    """
    
    def __init__(self):
        self.weights = config.traffic.parameter_weights
        self.status_threshold = config.traffic.status_threshold
        self.critical_threshold = config.traffic.critical_threshold
        
    def calculate_status_t(self, params: TrafficParameters, max_values: Dict[str, float]) -> float:
        """
        Calculate Status_t using normalized parameters
        Status_t = w_l·l + w_td·tđ + w_m·m + w_v·(1-v_norm) + w_g·(1-g_norm)
        """
        # Normalize parameters
        l_norm = min(params.l / max_values.get('l', 20.0), 1.0)
        td_norm = min(params.td / max_values.get('td', 300.0), 1.0)
        m_norm = params.m  # Already normalized
        v_norm = 1.0 - min(params.v / max_values.get('v', 50.0), 1.0)  # Invert - higher speed is better
        g_norm = 1.0 - min(params.g / max_values.get('g', 3600.0), 1.0)  # Invert - higher flow is better
        
        # Calculate weighted status
        status_t = (
            self.weights['l'] * l_norm +
            self.weights['td'] * td_norm +
            self.weights['m'] * m_norm +
            self.weights['v'] * v_norm +
            self.weights['g'] * g_norm
        )
        
        return min(max(status_t, 0.0), 1.0)  # Clamp between 0 and 1
    
    def evaluate_status(self, status_t: float) -> Dict[str, Any]:
        """
        Evaluate status_t against thresholds and provide recommendations
        """
        if status_t >= self.critical_threshold:
            level = "CRITICAL"
            priority = "HIGH"
            action = "IMMEDIATE_ATTENTION"
            adjustment = +15  # Significant green time increase
        elif status_t >= self.status_threshold:
            level = "CONGESTED" 
            priority = "MEDIUM"
            action = "INCREASE_GREEN_TIME"
            adjustment = +8  # Moderate green time increase
        elif status_t < self.status_threshold * 0.5:
            level = "SMOOTH"
            priority = "LOW"
            action = "OPTIMIZE_TIMING"
            adjustment = -5  # Slight green time decrease
        else:
            level = "NORMAL"
            priority = "LOW"
            action = "MAINTAIN"
            adjustment = 0  # No change
        
        return {
            'status_value': status_t,
            'level': level,
            'priority': priority,
            'recommended_action': action,
            'timing_adjustment': adjustment,
            'exceeds_threshold': status_t >= self.status_threshold,
            'is_critical': status_t >= self.critical_threshold
        }

class DirectionScoringSystem:
    """
    Direction/approach-level scoring system aggregating multiple lane scores
    """
    
    def __init__(self):
        self.lane_scorer = LaneScoringSystem()
        self.status_function = StatusFunction()
    
    def calculate_direction_status(self, approach: ApproachState) -> Dict[str, Any]:
        """
        Calculate direction status: status = w1·Σlane_score + w2·max(lane_score)
        """
        if not approach.lanes:
            return {'status': 0.0, 'components': {}, 'lane_scores': []}
        
        lane_scores = []
        status_values = []
        
        # Calculate scores for each lane
        for lane_id, lane_state in approach.lanes.items():
            scoring_result = self.lane_scorer.calculate_lane_score(lane_state)
            lane_scores.append(scoring_result.score)
            
            # Calculate status_t for this lane  
            status_t = self.status_function.calculate_status_t(
                lane_state.parameters,
                self.lane_scorer.max_values
            )
            status_values.append(status_t)
        
        # Direction status calculation
        avg_lane_score = sum(lane_scores) / len(lane_scores)
        max_lane_score = max(lane_scores)
        avg_status_t = sum(status_values) / len(status_values)
        max_status_t = max(status_values)
        
        # Combined direction status (weighted combination)
        w1, w2 = 0.7, 0.3  # Weights for average vs max
        direction_status = w1 * avg_status_t + w2 * max_status_t
        
        # Update approach metrics
        approach.total_score = sum(lane.lane_score for lane in approach.lanes.values())
        approach.avg_status = avg_status_t
        approach.priority_level = direction_status
        
        return {
            'status': direction_status,
            'avg_status_t': avg_status_t,
            'max_status_t': max_status_t,
            'avg_lane_score': avg_lane_score,
            'max_lane_score': max_lane_score,
            'total_accumulated_score': approach.total_score,
            'lane_scores': lane_scores,
            'status_values': status_values,
            'evaluation': self.status_function.evaluate_status(direction_status),
            'lane_count': len(approach.lanes)
        }

class CycleAggregator:
    """
    Aggregates lane scores after each cycle for dynamic time distribution
    """
    
    def __init__(self):
        self.direction_scorer = DirectionScoringSystem()
        self.cycle_history = []
        
    def aggregate_cycle_scores(self, intersection_state) -> Dict[str, Any]:
        """
        Aggregate lane scores after cycle completion
        Returns timing adjustments based on total score ratios
        """
        cycle_results = {
            'cycle_number': len(self.cycle_history) + 1,
            'timestamp': intersection_state.phase_start_time,
            'approaches': {},
            'total_scores': {},
            'timing_adjustments': {},
            'priority_ranking': []
        }
        
        # Calculate scores for each approach
        approach_scores = {}
        approach_statuses = {}
        
        for approach_id, approach in intersection_state.approaches.items():
            direction_result = self.direction_scorer.calculate_direction_status(approach)
            
            cycle_results['approaches'][approach_id] = direction_result
            approach_scores[approach_id] = approach.total_score
            approach_statuses[approach_id] = direction_result['status']
        
        # Calculate timing adjustments based on score ratios
        total_negative_score = sum(abs(score) for score in approach_scores.values() if score < 0)
        total_positive_score = sum(score for score in approach_scores.values() if score > 0)
        
        for approach_id, score in approach_scores.items():
            status = approach_statuses[approach_id]
            
            # Base adjustment from status evaluation
            base_adjustment = cycle_results['approaches'][approach_id]['evaluation']['timing_adjustment']
            
            # Proportional adjustment based on relative score
            if score < 0 and total_negative_score > 0:
                # Increase green time for congested approaches
                proportion = abs(score) / total_negative_score
                proportional_adjustment = int(20 * proportion)  # Up to 20 seconds
            elif score > 0 and total_positive_score > 0:
                # Slightly reduce time for smooth approaches
                proportion = score / total_positive_score
                proportional_adjustment = -int(5 * proportion)  # Up to -5 seconds
            else:
                proportional_adjustment = 0
            
            # Combined adjustment
            total_adjustment = base_adjustment + proportional_adjustment
            
            # Apply limits
            cycle_results['timing_adjustments'][approach_id] = {
                'base_adjustment': base_adjustment,
                'proportional_adjustment': proportional_adjustment,
                'total_adjustment': max(-10, min(30, total_adjustment)),  # Clamp between -10 and +30
                'score': score,
                'status': status
            }
        
        # Create priority ranking
        priority_list = sorted(
            approach_statuses.items(),
            key=lambda x: x[1],
            reverse=True
        )
        cycle_results['priority_ranking'] = priority_list
        
        # Store cycle history
        self.cycle_history.append(cycle_results)
        
        # Keep only last 50 cycles
        if len(self.cycle_history) > 50:
            self.cycle_history.pop(0)
        
        return cycle_results
    
    def get_timing_recommendations(self, intersection_state) -> Dict[str, Any]:
        """
        Get timing recommendations based on latest cycle aggregation
        """
        if not self.cycle_history:
            # No history, use default recommendations
            return self._get_default_recommendations(intersection_state)
        
        latest_cycle = self.cycle_history[-1]
        
        recommendations = {
            'cycle_number': latest_cycle['cycle_number'],
            'recommendations': {},
            'priority_approach': None,
            'emergency_override': intersection_state.emergency_detected
        }
        
        # Emergency override takes absolute priority
        if intersection_state.emergency_detected:
            recommendations['priority_approach'] = intersection_state.emergency_approach
            recommendations['emergency_override'] = True
            recommendations['action'] = 'EMERGENCY_PRIORITY'
            return recommendations
        
        # Find approach with highest priority (most congested)
        if latest_cycle['priority_ranking']:
            priority_approach = latest_cycle['priority_ranking'][0][0]
            recommendations['priority_approach'] = priority_approach
        
        # Generate recommendations for each approach
        for approach_id, adjustment_data in latest_cycle['timing_adjustments'].items():
            recommendations['recommendations'][approach_id] = {
                'green_time_change': adjustment_data['total_adjustment'],
                'priority_level': latest_cycle['approaches'][approach_id]['status'],
                'reason': self._generate_reason(adjustment_data),
                'urgency': self._determine_urgency(adjustment_data['status'])
            }
        
        return recommendations
    
    def _get_default_recommendations(self, intersection_state) -> Dict[str, Any]:
        """Generate default recommendations when no history exists"""
        return {
            'cycle_number': 0,
            'recommendations': {},
            'priority_approach': None,
            'emergency_override': intersection_state.emergency_detected,
            'action': 'INITIALIZE'
        }
    
    def _generate_reason(self, adjustment_data: Dict[str, Any]) -> str:
        """Generate human-readable reason for timing adjustment"""
        score = adjustment_data['score']
        status = adjustment_data['status']
        adjustment = adjustment_data['total_adjustment']
        
        if score < -10:
            return f"Heavily congested (score: {score:.1f}), extend green by {adjustment}s"
        elif score < 0:
            return f"Congested (score: {score:.1f}), increase green by {adjustment}s"
        elif score > 10:
            return f"Very smooth flow (score: {score:.1f}), optimize timing"
        elif adjustment < 0:
            return f"Smooth flow, reduce green by {abs(adjustment)}s"
        else:
            return f"Normal conditions (score: {score:.1f})"
    
    def _determine_urgency(self, status: float) -> str:
        """Determine urgency level based on status"""
        if status >= config.traffic.critical_threshold:
            return "HIGH"
        elif status >= config.traffic.status_threshold:
            return "MEDIUM"
        else:
            return "LOW"