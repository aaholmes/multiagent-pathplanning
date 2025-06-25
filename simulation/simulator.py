"""
Core simulation engine for multi-agent navigation.
"""

import numpy as np
from typing import List, Dict, Optional, Tuple, Callable
import navigation_core
from .scenario_loader import ScenarioConfig


class SimulationState:
    """Represents the current state of the simulation."""
    
    def __init__(self, agents: List[navigation_core.AgentState], time: float = 0.0):
        self.agents = agents
        self.time = time
        self.agent_trajectories: Dict[int, List[navigation_core.Point]] = {
            agent.id: [agent.position] for agent in agents
        }
    
    def update_agents(self, new_agents: List[navigation_core.AgentState], dt: float):
        """Update agent states and record trajectories."""
        self.agents = new_agents
        self.time += dt
        
        for agent in self.agents:
            if agent.id not in self.agent_trajectories:
                self.agent_trajectories[agent.id] = []
            self.agent_trajectories[agent.id].append(agent.position)
    
    def get_agent_by_id(self, agent_id: int) -> Optional[navigation_core.AgentState]:
        """Get agent by ID."""
        for agent in self.agents:
            if agent.id == agent_id:
                return agent
        return None


class Simulator:
    """Main simulation engine."""
    
    def __init__(self, config: ScenarioConfig):
        self.config = config
        self.grid = self._create_grid()
        self.tasks = self._create_tasks()
        self.initial_agents = self._create_agents()
        
        # Simulation parameters
        sim_params = config.simulation_params
        self.dt = sim_params.get('dt', 0.1)
        self.max_time = sim_params.get('max_time', 100.0)
        self.time_horizon = sim_params.get('time_horizon', 2.0)
        self.neighbor_distance = sim_params.get('neighbor_distance', 10.0)
        
        # Global paths (computed once)
        self.global_paths: Optional[Dict[int, List[navigation_core.Point]]] = None
        
        # Callbacks
        self.step_callback: Optional[Callable[[SimulationState], None]] = None
        
    def _create_grid(self) -> navigation_core.Grid:
        """Create grid from configuration."""
        grid = navigation_core.Grid(self.config.grid_width, self.config.grid_height)
        for x, y in self.config.obstacles:
            grid.set_obstacle(x, y, True)
        return grid
    
    def _create_tasks(self) -> List[navigation_core.Task]:
        """Create tasks from configuration."""
        tasks = []
        for agent_data in self.config.agents:
            task = navigation_core.Task(
                agent_data['id'],
                navigation_core.Point(agent_data['start'][0], agent_data['start'][1]),
                navigation_core.Point(agent_data['goal'][0], agent_data['goal'][1])
            )
            tasks.append(task)
        return tasks
    
    def _create_agents(self) -> List[navigation_core.AgentState]:
        """Create initial agent states from configuration."""
        agents = []
        for agent_data in self.config.agents:
            agent = navigation_core.AgentState(
                agent_data['id'],
                navigation_core.Point(agent_data['start'][0], agent_data['start'][1]),
                navigation_core.Vector2D(0.0, 0.0),
                agent_data.get('radius', 0.5),
                navigation_core.Vector2D(0.0, 0.0),
                agent_data.get('max_speed', 2.0)
            )
            agents.append(agent)
        return agents
    
    def compute_global_paths(self) -> bool:
        """Compute global paths using CBS."""
        print("Computing global paths with CBS...")
        self.global_paths = navigation_core.solve_cbs_py(self.grid, self.tasks)
        
        if self.global_paths is None:
            print("Failed to find global solution!")
            return False
        
        print(f"Found global paths for {len(self.global_paths)} agents")
        for agent_id, path in self.global_paths.items():
            print(f"  Agent {agent_id}: {len(path)} waypoints")
        
        return True
    
    def _get_preferred_velocity(self, agent: navigation_core.AgentState, current_time: float) -> navigation_core.Vector2D:
        """Compute preferred velocity for an agent based on its global path."""
        if self.global_paths is None or agent.id not in self.global_paths:
            return navigation_core.Vector2D(0.0, 0.0)
        
        path = self.global_paths[agent.id]
        if len(path) < 2:
            return navigation_core.Vector2D(0.0, 0.0)
        
        # Find the next waypoint to target
        current_pos = agent.position
        target_idx = 0
        
        # Find the closest waypoint on the path
        min_distance = float('inf')
        closest_idx = 0
        for i, waypoint in enumerate(path):
            distance = current_pos.distance(waypoint)
            if distance < min_distance:
                min_distance = distance
                closest_idx = i
        
        # Target the next waypoint ahead, or the goal if we're close to the closest point
        if min_distance < 1.5:  # If we're close to a waypoint
            target_idx = min(closest_idx + 1, len(path) - 1)  # Move to next waypoint
        else:
            target_idx = closest_idx  # Head to closest waypoint
        
        target = path[target_idx]
        direction = target - current_pos
        
        if direction.magnitude() < 0.2:
            # Very close to target, look further ahead or stop if at goal
            if target_idx < len(path) - 1:
                # Not at goal yet, target next waypoint
                target = path[target_idx + 1]
                direction = target - current_pos
            else:
                # At final goal, stop
                return navigation_core.Vector2D(0.0, 0.0)
        
        # Normalize and scale by preferred speed (slightly less than max for smooth motion)
        preferred_speed = agent.max_speed * 0.8
        return direction.normalize() * preferred_speed
    
    def _get_neighbors(self, agent: navigation_core.AgentState, all_agents: List[navigation_core.AgentState]) -> List[navigation_core.AgentState]:
        """Get neighboring agents within sensing range."""
        neighbors = []
        for other in all_agents:
            if other.id != agent.id:
                distance = agent.distance_to(other)
                if distance <= self.neighbor_distance:
                    neighbors.append(other)
        return neighbors
    
    def step(self, state: SimulationState) -> SimulationState:
        """Perform one simulation step."""
        new_agents = []
        
        for agent in state.agents:
            # Compute preferred velocity from global path
            pref_vel = self._get_preferred_velocity(agent, state.time)
            agent.pref_velocity = pref_vel
            
            # Get neighbors
            neighbors = self._get_neighbors(agent, state.agents)
            neighbor_refs = [neighbor for neighbor in neighbors]
            
            # Compute safe velocity using ORCA
            safe_velocity = navigation_core.compute_orca_velocity_py(
                agent, neighbor_refs, self.time_horizon
            )
            
            # Update agent
            new_position = agent.position + safe_velocity * self.dt
            new_agent = navigation_core.AgentState(
                agent.id,
                new_position,
                safe_velocity,
                agent.radius,
                pref_vel,
                agent.max_speed
            )
            
            new_agents.append(new_agent)
        
        # Create new state
        new_state = SimulationState(new_agents, state.time + self.dt)
        new_state.agent_trajectories = state.agent_trajectories.copy()
        new_state.update_agents(new_agents, 0.0)  # Just update trajectories
        
        return new_state
    
    def run(self, max_steps: Optional[int] = None) -> SimulationState:
        """Run the complete simulation."""
        # Compute global paths first
        if not self.compute_global_paths():
            raise RuntimeError("Failed to compute global paths")
        
        # Initialize simulation state
        state = SimulationState(self.initial_agents.copy())
        steps = 0
        max_steps = max_steps or int(self.max_time / self.dt)
        
        print(f"Starting simulation (max_steps={max_steps}, dt={self.dt})")
        
        while steps < max_steps and state.time < self.max_time:
            # Perform simulation step
            state = self.step(state)
            steps += 1
            
            # Check if all agents reached their goals (but don't exit early unless we're sure)
            if steps > 50 and self._all_agents_at_goal(state):  # Give some time before checking
                print(f"All agents reached their goals at time {state.time:.2f}")
                break
            
            # Call step callback if provided
            if self.step_callback:
                self.step_callback(state)
            
            # Progress reporting
            if steps % 50 == 0:
                print(f"Step {steps}, time {state.time:.2f}")
        
        print(f"Simulation completed: {steps} steps, {state.time:.2f} time units")
        return state
    
    def _all_agents_at_goal(self, state: SimulationState, tolerance: float = 1.0) -> bool:
        """Check if all agents have reached their goals."""
        agents_at_goal = 0
        total_agents = len(state.agents)
        
        for agent in state.agents:
            # Find corresponding task
            task = None
            for t in self.tasks:
                if t.agent_id == agent.id:
                    task = t
                    break
            
            if task is None:
                continue
            
            distance_to_goal = agent.position.distance(task.goal)
            if distance_to_goal <= tolerance:
                agents_at_goal += 1
        
        # Only return True if ALL agents are at their goals
        return agents_at_goal == total_agents
    
    def get_statistics(self, state: SimulationState) -> Dict:
        """Compute simulation statistics."""
        stats = {
            'total_time': state.time,
            'total_steps': len(next(iter(state.agent_trajectories.values()))),
            'agents': {}
        }
        
        for agent_id, trajectory in state.agent_trajectories.items():
            if len(trajectory) < 2:
                continue
            
            # Compute path length
            path_length = 0.0
            for i in range(1, len(trajectory)):
                path_length += trajectory[i-1].distance(trajectory[i])
            
            # Find corresponding task
            task = None
            for t in self.tasks:
                if t.agent_id == agent_id:
                    task = t
                    break
            
            # Distance to goal
            final_pos = trajectory[-1]
            distance_to_goal = final_pos.distance(task.goal) if task else float('inf')
            
            stats['agents'][agent_id] = {
                'path_length': path_length,
                'distance_to_goal': distance_to_goal,
                'reached_goal': distance_to_goal < 0.5,
                'trajectory_points': len(trajectory)
            }
        
        # Overall statistics
        total_path_length = sum(agent_stats['path_length'] for agent_stats in stats['agents'].values())
        agents_at_goal = sum(1 for agent_stats in stats['agents'].values() if agent_stats['reached_goal'])
        
        stats['summary'] = {
            'total_path_length': total_path_length,
            'average_path_length': total_path_length / len(stats['agents']) if stats['agents'] else 0,
            'success_rate': agents_at_goal / len(stats['agents']) if stats['agents'] else 0,
            'agents_at_goal': agents_at_goal,
            'total_agents': len(stats['agents'])
        }
        
        return stats
    
    def set_step_callback(self, callback: Callable[[SimulationState], None]):
        """Set a callback function to be called after each simulation step."""
        self.step_callback = callback