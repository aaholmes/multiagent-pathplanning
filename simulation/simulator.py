"""
Core simulation engine for multi-agent navigation.
"""

import numpy as np
from typing import List, Dict, Optional, Tuple, Callable
import navigation_core
from .scenario_loader import ScenarioConfig, ScenarioLoader


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
        self.grid = ScenarioLoader.create_grid_from_config(config)
        self.tasks = ScenarioLoader.create_tasks_from_config(config)
        self.initial_agents = ScenarioLoader.create_agent_states_from_config(config)
        
        # Simulation parameters
        sim_params = config.simulation_params
        self.dt = sim_params.get('dt', 0.1)
        self.max_time = sim_params.get('max_time', 100.0)
        self.time_horizon = sim_params.get('time_horizon', 2.0)
        self.neighbor_distance = sim_params.get('neighbor_distance', 10.0)
        
        # Global paths (computed once)
        self.global_paths: Optional[Dict[int, List[navigation_core.Point]]] = None
        # Per-agent index of the next waypoint to chase (monotone progress)
        self._waypoint_progress: Dict[int, int] = {}
        
        # Callbacks
        self.step_callback: Optional[Callable[[SimulationState], None]] = None
        
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
        """Compute preferred velocity for an agent based on its global path.

        Progress along the path is tracked with a monotone waypoint index. A
        closest-waypoint search would re-target earlier or later detour
        waypoints whenever ORCA pushes the agent off its path, which can stall
        an agent indefinitely.
        """
        if self.global_paths is None or agent.id not in self.global_paths:
            return navigation_core.Vector2D(0.0, 0.0)

        path = self.global_paths[agent.id]
        if not path:
            return navigation_core.Vector2D(0.0, 0.0)

        # Advance past waypoints the agent has effectively reached
        idx = self._waypoint_progress.get(agent.id, 0)
        while idx < len(path) - 1 and agent.position.distance(path[idx]) < 0.6:
            idx += 1
        self._waypoint_progress[agent.id] = idx

        target = path[idx]
        direction = target - agent.position

        # At the final waypoint: stop once close enough
        if idx == len(path) - 1 and direction.magnitude() < 0.2:
            return navigation_core.Vector2D(0.0, 0.0)

        # Normalize and scale by preferred speed (slightly less than max for smooth motion)
        preferred_speed = agent.max_speed * 0.8
        return direction.normalize() * preferred_speed

    def step(self, state: SimulationState) -> SimulationState:
        """Perform one simulation step."""
        # Set each agent's preferred velocity from its global path
        for agent in state.agents:
            agent.pref_velocity = self._get_preferred_velocity(agent, state.time)

        # One batched call computes ORCA velocities for all agents, with
        # neighbor selection and static grid-obstacle constraints in Rust.
        # Obstacle cells are approximated by discs slightly smaller than the
        # cell's inscribed circle: grid-legal paths may graze cell boundaries
        # (a 1-wide gap offers exactly zero clearance to a 0.5-radius agent),
        # so full-size discs would make legal corridors infeasible.
        safe_velocities = navigation_core.compute_orca_velocities_py(
            state.agents,
            self.time_horizon,
            self.neighbor_distance,
            grid=self.grid,
            obstacle_radius=0.35,
        )

        new_agents = []
        for agent, safe_velocity in zip(state.agents, safe_velocities):
            new_agents.append(navigation_core.AgentState(
                agent.id,
                agent.position + safe_velocity * self.dt,
                safe_velocity,
                agent.radius,
                agent.pref_velocity,
                agent.max_speed,
            ))

        # Create new state with copied trajectory lists. A shallow dict copy
        # would alias the lists across states, so appending here would
        # retroactively mutate every previously stored state (which made
        # animation frames all render the final trajectory).
        new_state = SimulationState(new_agents, state.time + self.dt)
        new_state.agent_trajectories = {
            agent_id: list(trajectory)
            for agent_id, trajectory in state.agent_trajectories.items()
        }
        for agent in new_agents:
            new_state.agent_trajectories.setdefault(agent.id, []).append(agent.position)

        return new_state

    def run(self, max_steps: Optional[int] = None) -> SimulationState:
        """Run the complete simulation."""
        # Compute global paths first
        if not self.compute_global_paths():
            raise RuntimeError("Failed to compute global paths")
        
        # Initialize simulation state
        self._waypoint_progress = {}
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
    
    def _all_agents_at_goal(self, state: SimulationState, tolerance: float = 0.5) -> bool:
        # Tolerance matches get_statistics' reached_goal threshold; using a
        # looser value here ended runs that statistics then scored as failures.
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