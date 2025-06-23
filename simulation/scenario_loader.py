"""
Scenario loading and configuration management.
"""

import json
import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
import navigation_core


@dataclass
class ScenarioConfig:
    """Configuration for a multi-agent scenario."""
    grid_width: int
    grid_height: int
    agents: List[Dict]
    obstacles: List[Tuple[int, int]]
    simulation_params: Dict
    
    
class ScenarioLoader:
    """Loads and manages simulation scenarios."""
    
    @staticmethod
    def load_from_file(filepath: str) -> ScenarioConfig:
        """Load scenario from JSON file."""
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        return ScenarioConfig(
            grid_width=data['grid']['width'],
            grid_height=data['grid']['height'],
            agents=data['agents'],
            obstacles=data.get('obstacles', []),
            simulation_params=data.get('simulation_params', {})
        )
    
    @staticmethod
    def save_to_file(config: ScenarioConfig, filepath: str) -> None:
        """Save scenario to JSON file."""
        data = {
            'grid': {
                'width': config.grid_width,
                'height': config.grid_height
            },
            'agents': config.agents,
            'obstacles': config.obstacles,
            'simulation_params': config.simulation_params
        }
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
    
    @staticmethod
    def create_grid_from_config(config: ScenarioConfig) -> navigation_core.Grid:
        """Create a Grid object from scenario configuration."""
        grid = navigation_core.Grid(config.grid_width, config.grid_height)
        
        for x, y in config.obstacles:
            grid.set_obstacle(x, y, True)
        
        return grid
    
    @staticmethod
    def create_tasks_from_config(config: ScenarioConfig) -> List[navigation_core.Task]:
        """Create Task objects from scenario configuration."""
        tasks = []
        
        for agent_data in config.agents:
            task = navigation_core.Task(
                agent_data['id'],
                navigation_core.Point(agent_data['start'][0], agent_data['start'][1]),
                navigation_core.Point(agent_data['goal'][0], agent_data['goal'][1])
            )
            tasks.append(task)
        
        return tasks
    
    @staticmethod
    def create_agent_states_from_config(config: ScenarioConfig) -> List[navigation_core.AgentState]:
        """Create AgentState objects from scenario configuration."""
        agents = []
        
        for agent_data in config.agents:
            agent = navigation_core.AgentState(
                agent_data['id'],
                navigation_core.Point(agent_data['start'][0], agent_data['start'][1]),
                navigation_core.Vector2D(0.0, 0.0),  # Initial velocity
                agent_data.get('radius', 0.5),
                navigation_core.Vector2D(0.0, 0.0),  # Will be set during simulation
                agent_data.get('max_speed', 2.0)
            )
            agents.append(agent)
        
        return agents
    
    @staticmethod
    def generate_simple_scenario(
        grid_width: int = 10,
        grid_height: int = 10,
        num_agents: int = 2,
        obstacle_density: float = 0.1,
        seed: Optional[int] = None
    ) -> ScenarioConfig:
        """Generate a simple random scenario."""
        if seed is not None:
            np.random.seed(seed)
        
        # Generate obstacles
        num_obstacles = int(grid_width * grid_height * obstacle_density)
        obstacles = []
        
        for _ in range(num_obstacles):
            x = np.random.randint(0, grid_width)
            y = np.random.randint(0, grid_height)
            # Avoid corners for start/goal positions
            if not ((x == 0 or x == grid_width - 1) and (y == 0 or y == grid_height - 1)):
                obstacles.append((x, y))
        
        # Generate agents with random start/goal positions
        agents = []
        used_positions = set(obstacles)
        
        for i in range(num_agents):
            # Find valid start position
            while True:
                start_x = np.random.randint(0, grid_width)
                start_y = np.random.randint(0, grid_height)
                if (start_x, start_y) not in used_positions:
                    break
            
            # Find valid goal position
            while True:
                goal_x = np.random.randint(0, grid_width)
                goal_y = np.random.randint(0, grid_height)
                if (goal_x, goal_y) not in used_positions and (goal_x, goal_y) != (start_x, start_y):
                    break
            
            used_positions.add((start_x, start_y))
            used_positions.add((goal_x, goal_y))
            
            agents.append({
                'id': i,
                'start': [start_x, start_y],
                'goal': [goal_x, goal_y],
                'radius': 0.5,
                'max_speed': 2.0
            })
        
        return ScenarioConfig(
            grid_width=grid_width,
            grid_height=grid_height,
            agents=agents,
            obstacles=obstacles,
            simulation_params={
                'dt': 0.1,
                'max_time': 100.0,
                'time_horizon': 2.0,
                'neighbor_distance': 10.0
            }
        )
    
    @staticmethod
    def generate_corridor_scenario(
        corridor_length: int = 20,
        corridor_width: int = 3,
        num_agents: int = 10
    ) -> ScenarioConfig:
        """Generate a corridor scenario where agents need to pass each other."""
        grid_width = corridor_length
        grid_height = corridor_width + 2  # Add walls
        
        # Create walls
        obstacles = []
        for x in range(grid_width):
            obstacles.append((x, 0))  # Bottom wall
            obstacles.append((x, grid_height - 1))  # Top wall
        
        # Create agents going in opposite directions
        agents = []
        middle_y = grid_height // 2
        
        for i in range(num_agents):
            if i % 2 == 0:
                # Agent going left to right
                start = [0, middle_y]
                goal = [grid_width - 1, middle_y]
            else:
                # Agent going right to left
                start = [grid_width - 1, middle_y]
                goal = [0, middle_y]
            
            agents.append({
                'id': i,
                'start': start,
                'goal': goal,
                'radius': 0.4,
                'max_speed': 1.5
            })
        
        return ScenarioConfig(
            grid_width=grid_width,
            grid_height=grid_height,
            agents=agents,
            obstacles=obstacles,
            simulation_params={
                'dt': 0.1,
                'max_time': 50.0,
                'time_horizon': 2.0,
                'neighbor_distance': 8.0
            }
        )
    
    @staticmethod
    def generate_intersection_scenario(
        size: int = 15,
        num_agents_per_direction: int = 3
    ) -> ScenarioConfig:
        """Generate an intersection scenario."""
        grid_width = size
        grid_height = size
        center = size // 2
        road_width = 3
        
        # Create intersection walls
        obstacles = []
        for x in range(grid_width):
            for y in range(grid_height):
                # Create roads: horizontal and vertical strips
                in_horizontal_road = center - road_width // 2 <= y <= center + road_width // 2
                in_vertical_road = center - road_width // 2 <= x <= center + road_width // 2
                
                if not (in_horizontal_road or in_vertical_road):
                    obstacles.append((x, y))
        
        # Create agents from four directions
        agents = []
        agent_id = 0
        
        # From left (going right)
        for i in range(num_agents_per_direction):
            agents.append({
                'id': agent_id,
                'start': [0, center],
                'goal': [grid_width - 1, center],
                'radius': 0.4,
                'max_speed': 1.5
            })
            agent_id += 1
        
        # From right (going left)
        for i in range(num_agents_per_direction):
            agents.append({
                'id': agent_id,
                'start': [grid_width - 1, center],
                'goal': [0, center],
                'radius': 0.4,
                'max_speed': 1.5
            })
            agent_id += 1
        
        # From top (going down)
        for i in range(num_agents_per_direction):
            agents.append({
                'id': agent_id,
                'start': [center, 0],
                'goal': [center, grid_height - 1],
                'radius': 0.4,
                'max_speed': 1.5
            })
            agent_id += 1
        
        # From bottom (going up)
        for i in range(num_agents_per_direction):
            agents.append({
                'id': agent_id,
                'start': [center, grid_height - 1],
                'goal': [center, 0],
                'radius': 0.4,
                'max_speed': 1.5
            })
            agent_id += 1
        
        return ScenarioConfig(
            grid_width=grid_width,
            grid_height=grid_height,
            agents=agents,
            obstacles=obstacles,
            simulation_params={
                'dt': 0.1,
                'max_time': 30.0,
                'time_horizon': 2.0,
                'neighbor_distance': 6.0
            }
        )