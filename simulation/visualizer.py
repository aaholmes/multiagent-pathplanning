"""
Real-time visualization for multi-agent navigation simulation.
"""

import matplotlib
# Set backend before importing pyplot to avoid tkinter dependency
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import numpy as np
from typing import List, Dict, Optional, Tuple
import navigation_core
from .simulator import SimulationState
from .scenario_loader import ScenarioConfig


class Visualizer:
    """Real-time visualization of multi-agent simulation."""
    
    def __init__(self, config: ScenarioConfig, figsize: Tuple[int, int] = (12, 8)):
        self.config = config
        self.fig, self.ax = plt.subplots(figsize=figsize)
        
        # Set up the plot
        self.ax.set_xlim(-0.5, config.grid_width - 0.5)
        self.ax.set_ylim(-0.5, config.grid_height - 0.5)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title('Multi-Agent Navigation Simulation')
        
        # Color palette for agents
        self.colors = plt.cm.tab10(np.linspace(0, 1, max(10, len(config.agents))))
        
        # Visualization elements
        self.agent_circles = {}
        self.agent_velocity_arrows = {}
        self.agent_trails = {}
        self.goal_markers = {}
        self.obstacle_patches = []
        
        # Animation
        self.animation = None
        self.states_history = []
        self.current_frame = 0
        
        self._setup_static_elements()
    
    def _setup_static_elements(self):
        """Set up static visualization elements (obstacles, goals)."""
        # Draw obstacles
        for x, y in self.config.obstacles:
            rect = patches.Rectangle(
                (x - 0.5, y - 0.5), 1, 1,
                linewidth=1, edgecolor='black', facecolor='gray', alpha=0.8
            )
            self.ax.add_patch(rect)
            self.obstacle_patches.append(rect)
        
        # Draw start and goal positions
        for i, agent_data in enumerate(self.config.agents):
            color = self.colors[i % len(self.colors)]
            
            # Start position (triangle)
            start_x, start_y = agent_data['start']
            self.ax.plot(start_x, start_y, marker='^', color=color, markersize=8, 
                        label=f'Agent {agent_data["id"]} start' if i < 5 else '')
            
            # Goal position (square)
            goal_x, goal_y = agent_data['goal']
            goal_marker = self.ax.plot(goal_x, goal_y, marker='s', color=color, 
                                     markersize=8, alpha=0.7)[0]
            self.goal_markers[agent_data['id']] = goal_marker
        
        # Add legend if not too many agents
        if len(self.config.agents) <= 5:
            self.ax.legend(loc='upper left', bbox_to_anchor=(1, 1))
    
    def update_visualization(self, state: SimulationState):
        """Update visualization with current simulation state."""
        # Clear previous agent visualizations
        for circle in self.agent_circles.values():
            circle.remove()
        for arrow in self.agent_velocity_arrows.values():
            arrow.remove()
        for trail in self.agent_trails.values():
            trail.remove()
        
        self.agent_circles.clear()
        self.agent_velocity_arrows.clear()
        self.agent_trails.clear()
        
        # Draw agents
        for agent in state.agents:
            color = self.colors[agent.id % len(self.colors)]
            
            # Agent circle
            circle = patches.Circle(
                (agent.position.x, agent.position.y),
                agent.radius,
                facecolor=color,
                edgecolor='black',
                alpha=0.7
            )
            self.ax.add_patch(circle)
            self.agent_circles[agent.id] = circle
            
            # Velocity arrow
            if agent.velocity.magnitude() > 0.01:
                arrow = self.ax.arrow(
                    agent.position.x, agent.position.y,
                    agent.velocity.x * 0.3, agent.velocity.y * 0.3,
                    head_width=0.1, head_length=0.1,
                    fc=color, ec=color, alpha=0.8
                )
                self.agent_velocity_arrows[agent.id] = arrow
            
            # Agent trail
            if agent.id in state.agent_trajectories and len(state.agent_trajectories[agent.id]) > 1:
                trajectory = state.agent_trajectories[agent.id]
                x_coords = [p.x for p in trajectory[-20:]]  # Last 20 points
                y_coords = [p.y for p in trajectory[-20:]]
                
                trail, = self.ax.plot(x_coords, y_coords, color=color, alpha=0.5, linewidth=2)
                self.agent_trails[agent.id] = trail
        
        # Update title with time information
        self.ax.set_title(f'Multi-Agent Navigation - Time: {state.time:.2f}s')
        
        # Refresh display
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def save_frame(self, filename: str, state: SimulationState):
        """Save current visualization as image."""
        self.update_visualization(state)
        self.fig.savefig(filename, dpi=150, bbox_inches='tight')
    
    def show_static(self, state: SimulationState):
        """Show static visualization of a single state."""
        self.update_visualization(state)
        # In non-interactive mode, save instead of show
        filename = "current_state.png"
        self.fig.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"Saved static visualization to: {filename}")
    
    def create_animation(self, states: List[SimulationState], interval: int = 100, save_path: Optional[str] = None):
        """Create animation from sequence of states."""
        self.states_history = states
        
        def animate(frame):
            if frame < len(self.states_history):
                self.update_visualization(self.states_history[frame])
            return []
        
        self.animation = animation.FuncAnimation(
            self.fig, animate, frames=len(states),
            interval=interval, blit=False, repeat=True
        )
        
        if save_path:
            print(f"Saving animation to {save_path}...")
            self.animation.save(save_path, writer='pillow', fps=10)
            print("Animation saved!")
        
        return self.animation
    
    def show_animation(self, states: List[SimulationState], interval: int = 100):
        """Show animation of simulation states."""
        # In non-interactive mode, save a GIF instead
        anim = self.create_animation(states, interval, save_path="animation_output.gif")
        print("Saved animation to: animation_output.gif")
        return anim


class StatisticsVisualizer:
    """Visualizer for simulation statistics and analysis."""
    
    @staticmethod
    def plot_trajectories(state: SimulationState, config: ScenarioConfig, figsize: Tuple[int, int] = (10, 8)):
        """Plot all agent trajectories."""
        fig, ax = plt.subplots(figsize=figsize)
        
        # Set up the plot
        ax.set_xlim(-0.5, config.grid_width - 0.5)
        ax.set_ylim(-0.5, config.grid_height - 0.5)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title('Agent Trajectories')
        
        # Draw obstacles
        for x, y in config.obstacles:
            rect = patches.Rectangle(
                (x - 0.5, y - 0.5), 1, 1,
                linewidth=1, edgecolor='black', facecolor='gray', alpha=0.8
            )
            ax.add_patch(rect)
        
        # Color palette
        colors = plt.cm.tab10(np.linspace(0, 1, max(10, len(config.agents))))
        
        # Plot trajectories
        for agent_id, trajectory in state.agent_trajectories.items():
            if len(trajectory) < 2:
                continue
            
            color = colors[agent_id % len(colors)]
            x_coords = [p.x for p in trajectory]
            y_coords = [p.y for p in trajectory]
            
            # Plot trajectory
            ax.plot(x_coords, y_coords, color=color, linewidth=2, alpha=0.8, label=f'Agent {agent_id}')
            
            # Mark start and end
            ax.plot(x_coords[0], y_coords[0], marker='o', color=color, markersize=8)
            ax.plot(x_coords[-1], y_coords[-1], marker='s', color=color, markersize=8)
        
        if len(config.agents) <= 10:
            ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        plt.tight_layout()
        
        # Save the plot since we can't display it interactively
        fig.savefig("trajectories.png", dpi=150, bbox_inches='tight')
        print("Saved trajectory plot to: trajectories.png")
        
        return fig, ax
    
    @staticmethod
    def plot_statistics(stats: Dict, figsize: Tuple[int, int] = (12, 8)):
        """Plot simulation statistics."""
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=figsize)
        
        agent_ids = list(stats['agents'].keys())
        path_lengths = [stats['agents'][aid]['path_length'] for aid in agent_ids]
        distances_to_goal = [stats['agents'][aid]['distance_to_goal'] for aid in agent_ids]
        success = [stats['agents'][aid]['reached_goal'] for aid in agent_ids]
        
        # Path lengths
        ax1.bar(agent_ids, path_lengths)
        ax1.set_title('Path Lengths by Agent')
        ax1.set_xlabel('Agent ID')
        ax1.set_ylabel('Path Length')
        
        # Distance to goal
        ax2.bar(agent_ids, distances_to_goal)
        ax2.set_title('Final Distance to Goal')
        ax2.set_xlabel('Agent ID')
        ax2.set_ylabel('Distance')
        ax2.axhline(y=0.5, color='r', linestyle='--', alpha=0.7, label='Success threshold')
        ax2.legend()
        
        # Success rate
        success_count = sum(success)
        fail_count = len(success) - success_count
        ax3.pie([success_count, fail_count], labels=['Success', 'Failed'], autopct='%1.1f%%')
        ax3.set_title('Success Rate')
        
        # Summary statistics
        summary_text = f"""
        Total Simulation Time: {stats['total_time']:.2f}s
        Total Steps: {stats['total_steps']}
        
        Average Path Length: {stats['summary']['average_path_length']:.2f}
        Total Path Length: {stats['summary']['total_path_length']:.2f}
        
        Success Rate: {stats['summary']['success_rate']:.1%}
        Agents at Goal: {stats['summary']['agents_at_goal']}/{stats['summary']['total_agents']}
        """
        
        ax4.text(0.1, 0.5, summary_text, transform=ax4.transAxes, fontsize=10, verticalalignment='center')
        ax4.set_xlim(0, 1)
        ax4.set_ylim(0, 1)
        ax4.axis('off')
        ax4.set_title('Summary Statistics')
        
        plt.tight_layout()
        
        # Save the plot since we can't display it interactively
        fig.savefig("statistics.png", dpi=150, bbox_inches='tight')
        print("Saved statistics plot to: statistics.png")
        
        return fig
    
    @staticmethod
    def plot_velocity_profiles(state: SimulationState, agent_ids: Optional[List[int]] = None, figsize: Tuple[int, int] = (12, 6)):
        """Plot velocity profiles over time for selected agents."""
        if agent_ids is None:
            agent_ids = list(state.agent_trajectories.keys())[:5]  # First 5 agents
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=figsize)
        colors = plt.cm.tab10(np.linspace(0, 1, len(agent_ids)))
        
        for i, agent_id in enumerate(agent_ids):
            if agent_id not in state.agent_trajectories:
                continue
            
            trajectory = state.agent_trajectories[agent_id]
            if len(trajectory) < 2:
                continue
            
            # Compute velocities
            times = np.arange(len(trajectory)) * 0.1  # Assuming dt=0.1
            velocities = []
            speeds = []
            
            for j in range(1, len(trajectory)):
                dt = 0.1  # Assumed time step
                vel = (trajectory[j] - trajectory[j-1])
                vel_vec = navigation_core.Vector2D(vel.x / dt, vel.y / dt)
                velocities.append(vel_vec)
                speeds.append(vel_vec.magnitude())
            
            # Plot speed over time
            ax1.plot(times[1:], speeds, color=colors[i], label=f'Agent {agent_id}', linewidth=2)
        
        ax1.set_title('Speed Profiles')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Speed')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot average speed by agent
        avg_speeds = []
        for agent_id in agent_ids:
            if agent_id not in state.agent_trajectories:
                avg_speeds.append(0)
                continue
            
            trajectory = state.agent_trajectories[agent_id]
            if len(trajectory) < 2:
                avg_speeds.append(0)
                continue
            
            total_distance = 0
            total_time = (len(trajectory) - 1) * 0.1
            
            for j in range(1, len(trajectory)):
                total_distance += trajectory[j-1].distance(trajectory[j])
            
            avg_speed = total_distance / total_time if total_time > 0 else 0
            avg_speeds.append(avg_speed)
        
        ax2.bar(agent_ids, avg_speeds, color=colors)
        ax2.set_title('Average Speeds')
        ax2.set_xlabel('Agent ID')
        ax2.set_ylabel('Average Speed')
        
        plt.tight_layout()
        
        # Save the plot since we can't display it interactively
        fig.savefig("velocity_profiles.png", dpi=150, bbox_inches='tight')
        print("Saved velocity profiles plot to: velocity_profiles.png")
        
        return fig