#!/usr/bin/env python3
"""
Main simulation runner script.
"""

import argparse
import sys
import time
from pathlib import Path

# Add the project root to the path so we can import the simulation modules
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from simulation import Simulator, Visualizer, ScenarioLoader, StatisticsVisualizer


def run_simulation_with_visualization(config_or_scenario, real_time=False, save_animation=None, save_plots=None):
    """Run simulation with real-time or post-processing visualization."""
    
    # Handle different input types
    if isinstance(config_or_scenario, str):
        config = ScenarioLoader.load_from_file(config_or_scenario)
    else:
        config = config_or_scenario
    
    # Create simulator
    simulator = Simulator(config)
    
    # Set up visualization
    visualizer = Visualizer(config)
    states_history = []
    
    if real_time:
        # Real-time visualization callback
        def step_callback(state):
            visualizer.update_visualization(state)
            states_history.append(state)
            time.sleep(0.05)  # Small delay for visualization
        
        simulator.set_step_callback(step_callback)
    
    print("Starting simulation...")
    start_time = time.time()
    
    # Run simulation
    final_state = simulator.run()
    
    end_time = time.time()
    print(f"Simulation completed in {end_time - start_time:.2f} seconds")
    
    # If not real-time, we need to run again to collect states for visualization
    if not real_time:
        print("Collecting states for visualization...")
        simulator_viz = Simulator(config)
        states_history = []
        
        def collect_states(state):
            states_history.append(state)
        
        simulator_viz.set_step_callback(collect_states)
        simulator_viz.run()
    
    # Get statistics
    stats = simulator.get_statistics(final_state)
    print("\nSimulation Statistics:")
    print(f"Success Rate: {stats['summary']['success_rate']:.1%}")
    print(f"Average Path Length: {stats['summary']['average_path_length']:.2f}")
    print(f"Total Simulation Time: {stats['total_time']:.2f}s")
    
    # Save or show animation
    if save_animation:
        print(f"Creating animation...")
        visualizer.create_animation(states_history, interval=100, save_path=save_animation)
    else:
        print("Creating static visualization (interactive display not available)...")
        # Save a static plot instead of trying to show interactive animation
        static_plot_path = "simulation_result.png"
        visualizer.save_frame(static_plot_path, final_state)
        print(f"Saved static visualization to: {static_plot_path}")
    
    # Generate and save plots
    if save_plots:
        print("Generating plots...")
        
        # Trajectory plot
        fig1, _ = StatisticsVisualizer.plot_trajectories(final_state, config)
        fig1.savefig(f"{save_plots}_trajectories.png", dpi=150, bbox_inches='tight')
        
        # Statistics plot
        fig2 = StatisticsVisualizer.plot_statistics(stats)
        fig2.savefig(f"{save_plots}_statistics.png", dpi=150, bbox_inches='tight')
        
        # Velocity profiles
        fig3 = StatisticsVisualizer.plot_velocity_profiles(final_state)
        fig3.savefig(f"{save_plots}_velocities.png", dpi=150, bbox_inches='tight')
        
        print(f"Plots saved with prefix: {save_plots}")
    
    return final_state, stats


def create_scenario_file(scenario_type, filepath, **kwargs):
    """Create and save a scenario file."""
    print(f"Creating {scenario_type} scenario...")
    
    if scenario_type == "simple":
        config = ScenarioLoader.generate_simple_scenario(**kwargs)
    elif scenario_type == "corridor":
        config = ScenarioLoader.generate_corridor_scenario(**kwargs)
    elif scenario_type == "intersection":
        config = ScenarioLoader.generate_intersection_scenario(**kwargs)
    else:
        raise ValueError(f"Unknown scenario type: {scenario_type}")
    
    ScenarioLoader.save_to_file(config, filepath)
    print(f"Scenario saved to {filepath}")
    return config


def main():
    parser = argparse.ArgumentParser(description="Multi-Agent Navigation Simulation")
    parser.add_argument("--scenario", "-s", type=str, help="Scenario file to load")
    parser.add_argument("--create", "-c", choices=["simple", "corridor", "intersection"], 
                       help="Create a new scenario file")
    parser.add_argument("--output", "-o", type=str, default="scenario.json", 
                       help="Output file for created scenario")
    parser.add_argument("--real-time", "-r", action="store_true", 
                       help="Show real-time visualization")
    parser.add_argument("--save-animation", type=str, 
                       help="Save animation to file (e.g., animation.gif)")
    parser.add_argument("--save-plots", type=str, 
                       help="Save plots with given prefix (e.g., results/sim)")
    
    # Scenario generation parameters
    parser.add_argument("--width", type=int, default=15, help="Grid width")
    parser.add_argument("--height", type=int, default=15, help="Grid height")
    parser.add_argument("--agents", type=int, default=4, help="Number of agents")
    parser.add_argument("--obstacles", type=float, default=0.1, help="Obstacle density")
    parser.add_argument("--seed", type=int, help="Random seed")
    
    args = parser.parse_args()
    
    try:
        if args.create:
            # Create scenario parameters based on type
            if args.create == "simple":
                kwargs = {
                    'grid_width': args.width,
                    'grid_height': args.height,
                    'num_agents': args.agents,
                    'obstacle_density': args.obstacles,
                    'seed': args.seed
                }
            elif args.create == "corridor":
                kwargs = {
                    'corridor_length': args.width,
                    'corridor_width': max(3, args.height // 5),
                    'num_agents': args.agents
                }
            elif args.create == "intersection":
                kwargs = {
                    'size': args.width,
                    'num_agents_per_direction': max(1, args.agents // 4)
                }
            
            config = create_scenario_file(args.create, args.output, **kwargs)
            
            # Ask if user wants to run the simulation
            response = input(f"Scenario created. Run simulation? (y/n): ")
            if response.lower() in ['y', 'yes']:
                run_simulation_with_visualization(
                    config, 
                    real_time=args.real_time,
                    save_animation=args.save_animation,
                    save_plots=args.save_plots
                )
        
        elif args.scenario:
            # Load and run existing scenario
            run_simulation_with_visualization(
                args.scenario,
                real_time=args.real_time,
                save_animation=args.save_animation,
                save_plots=args.save_plots
            )
        
        else:
            # Run default simple scenario
            print("No scenario specified. Running default simple scenario...")
            config = ScenarioLoader.generate_simple_scenario(
                grid_width=15, grid_height=15, num_agents=4, obstacle_density=0.1
            )
            run_simulation_with_visualization(
                config,
                real_time=args.real_time,
                save_animation=args.save_animation,
                save_plots=args.save_plots
            )
    
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    except Exception as e:
        print(f"Error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())