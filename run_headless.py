#!/usr/bin/env python3
"""
Headless simulation runner that works without any GUI dependencies.
This script runs simulations and outputs text-based results only.
"""

import argparse
import sys
import time
from pathlib import Path
import json

# Add the project root to the path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

def run_headless_simulation(scenario_file):
    """Run simulation without any visualization dependencies."""
    try:
        import navigation_core
        print("✅ Successfully imported navigation_core")
    except ImportError as e:
        print(f"❌ Failed to import navigation_core: {e}")
        print("💡 Run: maturin develop --release")
        return False
    
    # Load scenario
    print(f"📁 Loading scenario: {scenario_file}")
    try:
        with open(scenario_file, 'r') as f:
            scenario_data = json.load(f)
    except FileNotFoundError:
        print(f"❌ Scenario file not found: {scenario_file}")
        return False
    except json.JSONDecodeError as e:
        print(f"❌ Invalid JSON in scenario file: {e}")
        return False
    
    # Create grid
    grid_config = scenario_data['grid']
    grid = navigation_core.Grid(grid_config['width'], grid_config['height'])
    
    # Add obstacles
    if 'obstacles' in scenario_data:
        for x, y in scenario_data['obstacles']:
            grid.set_obstacle(x, y, True)
        print(f"🚧 Added {len(scenario_data['obstacles'])} obstacles")
    
    # Create tasks
    tasks = []
    for agent_data in scenario_data['agents']:
        task = navigation_core.Task(
            agent_data['id'],
            navigation_core.Point(agent_data['start'][0], agent_data['start'][1]),
            navigation_core.Point(agent_data['goal'][0], agent_data['goal'][1])
        )
        tasks.append(task)
    
    print(f"👥 Created {len(tasks)} agent tasks")
    
    # Run CBS to find global paths
    print("🧠 Running Conflict-Based Search (CBS)...")
    start_time = time.time()
    
    solution = navigation_core.solve_cbs_py(grid, tasks)
    
    cbs_time = time.time() - start_time
    
    if solution is None:
        print("❌ CBS failed to find a solution")
        return False
    
    print(f"✅ CBS found solution in {cbs_time:.3f} seconds")
    
    # Analyze solution
    total_path_length = 0
    max_path_length = 0
    
    print("\n📊 Solution Analysis:")
    print("Agent | Start → Goal | Path Length")
    print("-" * 35)
    
    for agent_id, path in solution.items():
        # Find corresponding task
        task = next(t for t in tasks if t.agent_id == agent_id)
        
        # Calculate path length
        path_length = 0
        if len(path) > 1:
            for i in range(1, len(path)):
                path_length += path[i-1].distance(path[i])
        
        total_path_length += path_length
        max_path_length = max(max_path_length, len(path))
        
        print(f"  {agent_id:2d}  | ({task.start.x:2.0f},{task.start.y:2.0f}) → ({task.goal.x:2.0f},{task.goal.y:2.0f}) | {path_length:8.2f}")
    
    print("-" * 35)
    print(f"Total path length: {total_path_length:.2f}")
    print(f"Average path length: {total_path_length / len(solution):.2f}")
    print(f"Maximum path steps: {max_path_length}")
    
    # Validate solution
    print(f"\n🔍 Validating solution...")
    valid = navigation_core.validate_solution(solution, tasks)
    
    if valid:
        print("✅ Solution is valid")
    else:
        print("❌ Solution validation failed")
    
    print(f"\n🎉 Simulation completed successfully!")
    return True

def create_simple_test_scenario():
    """Create a simple test scenario for demonstration."""
    scenario = {
        "grid": {
            "width": 8,
            "height": 8
        },
        "agents": [
            {
                "id": 0,
                "start": [0, 3],
                "goal": [7, 3],
                "radius": 0.5,
                "max_speed": 2.0
            },
            {
                "id": 1,
                "start": [7, 3],
                "goal": [0, 3],
                "radius": 0.5,
                "max_speed": 2.0
            }
        ],
        "obstacles": [
            [3, 2], [3, 4], [4, 2], [4, 4]
        ],
        "simulation_params": {
            "dt": 0.1,
            "max_time": 20.0,
            "time_horizon": 2.0,
            "neighbor_distance": 10.0
        }
    }
    
    with open("test_scenario.json", "w") as f:
        json.dump(scenario, f, indent=2)
    
    print("📝 Created test_scenario.json")
    return "test_scenario.json"

def main():
    parser = argparse.ArgumentParser(description="Headless Multi-Agent Navigation Simulation")
    parser.add_argument("--scenario", "-s", type=str, help="Scenario file to load")
    parser.add_argument("--test", action="store_true", help="Create and run a simple test scenario")
    
    args = parser.parse_args()
    
    try:
        if args.test:
            print("🧪 Creating and running test scenario...")
            scenario_file = create_simple_test_scenario()
            success = run_headless_simulation(scenario_file)
        elif args.scenario:
            success = run_headless_simulation(args.scenario)
        else:
            print("❓ No scenario specified. Use --scenario <file> or --test")
            print("Example: python run_headless.py --test")
            print("Example: python run_headless.py --scenario scenarios/simple_2_agents.json")
            return 1
        
        return 0 if success else 1
        
    except KeyboardInterrupt:
        print("\n⏹️  Simulation interrupted by user.")
        return 1
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())