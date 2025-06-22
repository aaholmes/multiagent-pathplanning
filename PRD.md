# Design Doc
Project: High-Performance Multi-Robot Navigation Stack (CBS + ORCA)

Goal: Implement a complete navigation system featuring a global MAPP solver (CBS) and a local collision avoidance algorithm (ORCA). The core will be a Rust library with Python bindings for simulation.

System Architecture: Hybrid Rust/Python Model

Rust Core Library (navigation_core): A compiled library containing the CBS solver, the ORCA implementation, and all related data structures.

Python Simulation Layer: A Python package for defining scenarios, calling the Rust library, and visualizing the results.

1. Rust Core Library (navigation_core)
Crate: navigation_core
Dependencies: pyo3, priority-queue.

Data Structures (src/structs.rs):

Point { x: f64, y: f64 }

Vector2D { x: f64, y: f64 }

AgentState { id: usize, position: Point, velocity: Vector2D, radius: f64, pref_velocity: Vector2D }

Path = Vec<Point>

Constraint { ... }

Conflict { ... }

CTNode { ... }

Line { point: Point, direction: Vector2D } // For ORCA half-planes

Core Logic:

Global Planner Module (src/cbs.rs):

Contains the full Conflict-Based Search (CBS) implementation as previously designed.

Its main function solve(tasks) -> Option<HashMap<usize, Path>> takes a set of agent tasks and returns a map of agent IDs to their complete, optimal, discrete paths. This is called once at the beginning of a simulation.

Local Planner Module (src/orca.rs):

compute_orca_velocity(agent: &AgentState, neighbors: &Vec<&AgentState>, time_horizon: f64) -> Vector2D

This is the main function for this module. It's called at every simulation step for each agent.

Initializes an empty list of Line objects, representing the linear constraints (ORCA half-planes).

For each neighbor in neighbors:

Calculate the Velocity Obstacle (VO) created by the neighbor.

Calculate the ORCA half-plane constraint based on the agent's and neighbor's current velocities and positions.

Add this Line to the list of constraints.

Solve for the optimal velocity: Find the velocity closest to the agent's pref_velocity that satisfies all the linear constraints. This can be solved with a few iterations of linear programming.

Return the new, safe velocity.

Python Bindings (src/lib.rs):

Expose all necessary data structures.

Expose the global planner: solve_cbs(...).

Expose the local planner: compute_orca_velocity(...).

2. Python Simulation Layer
Dependencies: matplotlib, numpy, navigation_core.

Main Logic in run_simulation.py:

Initialization (Setup Phase):

Load the scenario file (grid, agent start/goals, radii, etc.).

Create AgentState objects for each agent.

Call the Global Planner once: global_paths = navigation_core.solve_cbs(agent_tasks, grid).

Store these paths. If no solution, exit.

Instantiate the Visualizer.

Simulation Loop (Execution Phase, runs for T timesteps):

For each agent i:

Determine Preferred Velocity: Look at the agent's global_paths[i]. The preferred velocity (pref_velocity) is a vector pointing from the agent's current position towards the next waypoint on its global path.

Update agent.pref_velocity.

For each agent i:

Get Neighbors: Find all other agents within a certain radius.

Call the Local Planner: safe_velocity = navigation_core.compute_orca_velocity(agent, &neighbors, time_horizon).

Update the agent's velocity with this new safe_velocity.

Update All Positions: For each agent, update its position based on its new safe velocity and the timestep dt. agent.position += agent.velocity * dt.

Visualize: Call the Visualizer to draw the new world state.

Repeat.
