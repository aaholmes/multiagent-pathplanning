# Design Doc
Project: High-Performance MAPP Solver using CBS

Goal: Implement an optimal, high-performance MAPP solver based on the Conflict-Based Search (CBS) algorithm. The implementation will be a Rust core library with Python bindings for ease of use and visualization.

System Architecture: Hybrid Rust/Python Model

Rust Core Library (mapp_core): A compiled library containing the complete CBS solver and all related data structures and algorithms.
Python Simulation Layer: A Python package for defining MAPP problems, calling the Rust solver, and visualizing the results.
1. Rust Core Library (mapp_core)
Crate: mapp_core
Dependencies: pyo3, priority-queue (for A* open set).

Data Structures (src/structs.rs):

Point { x: i32, y: i32 }
AgentTask { id: usize, start: Point, goal: Point }
Path = Vec<Point>
Constraint { agent_id: usize, location: Point, time: usize }
Conflict { agent_1_id: usize, agent_2_id: usize, location: Point, time: usize }
CTNode { constraints: Vec<Constraint>, solution: HashMap<usize, Path>, cost: usize } // A node in the Constraint Tree
Core Logic (src/lib.rs and modules):

Low-Level Planner (src/a_star.rs):

find_path(agent_task: &AgentTask, constraints: &Vec<Constraint>, grid: &Grid) -> Option<Path>
Implements a space-time A* search.
Takes a single agent's task and a set of constraints that apply to it.
The A* state will be (Point, time).
The heuristic will be the Manhattan distance from the current point to the goal.
It must respect the constraints (i.e., not occupy a constrained location at a constrained time).
High-Level Solver (src/cbs.rs):

solve(tasks: Vec<AgentTask>, grid: &Grid) -> Option<HashMap<usize, Path>>
This is the main public function.
Initializes a priority queue for the Constraint Tree (CT) nodes, ordered by cost (sum of path lengths).
Creates the root CT node:
Find initial paths for all agents using the low-level A* planner with no constraints.
Calculate the initial cost.
Pushes the root node to the priority queue.
Main CBS Loop:
Pop the CT node with the lowest cost.
Check its solution for conflicts using a function find_first_conflict(&solution) -> Option<Conflict>.
If no conflict, this is the optimal solution. Return it.
If there is a conflict, create two new child CT nodes. For the conflict involving agent A and agent B at location L at time T:
Child 1: Add the constraint { agent_id: A, location: L, time: T } to the parent's constraints. Re-run the low-level planner only for agent A with this new set of constraints. Recalculate the cost and push the new CT node to the queue.
Child 2: Add the constraint { agent_id: B, location: L, time: T } to the parent's constraints. Re-run the low-level planner only for agent B. Recalculate the cost and push the new CT node to the queue.
Repeat the loop.
Python Bindings (using #[pyfunction]):

Expose AgentTask and other necessary structs.
Expose the main solve function so it can be called from Python with a list of agent tasks.
2. Python Simulation Layer
Dependencies: matplotlib, numpy, mapp_core.

Structure:

scenarios/: A directory containing JSON files that define different MAPP problems (grid layout, agent start/goal positions).
simulation/grid.py: A class to load and represent the static environment (obstacles).
simulation/visualizer.py: A class to animate the final paths found by the solver.
simulation/run_mapp.py: The main script.
Main Logic in run_mapp.py:

Setup:
Parse command-line arguments to get the scenario file path.
Load the scenario JSON file into Python data structures (a list of mapp_core.AgentTask objects).
Load the grid.
Solve:
Call the Rust solver: solution = mapp_core.solve(agent_tasks, grid).
This one function call does all the heavy lifting.
Animate:
If a solution is found, pass the map and the solution paths to the Visualizer.
The Visualizer uses matplotlib.animation to create a frame-by-frame animation of each agent moving along its path.
Save the animation as a GIF
