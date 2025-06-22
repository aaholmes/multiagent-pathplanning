# High-Performance Multi-Agent Path Planning (MAPP)

This project provides a high-performance, optimal solver for the classic Multi-Agent Path Planning (MAPP) problem. The implementation uses Conflict-Based Search (CBS), a state-of-the-art algorithm that finds the shortest set of paths for a team of agents while guaranteeing no collisions.

The solver is written in Rust for maximum performance and is wrapped in a Python API for easy use in simulations and other applications.

(Placeholder for a GIF/video of the simulation, showing dozens of agents seamlessly navigating a cluttered environment.)

Project Status
Status: Under Development

The core low-level A* planner and the high-level Constraint Tree search for the CBS algorithm are implemented in Rust. The Python simulation layer can call the solver and visualize the resulting paths.

Core Concepts
Multi-Agent Path Planning (MAPP) is the problem of finding a set of collision-free paths for multiple agents from their start to goal locations, typically while optimizing a metric like the sum of path lengths.

Conflict-Based Search (CBS) is a two-level algorithm that solves this problem optimally:

Low-Level Planner: At the low level, a standard single-agent pathfinder (like A*) finds the optimal path for each agent independently, ignoring all other agents.
High-Level Search: At the high level, CBS searches a Constraint Tree (CT). If the low-level paths result in a collision (a "conflict"), the high-level search branches into new nodes. In each new branch, a "constraint" is added (e.g., "Agent 5 cannot be at location (x,y) at time t"), and the low-level planner is called again for the constrained agent. This process continues until a conflict-free solution is found, which is guaranteed to be optimal.
Setup and Running the Simulation
This project uses maturin to build the Rust library and create Python bindings.

1. Clone the repository:

Bash

git clone https://github.com/your-username/mapp-solver.git
cd mapp-solver
2. Set up a Python virtual environment:

Bash

python3 -m venv .venv
source .venv/bin/activate
3. Install dependencies and build the Rust library:

Bash

pip install -r requirements.txt
maturin develop
4. Run a simulation:

Bash

python simulation/run_mapp.py --scenario scenarios/4_agents_simple.json
