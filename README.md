# High-Performance Multi-Robot Navigation Stack
This project implements a complete, two-layer navigation stack for multi-agent systems, combining optimal global path planning with real-time, reactive collision avoidance.

The solver is written in Rust for maximum performance and is wrapped in a Python API for easy use in simulations and other applications.

(Placeholder for a GIF/video of the simulation, showing dozens of agents seamlessly navigating a cluttered environment, making small, smooth adjustments to avoid each other.)


Project Status
Status: Under Development

The core algorithms for both the global and local planners are implemented in Rust. The Python simulation layer can run complex scenarios and visualize the resulting emergent behavior.

Core Concepts: A Two-Layer Architecture
A robust navigation system needs to operate on two levels: long-term strategy and short-term reflexes. This project implements both.

Global Planner: Conflict-Based Search (CBS)

At the global level, this project uses Conflict-Based Search (CBS), a state-of-the-art algorithm that finds the shortest possible set of paths for all agents from their start to goal locations. It acts as the "strategist," providing each agent with a complete, optimal plan assuming a perfect world.

Local Planner: Optimal Reciprocal Collision Avoidance (ORCA)

At the local level, this project uses Optimal Reciprocal Collision Avoidance (ORCA). ORCA acts as the "reflexes." At every moment, each agent calculates a safe velocity that avoids immediate collisions with its neighbors, while still trying to follow the global plan. This allows agents to gracefully handle dynamic situations and minor deviations from their paths.

By combining CBS and ORCA, this system generates paths that are both globally optimal and locally safe and smooth.

Setup and Running the Simulation
This project uses maturin to build the Rust library and create Python bindings.

1. Clone the repository:

git clone https://github.com/your-username/multi-robot-navigation.git
cd multi-robot-navigation

2. Set up a Python virtual environment:

python3 -m venv .venv
source .venv/bin/activate

3. Install dependencies and build the Rust library:

pip install -r requirements.txt
maturin develop

4. Run a simulation:

python simulation/run_simulation.py --scenario scenarios/4_agents_simple.json
