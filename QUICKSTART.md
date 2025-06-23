# Quick Start Guide

This guide will help you get the multi-agent navigation system running quickly.

## Installation

### 1. Prerequisites
- Python 3.8 or later
- Rust 1.70 or later
- Git

### 2. Clone and Setup
```bash
git clone https://github.com/your-username/multiagent-pathplanning.git
cd multiagent-pathplanning

# Create virtual environment
python3 -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install Python dependencies
pip install -r requirements.txt

# Build the Rust library
maturin develop --release
```

## Quick Tests

### 1. Verify Project Structure
```bash
python3 tests/test_structure.py
```

### 2. Run Rust Unit Tests
```bash
cargo test
```

### 3. Test Python Integration (requires built library)
```bash
python3 -c "
import navigation_core
print('✓ Navigation core imported successfully')

# Test basic data structures
point = navigation_core.Point(1.0, 2.0)
vector = navigation_core.Vector2D(1.0, 0.0)
print(f'✓ Created point: {point}')
print(f'✓ Created vector: {vector}')

# Test grid
grid = navigation_core.Grid(10, 10)
print(f'✓ Created grid: {grid}')

print('All basic tests passed!')
"
```

## Running Simulations

### 1. Simple 2-Agent Scenario
```bash
python simulation/run_simulation.py --scenario scenarios/simple_2_agents.json
```

### 2. Create and Run Custom Scenario
```bash
# Create a corridor scenario
python simulation/run_simulation.py --create corridor --agents 6 --width 20

# Create a simple random scenario
python simulation/run_simulation.py --create simple --agents 8 --width 15 --height 15 --obstacles 0.15
```

### 3. Real-time Visualization
```bash
python simulation/run_simulation.py --scenario scenarios/4_agents_crossing.json --real-time
```

### 4. Save Results
```bash
# Save animation and analysis plots
python simulation/run_simulation.py --scenario scenarios/complex_maze.json \
    --save-animation results.gif \
    --save-plots analysis/results
```

## Understanding the Output

When you run a simulation, you'll see:

1. **CBS Phase**: Global path computation
   ```
   Computing global paths with CBS...
   Found global paths for 4 agents
   ```

2. **Simulation Phase**: Real-time execution with ORCA
   ```
   Starting simulation (max_steps=300, dt=0.1)
   Step 50, time 5.0
   ```

3. **Results**: Success metrics and statistics
   ```
   Simulation Statistics:
   Success Rate: 100.0%
   Average Path Length: 15.2
   Total Simulation Time: 12.3s
   ```

## Common Issues

### Build Issues
- **PyO3 version error**: Update to latest maturin: `pip install --upgrade maturin`
- **Python not found**: Ensure Python development headers are installed
- **Rust compilation errors**: Update Rust: `rustup update`

### Runtime Issues
- **Import errors**: Make sure you ran `maturin develop` successfully
- **Visualization not showing**: Install matplotlib: `pip install matplotlib>=3.5.0`
- **Performance issues**: Use `--release` flag: `maturin develop --release`

## Next Steps

1. **Explore Scenarios**: Check out the `scenarios/` directory for example configurations
2. **Modify Parameters**: Edit scenario JSON files to experiment with different setups
3. **Algorithm Development**: Look at the Rust source code in `src/` for algorithm details
4. **Research Applications**: Use the statistics and visualization tools for analysis

## Getting Help

- Check the full README.md for detailed documentation
- Look at the example scenarios in the `scenarios/` directory
- Run the test suite to verify your installation
- Review the Python simulation code in the `simulation/` directory

Happy simulating! 🤖