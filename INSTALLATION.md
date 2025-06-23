# Installation Guide

This guide provides step-by-step instructions for installing the multi-agent navigation system.

## Prerequisites

- **Python 3.8+** (check with `python3 --version`)
- **Rust 1.70+** (install from [rustup.rs](https://rustup.rs))
- **Git** (for cloning the repository)

## Installation Steps

### 1. Clone the Repository
```bash
git clone https://github.com/your-username/multiagent-pathplanning.git
cd multiagent-pathplanning
```

### 2. Create Virtual Environment
```bash
python3 -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
```

### 3. Install Python Dependencies
```bash
pip install -r requirements.txt
```

This will install:
- `numpy>=1.20.0` - Numerical computations
- `matplotlib>=3.5.0` - Visualization
- `maturin>=1.0.0` - Rust-Python integration

### 4. Build the Rust Library
```bash
maturin develop --release
```

This compiles the high-performance Rust core and creates Python bindings.

### 5. Verify Installation

#### Test 1: Project Structure
```bash
python tests/test_structure.py
```

#### Test 2: Rust Unit Tests
```bash
cargo test
```

#### Test 3: Python Integration
```bash
python test_basic_import.py
```

## Quick Test Run

Once installed, test with a simple simulation:

```bash
python simulation/run_simulation.py --scenario scenarios/simple_2_agents.json
```

## Troubleshooting

### Common Issues

#### 1. "maturin not found"
**Solution**: Make sure you're in the virtual environment:
```bash
source .venv/bin/activate
pip install maturin
```

#### 2. "No module named 'navigation_core'"
**Solution**: The Rust library wasn't built. Run:
```bash
maturin develop --release
```

#### 3. "No module named 'numpy'"
**Solution**: Install Python dependencies:
```bash
pip install -r requirements.txt
```

#### 4. PyO3 compilation errors
**Solution**: Update PyO3 and maturin:
```bash
pip install --upgrade maturin
```

#### 5. "externally-managed-environment"
**Solution**: Use a virtual environment (see step 2 above).

### Build Modes

- **Release mode** (recommended): `maturin develop --release`
  - Optimized for performance
  - Use for simulations and benchmarks

- **Debug mode**: `maturin develop`
  - Faster compilation
  - Use for development and debugging

### Alternative: Docker Installation

If you prefer containerized installation:

```bash
# Create Dockerfile (example)
FROM python:3.11
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"
COPY . /app
WORKDIR /app
RUN pip install -r requirements.txt && maturin develop --release
```

## Development Setup

For development, you may want additional tools:

```bash
# Rust development tools
rustup component add rustfmt clippy

# Python development tools
pip install pytest black mypy

# Code formatting
cargo fmt
black simulation/
```

## Performance Notes

- **Release builds** are ~10x faster than debug builds
- Use `maturin develop --release` for all performance testing
- The Rust core handles the computationally intensive algorithms
- Python layer handles visualization and scenario management

## Next Steps

After successful installation:

1. **Run Examples**: Try the scenarios in `scenarios/`
2. **Read Documentation**: Check `README.md` for detailed usage
3. **Explore Code**: Look at the Rust source in `src/` and Python code in `simulation/`
4. **Create Scenarios**: Use the scenario generators or create custom JSON files

## Getting Help

- **Quick Test**: Run `python test_basic_import.py` to verify core functionality
- **Full Test**: Run the complete test suite with `cargo test` and `python tests/test_structure.py`
- **Documentation**: See `README.md` for comprehensive documentation
- **Examples**: Check the `scenarios/` directory for example configurations