#!/usr/bin/env python3
"""
Setup script for the multi-agent navigation system.
This script handles the complete installation process.
"""

import subprocess
import sys
import os
from pathlib import Path

def run_command(command, description, check=True):
    """Run a shell command with error handling."""
    print(f"🔧 {description}...")
    try:
        result = subprocess.run(command, shell=True, check=check, capture_output=True, text=True)
        if result.stdout:
            print(f"   Output: {result.stdout.strip()}")
        return result.returncode == 0
    except subprocess.CalledProcessError as e:
        print(f"❌ Failed: {e}")
        if e.stderr:
            print(f"   Error: {e.stderr}")
        return False

def check_prerequisites():
    """Check if required tools are installed."""
    print("🔍 Checking prerequisites...")
    
    # Check Python
    python_version = sys.version_info
    if python_version < (3, 8):
        print(f"❌ Python 3.8+ required, found {python_version.major}.{python_version.minor}")
        return False
    print(f"✅ Python {python_version.major}.{python_version.minor}.{python_version.micro}")
    
    # Check Rust
    if run_command("cargo --version", "Checking Rust installation", check=False):
        print("✅ Rust/Cargo found")
    else:
        print("❌ Rust not found. Install from: https://rustup.rs/")
        return False
    
    # Check maturin
    if run_command("maturin --version", "Checking maturin", check=False):
        print("✅ Maturin found")
    else:
        print("⚠️  Maturin not found, will install it")
    
    return True

def install_python_dependencies():
    """Install Python dependencies."""
    print("📦 Installing Python dependencies...")
    
    # Install maturin first
    if not run_command(f"{sys.executable} -m pip install maturin>=1.0", "Installing maturin"):
        return False
    
    # Install other dependencies
    dependencies = ["numpy>=1.20.0", "matplotlib>=3.5.0"]
    for dep in dependencies:
        if not run_command(f"{sys.executable} -m pip install '{dep}'", f"Installing {dep}"):
            return False
    
    return True

def build_rust_library():
    """Build the Rust library with maturin."""
    print("🦀 Building Rust library...")
    
    # First try with release mode for better performance
    if run_command("maturin develop --release", "Building with --release", check=False):
        print("✅ Release build successful")
        return True
    
    # If that fails, try debug mode
    if run_command("maturin develop", "Building in debug mode", check=False):
        print("✅ Debug build successful")
        return True
    
    print("❌ Failed to build Rust library")
    return False

def run_tests():
    """Run test suite to verify installation."""
    print("🧪 Running tests...")
    
    # Run Rust tests
    if not run_command("cargo test", "Running Rust unit tests"):
        return False
    
    # Run Python structure tests
    if not run_command("python3 tests/test_structure.py", "Running structure tests"):
        return False
    
    # Run basic import test
    if not run_command("python3 test_basic_import.py", "Testing Python imports"):
        return False
    
    return True

def main():
    """Main setup function."""
    print("Multi-Agent Navigation System Setup")
    print("=" * 50)
    
    # Change to project directory
    project_dir = Path(__file__).parent
    os.chdir(project_dir)
    print(f"📁 Working directory: {project_dir}")
    
    # Check prerequisites
    if not check_prerequisites():
        print("\n❌ Prerequisites check failed")
        return False
    
    print("\n" + "=" * 50)
    
    # Install Python dependencies
    if not install_python_dependencies():
        print("\n❌ Python dependency installation failed")
        return False
    
    print("\n" + "=" * 50)
    
    # Build Rust library
    if not build_rust_library():
        print("\n❌ Rust library build failed")
        return False
    
    print("\n" + "=" * 50)
    
    # Run tests
    if not run_tests():
        print("\n❌ Tests failed")
        return False
    
    print("\n" + "=" * 50)
    print("🎉 Setup completed successfully!")
    print("\nYou can now run simulations:")
    print("  python3 simulation/run_simulation.py --scenario scenarios/simple_2_agents.json")
    print("  python3 simulation/run_simulation.py --create simple --agents 4")
    print("\nFor more options, see README.md or run:")
    print("  python3 simulation/run_simulation.py --help")
    
    return True

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)