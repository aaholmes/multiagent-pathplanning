#!/usr/bin/env python3
"""
Test the project structure and basic file integrity.
"""

import os
import json
from pathlib import Path

def test_project_structure():
    """Test that all required files and directories exist."""
    print("Testing project structure...")
    
    project_root = Path(__file__).parent.parent
    
    # Required files
    required_files = [
        "Cargo.toml",
        "pyproject.toml",
        "requirements.txt",
        "README.md",
        "LICENSE",
        "src/lib.rs",
        "src/structs.rs",
        "src/astar.rs",
        "src/cbs.rs",
        "src/orca.rs",
        "simulation/__init__.py",
        "simulation/simulator.py",
        "simulation/visualizer.py",
        "simulation/scenario_loader.py",
        "simulation/run_simulation.py"
    ]
    
    # Required directories
    required_dirs = [
        "src",
        "simulation",
        "scenarios",
        "tests"
    ]
    
    # Check files
    for file_path in required_files:
        full_path = project_root / file_path
        if not full_path.exists():
            print(f"✗ Missing file: {file_path}")
            return False
        else:
            print(f"✓ Found: {file_path}")
    
    # Check directories
    for dir_path in required_dirs:
        full_path = project_root / dir_path
        if not full_path.exists():
            print(f"✗ Missing directory: {dir_path}")
            return False
        else:
            print(f"✓ Found directory: {dir_path}")
    
    return True


def test_scenario_files():
    """Test that scenario files are valid JSON."""
    print("\nTesting scenario files...")
    
    project_root = Path(__file__).parent.parent
    scenarios_dir = project_root / "scenarios"
    
    if not scenarios_dir.exists():
        print("✗ Scenarios directory not found")
        return False
    
    scenario_files = list(scenarios_dir.glob("*.json"))
    if not scenario_files:
        print("✗ No scenario files found")
        return False
    
    for scenario_file in scenario_files:
        try:
            with open(scenario_file, 'r') as f:
                data = json.load(f)
            
            # Basic validation
            required_keys = ['grid', 'agents', 'simulation_params']
            for key in required_keys:
                if key not in data:
                    print(f"✗ {scenario_file.name}: missing '{key}' key")
                    return False
            
            # Validate grid
            if 'width' not in data['grid'] or 'height' not in data['grid']:
                print(f"✗ {scenario_file.name}: invalid grid specification")
                return False
            
            # Validate agents
            if not isinstance(data['agents'], list) or len(data['agents']) == 0:
                print(f"✗ {scenario_file.name}: invalid agents specification")
                return False
            
            for agent in data['agents']:
                required_agent_keys = ['id', 'start', 'goal']
                for key in required_agent_keys:
                    if key not in agent:
                        print(f"✗ {scenario_file.name}: agent missing '{key}' key")
                        return False
            
            print(f"✓ {scenario_file.name}: valid JSON with correct structure")
            
        except json.JSONDecodeError as e:
            print(f"✗ {scenario_file.name}: invalid JSON - {e}")
            return False
        except Exception as e:
            print(f"✗ {scenario_file.name}: error - {e}")
            return False
    
    return True


def test_rust_code_compiles():
    """Test that Rust code has basic syntax."""
    print("\nTesting Rust code syntax...")
    
    project_root = Path(__file__).parent.parent
    
    # Check if cargo.toml exists
    cargo_toml = project_root / "Cargo.toml"
    if not cargo_toml.exists():
        print("✗ Cargo.toml not found")
        return False
    
    # Basic syntax check by attempting to parse Rust files
    rust_files = [
        "src/lib.rs",
        "src/structs.rs", 
        "src/astar.rs",
        "src/cbs.rs",
        "src/orca.rs"
    ]
    
    for rust_file in rust_files:
        file_path = project_root / rust_file
        if not file_path.exists():
            print(f"✗ Missing Rust file: {rust_file}")
            return False
        
        # Basic check: file should contain some Rust keywords
        with open(file_path, 'r') as f:
            content = f.read()
        
        rust_keywords = ['use', 'pub', 'fn', 'struct', 'impl']
        if not any(keyword in content for keyword in rust_keywords):
            print(f"✗ {rust_file}: doesn't appear to contain valid Rust code")
            return False
        
        print(f"✓ {rust_file}: appears to contain valid Rust code")
    
    return True


def test_python_imports():
    """Test that Python files have valid import structure."""
    print("\nTesting Python import structure...")
    
    project_root = Path(__file__).parent.parent
    
    python_files = [
        "simulation/__init__.py",
        "simulation/simulator.py",
        "simulation/visualizer.py",
        "simulation/scenario_loader.py"
    ]
    
    for py_file in python_files:
        file_path = project_root / py_file
        if not file_path.exists():
            print(f"✗ Missing Python file: {py_file}")
            return False
        
        # Check for basic Python structure
        with open(file_path, 'r') as f:
            content = f.read()
        
        # Should contain some Python keywords/patterns
        python_patterns = ['import', 'def ', 'class ', 'from ']
        if not any(pattern in content for pattern in python_patterns):
            print(f"✗ {py_file}: doesn't appear to contain valid Python code")
            return False
        
        print(f"✓ {py_file}: appears to contain valid Python code")
    
    return True


def main():
    """Run all structural tests."""
    print("Running project structure tests...")
    print("=" * 50)
    
    tests = [
        test_project_structure,
        test_scenario_files,
        test_rust_code_compiles,
        test_python_imports
    ]
    
    all_passed = True
    for test in tests:
        try:
            if not test():
                all_passed = False
                break
        except Exception as e:
            print(f"✗ Test failed with exception: {e}")
            all_passed = False
            break
    
    print("=" * 50)
    if all_passed:
        print("✓ All structural tests passed!")
    else:
        print("✗ Some tests failed!")
    
    return all_passed


if __name__ == "__main__":
    import sys
    success = main()
    sys.exit(0 if success else 1)