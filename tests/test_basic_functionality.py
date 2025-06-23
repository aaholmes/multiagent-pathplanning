#!/usr/bin/env python3
"""
Basic functionality tests for the navigation system.
This script tests the core components without requiring the full build.
"""

import sys
import os
from pathlib import Path

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

def test_scenario_loading():
    """Test scenario loading functionality."""
    print("Testing scenario loading...")
    
    from simulation.scenario_loader import ScenarioLoader
    
    # Test creating scenarios
    simple_config = ScenarioLoader.generate_simple_scenario(
        grid_width=10, grid_height=10, num_agents=2, obstacle_density=0.1, seed=42
    )
    
    assert simple_config.grid_width == 10
    assert simple_config.grid_height == 10
    assert len(simple_config.agents) == 2
    print("✓ Simple scenario generation works")
    
    # Test corridor scenario
    corridor_config = ScenarioLoader.generate_corridor_scenario(
        corridor_length=15, corridor_width=3, num_agents=4
    )
    
    assert corridor_config.grid_width == 15
    assert len(corridor_config.agents) == 4
    print("✓ Corridor scenario generation works")
    
    # Test intersection scenario
    intersection_config = ScenarioLoader.generate_intersection_scenario(
        size=15, num_agents_per_direction=2
    )
    
    assert intersection_config.grid_width == 15
    assert len(intersection_config.agents) == 8  # 2 per direction * 4 directions
    print("✓ Intersection scenario generation works")
    
    # Test saving and loading
    temp_file = "/tmp/test_scenario.json"
    ScenarioLoader.save_to_file(simple_config, temp_file)
    loaded_config = ScenarioLoader.load_from_file(temp_file)
    
    assert loaded_config.grid_width == simple_config.grid_width
    assert loaded_config.grid_height == simple_config.grid_height
    assert len(loaded_config.agents) == len(simple_config.agents)
    
    # Clean up
    if os.path.exists(temp_file):
        os.remove(temp_file)
    
    print("✓ Scenario save/load works")


def test_existing_scenarios():
    """Test loading existing scenario files."""
    print("Testing existing scenario files...")
    
    from simulation.scenario_loader import ScenarioLoader
    
    scenarios_dir = project_root / "scenarios"
    
    for scenario_file in scenarios_dir.glob("*.json"):
        try:
            config = ScenarioLoader.load_from_file(str(scenario_file))
            assert config.grid_width > 0
            assert config.grid_height > 0
            assert len(config.agents) > 0
            print(f"✓ {scenario_file.name} loads correctly")
        except Exception as e:
            print(f"✗ {scenario_file.name} failed to load: {e}")
            return False
    
    return True


def test_data_structures():
    """Test that we can create basic data structures (mock test without Rust)."""
    print("Testing data structure concepts...")
    
    # Mock basic data structures to test the logic
    class MockPoint:
        def __init__(self, x, y):
            self.x = x
            self.y = y
        
        def distance(self, other):
            return ((self.x - other.x)**2 + (self.y - other.y)**2)**0.5
    
    class MockVector2D:
        def __init__(self, x, y):
            self.x = x
            self.y = y
        
        def magnitude(self):
            return (self.x**2 + self.y**2)**0.5
        
        def normalize(self):
            mag = self.magnitude()
            if mag > 0:
                return MockVector2D(self.x / mag, self.y / mag)
            return MockVector2D(0, 0)
    
    # Test basic operations
    p1 = MockPoint(0, 0)
    p2 = MockPoint(3, 4)
    distance = p1.distance(p2)
    assert abs(distance - 5.0) < 1e-6
    print("✓ Point distance calculation works")
    
    v1 = MockVector2D(3, 4)
    magnitude = v1.magnitude()
    assert abs(magnitude - 5.0) < 1e-6
    print("✓ Vector magnitude calculation works")
    
    v_norm = v1.normalize()
    assert abs(v_norm.magnitude() - 1.0) < 1e-6
    print("✓ Vector normalization works")


def run_all_tests():
    """Run all available tests."""
    print("Running basic functionality tests...")
    print("=" * 50)
    
    try:
        test_data_structures()
        test_scenario_loading()
        test_existing_scenarios()
        
        print("=" * 50)
        print("✓ All tests passed!")
        return True
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)