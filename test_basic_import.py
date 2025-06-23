#!/usr/bin/env python3
"""
Basic test to verify the Rust library can be imported.
Run this after building with: maturin develop --release
"""

def test_navigation_core_import():
    """Test that the navigation_core library can be imported and basic functions work."""
    try:
        import navigation_core
        print("✅ Successfully imported navigation_core")
        
        # Test basic data structures
        point = navigation_core.Point(1.0, 2.0)
        print(f"✅ Created Point: {point}")
        
        vector = navigation_core.Vector2D(3.0, 4.0)
        print(f"✅ Created Vector2D: {vector}")
        print(f"   Vector magnitude: {vector.magnitude():.2f}")
        
        # Test grid
        grid = navigation_core.Grid(10, 10)
        print(f"✅ Created Grid: {grid}")
        grid.set_obstacle(5, 5, True)
        print(f"   Set obstacle at (5,5): {grid.is_obstacle(5, 5)}")
        
        # Test agent state
        agent = navigation_core.AgentState(
            0, point, vector, 0.5, vector, 2.0
        )
        print(f"✅ Created AgentState: {agent}")
        
        # Test task
        start = navigation_core.Point(0.0, 0.0)
        goal = navigation_core.Point(9.0, 9.0)
        task = navigation_core.Task(0, start, goal)
        print(f"✅ Created Task: {task}")
        
        # Test scenario creation
        grid, tasks = navigation_core.create_simple_scenario(10, 10)
        print(f"✅ Created simple scenario with {len(tasks)} agents")
        
        print("\n🎉 All basic tests passed! The Rust library is working correctly.")
        return True
        
    except ImportError as e:
        print(f"❌ Failed to import navigation_core: {e}")
        print("\n💡 To fix this, run: maturin develop --release")
        return False
    except Exception as e:
        print(f"❌ Error testing navigation_core: {e}")
        return False

def test_cbs_basic():
    """Test basic CBS functionality."""
    try:
        import navigation_core
        
        # Create a simple 2-agent scenario
        grid = navigation_core.Grid(5, 5)
        tasks = [
            navigation_core.Task(0, navigation_core.Point(0.0, 0.0), navigation_core.Point(4.0, 0.0)),
            navigation_core.Task(1, navigation_core.Point(4.0, 0.0), navigation_core.Point(0.0, 0.0))
        ]
        
        print("🔍 Testing CBS solver...")
        solution = navigation_core.solve_cbs_py(grid, tasks)
        
        if solution is not None:
            print(f"✅ CBS found solution for {len(solution)} agents")
            for agent_id, path in solution.items():
                print(f"   Agent {agent_id}: path length = {len(path)}")
            return True
        else:
            print("❌ CBS failed to find solution")
            return False
            
    except Exception as e:
        print(f"❌ CBS test failed: {e}")
        return False

def main():
    print("Testing Multi-Agent Navigation System")
    print("=" * 40)
    
    # Test basic import
    if not test_navigation_core_import():
        return False
    
    print("\n" + "=" * 40)
    
    # Test CBS
    if not test_cbs_basic():
        return False
    
    print("\n" + "=" * 40)
    print("🎉 All tests completed successfully!")
    print("\nNext steps:")
    print("1. Install Python dependencies: pip install numpy matplotlib")
    print("2. Run full simulation: python3 simulation/run_simulation.py --scenario scenarios/simple_2_agents.json")
    
    return True

if __name__ == "__main__":
    import sys
    success = main()
    sys.exit(0 if success else 1)