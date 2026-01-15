#!/usr/bin/env python3
"""
Performance benchmark for the multi-agent navigation stack.

This script measures the throughput of both CBS (global planning) and
ORCA (local collision avoidance) algorithms across varying agent counts.

Usage:
    python benchmark.py              # Run all benchmarks
    python benchmark.py --orca-only  # Run only ORCA benchmarks
    python benchmark.py --cbs-only   # Run only CBS benchmarks
"""

import argparse
import time
import sys
from typing import List, Tuple

def check_dependencies():
    """Verify that the Rust library is available."""
    try:
        import navigation_core
        return True
    except ImportError:
        print("Error: navigation_core not found.")
        print("Please build the Rust library first:")
        print("  maturin develop --release")
        return False


def benchmark_orca(agent_counts: List[int], iterations: int = 100) -> List[Tuple[int, float]]:
    """
    Benchmark ORCA collision avoidance performance.

    Args:
        agent_counts: List of agent counts to test
        iterations: Number of iterations per agent count

    Returns:
        List of (agent_count, agents_per_second) tuples
    """
    import navigation_core as nc

    results = []

    for num_agents in agent_counts:
        # Create agents in a grid pattern
        agents = []
        grid_size = int(num_agents ** 0.5) + 1

        for i in range(num_agents):
            x = (i % grid_size) * 2.0
            y = (i // grid_size) * 2.0

            agent = nc.AgentState(
                id=i,
                position=nc.Point(x, y),
                velocity=nc.Vector2D(0.0, 0.0),
                radius=0.3,
                max_speed=1.5,
                pref_velocity=nc.Vector2D(1.0, 0.0)
            )
            agents.append(agent)

        # Warm up
        for agent in agents[:min(10, len(agents))]:
            neighbors = [a for a in agents if a.id != agent.id][:10]
            nc.compute_orca_velocity_py(agent, neighbors, 2.0)

        # Benchmark
        total_computations = 0
        start_time = time.perf_counter()

        for _ in range(iterations):
            for agent in agents:
                # Each agent considers up to 10 nearest neighbors
                neighbors = [a for a in agents if a.id != agent.id][:10]
                nc.compute_orca_velocity_py(agent, neighbors, 2.0)
                total_computations += 1

        elapsed = time.perf_counter() - start_time
        agents_per_second = total_computations / elapsed

        results.append((num_agents, agents_per_second))
        print(f"  ORCA: {num_agents:4d} agents -> {agents_per_second:,.0f} velocity computations/sec")

    return results


def benchmark_cbs(agent_counts: List[int], grid_size: int = 20) -> List[Tuple[int, float, bool]]:
    """
    Benchmark CBS global path planning performance.

    Args:
        agent_counts: List of agent counts to test
        grid_size: Size of the grid environment

    Returns:
        List of (agent_count, solve_time_ms, solved) tuples
    """
    import navigation_core as nc

    results = []

    for num_agents in agent_counts:
        grid = nc.Grid(grid_size, grid_size)

        # Create tasks with agents starting on left, goals on right
        tasks = []
        for i in range(num_agents):
            y = i % grid_size
            start = nc.Point(0.0, float(y))
            goal = nc.Point(float(grid_size - 1), float((y + num_agents // 2) % grid_size))
            tasks.append(nc.Task(i, start, goal))

        # Benchmark with timeout
        start_time = time.perf_counter()
        try:
            solution = nc.solve_cbs_py(grid, tasks)
            elapsed_ms = (time.perf_counter() - start_time) * 1000
            solved = solution is not None
        except Exception:
            elapsed_ms = (time.perf_counter() - start_time) * 1000
            solved = False

        results.append((num_agents, elapsed_ms, solved))
        status = "solved" if solved else "failed/timeout"
        print(f"  CBS: {num_agents:4d} agents -> {elapsed_ms:8.2f} ms ({status})")

    return results


def print_summary(orca_results: List[Tuple[int, float]],
                  cbs_results: List[Tuple[int, float, bool]]):
    """Print a summary of benchmark results."""
    print("\n" + "=" * 60)
    print("BENCHMARK SUMMARY")
    print("=" * 60)

    if orca_results:
        print("\nORCA Performance:")
        print("-" * 40)
        avg_throughput = sum(r[1] for r in orca_results) / len(orca_results)
        max_throughput = max(r[1] for r in orca_results)
        print(f"  Average throughput: {avg_throughput:,.0f} computations/sec")
        print(f"  Peak throughput:    {max_throughput:,.0f} computations/sec")

    if cbs_results:
        print("\nCBS Performance:")
        print("-" * 40)
        solved = [r for r in cbs_results if r[2]]
        if solved:
            avg_time = sum(r[1] for r in solved) / len(solved)
            max_agents = max(r[0] for r in solved)
            print(f"  Average solve time: {avg_time:.2f} ms")
            print(f"  Max agents solved:  {max_agents}")
        else:
            print("  No problems solved within timeout")

    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description="Benchmark multi-agent navigation algorithms"
    )
    parser.add_argument("--orca-only", action="store_true",
                        help="Run only ORCA benchmarks")
    parser.add_argument("--cbs-only", action="store_true",
                        help="Run only CBS benchmarks")
    parser.add_argument("--quick", action="store_true",
                        help="Run quick benchmarks with fewer iterations")
    args = parser.parse_args()

    if not check_dependencies():
        sys.exit(1)

    print("=" * 60)
    print("Multi-Agent Navigation Stack - Performance Benchmark")
    print("=" * 60)

    orca_results = []
    cbs_results = []

    if args.quick:
        orca_agents = [10, 50, 100]
        cbs_agents = [2, 4, 6]
        orca_iterations = 20
    else:
        orca_agents = [10, 25, 50, 100, 200, 500]
        cbs_agents = [2, 4, 6, 8, 10, 12]
        orca_iterations = 100

    # Run ORCA benchmarks
    if not args.cbs_only:
        print("\nBenchmarking ORCA (local collision avoidance)...")
        print("-" * 40)
        orca_results = benchmark_orca(orca_agents, iterations=orca_iterations)

    # Run CBS benchmarks
    if not args.orca_only:
        print("\nBenchmarking CBS (global path planning)...")
        print("-" * 40)
        cbs_results = benchmark_cbs(cbs_agents)

    # Print summary
    print_summary(orca_results, cbs_results)


if __name__ == "__main__":
    main()
