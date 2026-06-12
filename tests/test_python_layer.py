"""
Pytest suite for the Python layer: bindings, scenario loading, simulator
behavior, and visualization smoke tests.

Requires the built extension (`maturin develop --release`).
Run with: pytest tests/test_python_layer.py -q
"""

import math
import os
import sys
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

nc = pytest.importorskip("navigation_core")

from simulation import ScenarioLoader, Simulator  # noqa: E402

SCENARIOS = [
    "scenarios/simple_2_agents.json",
    "scenarios/4_agents_crossing.json",
    "scenarios/complex_maze.json",
    # The circle-swap experiment from the ORCA paper (ORCA-only, no CBS)
    "scenarios/circle_12_agents.json",
]


# ---------------------------------------------------------------------------
# Bindings
# ---------------------------------------------------------------------------


class TestBindings:
    def test_solve_cbs_head_on_swap_is_optimal(self):
        # Two agents swapping on an open 5x5 grid: the optimal resolution is
        # one single-cell detour, 5 + 7 = 12 total waypoints.
        grid = nc.Grid(5, 5)
        tasks = [
            nc.Task(0, nc.Point(0.0, 0.0), nc.Point(4.0, 0.0)),
            nc.Task(1, nc.Point(4.0, 0.0), nc.Point(0.0, 0.0)),
        ]
        solution = nc.solve_cbs_py(grid, tasks)
        assert solution is not None
        assert sum(len(p) for p in solution.values()) == 12
        assert nc.validate_solution(solution, tasks)

    def test_solve_cbs_unsolvable_returns_none(self):
        grid = nc.Grid(3, 3)
        for x in range(3):
            grid.set_obstacle(x, 1, True)  # wall splits the grid
        tasks = [nc.Task(0, nc.Point(0.0, 0.0), nc.Point(0.0, 2.0))]
        assert nc.solve_cbs_py(grid, tasks) is None

    def test_zero_size_grid_raises_value_error(self):
        with pytest.raises(ValueError):
            nc.create_simple_scenario(0, 5)
        with pytest.raises(ValueError):
            nc.add_random_obstacles(nc.Grid(0, 5), 3, 42)

    def test_orca_no_neighbors_returns_preference(self):
        agent = nc.AgentState(
            0, nc.Point(0.0, 0.0), nc.Vector2D(0.0, 0.0), 0.5, nc.Vector2D(1.0, 0.0), 2.0
        )
        v = nc.compute_orca_velocity_py(agent, [], 2.0)
        assert math.isclose(v.x, 1.0, abs_tol=1e-9)
        assert math.isclose(v.y, 0.0, abs_tol=1e-9)

    def test_orca_invalid_horizon_raises(self):
        agent = nc.AgentState(
            0, nc.Point(0.0, 0.0), nc.Vector2D(0.0, 0.0), 0.5, nc.Vector2D(1.0, 0.0), 2.0
        )
        with pytest.raises(ValueError):
            nc.compute_orca_velocity_py(agent, [], 0.0)

    def test_orca_obstacle_blocks_forward_motion(self):
        # Agent driving straight at a wall of obstacle discs must deviate.
        agent = nc.AgentState(
            0, nc.Point(0.0, 0.0), nc.Vector2D(1.0, 0.0), 0.4, nc.Vector2D(1.0, 0.0), 1.0
        )
        wall = [nc.Point(1.0, -1.0), nc.Point(1.0, 0.0), nc.Point(1.0, 1.0)]
        v = nc.compute_orca_velocity_py(agent, [], 2.0, obstacles=wall)
        assert v.x < 0.5, f"agent should not keep driving into the wall, got ({v.x}, {v.y})"

    def test_batched_call_matches_per_agent_calls(self):
        agents = [
            nc.AgentState(
                0, nc.Point(0.0, 0.0), nc.Vector2D(1.0, 0.0), 0.4, nc.Vector2D(1.0, 0.0), 1.0
            ),
            nc.AgentState(
                1, nc.Point(4.0, 0.0), nc.Vector2D(-1.0, 0.0), 0.4, nc.Vector2D(-1.0, 0.0), 1.0
            ),
        ]
        batched = nc.compute_orca_velocities_py(agents, 2.0, 10.0)
        for agent, expected in zip(agents, batched):
            neighbors = [a for a in agents if a.id != agent.id]
            single = nc.compute_orca_velocity_py(agent, neighbors, 2.0)
            assert math.isclose(single.x, expected.x, abs_tol=1e-9)
            assert math.isclose(single.y, expected.y, abs_tol=1e-9)


# ---------------------------------------------------------------------------
# Scenario loading
# ---------------------------------------------------------------------------


class TestScenarioLoader:
    def test_load_bundled_scenarios(self):
        for scenario in SCENARIOS:
            config = ScenarioLoader.load_from_file(str(PROJECT_ROOT / scenario))
            assert config.grid_width > 0 and config.grid_height > 0
            assert len(config.agents) >= 1

    def test_save_load_round_trip(self, tmp_path):
        config = ScenarioLoader.generate_simple_scenario(
            grid_width=12, grid_height=9, num_agents=3, obstacle_density=0.1, seed=7
        )
        path = tmp_path / "scenario.json"
        ScenarioLoader.save_to_file(config, str(path))
        loaded = ScenarioLoader.load_from_file(str(path))
        assert loaded.grid_width == 12 and loaded.grid_height == 9
        assert len(loaded.agents) == 3
        assert sorted(map(tuple, loaded.obstacles)) == sorted(map(tuple, config.obstacles))

    def test_generation_is_seeded(self):
        a = ScenarioLoader.generate_simple_scenario(num_agents=4, seed=42)
        b = ScenarioLoader.generate_simple_scenario(num_agents=4, seed=42)
        assert a.obstacles == b.obstacles
        assert [ag["start"] for ag in a.agents] == [ag["start"] for ag in b.agents]

    def test_config_to_core_objects(self):
        config = ScenarioLoader.load_from_file(str(PROJECT_ROOT / SCENARIOS[0]))
        grid = ScenarioLoader.create_grid_from_config(config)
        tasks = ScenarioLoader.create_tasks_from_config(config)
        agents = ScenarioLoader.create_agent_states_from_config(config)
        assert grid.width == config.grid_width
        assert len(tasks) == len(agents) == len(config.agents)
        for x, y in config.obstacles:
            assert grid.is_obstacle(x, y)


# ---------------------------------------------------------------------------
# Simulator end-to-end
# ---------------------------------------------------------------------------


def run_with_history(scenario):
    sim = Simulator(ScenarioLoader.load_from_file(str(PROJECT_ROOT / scenario)))
    states = []
    sim.set_step_callback(states.append)
    final = sim.run()
    return sim, final, states


class TestSimulator:
    @pytest.mark.parametrize("scenario", SCENARIOS)
    def test_all_agents_reach_goals(self, scenario):
        sim, final, _ = run_with_history(scenario)
        stats = sim.get_statistics(final)
        assert stats["summary"]["success_rate"] == 1.0

    @pytest.mark.parametrize("scenario", SCENARIOS)
    def test_no_collisions_during_rollout(self, scenario):
        # Pairwise separation at every step, with the same 10% tolerance the
        # Rust rollout tests use for symmetric squeeze situations.
        _, _, states = run_with_history(scenario)
        for state in states:
            agents = state.agents
            for i in range(len(agents)):
                for j in range(i + 1, len(agents)):
                    distance = agents[i].position.distance(agents[j].position)
                    min_distance = agents[i].radius + agents[j].radius
                    assert distance >= 0.9 * min_distance, (
                        f"collision at t={state.time:.2f}: agents {agents[i].id} and "
                        f"{agents[j].id} are {distance:.3f} apart (min {min_distance})"
                    )

    @pytest.mark.parametrize("scenario", SCENARIOS)
    def test_agents_never_enter_obstacle_cells(self, scenario):
        sim, final, _ = run_with_history(scenario)
        for trajectory in final.agent_trajectories.values():
            for p in trajectory:
                cx, cy = int(round(p.x)), int(round(p.y))
                assert not sim.grid.is_obstacle(cx, cy), (
                    f"agent center entered obstacle cell ({cx}, {cy})"
                )

    def test_states_do_not_alias_trajectories(self):
        # Regression: stored states used to share trajectory lists, so every
        # earlier state retroactively grew to the final trajectory.
        _, _, states = run_with_history(SCENARIOS[0])
        first, last = states[0], states[-1]
        assert len(first.agent_trajectories[0]) < len(last.agent_trajectories[0])

    def test_statistics_shape(self):
        sim, final, _ = run_with_history(SCENARIOS[0])
        stats = sim.get_statistics(final)
        assert {"total_time", "total_steps", "agents", "summary"} <= set(stats)
        assert {"success_rate", "total_path_length", "agents_at_goal"} <= set(
            stats["summary"]
        )
        for agent_stats in stats["agents"].values():
            assert agent_stats["path_length"] >= 0.0


# ---------------------------------------------------------------------------
# Visualizer smoke test
# ---------------------------------------------------------------------------


class TestVisualizer:
    def test_save_frame_produces_image(self, tmp_path):
        import matplotlib

        matplotlib.use("Agg")
        from simulation import Visualizer

        sim, final, _ = run_with_history(SCENARIOS[0])
        config = ScenarioLoader.load_from_file(str(PROJECT_ROOT / SCENARIOS[0]))
        visualizer = Visualizer(config)
        out = tmp_path / "frame.png"
        visualizer.save_frame(str(out), final)
        assert out.exists() and os.path.getsize(out) > 1000
