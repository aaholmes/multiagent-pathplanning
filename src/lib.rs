//! # Navigation Core
//!
//! A multi-agent pathfinding and collision avoidance library with Python bindings.
//!
//! ## Algorithms
//!
//! - **CBS (Conflict-Based Search)**: Optimal multi-agent pathfinding that finds
//!   collision-free paths for multiple agents on a grid.
//! - **ORCA (Optimal Reciprocal Collision Avoidance)**: Real-time local collision
//!   avoidance using velocity obstacles and quadratic programming.
//! - **A\***: Single-agent pathfinding with time-space constraints.
//!
//! ## Usage
//!
//! This library is primarily used through Python bindings via PyO3.
//! Core functions: `solve_cbs_py`, `compute_orca_velocity_py`.

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use std::collections::HashMap;

mod astar;
mod cbs;
mod orca;
mod structs;

use cbs::solve_cbs;
use orca::{compute_all_velocities, compute_new_velocity_with_obstacles};
use structs::*;

// ---------------------------------------------------------------------------
// Plain-Rust implementations.
//
// The #[pyfunction] wrappers below stay thin so the logic itself is directly
// unit-testable (the test module calls these same functions, not copies).
// ---------------------------------------------------------------------------

fn path_length_impl(path: &[Point]) -> f64 {
    if path.len() < 2 {
        return 0.0;
    }
    let mut total_length = 0.0;
    for i in 1..path.len() {
        total_length += path[i - 1].distance(&path[i]);
    }
    total_length
}

fn solution_cost_impl(solution: &HashMap<usize, Vec<Point>>) -> f64 {
    solution.values().map(|path| path_length_impl(path)).sum()
}

fn validate_solution_impl(solution: &HashMap<usize, Vec<Point>>, tasks: &[Task]) -> bool {
    for task in tasks {
        if let Some(path) = solution.get(&task.agent_id) {
            if path.is_empty() {
                return false;
            }
            if path[0].distance(&task.start) > 1.0 {
                return false;
            }
            if path.last().unwrap().distance(&task.goal) > 1.0 {
                return false;
            }
        } else {
            return false;
        }
    }
    true
}

fn check_grid_dimensions(width: usize, height: usize) -> Result<(), String> {
    if width == 0 || height == 0 {
        Err(format!(
            "grid dimensions must be positive, got {}x{}",
            width, height
        ))
    } else {
        Ok(())
    }
}

fn create_simple_scenario_impl(width: usize, height: usize) -> Result<(Grid, Vec<Task>), String> {
    check_grid_dimensions(width, height)?;
    let grid = Grid::new(width, height);
    let tasks = vec![
        Task::new(0, Point::new(0.0, 0.0), Point::new((width - 1) as f64, 0.0)),
        Task::new(1, Point::new((width - 1) as f64, 0.0), Point::new(0.0, 0.0)),
    ];
    Ok((grid, tasks))
}

fn create_four_corner_scenario_impl(
    width: usize,
    height: usize,
) -> Result<(Grid, Vec<Task>), String> {
    check_grid_dimensions(width, height)?;
    let grid = Grid::new(width, height);
    let w = (width - 1) as f64;
    let h = (height - 1) as f64;

    let tasks = vec![
        Task::new(0, Point::new(0.0, 0.0), Point::new(w, h)),
        Task::new(1, Point::new(w, 0.0), Point::new(0.0, h)),
        Task::new(2, Point::new(w, h), Point::new(0.0, 0.0)),
        Task::new(3, Point::new(0.0, h), Point::new(w, 0.0)),
    ];
    Ok((grid, tasks))
}

fn add_random_obstacles_impl(
    grid: &mut Grid,
    obstacle_count: usize,
    seed: u64,
) -> Result<(), String> {
    check_grid_dimensions(grid.width, grid.height)?;

    // Simple LCG random number generator for reproducibility
    let mut rng_state = seed;
    for _ in 0..obstacle_count {
        // Simple LCG: next = (a * current + c) % m
        rng_state = (1664525_u64.wrapping_mul(rng_state).wrapping_add(1013904223)) % (1 << 32);
        let x = (rng_state % grid.width as u64) as usize;

        rng_state = (1664525_u64.wrapping_mul(rng_state).wrapping_add(1013904223)) % (1 << 32);
        let y = (rng_state % grid.height as u64) as usize;

        // Don't place obstacles on corners (common start/goal positions)
        if !((x == 0 || x == grid.width - 1) && (y == 0 || y == grid.height - 1)) {
            grid.set_obstacle(x, y, true);
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// PyO3 bindings
// ---------------------------------------------------------------------------

#[pyfunction]
fn solve_cbs_py(grid: Grid, tasks: Vec<Task>) -> PyResult<Option<HashMap<usize, Vec<Point>>>> {
    Ok(solve_cbs(grid, tasks))
}

#[pyfunction]
#[pyo3(signature = (agent, neighbors, time_horizon, obstacles=None, obstacle_radius=0.5, time_horizon_obst=1.0))]
fn compute_orca_velocity_py(
    agent: &AgentState,
    neighbors: Vec<AgentState>,
    time_horizon: f64,
    obstacles: Option<Vec<Point>>,
    obstacle_radius: f64,
    time_horizon_obst: f64,
) -> PyResult<Vector2D> {
    if time_horizon <= 0.0 || time_horizon_obst <= 0.0 {
        return Err(PyValueError::new_err("time horizons must be positive"));
    }
    let neighbor_refs: Vec<&AgentState> = neighbors.iter().collect();
    Ok(compute_new_velocity_with_obstacles(
        agent,
        &neighbor_refs,
        &obstacles.unwrap_or_default(),
        obstacle_radius,
        time_horizon,
        time_horizon_obst,
    ))
}

/// Batched simulation step: one FFI call computes new velocities for all
/// agents, with native neighbor selection and grid-obstacle constraints.
#[pyfunction]
#[pyo3(signature = (agents, time_horizon, neighbor_distance, max_neighbors=10, grid=None, obstacle_radius=0.5, time_horizon_obst=1.0))]
fn compute_orca_velocities_py(
    agents: Vec<AgentState>,
    time_horizon: f64,
    neighbor_distance: f64,
    max_neighbors: usize,
    grid: Option<Grid>,
    obstacle_radius: f64,
    time_horizon_obst: f64,
) -> PyResult<Vec<Vector2D>> {
    if time_horizon <= 0.0 || time_horizon_obst <= 0.0 {
        return Err(PyValueError::new_err("time horizons must be positive"));
    }
    Ok(compute_all_velocities(
        &agents,
        grid.as_ref(),
        time_horizon,
        time_horizon_obst,
        neighbor_distance,
        max_neighbors,
        obstacle_radius,
    ))
}

#[pyfunction]
fn create_simple_scenario(width: usize, height: usize) -> PyResult<(Grid, Vec<Task>)> {
    create_simple_scenario_impl(width, height).map_err(PyValueError::new_err)
}

#[pyfunction]
fn create_four_corner_scenario(width: usize, height: usize) -> PyResult<(Grid, Vec<Task>)> {
    create_four_corner_scenario_impl(width, height).map_err(PyValueError::new_err)
}

#[pyfunction]
fn add_random_obstacles(
    mut grid: Grid,
    obstacle_count: usize,
    seed: Option<u64>,
) -> PyResult<Grid> {
    add_random_obstacles_impl(&mut grid, obstacle_count, seed.unwrap_or(42))
        .map_err(PyValueError::new_err)?;
    Ok(grid)
}

#[pyfunction]
fn path_length(path: Vec<Point>) -> PyResult<f64> {
    Ok(path_length_impl(&path))
}

#[pyfunction]
fn solution_cost(solution: HashMap<usize, Vec<Point>>) -> PyResult<f64> {
    Ok(solution_cost_impl(&solution))
}

#[pyfunction]
fn validate_solution(solution: HashMap<usize, Vec<Point>>, tasks: Vec<Task>) -> PyResult<bool> {
    Ok(validate_solution_impl(&solution, &tasks))
}

#[pymodule]
fn navigation_core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Core data structures
    m.add_class::<Point>()?;
    m.add_class::<Vector2D>()?;
    m.add_class::<AgentState>()?;
    m.add_class::<Task>()?;
    m.add_class::<Line>()?;
    m.add_class::<Grid>()?;

    // Core algorithms
    m.add_function(wrap_pyfunction!(solve_cbs_py, m)?)?;
    m.add_function(wrap_pyfunction!(compute_orca_velocity_py, m)?)?;
    m.add_function(wrap_pyfunction!(compute_orca_velocities_py, m)?)?;

    // Utility functions
    m.add_function(wrap_pyfunction!(create_simple_scenario, m)?)?;
    m.add_function(wrap_pyfunction!(create_four_corner_scenario, m)?)?;
    m.add_function(wrap_pyfunction!(add_random_obstacles, m)?)?;
    m.add_function(wrap_pyfunction!(path_length, m)?)?;
    m.add_function(wrap_pyfunction!(solution_cost, m)?)?;
    m.add_function(wrap_pyfunction!(validate_solution, m)?)?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    // These tests exercise the same functions the Python bindings call
    // (previously they tested verbatim copies, leaving the real code uncovered).

    // --- Tests for create_simple_scenario ---

    #[test]
    fn test_create_simple_scenario_grid_size() {
        let (grid, tasks) = create_simple_scenario_impl(10, 5).unwrap();

        assert_eq!(grid.width, 10, "Grid width should match");
        assert_eq!(grid.height, 5, "Grid height should match");
        assert_eq!(tasks.len(), 2, "Should create 2 tasks");
    }

    #[test]
    fn test_create_simple_scenario_task_positions() {
        let (_, tasks) = create_simple_scenario_impl(10, 5).unwrap();

        // Agent 0: starts at (0,0), goes to (9,0)
        assert_eq!(tasks[0].agent_id, 0);
        assert!((tasks[0].start.x - 0.0).abs() < 0.01);
        assert!((tasks[0].goal.x - 9.0).abs() < 0.01);

        // Agent 1: starts at (9,0), goes to (0,0)
        assert_eq!(tasks[1].agent_id, 1);
        assert!((tasks[1].start.x - 9.0).abs() < 0.01);
        assert!((tasks[1].goal.x - 0.0).abs() < 0.01);
    }

    #[test]
    fn test_create_simple_scenario_zero_size_is_error() {
        // Regression: width 0 previously wrapped (width - 1) as usize and
        // silently returned a goal at ~1.8e19.
        assert!(create_simple_scenario_impl(0, 5).is_err());
        assert!(create_simple_scenario_impl(5, 0).is_err());
    }

    // --- Tests for create_four_corner_scenario ---

    #[test]
    fn test_create_four_corner_scenario_task_count() {
        let (grid, tasks) = create_four_corner_scenario_impl(8, 8).unwrap();

        assert_eq!(grid.width, 8);
        assert_eq!(grid.height, 8);
        assert_eq!(tasks.len(), 4, "Should create 4 tasks for four corners");
    }

    #[test]
    fn test_create_four_corner_scenario_diagonal_goals() {
        let (_, tasks) = create_four_corner_scenario_impl(10, 10).unwrap();

        // Agent 0: (0,0) -> (9,9)
        assert!((tasks[0].start.x - 0.0).abs() < 0.01);
        assert!((tasks[0].start.y - 0.0).abs() < 0.01);
        assert!((tasks[0].goal.x - 9.0).abs() < 0.01);
        assert!((tasks[0].goal.y - 9.0).abs() < 0.01);

        // Agent 2: (9,9) -> (0,0) - opposite of agent 0
        assert!((tasks[2].start.x - 9.0).abs() < 0.01);
        assert!((tasks[2].goal.x - 0.0).abs() < 0.01);
    }

    #[test]
    fn test_create_four_corner_scenario_zero_size_is_error() {
        assert!(create_four_corner_scenario_impl(0, 8).is_err());
    }

    // --- Tests for add_random_obstacles ---

    #[test]
    fn test_add_random_obstacles_deterministic() {
        // Same seed should produce same obstacles
        let mut grid1 = Grid::new(10, 10);
        let mut grid2 = Grid::new(10, 10);

        add_random_obstacles_impl(&mut grid1, 5, 42).unwrap();
        add_random_obstacles_impl(&mut grid2, 5, 42).unwrap();

        // Count obstacles in both grids - should be identical
        let mut count1 = 0;
        let mut count2 = 0;
        for x in 0..10 {
            for y in 0..10 {
                if grid1.is_obstacle(x, y) {
                    count1 += 1;
                }
                if grid2.is_obstacle(x, y) {
                    count2 += 1;
                }
            }
        }
        assert_eq!(
            count1, count2,
            "Same seed should produce same obstacle count"
        );
    }

    #[test]
    fn test_add_random_obstacles_preserves_corners() {
        let mut grid = Grid::new(10, 10);
        add_random_obstacles_impl(&mut grid, 100, 42).unwrap();

        // Corners should never have obstacles
        assert!(!grid.is_obstacle(0, 0), "Corner (0,0) should be clear");
        assert!(!grid.is_obstacle(9, 0), "Corner (9,0) should be clear");
        assert!(!grid.is_obstacle(0, 9), "Corner (0,9) should be clear");
        assert!(!grid.is_obstacle(9, 9), "Corner (9,9) should be clear");
    }

    #[test]
    fn test_add_random_obstacles_zero_count() {
        let mut grid = Grid::new(5, 5);
        add_random_obstacles_impl(&mut grid, 0, 42).unwrap();

        // No obstacles should be added
        for x in 0..5 {
            for y in 0..5 {
                assert!(!grid.is_obstacle(x, y), "No obstacles should be added");
            }
        }
    }

    #[test]
    fn test_add_random_obstacles_zero_size_grid_is_error() {
        // Regression: a zero-width grid previously caused a remainder-by-zero
        // panic that crossed the FFI boundary as a PanicException.
        let mut grid = Grid::new(0, 5);
        assert!(add_random_obstacles_impl(&mut grid, 3, 42).is_err());
    }

    // --- Tests for path_length ---

    #[test]
    fn test_path_length_empty() {
        let path: Vec<Point> = vec![];
        let length = path_length_impl(&path);
        assert!(
            (length - 0.0).abs() < 0.001,
            "Empty path should have zero length"
        );
    }

    #[test]
    fn test_path_length_single_point() {
        let path = vec![Point::new(0.0, 0.0)];
        let length = path_length_impl(&path);
        assert!(
            (length - 0.0).abs() < 0.001,
            "Single point path should have zero length"
        );
    }

    #[test]
    fn test_path_length_horizontal() {
        let path = vec![Point::new(0.0, 0.0), Point::new(5.0, 0.0)];
        let length = path_length_impl(&path);
        assert!(
            (length - 5.0).abs() < 0.001,
            "Horizontal path length should be 5.0"
        );
    }

    #[test]
    fn test_path_length_multi_segment() {
        let path = vec![
            Point::new(0.0, 0.0),
            Point::new(3.0, 0.0),
            Point::new(3.0, 4.0),
        ];
        let length = path_length_impl(&path);
        // 3 + 4 = 7
        assert!(
            (length - 7.0).abs() < 0.001,
            "Multi-segment path length should be 7.0"
        );
    }

    // --- Tests for solution_cost ---

    #[test]
    fn test_solution_cost_empty() {
        let solution: HashMap<usize, Vec<Point>> = HashMap::new();
        let cost = solution_cost_impl(&solution);
        assert!(
            (cost - 0.0).abs() < 0.001,
            "Empty solution should have zero cost"
        );
    }

    #[test]
    fn test_solution_cost_single_agent() {
        let mut solution = HashMap::new();
        solution.insert(0, vec![Point::new(0.0, 0.0), Point::new(4.0, 0.0)]);
        let cost = solution_cost_impl(&solution);
        assert!(
            (cost - 4.0).abs() < 0.001,
            "Single agent cost should be 4.0"
        );
    }

    #[test]
    fn test_solution_cost_multiple_agents() {
        let mut solution = HashMap::new();
        solution.insert(0, vec![Point::new(0.0, 0.0), Point::new(3.0, 0.0)]);
        solution.insert(1, vec![Point::new(0.0, 0.0), Point::new(0.0, 4.0)]);
        let cost = solution_cost_impl(&solution);
        // 3 + 4 = 7
        assert!((cost - 7.0).abs() < 0.001, "Total cost should be 7.0");
    }

    // --- Tests for validate_solution ---

    #[test]
    fn test_validate_solution_valid() {
        let mut solution = HashMap::new();
        solution.insert(
            0,
            vec![
                Point::new(0.0, 0.0),
                Point::new(1.0, 0.0),
                Point::new(2.0, 0.0),
            ],
        );

        let tasks = vec![Task::new(0, Point::new(0.0, 0.0), Point::new(2.0, 0.0))];

        assert!(
            validate_solution_impl(&solution, &tasks),
            "Valid solution should pass validation"
        );
    }

    #[test]
    fn test_validate_solution_wrong_start() {
        let mut solution = HashMap::new();
        solution.insert(
            0,
            vec![
                Point::new(5.0, 5.0), // Wrong start
                Point::new(2.0, 0.0),
            ],
        );

        let tasks = vec![Task::new(0, Point::new(0.0, 0.0), Point::new(2.0, 0.0))];

        assert!(
            !validate_solution_impl(&solution, &tasks),
            "Solution with wrong start should fail"
        );
    }

    #[test]
    fn test_validate_solution_wrong_goal() {
        let mut solution = HashMap::new();
        solution.insert(
            0,
            vec![
                Point::new(0.0, 0.0),
                Point::new(9.0, 9.0), // Wrong goal
            ],
        );

        let tasks = vec![Task::new(0, Point::new(0.0, 0.0), Point::new(2.0, 0.0))];

        assert!(
            !validate_solution_impl(&solution, &tasks),
            "Solution with wrong goal should fail"
        );
    }

    #[test]
    fn test_validate_solution_missing_agent() {
        let solution: HashMap<usize, Vec<Point>> = HashMap::new(); // No paths

        let tasks = vec![Task::new(0, Point::new(0.0, 0.0), Point::new(2.0, 0.0))];

        assert!(
            !validate_solution_impl(&solution, &tasks),
            "Solution missing agent should fail"
        );
    }

    #[test]
    fn test_validate_solution_empty_path() {
        let mut solution = HashMap::new();
        solution.insert(0, vec![]); // Empty path

        let tasks = vec![Task::new(0, Point::new(0.0, 0.0), Point::new(2.0, 0.0))];

        assert!(
            !validate_solution_impl(&solution, &tasks),
            "Solution with empty path should fail"
        );
    }
}
