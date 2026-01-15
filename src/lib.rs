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

use pyo3::prelude::*;
use std::collections::HashMap;

mod structs;
mod astar;
mod cbs;
mod orca;          // Correct ORCA from original paper with QP optimization
mod orca_simple;   // Simple greedy projection (approximate but faster)

use structs::*;
use cbs::solve_cbs;
use orca::compute_new_velocity as compute_orca_correct;

#[pyfunction]
fn solve_cbs_py(grid: Grid, tasks: Vec<Task>) -> PyResult<Option<HashMap<usize, Vec<Point>>>> {
    let result = solve_cbs(grid, tasks);
    Ok(result)
}

#[pyfunction]
fn compute_orca_velocity_py(
    agent: &AgentState,
    neighbors: Vec<AgentState>,
    time_horizon: f64,
) -> PyResult<Vector2D> {
    let neighbor_refs: Vec<&AgentState> = neighbors.iter().collect();
    let result = compute_orca_correct(agent, &neighbor_refs, time_horizon);
    Ok(result)
}

#[pyfunction]
fn create_simple_scenario(width: usize, height: usize) -> PyResult<(Grid, Vec<Task>)> {
    let grid = Grid::new(width, height);
    let tasks = vec![
        Task::new(0, Point::new(0.0, 0.0), Point::new((width - 1) as f64, 0.0)),
        Task::new(1, Point::new((width - 1) as f64, 0.0), Point::new(0.0, 0.0)),
    ];
    Ok((grid, tasks))
}

#[pyfunction]
fn create_four_corner_scenario(width: usize, height: usize) -> PyResult<(Grid, Vec<Task>)> {
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

#[pyfunction]
fn add_random_obstacles(mut grid: Grid, obstacle_count: usize, seed: Option<u64>) -> PyResult<Grid> {
    
    // Simple LCG random number generator for reproducibility
    let mut rng_state = seed.unwrap_or(42);
    
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
    
    Ok(grid)
}

#[pyfunction]
fn path_length(path: Vec<Point>) -> PyResult<f64> {
    if path.len() < 2 {
        return Ok(0.0);
    }
    
    let mut total_length = 0.0;
    for i in 1..path.len() {
        total_length += path[i-1].distance(&path[i]);
    }
    
    Ok(total_length)
}

#[pyfunction]
fn solution_cost(solution: HashMap<usize, Vec<Point>>) -> PyResult<f64> {
    let mut total_cost = 0.0;
    for path in solution.values() {
        total_cost += path_length(path.clone())?;
    }
    Ok(total_cost)
}

#[pyfunction]
fn validate_solution(solution: HashMap<usize, Vec<Point>>, tasks: Vec<Task>) -> PyResult<bool> {
    for task in tasks {
        if let Some(path) = solution.get(&task.agent_id) {
            if path.is_empty() {
                return Ok(false);
            }
            
            // Check start position
            let start_dist = path[0].distance(&task.start);
            if start_dist > 1.0 {
                return Ok(false);
            }
            
            // Check goal position
            let goal_dist = path.last().unwrap().distance(&task.goal);
            if goal_dist > 1.0 {
                return Ok(false);
            }
        } else {
            return Ok(false);
        }
    }
    
    Ok(true)
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

    // --- Test helper functions (internal only) ---

    fn compute_path_length(path: &[Point]) -> f64 {
        if path.len() < 2 {
            return 0.0;
        }
        let mut total_length = 0.0;
        for i in 1..path.len() {
            total_length += path[i - 1].distance(&path[i]);
        }
        total_length
    }

    fn compute_solution_cost(solution: &HashMap<usize, Vec<Point>>) -> f64 {
        solution.values().map(|path| compute_path_length(path)).sum()
    }

    fn is_solution_valid(solution: &HashMap<usize, Vec<Point>>, tasks: &[Task]) -> bool {
        for task in tasks {
            if let Some(path) = solution.get(&task.agent_id) {
                if path.is_empty() {
                    return false;
                }
                let start_dist = path[0].distance(&task.start);
                if start_dist > 1.0 {
                    return false;
                }
                let goal_dist = path.last().unwrap().distance(&task.goal);
                if goal_dist > 1.0 {
                    return false;
                }
            } else {
                return false;
            }
        }
        true
    }

    fn create_simple_scenario_internal(width: usize, height: usize) -> (Grid, Vec<Task>) {
        let grid = Grid::new(width, height);
        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new((width - 1) as f64, 0.0)),
            Task::new(1, Point::new((width - 1) as f64, 0.0), Point::new(0.0, 0.0)),
        ];
        (grid, tasks)
    }

    fn create_four_corner_scenario_internal(width: usize, height: usize) -> (Grid, Vec<Task>) {
        let grid = Grid::new(width, height);
        let w = (width - 1) as f64;
        let h = (height - 1) as f64;
        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(w, h)),
            Task::new(1, Point::new(w, 0.0), Point::new(0.0, h)),
            Task::new(2, Point::new(w, h), Point::new(0.0, 0.0)),
            Task::new(3, Point::new(0.0, h), Point::new(w, 0.0)),
        ];
        (grid, tasks)
    }

    fn add_random_obstacles_internal(grid: &mut Grid, obstacle_count: usize, seed: u64) {
        let mut rng_state = seed;
        for _ in 0..obstacle_count {
            rng_state = (1664525_u64.wrapping_mul(rng_state).wrapping_add(1013904223)) % (1 << 32);
            let x = (rng_state % grid.width as u64) as usize;
            rng_state = (1664525_u64.wrapping_mul(rng_state).wrapping_add(1013904223)) % (1 << 32);
            let y = (rng_state % grid.height as u64) as usize;
            if !((x == 0 || x == grid.width - 1) && (y == 0 || y == grid.height - 1)) {
                grid.set_obstacle(x, y, true);
            }
        }
    }

    // --- Tests for create_simple_scenario ---

    #[test]
    fn test_create_simple_scenario_grid_size() {
        let (grid, tasks) = create_simple_scenario_internal(10, 5);

        assert_eq!(grid.width, 10, "Grid width should match");
        assert_eq!(grid.height, 5, "Grid height should match");
        assert_eq!(tasks.len(), 2, "Should create 2 tasks");
    }

    #[test]
    fn test_create_simple_scenario_task_positions() {
        let (_, tasks) = create_simple_scenario_internal(10, 5);

        // Agent 0: starts at (0,0), goes to (9,0)
        assert_eq!(tasks[0].agent_id, 0);
        assert!((tasks[0].start.x - 0.0).abs() < 0.01);
        assert!((tasks[0].goal.x - 9.0).abs() < 0.01);

        // Agent 1: starts at (9,0), goes to (0,0)
        assert_eq!(tasks[1].agent_id, 1);
        assert!((tasks[1].start.x - 9.0).abs() < 0.01);
        assert!((tasks[1].goal.x - 0.0).abs() < 0.01);
    }

    // --- Tests for create_four_corner_scenario ---

    #[test]
    fn test_create_four_corner_scenario_task_count() {
        let (grid, tasks) = create_four_corner_scenario_internal(8, 8);

        assert_eq!(grid.width, 8);
        assert_eq!(grid.height, 8);
        assert_eq!(tasks.len(), 4, "Should create 4 tasks for four corners");
    }

    #[test]
    fn test_create_four_corner_scenario_diagonal_goals() {
        let (_, tasks) = create_four_corner_scenario_internal(10, 10);

        // Agent 0: (0,0) -> (9,9)
        assert!((tasks[0].start.x - 0.0).abs() < 0.01);
        assert!((tasks[0].start.y - 0.0).abs() < 0.01);
        assert!((tasks[0].goal.x - 9.0).abs() < 0.01);
        assert!((tasks[0].goal.y - 9.0).abs() < 0.01);

        // Agent 2: (9,9) -> (0,0) - opposite of agent 0
        assert!((tasks[2].start.x - 9.0).abs() < 0.01);
        assert!((tasks[2].goal.x - 0.0).abs() < 0.01);
    }

    // --- Tests for add_random_obstacles ---

    #[test]
    fn test_add_random_obstacles_deterministic() {
        // Same seed should produce same obstacles
        let mut grid1 = Grid::new(10, 10);
        let mut grid2 = Grid::new(10, 10);

        add_random_obstacles_internal(&mut grid1, 5, 42);
        add_random_obstacles_internal(&mut grid2, 5, 42);

        // Count obstacles in both grids - should be identical
        let mut count1 = 0;
        let mut count2 = 0;
        for x in 0..10 {
            for y in 0..10 {
                if grid1.is_obstacle(x, y) { count1 += 1; }
                if grid2.is_obstacle(x, y) { count2 += 1; }
            }
        }
        assert_eq!(count1, count2, "Same seed should produce same obstacle count");
    }

    #[test]
    fn test_add_random_obstacles_preserves_corners() {
        let mut grid = Grid::new(10, 10);
        add_random_obstacles_internal(&mut grid, 100, 42);

        // Corners should never have obstacles
        assert!(!grid.is_obstacle(0, 0), "Corner (0,0) should be clear");
        assert!(!grid.is_obstacle(9, 0), "Corner (9,0) should be clear");
        assert!(!grid.is_obstacle(0, 9), "Corner (0,9) should be clear");
        assert!(!grid.is_obstacle(9, 9), "Corner (9,9) should be clear");
    }

    #[test]
    fn test_add_random_obstacles_zero_count() {
        let mut grid = Grid::new(5, 5);
        add_random_obstacles_internal(&mut grid, 0, 42);

        // No obstacles should be added
        for x in 0..5 {
            for y in 0..5 {
                assert!(!grid.is_obstacle(x, y), "No obstacles should be added");
            }
        }
    }

    // --- Tests for path_length ---

    #[test]
    fn test_path_length_empty() {
        let path: Vec<Point> = vec![];
        let length = compute_path_length(&path);
        assert!((length - 0.0).abs() < 0.001, "Empty path should have zero length");
    }

    #[test]
    fn test_path_length_single_point() {
        let path = vec![Point::new(0.0, 0.0)];
        let length = compute_path_length(&path);
        assert!((length - 0.0).abs() < 0.001, "Single point path should have zero length");
    }

    #[test]
    fn test_path_length_horizontal() {
        let path = vec![
            Point::new(0.0, 0.0),
            Point::new(5.0, 0.0),
        ];
        let length = compute_path_length(&path);
        assert!((length - 5.0).abs() < 0.001, "Horizontal path length should be 5.0");
    }

    #[test]
    fn test_path_length_multi_segment() {
        let path = vec![
            Point::new(0.0, 0.0),
            Point::new(3.0, 0.0),
            Point::new(3.0, 4.0),
        ];
        let length = compute_path_length(&path);
        // 3 + 4 = 7
        assert!((length - 7.0).abs() < 0.001, "Multi-segment path length should be 7.0");
    }

    // --- Tests for solution_cost ---

    #[test]
    fn test_solution_cost_empty() {
        let solution: HashMap<usize, Vec<Point>> = HashMap::new();
        let cost = compute_solution_cost(&solution);
        assert!((cost - 0.0).abs() < 0.001, "Empty solution should have zero cost");
    }

    #[test]
    fn test_solution_cost_single_agent() {
        let mut solution = HashMap::new();
        solution.insert(0, vec![
            Point::new(0.0, 0.0),
            Point::new(4.0, 0.0),
        ]);
        let cost = compute_solution_cost(&solution);
        assert!((cost - 4.0).abs() < 0.001, "Single agent cost should be 4.0");
    }

    #[test]
    fn test_solution_cost_multiple_agents() {
        let mut solution = HashMap::new();
        solution.insert(0, vec![
            Point::new(0.0, 0.0),
            Point::new(3.0, 0.0),
        ]);
        solution.insert(1, vec![
            Point::new(0.0, 0.0),
            Point::new(0.0, 4.0),
        ]);
        let cost = compute_solution_cost(&solution);
        // 3 + 4 = 7
        assert!((cost - 7.0).abs() < 0.001, "Total cost should be 7.0");
    }

    // --- Tests for validate_solution ---

    #[test]
    fn test_validate_solution_valid() {
        let mut solution = HashMap::new();
        solution.insert(0, vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(2.0, 0.0),
        ]);

        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(2.0, 0.0)),
        ];

        assert!(is_solution_valid(&solution, &tasks), "Valid solution should pass validation");
    }

    #[test]
    fn test_validate_solution_wrong_start() {
        let mut solution = HashMap::new();
        solution.insert(0, vec![
            Point::new(5.0, 5.0), // Wrong start
            Point::new(2.0, 0.0),
        ]);

        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(2.0, 0.0)),
        ];

        assert!(!is_solution_valid(&solution, &tasks), "Solution with wrong start should fail");
    }

    #[test]
    fn test_validate_solution_wrong_goal() {
        let mut solution = HashMap::new();
        solution.insert(0, vec![
            Point::new(0.0, 0.0),
            Point::new(9.0, 9.0), // Wrong goal
        ]);

        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(2.0, 0.0)),
        ];

        assert!(!is_solution_valid(&solution, &tasks), "Solution with wrong goal should fail");
    }

    #[test]
    fn test_validate_solution_missing_agent() {
        let solution: HashMap<usize, Vec<Point>> = HashMap::new(); // No paths

        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(2.0, 0.0)),
        ];

        assert!(!is_solution_valid(&solution, &tasks), "Solution missing agent should fail");
    }

    #[test]
    fn test_validate_solution_empty_path() {
        let mut solution = HashMap::new();
        solution.insert(0, vec![]); // Empty path

        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(2.0, 0.0)),
        ];

        assert!(!is_solution_valid(&solution, &tasks), "Solution with empty path should fail");
    }
}