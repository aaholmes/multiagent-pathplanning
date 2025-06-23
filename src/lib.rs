use pyo3::prelude::*;
use std::collections::HashMap;

mod structs;
mod astar;
mod cbs;
mod orca;

use structs::*;
use cbs::solve_cbs;
use orca::compute_orca_velocity;

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
    let result = compute_orca_velocity(agent, &neighbor_refs, time_horizon);
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

// Alias functions for backwards compatibility and convenience
#[pyfunction]
fn solve_cbs_wrapper(grid: Grid, tasks: Vec<Task>) -> PyResult<Option<HashMap<usize, Vec<Point>>>> {
    solve_cbs_py(grid, tasks)
}

#[pyfunction]
fn compute_orca_wrapper(
    agent: &AgentState,
    neighbors: Vec<AgentState>,
    time_horizon: Option<f64>,
) -> PyResult<Vector2D> {
    let th = time_horizon.unwrap_or(2.0);
    compute_orca_velocity_py(agent, neighbors, th)
}