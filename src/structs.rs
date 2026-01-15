//! # Core Data Structures
//!
//! This module defines the fundamental data types used throughout the library:
//!
//! - **Point**: 2D position in continuous space
//! - **Vector2D**: 2D velocity/direction vector with arithmetic operations
//! - **AgentState**: Complete state of an agent (position, velocity, radius, etc.)
//! - **Task**: Pathfinding task with start/goal positions
//! - **Grid**: 2D obstacle grid for pathfinding
//! - **Constraint/Conflict/CTNode**: Internal types for CBS algorithm

use pyo3::prelude::*;
use std::collections::HashMap;
use std::ops::{Add, Sub, Mul};

#[pyclass]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    #[pyo3(get, set)]
    pub x: f64,
    #[pyo3(get, set)]
    pub y: f64,
}

#[pymethods]
impl Point {
    #[new]
    pub fn new(x: f64, y: f64) -> Self {
        Point { x, y }
    }

    pub fn distance(&self, other: &Point) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }

    pub fn manhattan_distance(&self, other: &Point) -> f64 {
        (self.x - other.x).abs() + (self.y - other.y).abs()
    }

    pub fn __str__(&self) -> String {
        format!("Point({:.2}, {:.2})", self.x, self.y)
    }

    pub fn __add__(&self, other: &Vector2D) -> Point {
        Point {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }

    pub fn __sub__(&self, other: &Point) -> Vector2D {
        Vector2D {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl Sub for Point {
    type Output = Vector2D;
    
    fn sub(self, other: Point) -> Vector2D {
        Vector2D {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl Add<Vector2D> for Point {
    type Output = Point;
    
    fn add(self, other: Vector2D) -> Point {
        Point {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

#[pyclass]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector2D {
    #[pyo3(get, set)]
    pub x: f64,
    #[pyo3(get, set)]
    pub y: f64,
}

#[pymethods]
impl Vector2D {
    #[new]
    pub fn new(x: f64, y: f64) -> Self {
        Vector2D { x, y }
    }

    pub fn magnitude(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    pub fn normalize(&self) -> Vector2D {
        let mag = self.magnitude();
        if mag > 0.0 {
            Vector2D {
                x: self.x / mag,
                y: self.y / mag,
            }
        } else {
            Vector2D { x: 0.0, y: 0.0 }
        }
    }

    pub fn dot(&self, other: &Vector2D) -> f64 {
        self.x * other.x + self.y * other.y
    }

    pub fn perpendicular(&self) -> Vector2D {
        Vector2D {
            x: -self.y,
            y: self.x,
        }
    }

    pub fn __str__(&self) -> String {
        format!("Vector2D({:.2}, {:.2})", self.x, self.y)
    }

    pub fn __add__(&self, other: &Vector2D) -> Vector2D {
        Vector2D {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }

    pub fn __sub__(&self, other: &Vector2D) -> Vector2D {
        Vector2D {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }

    pub fn __mul__(&self, scalar: f64) -> Vector2D {
        Vector2D {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }
}

impl Add for Vector2D {
    type Output = Vector2D;
    
    fn add(self, other: Vector2D) -> Vector2D {
        Vector2D {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl Sub for Vector2D {
    type Output = Vector2D;
    
    fn sub(self, other: Vector2D) -> Vector2D {
        Vector2D {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl Mul<f64> for Vector2D {
    type Output = Vector2D;
    
    fn mul(self, scalar: f64) -> Vector2D {
        Vector2D {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }
}

#[pyclass]
#[derive(Debug, Clone)]
pub struct AgentState {
    #[pyo3(get, set)]
    pub id: usize,
    #[pyo3(get, set)]
    pub position: Point,
    #[pyo3(get, set)]
    pub velocity: Vector2D,
    #[pyo3(get, set)]
    pub radius: f64,
    #[pyo3(get, set)]
    pub pref_velocity: Vector2D,
    #[pyo3(get, set)]
    pub max_speed: f64,
}

#[pymethods]
impl AgentState {
    #[new]
    pub fn new(
        id: usize,
        position: Point,
        velocity: Vector2D,
        radius: f64,
        pref_velocity: Vector2D,
        max_speed: f64,
    ) -> Self {
        AgentState {
            id,
            position,
            velocity,
            radius,
            pref_velocity,
            max_speed,
        }
    }

    pub fn distance_to(&self, other: &AgentState) -> f64 {
        self.position.distance(&other.position)
    }

    pub fn __str__(&self) -> String {
        format!(
            "Agent(id={}, pos={}, vel={}, r={:.2})",
            self.id, self.position.__str__(), self.velocity.__str__(), self.radius
        )
    }
}

#[pyclass]
#[derive(Debug, Clone)]
pub struct Task {
    #[pyo3(get, set)]
    pub agent_id: usize,
    #[pyo3(get, set)]
    pub start: Point,
    #[pyo3(get, set)]
    pub goal: Point,
}

#[pymethods]
impl Task {
    #[new]
    pub fn new(agent_id: usize, start: Point, goal: Point) -> Self {
        Task {
            agent_id,
            start,
            goal,
        }
    }

    pub fn __str__(&self) -> String {
        format!(
            "Task(agent={}, start={}, goal={})",
            self.agent_id,
            self.start.__str__(),
            self.goal.__str__()
        )
    }
}

pub type Path = Vec<Point>;

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Constraint {
    pub agent_id: usize,
    pub position: (i32, i32),
    pub time_step: usize,
}

impl Constraint {
    pub fn new(agent_id: usize, position: (i32, i32), time_step: usize) -> Self {
        Constraint {
            agent_id,
            position,
            time_step,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Conflict {
    pub agent_a: usize,
    pub agent_b: usize,
    pub position: (i32, i32),
    pub time_step: usize,
}

impl Conflict {
    pub fn new(agent_a: usize, agent_b: usize, position: (i32, i32), time_step: usize) -> Self {
        Conflict {
            agent_a,
            agent_b,
            position,
            time_step,
        }
    }
}

#[derive(Debug, Clone)]
pub struct CTNode {
    pub constraints: Vec<Constraint>,
    pub solution: HashMap<usize, Path>,
    pub cost: usize,
}

impl CTNode {
    pub fn new() -> Self {
        CTNode {
            constraints: Vec::new(),
            solution: HashMap::new(),
            cost: 0,
        }
    }

    pub fn with_constraints(constraints: Vec<Constraint>) -> Self {
        CTNode {
            constraints,
            solution: HashMap::new(),
            cost: 0,
        }
    }
}

#[pyclass]
#[derive(Debug, Clone, Copy)]
pub struct Line {
    #[pyo3(get, set)]
    pub point: Point,
    #[pyo3(get, set)]
    pub direction: Vector2D,
}

#[pymethods]
impl Line {
    #[new]
    pub fn new(point: Point, direction: Vector2D) -> Self {
        Line { point, direction }
    }

    pub fn __str__(&self) -> String {
        format!(
            "Line(point={}, direction={})",
            self.point.__str__(),
            self.direction.__str__()
        )
    }
}

#[pyclass]
#[derive(Debug, Clone)]
pub struct Grid {
    #[pyo3(get, set)]
    pub width: usize,
    #[pyo3(get, set)]
    pub height: usize,
    #[pyo3(get)]
    pub obstacles: Vec<Vec<bool>>,
}

#[pymethods]
impl Grid {
    #[new]
    pub fn new(width: usize, height: usize) -> Self {
        Grid {
            width,
            height,
            obstacles: vec![vec![false; width]; height],
        }
    }

    pub fn set_obstacle(&mut self, x: usize, y: usize, is_obstacle: bool) {
        if x < self.width && y < self.height {
            self.obstacles[y][x] = is_obstacle;
        }
    }

    pub fn is_obstacle(&self, x: usize, y: usize) -> bool {
        if x >= self.width || y >= self.height {
            true
        } else {
            self.obstacles[y][x]
        }
    }

    pub fn is_valid_position(&self, x: i32, y: i32) -> bool {
        x >= 0 && y >= 0 && (x as usize) < self.width && (y as usize) < self.height && !self.is_obstacle(x as usize, y as usize)
    }

    pub fn get_neighbors(&self, x: i32, y: i32) -> Vec<(i32, i32)> {
        let mut neighbors = Vec::new();
        let directions = [(-1, 0), (1, 0), (0, -1), (0, 1)];
        
        for (dx, dy) in directions.iter() {
            let nx = x + dx;
            let ny = y + dy;
            if self.is_valid_position(nx, ny) {
                neighbors.push((nx, ny));
            }
        }
        
        neighbors
    }

    pub fn __str__(&self) -> String {
        format!("Grid({}x{})", self.width, self.height)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ==================== Point Tests ====================

    #[test]
    fn test_point_new() {
        let p = Point::new(3.0, 4.0);
        assert_eq!(p.x, 3.0);
        assert_eq!(p.y, 4.0);
    }

    #[test]
    fn test_point_new_negative() {
        let p = Point::new(-5.0, -10.0);
        assert_eq!(p.x, -5.0);
        assert_eq!(p.y, -10.0);
    }

    #[test]
    fn test_point_new_zero() {
        let p = Point::new(0.0, 0.0);
        assert_eq!(p.x, 0.0);
        assert_eq!(p.y, 0.0);
    }

    #[test]
    fn test_point_distance_same() {
        let p1 = Point::new(0.0, 0.0);
        let p2 = Point::new(0.0, 0.0);
        assert_eq!(p1.distance(&p2), 0.0);
    }

    #[test]
    fn test_point_distance_horizontal() {
        let p1 = Point::new(0.0, 0.0);
        let p2 = Point::new(5.0, 0.0);
        assert_eq!(p1.distance(&p2), 5.0);
    }

    #[test]
    fn test_point_distance_vertical() {
        let p1 = Point::new(0.0, 0.0);
        let p2 = Point::new(0.0, 3.0);
        assert_eq!(p1.distance(&p2), 3.0);
    }

    #[test]
    fn test_point_distance_diagonal() {
        let p1 = Point::new(0.0, 0.0);
        let p2 = Point::new(3.0, 4.0);
        assert_eq!(p1.distance(&p2), 5.0); // 3-4-5 triangle
    }

    #[test]
    fn test_point_distance_symmetry() {
        let p1 = Point::new(1.0, 2.0);
        let p2 = Point::new(4.0, 6.0);
        assert_eq!(p1.distance(&p2), p2.distance(&p1));
    }

    #[test]
    fn test_point_manhattan_distance_same() {
        let p1 = Point::new(0.0, 0.0);
        let p2 = Point::new(0.0, 0.0);
        assert_eq!(p1.manhattan_distance(&p2), 0.0);
    }

    #[test]
    fn test_point_manhattan_distance_horizontal() {
        let p1 = Point::new(0.0, 0.0);
        let p2 = Point::new(5.0, 0.0);
        assert_eq!(p1.manhattan_distance(&p2), 5.0);
    }

    #[test]
    fn test_point_manhattan_distance_diagonal() {
        let p1 = Point::new(0.0, 0.0);
        let p2 = Point::new(3.0, 4.0);
        assert_eq!(p1.manhattan_distance(&p2), 7.0); // 3 + 4
    }

    #[test]
    fn test_point_manhattan_distance_negative() {
        let p1 = Point::new(-2.0, -3.0);
        let p2 = Point::new(2.0, 3.0);
        assert_eq!(p1.manhattan_distance(&p2), 10.0); // 4 + 6
    }

    #[test]
    fn test_point_sub_operator() {
        let p1 = Point::new(5.0, 7.0);
        let p2 = Point::new(2.0, 3.0);
        let v = p1 - p2;
        assert_eq!(v.x, 3.0);
        assert_eq!(v.y, 4.0);
    }

    #[test]
    fn test_point_add_vector_operator() {
        let p = Point::new(1.0, 2.0);
        let v = Vector2D::new(3.0, 4.0);
        let result = p + v;
        assert_eq!(result.x, 4.0);
        assert_eq!(result.y, 6.0);
    }

    #[test]
    fn test_point_str() {
        let p = Point::new(1.5, 2.5);
        assert_eq!(p.__str__(), "Point(1.50, 2.50)");
    }

    #[test]
    fn test_point_equality() {
        let p1 = Point::new(1.0, 2.0);
        let p2 = Point::new(1.0, 2.0);
        let p3 = Point::new(1.0, 3.0);
        assert_eq!(p1, p2);
        assert_ne!(p1, p3);
    }

    #[test]
    fn test_point_clone() {
        let p1 = Point::new(1.0, 2.0);
        let p2 = p1.clone();
        assert_eq!(p1, p2);
    }

    // ==================== Vector2D Tests ====================

    #[test]
    fn test_vector2d_new() {
        let v = Vector2D::new(3.0, 4.0);
        assert_eq!(v.x, 3.0);
        assert_eq!(v.y, 4.0);
    }

    #[test]
    fn test_vector2d_magnitude_zero() {
        let v = Vector2D::new(0.0, 0.0);
        assert_eq!(v.magnitude(), 0.0);
    }

    #[test]
    fn test_vector2d_magnitude_unit_x() {
        let v = Vector2D::new(1.0, 0.0);
        assert_eq!(v.magnitude(), 1.0);
    }

    #[test]
    fn test_vector2d_magnitude_unit_y() {
        let v = Vector2D::new(0.0, 1.0);
        assert_eq!(v.magnitude(), 1.0);
    }

    #[test]
    fn test_vector2d_magnitude_345() {
        let v = Vector2D::new(3.0, 4.0);
        assert_eq!(v.magnitude(), 5.0);
    }

    #[test]
    fn test_vector2d_magnitude_negative() {
        let v = Vector2D::new(-3.0, -4.0);
        assert_eq!(v.magnitude(), 5.0);
    }

    #[test]
    fn test_vector2d_normalize_zero() {
        let v = Vector2D::new(0.0, 0.0);
        let n = v.normalize();
        assert_eq!(n.x, 0.0);
        assert_eq!(n.y, 0.0);
    }

    #[test]
    fn test_vector2d_normalize_unit() {
        let v = Vector2D::new(1.0, 0.0);
        let n = v.normalize();
        assert_eq!(n.x, 1.0);
        assert_eq!(n.y, 0.0);
    }

    #[test]
    fn test_vector2d_normalize_345() {
        let v = Vector2D::new(3.0, 4.0);
        let n = v.normalize();
        assert!((n.x - 0.6).abs() < 1e-10);
        assert!((n.y - 0.8).abs() < 1e-10);
        assert!((n.magnitude() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_vector2d_normalize_negative() {
        let v = Vector2D::new(-3.0, -4.0);
        let n = v.normalize();
        assert!((n.x - (-0.6)).abs() < 1e-10);
        assert!((n.y - (-0.8)).abs() < 1e-10);
    }

    #[test]
    fn test_vector2d_dot_perpendicular() {
        let v1 = Vector2D::new(1.0, 0.0);
        let v2 = Vector2D::new(0.0, 1.0);
        assert_eq!(v1.dot(&v2), 0.0);
    }

    #[test]
    fn test_vector2d_dot_parallel() {
        let v1 = Vector2D::new(2.0, 0.0);
        let v2 = Vector2D::new(3.0, 0.0);
        assert_eq!(v1.dot(&v2), 6.0);
    }

    #[test]
    fn test_vector2d_dot_antiparallel() {
        let v1 = Vector2D::new(2.0, 0.0);
        let v2 = Vector2D::new(-3.0, 0.0);
        assert_eq!(v1.dot(&v2), -6.0);
    }

    #[test]
    fn test_vector2d_dot_general() {
        let v1 = Vector2D::new(1.0, 2.0);
        let v2 = Vector2D::new(3.0, 4.0);
        assert_eq!(v1.dot(&v2), 11.0); // 1*3 + 2*4
    }

    #[test]
    fn test_vector2d_perpendicular() {
        let v = Vector2D::new(1.0, 0.0);
        let p = v.perpendicular();
        assert_eq!(p.x, 0.0);
        assert_eq!(p.y, 1.0);
        assert_eq!(v.dot(&p), 0.0); // Should be perpendicular
    }

    #[test]
    fn test_vector2d_perpendicular_general() {
        let v = Vector2D::new(3.0, 4.0);
        let p = v.perpendicular();
        assert_eq!(p.x, -4.0);
        assert_eq!(p.y, 3.0);
        assert_eq!(v.dot(&p), 0.0);
    }

    #[test]
    fn test_vector2d_add_operator() {
        let v1 = Vector2D::new(1.0, 2.0);
        let v2 = Vector2D::new(3.0, 4.0);
        let result = v1 + v2;
        assert_eq!(result.x, 4.0);
        assert_eq!(result.y, 6.0);
    }

    #[test]
    fn test_vector2d_sub_operator() {
        let v1 = Vector2D::new(5.0, 7.0);
        let v2 = Vector2D::new(2.0, 3.0);
        let result = v1 - v2;
        assert_eq!(result.x, 3.0);
        assert_eq!(result.y, 4.0);
    }

    #[test]
    fn test_vector2d_mul_operator() {
        let v = Vector2D::new(2.0, 3.0);
        let result = v * 2.0;
        assert_eq!(result.x, 4.0);
        assert_eq!(result.y, 6.0);
    }

    #[test]
    fn test_vector2d_mul_zero() {
        let v = Vector2D::new(2.0, 3.0);
        let result = v * 0.0;
        assert_eq!(result.x, 0.0);
        assert_eq!(result.y, 0.0);
    }

    #[test]
    fn test_vector2d_mul_negative() {
        let v = Vector2D::new(2.0, 3.0);
        let result = v * -1.0;
        assert_eq!(result.x, -2.0);
        assert_eq!(result.y, -3.0);
    }

    #[test]
    fn test_vector2d_str() {
        let v = Vector2D::new(1.5, 2.5);
        assert_eq!(v.__str__(), "Vector2D(1.50, 2.50)");
    }

    #[test]
    fn test_vector2d_equality() {
        let v1 = Vector2D::new(1.0, 2.0);
        let v2 = Vector2D::new(1.0, 2.0);
        let v3 = Vector2D::new(1.0, 3.0);
        assert_eq!(v1, v2);
        assert_ne!(v1, v3);
    }

    // ==================== AgentState Tests ====================

    #[test]
    fn test_agent_state_new() {
        let agent = AgentState::new(
            0,
            Point::new(1.0, 2.0),
            Vector2D::new(0.5, 0.5),
            0.3,
            Vector2D::new(1.0, 0.0),
            1.5,
        );
        assert_eq!(agent.id, 0);
        assert_eq!(agent.position.x, 1.0);
        assert_eq!(agent.position.y, 2.0);
        assert_eq!(agent.velocity.x, 0.5);
        assert_eq!(agent.velocity.y, 0.5);
        assert_eq!(agent.radius, 0.3);
        assert_eq!(agent.max_speed, 1.5);
    }

    #[test]
    fn test_agent_state_distance_to() {
        let agent1 = AgentState::new(
            0,
            Point::new(0.0, 0.0),
            Vector2D::new(0.0, 0.0),
            0.3,
            Vector2D::new(0.0, 0.0),
            1.5,
        );
        let agent2 = AgentState::new(
            1,
            Point::new(3.0, 4.0),
            Vector2D::new(0.0, 0.0),
            0.3,
            Vector2D::new(0.0, 0.0),
            1.5,
        );
        assert_eq!(agent1.distance_to(&agent2), 5.0);
    }

    #[test]
    fn test_agent_state_distance_to_same() {
        let agent1 = AgentState::new(
            0,
            Point::new(5.0, 5.0),
            Vector2D::new(0.0, 0.0),
            0.3,
            Vector2D::new(0.0, 0.0),
            1.5,
        );
        let agent2 = AgentState::new(
            1,
            Point::new(5.0, 5.0),
            Vector2D::new(0.0, 0.0),
            0.3,
            Vector2D::new(0.0, 0.0),
            1.5,
        );
        assert_eq!(agent1.distance_to(&agent2), 0.0);
    }

    #[test]
    fn test_agent_state_str() {
        let agent = AgentState::new(
            0,
            Point::new(1.0, 2.0),
            Vector2D::new(0.5, 0.5),
            0.30,
            Vector2D::new(1.0, 0.0),
            1.5,
        );
        let s = agent.__str__();
        assert!(s.contains("id=0"));
        assert!(s.contains("r=0.30"));
    }

    // ==================== Task Tests ====================

    #[test]
    fn test_task_new() {
        let task = Task::new(
            0,
            Point::new(0.0, 0.0),
            Point::new(10.0, 10.0),
        );
        assert_eq!(task.agent_id, 0);
        assert_eq!(task.start.x, 0.0);
        assert_eq!(task.start.y, 0.0);
        assert_eq!(task.goal.x, 10.0);
        assert_eq!(task.goal.y, 10.0);
    }

    #[test]
    fn test_task_str() {
        let task = Task::new(
            5,
            Point::new(1.0, 2.0),
            Point::new(3.0, 4.0),
        );
        let s = task.__str__();
        assert!(s.contains("agent=5"));
        assert!(s.contains("start="));
        assert!(s.contains("goal="));
    }

    // ==================== Constraint Tests ====================

    #[test]
    fn test_constraint_new() {
        let c = Constraint::new(0, (5, 5), 10);
        assert_eq!(c.agent_id, 0);
        assert_eq!(c.position, (5, 5));
        assert_eq!(c.time_step, 10);
    }

    #[test]
    fn test_constraint_equality() {
        let c1 = Constraint::new(0, (5, 5), 10);
        let c2 = Constraint::new(0, (5, 5), 10);
        let c3 = Constraint::new(0, (5, 5), 11);
        assert_eq!(c1, c2);
        assert_ne!(c1, c3);
    }

    #[test]
    fn test_constraint_hash() {
        use std::collections::HashSet;
        let mut set = HashSet::new();
        set.insert(Constraint::new(0, (5, 5), 10));
        set.insert(Constraint::new(0, (5, 5), 10)); // Duplicate
        set.insert(Constraint::new(1, (5, 5), 10));
        assert_eq!(set.len(), 2);
    }

    // ==================== Conflict Tests ====================

    #[test]
    fn test_conflict_new() {
        let c = Conflict::new(0, 1, (5, 5), 10);
        assert_eq!(c.agent_a, 0);
        assert_eq!(c.agent_b, 1);
        assert_eq!(c.position, (5, 5));
        assert_eq!(c.time_step, 10);
    }

    // ==================== CTNode Tests ====================

    #[test]
    fn test_ctnode_new() {
        let node = CTNode::new();
        assert!(node.constraints.is_empty());
        assert!(node.solution.is_empty());
        assert_eq!(node.cost, 0);
    }

    #[test]
    fn test_ctnode_with_constraints() {
        let constraints = vec![
            Constraint::new(0, (1, 1), 5),
            Constraint::new(1, (2, 2), 6),
        ];
        let node = CTNode::with_constraints(constraints.clone());
        assert_eq!(node.constraints.len(), 2);
        assert!(node.solution.is_empty());
        assert_eq!(node.cost, 0);
    }

    // ==================== Line Tests ====================

    #[test]
    fn test_line_new() {
        let line = Line::new(
            Point::new(1.0, 2.0),
            Vector2D::new(1.0, 0.0),
        );
        assert_eq!(line.point.x, 1.0);
        assert_eq!(line.point.y, 2.0);
        assert_eq!(line.direction.x, 1.0);
        assert_eq!(line.direction.y, 0.0);
    }

    #[test]
    fn test_line_str() {
        let line = Line::new(
            Point::new(1.0, 2.0),
            Vector2D::new(0.0, 1.0),
        );
        let s = line.__str__();
        assert!(s.contains("Line"));
        assert!(s.contains("point="));
        assert!(s.contains("direction="));
    }

    // ==================== Grid Tests ====================

    #[test]
    fn test_grid_new() {
        let grid = Grid::new(10, 20);
        assert_eq!(grid.width, 10);
        assert_eq!(grid.height, 20);
        assert_eq!(grid.obstacles.len(), 20);
        assert_eq!(grid.obstacles[0].len(), 10);
    }

    #[test]
    fn test_grid_new_small() {
        let grid = Grid::new(1, 1);
        assert_eq!(grid.width, 1);
        assert_eq!(grid.height, 1);
        assert!(!grid.is_obstacle(0, 0));
    }

    #[test]
    fn test_grid_set_obstacle() {
        let mut grid = Grid::new(10, 10);
        assert!(!grid.is_obstacle(5, 5));
        grid.set_obstacle(5, 5, true);
        assert!(grid.is_obstacle(5, 5));
        grid.set_obstacle(5, 5, false);
        assert!(!grid.is_obstacle(5, 5));
    }

    #[test]
    fn test_grid_set_obstacle_out_of_bounds() {
        let mut grid = Grid::new(10, 10);
        // Should not panic, just ignore
        grid.set_obstacle(100, 100, true);
        // Out of bounds returns true (treated as obstacle)
        assert!(grid.is_obstacle(100, 100));
    }

    #[test]
    fn test_grid_is_obstacle_out_of_bounds() {
        let grid = Grid::new(10, 10);
        // Out of bounds should be treated as obstacle
        assert!(grid.is_obstacle(10, 0));
        assert!(grid.is_obstacle(0, 10));
        assert!(grid.is_obstacle(100, 100));
    }

    #[test]
    fn test_grid_is_valid_position_valid() {
        let grid = Grid::new(10, 10);
        assert!(grid.is_valid_position(0, 0));
        assert!(grid.is_valid_position(5, 5));
        assert!(grid.is_valid_position(9, 9));
    }

    #[test]
    fn test_grid_is_valid_position_out_of_bounds() {
        let grid = Grid::new(10, 10);
        assert!(!grid.is_valid_position(-1, 0));
        assert!(!grid.is_valid_position(0, -1));
        assert!(!grid.is_valid_position(10, 0));
        assert!(!grid.is_valid_position(0, 10));
    }

    #[test]
    fn test_grid_is_valid_position_with_obstacle() {
        let mut grid = Grid::new(10, 10);
        grid.set_obstacle(5, 5, true);
        assert!(!grid.is_valid_position(5, 5));
    }

    #[test]
    fn test_grid_get_neighbors_center() {
        let grid = Grid::new(10, 10);
        let neighbors = grid.get_neighbors(5, 5);
        assert_eq!(neighbors.len(), 4);
        assert!(neighbors.contains(&(4, 5)));
        assert!(neighbors.contains(&(6, 5)));
        assert!(neighbors.contains(&(5, 4)));
        assert!(neighbors.contains(&(5, 6)));
    }

    #[test]
    fn test_grid_get_neighbors_corner() {
        let grid = Grid::new(10, 10);
        let neighbors = grid.get_neighbors(0, 0);
        assert_eq!(neighbors.len(), 2);
        assert!(neighbors.contains(&(1, 0)));
        assert!(neighbors.contains(&(0, 1)));
    }

    #[test]
    fn test_grid_get_neighbors_edge() {
        let grid = Grid::new(10, 10);
        let neighbors = grid.get_neighbors(5, 0);
        assert_eq!(neighbors.len(), 3);
        assert!(neighbors.contains(&(4, 0)));
        assert!(neighbors.contains(&(6, 0)));
        assert!(neighbors.contains(&(5, 1)));
    }

    #[test]
    fn test_grid_get_neighbors_with_obstacles() {
        let mut grid = Grid::new(10, 10);
        grid.set_obstacle(4, 5, true);
        grid.set_obstacle(6, 5, true);
        let neighbors = grid.get_neighbors(5, 5);
        assert_eq!(neighbors.len(), 2);
        assert!(neighbors.contains(&(5, 4)));
        assert!(neighbors.contains(&(5, 6)));
    }

    #[test]
    fn test_grid_str() {
        let grid = Grid::new(15, 20);
        assert_eq!(grid.__str__(), "Grid(15x20)");
    }

    #[test]
    fn test_grid_clone() {
        let mut grid1 = Grid::new(10, 10);
        grid1.set_obstacle(5, 5, true);
        let grid2 = grid1.clone();
        assert!(grid2.is_obstacle(5, 5));
        assert_eq!(grid1.width, grid2.width);
        assert_eq!(grid1.height, grid2.height);
    }
}