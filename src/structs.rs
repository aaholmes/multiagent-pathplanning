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