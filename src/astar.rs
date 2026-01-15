//! # A* Pathfinding
//!
//! Time-space A* pathfinding algorithm for single-agent navigation with constraints.
//!
//! ## Algorithm
//!
//! This implementation extends classic A* to operate in a 3D time-space graph:
//! - Each state is (x, y, t) representing position at a specific time step
//! - Supports temporal constraints to avoid conflicts with other agents
//! - Uses Manhattan distance heuristic for grid-based movement
//!
//! ## Usage
//!
//! Primarily used internally by CBS for finding individual agent paths.
//! Call `find_single_agent_path` for standalone pathfinding.

use crate::structs::{Point, Grid, Path, Constraint};
use ordered_float::OrderedFloat;
use priority_queue::PriorityQueue;
use std::collections::{HashMap, HashSet};
use std::cmp::Reverse;

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct AStarNode {
    position: (i32, i32),
    time_step: usize,
}

impl AStarNode {
    fn new(position: (i32, i32), time_step: usize) -> Self {
        AStarNode { position, time_step }
    }
}

pub struct AStar {
    grid: Grid,
    constraints: HashSet<(usize, (i32, i32), usize)>, // (agent_id, position, time_step)
}

impl AStar {
    pub fn new(grid: Grid) -> Self {
        AStar {
            grid,
            constraints: HashSet::new(),
        }
    }

    pub fn with_constraints(grid: Grid, constraints: &[Constraint], agent_id: usize) -> Self {
        let mut astar = AStar::new(grid);
        for constraint in constraints {
            if constraint.agent_id == agent_id {
                astar.constraints.insert((
                    constraint.agent_id,
                    constraint.position,
                    constraint.time_step,
                ));
            }
        }
        astar
    }

    fn heuristic(&self, current: (i32, i32), goal: (i32, i32)) -> f64 {
        ((current.0 - goal.0).abs() + (current.1 - goal.1).abs()) as f64
    }

    fn is_constrained(&self, agent_id: usize, position: (i32, i32), time_step: usize) -> bool {
        self.constraints.contains(&(agent_id, position, time_step))
    }

    pub fn find_path(
        &self,
        start: Point,
        goal: Point,
        agent_id: usize,
        max_time_steps: Option<usize>,
    ) -> Option<Path> {
        let start_pos = (start.x.round() as i32, start.y.round() as i32);
        let goal_pos = (goal.x.round() as i32, goal.y.round() as i32);
        let max_time = max_time_steps.unwrap_or(1000);

        if !self.grid.is_valid_position(start_pos.0, start_pos.1)
            || !self.grid.is_valid_position(goal_pos.0, goal_pos.1)
        {
            return None;
        }

        let mut open_set = PriorityQueue::new();
        let mut came_from: HashMap<AStarNode, AStarNode> = HashMap::new();
        let mut g_score: HashMap<AStarNode, f64> = HashMap::new();
        let mut f_score: HashMap<AStarNode, f64> = HashMap::new();

        let start_node = AStarNode::new(start_pos, 0);
        g_score.insert(start_node.clone(), 0.0);
        f_score.insert(start_node.clone(), self.heuristic(start_pos, goal_pos));
        open_set.push(start_node.clone(), Reverse(OrderedFloat(self.heuristic(start_pos, goal_pos))));

        while let Some((current_node, _)) = open_set.pop() {
            if current_node.position == goal_pos {
                return Some(self.reconstruct_path(&came_from, &current_node, start, goal));
            }

            if current_node.time_step >= max_time {
                continue;
            }

            let neighbors = self.grid.get_neighbors(current_node.position.0, current_node.position.1);
            
            // Add staying in place as an option
            let mut all_moves = neighbors;
            all_moves.push(current_node.position);

            for next_pos in all_moves {
                let next_time = current_node.time_step + 1;
                let next_node = AStarNode::new(next_pos, next_time);

                // Check if this move is constrained
                if self.is_constrained(agent_id, next_pos, next_time) {
                    continue;
                }

                let tentative_g_score = g_score.get(&current_node).unwrap_or(&f64::INFINITY) + 1.0;

                let current_g = g_score.get(&next_node).unwrap_or(&f64::INFINITY);
                if tentative_g_score < *current_g {
                    came_from.insert(next_node.clone(), current_node.clone());
                    g_score.insert(next_node.clone(), tentative_g_score);
                    
                    let h_score = self.heuristic(next_pos, goal_pos);
                    let f = tentative_g_score + h_score;
                    f_score.insert(next_node.clone(), f);

                    // Add time penalty to encourage faster solutions
                    let priority = f + (next_time as f64) * 0.01;
                    open_set.push(next_node, Reverse(OrderedFloat(priority)));
                }
            }
        }

        None
    }

    fn reconstruct_path(&self, came_from: &HashMap<AStarNode, AStarNode>, current: &AStarNode, start: Point, goal: Point) -> Path {
        let mut path = Vec::new();
        let mut current_node = current;

        while let Some(parent) = came_from.get(current_node) {
            path.push(Point::new(current_node.position.0 as f64, current_node.position.1 as f64));
            current_node = parent;
        }
        
        // Add the start position
        path.push(Point::new(current_node.position.0 as f64, current_node.position.1 as f64));
        path.reverse();

        // Ensure the path ends exactly at the goal
        if let Some(last) = path.last_mut() {
            *last = goal;
        }
        if let Some(first) = path.first_mut() {
            *first = start;
        }

        path
    }
}

pub fn find_single_agent_path(
    grid: &Grid,
    start: Point,
    goal: Point,
    constraints: &[Constraint],
    agent_id: usize,
) -> Option<Path> {
    let astar = AStar::with_constraints(grid.clone(), constraints, agent_id);
    astar.find_path(start, goal, agent_id, Some(500))
}

#[cfg(test)]
mod tests {
    use super::*;

    // ==================== AStarNode Tests ====================

    #[test]
    fn test_astar_node_new() {
        let node = AStarNode::new((5, 10), 3);
        assert_eq!(node.position, (5, 10));
        assert_eq!(node.time_step, 3);
    }

    #[test]
    fn test_astar_node_equality() {
        let node1 = AStarNode::new((5, 10), 3);
        let node2 = AStarNode::new((5, 10), 3);
        let node3 = AStarNode::new((5, 10), 4);
        assert_eq!(node1, node2);
        assert_ne!(node1, node3);
    }

    #[test]
    fn test_astar_node_hash() {
        use std::collections::HashSet;
        let mut set = HashSet::new();
        set.insert(AStarNode::new((5, 10), 3));
        set.insert(AStarNode::new((5, 10), 3)); // Duplicate
        set.insert(AStarNode::new((5, 10), 4));
        assert_eq!(set.len(), 2);
    }

    // ==================== AStar Constructor Tests ====================

    #[test]
    fn test_astar_new() {
        let grid = Grid::new(10, 10);
        let astar = AStar::new(grid);
        assert!(astar.constraints.is_empty());
        assert_eq!(astar.grid.width, 10);
        assert_eq!(astar.grid.height, 10);
    }

    #[test]
    fn test_astar_with_constraints_filters_by_agent() {
        let grid = Grid::new(10, 10);
        let constraints = vec![
            Constraint::new(0, (1, 1), 1),
            Constraint::new(0, (2, 2), 2),
            Constraint::new(1, (3, 3), 3), // Different agent
            Constraint::new(1, (4, 4), 4), // Different agent
        ];
        let astar = AStar::with_constraints(grid, &constraints, 0);
        assert_eq!(astar.constraints.len(), 2);
    }

    #[test]
    fn test_astar_with_no_matching_constraints() {
        let grid = Grid::new(10, 10);
        let constraints = vec![
            Constraint::new(1, (1, 1), 1),
            Constraint::new(2, (2, 2), 2),
        ];
        let astar = AStar::with_constraints(grid, &constraints, 0);
        assert!(astar.constraints.is_empty());
    }

    #[test]
    fn test_astar_with_empty_constraints() {
        let grid = Grid::new(10, 10);
        let constraints: Vec<Constraint> = vec![];
        let astar = AStar::with_constraints(grid, &constraints, 0);
        assert!(astar.constraints.is_empty());
    }

    // ==================== Heuristic Tests ====================

    #[test]
    fn test_heuristic_same_position() {
        let grid = Grid::new(10, 10);
        let astar = AStar::new(grid);
        assert_eq!(astar.heuristic((5, 5), (5, 5)), 0.0);
    }

    #[test]
    fn test_heuristic_horizontal() {
        let grid = Grid::new(10, 10);
        let astar = AStar::new(grid);
        assert_eq!(astar.heuristic((0, 0), (5, 0)), 5.0);
    }

    #[test]
    fn test_heuristic_vertical() {
        let grid = Grid::new(10, 10);
        let astar = AStar::new(grid);
        assert_eq!(astar.heuristic((0, 0), (0, 5)), 5.0);
    }

    #[test]
    fn test_heuristic_diagonal() {
        let grid = Grid::new(10, 10);
        let astar = AStar::new(grid);
        // Manhattan distance for diagonal
        assert_eq!(astar.heuristic((0, 0), (3, 4)), 7.0);
    }

    #[test]
    fn test_heuristic_negative_coords() {
        let grid = Grid::new(10, 10);
        let astar = AStar::new(grid);
        assert_eq!(astar.heuristic((-3, -4), (0, 0)), 7.0);
    }

    // ==================== is_constrained Tests ====================

    #[test]
    fn test_is_constrained_true() {
        let grid = Grid::new(10, 10);
        let constraints = vec![Constraint::new(0, (5, 5), 10)];
        let astar = AStar::with_constraints(grid, &constraints, 0);
        assert!(astar.is_constrained(0, (5, 5), 10));
    }

    #[test]
    fn test_is_constrained_false_wrong_position() {
        let grid = Grid::new(10, 10);
        let constraints = vec![Constraint::new(0, (5, 5), 10)];
        let astar = AStar::with_constraints(grid, &constraints, 0);
        assert!(!astar.is_constrained(0, (5, 6), 10));
    }

    #[test]
    fn test_is_constrained_false_wrong_time() {
        let grid = Grid::new(10, 10);
        let constraints = vec![Constraint::new(0, (5, 5), 10)];
        let astar = AStar::with_constraints(grid, &constraints, 0);
        assert!(!astar.is_constrained(0, (5, 5), 11));
    }

    // ==================== find_path Basic Tests ====================

    #[test]
    fn test_simple_path() {
        let grid = Grid::new(5, 5);
        let astar = AStar::new(grid);

        let start = Point::new(0.0, 0.0);
        let goal = Point::new(4.0, 4.0);

        let path = astar.find_path(start, goal, 0, Some(100));
        assert!(path.is_some());

        let path = path.unwrap();
        assert_eq!(path.first().unwrap(), &start);
        assert_eq!(path.last().unwrap(), &goal);
    }

    #[test]
    fn test_path_same_start_goal() {
        let grid = Grid::new(5, 5);
        let astar = AStar::new(grid);

        let pos = Point::new(2.0, 2.0);
        let path = astar.find_path(pos, pos, 0, Some(100));
        assert!(path.is_some());

        let path = path.unwrap();
        assert_eq!(path.len(), 1);
        assert_eq!(path[0], pos);
    }

    #[test]
    fn test_path_horizontal() {
        let grid = Grid::new(10, 10);
        let astar = AStar::new(grid);

        let start = Point::new(0.0, 5.0);
        let goal = Point::new(9.0, 5.0);

        let path = astar.find_path(start, goal, 0, Some(100));
        assert!(path.is_some());

        let path = path.unwrap();
        assert_eq!(path.first().unwrap(), &start);
        assert_eq!(path.last().unwrap(), &goal);
        // Optimal path length is 10 (including start)
        assert_eq!(path.len(), 10);
    }

    #[test]
    fn test_path_vertical() {
        let grid = Grid::new(10, 10);
        let astar = AStar::new(grid);

        let start = Point::new(5.0, 0.0);
        let goal = Point::new(5.0, 9.0);

        let path = astar.find_path(start, goal, 0, Some(100));
        assert!(path.is_some());

        let path = path.unwrap();
        assert_eq!(path.len(), 10);
    }

    // ==================== find_path Edge Cases ====================

    #[test]
    fn test_path_invalid_start() {
        let grid = Grid::new(5, 5);
        let astar = AStar::new(grid);

        let start = Point::new(-1.0, 0.0);
        let goal = Point::new(4.0, 4.0);

        let path = astar.find_path(start, goal, 0, Some(100));
        assert!(path.is_none());
    }

    #[test]
    fn test_path_invalid_goal() {
        let grid = Grid::new(5, 5);
        let astar = AStar::new(grid);

        let start = Point::new(0.0, 0.0);
        let goal = Point::new(10.0, 10.0);

        let path = astar.find_path(start, goal, 0, Some(100));
        assert!(path.is_none());
    }

    #[test]
    fn test_path_start_on_obstacle() {
        let mut grid = Grid::new(5, 5);
        grid.set_obstacle(0, 0, true);
        let astar = AStar::new(grid);

        let start = Point::new(0.0, 0.0);
        let goal = Point::new(4.0, 4.0);

        let path = astar.find_path(start, goal, 0, Some(100));
        assert!(path.is_none());
    }

    #[test]
    fn test_path_goal_on_obstacle() {
        let mut grid = Grid::new(5, 5);
        grid.set_obstacle(4, 4, true);
        let astar = AStar::new(grid);

        let start = Point::new(0.0, 0.0);
        let goal = Point::new(4.0, 4.0);

        let path = astar.find_path(start, goal, 0, Some(100));
        assert!(path.is_none());
    }

    #[test]
    fn test_path_unreachable_surrounded() {
        let mut grid = Grid::new(5, 5);
        // Surround goal with obstacles
        grid.set_obstacle(3, 3, true);
        grid.set_obstacle(3, 4, true);
        grid.set_obstacle(4, 3, true);
        let astar = AStar::new(grid);

        let start = Point::new(0.0, 0.0);
        let goal = Point::new(4.0, 4.0);

        let path = astar.find_path(start, goal, 0, Some(100));
        assert!(path.is_none());
    }

    // ==================== find_path with Obstacles ====================

    #[test]
    fn test_path_with_obstacles() {
        let mut grid = Grid::new(5, 5);
        grid.set_obstacle(2, 2, true);
        grid.set_obstacle(2, 1, true);
        grid.set_obstacle(2, 3, true);

        let astar = AStar::new(grid);
        let start = Point::new(0.0, 2.0);
        let goal = Point::new(4.0, 2.0);

        let path = astar.find_path(start, goal, 0, Some(100));
        assert!(path.is_some());

        // Path should go around the obstacles
        let path = path.unwrap();
        assert!(path.len() > 5); // Should be longer than direct path
    }

    #[test]
    fn test_path_around_wall() {
        let mut grid = Grid::new(10, 10);
        // Create a wall from (5, 0) to (5, 8)
        for y in 0..9 {
            grid.set_obstacle(5, y, true);
        }

        let astar = AStar::new(grid);
        let start = Point::new(0.0, 5.0);
        let goal = Point::new(9.0, 5.0);

        let path = astar.find_path(start, goal, 0, Some(200));
        assert!(path.is_some());

        let path = path.unwrap();
        // Must go around the wall
        assert!(path.len() > 10);
    }

    // ==================== find_path with Constraints ====================

    #[test]
    fn test_path_with_constraints() {
        let grid = Grid::new(5, 5);
        let constraints = vec![
            Constraint::new(0, (1, 0), 1),
            Constraint::new(0, (2, 0), 2),
        ];

        let astar = AStar::with_constraints(grid, &constraints, 0);
        let start = Point::new(0.0, 0.0);
        let goal = Point::new(4.0, 0.0);

        let path = astar.find_path(start, goal, 0, Some(100));
        assert!(path.is_some());

        // Should find alternative path avoiding constraints
        let path = path.unwrap();
        assert!(path.len() > 5);
    }

    #[test]
    fn test_path_constraints_different_agent_ignored() {
        let grid = Grid::new(5, 5);
        // These constraints are for agent 1, not agent 0
        let constraints = vec![
            Constraint::new(1, (1, 0), 1),
            Constraint::new(1, (2, 0), 2),
            Constraint::new(1, (3, 0), 3),
        ];

        let astar = AStar::with_constraints(grid, &constraints, 0);
        let start = Point::new(0.0, 0.0);
        let goal = Point::new(4.0, 0.0);

        let path = astar.find_path(start, goal, 0, Some(100));
        assert!(path.is_some());

        // Should take direct path since constraints don't apply
        let path = path.unwrap();
        assert_eq!(path.len(), 5);
    }

    #[test]
    fn test_path_multiple_time_constraints() {
        let grid = Grid::new(5, 1);
        // Block straight path at various times
        let constraints = vec![
            Constraint::new(0, (1, 0), 1),
            Constraint::new(0, (2, 0), 2),
            Constraint::new(0, (3, 0), 3),
        ];

        let astar = AStar::with_constraints(grid, &constraints, 0);
        let start = Point::new(0.0, 0.0);
        let goal = Point::new(4.0, 0.0);

        let path = astar.find_path(start, goal, 0, Some(100));
        assert!(path.is_some());

        // Agent must wait or find alternative timing
        let path = path.unwrap();
        assert!(path.len() > 5);
    }

    // ==================== find_path Max Time Steps ====================

    #[test]
    fn test_path_max_time_exceeded() {
        let grid = Grid::new(10, 10);
        let astar = AStar::new(grid);

        let start = Point::new(0.0, 0.0);
        let goal = Point::new(9.0, 9.0);

        // Very short max time
        let path = astar.find_path(start, goal, 0, Some(5));
        // Should fail because goal is too far
        assert!(path.is_none());
    }

    #[test]
    fn test_path_exact_max_time() {
        let grid = Grid::new(5, 1);
        let astar = AStar::new(grid);

        let start = Point::new(0.0, 0.0);
        let goal = Point::new(4.0, 0.0);

        // Exactly enough time
        let path = astar.find_path(start, goal, 0, Some(4));
        assert!(path.is_some());
    }

    // ==================== find_single_agent_path Tests ====================

    #[test]
    fn test_find_single_agent_path_basic() {
        let grid = Grid::new(10, 10);
        let constraints: Vec<Constraint> = vec![];

        let start = Point::new(0.0, 0.0);
        let goal = Point::new(5.0, 5.0);

        let path = find_single_agent_path(&grid, start, goal, &constraints, 0);
        assert!(path.is_some());

        let path = path.unwrap();
        assert_eq!(path.first().unwrap(), &start);
        assert_eq!(path.last().unwrap(), &goal);
    }

    #[test]
    fn test_find_single_agent_path_with_constraints() {
        let grid = Grid::new(10, 10);
        let constraints = vec![
            Constraint::new(0, (1, 0), 1),
            Constraint::new(0, (0, 1), 1),
        ];

        let start = Point::new(0.0, 0.0);
        let goal = Point::new(5.0, 5.0);

        let path = find_single_agent_path(&grid, start, goal, &constraints, 0);
        assert!(path.is_some());
    }

    #[test]
    fn test_find_single_agent_path_with_obstacles() {
        let mut grid = Grid::new(10, 10);
        grid.set_obstacle(1, 0, true);
        grid.set_obstacle(0, 1, true);
        grid.set_obstacle(1, 1, true);

        let constraints: Vec<Constraint> = vec![];
        let start = Point::new(0.0, 0.0);
        let goal = Point::new(5.0, 5.0);

        let path = find_single_agent_path(&grid, start, goal, &constraints, 0);
        // Should be unreachable - start is surrounded
        assert!(path.is_none());
    }

    // ==================== Large Grid Tests ====================

    #[test]
    fn test_path_large_grid() {
        let grid = Grid::new(50, 50);
        let astar = AStar::new(grid);

        let start = Point::new(0.0, 0.0);
        let goal = Point::new(49.0, 49.0);

        let path = astar.find_path(start, goal, 0, Some(200));
        assert!(path.is_some());

        let path = path.unwrap();
        assert_eq!(path.first().unwrap(), &start);
        assert_eq!(path.last().unwrap(), &goal);
    }

    #[test]
    fn test_path_large_grid_with_maze() {
        let mut grid = Grid::new(20, 20);
        // Create a simple maze pattern
        for i in 0..18 {
            grid.set_obstacle(5, i, true);
            grid.set_obstacle(10, 19 - i, true);
            grid.set_obstacle(15, i, true);
        }

        let astar = AStar::new(grid);
        let start = Point::new(0.0, 10.0);
        let goal = Point::new(19.0, 10.0);

        let path = astar.find_path(start, goal, 0, Some(500));
        assert!(path.is_some());
    }
}