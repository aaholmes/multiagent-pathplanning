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
}