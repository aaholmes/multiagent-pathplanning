//! # Conflict-Based Search (CBS)
//!
//! Optimal multi-agent pathfinding algorithm that finds collision-free paths.
//!
//! ## Algorithm
//!
//! CBS operates in two levels:
//! 1. **High-level**: Searches a constraint tree (CT) where each node represents
//!    a set of constraints. Expands nodes by detecting conflicts and adding constraints.
//! 2. **Low-level**: Uses A* to find optimal paths for individual agents subject
//!    to the constraints at that CT node.
//!
//! ## Conflict Types
//!
//! - **Vertex conflict**: Two agents at the same position at the same time
//! - **Edge conflict**: Two agents swap positions (cross paths) at the same time
//!
//! ## Reference
//!
//! Sharon et al., "Conflict-Based Search for Optimal Multi-Agent Pathfinding" (AAAI 2012)

use crate::astar::find_single_agent_path;
use crate::structs::{Grid, Task, Path, Constraint, Conflict, CTNode};
use ordered_float::OrderedFloat;
use std::collections::{HashMap, BinaryHeap};
use std::cmp::Ordering;

pub struct ConflictBasedSearch {
    grid: Grid,
    tasks: Vec<Task>,
}

impl ConflictBasedSearch {
    pub fn new(grid: Grid, tasks: Vec<Task>) -> Self {
        ConflictBasedSearch { grid, tasks }
    }

    pub fn solve(&self) -> Option<HashMap<usize, Path>> {
        // Use a simpler approach with BinaryHeap and wrapper struct
        #[derive(Clone)]
        struct PriorityNode {
            node: CTNode,
            priority: OrderedFloat<f64>,
        }
        
        impl PartialEq for PriorityNode {
            fn eq(&self, other: &Self) -> bool {
                self.priority == other.priority
            }
        }
        
        impl Eq for PriorityNode {}
        
        impl PartialOrd for PriorityNode {
            fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
                Some(self.cmp(other))
            }
        }
        
        impl Ord for PriorityNode {
            fn cmp(&self, other: &Self) -> Ordering {
                other.priority.cmp(&self.priority) // Reverse for min-heap
            }
        }

        let mut open_list = BinaryHeap::new();

        // Create root node with no constraints
        let mut root = CTNode::new();

        // Find initial solution for all agents
        if let Some(initial_solution) = self.find_solution(&root.constraints) {
            root.solution = initial_solution;
            root.cost = self.calculate_total_cost(&root.solution);
            let priority = OrderedFloat(root.cost as f64);
            open_list.push(PriorityNode { node: root, priority });
        } else {
            // No initial solution possible
            return None;
        }

        while let Some(priority_node) = open_list.pop() {
            let current_node = priority_node.node;

            // Goal test on pop: the popped node has minimum cost in OPEN, so a
            // conflict-free solution found here is optimal (Sharon et al. 2015).
            // Testing children at generation instead would forfeit optimality.
            let Some(conflict) = self.find_first_conflict(&current_node.solution) else {
                return Some(current_node.solution);
            };

            // Create two child nodes with additional constraints. Only the
            // newly-constrained agent needs replanning; the other agents'
            // paths are unaffected and are reused from the parent.
            for mut child in self.generate_child_nodes(&current_node, &conflict) {
                let constrained_agent = child
                    .constraints
                    .last()
                    .expect("child node always adds a constraint")
                    .agent_id;
                let Some(task) = self.tasks.iter().find(|t| t.agent_id == constrained_agent)
                else {
                    continue;
                };

                if let Some(path) = find_single_agent_path(
                    &self.grid,
                    task.start,
                    task.goal,
                    &child.constraints,
                    constrained_agent,
                ) {
                    child.solution = current_node.solution.clone();
                    child.solution.insert(constrained_agent, path);
                    child.cost = self.calculate_total_cost(&child.solution);
                    let priority = OrderedFloat(child.cost as f64);
                    open_list.push(PriorityNode { node: child, priority });
                }
            }
        }

        None
    }

    fn find_solution(&self, constraints: &[Constraint]) -> Option<HashMap<usize, Path>> {
        let mut solution = HashMap::new();
        
        for task in &self.tasks {
            if let Some(path) = find_single_agent_path(
                &self.grid,
                task.start,
                task.goal,
                constraints,
                task.agent_id,
            ) {
                solution.insert(task.agent_id, path);
            } else {
                return None;
            }
        }
        
        Some(solution)
    }

    fn find_first_conflict(&self, solution: &HashMap<usize, Path>) -> Option<Conflict> {
        // Sort agent IDs: HashMap iteration order is nondeterministic, which
        // would make conflict selection (and thus returned costs) vary per run.
        let mut agents: Vec<_> = solution.keys().cloned().collect();
        agents.sort_unstable();

        for i in 0..agents.len() {
            for j in (i + 1)..agents.len() {
                let agent_a = agents[i];
                let agent_b = agents[j];
                
                if let (Some(path_a), Some(path_b)) = (solution.get(&agent_a), solution.get(&agent_b)) {
                    if let Some(conflict) = self.detect_conflict(agent_a, path_a, agent_b, path_b) {
                        return Some(conflict);
                    }
                }
            }
        }
        
        None
    }

    fn detect_conflict(&self, agent_a: usize, path_a: &Path, agent_b: usize, path_b: &Path) -> Option<Conflict> {
        let max_length = path_a.len().max(path_b.len());
        
        for t in 0..max_length {
            let pos_a = if t < path_a.len() {
                (path_a[t].x.round() as i32, path_a[t].y.round() as i32)
            } else {
                // Agent stays at goal
                let last = path_a.last()?;
                (last.x.round() as i32, last.y.round() as i32)
            };
            
            let pos_b = if t < path_b.len() {
                (path_b[t].x.round() as i32, path_b[t].y.round() as i32)
            } else {
                // Agent stays at goal
                let last = path_b.last()?;
                (last.x.round() as i32, last.y.round() as i32)
            };
            
            // Vertex conflict: same position at same time
            if pos_a == pos_b {
                return Some(Conflict::new(agent_a, agent_b, pos_a, t));
            }
            
            // Edge conflict: agents swap positions
            if t > 0 {
                let prev_pos_a = if t - 1 < path_a.len() {
                    (path_a[t - 1].x.round() as i32, path_a[t - 1].y.round() as i32)
                } else {
                    pos_a
                };
                
                let prev_pos_b = if t - 1 < path_b.len() {
                    (path_b[t - 1].x.round() as i32, path_b[t - 1].y.round() as i32)
                } else {
                    pos_b
                };
                
                if pos_a == prev_pos_b && pos_b == prev_pos_a && pos_a != pos_b {
                    // Swap conflict: A moved prev_pos_a -> pos_a, B the reverse
                    return Some(Conflict::edge(agent_a, agent_b, pos_a, prev_pos_a, t));
                }
            }
        }
        
        None
    }

    fn generate_child_nodes(&self, parent: &CTNode, conflict: &Conflict) -> Vec<CTNode> {
        // Each child forbids one agent's side of the conflict, so every
        // solution survives in at least one branch (required for optimality).
        let (constraint_a, constraint_b) = match conflict.prev_position {
            // Vertex conflict: both agents at `position` at `time_step`
            None => (
                Constraint::new(conflict.agent_a, conflict.position, conflict.time_step),
                Constraint::new(conflict.agent_b, conflict.position, conflict.time_step),
            ),
            // Edge (swap) conflict: A traversed prev -> position, B the reverse
            Some(prev) => (
                Constraint::edge(conflict.agent_a, prev, conflict.position, conflict.time_step),
                Constraint::edge(conflict.agent_b, conflict.position, prev, conflict.time_step),
            ),
        };

        let mut constraints_a = parent.constraints.clone();
        constraints_a.push(constraint_a);
        let mut constraints_b = parent.constraints.clone();
        constraints_b.push(constraint_b);

        vec![
            CTNode::with_constraints(constraints_a),
            CTNode::with_constraints(constraints_b),
        ]
    }

    fn calculate_total_cost(&self, solution: &HashMap<usize, Path>) -> usize {
        solution.values().map(|path| path.len()).sum()
    }
}

pub fn solve_cbs(grid: Grid, tasks: Vec<Task>) -> Option<HashMap<usize, Path>> {
    let cbs = ConflictBasedSearch::new(grid, tasks);
    cbs.solve()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::structs::Point;

    // ==================== Constructor Tests ====================

    #[test]
    fn test_cbs_new() {
        let grid = Grid::new(10, 10);
        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(5.0, 5.0)),
        ];
        let cbs = ConflictBasedSearch::new(grid.clone(), tasks.clone());
        assert_eq!(cbs.grid.width, 10);
        assert_eq!(cbs.grid.height, 10);
        assert_eq!(cbs.tasks.len(), 1);
    }

    #[test]
    fn test_cbs_new_empty_tasks() {
        let grid = Grid::new(10, 10);
        let tasks: Vec<Task> = vec![];
        let cbs = ConflictBasedSearch::new(grid, tasks);
        assert!(cbs.tasks.is_empty());
    }

    // ==================== find_solution Tests ====================

    #[test]
    fn test_find_solution_single_agent() {
        let grid = Grid::new(10, 10);
        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(5.0, 5.0)),
        ];
        let cbs = ConflictBasedSearch::new(grid, tasks);
        let constraints: Vec<Constraint> = vec![];

        let solution = cbs.find_solution(&constraints);
        assert!(solution.is_some());
        let solution = solution.unwrap();
        assert_eq!(solution.len(), 1);
        assert!(solution.contains_key(&0));
    }

    #[test]
    fn test_find_solution_with_constraints() {
        let grid = Grid::new(10, 10);
        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(5.0, 0.0)),
        ];
        let cbs = ConflictBasedSearch::new(grid, tasks);
        let constraints = vec![
            Constraint::new(0, (1, 0), 1),
            Constraint::new(0, (2, 0), 2),
        ];

        let solution = cbs.find_solution(&constraints);
        assert!(solution.is_some());
    }

    #[test]
    fn test_find_solution_impossible() {
        let mut grid = Grid::new(5, 5);
        // Surround start
        grid.set_obstacle(1, 0, true);
        grid.set_obstacle(0, 1, true);

        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(4.0, 4.0)),
        ];
        let cbs = ConflictBasedSearch::new(grid, tasks);
        let constraints: Vec<Constraint> = vec![];

        let solution = cbs.find_solution(&constraints);
        assert!(solution.is_none());
    }

    // ==================== find_first_conflict Tests ====================

    #[test]
    fn test_find_first_conflict_no_conflict() {
        let grid = Grid::new(10, 10);
        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(5.0, 0.0)),
            Task::new(1, Point::new(0.0, 5.0), Point::new(5.0, 5.0)),
        ];
        let cbs = ConflictBasedSearch::new(grid, tasks);

        let mut solution = HashMap::new();
        solution.insert(0, vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(2.0, 0.0),
        ]);
        solution.insert(1, vec![
            Point::new(0.0, 5.0),
            Point::new(1.0, 5.0),
            Point::new(2.0, 5.0),
        ]);

        assert!(cbs.find_first_conflict(&solution).is_none());
    }

    #[test]
    fn test_find_first_conflict_vertex_conflict() {
        let grid = Grid::new(10, 10);
        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(2.0, 0.0)),
            Task::new(1, Point::new(2.0, 0.0), Point::new(0.0, 0.0)),
        ];
        let cbs = ConflictBasedSearch::new(grid, tasks);

        let mut solution = HashMap::new();
        solution.insert(0, vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0), // Conflict here
            Point::new(2.0, 0.0),
        ]);
        solution.insert(1, vec![
            Point::new(2.0, 0.0),
            Point::new(1.0, 0.0), // Conflict here
            Point::new(0.0, 0.0),
        ]);

        let conflict = cbs.find_first_conflict(&solution);
        assert!(conflict.is_some());
        let conflict = conflict.unwrap();
        assert_eq!(conflict.position, (1, 0));
        assert_eq!(conflict.time_step, 1);
    }

    #[test]
    fn test_find_first_conflict_edge_conflict() {
        let grid = Grid::new(10, 10);
        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(1.0, 0.0)),
            Task::new(1, Point::new(1.0, 0.0), Point::new(0.0, 0.0)),
        ];
        let cbs = ConflictBasedSearch::new(grid, tasks);

        // Agents swap positions
        let mut solution = HashMap::new();
        solution.insert(0, vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
        ]);
        solution.insert(1, vec![
            Point::new(1.0, 0.0),
            Point::new(0.0, 0.0),
        ]);

        let conflict = cbs.find_first_conflict(&solution);
        assert!(conflict.is_some());
    }

    // ==================== detect_conflict Tests ====================

    #[test]
    fn test_detect_conflict_same_position() {
        let grid = Grid::new(10, 10);
        let cbs = ConflictBasedSearch::new(grid, vec![]);

        let path_a = vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
        ];
        let path_b = vec![
            Point::new(2.0, 0.0),
            Point::new(1.0, 0.0), // Same as path_a at t=1
        ];

        let conflict = cbs.detect_conflict(0, &path_a, 1, &path_b);
        assert!(conflict.is_some());
        let conflict = conflict.unwrap();
        assert_eq!(conflict.position, (1, 0));
        assert_eq!(conflict.time_step, 1);
    }

    #[test]
    fn test_detect_conflict_agent_waits_at_goal() {
        let grid = Grid::new(10, 10);
        let cbs = ConflictBasedSearch::new(grid, vec![]);

        // Agent 0 reaches goal early
        let path_a = vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
        ];
        // Agent 1 arrives at agent 0's goal later
        let path_b = vec![
            Point::new(3.0, 0.0),
            Point::new(2.0, 0.0),
            Point::new(1.0, 0.0), // Conflict at t=2
        ];

        let conflict = cbs.detect_conflict(0, &path_a, 1, &path_b);
        assert!(conflict.is_some());
        let conflict = conflict.unwrap();
        assert_eq!(conflict.time_step, 2);
    }

    #[test]
    fn test_detect_conflict_no_conflict() {
        let grid = Grid::new(10, 10);
        let cbs = ConflictBasedSearch::new(grid, vec![]);

        let path_a = vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(2.0, 0.0),
        ];
        let path_b = vec![
            Point::new(0.0, 1.0),
            Point::new(1.0, 1.0),
            Point::new(2.0, 1.0),
        ];

        let conflict = cbs.detect_conflict(0, &path_a, 1, &path_b);
        assert!(conflict.is_none());
    }

    // ==================== generate_child_nodes Tests ====================

    #[test]
    fn test_generate_child_nodes() {
        let grid = Grid::new(10, 10);
        let cbs = ConflictBasedSearch::new(grid, vec![]);

        let parent = CTNode::new();
        let conflict = Conflict::new(0, 1, (5, 5), 10);

        let children = cbs.generate_child_nodes(&parent, &conflict);

        assert_eq!(children.len(), 2);

        // First child has constraint for agent 0
        assert_eq!(children[0].constraints.len(), 1);
        assert_eq!(children[0].constraints[0].agent_id, 0);
        assert_eq!(children[0].constraints[0].position, (5, 5));
        assert_eq!(children[0].constraints[0].time_step, 10);

        // Second child has constraint for agent 1
        assert_eq!(children[1].constraints.len(), 1);
        assert_eq!(children[1].constraints[0].agent_id, 1);
    }

    #[test]
    fn test_generate_child_nodes_inherits_parent_constraints() {
        let grid = Grid::new(10, 10);
        let cbs = ConflictBasedSearch::new(grid, vec![]);

        let parent = CTNode::with_constraints(vec![
            Constraint::new(0, (1, 1), 1),
            Constraint::new(1, (2, 2), 2),
        ]);
        let conflict = Conflict::new(0, 1, (5, 5), 10);

        let children = cbs.generate_child_nodes(&parent, &conflict);

        // Both children should have 3 constraints (2 from parent + 1 new)
        assert_eq!(children[0].constraints.len(), 3);
        assert_eq!(children[1].constraints.len(), 3);
    }

    // ==================== calculate_total_cost Tests ====================

    #[test]
    fn test_calculate_total_cost_empty() {
        let grid = Grid::new(10, 10);
        let cbs = ConflictBasedSearch::new(grid, vec![]);

        let solution: HashMap<usize, Path> = HashMap::new();
        assert_eq!(cbs.calculate_total_cost(&solution), 0);
    }

    #[test]
    fn test_calculate_total_cost_single_agent() {
        let grid = Grid::new(10, 10);
        let cbs = ConflictBasedSearch::new(grid, vec![]);

        let mut solution = HashMap::new();
        solution.insert(0, vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(2.0, 0.0),
        ]);

        assert_eq!(cbs.calculate_total_cost(&solution), 3);
    }

    #[test]
    fn test_calculate_total_cost_multiple_agents() {
        let grid = Grid::new(10, 10);
        let cbs = ConflictBasedSearch::new(grid, vec![]);

        let mut solution = HashMap::new();
        solution.insert(0, vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
        ]); // Cost: 2
        solution.insert(1, vec![
            Point::new(0.0, 1.0),
            Point::new(1.0, 1.0),
            Point::new(2.0, 1.0),
        ]); // Cost: 3

        assert_eq!(cbs.calculate_total_cost(&solution), 5);
    }

    // ==================== solve_cbs Integration Tests ====================

    #[test]
    fn test_cbs_simple_case() {
        let grid = Grid::new(5, 5);
        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(4.0, 0.0)),
            Task::new(1, Point::new(4.0, 0.0), Point::new(0.0, 0.0)),
        ];

        let solution = solve_cbs(grid, tasks);
        assert!(solution.is_some());

        let solution = solution.unwrap();
        assert_eq!(solution.len(), 2);
        assert!(solution.contains_key(&0));
        assert!(solution.contains_key(&1));
    }

    #[test]
    fn test_cbs_no_conflict() {
        let grid = Grid::new(5, 5);
        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(2.0, 0.0)),
            Task::new(1, Point::new(0.0, 2.0), Point::new(2.0, 2.0)),
        ];

        let solution = solve_cbs(grid.clone(), tasks.clone());
        assert!(solution.is_some());

        let solution = solution.unwrap();
        assert_eq!(solution.len(), 2);

        // Verify no conflicts
        let cbs = ConflictBasedSearch::new(grid, tasks);
        assert!(cbs.find_first_conflict(&solution).is_none());
    }

    #[test]
    fn test_cbs_impossible_case() {
        let mut grid = Grid::new(3, 1);
        // Block the middle
        grid.set_obstacle(1, 0, true);

        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(2.0, 0.0)),
            Task::new(1, Point::new(2.0, 0.0), Point::new(0.0, 0.0)),
        ];

        let solution = solve_cbs(grid, tasks);
        assert!(solution.is_none());
    }

    #[test]
    fn test_cbs_single_agent() {
        let grid = Grid::new(10, 10);
        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(5.0, 5.0)),
        ];

        let solution = solve_cbs(grid, tasks);
        assert!(solution.is_some());

        let solution = solution.unwrap();
        assert_eq!(solution.len(), 1);

        let path = solution.get(&0).unwrap();
        assert_eq!(path.first().unwrap(), &Point::new(0.0, 0.0));
        assert_eq!(path.last().unwrap(), &Point::new(5.0, 5.0));
    }

    #[test]
    fn test_cbs_three_agents() {
        let grid = Grid::new(10, 10);
        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(9.0, 0.0)),
            Task::new(1, Point::new(9.0, 0.0), Point::new(0.0, 0.0)),
            Task::new(2, Point::new(5.0, 0.0), Point::new(5.0, 9.0)),
        ];

        let solution = solve_cbs(grid.clone(), tasks.clone());
        assert!(solution.is_some());

        let solution = solution.unwrap();
        assert_eq!(solution.len(), 3);

        // Verify no conflicts
        let cbs = ConflictBasedSearch::new(grid, tasks);
        assert!(cbs.find_first_conflict(&solution).is_none());
    }

    #[test]
    fn test_cbs_four_corners() {
        let grid = Grid::new(5, 5);
        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(4.0, 4.0)),
            Task::new(1, Point::new(4.0, 0.0), Point::new(0.0, 4.0)),
            Task::new(2, Point::new(4.0, 4.0), Point::new(0.0, 0.0)),
            Task::new(3, Point::new(0.0, 4.0), Point::new(4.0, 0.0)),
        ];

        let solution = solve_cbs(grid.clone(), tasks.clone());
        assert!(solution.is_some());

        let solution = solution.unwrap();
        assert_eq!(solution.len(), 4);

        // Verify no conflicts
        let cbs = ConflictBasedSearch::new(grid, tasks);
        assert!(cbs.find_first_conflict(&solution).is_none());
    }

    #[test]
    fn test_cbs_with_obstacles() {
        let mut grid = Grid::new(10, 10);
        // Create some obstacles
        grid.set_obstacle(5, 4, true);
        grid.set_obstacle(5, 5, true);
        grid.set_obstacle(5, 6, true);

        let tasks = vec![
            Task::new(0, Point::new(0.0, 5.0), Point::new(9.0, 5.0)),
            Task::new(1, Point::new(9.0, 5.0), Point::new(0.0, 5.0)),
        ];

        let solution = solve_cbs(grid.clone(), tasks.clone());
        assert!(solution.is_some());

        let solution = solution.unwrap();

        // Verify no conflicts
        let cbs = ConflictBasedSearch::new(grid, tasks);
        assert!(cbs.find_first_conflict(&solution).is_none());
    }

    #[test]
    fn test_cbs_narrow_corridor() {
        // Test narrow corridor where agents move in same direction
        // Key: Agent goals must not block each other's paths
        let mut grid = Grid::new(6, 3);
        // Create walls leaving only middle row open
        for x in 0..6 {
            grid.set_obstacle(x, 0, true);
            grid.set_obstacle(x, 2, true);
        }

        // Agent 0 starts behind Agent 1, but Agent 0's goal is BEFORE Agent 1's goal
        // This way Agent 0 can stop and let Agent 1 pass through
        // Agent 0: (0,1) -> (2,1)
        // Agent 1: (1,1) -> (5,1)
        let tasks = vec![
            Task::new(0, Point::new(0.0, 1.0), Point::new(2.0, 1.0)),
            Task::new(1, Point::new(1.0, 1.0), Point::new(5.0, 1.0)),
        ];

        let solution = solve_cbs(grid.clone(), tasks.clone());
        assert!(solution.is_some());

        let solution = solution.unwrap();

        // Verify no conflicts
        let cbs = ConflictBasedSearch::new(grid, tasks);
        assert!(cbs.find_first_conflict(&solution).is_none());
    }

    #[test]
    fn test_cbs_same_start_goal() {
        let grid = Grid::new(10, 10);
        let tasks = vec![
            Task::new(0, Point::new(5.0, 5.0), Point::new(5.0, 5.0)),
        ];

        let solution = solve_cbs(grid, tasks);
        assert!(solution.is_some());

        let solution = solution.unwrap();
        let path = solution.get(&0).unwrap();
        assert_eq!(path.len(), 1);
    }

    #[test]
    fn test_cbs_solution_validity() {
        let grid = Grid::new(10, 10);
        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(5.0, 5.0)),
            Task::new(1, Point::new(9.0, 9.0), Point::new(4.0, 4.0)),
        ];

        let solution = solve_cbs(grid, tasks.clone());
        assert!(solution.is_some());

        let solution = solution.unwrap();

        // Verify each path starts and ends at correct positions
        for task in &tasks {
            let path = solution.get(&task.agent_id).unwrap();
            assert_eq!(path.first().unwrap(), &task.start);
            assert_eq!(path.last().unwrap(), &task.goal);
        }
    }

    #[test]
    fn test_cbs_empty_tasks() {
        let grid = Grid::new(10, 10);
        let tasks: Vec<Task> = vec![];

        let solution = solve_cbs(grid, tasks);
        assert!(solution.is_some());

        let solution = solution.unwrap();
        assert!(solution.is_empty());
    }

    // ==================== Correctness Regression Tests ====================
    // These cover the three CBS defects found in review: suboptimal/
    // nondeterministic solutions, vacuous edge-conflict constraints, and
    // non-termination when an agent must delay arrival at its goal.

    /// Asserts that every path is physically legal: starts/ends on the task
    /// endpoints, every step is a 4-neighbor move or a wait, and no waypoint
    /// is on an obstacle.
    fn assert_paths_legal(solution: &HashMap<usize, Path>, grid: &Grid, tasks: &[Task]) {
        for task in tasks {
            let path = solution.get(&task.agent_id).expect("missing agent path");
            assert!(!path.is_empty());
            assert!(path[0].distance(&task.start) < 0.5, "path must start at task start");
            assert!(path.last().unwrap().distance(&task.goal) < 0.5, "path must end at task goal");
            for window in path.windows(2) {
                let (x0, y0) = (window[0].x.round() as i32, window[0].y.round() as i32);
                let (x1, y1) = (window[1].x.round() as i32, window[1].y.round() as i32);
                let step = (x1 - x0).abs() + (y1 - y0).abs();
                assert!(step <= 1, "step from ({},{}) to ({},{}) is not a 4-neighbor move or wait", x0, y0, x1, y1);
                assert!(!grid.is_obstacle(x1 as usize, y1 as usize), "path enters obstacle at ({},{})", x1, y1);
            }
        }
    }

    #[test]
    fn test_cbs_head_on_swap_is_optimal_and_deterministic() {
        // Two agents swap places on an open 5x5 grid. The optimal resolution
        // is one agent detouring a single cell: 5 + 7 = 12 total waypoints.
        // Before the goal-test-on-pop fix this returned 12 or 13 depending on
        // HashMap iteration order.
        let grid = Grid::new(5, 5);
        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(4.0, 0.0)),
            Task::new(1, Point::new(4.0, 0.0), Point::new(0.0, 0.0)),
        ];

        for _ in 0..5 {
            let solution = solve_cbs(grid.clone(), tasks.clone()).expect("solvable");
            let cbs = ConflictBasedSearch::new(grid.clone(), tasks.clone());
            assert!(cbs.find_first_conflict(&solution).is_none());
            assert_paths_legal(&solution, &grid, &tasks);
            let total: usize = solution.values().map(|p| p.len()).sum();
            assert_eq!(total, 12, "head-on swap must always return the optimal cost");
        }
    }

    #[test]
    fn test_cbs_agent_must_delay_goal_arrival() {
        // Agent 1 starts on its own goal, directly on agent 0's only route.
        // It must step aside and return - which requires A* to honor
        // constraints on the goal cell at times after first arrival.
        // Before the goal-deadline fix this instance never terminated.
        let mut grid = Grid::new(4, 2);
        grid.set_obstacle(0, 1, true);
        grid.set_obstacle(3, 1, true); // leave (1,1) and (2,1) as side pockets

        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(3.0, 0.0)),
            Task::new(1, Point::new(1.0, 0.0), Point::new(1.0, 0.0)),
        ];

        let solution = solve_cbs(grid.clone(), tasks.clone()).expect("solvable: agent 1 can step aside");
        let cbs = ConflictBasedSearch::new(grid.clone(), tasks.clone());
        assert!(cbs.find_first_conflict(&solution).is_none());
        assert_paths_legal(&solution, &grid, &tasks);
        assert!(solution.get(&1).unwrap().len() > 1, "agent 1 must move out of the way");
    }

    #[test]
    fn test_cbs_corridor_swap_resolved_with_edge_constraints() {
        // Head-on swap in a width-1 corridor with a single passing pocket.
        // The agents meet mid-corridor as an edge conflict; resolving it
        // requires real edge constraints (a vertex constraint on the second
        // agent is satisfied by its existing path and changes nothing).
        let mut grid = Grid::new(5, 2);
        for x in 0..5 {
            if x != 2 {
                grid.set_obstacle(x, 1, true); // only pocket: (2,1)
            }
        }

        let tasks = vec![
            Task::new(0, Point::new(0.0, 0.0), Point::new(4.0, 0.0)),
            Task::new(1, Point::new(4.0, 0.0), Point::new(0.0, 0.0)),
        ];

        let solution = solve_cbs(grid.clone(), tasks.clone()).expect("solvable via the pocket");
        let cbs = ConflictBasedSearch::new(grid.clone(), tasks.clone());
        assert!(cbs.find_first_conflict(&solution).is_none());
        assert_paths_legal(&solution, &grid, &tasks);
    }
}
