use crate::astar::find_single_agent_path;
use crate::structs::{Grid, Task, Path, Constraint, Conflict, CTNode};
use ordered_float::OrderedFloat;
use std::collections::{HashMap, HashSet, BinaryHeap};
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
        let mut closed_list = HashSet::new();

        // Create root node with no constraints
        let mut root = CTNode::new();
        
        // Find initial solution for all agents
        if let Some(initial_solution) = self.find_solution(&root.constraints) {
            root.solution = initial_solution;
            root.cost = self.calculate_total_cost(&root.solution);
            
            // Check for conflicts
            if let Some(_conflict) = self.find_first_conflict(&root.solution) {
                open_list.push(PriorityNode {
                    node: root,
                    priority: OrderedFloat(0.0),
                });
            } else {
                // No conflicts, return solution
                return Some(root.solution);
            }
        } else {
            // No initial solution possible
            return None;
        }

        while let Some(priority_node) = open_list.pop() {
            let current_node = priority_node.node;
            let node_hash = self.hash_constraints(&current_node.constraints);
            if closed_list.contains(&node_hash) {
                continue;
            }
            closed_list.insert(node_hash);

            if let Some(conflict) = self.find_first_conflict(&current_node.solution) {
                // Create two child nodes with additional constraints
                let child_nodes = self.generate_child_nodes(&current_node, &conflict);
                
                for mut child in child_nodes {
                    if let Some(solution) = self.find_solution(&child.constraints) {
                        child.solution = solution;
                        child.cost = self.calculate_total_cost(&child.solution);
                        
                        if self.find_first_conflict(&child.solution).is_none() {
                            // Found conflict-free solution
                            return Some(child.solution);
                        }
                        
                        let priority = child.cost as f64;
                        open_list.push(PriorityNode {
                            node: child,
                            priority: OrderedFloat(priority),
                        });
                    }
                }
            } else {
                // No conflicts in current solution
                return Some(current_node.solution);
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
        let agents: Vec<_> = solution.keys().cloned().collect();
        
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
                
                if pos_a == prev_pos_b && pos_b == prev_pos_a {
                    return Some(Conflict::new(agent_a, agent_b, pos_a, t));
                }
            }
        }
        
        None
    }

    fn generate_child_nodes(&self, parent: &CTNode, conflict: &Conflict) -> Vec<CTNode> {
        let mut children = Vec::new();
        
        // Child 1: Add constraint for agent A
        let mut constraints_a = parent.constraints.clone();
        constraints_a.push(Constraint::new(conflict.agent_a, conflict.position, conflict.time_step));
        children.push(CTNode::with_constraints(constraints_a));
        
        // Child 2: Add constraint for agent B
        let mut constraints_b = parent.constraints.clone();
        constraints_b.push(Constraint::new(conflict.agent_b, conflict.position, conflict.time_step));
        children.push(CTNode::with_constraints(constraints_b));
        
        children
    }

    fn calculate_total_cost(&self, solution: &HashMap<usize, Path>) -> usize {
        solution.values().map(|path| path.len()).sum()
    }

    fn hash_constraints(&self, constraints: &[Constraint]) -> u64 {
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        
        let mut hasher = DefaultHasher::new();
        for constraint in constraints {
            constraint.hash(&mut hasher);
        }
        hasher.finish()
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

    // ==================== hash_constraints Tests ====================

    #[test]
    fn test_hash_constraints_empty() {
        let grid = Grid::new(10, 10);
        let cbs = ConflictBasedSearch::new(grid, vec![]);

        let constraints: Vec<Constraint> = vec![];
        let hash = cbs.hash_constraints(&constraints);
        // Just verify it doesn't panic and returns a value
        assert!(hash >= 0);
    }

    #[test]
    fn test_hash_constraints_deterministic() {
        let grid = Grid::new(10, 10);
        let cbs = ConflictBasedSearch::new(grid, vec![]);

        let constraints = vec![
            Constraint::new(0, (1, 1), 1),
            Constraint::new(1, (2, 2), 2),
        ];

        let hash1 = cbs.hash_constraints(&constraints);
        let hash2 = cbs.hash_constraints(&constraints);
        assert_eq!(hash1, hash2);
    }

    #[test]
    fn test_hash_constraints_different_for_different_constraints() {
        let grid = Grid::new(10, 10);
        let cbs = ConflictBasedSearch::new(grid, vec![]);

        let constraints1 = vec![Constraint::new(0, (1, 1), 1)];
        let constraints2 = vec![Constraint::new(0, (1, 1), 2)];

        let hash1 = cbs.hash_constraints(&constraints1);
        let hash2 = cbs.hash_constraints(&constraints2);
        assert_ne!(hash1, hash2);
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
}