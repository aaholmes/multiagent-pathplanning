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
}