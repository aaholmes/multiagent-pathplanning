use crate::structs::{AgentState, Vector2D};
use osqp::{CscMatrix, Problem, Settings};

const EPSILON: f64 = 1e-10;
const PERTURBATION_EPSILON: f64 = 0.0001;

/// Represents an ORCA constraint as a half-plane in velocity space.
/// A velocity 'v' is considered valid if: (v - point) · direction >= 0
#[derive(Debug, Clone)]
pub struct OrcaLine {
    /// A point on the dividing line of the half-plane
    pub point: Vector2D,
    /// The outward-pointing normal vector of the half-plane's boundary
    pub direction: Vector2D,
}

impl OrcaLine {
    pub fn new(point: Vector2D, direction: Vector2D) -> Self {
        OrcaLine { point, direction }
    }
}

/// Computes a new, safe velocity for the given agent using correct ORCA algorithm
pub fn compute_new_velocity(
    agent: &AgentState,
    neighbors: &[&AgentState],
    time_horizon: f64,
) -> Vector2D {
    // Step 1: Collect all ORCA constraints from neighbors
    let mut orca_lines = Vec::new();
    
    for &neighbor in neighbors {
        if let Some(line) = compute_orca_line_for_agent(agent, neighbor, time_horizon) {
            orca_lines.push(line);
        }
    }
    
    // Special case: no ORCA constraints, just return preferred velocity (clipped to max speed)
    if orca_lines.is_empty() {
        let pref_magnitude = agent.pref_velocity.magnitude();
        if pref_magnitude <= agent.max_speed {
            return agent.pref_velocity;
        } else {
            return agent.pref_velocity.normalize() * agent.max_speed;
        }
    }
    
    // Step 2: Attempt to solve 2D quadratic program (feasible case)
    if let Some(new_velocity) = solve_2d_quadratic_program(&orca_lines, agent.max_speed, &agent.pref_velocity) {
        return new_velocity;
    }
    
    // Step 3: Handle infeasible case with 3D linear program
    solve_3d_linear_program(&orca_lines, agent.max_speed)
}

/// Computes an ORCA line constraint for agent-agent interaction
fn compute_orca_line_for_agent(
    agent: &AgentState,
    neighbor: &AgentState,
    time_horizon: f64,
) -> Option<OrcaLine> {
    // Calculate relative state
    let relative_position = neighbor.position - agent.position;
    let combined_radius = agent.radius + neighbor.radius;
    
    // Degeneracy check: agents overlapping
    if relative_position.magnitude() < EPSILON {
        return None; // Emergency separation needed
    }
    
    // Define the truncated Velocity Obstacle (VO) in velocity space
    let vo_center = relative_position * (1.0 / time_horizon);
    let vo_radius = combined_radius / time_horizon;
    
    // Current relative velocity (this ensures reciprocity)
    let relative_velocity = agent.velocity - neighbor.velocity;
    
    // Find closest point on VO boundary to relative_velocity
    let closest_point = find_closest_point_on_vo_boundary(relative_velocity, vo_center, vo_radius);
    
    // Calculate correction vector
    let mut u = closest_point - relative_velocity;
    
    // --- DETERMINISTIC SYMMETRY-BREAKING PERTURBATION ---
    // Handle the deadlock/perfect symmetry case where u becomes zero
    if u.dot(&u) < PERTURBATION_EPSILON * PERTURBATION_EPSILON {
        // Deadlock detected. The relative velocity is on the VO boundary.
        // Use relative_position as shared reference frame for bilateral coordination.
        // This ensures both agents use the same perturbation direction but with opposite effects.
        
        let perturb_dir = if relative_position.dot(&relative_position) > EPSILON {
            // Use perpendicular to relative position vector
            // This creates a shared reference frame: both agents see the same line,
            // but agent A goes one way, agent B goes the other way
            Vector2D::new(-relative_position.y, relative_position.x).normalize()
        } else if agent.pref_velocity.dot(&agent.pref_velocity) > 0.0 {
            // Fallback: use perpendicular of preferred velocity
            Vector2D::new(-agent.pref_velocity.y, agent.pref_velocity.x).normalize()
        } else {
            // Last resort: arbitrary fixed direction
            Vector2D::new(0.0, 1.0)
        };
        
        // Overwrite the near-zero `u` vector with the tiny perturbation
        u = perturb_dir * PERTURBATION_EPSILON;
    }
    
    // Normal case: construct ORCA line with correction vector
    let orca_direction = u.normalize();
    let orca_point = agent.velocity + u * 0.5;
    
    Some(OrcaLine::new(orca_point, orca_direction))
}

/// Finds the closest point on the boundary of a truncated velocity obstacle
fn find_closest_point_on_vo_boundary(
    point: Vector2D,
    vo_center: Vector2D,
    vo_radius: f64,
) -> Vector2D {
    // Vector from VO center to query point
    let center_to_point = point - vo_center;
    let distance_to_center = center_to_point.magnitude();
    
    // Case 1: Point is inside or on the disk boundary - project to disk boundary
    if distance_to_center <= vo_radius {
        if distance_to_center < EPSILON {
            // Point is at center, choose arbitrary direction
            return vo_center + Vector2D::new(vo_radius, 0.0);
        }
        // Project to disk edge
        let direction = center_to_point.normalize();
        return vo_center + direction * vo_radius;
    }
    
    // Case 2: Point is outside disk - check cone legs
    
    // Compute tangent points from origin to disk
    let distance_sq = vo_center.dot(&vo_center);
    let distance = distance_sq.sqrt();
    
    if distance < EPSILON {
        // VO center is at origin, no cone
        let direction = center_to_point.normalize();
        return vo_center + direction * vo_radius;
    }
    
    // Tangent length from origin to disk
    let tangent_length_sq = distance_sq - vo_radius * vo_radius;
    if tangent_length_sq <= 0.0 {
        // Origin is inside disk (shouldn't happen in practice)
        let direction = center_to_point.normalize();
        return vo_center + direction * vo_radius;
    }
    
    let tangent_length = tangent_length_sq.sqrt();
    
    // Compute tangent directions
    let sin_theta = vo_radius / distance;
    let cos_theta = tangent_length / distance;
    
    // Unit vector from origin to VO center
    let to_center_unit = vo_center.normalize();
    
    // Two tangent directions
    let tangent1 = Vector2D::new(
        to_center_unit.x * cos_theta - to_center_unit.y * sin_theta,
        to_center_unit.x * sin_theta + to_center_unit.y * cos_theta,
    );
    let tangent2 = Vector2D::new(
        to_center_unit.x * cos_theta + to_center_unit.y * sin_theta,
        -to_center_unit.x * sin_theta + to_center_unit.y * cos_theta,
    );
    
    // Find closest point among: disk edge, left leg, right leg
    let mut closest = vo_center + center_to_point.normalize() * vo_radius;
    let mut min_distance_sq = (closest - point).dot(&(closest - point));
    
    // Check projection onto left tangent leg
    let proj1 = project_point_onto_ray(point, Vector2D::new(0.0, 0.0), tangent1);
    let dist1_sq = (proj1 - point).dot(&(proj1 - point));
    if dist1_sq < min_distance_sq {
        closest = proj1;
        min_distance_sq = dist1_sq;
    }
    
    // Check projection onto right tangent leg
    let proj2 = project_point_onto_ray(point, Vector2D::new(0.0, 0.0), tangent2);
    let dist2_sq = (proj2 - point).dot(&(proj2 - point));
    if dist2_sq < min_distance_sq {
        closest = proj2;
    }
    
    closest
}

/// Projects a point onto a ray (half-line) starting from origin in given direction
fn project_point_onto_ray(point: Vector2D, ray_origin: Vector2D, ray_direction: Vector2D) -> Vector2D {
    let to_point = point - ray_origin;
    let projection_length = to_point.dot(&ray_direction);
    
    if projection_length <= 0.0 {
        // Projection is behind ray origin
        ray_origin
    } else {
        // Projection is on ray
        ray_origin + ray_direction * projection_length
    }
}



/// Solves 2D quadratic program to find optimal velocity using L2 distance
/// Objective: Minimize ||v - pref_velocity||^2 = (vx - pvx)^2 + (vy - pvy)^2
/// Uses OSQP quadratic programming solver for true L2 optimization
fn solve_2d_quadratic_program(
    orca_lines: &[OrcaLine],
    max_speed: f64,
    pref_velocity: &Vector2D,
) -> Option<Vector2D> {
    // Special case: no ORCA constraints, just enforce speed limit
    if orca_lines.is_empty() {
        let pref_magnitude = pref_velocity.magnitude();
        if pref_magnitude <= max_speed {
            return Some(*pref_velocity);
        } else {
            return Some(pref_velocity.normalize() * max_speed);
        }
    }
    
    println!("Debug: Entering 2D QP solver with {} ORCA lines", orca_lines.len());
    
    // Define the QP Problem in OSQP format
    
    // q vector (linear term) for minimizing ||v - pref_velocity||^2
    let q = &[-2.0 * pref_velocity.x, -2.0 * pref_velocity.y];
    println!("Debug: q vector = [{:.3}, {:.3}]", q[0], q[1]);
    
    // Define the Linear Constraints in the form l <= Ax <= u
    let mut constraints_a: Vec<[f64; 2]> = Vec::new();
    let mut l_bounds = Vec::new();
    let mut u_bounds = Vec::new();
    
    // Add ORCA constraints: v·d >= p·d
    for (i, line) in orca_lines.iter().enumerate() {
        constraints_a.push([line.direction.x, line.direction.y]);
        let rhs = line.point.dot(&line.direction);
        l_bounds.push(rhs);
        u_bounds.push(f64::INFINITY);
        println!("Debug: ORCA constraint {}: [{:.3}, {:.3}] * v >= {:.3}", 
                 i, line.direction.x, line.direction.y, rhs);
    }
    
    // Add speed constraints using polygonal approximation
    let num_sides = 16;
    for i in 0..num_sides {
        let angle = (i as f64 / num_sides as f64) * 2.0 * std::f64::consts::PI;
        let dir = Vector2D::new(angle.cos(), angle.sin());
        constraints_a.push([dir.x, dir.y]);
        l_bounds.push(f64::NEG_INFINITY);
        u_bounds.push(max_speed);
    }
    
    // Create P matrix for OSQP - upper triangular format required
    // P matrix is [[2.0, 0.0], [0.0, 2.0]] for objective ||v - v_pref||²
    let p_matrix = CscMatrix {
        nrows: 2,
        ncols: 2,
        indptr: std::borrow::Cow::Borrowed(&[0, 1, 2]),
        indices: std::borrow::Cow::Borrowed(&[0, 1]),
        data: std::borrow::Cow::Borrowed(&[2.0, 2.0]),
    };
    
    // Build A matrix in CSC format (column-major sparse format)
    // A is m x n where m = num_constraints, n = 2 (variables vx, vy)
    let mut a_data = Vec::new();
    let mut a_indices = Vec::new();
    let mut a_indptr = vec![0];
    
    // Column 0 (vx coefficients)
    for (row_idx, row_data) in constraints_a.iter().enumerate() {
        if row_data[0] != 0.0 {
            a_data.push(row_data[0]);
            a_indices.push(row_idx);
        }
    }
    a_indptr.push(a_data.len());
    
    // Column 1 (vy coefficients)  
    for (row_idx, row_data) in constraints_a.iter().enumerate() {
        if row_data[1] != 0.0 {
            a_data.push(row_data[1]);
            a_indices.push(row_idx);
        }
    }
    a_indptr.push(a_data.len());
    
    let a_matrix = CscMatrix {
        nrows: constraints_a.len(),
        ncols: 2,
        indptr: std::borrow::Cow::Owned(a_indptr),
        indices: std::borrow::Cow::Owned(a_indices),
        data: std::borrow::Cow::Owned(a_data),
    };
    
    // Create and Solve the Problem
    let settings = Settings::default().verbose(false);
    
    let mut problem = match Problem::new(
        p_matrix,
        q,
        a_matrix,
        &l_bounds,
        &u_bounds,
        &settings,
    ) {
        Ok(p) => p,
        Err(e) => {
            return None;
        }
    };
    
    let result = problem.solve();
    match result {
        osqp::Status::Solved(solution) => {
            let sol = solution.x();
            let velocity = Vector2D::new(sol[0], sol[1]);
            println!("Debug: 2D QP solved successfully: ({:.6}, {:.6})", velocity.x, velocity.y);
            Some(velocity)
        },
        _ => {
            println!("Debug: 2D QP failed, falling back to 3D LP");
            None // Infeasible or other error
        }
    }
}

/// Projects a velocity onto the ORCA constraints and speed limit
fn project_onto_constraints(
    velocity: Vector2D,
    orca_lines: &[OrcaLine],
    max_speed: f64,
) -> Vector2D {
    let mut projected_v = velocity;
    
    // Project onto ORCA line constraints
    for line in orca_lines {
        let constraint_val = (projected_v - line.point).dot(&line.direction);
        if constraint_val < 0.0 {
            // Violates constraint, project onto the line
            projected_v = projected_v - line.direction * constraint_val;
        }
    }
    
    // Project onto speed constraint
    if projected_v.magnitude() > max_speed {
        if projected_v.magnitude() > EPSILON {
            projected_v = projected_v.normalize() * max_speed;
        } else {
            projected_v = Vector2D::new(0.0, 0.0);
        }
    }
    
    projected_v
}


/// Solves 3D linear program for infeasible case using OSQP
/// Variables: x = [vx, vy, d] where d is the minimum penetration distance
/// Objective: minimize d (minimize constraint violations)
fn solve_3d_linear_program(orca_lines: &[OrcaLine], max_speed: f64) -> Vector2D {
    println!("Debug: Entering 3D LP solver with {} ORCA lines", orca_lines.len());
    
    // Variables: [vx, vy, d] where d is the maximum constraint violation
    // Objective: minimize d
    let q = &[0.0, 0.0, 1.0]; // minimize d (3rd variable)
    
    // P matrix is zero for linear program (no quadratic terms)
    let p_matrix = CscMatrix {
        nrows: 3,
        ncols: 3,
        indptr: std::borrow::Cow::Borrowed(&[0, 0, 0, 0]), // All columns empty
        indices: std::borrow::Cow::Borrowed(&[]),
        data: std::borrow::Cow::Borrowed(&[]),
    };
    
    // Build constraint matrix A for: Ax <= u, l <= Ax
    let mut constraints_a: Vec<[f64; 3]> = Vec::new();
    let mut l_bounds = Vec::new();
    let mut u_bounds = Vec::new();
    
    // Add ORCA constraints: vx*dx + vy*dy - d >= px*dx + py*dy
    // Rearranged: -vx*dx - vy*dy + d <= -px*dx - py*dy
    for line in orca_lines {
        constraints_a.push([-line.direction.x, -line.direction.y, 1.0]);
        let rhs = -line.point.dot(&line.direction);
        l_bounds.push(f64::NEG_INFINITY);
        u_bounds.push(rhs);
    }
    
    // Add speed constraints using circular approximation (16-sided polygon)
    let num_sides = 16;
    for i in 0..num_sides {
        let angle = (i as f64 / num_sides as f64) * 2.0 * std::f64::consts::PI;
        let dir = Vector2D::new(angle.cos(), angle.sin());
        constraints_a.push([dir.x, dir.y, 0.0]); // d doesn't affect speed constraint
        l_bounds.push(f64::NEG_INFINITY);
        u_bounds.push(max_speed);
    }
    
    // Bound d >= 0 (non-negative violation)
    constraints_a.push([0.0, 0.0, 1.0]);
    l_bounds.push(0.0);
    u_bounds.push(f64::INFINITY);
    
    // Build A matrix in CSC format
    let mut a_data = Vec::new();
    let mut a_indices = Vec::new();
    let mut a_indptr = vec![0];
    
    // Column 0 (vx coefficients)
    for (row_idx, row_data) in constraints_a.iter().enumerate() {
        if row_data[0] != 0.0 {
            a_data.push(row_data[0]);
            a_indices.push(row_idx);
        }
    }
    a_indptr.push(a_data.len());
    
    // Column 1 (vy coefficients)
    for (row_idx, row_data) in constraints_a.iter().enumerate() {
        if row_data[1] != 0.0 {
            a_data.push(row_data[1]);
            a_indices.push(row_idx);
        }
    }
    a_indptr.push(a_data.len());
    
    // Column 2 (d coefficients)
    for (row_idx, row_data) in constraints_a.iter().enumerate() {
        if row_data[2] != 0.0 {
            a_data.push(row_data[2]);
            a_indices.push(row_idx);
        }
    }
    a_indptr.push(a_data.len());
    
    let a_matrix = CscMatrix {
        nrows: constraints_a.len(),
        ncols: 3,
        indptr: std::borrow::Cow::Owned(a_indptr),
        indices: std::borrow::Cow::Owned(a_indices),
        data: std::borrow::Cow::Owned(a_data),
    };
    
    // Solve with OSQP
    let settings = Settings::default().verbose(false);
    
    let mut problem = match Problem::new(
        p_matrix,
        q,
        a_matrix,
        &l_bounds,
        &u_bounds,
        &settings,
    ) {
        Ok(p) => p,
        Err(_e) => {
            println!("Debug: 3D LP setup failed, using fallback");
            return solve_3d_linear_program_fallback(orca_lines, max_speed);
        }
    };
    
    let result = problem.solve();
    match result {
        osqp::Status::Solved(solution) => {
            let sol = solution.x();
            let velocity = Vector2D::new(sol[0], sol[1]);
            let violation = sol[2];
            println!("Debug: 3D LP solved successfully: v=({:.6}, {:.6}), d={:.6}", 
                     velocity.x, velocity.y, violation);
            velocity
        },
        _ => {
            println!("Debug: 3D LP failed, using fallback");
            solve_3d_linear_program_fallback(orca_lines, max_speed)
        }
    }
}

/// Fallback constraint satisfaction approach when OSQP fails
/// This implements a simple constraint satisfaction that prioritizes finding
/// any feasible velocity, with preference for minimal violations
fn solve_3d_linear_program_fallback(orca_lines: &[OrcaLine], max_speed: f64) -> Vector2D {
    // Start with zero velocity and try to find something that satisfies constraints
    let mut best_velocity = Vector2D::new(0.0, 0.0);
    let mut min_violation = f64::INFINITY;
    
    // Try various directions, including small perturbations for symmetry breaking
    let num_samples = 32;
    for i in 0..num_samples {
        let angle = (i as f64 / num_samples as f64) * 2.0 * std::f64::consts::PI;
        
        // Try different speeds
        for speed in [0.1, 0.5, max_speed] {
            let candidate = Vector2D::new(
                speed * angle.cos(),
                speed * angle.sin(),
            );
            
            // Compute violation
            let mut max_violation = 0.0;
            for line in orca_lines {
                let relative_velocity = candidate - line.point;
                let violation = -relative_velocity.dot(&line.direction);
                if violation > max_violation {
                    max_violation = violation;
                }
            }
            
            if max_violation < min_violation {
                min_violation = max_violation;
                best_velocity = candidate;
            }
        }
    }
    
    // If we haven't found a good solution, try small perturbations around zero
    if min_violation > 0.1 {
        for i in 0..8 {
            let angle = (i as f64 / 8.0) * 2.0 * std::f64::consts::PI;
            let candidate = Vector2D::new(
                PERTURBATION_EPSILON * angle.cos(),
                PERTURBATION_EPSILON * angle.sin(),
            );
            
            let mut max_violation = 0.0;
            for line in orca_lines {
                let relative_velocity = candidate - line.point;
                let violation = -relative_velocity.dot(&line.direction);
                if violation > max_violation {
                    max_violation = violation;
                }
            }
            
            if max_violation < min_violation {
                min_violation = max_violation;
                best_velocity = candidate;
            }
        }
    }
    
    best_velocity
}

/// Computes maximum constraint violation for a given velocity
fn compute_max_violation(velocity: &Vector2D, orca_lines: &[OrcaLine]) -> f64 {
    let mut max_violation = 0.0;
    
    for line in orca_lines {
        let relative_velocity = *velocity - line.point;
        let violation = -relative_velocity.dot(&line.direction);
        if violation > max_violation {
            max_violation = violation;
        }
    }
    
    max_violation
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::structs::Point;
    
    #[test]
    fn test_head_on_collision() {
        let agent1 = AgentState::new(
            0,
            Point::new(0.0, 0.0),
            Vector2D::new(1.0, 0.0),
            0.5,
            Vector2D::new(1.0, 0.0),
            1.0,
        );
        
        let agent2 = AgentState::new(
            1,
            Point::new(3.0, 0.0),
            Vector2D::new(-1.0, 0.0),
            0.5,
            Vector2D::new(-1.0, 0.0),
            1.0,
        );
        
        let neighbors = vec![&agent2];
        let result = compute_new_velocity(&agent1, &neighbors, 2.0);
        
        
        // Should have lateral movement
        assert!(result.y.abs() > 0.001, "Expected lateral movement, got: {:?}", result);
    }
    
    #[test]
    fn test_no_neighbors() {
        let agent = AgentState::new(
            0,
            Point::new(0.0, 0.0),
            Vector2D::new(0.0, 0.0),
            0.5,
            Vector2D::new(1.0, 0.0),
            1.0,
        );
        
        let neighbors = vec![];
        let result = compute_new_velocity(&agent, &neighbors, 2.0);
        
        // Should return preferred velocity
        assert!((result - agent.pref_velocity).magnitude() < EPSILON);
    }
    
    #[test]
    fn test_infeasible_scenario_fallback() {
        // Setup: Place Agent A at (0, 0). Place three non-reciprocal agents B, C, D 
        // in a tight, equilateral triangle around A (e.g., at a distance of 2 * A.radius). 
        // Set the preferred velocity of all three outer agents to (0, 0). 
        // Set the preferred velocity of Agent A to (1, 0). 
        // The time horizon tau should be small enough that the VOs heavily overlap, 
        // creating an empty feasible region for A.
        
        let agent_a = AgentState::new(
            0,
            Point::new(0.0, 0.0),
            Vector2D::new(0.0, 0.0),
            0.5,
            Vector2D::new(1.0, 0.0), // Wants to move right
            1.0,
        );
        
        // Place three agents in a tight triangle around A
        let distance = 2.0 * agent_a.radius; // Very close
        let agent_b = AgentState::new(
            1,
            Point::new(distance, 0.0),
            Vector2D::new(0.0, 0.0), // Stationary
            0.5,
            Vector2D::new(0.0, 0.0),
            1.0,
        );
        
        let agent_c = AgentState::new(
            2,
            Point::new(-distance * 0.5, distance * 0.866), // 60 degrees
            Vector2D::new(0.0, 0.0), // Stationary
            0.5,
            Vector2D::new(0.0, 0.0),
            1.0,
        );
        
        let agent_d = AgentState::new(
            3,
            Point::new(-distance * 0.5, -distance * 0.866), // -60 degrees
            Vector2D::new(0.0, 0.0), // Stationary
            0.5,
            Vector2D::new(0.0, 0.0),
            1.0,
        );
        
        let neighbors = vec![&agent_b, &agent_c, &agent_d];
        let time_horizon = 0.5; // Small time horizon creates heavily overlapping VOs
        
        // Action: Call compute_new_velocity for Agent A
        let result = compute_new_velocity(&agent_a, &neighbors, time_horizon);
        
        // Assertion: Verify that the returned new velocity for A is close to (0, 0) with a small tolerance. 
        // The safest action in this inescapable scenario is to stop.
        assert!(result.magnitude() < 0.1, "Expected near-zero velocity in infeasible scenario, got: {:?}", result);
    }
    
    #[test]
    fn test_symmetry_deadlock_resolution() {
        // Setup: The classic head-on scenario. Agent A at (0, 0) with pref_velocity=(1, 0). 
        // Agent B at (4, 0) with pref_velocity=(-1, 0). Both have radius=0.5.
        
        let agent_a = AgentState::new(
            0,
            Point::new(0.0, 0.0),
            Vector2D::new(1.0, 0.0), // Moving right
            0.5,
            Vector2D::new(1.0, 0.0), // Wants to keep moving right
            1.0,
        );
        
        let agent_b = AgentState::new(
            1,
            Point::new(4.0, 0.0),
            Vector2D::new(-1.0, 0.0), // Moving left
            0.5,
            Vector2D::new(-1.0, 0.0), // Wants to keep moving left
            1.0,
        );
        
        // Action: Call compute_new_velocity for both A and B
        let neighbors_a = vec![&agent_b];
        let neighbors_b = vec![&agent_a];
        let time_horizon = 2.0;
        
        let result_a = compute_new_velocity(&agent_a, &neighbors_a, time_horizon);
        let result_b = compute_new_velocity(&agent_b, &neighbors_b, time_horizon);
        
        // Assertion:
        // Assert that A.new_velocity.y is non-zero (e.g., > 0).
        assert!(result_a.y.abs() > 0.001, "Agent A should have lateral movement, got: {:?}", result_a);
        
        // Assert that B.new_velocity.y is non-zero and has the opposite sign of A.new_velocity.y.
        assert!(result_b.y.abs() > 0.001, "Agent B should have lateral movement, got: {:?}", result_b);
        assert!(result_a.y * result_b.y < 0.0, "Agents should move in opposite lateral directions. A: {:?}, B: {:?}", result_a, result_b);
        
        // Simulate one step and confirm their new positions are no longer on the x-axis
        let dt = 0.1;
        let new_pos_a = agent_a.position + result_a * dt;
        let new_pos_b = agent_b.position + result_b * dt;
        
        assert!(new_pos_a.y.abs() > 0.0001, "Agent A should move off x-axis, new pos: {:?}", new_pos_a);
        assert!(new_pos_b.y.abs() > 0.0001, "Agent B should move off x-axis, new pos: {:?}", new_pos_b);
    }
    
    #[test]
    fn test_circle_dance_stability() {
        // Setup: Place 10 agents evenly on the circumference of a circle with a large radius. 
        // Set each agent's pref_velocity to point directly toward the position of the agent 
        // on the opposite side of the circle. Set a realistic max_speed.
        
        let num_agents = 10;
        let circle_radius = 5.0;
        let agent_radius = 0.3;
        let max_speed = 1.0;
        
        let mut agents = Vec::new();
        
        // Create agents on circle circumference
        for i in 0..num_agents {
            let angle = (i as f64 / num_agents as f64) * 2.0 * std::f64::consts::PI;
            let pos = Point::new(
                circle_radius * angle.cos(),
                circle_radius * angle.sin(),
            );
            
            // Preferred velocity points to opposite side of circle
            let opposite_angle = angle + std::f64::consts::PI;
            let opposite_pos = Point::new(
                circle_radius * opposite_angle.cos(),
                circle_radius * opposite_angle.sin(),
            );
            
            let pref_vel = (opposite_pos - pos).normalize() * max_speed;
            
            let agent = AgentState::new(
                i,
                pos,
                Vector2D::new(0.0, 0.0), // Start stationary
                agent_radius,
                pref_vel,
                max_speed,
            );
            
            agents.push(agent);
        }
        
        // Action: Run the simulation for 200-300 steps
        let num_steps = 250;
        let dt = 0.05;
        let time_horizon = 1.0;
        let mut agent_positions = agents.clone();
        let mut speed_history = Vec::new();
        
        for step in 0..num_steps {
            let mut new_velocities = Vec::new();
            
            // Compute new velocities for all agents
            for (i, agent) in agent_positions.iter().enumerate() {
                let neighbors: Vec<&AgentState> = agent_positions.iter()
                    .enumerate()
                    .filter(|(j, _)| *j != i)
                    .map(|(_, neighbor)| neighbor)
                    .collect();
                
                let new_vel = compute_new_velocity(agent, &neighbors, time_horizon);
                new_velocities.push(new_vel);
            }
            
            // Update positions and velocities
            for (i, agent) in agent_positions.iter_mut().enumerate() {
                agent.velocity = new_velocities[i];
                agent.position = agent.position + agent.velocity * dt;
            }
            
            // Record speeds for the last 50 steps
            if step >= num_steps - 50 {
                let total_speed: f64 = agent_positions.iter()
                    .map(|agent| agent.velocity.magnitude())
                    .sum();
                let avg_speed = total_speed / num_agents as f64;
                speed_history.push(avg_speed);
            }
            
            // Assertion: At every step, assert that no two agents have collided
            for i in 0..agent_positions.len() {
                for j in (i + 1)..agent_positions.len() {
                    let distance = agent_positions[i].position.distance(&agent_positions[j].position);
                    let min_distance = agent_positions[i].radius + agent_positions[j].radius;
                    assert!(distance >= min_distance - 0.01, 
                        "Collision detected at step {}: agents {} and {} are {} apart (min: {})", 
                        step, i, j, distance, min_distance);
                }
            }
        }
        
        // Assertion: At the end of the simulation, calculate the average speed of all agents 
        // over the last 50 steps. Assert that this average speed is greater than a reasonable 
        // fraction of max_speed (e.g., > 0.25 * max_speed), proving the system did not grind to a halt (deadlock).
        let final_avg_speed: f64 = speed_history.iter().sum::<f64>() / speed_history.len() as f64;
        let min_expected_speed = 0.25 * max_speed;
        
        assert!(final_avg_speed > min_expected_speed, 
            "System appears to have deadlocked. Average speed over last 50 steps: {:.3}, expected > {:.3}", 
            final_avg_speed, min_expected_speed);
    }
}