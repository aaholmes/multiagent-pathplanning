use crate::structs::{AgentState, Vector2D};
use osqp::{CscMatrix, Problem, Settings};

const EPSILON: f64 = 1e-10;

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
    
    // --- SYMMETRY-BREAKING PERTURBATION ---
    // Handle the deadlock/perfect symmetry case where u becomes zero
    if u.magnitude() < EPSILON {
        // This is the deadlock case - relative velocity lies on VO boundary
        // Introduce a small, deterministic perturbation to break symmetry
        
        let perturb_dir = if agent.pref_velocity.magnitude() > EPSILON {
            // Use direction perpendicular to agent's preferred velocity
            Vector2D::new(-agent.pref_velocity.y, agent.pref_velocity.x).normalize()
        } else if agent.velocity.magnitude() > EPSILON {
            // Fallback: perpendicular to current velocity
            Vector2D::new(-agent.velocity.y, agent.velocity.x).normalize()
        } else {
            // Final fallback: arbitrary perpendicular direction
            Vector2D::new(0.0, 1.0)
        };
        
        // Apply more significant perturbation - this breaks perfect symmetry
        u = perturb_dir * 0.001; // Larger perturbation to ensure constraint generation
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
    // Define the QP Problem in OSQP format
    
    // q vector (linear term) for minimizing ||v - pref_velocity||^2
    let q = &[-2.0 * pref_velocity.x, -2.0 * pref_velocity.y];
    
    // Define the Linear Constraints in the form l <= Ax <= u
    let mut constraints_a: Vec<[f64; 2]> = Vec::new();
    let mut l_bounds = Vec::new();
    let mut u_bounds = Vec::new();
    
    // Add ORCA constraints: v·d >= p·d
    for line in orca_lines {
        constraints_a.push([line.direction.x, line.direction.y]);
        l_bounds.push(line.point.dot(&line.direction));
        u_bounds.push(f64::INFINITY);
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
    
    // Create CSC matrices using dense construction for OSQP v1.0 API
    let p_matrix = CscMatrix::from_column_iter_dense(
        2, // rows
        2, // cols  
        [2.0, 0.0, 0.0, 2.0].iter().copied(),
    );
    
    // Build A matrix from constraints (row-major)
    let mut a_dense = Vec::new();
    for row_data in &constraints_a {
        a_dense.push(row_data[0]);
        a_dense.push(row_data[1]);
    }
    
    let a_matrix = CscMatrix::from_row_iter_dense(
        constraints_a.len(), // rows
        2, // cols
        a_dense.iter().copied(),
    );
    
    // Create and Solve the Problem
    let settings = Settings::default().verbose(false);
    
    let mut problem = match Problem::new(p_matrix, q, a_matrix, &l_bounds, &u_bounds, &settings) {
        Ok(p) => p,
        Err(_) => return None,
    };
    
    let result = problem.solve();
    match result {
        osqp::Status::Solved(solution) => {
            let sol = solution.x();
            Some(Vector2D::new(sol[0], sol[1]))
        },
        _ => None, // Infeasible or other error
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


/// Solves 3D linear program for infeasible case (emergency mode)
/// Uses grid sampling as a fallback when quadratic programming fails
fn solve_3d_linear_program(orca_lines: &[OrcaLine], max_speed: f64) -> Vector2D {
    // Grid sampling approach for infeasible cases
    // Sample velocities on the speed circle and find the one with minimum violation
    
    let num_samples = 32;
    let mut best_velocity = Vector2D::new(0.0, 0.0);
    let mut min_violation = f64::INFINITY;
    
    for i in 0..num_samples {
        let angle = (i as f64 / num_samples as f64) * 2.0 * std::f64::consts::PI;
        let candidate = Vector2D::new(
            max_speed * angle.cos(),
            max_speed * angle.sin(),
        );
        
        let violation = compute_max_violation(&candidate, orca_lines);
        if violation < min_violation {
            min_violation = violation;
            best_velocity = candidate;
        }
    }
    
    // Also try the zero velocity
    let zero_violation = compute_max_violation(&Vector2D::new(0.0, 0.0), orca_lines);
    if zero_violation < min_violation {
        best_velocity = Vector2D::new(0.0, 0.0);
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
        assert!(result.y.abs() > 0.1, "Expected lateral movement, got: {:?}", result);
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
}