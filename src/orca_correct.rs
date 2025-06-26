use crate::structs::{AgentState, Vector2D};
use good_lp::*;

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
    
    // Step 2: Attempt to solve 2D linear program (feasible case)
    if let Some(new_velocity) = solve_2d_linear_program(&orca_lines, agent.max_speed, &agent.pref_velocity) {
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
    let u = closest_point - relative_velocity;
    
    // Check if correction vector is meaningful
    if u.magnitude() < EPSILON {
        // Special case: relative velocity is exactly on VO boundary
        let distance_to_vo_center = (relative_velocity - vo_center).magnitude();
        if (distance_to_vo_center - vo_radius).abs() < EPSILON {
            // We're on the boundary - still need a constraint for grazing collision
            let center_to_rel_vel = relative_velocity - vo_center;
            if center_to_rel_vel.magnitude() < EPSILON {
                // Relative velocity at VO center - use arbitrary direction
                let orca_direction = Vector2D::new(1.0, 0.0);
                let orca_point = agent.velocity; // No correction needed, just constrain direction
                return Some(OrcaLine::new(orca_point, orca_direction));
            } else {
                // On boundary - constraint should push away from VO center
                let orca_direction = center_to_rel_vel.normalize();
                let orca_point = agent.velocity; // No correction needed
                return Some(OrcaLine::new(orca_point, orca_direction));
            }
        } else {
            // Truly safe case
            return None;
        }
    }
    
    // Normal case: meaningful correction vector
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

/// Helper function to handle the non-linear speed constraint
fn create_speed_constraint_polygon(max_speed: f64) -> Vec<(f64, f64, f64)> {
    let mut constraints = Vec::new();
    let num_sides = 16; // Approximate the circle with a 16-sided polygon
    for i in 0..num_sides {
        let angle = (i as f64 / num_sides as f64) * 2.0 * std::f64::consts::PI;
        let dir_x = angle.cos();
        let dir_y = angle.sin();
        // Constraint: v · dir <= max_speed
        // In the form ax + by <= c: dir_x * vx + dir_y * vy <= max_speed
        constraints.push((dir_x, dir_y, max_speed));
    }
    constraints
}

/// Solves 2D linear program to find optimal velocity
fn solve_2d_linear_program(
    orca_lines: &[OrcaLine],
    max_speed: f64,
    pref_velocity: &Vector2D,
) -> Option<Vector2D> {
    use good_lp::*;
    
    // Create variables for velocity components
    variables! {
        vars:
            -10000.0 <= vx <= 10000.0;
            -10000.0 <= vy <= 10000.0;
            0.0 <= dx_pos;
            0.0 <= dx_neg;
            0.0 <= dy_pos;
            0.0 <= dy_neg;
    }
    
    // Build the problem: minimize L1 distance to preferred velocity
    let mut problem = vars
        .minimise(dx_pos + dx_neg + dy_pos + dy_neg)
        .using(default_solver)
        .with(constraint!(vx - dx_pos + dx_neg == pref_velocity.x))
        .with(constraint!(vy - dy_pos + dy_neg == pref_velocity.y));
    
    // Add ORCA Line Constraints: (v - p) · d >= 0  =>  v·d >= p·d
    for line in orca_lines {
        let rhs = line.point.x * line.direction.x + line.point.y * line.direction.y;
        problem = problem.with(constraint!(
            vx * line.direction.x + vy * line.direction.y >= rhs
        ));
    }
    
    // Add Speed Constraint: ||v|| <= max_speed (approximated as polygon)
    let speed_constraints = create_speed_constraint_polygon(max_speed);
    for (dir_x, dir_y, max_val) in speed_constraints {
        problem = problem.with(constraint!(
            vx * dir_x + vy * dir_y <= max_val
        ));
    }
    
    // Solve the problem
    match problem.solve() {
        Ok(solution) => {
            let vx_val = solution.value(vx);
            let vy_val = solution.value(vy);
            Some(Vector2D::new(vx_val, vy_val))
        }
        Err(_) => {
            // The problem is infeasible
            None
        }
    }
}


/// Solves 3D linear program for infeasible case (emergency mode)
fn solve_3d_linear_program(orca_lines: &[OrcaLine], max_speed: f64) -> Vector2D {
    use good_lp::*;
    
    // Create variables: vx, vy, and d (penetration depth to minimize)
    variables! {
        vars:
            -10000.0 <= vx <= 10000.0;
            -10000.0 <= vy <= 10000.0;
            0.0 <= d;
    }
    
    // Objective: Minimize d (the maximum constraint violation)
    let mut problem = vars.minimise(d).using(default_solver);
    
    // Add ORCA Line Constraints: (v - p) · d + d >= 0
    // This relaxes the constraint by allowing violation d
    for line in orca_lines {
        let rhs = line.point.x * line.direction.x + line.point.y * line.direction.y;
        problem = problem.with(constraint!(
            vx * line.direction.x + vy * line.direction.y + d >= rhs
        ));
    }
    
    // Add Speed Constraint: ||v|| <= max_speed (approximated as polygon)
    let speed_constraints = create_speed_constraint_polygon(max_speed);
    for (dir_x, dir_y, max_val) in speed_constraints {
        problem = problem.with(constraint!(
            vx * dir_x + vy * dir_y <= max_val
        ));
    }
    
    // This problem should always be feasible since we allow constraint violations
    match problem.solve() {
        Ok(solution) => {
            let vx_val = solution.value(vx);
            let vy_val = solution.value(vy);
            Vector2D::new(vx_val, vy_val)
        }
        Err(_) => {
            // Fallback to zero velocity if solver fails
            Vector2D::new(0.0, 0.0)
        }
    }
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