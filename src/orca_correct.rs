use crate::structs::{AgentState, Vector2D, Point};

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
    
    // Degeneracy check: already safe
    if u.magnitude() < EPSILON {
        return None;
    }
    
    // Construct ORCA half-plane (each agent takes half responsibility)
    let orca_point = agent.velocity + u * 0.5;
    let orca_direction = u.normalize();
    
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
    
    // Case 1: Point is inside the disk - project to disk boundary
    if distance_to_center < vo_radius {
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

/// Solves 2D linear program to find optimal velocity
fn solve_2d_linear_program(
    orca_lines: &[OrcaLine],
    max_speed: f64,
    pref_velocity: &Vector2D,
) -> Option<Vector2D> {
    // Start with preferred velocity
    let mut optimal_velocity = *pref_velocity;
    
    // Clip to max speed constraint
    if optimal_velocity.magnitude() > max_speed {
        optimal_velocity = optimal_velocity.normalize() * max_speed;
    }
    
    // Apply each ORCA constraint
    for (i, line) in orca_lines.iter().enumerate() {
        if violates_constraint(&optimal_velocity, line) {
            // Project to feasible region
            if let Some(projected) = project_to_orca_line(
                line,
                &optimal_velocity,
                max_speed,
                orca_lines,
                i,
                pref_velocity,
            ) {
                optimal_velocity = projected;
            } else {
                // No feasible solution
                return None;
            }
        }
    }
    
    Some(optimal_velocity)
}

/// Checks if a velocity violates an ORCA constraint
fn violates_constraint(velocity: &Vector2D, line: &OrcaLine) -> bool {
    let relative_velocity = *velocity - line.point;
    relative_velocity.dot(&line.direction) < -EPSILON
}

/// Projects velocity to ORCA line while maintaining feasibility
fn project_to_orca_line(
    line: &OrcaLine,
    velocity: &Vector2D,
    max_speed: f64,
    orca_lines: &[OrcaLine],
    line_index: usize,
    pref_velocity: &Vector2D,
) -> Option<Vector2D> {
    // Project velocity onto the ORCA line
    let relative_velocity = *velocity - line.point;
    let dot_product = relative_velocity.dot(&line.direction);
    let projected = line.point + line.direction * dot_product;
    
    // Check if projection satisfies speed constraint
    if projected.magnitude() <= max_speed + EPSILON {
        // Check if projected velocity satisfies all previous constraints
        let mut feasible = true;
        for (i, other_line) in orca_lines.iter().enumerate() {
            if i < line_index && violates_constraint(&projected, other_line) {
                feasible = false;
                break;
            }
        }
        
        if feasible {
            return Some(projected);
        }
    }
    
    // Find intersection with speed circle if needed
    find_line_circle_intersection(line, max_speed, pref_velocity)
}

/// Finds intersection of ORCA line with speed circle
fn find_line_circle_intersection(
    line: &OrcaLine,
    max_speed: f64,
    pref_velocity: &Vector2D,
) -> Option<Vector2D> {
    // Solve for intersection of line with circle of radius max_speed centered at origin
    let h = line.point;
    let d = line.direction;
    
    // Distance from origin to line
    let distance_to_line = h.dot(&d);
    let closest_point_on_line = d * distance_to_line;
    let perpendicular_distance_sq = h.dot(&h) - distance_to_line * distance_to_line;
    
    let max_speed_sq = max_speed * max_speed;
    
    if perpendicular_distance_sq > max_speed_sq + EPSILON {
        // Line doesn't intersect circle
        return None;
    }
    
    // Compute intersection points
    let chord_half_length_sq = max_speed_sq - perpendicular_distance_sq;
    if chord_half_length_sq < 0.0 {
        return None;
    }
    
    let chord_half_length = chord_half_length_sq.sqrt();
    let intersection1 = closest_point_on_line + d * chord_half_length;
    let intersection2 = closest_point_on_line - d * chord_half_length;
    
    // Choose intersection closer to preferred velocity
    let dist1_sq = (intersection1 - *pref_velocity).dot(&(intersection1 - *pref_velocity));
    let dist2_sq = (intersection2 - *pref_velocity).dot(&(intersection2 - *pref_velocity));
    
    if dist1_sq < dist2_sq {
        Some(intersection1)
    } else {
        Some(intersection2)
    }
}

/// Solves 3D linear program for infeasible case (emergency mode)
fn solve_3d_linear_program(orca_lines: &[OrcaLine], max_speed: f64) -> Vector2D {
    // In infeasible case, find velocity that minimally violates constraints
    // For simplicity, return velocity that minimizes maximum constraint violation
    
    // Start with zero velocity (safest fallback)
    let mut best_velocity = Vector2D::new(0.0, 0.0);
    let mut min_max_violation = f64::INFINITY;
    
    // Sample candidate velocities in a grid pattern
    let num_samples = 32;
    let angle_step = 2.0 * std::f64::consts::PI / num_samples as f64;
    
    for i in 0..num_samples {
        let angle = i as f64 * angle_step;
        let candidate = Vector2D::new(angle.cos(), angle.sin()) * max_speed;
        
        let max_violation = compute_max_violation(&candidate, orca_lines);
        if max_violation < min_max_violation {
            min_max_violation = max_violation;
            best_velocity = candidate;
        }
    }
    
    // Also try zero velocity
    let zero_violation = compute_max_violation(&Vector2D::new(0.0, 0.0), orca_lines);
    if zero_violation < min_max_violation {
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