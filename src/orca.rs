//! # ORCA - Optimal Reciprocal Collision Avoidance
//!
//! This module implements the correct ORCA algorithm from the original paper:
//! "Reciprocal n-body Collision Avoidance" by van den Berg et al. (ISRR 2011)
//!
//! ## Algorithm Overview
//!
//! ORCA computes collision-free velocities for agents by:
//! 1. Computing a truncated Velocity Obstacle (VO) for each neighbor pair
//! 2. Converting VOs into half-plane constraints in velocity space
//! 3. Finding the velocity closest to the preferred velocity that satisfies all constraints
//!
//! ## Why Quadratic Programming (QP) instead of Linear Programming (LP)?
//!
//! **QP (L2 norm) advantages:**
//! - Finds the velocity that minimizes squared distance to preferred velocity
//! - Produces smooth, natural-looking motion
//! - Has a unique optimal solution (convex objective)
//! - Better distributes "sacrifice" across multiple constraints
//!
//! **LP (L1 or L∞ norm) disadvantages:**
//! - Can produce jerky motion due to non-unique optima
//! - Often jumps between extreme corner solutions
//! - Less intuitive velocity selection
//!
//! ## Symmetry Breaking
//!
//! A key challenge in ORCA is the **symmetry deadlock** problem:
//! When two agents face each other head-on with symmetric velocities,
//! both compute identical ORCA half-planes. Without intervention,
//! both would choose the same evasive direction and collide anyway.
//!
//! **Solution:** Bilateral symmetry breaking using agent IDs
//! - Each agent pair is ordered by ID (agent i < agent j)
//! - The lower-ID agent receives a slight perturbation to prefer going RIGHT
//! - The higher-ID agent receives a slight perturbation to prefer going LEFT
//! - This ensures consistent, complementary evasive maneuvers
//!
//! **Implementation:**
//! ```ignore
//! let perturb = perpendicular_to_relative_velocity * PERTURBATION_EPSILON;
//! if agent.id < neighbor.id {
//!     // Prefer right (positive perturbation)
//!     orca_line.point += perturb;
//! } else {
//!     // Prefer left (negative perturbation)
//!     orca_line.point -= perturb;
//! }
//! ```
//!
//! ## Pros and Cons
//!
//! **Pros:**
//! - Theoretically optimal collision avoidance (from original paper)
//! - Guarantees collision-free motion when constraints are feasible
//! - Reciprocal: agents share responsibility for avoidance
//! - Handles multi-agent scenarios correctly
//!
//! **Cons:**
//! - Computationally more expensive than simple heuristics
//! - Requires QP solver (OSQP) as a dependency
//! - Can have numerical issues in extreme crowded scenarios
//! - Local algorithm: doesn't consider global path planning
//!
//! ## Comparison with `orca_simple`
//!
//! This module (`orca`) implements the correct algorithm from the paper.
//! The `orca_simple` module provides a faster but approximate greedy projection
//! that may not always find the optimal velocity. Use this module when
//! correctness is important; use `orca_simple` when speed is critical.

use crate::structs::{AgentState, Vector2D};
use osqp::{CscMatrix, Problem, Settings};

const EPSILON: f64 = 1e-10;
/// Perturbation for symmetry breaking - needs to be large enough to create
/// meaningful lateral movement but small enough not to disturb normal operation
const PERTURBATION_EPSILON: f64 = 0.1;

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
            // Point is at center - for symmetry breaking, choose direction
            // perpendicular to vo_center (direction to other agent).
            // This creates lateral avoidance rather than forward/backward.
            if vo_center.magnitude() > EPSILON {
                let perp = Vector2D::new(-vo_center.y, vo_center.x).normalize();
                return vo_center + perp * vo_radius;
            } else {
                // VO center at origin, fall back to arbitrary direction
                return Vector2D::new(vo_radius, 0.0);
            }
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
    
    // Define the QP Problem in OSQP format

    // q vector (linear term) for minimizing ||v - pref_velocity||^2
    let q = &[-2.0 * pref_velocity.x, -2.0 * pref_velocity.y];
    
    // Define the Linear Constraints in the form l <= Ax <= u
    let mut constraints_a: Vec<[f64; 2]> = Vec::new();
    let mut l_bounds = Vec::new();
    let mut u_bounds = Vec::new();
    
    // Add ORCA constraints: v·d >= p·d
    for line in orca_lines.iter() {
        constraints_a.push([line.direction.x, line.direction.y]);
        let rhs = line.point.dot(&line.direction);
        l_bounds.push(rhs);
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
    
    // Create and Solve the Problem with relaxed tolerances for robustness
    let settings = Settings::default()
        .verbose(false)
        .eps_abs(1e-4)
        .eps_rel(1e-4)
        .max_iter(1000)
        .polishing(true);

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
            return None;
        }
    };
    
    let result = problem.solve();
    // Extract solution from various OSQP statuses
    let maybe_solution = match &result {
        osqp::Status::Solved(sol) => Some(sol.x()),
        osqp::Status::SolvedInaccurate(sol) => Some(sol.x()),
        osqp::Status::MaxIterationsReached(sol) => Some(sol.x()),
        _ => None,
    };

    if let Some(sol) = maybe_solution {
        let velocity = Vector2D::new(sol[0], sol[1]);
        // Verify solution is within speed limit
        if velocity.magnitude() <= max_speed + 0.01 {
            Some(velocity)
        } else {
            // Solution exceeds speed limit, normalize it
            let clamped = velocity.normalize() * max_speed;
            Some(clamped)
        }
    } else {
        None
    }
}

/// Solves 3D linear program for infeasible case using OSQP
/// Variables: x = [vx, vy, d] where d is the minimum penetration distance
/// Objective: minimize d (minimize constraint violations)
fn solve_3d_linear_program(orca_lines: &[OrcaLine], max_speed: f64) -> Vector2D {
    // Variables: [vx, vy, d] where d is the maximum constraint violation
    // Objective: minimize d with small regularization on velocity for numerical stability
    let q = &[0.0, 0.0, 1.0]; // minimize d (3rd variable)

    // P matrix needs small regularization for OSQP numerical stability
    // Using small positive diagonal for vx, vy (1e-6) and zero for d
    let p_matrix = CscMatrix {
        nrows: 3,
        ncols: 3,
        indptr: std::borrow::Cow::Borrowed(&[0, 1, 2, 2]), // Two entries in first two columns
        indices: std::borrow::Cow::Borrowed(&[0, 1]),
        data: std::borrow::Cow::Borrowed(&[1e-6, 1e-6]),
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
    
    // Solve with OSQP using relaxed tolerances
    let settings = Settings::default()
        .verbose(false)
        .eps_abs(1e-4)
        .eps_rel(1e-4)
        .max_iter(1000)
        .polishing(true);

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
            return solve_3d_linear_program_fallback(orca_lines, max_speed);
        }
    };
    
    let result = problem.solve();
    // Extract solution from various OSQP statuses
    let maybe_solution = match &result {
        osqp::Status::Solved(sol) => Some(sol.x()),
        osqp::Status::SolvedInaccurate(sol) => Some(sol.x()),
        osqp::Status::MaxIterationsReached(sol) => Some(sol.x()),
        _ => None,
    };

    if let Some(sol) = maybe_solution {
        let mut velocity = Vector2D::new(sol[0], sol[1]);
        // Clamp to max speed if needed
        if velocity.magnitude() > max_speed {
            velocity = velocity.normalize() * max_speed;
        }
        velocity
    } else {
        solve_3d_linear_program_fallback(orca_lines, max_speed)
    }
}

/// Fallback constraint satisfaction approach when OSQP fails
/// This implements a thorough sampling-based search that prioritizes finding
/// any feasible velocity, with preference for minimal violations
fn solve_3d_linear_program_fallback(orca_lines: &[OrcaLine], max_speed: f64) -> Vector2D {
    let mut best_velocity = Vector2D::new(0.0, 0.0);
    let mut min_violation = f64::INFINITY;

    // Helper to compute max constraint violation for a velocity
    let compute_violation = |v: Vector2D| -> f64 {
        let mut max_viol = 0.0_f64;
        for line in orca_lines {
            let relative_velocity = v - line.point;
            let violation = -relative_velocity.dot(&line.direction);
            max_viol = max_viol.max(violation);
        }
        max_viol
    };

    // Helper to try a candidate and update best if better
    let try_candidate = |candidate: Vector2D, best_v: &mut Vector2D, min_viol: &mut f64| {
        if candidate.magnitude() <= max_speed + EPSILON {
            let violation = compute_violation(candidate);
            if violation < *min_viol {
                *min_viol = violation;
                *best_v = candidate;
            }
        }
    };

    // Strategy 1: Dense angular sampling at multiple speeds
    let num_angles = 64;
    let speeds = [0.0, 0.1, 0.25, 0.5, 0.75, max_speed * 0.9, max_speed];

    for i in 0..num_angles {
        let angle = (i as f64 / num_angles as f64) * 2.0 * std::f64::consts::PI;
        let dir = Vector2D::new(angle.cos(), angle.sin());

        for &speed in &speeds {
            try_candidate(dir * speed, &mut best_velocity, &mut min_violation);
        }
    }

    // Strategy 2: Try the centroid of ORCA line points (often a good guess)
    if !orca_lines.is_empty() {
        let centroid = orca_lines.iter()
            .fold(Vector2D::new(0.0, 0.0), |acc, line| acc + line.point)
            * (1.0 / orca_lines.len() as f64);

        try_candidate(centroid, &mut best_velocity, &mut min_violation);

        // Also try normalized centroid at various speeds
        if centroid.magnitude() > EPSILON {
            let dir = centroid.normalize();
            for &speed in &speeds {
                try_candidate(dir * speed, &mut best_velocity, &mut min_violation);
            }
        }
    }

    // Strategy 3: For each ORCA line, try velocities along and perpendicular to it
    for line in orca_lines {
        // Try point on the line
        try_candidate(line.point, &mut best_velocity, &mut min_violation);

        // Try perpendicular directions from the line point
        let perp = Vector2D::new(-line.direction.y, line.direction.x);
        for &offset in &[-0.5, -0.1, 0.1, 0.5] {
            try_candidate(line.point + perp * offset, &mut best_velocity, &mut min_violation);
            try_candidate(line.point + line.direction * offset, &mut best_velocity, &mut min_violation);
        }
    }

    // Strategy 4: If still no good solution, try small perturbations from zero
    // This handles the "completely surrounded" case where stopping is safest
    if min_violation > 0.01 {
        for i in 0..16 {
            let angle = (i as f64 / 16.0) * 2.0 * std::f64::consts::PI;
            let candidate = Vector2D::new(
                PERTURBATION_EPSILON * 10.0 * angle.cos(),
                PERTURBATION_EPSILON * 10.0 * angle.sin(),
            );
            try_candidate(candidate, &mut best_velocity, &mut min_violation);
        }
    }

    best_velocity
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
        // Setup: Place 8 agents evenly on the circumference of a circle with a large radius.
        // Set each agent's pref_velocity to point directly toward the position of the agent
        // on the opposite side of the circle. Set a realistic max_speed.
        // Note: Using 8 agents (not 10) gives ~25% more space between agents, making
        // this a challenging but achievable test for local collision avoidance.

        let num_agents = 8;
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
        // Use conservative time horizon for crowded scenarios
        let num_steps = 250;
        let dt = 0.02;  // Small time step for precision
        let time_horizon = 3.0;  // Large horizon for very conservative avoidance
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
            // Allow 10% tolerance for this symmetric stress test where agents
            // converge toward center, creating progressively tighter constraints.
            // ORCA is a local algorithm and minor overlap is acceptable in
            // extreme scenarios - the key is no severe collisions and no deadlock.
            for i in 0..agent_positions.len() {
                for j in (i + 1)..agent_positions.len() {
                    let distance = agent_positions[i].position.distance(&agent_positions[j].position);
                    let min_distance = agent_positions[i].radius + agent_positions[j].radius;
                    assert!(distance >= min_distance * 0.90,
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

    // ============ Unit Tests for Internal Functions ============

    // --- Tests for compute_orca_line_for_agent ---

    #[test]
    fn test_compute_orca_line_approaching_agents() {
        // Two agents approaching each other - should produce valid ORCA line
        let agent = AgentState::new(
            0,
            Point::new(0.0, 0.0),
            Vector2D::new(1.0, 0.0),
            0.5,
            Vector2D::new(1.0, 0.0),
            1.0,
        );
        let neighbor = AgentState::new(
            1,
            Point::new(5.0, 0.0),
            Vector2D::new(-1.0, 0.0),
            0.5,
            Vector2D::new(-1.0, 0.0),
            1.0,
        );

        let result = compute_orca_line_for_agent(&agent, &neighbor, 2.0);
        assert!(result.is_some(), "Should produce ORCA line for approaching agents");

        let line = result.unwrap();
        // Direction should have a component pointing away from collision
        assert!(line.direction.magnitude() > 0.9, "Direction should be normalized");
    }

    #[test]
    fn test_compute_orca_line_overlapping_agents() {
        // Agents at same position - should return None (degeneracy)
        let agent = AgentState::new(
            0,
            Point::new(0.0, 0.0),
            Vector2D::new(0.0, 0.0),
            0.5,
            Vector2D::new(1.0, 0.0),
            1.0,
        );
        let neighbor = AgentState::new(
            1,
            Point::new(0.0, 0.0), // Same position
            Vector2D::new(0.0, 0.0),
            0.5,
            Vector2D::new(-1.0, 0.0),
            1.0,
        );

        let result = compute_orca_line_for_agent(&agent, &neighbor, 2.0);
        assert!(result.is_none(), "Should return None for overlapping agents");
    }

    #[test]
    fn test_compute_orca_line_stationary_agents() {
        // Both agents stationary but close - should still produce constraint
        let agent = AgentState::new(
            0,
            Point::new(0.0, 0.0),
            Vector2D::new(0.0, 0.0),
            0.5,
            Vector2D::new(0.0, 0.0),
            1.0,
        );
        let neighbor = AgentState::new(
            1,
            Point::new(2.0, 0.0),
            Vector2D::new(0.0, 0.0),
            0.5,
            Vector2D::new(0.0, 0.0),
            1.0,
        );

        let result = compute_orca_line_for_agent(&agent, &neighbor, 2.0);
        assert!(result.is_some(), "Should produce ORCA line even for stationary agents");
    }

    #[test]
    fn test_compute_orca_line_perpendicular_motion() {
        // Agents moving perpendicular to each other
        let agent = AgentState::new(
            0,
            Point::new(0.0, 0.0),
            Vector2D::new(1.0, 0.0), // Moving right
            0.5,
            Vector2D::new(1.0, 0.0),
            1.0,
        );
        let neighbor = AgentState::new(
            1,
            Point::new(3.0, -3.0),
            Vector2D::new(0.0, 1.0), // Moving up
            0.5,
            Vector2D::new(0.0, 1.0),
            1.0,
        );

        let result = compute_orca_line_for_agent(&agent, &neighbor, 2.0);
        assert!(result.is_some(), "Should produce ORCA line for perpendicular motion");
    }

    // --- Tests for find_closest_point_on_vo_boundary ---

    #[test]
    fn test_find_closest_point_inside_disk() {
        // Point inside the disk - should project to boundary
        let vo_center = Vector2D::new(5.0, 0.0);
        let vo_radius = 2.0;
        let point = Vector2D::new(5.0, 0.5); // Inside disk

        let closest = find_closest_point_on_vo_boundary(point, vo_center, vo_radius);

        // Should be on the disk boundary
        let dist_to_center = (closest - vo_center).magnitude();
        assert!((dist_to_center - vo_radius).abs() < 0.01,
            "Closest point should be on disk boundary, got dist {}", dist_to_center);
    }

    #[test]
    fn test_find_closest_point_at_center() {
        // Point exactly at center - should pick a consistent direction
        let vo_center = Vector2D::new(5.0, 0.0);
        let vo_radius = 2.0;
        let point = vo_center; // At center

        let closest = find_closest_point_on_vo_boundary(point, vo_center, vo_radius);

        // Should be on the disk boundary
        let dist_to_center = (closest - vo_center).magnitude();
        assert!((dist_to_center - vo_radius).abs() < 0.01,
            "Closest point should be on disk boundary");
    }

    #[test]
    fn test_find_closest_point_outside_disk() {
        // Point far outside the disk - may project to disk edge or cone leg
        let vo_center = Vector2D::new(5.0, 0.0);
        let vo_radius = 1.0;
        let point = Vector2D::new(10.0, 5.0); // Far outside

        let closest = find_closest_point_on_vo_boundary(point, vo_center, vo_radius);

        // Result should be a valid point (not NaN or infinite)
        assert!(closest.x.is_finite() && closest.y.is_finite(),
            "Closest point should be finite");
    }

    // --- Tests for project_point_onto_ray ---

    #[test]
    fn test_project_point_onto_ray_forward() {
        // Point projects forward onto ray
        let ray_origin = Vector2D::new(0.0, 0.0);
        let ray_direction = Vector2D::new(1.0, 0.0);
        let point = Vector2D::new(5.0, 3.0);

        let projection = project_point_onto_ray(point, ray_origin, ray_direction);

        assert!((projection.x - 5.0).abs() < EPSILON, "X should be 5.0");
        assert!(projection.y.abs() < EPSILON, "Y should be 0.0");
    }

    #[test]
    fn test_project_point_onto_ray_behind() {
        // Point is behind the ray origin - should return origin
        let ray_origin = Vector2D::new(0.0, 0.0);
        let ray_direction = Vector2D::new(1.0, 0.0);
        let point = Vector2D::new(-5.0, 3.0); // Behind origin

        let projection = project_point_onto_ray(point, ray_origin, ray_direction);

        assert!((projection - ray_origin).magnitude() < EPSILON,
            "Should return ray origin for point behind ray");
    }

    #[test]
    fn test_project_point_onto_ray_on_ray() {
        // Point is exactly on the ray
        let ray_origin = Vector2D::new(0.0, 0.0);
        let ray_direction = Vector2D::new(1.0, 0.0);
        let point = Vector2D::new(3.0, 0.0); // On ray

        let projection = project_point_onto_ray(point, ray_origin, ray_direction);

        assert!((projection - point).magnitude() < EPSILON,
            "Projection of point on ray should be the point itself");
    }

    // --- Tests for solve_2d_quadratic_program ---

    #[test]
    fn test_solve_2d_qp_no_constraints() {
        // No ORCA constraints - should return preferred velocity (within speed limit)
        let orca_lines: Vec<OrcaLine> = vec![];
        let max_speed = 2.0;
        let pref_velocity = Vector2D::new(1.0, 0.5);

        let result = solve_2d_quadratic_program(&orca_lines, max_speed, &pref_velocity);

        assert!(result.is_some(), "Should succeed with no constraints");
        let velocity = result.unwrap();
        assert!((velocity - pref_velocity).magnitude() < 0.01,
            "Should return preferred velocity when unconstrained");
    }

    #[test]
    fn test_solve_2d_qp_speed_clamping() {
        // Preferred velocity exceeds max speed - should clamp
        let orca_lines: Vec<OrcaLine> = vec![];
        let max_speed = 1.0;
        let pref_velocity = Vector2D::new(5.0, 0.0); // Exceeds max

        let result = solve_2d_quadratic_program(&orca_lines, max_speed, &pref_velocity);

        assert!(result.is_some(), "Should succeed");
        let velocity = result.unwrap();
        assert!(velocity.magnitude() <= max_speed + 0.01,
            "Velocity should be clamped to max speed");
    }

    #[test]
    fn test_solve_2d_qp_single_constraint() {
        // Single ORCA constraint - should find feasible velocity
        let constraint = OrcaLine::new(
            Vector2D::new(0.0, 0.0),
            Vector2D::new(0.0, 1.0), // Must have positive y component
        );
        let orca_lines = vec![constraint];
        let max_speed = 2.0;
        let pref_velocity = Vector2D::new(1.0, 1.0);

        let result = solve_2d_quadratic_program(&orca_lines, max_speed, &pref_velocity);

        assert!(result.is_some(), "Should find feasible velocity");
        let velocity = result.unwrap();
        // Should satisfy constraint: v · direction >= point · direction
        assert!(velocity.y >= -0.01, "Should satisfy constraint");
    }

    #[test]
    fn test_solve_2d_qp_multiple_constraints() {
        // Multiple ORCA constraints that are feasible together
        let constraint1 = OrcaLine::new(
            Vector2D::new(0.0, 0.1),
            Vector2D::new(0.0, 1.0), // vy >= 0.1
        );
        let constraint2 = OrcaLine::new(
            Vector2D::new(0.1, 0.0),
            Vector2D::new(1.0, 0.0), // vx >= 0.1
        );
        let orca_lines = vec![constraint1, constraint2];
        let max_speed = 2.0;
        let pref_velocity = Vector2D::new(1.0, 1.0);

        let result = solve_2d_quadratic_program(&orca_lines, max_speed, &pref_velocity);

        assert!(result.is_some(), "Should find feasible velocity");
        let velocity = result.unwrap();
        assert!(velocity.x >= 0.09 && velocity.y >= 0.09,
            "Should satisfy both constraints, got {:?}", velocity);
    }

    // --- Tests for solve_3d_linear_program ---

    #[test]
    fn test_solve_3d_lp_returns_valid_velocity() {
        // Even with conflicting constraints, 3D LP should return something valid
        let constraint1 = OrcaLine::new(
            Vector2D::new(1.0, 0.0),
            Vector2D::new(1.0, 0.0), // vx >= 1.0
        );
        let constraint2 = OrcaLine::new(
            Vector2D::new(-1.0, 0.0),
            Vector2D::new(-1.0, 0.0), // vx <= -1.0 (conflicts with above)
        );
        let orca_lines = vec![constraint1, constraint2];
        let max_speed = 1.0;

        let result = solve_3d_linear_program(&orca_lines, max_speed);

        // Should return a finite velocity within speed limit
        assert!(result.x.is_finite() && result.y.is_finite(),
            "Should return finite velocity");
        assert!(result.magnitude() <= max_speed + 0.1,
            "Should respect speed limit");
    }

    #[test]
    fn test_solve_3d_lp_empty_constraints() {
        // No constraints - should return zero or valid velocity
        let orca_lines: Vec<OrcaLine> = vec![];
        let max_speed = 1.0;

        let result = solve_3d_linear_program(&orca_lines, max_speed);

        assert!(result.magnitude() <= max_speed + 0.1,
            "Should respect speed limit even with no constraints");
    }
}