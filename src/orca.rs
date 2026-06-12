//! # ORCA - Optimal Reciprocal Collision Avoidance
//!
//! This module implements the ORCA algorithm from the original paper:
//! "Reciprocal n-body Collision Avoidance" by van den Berg et al. (ISRR 2011)
//!
//! ## Algorithm Overview
//!
//! ORCA computes collision-free velocities for agents by:
//! 1. Computing a truncated Velocity Obstacle (VO) for each neighbor pair and
//!    each nearby static obstacle
//! 2. Converting VOs into half-plane constraints (ORCA lines)
//! 3. Finding the velocity closest to the preferred velocity that satisfies
//!    all constraints
//!
//! ## Velocity selection
//!
//! Step 3 minimizes the Euclidean (L2) distance to the preferred velocity
//! subject to the half-plane constraints and a speed cap. This is solved
//! exactly with the incremental geometric linear program from the paper (the
//! same approach as the reference RVO2 library): constraints are processed in
//! order, and whenever the current optimum violates a constraint, the optimum
//! is re-projected onto that constraint's boundary subject to all previous
//! constraints. The objective is quadratic, but each step is a closed-form 1D
//! projection, so the solve costs ~1 microsecond with no external solver.
//!
//! If the constraints are jointly infeasible (dense crowds), a second pass
//! ("3D LP" in the paper) finds the velocity minimizing the maximum
//! constraint violation, while keeping static-obstacle constraints hard.
//!
//! ## Static obstacles
//!
//! Grid obstacle cells are approximated as stationary discs. Unlike
//! agent-agent constraints, the agent takes *full* responsibility for
//! avoiding an obstacle (the obstacle will not reciprocate), and obstacle
//! constraints are never relaxed in the infeasible case.
//!
//! ## Symmetry Breaking
//!
//! A key challenge in ORCA is the **symmetry deadlock** problem:
//! When two agents face each other head-on with symmetric velocities,
//! both compute identical ORCA half-planes. Without intervention,
//! both would slow to a crawl, mirroring each other indefinitely.
//!
//! **Solution:** Bilateral perturbation in a shared reference frame.
//! When a closing pair's correction vector `u` is near zero (the relative
//! velocity sits on the VO boundary), `u` is replaced by a small vector
//! perpendicular to the relative position. Because the relative position
//! flips sign between the two agents of a pair, each agent perturbs in the
//! opposite lateral direction, producing complementary evasive maneuvers
//! without randomness or ID coordination.
//!
//! The perturbation only applies when the agents are actually closing
//! (`v_rel · p_rel > 0`); diverging or passing agents are never perturbed.

use crate::structs::{AgentState, Grid, Point, Vector2D};

const EPSILON: f64 = 1e-5;
/// Perturbation for symmetry breaking - needs to be large enough to create
/// meaningful lateral movement but small enough not to disturb normal operation
const PERTURBATION_EPSILON: f64 = 0.1;
/// Assumed simulation time step for resolving already-overlapping discs.
/// When interpenetrating, the VO is cut off at one time step rather than the
/// full horizon so the constraint pushes the agent out immediately.
const COLLISION_RESOLUTION_TIME_STEP: f64 = 0.1;

/// An ORCA constraint: the half-plane of velocities on the LEFT of the
/// directed line through `point` with direction `direction`.
/// A velocity `v` is valid iff det(direction, point - v) <= 0.
#[derive(Debug, Clone)]
pub struct OrcaLine {
    /// A point on the dividing line of the half-plane
    pub point: Vector2D,
    /// Unit direction of the dividing line (feasible side on the left)
    pub direction: Vector2D,
}

impl OrcaLine {
    pub fn new(point: Vector2D, direction: Vector2D) -> Self {
        OrcaLine { point, direction }
    }
}

fn det(a: &Vector2D, b: &Vector2D) -> f64 {
    a.x * b.y - a.y * b.x
}

/// Computes a new, safe velocity considering both agent neighbors and static
/// obstacle discs (e.g. grid obstacle cells approximated by inscribed discs).
pub fn compute_new_velocity_with_obstacles(
    agent: &AgentState,
    neighbors: &[&AgentState],
    obstacles: &[Point],
    obstacle_radius: f64,
    time_horizon: f64,
    time_horizon_obst: f64,
) -> Vector2D {
    // Obstacle lines FIRST: the infeasible-case solver keeps the first
    // `num_obstacle_lines` constraints hard while relaxing agent constraints.
    let mut orca_lines = Vec::with_capacity(obstacles.len() + neighbors.len());

    for obstacle in obstacles {
        if let Some(line) =
            compute_orca_line_for_obstacle(agent, obstacle, obstacle_radius, time_horizon_obst)
        {
            orca_lines.push(line);
        }
    }
    let num_obstacle_lines = orca_lines.len();

    for &neighbor in neighbors {
        if let Some(line) = compute_orca_line_for_agent(agent, neighbor, time_horizon) {
            orca_lines.push(line);
        }
    }

    let (fail_index, mut result) =
        linear_program2(&orca_lines, agent.max_speed, &agent.pref_velocity, false);

    if fail_index < orca_lines.len() {
        // Constraints jointly infeasible: minimize the maximum violation of
        // the agent constraints while keeping obstacle constraints hard.
        linear_program3(
            &orca_lines,
            num_obstacle_lines,
            fail_index,
            agent.max_speed,
            &mut result,
        );
    }

    result
}

/// Batched simulation step: computes new velocities for every agent.
///
/// Neighbor selection (within `neighbor_distance`, nearest `max_neighbors`)
/// and obstacle-cell gathering happen natively, so the Python simulation loop
/// makes one FFI call per tick instead of one per agent.
#[allow(clippy::too_many_arguments)]
pub fn compute_all_velocities(
    agents: &[AgentState],
    grid: Option<&Grid>,
    time_horizon: f64,
    time_horizon_obst: f64,
    neighbor_distance: f64,
    max_neighbors: usize,
    obstacle_radius: f64,
) -> Vec<Vector2D> {
    // Collect obstacle cell centers once.
    let obstacle_cells: Vec<Point> = match grid {
        Some(grid) => {
            let mut cells = Vec::new();
            for y in 0..grid.height {
                for x in 0..grid.width {
                    if grid.is_obstacle(x, y) {
                        cells.push(Point::new(x as f64, y as f64));
                    }
                }
            }
            cells
        }
        None => Vec::new(),
    };

    agents
        .iter()
        .map(|agent| {
            // Nearest neighbors within sensing range
            let mut neighbors: Vec<&AgentState> = agents
                .iter()
                .filter(|other| {
                    other.id != agent.id && agent.distance_to(other) <= neighbor_distance
                })
                .collect();
            neighbors.sort_by(|a, b| {
                agent
                    .distance_to(a)
                    .partial_cmp(&agent.distance_to(b))
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            neighbors.truncate(max_neighbors);

            // Obstacle cells close enough to constrain this agent within the
            // obstacle horizon.
            let obstacle_range =
                agent.radius + obstacle_radius + agent.max_speed * time_horizon_obst + 0.5;
            let nearby_obstacles: Vec<Point> = obstacle_cells
                .iter()
                .filter(|cell| agent.position.distance(cell) <= obstacle_range)
                .copied()
                .collect();

            compute_new_velocity_with_obstacles(
                agent,
                &neighbors,
                &nearby_obstacles,
                obstacle_radius,
                time_horizon,
                time_horizon_obst,
            )
        })
        .collect()
}

/// Shared truncated-VO case analysis (van den Berg et al. 2011 / RVO2).
///
/// Returns the correction vector `u` (from the relative velocity to the
/// closest point on the VO boundary) and the unit line direction whose left
/// side is the feasible half-plane.
fn vo_correction(
    relative_position: Vector2D,
    relative_velocity: Vector2D,
    combined_radius: f64,
    time_horizon: f64,
) -> (Vector2D, Vector2D) {
    let dist_sq = relative_position.dot(&relative_position);
    let combined_radius_sq = combined_radius * combined_radius;

    if dist_sq > combined_radius_sq {
        // No current collision: VO is the cutoff disk at p/tau plus tangent legs.
        let inv_time_horizon = 1.0 / time_horizon;
        let w = relative_velocity - relative_position * inv_time_horizon;
        let w_length_sq = w.dot(&w);
        let dot1 = w.dot(&relative_position);

        if dot1 < 0.0 && dot1 * dot1 > combined_radius_sq * w_length_sq {
            // Closest boundary point is on the cutoff disk.
            let w_length = w_length_sq.sqrt();
            let unit_w = w * (1.0 / w_length);
            let direction = Vector2D::new(unit_w.y, -unit_w.x);
            let u = unit_w * (combined_radius * inv_time_horizon - w_length);
            (u, direction)
        } else {
            // Closest boundary point is on a tangent leg.
            let leg = (dist_sq - combined_radius_sq).sqrt();
            let direction = if det(&relative_position, &w) > 0.0 {
                // Left leg
                Vector2D::new(
                    relative_position.x * leg - relative_position.y * combined_radius,
                    relative_position.x * combined_radius + relative_position.y * leg,
                ) * (1.0 / dist_sq)
            } else {
                // Right leg
                Vector2D::new(
                    relative_position.x * leg + relative_position.y * combined_radius,
                    -relative_position.x * combined_radius + relative_position.y * leg,
                ) * (-1.0 / dist_sq)
            };
            let dot2 = relative_velocity.dot(&direction);
            let u = direction * dot2 - relative_velocity;
            (u, direction)
        }
    } else {
        // Discs already overlap: cut off at one time step to separate now.
        let inv_time_step = 1.0 / COLLISION_RESOLUTION_TIME_STEP;
        let w = relative_velocity - relative_position * inv_time_step;
        let w_length = w.magnitude();
        let unit_w = if w_length > EPSILON {
            w * (1.0 / w_length)
        } else {
            // Relative velocity exactly at the disk center: push out laterally
            Vector2D::new(-relative_position.y, relative_position.x).normalize()
        };
        let direction = Vector2D::new(unit_w.y, -unit_w.x);
        let u = unit_w * (combined_radius * inv_time_step - w_length);
        (u, direction)
    }
}

/// Computes an ORCA line constraint for agent-agent interaction.
/// Each agent takes half the responsibility for avoiding the collision.
fn compute_orca_line_for_agent(
    agent: &AgentState,
    neighbor: &AgentState,
    time_horizon: f64,
) -> Option<OrcaLine> {
    let relative_position = neighbor.position - agent.position;
    let relative_velocity = agent.velocity - neighbor.velocity;

    // Degeneracy check: agents at the same position
    if relative_position.dot(&relative_position) < EPSILON * EPSILON {
        return None; // Emergency separation needed
    }

    let combined_radius = agent.radius + neighbor.radius;
    let (u, direction) = vo_correction(
        relative_position,
        relative_velocity,
        combined_radius,
        time_horizon,
    );

    // --- DETERMINISTIC SYMMETRY-BREAKING PERTURBATION ---
    // Only for closing pairs whose relative velocity sits on the VO boundary
    // (the mirror-deadlock case). Diverging or passing agents are never
    // perturbed.
    let closing = relative_velocity.dot(&relative_position) > 0.0;
    let (u, direction) = if closing && u.dot(&u) < PERTURBATION_EPSILON * PERTURBATION_EPSILON {
        // Use relative_position as a shared reference frame: it flips sign
        // between the two agents of the pair, so each perturbs the opposite way.
        let normal = Vector2D::new(-relative_position.y, relative_position.x).normalize();
        (
            normal * PERTURBATION_EPSILON,
            Vector2D::new(normal.y, -normal.x),
        )
    } else {
        (u, direction)
    };

    // Each agent takes half the responsibility for avoiding the collision
    Some(OrcaLine::new(agent.velocity + u * 0.5, direction))
}

/// Computes an ORCA line constraint for a static obstacle disc.
/// The agent takes FULL responsibility (the obstacle will not reciprocate).
fn compute_orca_line_for_obstacle(
    agent: &AgentState,
    obstacle_center: &Point,
    obstacle_radius: f64,
    time_horizon_obst: f64,
) -> Option<OrcaLine> {
    let relative_position = *obstacle_center - agent.position;

    if relative_position.dot(&relative_position) < EPSILON * EPSILON {
        return None;
    }

    let combined_radius = agent.radius + obstacle_radius;
    // Obstacle is static, so the relative velocity is the agent's velocity.
    let (u, direction) = vo_correction(
        relative_position,
        agent.velocity,
        combined_radius,
        time_horizon_obst,
    );

    Some(OrcaLine::new(agent.velocity + u, direction))
}

// ---------------------------------------------------------------------------
// Geometric linear programs (van den Berg et al. 2011, Sec. 5.4 / RVO2)
// ---------------------------------------------------------------------------

/// Re-optimizes on the boundary line of constraint `line_no`, subject to all
/// constraints before it and the speed disc. Returns false if infeasible.
fn linear_program1(
    lines: &[OrcaLine],
    line_no: usize,
    radius: f64,
    opt_velocity: &Vector2D,
    direction_opt: bool,
    result: &mut Vector2D,
) -> bool {
    let line = &lines[line_no];
    let dot_product = line.point.dot(&line.direction);
    let discriminant = dot_product * dot_product + radius * radius - line.point.dot(&line.point);

    if discriminant < 0.0 {
        // The speed disc does not intersect this constraint's boundary line.
        return false;
    }

    let sqrt_discriminant = discriminant.sqrt();
    let mut t_left = -dot_product - sqrt_discriminant;
    let mut t_right = -dot_product + sqrt_discriminant;

    for prev in lines.iter().take(line_no) {
        let denominator = det(&line.direction, &prev.direction);
        let numerator = det(&prev.direction, &(line.point - prev.point));

        if denominator.abs() <= EPSILON {
            // Lines are parallel
            if numerator < 0.0 {
                return false;
            }
            continue;
        }

        let t = numerator / denominator;
        if denominator >= 0.0 {
            t_right = t_right.min(t);
        } else {
            t_left = t_left.max(t);
        }

        if t_left > t_right {
            return false;
        }
    }

    if direction_opt {
        // Optimize direction
        if opt_velocity.dot(&line.direction) > 0.0 {
            *result = line.point + line.direction * t_right;
        } else {
            *result = line.point + line.direction * t_left;
        }
    } else {
        // Optimize closest point
        let t = line.direction.dot(&(*opt_velocity - line.point));
        *result = line.point + line.direction * t.clamp(t_left, t_right);
    }

    true
}

/// Finds the velocity closest to `opt_velocity` satisfying all `lines` and
/// the speed disc. Returns (lines.len(), result) on success, or the index of
/// the first unsatisfiable constraint with the best result so far.
fn linear_program2(
    lines: &[OrcaLine],
    radius: f64,
    opt_velocity: &Vector2D,
    direction_opt: bool,
) -> (usize, Vector2D) {
    let mut result = if direction_opt {
        // opt_velocity is a unit direction in this mode
        *opt_velocity * radius
    } else if opt_velocity.dot(opt_velocity) > radius * radius {
        opt_velocity.normalize() * radius
    } else {
        *opt_velocity
    };

    for (i, line) in lines.iter().enumerate() {
        if det(&line.direction, &(line.point - result)) > 0.0 {
            // Current result violates constraint i: re-project.
            let temp_result = result;
            if !linear_program1(lines, i, radius, opt_velocity, direction_opt, &mut result) {
                return (i, temp_result);
            }
        }
    }

    (lines.len(), result)
}

/// Infeasible case: finds the velocity minimizing the maximum violation of
/// the constraints from `begin_line` onward, keeping the first
/// `num_obstacle_lines` constraints hard.
fn linear_program3(
    lines: &[OrcaLine],
    num_obstacle_lines: usize,
    begin_line: usize,
    radius: f64,
    result: &mut Vector2D,
) {
    let mut distance = 0.0;

    for i in begin_line..lines.len() {
        if det(&lines[i].direction, &(lines[i].point - *result)) > distance {
            // result violates constraint i beyond the current worst violation.
            let mut proj_lines: Vec<OrcaLine> = lines[..num_obstacle_lines].to_vec();

            for j in num_obstacle_lines..i {
                let determinant = det(&lines[i].direction, &lines[j].direction);
                let point = if determinant.abs() <= EPSILON {
                    // Parallel lines
                    if lines[i].direction.dot(&lines[j].direction) > 0.0 {
                        continue; // same direction: j is redundant here
                    }
                    (lines[i].point + lines[j].point) * 0.5
                } else {
                    lines[i].point
                        + lines[i].direction
                            * (det(&lines[j].direction, &(lines[i].point - lines[j].point))
                                / determinant)
                };
                let direction = (lines[j].direction - lines[i].direction).normalize();
                proj_lines.push(OrcaLine::new(point, direction));
            }

            let temp_result = *result;
            let opt_direction = Vector2D::new(-lines[i].direction.y, lines[i].direction.x);
            let (fail, projected) = linear_program2(&proj_lines, radius, &opt_direction, true);
            if fail < proj_lines.len() {
                // This should in principle not happen: keep the previous result.
                *result = temp_result;
            } else {
                *result = projected;
            }

            distance = det(&lines[i].direction, &(lines[i].point - *result));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::structs::Point;

    /// A velocity satisfies an ORCA line iff it lies on the left of the line.
    fn satisfies(line: &OrcaLine, v: &Vector2D) -> bool {
        det(&line.direction, &(line.point - *v)) <= EPSILON
    }

    /// Agent-only convenience wrapper used throughout these tests.
    fn compute_new_velocity(
        agent: &AgentState,
        neighbors: &[&AgentState],
        time_horizon: f64,
    ) -> Vector2D {
        compute_new_velocity_with_obstacles(agent, neighbors, &[], 0.5, time_horizon, 1.0)
    }

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
        assert!(
            result.y.abs() > 0.001,
            "Expected lateral movement, got: {:?}",
            result
        );
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
        assert!((result - agent.pref_velocity).magnitude() < 1e-9);
    }

    #[test]
    fn test_infeasible_scenario_fallback() {
        // Agent A surrounded by three very close stationary agents; the VOs
        // heavily overlap leaving no feasible region. The safest action is to
        // (nearly) stop.
        let agent_a = AgentState::new(
            0,
            Point::new(0.0, 0.0),
            Vector2D::new(0.0, 0.0),
            0.5,
            Vector2D::new(1.0, 0.0),
            1.0,
        );

        let distance = 2.0 * agent_a.radius;
        let agent_b = AgentState::new(
            1,
            Point::new(distance, 0.0),
            Vector2D::new(0.0, 0.0),
            0.5,
            Vector2D::new(0.0, 0.0),
            1.0,
        );
        let agent_c = AgentState::new(
            2,
            Point::new(-distance * 0.5, distance * 0.866),
            Vector2D::new(0.0, 0.0),
            0.5,
            Vector2D::new(0.0, 0.0),
            1.0,
        );
        let agent_d = AgentState::new(
            3,
            Point::new(-distance * 0.5, -distance * 0.866),
            Vector2D::new(0.0, 0.0),
            0.5,
            Vector2D::new(0.0, 0.0),
            1.0,
        );

        let neighbors = vec![&agent_b, &agent_c, &agent_d];
        let result = compute_new_velocity(&agent_a, &neighbors, 0.5);

        assert!(
            result.magnitude() < 0.35,
            "Expected near-zero velocity in infeasible scenario, got: {:?}",
            result
        );
    }

    #[test]
    fn test_symmetry_deadlock_resolution() {
        // The classic head-on scenario.
        let agent_a = AgentState::new(
            0,
            Point::new(0.0, 0.0),
            Vector2D::new(1.0, 0.0),
            0.5,
            Vector2D::new(1.0, 0.0),
            1.0,
        );
        let agent_b = AgentState::new(
            1,
            Point::new(4.0, 0.0),
            Vector2D::new(-1.0, 0.0),
            0.5,
            Vector2D::new(-1.0, 0.0),
            1.0,
        );

        let result_a = compute_new_velocity(&agent_a, &[&agent_b], 2.0);
        let result_b = compute_new_velocity(&agent_b, &[&agent_a], 2.0);

        assert!(
            result_a.y.abs() > 0.001,
            "Agent A should have lateral movement, got: {:?}",
            result_a
        );
        assert!(
            result_b.y.abs() > 0.001,
            "Agent B should have lateral movement, got: {:?}",
            result_b
        );
        assert!(
            result_a.y * result_b.y < 0.0,
            "Agents should move in opposite lateral directions. A: {:?}, B: {:?}",
            result_a,
            result_b
        );
    }

    #[test]
    fn test_circle_dance_stability() {
        // 8 agents on a circle, each heading to the antipodal point.
        let num_agents = 8;
        let circle_radius = 5.0;
        let agent_radius = 0.3;
        let max_speed = 1.0;

        let mut agents = Vec::new();
        for i in 0..num_agents {
            let angle = (i as f64 / num_agents as f64) * 2.0 * std::f64::consts::PI;
            let pos = Point::new(circle_radius * angle.cos(), circle_radius * angle.sin());
            let opposite_angle = angle + std::f64::consts::PI;
            let opposite_pos = Point::new(
                circle_radius * opposite_angle.cos(),
                circle_radius * opposite_angle.sin(),
            );
            let pref_vel = (opposite_pos - pos).normalize() * max_speed;
            agents.push(AgentState::new(
                i,
                pos,
                Vector2D::new(0.0, 0.0),
                agent_radius,
                pref_vel,
                max_speed,
            ));
        }

        let num_steps = 250;
        let dt = 0.02;
        let time_horizon = 3.0;
        let mut agent_positions = agents.clone();
        let mut speed_history = Vec::new();

        for step in 0..num_steps {
            let mut new_velocities = Vec::new();
            for (i, agent) in agent_positions.iter().enumerate() {
                let neighbors: Vec<&AgentState> = agent_positions
                    .iter()
                    .enumerate()
                    .filter(|(j, _)| *j != i)
                    .map(|(_, neighbor)| neighbor)
                    .collect();
                new_velocities.push(compute_new_velocity(agent, &neighbors, time_horizon));
            }

            for (i, agent) in agent_positions.iter_mut().enumerate() {
                agent.velocity = new_velocities[i];
                agent.position = agent.position + agent.velocity * dt;
            }

            if step >= num_steps - 50 {
                let total_speed: f64 = agent_positions
                    .iter()
                    .map(|agent| agent.velocity.magnitude())
                    .sum();
                speed_history.push(total_speed / num_agents as f64);
            }

            // Allow 10% tolerance in this symmetric stress test.
            for i in 0..agent_positions.len() {
                for j in (i + 1)..agent_positions.len() {
                    let distance = agent_positions[i]
                        .position
                        .distance(&agent_positions[j].position);
                    let min_distance = agent_positions[i].radius + agent_positions[j].radius;
                    assert!(
                        distance >= min_distance * 0.90,
                        "Collision detected at step {}: agents {} and {} are {} apart (min: {})",
                        step,
                        i,
                        j,
                        distance,
                        min_distance
                    );
                }
            }
        }

        let final_avg_speed: f64 = speed_history.iter().sum::<f64>() / speed_history.len() as f64;
        assert!(
            final_avg_speed > 0.25 * max_speed,
            "System appears to have deadlocked. Average speed over last 50 steps: {:.3}",
            final_avg_speed
        );
    }

    // --- ORCA line construction ---

    #[test]
    fn test_compute_orca_line_approaching_agents() {
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
        assert!(
            result.is_some(),
            "Should produce ORCA line for approaching agents"
        );
        let line = result.unwrap();
        assert!(
            (line.direction.magnitude() - 1.0).abs() < 0.01,
            "Direction should be normalized"
        );
    }

    #[test]
    fn test_compute_orca_line_overlapping_agents() {
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
        assert!(
            result.is_none(),
            "Should return None for overlapping agents"
        );
    }

    #[test]
    fn test_compute_orca_line_stationary_agents() {
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
        assert!(
            result.is_some(),
            "Should produce ORCA line even for stationary agents"
        );
    }

    // --- Regression tests for the outside-VO half-plane orientation ---

    #[test]
    fn test_diverging_agents_keep_preferred_velocity() {
        // Two agents moving directly apart: no predicted collision, so ORCA
        // must not deflect or slow either of them.
        let agent = AgentState::new(
            0,
            Point::new(0.0, 0.0),
            Vector2D::new(-1.0, 0.0),
            0.5,
            Vector2D::new(-1.0, 0.0),
            1.0,
        );
        let neighbor = AgentState::new(
            1,
            Point::new(5.0, 0.0),
            Vector2D::new(1.0, 0.0),
            0.5,
            Vector2D::new(1.0, 0.0),
            1.0,
        );

        let result = compute_new_velocity(&agent, &[&neighbor], 2.0);
        assert!(
            (result - agent.pref_velocity).magnitude() < 0.05,
            "Diverging agent should keep its preferred velocity, got {:?}",
            result
        );
    }

    #[test]
    fn test_distant_stationary_neighbor_does_not_slow_agent() {
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
            Point::new(0.0, 8.0), // 8 units away, perpendicular to motion
            Vector2D::new(0.0, 0.0),
            0.5,
            Vector2D::new(0.0, 0.0),
            1.0,
        );

        let result = compute_new_velocity(&agent, &[&neighbor], 2.0);
        assert!(
            (result - agent.pref_velocity).magnitude() < 0.05,
            "Agent passing a distant neighbor should keep its preferred velocity, got {:?}",
            result
        );
    }

    #[test]
    fn test_orca_line_satisfied_with_slack_when_no_collision_course() {
        let agent = AgentState::new(
            0,
            Point::new(0.0, 0.0),
            Vector2D::new(-1.0, 0.0),
            0.5,
            Vector2D::new(-1.0, 0.0),
            1.0,
        );
        let neighbor = AgentState::new(
            1,
            Point::new(5.0, 0.0),
            Vector2D::new(1.0, 0.0),
            0.5,
            Vector2D::new(1.0, 0.0),
            1.0,
        );

        let line = compute_orca_line_for_agent(&agent, &neighbor, 2.0).unwrap();
        assert!(
            satisfies(&line, &agent.velocity),
            "Current velocity of a non-colliding agent must satisfy its own ORCA constraint"
        );
    }

    // --- Linear program tests ---

    #[test]
    fn test_lp2_no_constraints_returns_clipped_preference() {
        let (fail, result) = linear_program2(&[], 1.0, &Vector2D::new(5.0, 0.0), false);
        assert_eq!(fail, 0);
        assert!(
            (result.magnitude() - 1.0).abs() < 1e-9,
            "Should clip to max speed"
        );
        assert!(result.x > 0.99, "Should preserve direction");
    }

    #[test]
    fn test_lp2_single_constraint_projects_exactly() {
        // Constraint: vy >= 0.5 (line through (0, 0.5) pointing +x; left = above)
        let lines = vec![OrcaLine::new(
            Vector2D::new(0.0, 0.5),
            Vector2D::new(1.0, 0.0),
        )];
        let pref = Vector2D::new(0.3, 0.0);
        let (fail, result) = linear_program2(&lines, 2.0, &pref, false);
        assert_eq!(fail, lines.len(), "Should be feasible");
        // Exact L2 projection of (0.3, 0) onto vy = 0.5 is (0.3, 0.5)
        assert!(
            (result.x - 0.3).abs() < 1e-6 && (result.y - 0.5).abs() < 1e-6,
            "Expected exact projection (0.3, 0.5), got {:?}",
            result
        );
    }

    #[test]
    fn test_lp2_solution_satisfies_all_constraints() {
        let lines = vec![
            OrcaLine::new(Vector2D::new(0.0, 0.1), Vector2D::new(1.0, 0.0)), // vy >= 0.1
            OrcaLine::new(Vector2D::new(0.1, 0.0), Vector2D::new(0.0, -1.0)), // vx >= 0.1
        ];
        let pref = Vector2D::new(1.0, 1.0);
        let (fail, result) = linear_program2(&lines, 2.0, &pref, false);
        assert_eq!(fail, lines.len());
        for line in &lines {
            assert!(
                satisfies(line, &result),
                "Constraint violated by {:?}",
                result
            );
        }
    }

    #[test]
    fn test_lp3_infeasible_returns_bounded_velocity() {
        // Contradictory constraints: vx >= 1 and vx <= -1
        let lines = vec![
            OrcaLine::new(Vector2D::new(1.0, 0.0), Vector2D::new(0.0, -1.0)), // vx >= 1
            OrcaLine::new(Vector2D::new(-1.0, 0.0), Vector2D::new(0.0, 1.0)), // vx <= -1
        ];
        let (fail, mut result) = linear_program2(&lines, 1.0, &Vector2D::new(0.5, 0.0), false);
        assert!(fail < lines.len(), "Should be infeasible");
        linear_program3(&lines, 0, fail, 1.0, &mut result);
        assert!(result.x.is_finite() && result.y.is_finite());
        assert!(
            result.magnitude() <= 1.0 + 1e-6,
            "Should respect speed limit"
        );
        // Minimizing max violation of vx>=1 and vx<=-1 gives vx = 0
        assert!(
            result.x.abs() < 0.01,
            "Expected balanced violation, got {:?}",
            result
        );
    }

    // --- Obstacle avoidance tests ---

    #[test]
    fn test_agent_driving_at_wall_never_penetrates() {
        // Agent whose preferred velocity points straight into an obstacle
        // cell. Over a rollout it must deviate (slide around or stop) and
        // never overlap the obstacle disc.
        let obstacle = Point::new(2.0, 0.0);
        let obstacle_radius = 0.5;
        let mut agent = AgentState::new(
            0,
            Point::new(0.0, 0.0),
            Vector2D::new(1.0, 0.0),
            0.4,
            Vector2D::new(1.0, 0.0),
            1.0,
        );

        let dt = 0.05;
        for step in 0..60 {
            let v = compute_new_velocity_with_obstacles(
                &agent,
                &[],
                &[obstacle],
                obstacle_radius,
                2.0,
                1.0,
            );
            agent.velocity = v;
            agent.position = agent.position + v * dt;
            let gap = agent.position.distance(&obstacle) - (agent.radius + obstacle_radius);
            assert!(
                gap >= -0.01,
                "Agent penetrated the obstacle at step {}: gap {}",
                step,
                gap
            );
        }
    }

    #[test]
    fn test_agent_moving_parallel_to_wall_keeps_speed() {
        // Agent sliding along a wall one cell away, moving parallel to it.
        let agent = AgentState::new(
            0,
            Point::new(0.0, 0.0),
            Vector2D::new(1.0, 0.0),
            0.4,
            Vector2D::new(1.0, 0.0),
            1.0,
        );
        // Wall cells above the agent's path
        let obstacles = vec![
            Point::new(0.0, 1.5),
            Point::new(1.0, 1.5),
            Point::new(2.0, 1.5),
        ];

        let result = compute_new_velocity_with_obstacles(&agent, &[], &obstacles, 0.5, 2.0, 1.0);
        assert!(
            (result - agent.pref_velocity).magnitude() < 0.1,
            "Agent parallel to a wall should keep its preferred velocity, got {:?}",
            result
        );
    }

    #[test]
    fn test_obstacle_constraint_uses_full_responsibility() {
        // For a static obstacle the line passes through velocity + u (full
        // responsibility), not velocity + u/2.
        let agent = AgentState::new(
            0,
            Point::new(0.0, 0.0),
            Vector2D::new(1.0, 0.0),
            0.4,
            Vector2D::new(1.0, 0.0),
            1.0,
        );
        let obstacle = Point::new(1.2, 0.0);
        let obstacle_line = compute_orca_line_for_obstacle(&agent, &obstacle, 0.5, 1.0).unwrap();

        // Compare with an agent-agent line for a stationary "twin" of the
        // obstacle: same VO geometry, half responsibility.
        let twin = AgentState::new(
            1,
            Point::new(1.2, 0.0),
            Vector2D::new(0.0, 0.0),
            0.5,
            Vector2D::new(0.0, 0.0),
            1.0,
        );
        let agent_line = compute_orca_line_for_agent(&agent, &twin, 1.0).unwrap();

        let obstacle_offset = (obstacle_line.point - agent.velocity).magnitude();
        let agent_offset = (agent_line.point - agent.velocity).magnitude();
        assert!(
            (obstacle_offset - 2.0 * agent_offset).abs() < 1e-6,
            "Obstacle line offset should be exactly twice the reciprocal agent offset"
        );
    }

    #[test]
    fn test_batched_step_matches_singleton_calls() {
        let agents = vec![
            AgentState::new(
                0,
                Point::new(0.0, 0.0),
                Vector2D::new(1.0, 0.0),
                0.4,
                Vector2D::new(1.0, 0.0),
                1.0,
            ),
            AgentState::new(
                1,
                Point::new(4.0, 0.0),
                Vector2D::new(-1.0, 0.0),
                0.4,
                Vector2D::new(-1.0, 0.0),
                1.0,
            ),
        ];

        let batched = compute_all_velocities(&agents, None, 2.0, 1.0, 10.0, 10, 0.5);
        let single_0 = compute_new_velocity(&agents[0], &[&agents[1]], 2.0);
        let single_1 = compute_new_velocity(&agents[1], &[&agents[0]], 2.0);

        assert!((batched[0] - single_0).magnitude() < 1e-9);
        assert!((batched[1] - single_1).magnitude() < 1e-9);
    }
}
