# Design Document: Optimal Reciprocal Collision Avoidance (ORCA) in Rust

1. High-Level Goal
The objective is to create a robust, efficient, and production-ready Rust library that implements the Optimal Reciprocal Collision Avoidance (ORCA) algorithm. For any given agent, the library will compute a new velocity that is guaranteed to be collision-free for a given time horizon, provided all other agents follow the same protocol. The computed velocity will be as close as possible to the agent's preferred velocity while respecting its maximum speed.

This implementation will be a faithful translation of the algorithm described in the paper "Reciprocal n-Body Collision Avoidance", paying special attention to the precise geometry of Velocity Obstacles (VOs), the handling of both agent and static obstacle interactions, and the robust management of dense, infeasible scenarios.

2. Core Data Structures
Rust

// A 2D vector type is assumed, e.g., glam::Vec2.

/// Represents the state of a dynamic agent.
pub struct Agent {
    /// Current position in 2D space.
    pub position: Vec2,
    /// Current velocity vector.
    pub velocity: Vec2,
    /// Radius of the agent's circular footprint.
    pub radius: f32,
    /// The velocity the agent would choose if there were no obstacles.
    pub pref_velocity: Vec2,
    /// The maximum speed the agent can travel at.
    pub max_speed: f32,
}

/// Defines an ORCA constraint as a half-plane in velocity space.
/// A velocity 'v' is considered valid if it lies on the correct side of the line.
/// The condition for validity is: (v - self.point) · self.direction >= 0.
pub struct OrcaLine {
    /// A point on the dividing line of the half-plane.
    pub point: Vec2,
    /// The outward-pointing normal vector of the half-plane's boundary.
    pub direction: Vec2,
}

/// Represents a static, impassable line-segment obstacle in the environment.
pub struct LineObstacle {
    pub point_a: Vec2,
    pub point_b: Vec2,
}
3. Main Function: compute_new_velocity
This function serves as the primary entry point for the algorithm. It orchestrates the creation of constraints and the final velocity selection.

Rust

/// Computes a new, safe velocity for the given agent.
///
/// # Arguments
/// * `agent`: The agent for which to compute the new velocity.
/// * `neighbors`: A slice of other agents in the vicinity.
/// * `obstacles`: A slice of static line-segment obstacles.
/// * `time_horizon`: The look-ahead time `τ` for agent-agent interactions.
/// * `obstacle_time_horizon`: The look-ahead time for agent-obstacle interactions.
///
/// # Returns
/// * `Ok(Vec2)`: The computed optimal and safe new velocity.
/// * `Err(String)`: If an unrecoverable error occurs.
pub fn compute_new_velocity(
    agent: &Agent,
    neighbors: &[Agent],
    obstacles: &[LineObstacle],
    time_horizon: f32,
    obstacle_time_horizon: f32,
) -> Result<Vec2, String> {
    // Implementation logic follows the steps outlined below.
}
4. Algorithmic Steps
Step 1: Compute Agent-Agent ORCA Constraints
For each neighboring agent, we compute an OrcaLine that defines the half-plane of permissible velocities for our agent.

Function Signature:

Rust

fn compute_orca_line_for_agent(agent: &Agent, neighbor: &Agent, time_horizon: f32) -> Option<OrcaLine>
Logic:

Calculate Relative State:

relative_position = neighbor.position - agent.position
combined_radius = agent.radius + neighbor.radius
Degeneracy Check: If relative_position is near-zero (agents are overlapping), return None to avoid numerical instability. A higher-level emergency separation behavior should be triggered.
Define the Truncated Velocity Obstacle (VO):

The VO is a cone in velocity space, truncated by a circular cap.
The cap is a disk centered at vo_disk_center = relative_position / time_horizon.
The disk's radius is vo_disk_radius = combined_radius / time_horizon.
Find Closest Point on VO Boundary:

Determine the relative "optimization velocity", which is the current relative velocity: v_opt_relative = agent.velocity - neighbor.velocity. This ensures reciprocity as it's based on observable state.

Find the closest point on the entire boundary of the truncated VO (\partial VO) to v_opt_relative. This requires checking both the circular cap and the cone's legs.
If v_opt_relative is inside the cone, the closest point will be on the boundary.
If v_opt_relative is outside, the closest point is still on the boundary, effectively pushing the velocity away from the VO.
This projection is the core geometric calculation of the algorithm.
Calculate the Correction Vector u:

The vector u is the displacement from v_opt_relative to the closest point on the VO boundary.
u = closest_point_on_boundary - v_opt_relative
Degeneracy Check: If u is near-zero, the relative velocity is already safe. No constraint is needed, so return None.
Construct the ORCA Half-Plane:

Each agent takes half the responsibility for the correction.
The point on the dividing line is point = agent.velocity + 0.5 * u.
The outward normal of the half-plane is direction = u.normalize().
Return Some(OrcaLine { point, direction }).
Step 2: Compute Agent-Obstacle ORCA Constraints
Static obstacles are handled differently; the agent assumes full responsibility for avoidance.

Function Signature:

Rust

fn compute_orca_line_for_obstacle(agent: &Agent, obstacle: &LineObstacle, time_horizon: f32) -> Option<OrcaLine>
Logic:

Construct the Obstacle VO: The VO for a line-segment obstacle is the Minkowski sum of the swept volume of the agent and the obstacle shape, as shown in Figure 6 of the paper. This results in a shape with two flat sides and two semi-circular ends.
Find Closest Point on VO Boundary: For obstacles, the agent's optimization velocity is assumed to be zero (v_opt = 0). We find the closest point u on the obstacle's VO boundary to the origin.
Construct the ORCA Half-Plane:
The agent takes 100% responsibility. The line is defined as being tangent to the VO at point u.
The point on the line is point = u.
The outward normal is direction = u.normalize().
Return Some(OrcaLine { point, direction }).
Step 3: Solve for New Velocity (Feasible Case)
This step finds the best velocity within the combined constraints.

Gather Constraints: Collect all OrcaLines from agents and obstacles into a list. Add the max speed constraint, which is a circle D(0, agent.max_speed).
Define Objective: Find the velocity v_new in the feasible region that minimizes the Euclidean distance to the preferred velocity: argmin ||v_new - agent.pref_velocity||.
Solve with 2D Linear Programming: This optimization problem can be solved efficiently. The paper recommends the randomized incremental algorithm with O(n) expected runtime, where n is the number of constraints.
Result: If a solution exists, the solver returns Some(v_new). This is the new velocity for the agent.
Step 4: Handle Infeasibility (Dense Case)
If the 2D linear program has no solution, the feasible region is empty. This can happen in very dense scenarios. We must find the "safest possible" velocity instead of failing.

Trigger: The 2D LP solver returns None.
Define New Objective: Find the velocity that minimally violates the constraints. This is framed as a 3D linear program that minimizes a penalty variable d, which represents the worst penetration distance into a forbidden half-plane.

Solve with 3D Linear Programming:
Objective: minimize d
Constraints:
For each agent OrcaLine: (v_new - line.point) · line.direction + d >= 0. The d allows the constraint to be relaxed.
For each obstacle OrcaLine: (v_new - line.point) · line.direction >= 0. Obstacle constraints are "hard" and must not be relaxed.
Max speed constraint: ||v_new|| <= agent.max_speed.
Result: This 3D LP is always feasible and will return the velocity that is safest under the circumstances. In this mode, the agent's preferred velocity is ignored; its motion is entirely dictated by the surrounding agents to resolve the density.

5. Putting It All Together: Control Flow
Code snippet

function compute_new_velocity(agent, neighbors, obstacles, time_horizon, obstacle_time_horizon):
    // Step 1 & 2: Collect all constraints
    orca_lines = []
    for neighbor in neighbors:
        if let Some(line) = compute_orca_line_for_agent(agent, neighbor, time_horizon):
            orca_lines.push(line)

    for obstacle in obstacles:
        if let Some(line) = compute_orca_line_for_obstacle(agent, obstacle, obstacle_time_horizon):
            orca_lines.push(line)

    // Step 3: Attempt to solve the 2D Linear Program
    result = solve_2d_linear_program(
        orca_lines,
        agent.max_speed,
        agent.pref_velocity
    )

    if let Some(new_velocity) = result:
        // Feasible case: A perfect solution was found
        return Ok(new_velocity)
    else:
        // Step 4: Infeasible case, fall back to the 3D safety program
        safest_velocity = solve_3d_linear_program(
            orca_lines,
            agent.max_speed
        )
        return Ok(safest_velocity)
