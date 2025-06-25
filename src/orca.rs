use crate::structs::{AgentState, Vector2D, Point, Line};

const EPSILON: f64 = 1e-10;

pub struct ORCASimulator {
    time_horizon: f64,
    neighbor_distance: f64,
    max_neighbors: usize,
}

impl ORCASimulator {
    pub fn new() -> Self {
        ORCASimulator {
            time_horizon: 1.5,
            neighbor_distance: 8.0,
            max_neighbors: 10,
        }
    }

    pub fn with_params(time_horizon: f64, neighbor_distance: f64, max_neighbors: usize) -> Self {
        ORCASimulator {
            time_horizon,
            neighbor_distance,
            max_neighbors,
        }
    }

    pub fn compute_velocity(&self, agent: &AgentState, neighbors: &[&AgentState]) -> Vector2D {
        let relevant_neighbors = self.get_relevant_neighbors(agent, neighbors);
        let orca_lines = self.compute_orca_lines(agent, &relevant_neighbors);
        self.solve_linear_program(&orca_lines, agent.max_speed, &agent.pref_velocity)
    }

    fn get_relevant_neighbors<'a>(&self, agent: &AgentState, neighbors: &[&'a AgentState]) -> Vec<&'a AgentState> {
        let mut relevant = Vec::new();
        
        for &neighbor in neighbors {
            if neighbor.id != agent.id {
                let distance = agent.distance_to(neighbor);
                if distance < self.neighbor_distance {
                    relevant.push(neighbor);
                }
            }
        }
        
        // Sort by distance and take closest neighbors
        relevant.sort_by(|a, b| {
            let dist_a = agent.distance_to(a);
            let dist_b = agent.distance_to(b);
            dist_a.partial_cmp(&dist_b).unwrap()
        });
        
        relevant.truncate(self.max_neighbors);
        relevant
    }

    fn compute_orca_lines(&self, agent: &AgentState, neighbors: &[&AgentState]) -> Vec<Line> {
        let mut orca_lines = Vec::new();
        
        for &neighbor in neighbors {
            let relative_position = neighbor.position - agent.position;
            let relative_velocity = agent.velocity - neighbor.velocity;
            let distance_sq = relative_position.dot(&relative_position);
            let combined_radius = agent.radius + neighbor.radius;
            let combined_radius_sq = combined_radius * combined_radius;
            
            let line = if distance_sq > combined_radius_sq {
                // No collision, compute ORCA line
                self.compute_orca_line_no_collision(
                    &relative_position,
                    &relative_velocity,
                    combined_radius,
                )
            } else {
                // Collision, compute collision avoidance line
                self.compute_orca_line_collision(
                    &relative_position,
                    &relative_velocity,
                    combined_radius,
                )
            };
            
            orca_lines.push(line);
        }
        
        orca_lines
    }

    fn compute_orca_line_no_collision(
        &self,
        relative_position: &Vector2D,
        relative_velocity: &Vector2D,
        combined_radius: f64,
    ) -> Line {
        let w = *relative_velocity - (*relative_position * (1.0 / self.time_horizon));
        let w_length_sq = w.dot(&w);
        
        let dot_product1 = w.dot(relative_position);
        
        if dot_product1 < 0.0 && dot_product1 * dot_product1 > combined_radius * combined_radius * w_length_sq {
            // Project on circle
            let w_length = w_length_sq.sqrt();
            let unit_w = w * (1.0 / w_length);
            
            let direction = Vector2D::new(unit_w.y, -unit_w.x);
            let u = unit_w * (combined_radius / self.time_horizon - w_length);
            
            Line::new(
                Point::new(u.x * 0.5, u.y * 0.5),
                direction,
            )
        } else {
            // Project on legs
            let distance_sq = relative_position.dot(relative_position);
            let leg = (distance_sq - combined_radius * combined_radius).sqrt();
            
            if det(relative_position, &w) > 0.0 {
                // Project on left leg
                let direction = Vector2D::new(
                    relative_position.x * leg - relative_position.y * combined_radius,
                    relative_position.x * combined_radius + relative_position.y * leg,
                ) * (1.0 / distance_sq);
                
                let u = direction * (relative_velocity.dot(&direction));
                
                Line::new(
                    Point::new(u.x * 0.5, u.y * 0.5),
                    Vector2D::new(-direction.y, direction.x),
                )
            } else {
                // Project on right leg
                let direction = Vector2D::new(
                    relative_position.x * leg + relative_position.y * combined_radius,
                    -relative_position.x * combined_radius + relative_position.y * leg,
                ) * (-1.0 / distance_sq);
                
                let u = direction * (relative_velocity.dot(&direction));
                
                Line::new(
                    Point::new(u.x * 0.5, u.y * 0.5),
                    Vector2D::new(-direction.y, direction.x),
                )
            }
        }
    }

    fn compute_orca_line_collision(
        &self,
        relative_position: &Vector2D,
        relative_velocity: &Vector2D,
        combined_radius: f64,
    ) -> Line {
        let time = (combined_radius - relative_position.magnitude()) / relative_velocity.magnitude().max(EPSILON);
        let w = *relative_velocity - (*relative_position * (1.0 / time.max(EPSILON)));
        let w_length = w.magnitude();
        
        let unit_w = if w_length > EPSILON {
            w * (1.0 / w_length)
        } else {
            Vector2D::new(1.0, 0.0)
        };
        
        Line::new(
            Point::new(unit_w.x * 0.5, unit_w.y * 0.5),
            Vector2D::new(-unit_w.y, unit_w.x),
        )
    }

    fn solve_linear_program(&self, orca_lines: &[Line], max_speed: f64, pref_velocity: &Vector2D) -> Vector2D {
        let mut optimal_velocity = *pref_velocity;
        
        // Clip to maximum speed
        if optimal_velocity.magnitude() > max_speed {
            optimal_velocity = optimal_velocity.normalize() * max_speed;
        }
        
        // Apply ORCA constraints
        for (i, line) in orca_lines.iter().enumerate() {
            if det(&(optimal_velocity - Vector2D::new(line.point.x, line.point.y)), &line.direction) > 0.0 {
                // Current velocity violates constraint, project to feasible region
                optimal_velocity = self.project_to_line(line, &optimal_velocity, max_speed, orca_lines, i);
            }
        }
        
        optimal_velocity
    }

    fn project_to_line(
        &self,
        line: &Line,
        velocity: &Vector2D,
        max_speed: f64,
        orca_lines: &[Line],
        line_index: usize,
    ) -> Vector2D {
        let line_point = Vector2D::new(line.point.x, line.point.y);
        let dot_product = (*velocity - line_point).dot(&line.direction);
        let projected = line_point + line.direction * dot_product;
        
        // Ensure projected velocity doesn't exceed max speed
        if projected.magnitude() <= max_speed {
            // Check if projected velocity satisfies all previous constraints
            let mut feasible = true;
            for (i, other_line) in orca_lines.iter().enumerate() {
                if i < line_index {
                    let other_point = Vector2D::new(other_line.point.x, other_line.point.y);
                    if det(&(projected - other_point), &other_line.direction) > EPSILON {
                        feasible = false;
                        break;
                    }
                }
            }
            
            if feasible {
                return projected;
            }
        }
        
        // Find intersection with circle of max speed
        let line_point_to_center = line_point * -1.0;
        let discriminant = line_point_to_center.dot(&line.direction).powi(2) +
            max_speed * max_speed - line_point_to_center.dot(&line_point_to_center);
        
        if discriminant < 0.0 {
            // No intersection, return closest point on line
            projected
        } else {
            let sqrt_discriminant = discriminant.sqrt();
            let t1 = -line_point_to_center.dot(&line.direction) - sqrt_discriminant;
            let t2 = -line_point_to_center.dot(&line.direction) + sqrt_discriminant;
            
            let candidate1 = line_point + line.direction * t1;
            let candidate2 = line_point + line.direction * t2;
            
            // Choose the candidate closer to preferred velocity
            let dist1 = (candidate1 - *velocity).magnitude();
            let dist2 = (candidate2 - *velocity).magnitude();
            
            if dist1 < dist2 {
                candidate1
            } else {
                candidate2
            }
        }
    }
}

fn det(vector1: &Vector2D, vector2: &Vector2D) -> f64 {
    vector1.x * vector2.y - vector1.y * vector2.x
}

pub fn compute_orca_velocity(
    agent: &AgentState,
    neighbors: &[&AgentState],
    time_horizon: f64,
) -> Vector2D {
    let simulator = ORCASimulator::with_params(time_horizon, 10.0, 10);
    simulator.compute_velocity(agent, neighbors)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_no_neighbors() {
        let agent = AgentState::new(
            0,
            Point::new(0.0, 0.0),
            Vector2D::new(0.0, 0.0),
            1.0,
            Vector2D::new(1.0, 0.0),
            2.0,
        );
        
        let neighbors = vec![];
        let result = compute_orca_velocity(&agent, &neighbors, 2.0);
        
        // Should return preferred velocity
        assert!((result.x - 1.0).abs() < EPSILON);
        assert!((result.y - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_head_on_collision() {
        let agent1 = AgentState::new(
            0,
            Point::new(0.0, 0.0),
            Vector2D::new(1.0, 0.0),
            1.0,
            Vector2D::new(1.0, 0.0),
            2.0,
        );
        
        let agent2 = AgentState::new(
            1,
            Point::new(5.0, 0.0),
            Vector2D::new(-1.0, 0.0),
            1.0,
            Vector2D::new(-1.0, 0.0),
            2.0,
        );
        
        let neighbors = vec![&agent2];
        let result = compute_orca_velocity(&agent1, &neighbors, 2.0);
        
        // Should deviate from straight path
        assert!(result.y.abs() > EPSILON || result.x < 1.0);
    }

    #[test]
    fn test_max_speed_constraint() {
        let agent = AgentState::new(
            0,
            Point::new(0.0, 0.0),
            Vector2D::new(0.0, 0.0),
            1.0,
            Vector2D::new(5.0, 0.0), // Preferred velocity exceeds max speed
            2.0, // Max speed
        );
        
        let neighbors = vec![];
        let result = compute_orca_velocity(&agent, &neighbors, 2.0);
        
        // Should be clamped to max speed
        assert!(result.magnitude() <= 2.0 + EPSILON);
        assert!((result.magnitude() - 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_det_function() {
        let v1 = Vector2D::new(1.0, 0.0);
        let v2 = Vector2D::new(0.0, 1.0);
        assert!((det(&v1, &v2) - 1.0).abs() < EPSILON);
        
        let v3 = Vector2D::new(1.0, 1.0);
        let v4 = Vector2D::new(1.0, -1.0);
        assert!((det(&v3, &v4) - (-2.0)).abs() < EPSILON);
    }
}