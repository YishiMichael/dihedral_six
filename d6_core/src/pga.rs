use geometric_algebra::ppga3d::Line;
use geometric_algebra::ppga3d::Motor;
use geometric_algebra::ppga3d::Point;
use geometric_algebra::Exp;
use geometric_algebra::GeometricProduct;
use geometric_algebra::Magnitude;
use geometric_algebra::RegressiveProduct;
use geometric_algebra::Signum;
use geometric_algebra::Transformation;
use glam::Mat3;
use glam::Mat4;
use glam::Quat;
use glam::Vec3;

// PGA4CS, section 6.7 Example: Univresal Motors, pp.62-64
// https://enkimute.github.io/ganja.js/examples/coffeeshop.html#chapter11_motors
#[derive(Clone, Copy, Debug)]
pub struct Pivot(Line);

impl Pivot {
    // Plucker coordinates convention: (q - p : p cross q) <=> line from p to q
    pub fn from_plucker(d: Vec3, m: Vec3) -> Self {
        Self(Line::new(m.x, m.y, m.z, d.x, d.y, d.z))
    }

    pub fn from_rotation_matrix(matrix: Mat3) -> Self {
        Self::from_plucker(Quat::from_mat3(&matrix).to_scaled_axis(), Vec3::ZERO)
    }

    pub fn from_translation_vector(vector: Vec3) -> Self {
        Self::from_plucker(Vec3::ZERO, vector)
    }

    pub fn zero() -> Self {
        Self::from_plucker(Vec3::ZERO, Vec3::ZERO)
    }

    fn as_motor(&self) -> Motor {
        (self.0 * (-1.0 / 2.0)).exp()
    }

    fn distance(&self, point: Point) -> f32 {
        point.regressive_product(self.0).magnitude()
    }

    fn scale(&self, alpha: f32) -> Self {
        Self(self.0 * alpha)
    }
}

// https://rigidgeometricalgebra.org/wiki/index.php?title=Motor
#[derive(Clone, Debug)]
pub struct PivotalMotion {
    pivots: Vec<Pivot>,
    pre_motor: Motor,
    post_motor: Motor,
}

impl PivotalMotion {
    pub fn from_pivots(pivots: Vec<Pivot>) -> Self {
        Self {
            pivots,
            pre_motor: Pivot::zero().as_motor(),
            post_motor: Pivot::zero().as_motor(),
        }
    }

    pub fn target(&self) -> Mat4 {
        Self::matrix_from_motor(
            self.pivots
                .iter()
                .fold(self.post_motor, |motor, pivot| {
                    motor.geometric_product(pivot.as_motor())
                })
                .geometric_product(self.pre_motor),
        )
    }

    pub fn pivotal_local_transform(self, pivot: Pivot) -> Self {
        Self {
            pivots: self.pivots,
            pre_motor: pivot.as_motor().geometric_product(self.pre_motor),
            post_motor: self.post_motor,
        }
    }

    pub fn pivotal_global_transform(self, pivot: Pivot) -> Self {
        Self {
            pivots: self.pivots,
            pre_motor: self.pre_motor,
            post_motor: pivot.as_motor().geometric_product(self.post_motor),
        }
    }

    pub fn rewind(self) -> Self {
        Self {
            pivots: self
                .pivots
                .iter()
                .rev()
                .map(|pivot| pivot.scale(-1.0))
                .collect(),
            pre_motor: self.pre_motor,
            post_motor: self.pivots.iter().fold(self.post_motor, |motor, pivot| {
                motor.geometric_product(pivot.as_motor())
            }),
        }
    }

    fn matrix_from_motor(motor: Motor) -> Mat4 {
        let x_axis = motor.transformation(Point::new(0.0, 1.0, 0.0, 0.0));
        let y_axis = motor.transformation(Point::new(0.0, 0.0, 1.0, 0.0));
        let z_axis = motor.transformation(Point::new(0.0, 0.0, 0.0, 1.0));
        let w_axis = motor
            .transformation(Point::new(1.0, 0.0, 0.0, 0.0))
            .signum();
        Mat4::from_cols_array_2d(&[
            [x_axis[1], x_axis[2], x_axis[3], x_axis[0]],
            [y_axis[1], y_axis[2], y_axis[3], y_axis[0]],
            [z_axis[1], z_axis[2], z_axis[3], z_axis[0]],
            [w_axis[1], w_axis[2], w_axis[3], w_axis[0]],
        ])
    }
}

#[derive(Clone, Debug)]
pub struct PivotalMotionTrajectory(Vec<(Pivot, Motor, Motor, f32)>);

impl PivotalMotionTrajectory {
    pub fn from_pivotal_motions(pivotal_motions: Vec<PivotalMotion>) -> Self {
        Self(
            pivotal_motions
                .into_iter()
                .flat_map(|pivotal_motion| {
                    let point = pivotal_motion
                        .pre_motor
                        .transformation(Point::new(1.0, 0.0, 0.0, 0.0))
                        .signum();
                    pivotal_motion.pivots.into_iter().scan(
                        pivotal_motion.post_motor,
                        move |motor_state, pivot| {
                            let post_motor = *motor_state;
                            let distance = pivot.distance(point);
                            *motor_state = post_motor.geometric_product(pivot.as_motor());
                            Some((
                                pivot.scale(1.0 / distance),
                                pivotal_motion.pre_motor,
                                post_motor,
                                distance,
                            ))
                        },
                    )
                })
                .collect::<Vec<_>>()
                .into_iter()
                .rev()
                .collect(),
        )
    }

    pub fn consume_distance(&mut self, consumed_distance: f32) -> Option<Mat4> {
        let (pivot, pre_motor, post_motor, distance) = self.0.pop()?;
        (consumed_distance <= distance)
            .then(|| {
                let next_post_motor =
                    post_motor.geometric_product(pivot.scale(consumed_distance).as_motor());
                self.0.push((
                    pivot,
                    pre_motor,
                    next_post_motor,
                    distance - consumed_distance,
                ));
                PivotalMotion::matrix_from_motor(next_post_motor.geometric_product(pre_motor))
            })
            .or_else(|| self.consume_distance(consumed_distance - distance))
    }
}
