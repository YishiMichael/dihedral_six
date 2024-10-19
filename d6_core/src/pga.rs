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
    motor: Motor,
}

impl PivotalMotion {
    pub fn from_pivots(pivots: Vec<Pivot>) -> Self {
        Self {
            pivots,
            motor: Pivot::zero().as_motor(),
        }
    }

    // pub fn initial_motor(&self) -> Motor {
    //     self.motor
    // }

    pub fn target(&self) -> Mat4 {
        Self::matrix_from_motor(self.pivots.iter().fold(self.motor, |motor, pivot| {
            motor.geometric_product(pivot.as_motor())
        }))
    }

    pub fn pivotal_transform(self, pivot: Pivot) -> Self {
        Self {
            pivots: self.pivots,
            motor: pivot.as_motor().geometric_product(self.motor),
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
            motor: self.pivots.iter().fold(self.motor, |motor, pivot| {
                motor.geometric_product(pivot.as_motor())
            }),
        }
    }

    fn matrix_from_motor(motor: Motor) -> Mat4 {
        let x_axis = motor.transformation(Point::new(0.0, 1.0, 0.0, 0.0));
        let y_axis = motor.transformation(Point::new(0.0, 0.0, 1.0, 0.0));
        let z_axis = motor.transformation(Point::new(0.0, 0.0, 0.0, 1.0));
        let w_axis = motor
            .transformation(Point::new(1.0, 0.0, 0.0, 1.0))
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
pub struct PivotalMotionPath(Vec<(Pivot, Motor, f32)>);

impl PivotalMotionPath {
    pub fn from_pivotal_motions(pivotal_motions: Vec<PivotalMotion>) -> Self {
        Self(
            pivotal_motions
                .into_iter()
                .flat_map(|pivotal_motion| {
                    pivotal_motion.pivots.into_iter().scan(
                        pivotal_motion.motor,
                        move |motor_state, pivot| {
                            let motor = *motor_state;
                            let distance = pivot.distance(
                                motor
                                    .transformation(Point::new(1.0, 0.0, 0.0, 0.0))
                                    .signum(),
                            );
                            *motor_state = motor.geometric_product(pivot.as_motor());
                            Some((pivot.scale(1.0 / distance), motor, distance))
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
        let (pivot, motor, distance) = self.0.pop()?;
        (consumed_distance <= distance)
            .then(|| {
                let next_motor = motor.geometric_product(pivot.scale(consumed_distance).as_motor());
                self.0
                    .push((pivot, next_motor, distance - consumed_distance));
                PivotalMotion::matrix_from_motor(next_motor)
            })
            .or_else(|| self.consume_distance(consumed_distance - distance))
    }

    // pub fn initial_motor(&self) -> Motor {
    //     self.0
    //         .last()
    //         .map(|(_, motor, _)| motor.clone())
    //         .unwrap_or_else(|| Pivot::zero().as_motor())
    // }

    // pub fn terminal_motor(&self) -> Motor {
    //     self.0
    //         .first()
    //         .map(|(pivot, motor, distance)| {
    //             motor.geometric_product(pivot.scale(*distance).as_motor())
    //         })
    //         .unwrap_or_else(|| Pivot::zero().as_motor())
    // }

    // pub fn from_successive_pivots(initial_motor: Motor, pivots: Vec<Pivot>) -> Self {
    //     Self {
    //         knots: std::iter::once((initial_motor, 0.0))
    //             .chain(
    //                 pivots
    //                     .iter()
    //                     .scan((initial_motor, 0.0), |(motor, distance), pivot| {
    //                         *distance += pivot.distance(
    //                             motor
    //                                 .transformation(Point::new(1.0, 0.0, 0.0, 0.0))
    //                                 .signum(),
    //                         );
    //                         *motor = pivot.as_motor().geometric_product(*motor);
    //                         Some((*motor, *distance))
    //                     }),
    //             )
    //             .collect(),
    //         pivots,
    //     }
    // }

    // pub fn rewind(self) -> Self {
    //     let distance = self
    //         .knots
    //         .first()
    //         .zip(self.knots.last())
    //         .map_or(0.0, |((_, first_distance), (_, last_distance))| {
    //             first_distance + last_distance
    //         });
    //     Self {
    //         knots: self
    //             .knots
    //             .into_iter()
    //             .rev()
    //             .map(|(knot_motor, knot_distance)| (knot_motor, distance - knot_distance))
    //             .collect(),
    //         pivots: self
    //             .pivots
    //             .into_iter()
    //             .rev()
    //             .map(|pivot| pivot.rewind())
    //             .collect(),
    //     }
    // }

    // pub fn global_transform(self, motor: Motor) -> Self {
    //     Self {
    //         knots: self
    //             .knots
    //             .into_iter()
    //             .map(|(knot_motor, knot_distance)| {
    //                 (motor.geometric_product(knot_motor), knot_distance)
    //             })
    //             .collect(),
    //         pivots: self.pivots,
    //     }
    // }

    // pub fn local_transform(self, motor: Motor) -> Self {
    //     Self {
    //         knots: self
    //             .knots
    //             .into_iter()
    //             .map(|(knot_motor, knot_distance)| {
    //                 (knot_motor.geometric_product(motor), knot_distance)
    //             })
    //             .collect(),
    //         pivots: self.pivots,
    //     }
    // }

    // pub fn chain(self, other: Self) -> Self {
    //     let distance = self
    //         .knots
    //         .first()
    //         .zip(self.knots.last())
    //         .map_or(0.0, |((_, first_distance), (_, last_distance))| {
    //             first_distance + last_distance
    //         });
    //     let intermediate_pivot = self.knots.last().zip(other.knots.first()).map_or(
    //         Pivot::zero(),
    //         |((initial_motor, _), (terminal_motor, _))| {
    //             Pivot(terminal_motor.geometric_quotient(*initial_motor).ln())
    //         },
    //     );
    //     Self {
    //         knots: self
    //             .knots
    //             .into_iter()
    //             .chain(
    //                 other
    //                     .knots
    //                     .into_iter()
    //                     .map(|(knot_pivot, knot_distance)| (knot_pivot, distance + knot_distance)),
    //             )
    //             .collect(),
    //         pivots: self
    //             .pivots
    //             .into_iter()
    //             .chain(std::iter::once(intermediate_pivot))
    //             .chain(other.pivots.into_iter())
    //             .collect(),
    //     }
    // }

    // pub fn initial_motor(&self) -> Motor {
    //     self.knots.first().unwrap().0.clone()
    // }

    // pub fn terminal_motor(&self) -> Motor {
    //     self.knots.last().unwrap().0.clone()
    // }
}
