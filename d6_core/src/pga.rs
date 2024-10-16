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

#[derive(Clone, Copy, Debug)]
pub struct Pivot(Line);

impl Pivot {
    pub fn from_plucker(d: Vec3, m: Vec3) -> Self {
        Self(Line::new(d.x, d.y, d.z, m.x, m.y, m.z) * (1.0 / 2.0))
    }

    pub fn from_translation_vector(vector: Vec3) -> Self {
        Self::from_plucker(vector, Vec3::ZERO)
    }

    pub fn from_rotation_matrix(matrix: Mat3) -> Self {
        Self::from_plucker(Vec3::ZERO, Quat::from_mat3(&matrix).to_scaled_axis())
    }

    pub fn zero() -> Self {
        Self::from_plucker(Vec3::ZERO, Vec3::ZERO)
    }

    pub fn as_motor(&self) -> Motor {
        self.0.exp()
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

    pub fn pre_transform(self, motor: Motor) -> Self {
        Self {
            pivots: self.pivots,
            pre_motor: self.pre_motor.geometric_product(motor),
            post_motor: self.post_motor,
        }
    }

    pub fn post_transform(self, motor: Motor) -> Self {
        Self {
            pivots: self.pivots,
            pre_motor: self.pre_motor,
            post_motor: motor.geometric_product(self.post_motor),
        }
    }

    pub fn rewind(self) -> Self {
        Self {
            pivots: self
                .pivots
                .into_iter()
                .rev()
                .map(|pivot| pivot.scale(-1.0))
                .collect(),
            pre_motor: self.pre_motor,
            post_motor: self.post_motor,
        }
    }

    pub fn matrix_from_motor(motor: Motor) -> Mat4 {
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
pub struct PivotalMotionPath(Vec<(Pivot, Motor, Motor, f32)>);

impl PivotalMotionPath {
    pub fn from_pivotal_motions(pivotal_motions: Vec<PivotalMotion>) -> Self {
        Self(
            pivotal_motions
                .into_iter()
                .flat_map(|pivotal_motion| {
                    pivotal_motion.pivots.into_iter().scan(
                        pivotal_motion.pre_motor,
                        move |motor, pivot| {
                            let pre_motor = *motor;
                            let distance = pivot.distance(
                                pre_motor
                                    .transformation(Point::new(1.0, 0.0, 0.0, 0.0))
                                    .signum(),
                            );
                            *motor = pivot.as_motor().geometric_product(pre_motor);
                            Some((
                                pivot.scale(1.0 / distance),
                                pre_motor,
                                pivotal_motion.post_motor,
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

    pub fn consume_distance(&mut self, consumed_distance: f32) -> Option<Motor> {
        let (pivot, pre_motor, post_motor, distance) = self.0.pop()?;
        (consumed_distance <= distance)
            .then(|| {
                let next_pre_motor = pivot
                    .scale(consumed_distance)
                    .as_motor()
                    .geometric_product(pre_motor);
                self.0.push((
                    pivot,
                    next_pre_motor,
                    post_motor,
                    distance - consumed_distance,
                ));
                post_motor.geometric_product(next_pre_motor)
            })
            .or_else(|| self.consume_distance(consumed_distance - distance))
    }

    pub fn initial_motor(&self) -> Motor {
        self.0
            .last()
            .map(|(_, pre_motor, post_motor, _)| post_motor.geometric_product(*pre_motor))
            .unwrap_or_else(|| Pivot::zero().as_motor())
    }

    pub fn terminal_motor(&self) -> Motor {
        self.0
            .first()
            .map(|(pivot, pre_motor, post_motor, distance)| {
                post_motor.geometric_product(
                    pivot
                        .scale(*distance)
                        .as_motor()
                        .geometric_product(*pre_motor),
                )
            })
            .unwrap_or_else(|| Pivot::zero().as_motor())
    }

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
