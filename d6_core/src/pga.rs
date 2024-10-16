use geometric_algebra::ppga3d::Line;
use geometric_algebra::ppga3d::Motor;
use geometric_algebra::ppga3d::Point;
use geometric_algebra::Exp;
use geometric_algebra::GeometricProduct;
use geometric_algebra::GeometricQuotient;
use geometric_algebra::Ln;
use geometric_algebra::Magnitude;
use geometric_algebra::RegressiveProduct;
use geometric_algebra::Signum;
use geometric_algebra::Transformation;
use geometric_algebra::Zero;
use glam::Mat3;
use glam::Mat4;
use glam::Quat;
use glam::Vec3;
use itertools::Itertools;

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

    pub fn as_motor(&self) -> Motor {
        self.0.exp()
    }

    fn distance(&self, point: Point) -> f32 {
        point.regressive_product(self.0).magnitude()
    }

    fn rewind(self) -> Self {
        Self(-self.0)
    }

    fn interpolate(&self, alpha: f32) -> Motor {
        (self.0 * alpha).exp()
    }
}

// https://rigidgeometricalgebra.org/wiki/index.php?title=Motor
#[derive(Clone, Debug)]
pub struct MotorInterpolant {
    knots: Vec<(Motor, f32)>,
    pivots: Vec<Pivot>,
}

impl MotorInterpolant {
    pub fn from_successive_pivots(initial_motor: Motor, pivots: Vec<Pivot>) -> Self {
        Self {
            knots: std::iter::once((initial_motor, 0.0))
                .chain(
                    pivots
                        .iter()
                        .scan((initial_motor, 0.0), |(motor, distance), pivot| {
                            *distance += pivot.distance(
                                motor
                                    .transformation(Point::new(1.0, 0.0, 0.0, 0.0))
                                    .signum(),
                            );
                            *motor = pivot.as_motor().geometric_product(*motor);
                            Some((*motor, *distance))
                        }),
                )
                .collect(),
            pivots,
        }
    }

    pub fn rewind(self) -> Self {
        let distance = self
            .knots
            .first()
            .zip(self.knots.last())
            .map_or(0.0, |((_, first_distance), (_, last_distance))| {
                first_distance + last_distance
            });
        Self {
            knots: self
                .knots
                .into_iter()
                .rev()
                .map(|(knot_motor, knot_distance)| (knot_motor, distance - knot_distance))
                .collect(),
            pivots: self
                .pivots
                .into_iter()
                .rev()
                .map(|pivot| pivot.rewind())
                .collect(),
        }
    }

    pub fn global_transform(self, motor: Motor) -> Self {
        Self {
            knots: self
                .knots
                .into_iter()
                .map(|(knot_motor, knot_distance)| {
                    (motor.geometric_product(knot_motor), knot_distance)
                })
                .collect(),
            pivots: self.pivots,
        }
    }

    pub fn local_transform(self, motor: Motor) -> Self {
        Self {
            knots: self
                .knots
                .into_iter()
                .map(|(knot_motor, knot_distance)| {
                    (knot_motor.geometric_product(motor), knot_distance)
                })
                .collect(),
            pivots: self.pivots,
        }
    }

    pub fn chain(self, other: Self) -> Self {
        let distance = self
            .knots
            .first()
            .zip(self.knots.last())
            .map_or(0.0, |((_, first_distance), (_, last_distance))| {
                first_distance + last_distance
            });
        let intermediate_pivot = Pivot(self.knots.last().zip(other.knots.first()).map_or(
            Line::zero(),
            |((initial_motor, _), (terminal_motor, _))| {
                terminal_motor.geometric_quotient(*initial_motor).ln()
            },
        ));
        Self {
            knots: self
                .knots
                .into_iter()
                .chain(
                    other
                        .knots
                        .into_iter()
                        .map(|(knot_pivot, knot_distance)| (knot_pivot, distance + knot_distance)),
                )
                .collect(),
            pivots: self
                .pivots
                .into_iter()
                .chain(std::iter::once(intermediate_pivot))
                .chain(other.pivots.into_iter())
                .collect(),
        }
    }

    pub fn interpolate(&self, distance: f32) -> Option<Motor> {
        self.knots
            .iter()
            .tuple_windows::<(_, _)>()
            .zip_eq(self.pivots.as_slice())
            .find_map(
                |(((initial_motor, initial_distance), (_, terminal_distance)), pivot)| {
                    (distance < *terminal_distance).then(|| {
                        pivot
                            .interpolate(
                                (distance - initial_distance)
                                    / (terminal_distance - initial_distance),
                            )
                            .geometric_product(*initial_motor)
                    })
                },
            )
    }

    pub fn initial_motor(&self) -> Motor {
        self.knots.first().unwrap().0.clone()
    }

    pub fn terminal_motor(&self) -> Motor {
        self.knots.last().unwrap().0.clone()
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
