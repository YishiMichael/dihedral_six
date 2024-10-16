use std::collections::HashMap;
use std::collections::HashSet;

use geometric_algebra::ppga3d::Motor;
use geometric_algebra::GeometricProduct;
use glam::I16Vec3;
use glam::Mat3;
use glam::Mat4;
use glam::Vec2;
use glam::Vec3;
use glam::Vec3Swizzles;

use super::d6::AxisSystem;
use super::d6::Direction;
use super::d6::D6;
use super::fragment::TileFragment;
use super::fragment::POLYGONS_DICT;
use super::pga::Pivot;
use super::pga::PivotalMotion;
use super::pga::PivotalMotionPath;
use super::polygon::Polygons;
use super::polygon::FRAME_POLYGONS;
use super::polygon::MARKER_POLYGONS;
use super::polygon::PLAYER_POLYGONS;

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
enum TileInternalAnchorPositionAxis {
    PlaneForeZ,
    PlaneRearZ,
    LadderMajorFaceX,
    LadderMajorFaceY,
    LadderMinorFaceX,
    LadderMinorFaceY,
    ArchMajorFaceXY,
    ArchMinorFaceXY,
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
enum TileExternalAnchorPosition {
    ForeLeft,
    ForeRight,
    SideLeft,
    SideRight,
    RearLeft,
    RearRight,
}

impl TileExternalAnchorPosition {
    fn into_vec3(self) -> Vec3 {
        match self {
            Self::ForeLeft => Vec3::new(2.0, 1.0, 0.0),
            Self::ForeRight => Vec3::new(1.0, 2.0, 0.0),
            Self::SideLeft => Vec3::new(1.0, -1.0, 0.0),
            Self::SideRight => Vec3::new(-1.0, 1.0, 0.0),
            Self::RearLeft => Vec3::new(-1.0, -2.0, 0.0),
            Self::RearRight => Vec3::new(-2.0, -1.0, 0.0),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
enum TileExternalAnchorAxis {
    X,
    Y,
    Z,
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
enum TileAnchorPositionAxis {
    Internal(TileInternalAnchorPositionAxis),
    External(TileExternalAnchorPosition, TileExternalAnchorAxis),
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
enum TileAnchorSign {
    Pos,
    Neg,
}

impl TileAnchorSign {
    fn flip(self) -> Self {
        match self {
            Self::Pos => Self::Neg,
            Self::Neg => Self::Pos,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
struct TileAnchor {
    position_axis: TileAnchorPositionAxis,
    sign: TileAnchorSign,
    stationery: bool,
}

impl TileAnchor {
    fn flip(self) -> Self {
        Self {
            position_axis: self.position_axis,
            sign: self.sign.flip(),
            stationery: self.stationery,
        }
    }

    fn act(self, action: D6) -> Self {
        #[rustfmt::skip]
        const TILE_EXTERNAL_ANCHOR_POSITION_ACTION_TABLE: [[TileExternalAnchorPosition; 6]; 12] = {
            use TileExternalAnchorPosition as Z6;
            [
                [Z6::ForeLeft, Z6::ForeRight, Z6::SideLeft, Z6::SideRight, Z6::RearLeft, Z6::RearRight],
                [Z6::ForeRight, Z6::SideRight, Z6::ForeLeft, Z6::RearRight, Z6::SideLeft, Z6::RearLeft],
                [Z6::SideRight, Z6::RearRight, Z6::ForeRight, Z6::RearLeft, Z6::ForeLeft, Z6::SideLeft],
                [Z6::RearRight, Z6::RearLeft, Z6::SideRight, Z6::SideLeft, Z6::ForeRight, Z6::ForeLeft],
                [Z6::RearLeft, Z6::SideLeft, Z6::RearRight, Z6::ForeLeft, Z6::SideRight, Z6::ForeRight],
                [Z6::SideLeft, Z6::ForeLeft, Z6::RearLeft, Z6::ForeRight, Z6::RearRight, Z6::SideRight],
                [Z6::RearLeft, Z6::RearRight, Z6::SideLeft, Z6::SideRight, Z6::ForeLeft, Z6::ForeRight],
                [Z6::SideLeft, Z6::RearLeft, Z6::ForeLeft, Z6::RearRight, Z6::ForeRight, Z6::SideRight],
                [Z6::ForeLeft, Z6::SideLeft, Z6::ForeRight, Z6::RearLeft, Z6::SideRight, Z6::RearRight],
                [Z6::ForeRight, Z6::ForeLeft, Z6::SideRight, Z6::SideLeft, Z6::RearRight, Z6::RearLeft],
                [Z6::SideRight, Z6::ForeRight, Z6::RearRight, Z6::ForeLeft, Z6::RearLeft, Z6::SideLeft],
                [Z6::RearRight, Z6::SideRight, Z6::RearLeft, Z6::ForeRight, Z6::SideLeft, Z6::ForeLeft],
            ]
        };
        #[rustfmt::skip]
        const DIRECTION_ACTION_TABLE: [[Direction; 6]; 12] = {
            use Direction as Z6;
            [
                [Z6::PosX, Z6::PosY, Z6::PosZ, Z6::NegX, Z6::NegY, Z6::NegZ],
                [Z6::NegZ, Z6::NegX, Z6::NegY, Z6::PosZ, Z6::PosX, Z6::PosY],
                [Z6::PosY, Z6::PosZ, Z6::PosX, Z6::NegY, Z6::NegZ, Z6::NegX],
                [Z6::NegX, Z6::NegY, Z6::NegZ, Z6::PosX, Z6::PosY, Z6::PosZ],
                [Z6::PosZ, Z6::PosX, Z6::PosY, Z6::NegZ, Z6::NegX, Z6::NegY],
                [Z6::NegY, Z6::NegZ, Z6::NegX, Z6::PosY, Z6::PosZ, Z6::PosX],
                [Z6::NegY, Z6::NegX, Z6::NegZ, Z6::PosY, Z6::PosX, Z6::PosZ],
                [Z6::PosX, Z6::PosZ, Z6::PosY, Z6::NegX, Z6::NegZ, Z6::NegY],
                [Z6::NegZ, Z6::NegY, Z6::NegX, Z6::PosZ, Z6::PosY, Z6::PosX],
                [Z6::PosY, Z6::PosX, Z6::PosZ, Z6::NegY, Z6::NegX, Z6::NegZ],
                [Z6::NegX, Z6::NegZ, Z6::NegY, Z6::PosX, Z6::PosZ, Z6::PosY],
                [Z6::PosZ, Z6::PosY, Z6::PosX, Z6::NegZ, Z6::NegY, Z6::NegX],
            ]
        };

        match self {
            Self {
                position_axis: TileAnchorPositionAxis::Internal(position_axis),
                sign,
                stationery,
            } => Self {
                position_axis: TileAnchorPositionAxis::Internal(position_axis),
                sign,
                stationery,
            },
            Self {
                position_axis: TileAnchorPositionAxis::External(external_position, external_axis),
                sign,
                stationery,
            } => {
                let new_external_position = TILE_EXTERNAL_ANCHOR_POSITION_ACTION_TABLE
                    [action as usize][external_position as usize]
                    as TileExternalAnchorPosition;
                let (new_sign, new_external_axis) = (DIRECTION_ACTION_TABLE[action as usize]
                    [Direction::from_tuple((sign, external_axis)) as usize]
                    as Direction)
                    .into_tuple();
                Self {
                    position_axis: TileAnchorPositionAxis::External(
                        new_external_position,
                        new_external_axis,
                    ),
                    sign: new_sign,
                    stationery,
                }
            }
        }
    }
}

impl Direction {
    fn from_tuple(direction_tuple: (TileAnchorSign, TileExternalAnchorAxis)) -> Self {
        match direction_tuple {
            (TileAnchorSign::Pos, TileExternalAnchorAxis::X) => Self::PosX,
            (TileAnchorSign::Pos, TileExternalAnchorAxis::Y) => Self::PosY,
            (TileAnchorSign::Pos, TileExternalAnchorAxis::Z) => Self::PosZ,
            (TileAnchorSign::Neg, TileExternalAnchorAxis::X) => Self::NegX,
            (TileAnchorSign::Neg, TileExternalAnchorAxis::Y) => Self::NegY,
            (TileAnchorSign::Neg, TileExternalAnchorAxis::Z) => Self::NegZ,
        }
    }

    fn into_tuple(self) -> (TileAnchorSign, TileExternalAnchorAxis) {
        match self {
            Self::PosX => (TileAnchorSign::Pos, TileExternalAnchorAxis::X),
            Self::PosY => (TileAnchorSign::Pos, TileExternalAnchorAxis::Y),
            Self::PosZ => (TileAnchorSign::Pos, TileExternalAnchorAxis::Z),
            Self::NegX => (TileAnchorSign::Neg, TileExternalAnchorAxis::X),
            Self::NegY => (TileAnchorSign::Neg, TileExternalAnchorAxis::Y),
            Self::NegZ => (TileAnchorSign::Neg, TileExternalAnchorAxis::Z),
        }
    }
}

#[derive(Clone)]
struct Route {
    initial_anchor: TileAnchor,
    terminal_anchor: TileAnchor,
    pivotal_motion: PivotalMotion,
    fragments_requirement: HashSet<TileFragment>,
}

impl Route {
    fn flip(self) -> Self {
        Self {
            initial_anchor: self.initial_anchor.flip(),
            terminal_anchor: self.terminal_anchor.flip(),
            pivotal_motion: self.pivotal_motion.pre_transform(
                Pivot::from_rotation_matrix(AxisSystem::NegXPosYNegZ.into_mat3()).as_motor(),
            ),
            fragments_requirement: self.fragments_requirement.clone(),
        }
    }

    fn backward(self) -> Self {
        Self {
            initial_anchor: self.terminal_anchor,
            terminal_anchor: self.initial_anchor,
            pivotal_motion: self
                .pivotal_motion
                .pre_transform(
                    Pivot::from_rotation_matrix(AxisSystem::NegXNegYPosZ.into_mat3()).as_motor(),
                )
                .rewind(),
            fragments_requirement: self.fragments_requirement.clone(),
        }
    }

    fn from_external_internal(
        external_position: TileExternalAnchorPosition,
        external_axis_system: AxisSystem,
        internal_position_axis: TileInternalAnchorPositionAxis,
        stem_altitude_slope: f32,
        stem_pivot_angle: f32,
        branch_fragment: Option<TileFragment>,
        stem_fragment: TileFragment,
    ) -> Self {
        let (_, _, z_direction) = external_axis_system.into_triplet();
        let initial_motor = Pivot::from_translation_vector(external_position.into_vec3())
            .as_motor()
            .geometric_product(
                Pivot::from_rotation_matrix(external_axis_system.into_mat3()).as_motor(),
            );
        let angle_cot_angle = (stem_pivot_angle != 0.0)
            .then(|| stem_pivot_angle / stem_pivot_angle.tan())
            .unwrap_or(1.0);
        let stem_pivot = Pivot::from_plucker(
            (angle_cot_angle - stem_pivot_angle * stem_altitude_slope) * Vec3::Y
                + (angle_cot_angle * stem_altitude_slope + stem_pivot_angle) * Vec3::Z,
            stem_pivot_angle * Vec3::X,
        );
        Self {
            initial_anchor: TileAnchor {
                position_axis: TileAnchorPositionAxis::External(
                    external_position,
                    z_direction.into_tuple().1,
                ),
                sign: z_direction.into_tuple().0,
                stationery: branch_fragment.is_some(),
            },
            terminal_anchor: TileAnchor {
                position_axis: TileAnchorPositionAxis::Internal(internal_position_axis),
                sign: TileAnchorSign::Pos,
                stationery: true,
            },
            pivotal_motion: PivotalMotion::from_pivots(
                branch_fragment
                    .is_some()
                    .then(|| Pivot::from_translation_vector(Vec3::Y))
                    .into_iter()
                    .chain(std::iter::once(stem_pivot))
                    .collect(),
            )
            .pre_transform(initial_motor),
            fragments_requirement: branch_fragment
                .into_iter()
                .chain(std::iter::once(stem_fragment))
                .collect(),
        }
    }
}

lazy_static::lazy_static! {
    static ref ROUTE_LIST: Vec<Route> = [
        // plane
        Route::from_external_internal(
            TileExternalAnchorPosition::ForeLeft,
            AxisSystem::PosYNegXPosZ,
            TileInternalAnchorPositionAxis::PlaneForeZ,
            0.0,
            0.0,
            None,
            TileFragment::TriangleZForeLeft,
        ),
        Route::from_external_internal(
            TileExternalAnchorPosition::ForeRight,
            AxisSystem::NegXNegYPosZ,
            TileInternalAnchorPositionAxis::PlaneForeZ,
            0.0,
            0.0,
            None,
            TileFragment::TriangleZForeRight,
        ),
        Route::from_external_internal(
            TileExternalAnchorPosition::SideLeft,
            AxisSystem::PosXPosYPosZ,
            TileInternalAnchorPositionAxis::PlaneForeZ,
            0.0,
            0.0,
            Some(TileFragment::TriangleZSideLeft),
            TileFragment::TriangleZForeLeft,
        ),
        Route::from_external_internal(
            TileExternalAnchorPosition::SideRight,
            AxisSystem::NegYPosXPosZ,
            TileInternalAnchorPositionAxis::PlaneForeZ,
            0.0,
            0.0,
            Some(TileFragment::TriangleZSideRight),
            TileFragment::TriangleZForeRight,
        ),
        Route::from_external_internal(
            TileExternalAnchorPosition::RearLeft,
            AxisSystem::PosXPosYPosZ,
            TileInternalAnchorPositionAxis::PlaneRearZ,
            0.0,
            0.0,
            None,
            TileFragment::TriangleZRearLeft,
        ),
        Route::from_external_internal(
            TileExternalAnchorPosition::RearRight,
            AxisSystem::NegYPosXPosZ,
            TileInternalAnchorPositionAxis::PlaneRearZ,
            0.0,
            0.0,
            None,
            TileFragment::TriangleZRearRight,
        ),
        Route::from_external_internal(
            TileExternalAnchorPosition::SideLeft,
            AxisSystem::PosYNegXPosZ,
            TileInternalAnchorPositionAxis::PlaneRearZ,
            0.0,
            0.0,
            Some(TileFragment::TriangleZSideLeft),
            TileFragment::TriangleZRearLeft,
        ),
        Route::from_external_internal(
            TileExternalAnchorPosition::SideRight,
            AxisSystem::NegXNegYPosZ,
            TileInternalAnchorPositionAxis::PlaneRearZ,
            0.0,
            0.0,
            Some(TileFragment::TriangleZSideRight),
            TileFragment::TriangleZRearRight,
        ),
        // ladder
        Route::from_external_internal(
            TileExternalAnchorPosition::SideLeft,
            AxisSystem::PosZPosYNegX,
            TileInternalAnchorPositionAxis::LadderMajorFaceX,
            1.0,
            0.0,
            None,
            TileFragment::LadderMajorFace,
        ),
        Route::from_external_internal(
            TileExternalAnchorPosition::SideRight,
            AxisSystem::NegZPosXNegY,
            TileInternalAnchorPositionAxis::LadderMajorFaceY,
            1.0,
            0.0,
            None,
            TileFragment::LadderMajorFace,
        ),
        Route::from_external_internal(
            TileExternalAnchorPosition::ForeRight,
            AxisSystem::NegZNegYNegX,
            TileInternalAnchorPositionAxis::LadderMinorFaceX,
            1.0,
            0.0,
            Some(TileFragment::TriangleXFore),
            TileFragment::LadderMinorFace,
        ),
        Route::from_external_internal(
            TileExternalAnchorPosition::ForeRight,
            AxisSystem::PosZPosXPosY,
            TileInternalAnchorPositionAxis::LadderMinorFaceY,
            1.0,
            0.0,
            Some(TileFragment::TriangleYRear),
            TileFragment::LadderMinorFace,
        ),
        Route::from_external_internal(
            TileExternalAnchorPosition::SideRight,
            AxisSystem::PosZNegYPosX,
            TileInternalAnchorPositionAxis::LadderMajorFaceX,
            1.0,
            0.0,
            None,
            TileFragment::LadderMajorFace,
        ),
        Route::from_external_internal(
            TileExternalAnchorPosition::SideLeft,
            AxisSystem::NegZNegXPosY,
            TileInternalAnchorPositionAxis::LadderMajorFaceY,
            1.0,
            0.0,
            None,
            TileFragment::LadderMajorFace,
        ),
        Route::from_external_internal(
            TileExternalAnchorPosition::RearLeft,
            AxisSystem::NegZPosYPosX,
            TileInternalAnchorPositionAxis::LadderMinorFaceX,
            1.0,
            0.0,
            Some(TileFragment::TriangleXRear),
            TileFragment::LadderMinorFace,
        ),
        Route::from_external_internal(
            TileExternalAnchorPosition::RearLeft,
            AxisSystem::PosZNegXNegY,
            TileInternalAnchorPositionAxis::LadderMinorFaceY,
            1.0,
            0.0,
            Some(TileFragment::TriangleYFore),
            TileFragment::LadderMinorFace,
        ),
        // arch
        Route::from_external_internal(
            TileExternalAnchorPosition::SideRight,
            AxisSystem::PosZNegYPosX,
            TileInternalAnchorPositionAxis::ArchMajorFaceXY,
            1.0,
            std::f32::consts::FRAC_PI_4,
            None,
            TileFragment::ArchMajorFace,
        ),
        Route::from_external_internal(
            TileExternalAnchorPosition::SideLeft,
            AxisSystem::NegZNegXPosY,
            TileInternalAnchorPositionAxis::ArchMajorFaceXY,
            1.0,
            std::f32::consts::FRAC_PI_4,
            None,
            TileFragment::ArchMajorFace,
        ),
        Route::from_external_internal(
            TileExternalAnchorPosition::RearLeft,
            AxisSystem::NegZPosYPosX,
            TileInternalAnchorPositionAxis::ArchMinorFaceXY,
            1.0,
            std::f32::consts::FRAC_PI_4,
            Some(TileFragment::TriangleXRear),
            TileFragment::ArchMinorFace,
        ),
        Route::from_external_internal(
            TileExternalAnchorPosition::ForeLeft,
            AxisSystem::PosZNegXNegY,
            TileInternalAnchorPositionAxis::ArchMinorFaceXY,
            1.0,
            std::f32::consts::FRAC_PI_4,
            Some(TileFragment::TriangleYFore),
            TileFragment::ArchMinorFace,
        ),
    ].into_iter().flat_map(|route| [
        route.clone(),
        route.clone().flip(),
        route.clone().backward(),
        route.flip().backward(),
    ]).collect();
}

#[derive(Clone)]
struct Tile {
    fragments: HashSet<TileFragment>,
    action: D6,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MovementState {
    world_coord: I16Vec3,
    anchor: TileAnchor,
}

#[derive(Clone)]
pub struct World {
    tile_dict: HashMap<I16Vec3, Tile>,
    movement_state: MovementState,
    motor: Motor,
}

impl World {
    fn world_coord_as_vec3(world_coord: I16Vec3) -> Vec3 {
        2.0 * world_coord.as_vec3()
    }

    fn rotation_matrix_from_action(action: D6) -> Mat3 {
        const REFLECTION_MATRIX: Mat3 = Mat3::from_cols_array_2d(&[
            [-1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0],
            [2.0 / 3.0, -1.0 / 3.0, 2.0 / 3.0],
            [2.0 / 3.0, 2.0 / 3.0, -1.0 / 3.0],
        ]);
        let (axis_system, reflect) = match action {
            D6::R0 => (AxisSystem::PosXPosYPosZ, false),
            D6::R1 => (AxisSystem::PosZPosXPosY, true),
            D6::R2 => (AxisSystem::PosYPosZPosX, false),
            D6::R3 => (AxisSystem::PosXPosYPosZ, true),
            D6::R4 => (AxisSystem::PosZPosXPosY, false),
            D6::R5 => (AxisSystem::PosYPosZPosX, true),
            D6::S0 => (AxisSystem::NegYNegXNegZ, false),
            D6::S1 => (AxisSystem::NegXNegZNegY, true),
            D6::S2 => (AxisSystem::NegZNegYNegX, false),
            D6::S3 => (AxisSystem::NegYNegXNegZ, true),
            D6::S4 => (AxisSystem::NegXNegZNegY, false),
            D6::S5 => (AxisSystem::NegZNegYNegX, true),
        };
        reflect
            .then_some(REFLECTION_MATRIX)
            .unwrap_or(Mat3::IDENTITY)
            * axis_system.into_mat3()
    }

    fn movement_state_synonym(movement_state: MovementState) -> Option<MovementState> {
        #[rustfmt::skip]
        const COORD_OFFSET_ITEMS: [(TileExternalAnchorPosition, I16Vec3); 6] = [
            (TileExternalAnchorPosition::ForeLeft, I16Vec3 { x: 1, y: 0, z: -1 }),
            (TileExternalAnchorPosition::ForeRight, I16Vec3 { x: 0, y: 1, z: -1 }),
            (TileExternalAnchorPosition::SideLeft, I16Vec3 { x: 1, y: -1, z: 0 }),
            (TileExternalAnchorPosition::SideRight, I16Vec3 { x: -1, y: 1, z: 0 }),
            (TileExternalAnchorPosition::RearLeft, I16Vec3 { x: 0, y: -1, z: 1 }),
            (TileExternalAnchorPosition::RearRight, I16Vec3 { x: -1, y: 0, z: 1 }),
        ];
        match movement_state.anchor.position_axis {
            TileAnchorPositionAxis::Internal(_) => None,
            TileAnchorPositionAxis::External(external_position, external_axis) => {
                let coord_offset = COORD_OFFSET_ITEMS
                    .iter()
                    .find_map(|(position, offset)| {
                        (*position == external_position).then_some(*offset)
                    })
                    .unwrap();
                let external_position = COORD_OFFSET_ITEMS
                    .iter()
                    .find_map(|(position, offset)| (*offset == -coord_offset).then_some(*position))
                    .unwrap();
                Some(MovementState {
                    world_coord: movement_state.world_coord + coord_offset,
                    anchor: TileAnchor {
                        position_axis: TileAnchorPositionAxis::External(
                            external_position,
                            external_axis,
                        ),
                        sign: movement_state.anchor.sign,
                        stationery: movement_state.anchor.stationery,
                    },
                })
            }
        }
    }

    fn iter_possible_next_movement_states_from(
        movement_state: MovementState,
        tile_dict: &HashMap<I16Vec3, Tile>,
    ) -> Box<dyn Iterator<Item = (MovementState, Vec<PivotalMotion>)> + '_> {
        Box::new(
            std::iter::once(movement_state)
                .chain(World::movement_state_synonym(movement_state))
                .flat_map(move |initial_movement_state| {
                    tile_dict
                        .get(&initial_movement_state.world_coord)
                        .into_iter()
                        .flat_map(move |tile| {
                            ROUTE_LIST.iter().filter_map(move |route| {
                                let fragments = &tile.fragments;
                                let action = tile.action;
                                (route.fragments_requirement.is_subset(fragments)
                                    && route.initial_anchor.act(action)
                                        == initial_movement_state.anchor)
                                    .then(|| {
                                        (
                                            MovementState {
                                                world_coord: initial_movement_state.world_coord,
                                                anchor: route.terminal_anchor.act(action),
                                            },
                                            route.pivotal_motion.clone().post_transform(
                                                Pivot::from_translation_vector(
                                                    Self::world_coord_as_vec3(
                                                        initial_movement_state.world_coord,
                                                    ),
                                                )
                                                .as_motor()
                                                .geometric_product(
                                                    Pivot::from_rotation_matrix(
                                                        Self::rotation_matrix_from_action(action),
                                                    )
                                                    .as_motor(),
                                                ),
                                            ),
                                        )
                                    })
                            })
                        })
                })
                .flat_map(|(terminal_movement_state, pivotal_motion)| {
                    (!terminal_movement_state.anchor.stationery)
                        .then(|| {
                            Self::iter_possible_next_movement_states_from(
                                terminal_movement_state,
                                tile_dict,
                            )
                        })
                        .unwrap_or_else(|| {
                            Box::new(std::iter::once((terminal_movement_state, Vec::new())))
                        })
                        .map(move |(terminal_movement_state, pivotal_motions)| {
                            (
                                terminal_movement_state,
                                std::iter::once(pivotal_motion.clone())
                                    .chain(pivotal_motions)
                                    .collect::<Vec<_>>(),
                            )
                        })
                })
                .filter(move |(terminal_movement_state, _)| {
                    std::iter::once(movement_state)
                        .chain(World::movement_state_synonym(movement_state))
                        .all(|initial_movement_state| {
                            initial_movement_state != *terminal_movement_state
                        })
                }),
        )
    }

    fn iter_possible_next_movement_states(
        &self,
    ) -> Box<dyn Iterator<Item = (MovementState, Vec<PivotalMotion>)> + '_> {
        Self::iter_possible_next_movement_states_from(self.movement_state, &self.tile_dict)
    }

    pub fn iter_coords(&self) -> impl Iterator<Item = I16Vec3> + '_ {
        self.tile_dict.keys().cloned()
    }

    fn conformal_transform(vector: Vec3) -> Vec2 {
        // The rotation transforms:
        // normalize((-1, +1, 00)) |-> (1, 0, 0)
        // normalize((-1, -1, +2)) |-> (0, 1, 0)
        // normalize((+1, +1, +1)) |-> (0, 0, 1)
        // This is a unitary matrix, so the inverse is its transpose.
        lazy_static::lazy_static! {
            static ref CONFORMAL_PROJECTION_MATRIX: Mat3 = Mat3::from_cols(
                Vec3::new(-1.0, 1.0, 0.0).normalize(),
                Vec3::new(-1.0, -1.0, 2.0).normalize(),
                Vec3::new(1.0, 1.0, 1.0).normalize(),
            ).transpose();
        }
        CONFORMAL_PROJECTION_MATRIX.mul_vec3(vector).xy()
    }

    fn iter_shapes_from_polygons(polygons: Polygons) -> impl Iterator<Item = (Vec<Vec2>, Vec3)> {
        polygons.0.into_iter().map(|polygon| {
            (
                polygon
                    .vertices
                    .into_iter()
                    .map(|vertex| Self::conformal_transform(vertex))
                    .collect(),
                polygon.normal,
            )
        })
    }

    pub fn iter_tile_fragment_shapes(
        &self,
        coord: I16Vec3,
    ) -> impl Iterator<Item = (Vec<Vec2>, Vec3)> + '_ {
        self.tile_dict
            .get(&coord)
            .into_iter()
            .flat_map(move |tile| &tile.fragments)
            .flat_map(move |tile_fragment| {
                Self::iter_shapes_from_polygons(
                    POLYGONS_DICT
                        .get(tile_fragment)
                        .unwrap()
                        .clone()
                        .transform(Mat4::from_translation(Self::world_coord_as_vec3(coord))),
                )
            })
    }

    pub fn iter_tile_frame_shapes(
        &self,
        coord: I16Vec3,
    ) -> impl Iterator<Item = (Vec<Vec2>, Vec3)> + '_ {
        self.tile_dict.get(&coord).into_iter().flat_map(move |_| {
            Self::iter_shapes_from_polygons(
                FRAME_POLYGONS
                    .clone()
                    .transform(Mat4::from_translation(Self::world_coord_as_vec3(coord))),
            )
        })
    }

    pub fn iter_player_shapes(&self) -> impl Iterator<Item = (Vec<Vec2>, Vec3)> + '_ {
        Self::iter_shapes_from_polygons(
            PLAYER_POLYGONS
                .clone()
                .transform(PivotalMotion::matrix_from_motor(self.motor)),
        )
    }

    pub fn iter_marker_shapes(&self) -> impl Iterator<Item = (Vec<Vec2>, Vec3)> + '_ {
        self.iter_possible_next_movement_states()
            .map(|(_, pivotal_motions)| {
                PivotalMotion::matrix_from_motor(
                    PivotalMotionPath::from_pivotal_motions(pivotal_motions).terminal_motor(),
                )
            })
            .flat_map(|transform| {
                Self::iter_shapes_from_polygons(MARKER_POLYGONS.clone().transform(transform))
            })
    }

    pub fn motion(&mut self, cursor_coord: Vec2) -> Option<PivotalMotionPath> {
        const RADIUS_THRESHOLD: f32 = 1.0;
        const ANGLE_THRESHOLD: f32 = std::f32::consts::FRAC_PI_6;
        self.iter_possible_next_movement_states()
            .filter_map(|(movement_state, pivotal_motions)| {
                let player_coord = Self::conformal_transform(
                    PivotalMotion::matrix_from_motor(self.motor).transform_point3(Vec3::ZERO),
                );
                let player_coord = Some(player_coord).filter(|player_coord| {
                    (cursor_coord - *player_coord).length() > RADIUS_THRESHOLD
                })?;
                let pivotal_motion_path = PivotalMotionPath::from_pivotal_motions(pivotal_motions);
                let target_coord = Self::conformal_transform(
                    PivotalMotion::matrix_from_motor(pivotal_motion_path.terminal_motor())
                        .transform_point3(Vec3::ZERO),
                );
                let abs_angle = (target_coord - player_coord)
                    .angle_to(cursor_coord - player_coord)
                    .abs();
                let abs_angle = Some(abs_angle).filter(|abs_angle| *abs_angle < ANGLE_THRESHOLD)?;
                Some((movement_state, pivotal_motion_path, abs_angle))
            })
            .min_by(|(_, _, abs_angle_0), (_, _, abs_angle_1)| abs_angle_0.total_cmp(abs_angle_1))
            .map(|(movement_state, pivotal_motion_path, _)| {
                self.movement_state = movement_state;
                pivotal_motion_path
            })
    }

    pub fn set_motor(&mut self, motor: Motor) {
        self.motor = motor;
    }
}

lazy_static::lazy_static! {
    pub static ref WORLD_LIST: Vec<World> = vec![
        World {
            tile_dict: map_macro::hash_map! {
                I16Vec3::new(0, 0, 0) => Tile {
                    fragments: map_macro::hash_set! {
                        TileFragment::TriangleZForeLeft,
                        TileFragment::TriangleZForeRight,
                        TileFragment::TriangleZSideLeft,
                        TileFragment::TriangleZSideRight,
                        TileFragment::TriangleZRearLeft,
                        TileFragment::TriangleZRearRight,
                    },
                    action: D6::R0,
                },
                I16Vec3::new(-1, 1, 0) => Tile {
                    fragments: map_macro::hash_set! {
                        TileFragment::TriangleZForeLeft,
                        TileFragment::TriangleZForeRight,
                        TileFragment::TriangleZSideLeft,
                        TileFragment::TriangleZSideRight,
                        TileFragment::TriangleZRearLeft,
                        TileFragment::TriangleZRearRight,
                    },
                    action: D6::R0,
                },
                I16Vec3::new(0, -1, 1) => Tile {
                    fragments: map_macro::hash_set! {
                        TileFragment::TriangleZForeLeft,
                        TileFragment::TriangleZForeRight,
                        TileFragment::TriangleZSideLeft,
                        TileFragment::TriangleZSideRight,
                        TileFragment::TriangleZRearLeft,
                        TileFragment::TriangleZRearRight,
                    },
                    action: D6::R0,
                },
                I16Vec3::new(1, 0, -1) => Tile {
                    fragments: map_macro::hash_set! {
                        TileFragment::TriangleZForeLeft,
                        TileFragment::TriangleZForeRight,
                        TileFragment::TriangleZSideLeft,
                        TileFragment::TriangleZSideRight,
                        TileFragment::TriangleZRearLeft,
                        TileFragment::TriangleZRearRight,
                    },
                    action: D6::R0,
                },
                I16Vec3::new(0, 1, -1) => Tile {
                    fragments: map_macro::hash_set! {
                        TileFragment::TriangleZForeLeft,
                        TileFragment::TriangleZForeRight,
                        TileFragment::TriangleZSideLeft,
                        TileFragment::TriangleZSideRight,
                        TileFragment::TriangleZRearLeft,
                        TileFragment::TriangleZRearRight,
                    },
                    action: D6::R0,
                },
                I16Vec3::new(-1, 0, 1) => Tile {
                    fragments: map_macro::hash_set! {
                        TileFragment::TriangleZForeLeft,
                        TileFragment::TriangleZForeRight,
                        TileFragment::TriangleZSideLeft,
                        TileFragment::TriangleZSideRight,
                        TileFragment::TriangleZRearLeft,
                        TileFragment::TriangleZRearRight,
                    },
                    action: D6::R0,
                },
                I16Vec3::new(1, -1, 0) => Tile {
                    fragments: map_macro::hash_set! {
                        TileFragment::TriangleZForeLeft,
                        TileFragment::TriangleZForeRight,
                        TileFragment::TriangleZSideLeft,
                        TileFragment::TriangleZSideRight,
                        TileFragment::TriangleZRearLeft,
                        TileFragment::TriangleZRearRight,
                    },
                    action: D6::R0,
                },
            },
            movement_state: MovementState {
                world_coord: I16Vec3::new(0, 0, 0),
                anchor: TileAnchor {
                    position_axis: TileAnchorPositionAxis::Internal(
                        TileInternalAnchorPositionAxis::PlaneForeZ,
                    ),
                    sign: TileAnchorSign::Pos,
                    stationery: true,
                },
            },
            motor: Pivot::from_translation_vector(Vec3::new(1.0, 1.0, 0.0)).as_motor(),
        },
        World {
            tile_dict: map_macro::hash_map! {
                I16Vec3::new(0, 0, 0) => Tile {
                    fragments: map_macro::hash_set! {
                        TileFragment::TriangleZSideLeft,
                        TileFragment::TriangleZRearLeft,
                        TileFragment::TriangleZRearRight,
                    },
                    action: D6::R0,
                },
                I16Vec3::new(-1, 0, 1) => Tile {
                    fragments: map_macro::hash_set! {
                        TileFragment::LadderMajorFace
                    },
                    action: D6::R1,
                },
                I16Vec3::new(-2, 0, 2) => Tile {
                    fragments: map_macro::hash_set! {
                        TileFragment::TriangleZForeLeft,
                        TileFragment::TriangleZForeRight,
                        TileFragment::TriangleZSideLeft,
                    },
                    action: D6::R0,
                },
                I16Vec3::new(-1, -1, 2) => Tile {
                    fragments: map_macro::hash_set! {
                        TileFragment::TriangleZSideRight,
                        TileFragment::TriangleZRearLeft,
                        TileFragment::TriangleZRearRight,
                    },
                    action: D6::R0,
                },
                I16Vec3::new(-1, -2, 3) => Tile {
                    fragments: map_macro::hash_set! {
                        TileFragment::TriangleZForeLeft,
                        TileFragment::TriangleZForeRight,
                        TileFragment::TriangleZSideLeft,
                    },
                    action: D6::R0,
                },
                I16Vec3::new(0, -3, 3) => Tile {
                    fragments: map_macro::hash_set! {
                        TileFragment::TriangleXRear,
                        TileFragment::TriangleYFore,
                        TileFragment::ArchMinorFace,
                    },
                    action: D6::R2,
                },
                I16Vec3::new(1, -3, 2) => Tile {
                    fragments: map_macro::hash_set! {
                        TileFragment::TriangleZSideRight,
                        TileFragment::TriangleZRearLeft,
                        TileFragment::TriangleZRearRight,
                    },
                    action: D6::R5,
                },
                I16Vec3::new(1, -2, 1) => Tile {
                    fragments: map_macro::hash_set! {
                        TileFragment::ArchMajorFace,
                    },
                    action: D6::R2,
                },
                I16Vec3::new(1, -1, 0) => Tile {
                    fragments: map_macro::hash_set! {
                        TileFragment::TriangleZSideRight,
                        TileFragment::TriangleZRearLeft,
                        TileFragment::TriangleZRearRight,
                    },
                    action: D6::R0,
                },
            },
            movement_state: MovementState {
                world_coord: I16Vec3::new(0, 0, 0),
                anchor: TileAnchor {
                    position_axis: TileAnchorPositionAxis::Internal(
                        TileInternalAnchorPositionAxis::PlaneForeZ,
                    ),
                    sign: TileAnchorSign::Pos,
                    stationery: true,
                },
            },
            motor: Pivot::from_translation_vector(Vec3::new(1.0, 1.0, 0.0)).as_motor(),
        },
    ];
}

#[test]
fn test() {
    let world = &WORLD_LIST[0];
    world
        .iter_possible_next_movement_states()
        .for_each(|(movement_state, pivotal_motions)| {
            let pivotal_motion_path = PivotalMotionPath::from_pivotal_motions(pivotal_motions);
            dbg!(
                movement_state,
                pivotal_motion_path.initial_motor(),
                pivotal_motion_path.terminal_motor()
            );
        });
}
