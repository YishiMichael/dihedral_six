use std::collections::HashMap;
use std::collections::HashSet;

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
use super::pga::PivotalMotionTrajectory;
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
    fn from_offset(offset: I16Vec3) -> Self {
        match offset {
            I16Vec3 { x: 1, y: 0, z: -1 } => Self::ForeLeft,
            I16Vec3 { x: 0, y: 1, z: -1 } => Self::ForeRight,
            I16Vec3 { x: 1, y: -1, z: 0 } => Self::SideLeft,
            I16Vec3 { x: -1, y: 1, z: 0 } => Self::SideRight,
            I16Vec3 { x: 0, y: -1, z: 1 } => Self::RearLeft,
            I16Vec3 { x: -1, y: 0, z: 1 } => Self::RearRight,
            _ => unreachable!(),
        }
    }

    fn into_offset(self) -> I16Vec3 {
        match self {
            Self::ForeLeft => I16Vec3 { x: 1, y: 0, z: -1 },
            Self::ForeRight => I16Vec3 { x: 0, y: 1, z: -1 },
            Self::SideLeft => I16Vec3 { x: 1, y: -1, z: 0 },
            Self::SideRight => I16Vec3 { x: -1, y: 1, z: 0 },
            Self::RearLeft => I16Vec3 { x: 0, y: -1, z: 1 },
            Self::RearRight => I16Vec3 { x: -1, y: 0, z: 1 },
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

impl std::ops::BitXor<bool> for TileAnchorSign {
    type Output = Self;

    fn bitxor(self, rhs: bool) -> Self::Output {
        match (self, rhs) {
            (Self::Pos, false) | (Self::Neg, true) => Self::Pos,
            (Self::Neg, false) | (Self::Pos, true) => Self::Neg,
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

enum RouteMotionPrimitive {
    Plane,
    PlaneExt,
    Ladder,
    LadderExt,
    Arch,
    ArchExt,
}

impl RouteMotionPrimitive {
    fn pivot_motion(&self, backward: bool, flip: bool) -> PivotalMotion {
        let stem_pivot = {
            let slope = self.slope();
            let angle = self.rotation_angle();
            let angle_cot_angle = (angle != 0.0).then(|| angle / angle.tan()).unwrap_or(1.0);
            Pivot::from_plucker(
                angle * Vec3::X,
                (angle_cot_angle + angle * slope) * Vec3::Y
                    + (angle_cot_angle * slope - angle) * Vec3::Z,
            )
        };
        let branch_pivot = self
            .is_extended()
            .then(|| Pivot::from_translation_vector(Vec3::Y));
        let motion = PivotalMotion::from_pivots(
            branch_pivot
                .into_iter()
                .chain(std::iter::once(stem_pivot))
                .collect(),
        );
        let motion = if backward {
            motion
                .rewind()
                .pivotal_local_transform(Pivot::from_rotation_matrix(
                    AxisSystem::NegXNegYPosZ.into_mat3(),
                ))
        } else {
            motion
        };
        let motion = if flip {
            motion.pivotal_local_transform(Pivot::from_rotation_matrix(
                AxisSystem::NegXPosYNegZ.into_mat3(),
            ))
        } else {
            motion
        };
        motion
    }

    fn slope(&self) -> f32 {
        match self {
            &Self::Plane | &Self::PlaneExt => 0.0,
            &Self::Ladder | &Self::LadderExt | &Self::Arch | &Self::ArchExt => 1.0,
        }
    }

    fn rotation_angle(&self) -> f32 {
        match self {
            &Self::Plane | &Self::PlaneExt | &Self::Ladder | &Self::LadderExt => 0.0,
            &Self::Arch | &Self::ArchExt => std::f32::consts::FRAC_PI_4,
        }
    }

    fn is_extended(&self) -> bool {
        match self {
            &Self::Plane | &Self::Ladder | &Self::Arch => false,
            &Self::PlaneExt | &Self::LadderExt | &Self::ArchExt => true,
        }
    }
}

struct RouteFamilyInfo {
    motion_primitive: RouteMotionPrimitive,
    axis_system: AxisSystem,
    external_position: TileExternalAnchorPosition,
    internal_position_axis: TileInternalAnchorPositionAxis,
    fragments_requirement: &'static [TileFragment],
}

impl RouteFamilyInfo {
    fn route(&self, backward: bool, flip: bool) -> Route {
        let (external_sign, external_axis) = self.axis_system.into_triplet().2.into_tuple();
        let external_anchor = TileAnchor {
            position_axis: TileAnchorPositionAxis::External(self.external_position, external_axis),
            sign: external_sign ^ flip,
            stationery: self.motion_primitive.is_extended(),
        };
        let internal_anchor = TileAnchor {
            position_axis: TileAnchorPositionAxis::Internal(self.internal_position_axis),
            sign: TileAnchorSign::Pos ^ flip,
            stationery: true,
        };
        let (initial_anchor, terminal_anchor) = if backward {
            (internal_anchor, external_anchor)
        } else {
            (external_anchor, internal_anchor)
        };
        let pivotal_motion = self
            .motion_primitive
            .pivot_motion(backward, flip)
            .pivotal_global_transform(Pivot::from_rotation_matrix(self.axis_system.into_mat3()))
            .pivotal_global_transform(Pivot::from_translation_vector(
                self.external_position.into_offset().as_vec3(),
            ));
        Route {
            initial_anchor,
            terminal_anchor,
            pivotal_motion,
            fragments_requirement: self.fragments_requirement.to_vec().into_iter().collect(),
        }
    }
}

struct Route {
    initial_anchor: TileAnchor,
    terminal_anchor: TileAnchor,
    pivotal_motion: PivotalMotion,
    fragments_requirement: HashSet<TileFragment>,
}

#[rustfmt::skip]
static ROUTE_FAMILY_INFO_LIST: &[RouteFamilyInfo] = &[
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::Plane,
        axis_system: AxisSystem::PosYNegXPosZ,
        external_position: TileExternalAnchorPosition::ForeLeft,
        internal_position_axis: TileInternalAnchorPositionAxis::PlaneForeZ,
        fragments_requirement: &[TileFragment::TriangleZForeLeft],
    },
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::Plane,
        axis_system: AxisSystem::NegXNegYPosZ,
        external_position: TileExternalAnchorPosition::ForeRight,
        internal_position_axis: TileInternalAnchorPositionAxis::PlaneForeZ,
        fragments_requirement: &[TileFragment::TriangleZForeRight],
    },
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::Plane,
        axis_system: AxisSystem::PosXPosYPosZ,
        external_position: TileExternalAnchorPosition::RearLeft,
        internal_position_axis: TileInternalAnchorPositionAxis::PlaneRearZ,
        fragments_requirement: &[TileFragment::TriangleZRearLeft],
    },
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::Plane,
        axis_system: AxisSystem::NegYPosXPosZ,
        external_position: TileExternalAnchorPosition::RearRight,
        internal_position_axis: TileInternalAnchorPositionAxis::PlaneRearZ,
        fragments_requirement: &[TileFragment::TriangleZRearRight],
    },
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::PlaneExt,
        axis_system: AxisSystem::PosXPosYPosZ,
        external_position: TileExternalAnchorPosition::SideLeft,
        internal_position_axis: TileInternalAnchorPositionAxis::PlaneForeZ,
        fragments_requirement: &[TileFragment::TriangleZSideLeft, TileFragment::TriangleZForeLeft],
    },
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::PlaneExt,
        axis_system: AxisSystem::NegYPosXPosZ,
        external_position: TileExternalAnchorPosition::SideRight,
        internal_position_axis: TileInternalAnchorPositionAxis::PlaneForeZ,
        fragments_requirement: &[TileFragment::TriangleZSideRight, TileFragment::TriangleZForeRight],
    },
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::PlaneExt,
        axis_system: AxisSystem::PosYNegXPosZ,
        external_position: TileExternalAnchorPosition::SideLeft,
        internal_position_axis: TileInternalAnchorPositionAxis::PlaneRearZ,
        fragments_requirement: &[TileFragment::TriangleZSideLeft, TileFragment::TriangleZRearLeft],
    },
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::PlaneExt,
        axis_system: AxisSystem::NegXNegYPosZ,
        external_position: TileExternalAnchorPosition::SideRight,
        internal_position_axis: TileInternalAnchorPositionAxis::PlaneRearZ,
        fragments_requirement: &[TileFragment::TriangleZSideRight, TileFragment::TriangleZRearRight],
    },
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::Ladder,
        axis_system: AxisSystem::PosZPosYNegX,
        external_position: TileExternalAnchorPosition::SideLeft,
        internal_position_axis: TileInternalAnchorPositionAxis::LadderMajorFaceX,
        fragments_requirement: &[TileFragment::LadderMajorFace],
    },
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::Ladder,
        axis_system: AxisSystem::NegZPosXNegY,
        external_position: TileExternalAnchorPosition::SideRight,
        internal_position_axis: TileInternalAnchorPositionAxis::LadderMajorFaceY,
        fragments_requirement: &[TileFragment::LadderMajorFace],
    },
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::Ladder,
        axis_system: AxisSystem::PosZNegYPosX,
        external_position: TileExternalAnchorPosition::SideRight,
        internal_position_axis: TileInternalAnchorPositionAxis::LadderMajorFaceX,
        fragments_requirement: &[TileFragment::LadderMajorFace],
    },
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::Ladder,
        axis_system: AxisSystem::NegZNegXPosY,
        external_position: TileExternalAnchorPosition::SideLeft,
        internal_position_axis: TileInternalAnchorPositionAxis::LadderMajorFaceY,
        fragments_requirement: &[TileFragment::LadderMajorFace],
    },
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::LadderExt,
        axis_system: AxisSystem::NegZNegYNegX,
        external_position: TileExternalAnchorPosition::ForeRight,
        internal_position_axis: TileInternalAnchorPositionAxis::LadderMinorFaceX,
        fragments_requirement: &[TileFragment::TriangleXFore, TileFragment::LadderMinorFace],
    },
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::LadderExt,
        axis_system: AxisSystem::PosZPosXPosY,
        external_position: TileExternalAnchorPosition::ForeRight,
        internal_position_axis: TileInternalAnchorPositionAxis::LadderMinorFaceY,
        fragments_requirement: &[TileFragment::TriangleYRear, TileFragment::LadderMinorFace],
    },
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::LadderExt,
        axis_system: AxisSystem::NegZPosYPosX,
        external_position: TileExternalAnchorPosition::RearLeft,
        internal_position_axis: TileInternalAnchorPositionAxis::LadderMinorFaceX,
        fragments_requirement: &[TileFragment::TriangleXRear, TileFragment::LadderMinorFace],
    },
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::LadderExt,
        axis_system: AxisSystem::PosZNegXNegY,
        external_position: TileExternalAnchorPosition::RearLeft,
        internal_position_axis: TileInternalAnchorPositionAxis::LadderMinorFaceY,
        fragments_requirement: &[TileFragment::TriangleYFore, TileFragment::LadderMinorFace],
    },
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::Arch,
        axis_system: AxisSystem::PosZNegYPosX,
        external_position: TileExternalAnchorPosition::SideRight,
        internal_position_axis: TileInternalAnchorPositionAxis::ArchMajorFaceXY,
        fragments_requirement: &[TileFragment::ArchMajorFace],
    },
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::Arch,
        axis_system: AxisSystem::NegZNegXPosY,
        external_position: TileExternalAnchorPosition::SideLeft,
        internal_position_axis: TileInternalAnchorPositionAxis::ArchMajorFaceXY,
        fragments_requirement: &[TileFragment::ArchMajorFace],
    },
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::ArchExt,
        axis_system: AxisSystem::NegZPosYPosX,
        external_position: TileExternalAnchorPosition::RearLeft,
        internal_position_axis: TileInternalAnchorPositionAxis::ArchMinorFaceXY,
        fragments_requirement: &[TileFragment::TriangleXRear, TileFragment::ArchMinorFace],
    },
    RouteFamilyInfo {
        motion_primitive: RouteMotionPrimitive::ArchExt,
        axis_system: AxisSystem::PosZNegXNegY,
        external_position: TileExternalAnchorPosition::ForeLeft,
        internal_position_axis: TileInternalAnchorPositionAxis::ArchMinorFaceXY,
        fragments_requirement: &[TileFragment::TriangleYFore, TileFragment::ArchMinorFace],
    },
];

lazy_static::lazy_static! {
    static ref ROUTE_LIST: Vec<Route> = ROUTE_FAMILY_INFO_LIST
        .into_iter()
        .flat_map(|route_family_info| {
            [
                route_family_info.route(false, false),
                route_family_info.route(true, false),
                route_family_info.route(false, true),
                route_family_info.route(true, true),
            ]
        })
        .collect();
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

#[derive(Clone, Debug)]
pub struct MovementTarget {
    movement_state: MovementState,
    transform: Mat4,
    pivotal_motions: Vec<PivotalMotion>,
}

#[derive(Clone)]
pub struct World {
    tile_dict: HashMap<I16Vec3, Tile>,
    movement_state: MovementState,
    player_transform: Mat4,
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
        match movement_state.anchor.position_axis {
            TileAnchorPositionAxis::Internal(_) => None,
            TileAnchorPositionAxis::External(external_position, external_axis) => {
                let coord_offset = external_position.into_offset();
                let external_position = TileExternalAnchorPosition::from_offset(-coord_offset);
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

    fn next_movement_targets_from(
        movement_state: MovementState,
        tile_dict: &HashMap<I16Vec3, Tile>,
    ) -> Vec<MovementTarget> {
        std::iter::once(movement_state)
            .chain(World::movement_state_synonym(movement_state))
            .flat_map(move |initial_movement_state| {
                tile_dict
                    .get(&initial_movement_state.world_coord)
                    .into_iter()
                    .flat_map(move |tile| {
                        ROUTE_LIST.iter().filter_map(move |route| {
                            route
                                .fragments_requirement
                                .is_subset(&tile.fragments)
                                .then_some(())?;
                            let action = tile.action;
                            (route.initial_anchor.act(action) == initial_movement_state.anchor)
                                .then_some(())?;
                            Some((
                                MovementState {
                                    world_coord: initial_movement_state.world_coord,
                                    anchor: route.terminal_anchor.act(action),
                                },
                                route
                                    .pivotal_motion
                                    .clone()
                                    .pivotal_global_transform(Pivot::from_rotation_matrix(
                                        Self::rotation_matrix_from_action(action),
                                    ))
                                    .pivotal_global_transform(Pivot::from_translation_vector(
                                        Self::world_coord_as_vec3(
                                            initial_movement_state.world_coord,
                                        ),
                                    )),
                            ))
                        })
                    })
            })
            .flat_map(|(terminal_movement_state, pivotal_motion)| {
                terminal_movement_state
                    .anchor
                    .stationery
                    .then(|| {
                        std::iter::once(MovementTarget {
                            movement_state: terminal_movement_state,
                            transform: pivotal_motion.target(),
                            pivotal_motions: Vec::new(),
                        })
                        .collect::<Vec<_>>()
                    })
                    .unwrap_or_else(|| {
                        Self::next_movement_targets_from(terminal_movement_state, tile_dict)
                    })
                    .into_iter()
                    .map(move |successive_movement_target| MovementTarget {
                        movement_state: successive_movement_target.movement_state,
                        transform: successive_movement_target.transform,
                        pivotal_motions: std::iter::once(pivotal_motion.clone())
                            .chain(successive_movement_target.pivotal_motions)
                            .collect(),
                    })
            })
            .filter(move |movement_target| {
                std::iter::once(movement_state)
                    .chain(World::movement_state_synonym(movement_state))
                    .all(|initial_movement_state| {
                        initial_movement_state != movement_target.movement_state
                    })
            })
            .collect()
    }

    fn next_movement_targets(&self) -> Vec<MovementTarget> {
        Self::next_movement_targets_from(self.movement_state, &self.tile_dict)
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
        Self::iter_shapes_from_polygons(PLAYER_POLYGONS.clone().transform(self.player_transform))
    }

    pub fn iter_marker_shapes(&self) -> impl Iterator<Item = (Vec<Vec2>, Vec3)> + '_ {
        self.next_movement_targets()
            .into_iter()
            .flat_map(|movement_target| {
                Self::iter_shapes_from_polygons(
                    MARKER_POLYGONS.clone().transform(movement_target.transform),
                )
            })
    }

    pub fn motion(&mut self, cursor_coord: Vec2) -> Option<PivotalMotionTrajectory> {
        const RADIUS_THRESHOLD: f32 = 1.0;
        const ANGLE_THRESHOLD: f32 = std::f32::consts::FRAC_PI_6;
        self.next_movement_targets()
            .into_iter()
            .filter_map(|movement_target| {
                let player_coord =
                    Self::conformal_transform(self.player_transform.transform_point3(Vec3::ZERO));
                ((cursor_coord - player_coord).length() > RADIUS_THRESHOLD).then_some(())?;
                let target_coord = Self::conformal_transform(
                    movement_target.transform.transform_point3(Vec3::ZERO),
                );
                let abs_angle = (target_coord - player_coord)
                    .angle_to(cursor_coord - player_coord)
                    .abs();
                (abs_angle < ANGLE_THRESHOLD).then_some(())?;
                Some((movement_target, abs_angle))
            })
            .min_by(|(_, abs_angle_0), (_, abs_angle_1)| abs_angle_0.total_cmp(abs_angle_1))
            .map(|(movement_target, _)| {
                self.movement_state = movement_target.movement_state;
                PivotalMotionTrajectory::from_pivotal_motions(movement_target.pivotal_motions)
            })
    }

    pub fn set_player_transform(&mut self, player_transform: Mat4) {
        self.player_transform = player_transform;
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
            player_transform: Mat4::from_translation(Vec3::new(1.0, 1.0, 0.0)),
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
            player_transform: Mat4::from_translation(Vec3::new(1.0, 1.0, 0.0)),
        },
    ];
}

#[test]
fn test() {
    let world = &WORLD_LIST[0];
    world
        .next_movement_targets()
        .into_iter()
        // .skip(0)
        // .next()
        .for_each(|movement_target| {
            dbg!(movement_target.clone());
            let mut pivotal_motion_path =
                PivotalMotionTrajectory::from_pivotal_motions(movement_target.pivotal_motions);
            while let Some(transform) = pivotal_motion_path.consume_distance(0.4) {
                dbg!(transform.transform_point3(Vec3::ZERO));
            }
        });
}
