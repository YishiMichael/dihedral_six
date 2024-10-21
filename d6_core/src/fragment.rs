use std::collections::HashMap;

use glam::Mat4;
use glam::Vec2;
use glam::Vec3;
use itertools::Itertools;

use super::d6::AxisSystem;
use super::polygon::Polygon;
use super::polygon::Polygons;

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum TileFragment {
    TriangleXFore,
    TriangleXRear,
    TriangleYFore,
    TriangleYRear,
    TriangleZForeLeft,
    TriangleZForeRight,
    TriangleZSideLeft,
    TriangleZSideRight,
    TriangleZRearLeft,
    TriangleZRearRight,
    LadderMajorFace,
    LadderMajorBulkSide,
    LadderMajorCompSide,
    LadderMinorFace,
    LadderMinorBulkSide,
    LadderMinorCompSide,
    ArchMajorFace,
    ArchMajorBulkSide,
    ArchMajorCompSide,
    ArchMinorFace,
    ArchMinorBulkSide,
    ArchMinorCompSide,
}

fn iter_ladder_coords() -> impl Iterator<Item = Vec2> {
    const RESOLUTION: usize = 4;
    (RESOLUTION..=0)
        .flat_map(|i| itertools::repeat_n(i, 2))
        .zip(
            (0..=RESOLUTION)
                .flat_map(|i| itertools::repeat_n(i, 2))
                .skip(1),
        )
        .map(|(i, j)| {
            Vec2::new(i as f32 / RESOLUTION as f32, j as f32 / RESOLUTION as f32) * 2.0 - 1.0
        })
}

fn iter_arch_coords() -> impl Iterator<Item = Vec2> {
    const RESOLUTION: usize = 16;
    (0..=RESOLUTION).map(|i| {
        let (s, c) = (i as f32 / RESOLUTION as f32 * std::f32::consts::FRAC_PI_2).sin_cos();
        Vec2::new(c, s) * 2.0 - 1.0
    })
}

fn face_polygons(coords_iter: impl Iterator<Item = Vec2>) -> Polygons {
    Polygons(
        coords_iter
            .tuple_windows()
            .map(|(prev, next)| Polygon {
                vertices: Vec::from([
                    Vec3::from((prev, -1.0)),
                    Vec3::from((prev, 1.0)),
                    Vec3::from((next, 1.0)),
                    Vec3::from((next, -1.0)),
                ]),
                normal: Vec3::new(prev.y - next.y, next.x - prev.x, 0.0),
            })
            .collect(),
    )
}

fn bulk_side_polygons(coords_iter: impl Iterator<Item = Vec2>) -> Polygons {
    Polygons(
        std::iter::once(Polygon {
            vertices: std::iter::once(Vec2::new(-1.0, -1.0))
                .chain(coords_iter)
                .map(|coord| Vec3::from((coord, 1.0)))
                .collect(),
            normal: Vec3::new(0.0, 0.0, 1.0),
        })
        .collect(),
    )
}

fn comp_side_polygons(coords_iter: impl Iterator<Item = Vec2>) -> Polygons {
    Polygons(
        std::iter::once(Polygon {
            vertices: std::iter::once(Vec2::new(1.0, 1.0))
                .chain(coords_iter.collect::<Vec<_>>().into_iter().rev())
                .map(|coord| Vec3::from((coord, -1.0)))
                .collect(),
            normal: Vec3::new(0.0, 0.0, 1.0),
        })
        .collect(),
    )
}

fn triangle_polygons() -> Polygons {
    Polygons(
        std::iter::once(Polygon {
            vertices: Vec::from([
                Vec3::new(-1.0, -1.0, 0.0),
                Vec3::new(1.0, -1.0, 0.0),
                Vec3::new(-1.0, 1.0, 0.0),
            ]),
            normal: Vec3::new(0.0, 0.0, 1.0),
        })
        .collect(),
    )
}

lazy_static::lazy_static! {
    pub static ref POLYGONS_DICT: HashMap<TileFragment, Polygons> = map_macro::hash_map! {
        TileFragment::TriangleXFore => triangle_polygons().transform(
            Mat4::from_translation(Vec3::new(0.0, 2.0, 0.0)) * Mat4::from_mat3(AxisSystem::NegZPosYPosX.into_mat3()),
        ),
        TileFragment::TriangleXRear => triangle_polygons().transform(
            Mat4::from_translation(Vec3::new(0.0, -2.0, 0.0)) * Mat4::from_mat3(AxisSystem::PosZNegYPosX.into_mat3()),
        ),
        TileFragment::TriangleYFore => triangle_polygons().transform(
            Mat4::from_translation(Vec3::new(2.0, 0.0, 0.0)) * Mat4::from_mat3(AxisSystem::PosXNegZPosY.into_mat3()),
        ),
        TileFragment::TriangleYRear => triangle_polygons().transform(
            Mat4::from_translation(Vec3::new(-2.0, 0.0, 0.0)) * Mat4::from_mat3(AxisSystem::NegXPosZPosY.into_mat3()),
        ),
        TileFragment::TriangleZForeLeft => triangle_polygons().transform(
            Mat4::from_translation(Vec3::new(1.0, 1.0, 0.0)) * Mat4::from_mat3(AxisSystem::PosYNegXPosZ.into_mat3()),
        ),
        TileFragment::TriangleZForeRight => triangle_polygons().transform(
            Mat4::from_translation(Vec3::new(1.0, 1.0, 0.0)) * Mat4::from_mat3(AxisSystem::NegYPosXPosZ.into_mat3()),
        ),
        TileFragment::TriangleZSideLeft => triangle_polygons().transform(
            Mat4::from_translation(Vec3::new(1.0, -1.0, 0.0)) * Mat4::from_mat3(AxisSystem::NegYPosXPosZ.into_mat3()),
        ),
        TileFragment::TriangleZSideRight => triangle_polygons().transform(
            Mat4::from_translation(Vec3::new(-1.0, 1.0, 0.0)) * Mat4::from_mat3(AxisSystem::PosYNegXPosZ.into_mat3()),
        ),
        TileFragment::TriangleZRearLeft => triangle_polygons().transform(
            Mat4::from_translation(Vec3::new(-1.0, -1.0, 0.0)) * Mat4::from_mat3(AxisSystem::PosYNegXPosZ.into_mat3()),
        ),
        TileFragment::TriangleZRearRight => triangle_polygons().transform(
            Mat4::from_translation(Vec3::new(-1.0, -1.0, 0.0)) * Mat4::from_mat3(AxisSystem::NegYPosXPosZ.into_mat3()),
        ),
        TileFragment::LadderMajorFace => face_polygons(iter_ladder_coords()).transform(
            Mat4::from_mat3(AxisSystem::NegXNegYPosZ.into_mat3()),
        ),
        TileFragment::LadderMajorBulkSide => bulk_side_polygons(iter_ladder_coords()).transform(
            Mat4::from_mat3(AxisSystem::NegXNegYPosZ.into_mat3()),
        ),
        TileFragment::LadderMajorCompSide => comp_side_polygons(iter_ladder_coords()).transform(
            Mat4::from_mat3(AxisSystem::NegXNegYPosZ.into_mat3()),
        ),
        TileFragment::LadderMinorFace => face_polygons(iter_ladder_coords()).transform(
            Mat4::from_mat3(AxisSystem::PosYNegXPosZ.into_mat3()),
        ),
        TileFragment::LadderMinorBulkSide => bulk_side_polygons(iter_ladder_coords()).transform(
            Mat4::from_mat3(AxisSystem::PosYNegXPosZ.into_mat3()),
        ),
        TileFragment::LadderMinorCompSide => comp_side_polygons(iter_ladder_coords()).transform(
            Mat4::from_mat3(AxisSystem::PosYNegXPosZ.into_mat3()),
        ),
        TileFragment::ArchMajorFace => face_polygons(iter_arch_coords()).transform(
            Mat4::from_mat3(AxisSystem::NegXNegYPosZ.into_mat3()),
        ),
        TileFragment::ArchMajorBulkSide => bulk_side_polygons(iter_arch_coords()).transform(
            Mat4::from_mat3(AxisSystem::NegXNegYPosZ.into_mat3()),
        ),
        TileFragment::ArchMajorCompSide => comp_side_polygons(iter_arch_coords()).transform(
            Mat4::from_mat3(AxisSystem::NegXNegYPosZ.into_mat3()),
        ),
        TileFragment::ArchMinorFace => face_polygons(iter_arch_coords()).transform(
            Mat4::from_mat3(AxisSystem::PosYNegXPosZ.into_mat3()),
        ),
        TileFragment::ArchMinorBulkSide => bulk_side_polygons(iter_arch_coords()).transform(
            Mat4::from_mat3(AxisSystem::PosYNegXPosZ.into_mat3()),
        ),
        TileFragment::ArchMinorCompSide => comp_side_polygons(iter_arch_coords()).transform(
            Mat4::from_mat3(AxisSystem::PosYNegXPosZ.into_mat3()),
        ),
    };
}
