use glam::Mat4;
use glam::Vec3;

#[derive(Clone)]
pub struct Polygon {
    pub vertices: Vec<Vec3>,
    pub normal: Vec3,
}

impl Polygon {
    fn transform(self, matrix: Mat4) -> Self {
        Self {
            vertices: self
                .vertices
                .into_iter()
                .map(|vertex| matrix.transform_point3(vertex))
                .collect(),
            normal: matrix.transform_vector3(self.normal),
        }
    }
}

#[derive(Clone)]
pub struct Polygons(pub Vec<Polygon>);

impl Polygons {
    pub fn transform(self, transform: Mat4) -> Self {
        Self(
            self.0
                .into_iter()
                .map(|polygon| polygon.transform(transform))
                .collect(),
        )
    }
}

lazy_static::lazy_static! {
    pub static ref PLAYER_POLYGONS: Polygons = Polygons(Vec::from([
        Polygon {
            vertices: Vec::from([
                Vec3::new(0.0, -0.6, -1.0),
                Vec3::new(0.0, 0.6, -1.0),
                Vec3::new(0.0, 0.0, 0.6),
            ]),
            normal: Vec3::new(1.0, 0.0, 0.0),
        },
        Polygon {
            vertices: Vec::from([
                Vec3::new(-0.6, 0.0, -1.0),
                Vec3::new(0.6, 0.0, -1.0),
                Vec3::new(0.0, 0.0, 0.6),
            ]),
            normal: Vec3::new(0.0, 1.0, 0.0),
        },
        Polygon {
            vertices: Vec::from([
                Vec3::new(-0.4, -0.7, 0.6),
                Vec3::new(-0.4, -0.7, 1.4),
                Vec3::new(-0.4, 0.7, 1.4),
                Vec3::new(-0.4, 0.7, 0.6),
            ]),
            normal: Vec3::new(1.0, 0.0, 0.0),
        },
        Polygon {
            vertices: Vec::from([
                Vec3::new(0.4, -0.7, 0.6),
                Vec3::new(0.4, -0.7, 1.4),
                Vec3::new(0.4, 0.7, 1.4),
                Vec3::new(0.4, 0.7, 0.6),
            ]),
            normal: Vec3::new(1.0, 0.0, 0.0),
        },
        Polygon {
            vertices: Vec::from([
                Vec3::new(-0.4, -0.7, 0.6),
                Vec3::new(-0.4, -0.7, 1.4),
                Vec3::new(0.4, -0.7, 1.4),
                Vec3::new(0.4, -0.7, 0.6),
            ]),
            normal: Vec3::new(0.0, 1.0, 0.0),
        },
        Polygon {
            vertices: Vec::from([
                Vec3::new(-0.4, 0.7, 0.6),
                Vec3::new(-0.4, 0.7, 1.4),
                Vec3::new(0.4, 0.7, 1.4),
                Vec3::new(0.4, 0.7, 0.6),
            ]),
            normal: Vec3::new(0.0, 1.0, 0.0),
        },
        Polygon {
            vertices: Vec::from([
                Vec3::new(-0.4, -0.7, 1.4),
                Vec3::new(0.4, -0.7, 1.4),
                Vec3::new(0.4, 0.7, 1.4),
                Vec3::new(-0.4, 0.7, 1.4),
            ]),
            normal: Vec3::new(0.0, 0.0, 1.0),
        },
        Polygon {
            vertices: Vec::from([
                Vec3::new(-0.4, -0.7, 0.6),
                Vec3::new(0.4, -0.7, 0.6),
                Vec3::new(0.4, 0.7, 0.6),
                Vec3::new(-0.4, 0.7, 0.6),
            ]),
            normal: Vec3::new(0.0, 0.0, 1.0),
        }
    ]));
    pub static ref MARKER_POLYGONS: Polygons = Polygons(Vec::from([
        Polygon {
            vertices: Vec::from([
                Vec3::new(-0.6, -0.6, -1.0),
                Vec3::new(0.6, -0.6, -1.0),
                Vec3::new(0.6, 0.6, -1.0),
                Vec3::new(-0.6, 0.6, -1.0),
            ]),
            normal: Vec3::new(0.0, 0.0, 1.0),
        },
    ]));
    pub static ref FRAME_POLYGONS: Polygons = Polygons(Vec::from([
        Polygon {
            vertices: Vec::from([
                Vec3::new(-1.0, 1.025, -1.0),
                Vec3::new(-1.0, 1.025, 1.025),
                Vec3::new(-1.0, 0.975, 1.025),
                Vec3::new(-1.0, 0.975, -1.0),
            ]),
            normal: Vec3::new(1.0, 0.0, 0.0),
        },
        Polygon {
            vertices: Vec::from([
                Vec3::new(-1.0, 1.025, 1.025),
                Vec3::new(-1.0, -1.0, 1.025),
                Vec3::new(-1.0, -1.0, 0.975),
                Vec3::new(-1.0, 1.025, 0.975),
            ]),
            normal: Vec3::new(1.0, 0.0, 0.0),
        },
        Polygon {
            vertices: Vec::from([
                Vec3::new(1.025, -1.0, -1.0),
                Vec3::new(1.025, -1.0, 1.025),
                Vec3::new(0.975, -1.0, 1.025),
                Vec3::new(0.975, -1.0, -1.0),
            ]),
            normal: Vec3::new(0.0, 1.0, 0.0),
        },
        Polygon {
            vertices: Vec::from([
                Vec3::new(1.025, -1.0, 1.025),
                Vec3::new(-1.0, -1.0, 1.025),
                Vec3::new(-1.0, -1.0, 0.975),
                Vec3::new(1.025, -1.0, 0.975),
            ]),
            normal: Vec3::new(0.0, 1.0, 0.0),
        },
        Polygon {
            vertices: Vec::from([
                Vec3::new(1.025, -1.0, -1.0),
                Vec3::new(1.025, 1.025, -1.0),
                Vec3::new(0.975, 1.025, -1.0),
                Vec3::new(0.975, -1.0, -1.0),
            ]),
            normal: Vec3::new(0.0, 0.0, 1.0),
        },
        Polygon {
            vertices: Vec::from([
                Vec3::new(1.025, 1.025, -1.0),
                Vec3::new(-1.0, 1.025, -1.0),
                Vec3::new(-1.0, 0.975, -1.0),
                Vec3::new(1.025, 0.975, -1.0),
            ]),
            normal: Vec3::new(0.0, 0.0, 1.0),
        },
    ]));
}
