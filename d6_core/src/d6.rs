// Notation: https://en.wikipedia.org/wiki/Dihedral_group

use glam::Mat3;
use glam::Vec3;

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum D6 {
    R0,
    R1,
    R2,
    R3,
    R4,
    R5,
    S0,
    S1,
    S2,
    S3,
    S4,
    S5,
}

impl std::ops::Mul<Self> for D6 {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self::Output {
        #[rustfmt::skip]
        const MULTIPLICATION_TABLE: [[D6; 12]; 12] = [
            [D6::R0, D6::R1, D6::R2, D6::R3, D6::R4, D6::R5, D6::S0, D6::S1, D6::S2, D6::S3, D6::S4, D6::S5],
            [D6::R1, D6::R2, D6::R3, D6::R4, D6::R5, D6::R0, D6::S1, D6::S2, D6::S3, D6::S4, D6::S5, D6::S0],
            [D6::R2, D6::R3, D6::R4, D6::R5, D6::R0, D6::R1, D6::S2, D6::S3, D6::S4, D6::S5, D6::S0, D6::S1],
            [D6::R3, D6::R4, D6::R5, D6::R0, D6::R1, D6::R2, D6::S3, D6::S4, D6::S5, D6::S0, D6::S1, D6::S2],
            [D6::R4, D6::R5, D6::R0, D6::R1, D6::R2, D6::R3, D6::S4, D6::S5, D6::S0, D6::S1, D6::S2, D6::S3],
            [D6::R5, D6::R0, D6::R1, D6::R2, D6::R3, D6::R4, D6::S5, D6::S0, D6::S1, D6::S2, D6::S3, D6::S4],
            [D6::S0, D6::S5, D6::S4, D6::S3, D6::S2, D6::S1, D6::R0, D6::R5, D6::R4, D6::R3, D6::R2, D6::R1],
            [D6::S1, D6::S0, D6::S5, D6::S4, D6::S3, D6::S2, D6::R1, D6::R0, D6::R5, D6::R4, D6::R3, D6::R2],
            [D6::S2, D6::S1, D6::S0, D6::S5, D6::S4, D6::S3, D6::R2, D6::R1, D6::R0, D6::R5, D6::R4, D6::R3],
            [D6::S3, D6::S2, D6::S1, D6::S0, D6::S5, D6::S4, D6::R3, D6::R2, D6::R1, D6::R0, D6::R5, D6::R4],
            [D6::S4, D6::S3, D6::S2, D6::S1, D6::S0, D6::S5, D6::R4, D6::R3, D6::R2, D6::R1, D6::R0, D6::R5],
            [D6::S5, D6::S4, D6::S3, D6::S2, D6::S1, D6::S0, D6::R5, D6::R4, D6::R3, D6::R2, D6::R1, D6::R0],
        ];
        MULTIPLICATION_TABLE[self as usize][rhs as usize] as Self
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum Direction {
    PosX,
    PosY,
    PosZ,
    NegX,
    NegY,
    NegZ,
}

impl Direction {
    pub fn into_vec3(self) -> Vec3 {
        match self {
            Self::PosX => Vec3::X,
            Self::PosY => Vec3::Y,
            Self::PosZ => Vec3::Z,
            Self::NegX => Vec3::NEG_X,
            Self::NegY => Vec3::NEG_Y,
            Self::NegZ => Vec3::NEG_Z,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
#[allow(dead_code)]
pub enum AxisSystem {
    PosXPosYPosZ,
    NegXNegYPosZ,
    PosXNegYNegZ,
    NegXPosYNegZ,
    PosXPosZNegY,
    NegXNegZNegY,
    PosXNegZPosY,
    NegXPosZPosY,
    PosYPosZPosX,
    NegYNegZPosX,
    PosYNegZNegX,
    NegYPosZNegX,
    PosYPosXNegZ,
    NegYNegXNegZ,
    PosYNegXPosZ,
    NegYPosXPosZ,
    PosZPosXPosY,
    NegZNegXPosY,
    PosZNegXNegY,
    NegZPosXNegY,
    PosZPosYNegX,
    NegZNegYNegX,
    PosZNegYPosX,
    NegZPosYPosX,
}

impl AxisSystem {
    pub fn into_triplet(self) -> (Direction, Direction, Direction) {
        match self {
            Self::PosXPosYPosZ => (Direction::PosX, Direction::PosY, Direction::PosZ),
            Self::NegXNegYPosZ => (Direction::NegX, Direction::NegY, Direction::PosZ),
            Self::PosXNegYNegZ => (Direction::PosX, Direction::NegY, Direction::NegZ),
            Self::NegXPosYNegZ => (Direction::NegX, Direction::PosY, Direction::NegZ),
            Self::PosXPosZNegY => (Direction::PosX, Direction::PosZ, Direction::NegY),
            Self::NegXNegZNegY => (Direction::NegX, Direction::NegZ, Direction::NegY),
            Self::PosXNegZPosY => (Direction::PosX, Direction::NegZ, Direction::PosY),
            Self::NegXPosZPosY => (Direction::NegX, Direction::PosZ, Direction::PosY),
            Self::PosYPosZPosX => (Direction::PosY, Direction::PosZ, Direction::PosX),
            Self::NegYNegZPosX => (Direction::NegY, Direction::NegZ, Direction::PosX),
            Self::PosYNegZNegX => (Direction::PosY, Direction::NegZ, Direction::NegX),
            Self::NegYPosZNegX => (Direction::NegY, Direction::PosZ, Direction::NegX),
            Self::PosYPosXNegZ => (Direction::PosY, Direction::PosX, Direction::NegZ),
            Self::NegYNegXNegZ => (Direction::NegY, Direction::NegX, Direction::NegZ),
            Self::PosYNegXPosZ => (Direction::PosY, Direction::NegX, Direction::PosZ),
            Self::NegYPosXPosZ => (Direction::NegY, Direction::PosX, Direction::PosZ),
            Self::PosZPosXPosY => (Direction::PosZ, Direction::PosX, Direction::PosY),
            Self::NegZNegXPosY => (Direction::NegZ, Direction::NegX, Direction::PosY),
            Self::PosZNegXNegY => (Direction::PosZ, Direction::NegX, Direction::NegY),
            Self::NegZPosXNegY => (Direction::NegZ, Direction::PosX, Direction::NegY),
            Self::PosZPosYNegX => (Direction::PosZ, Direction::PosY, Direction::NegX),
            Self::NegZNegYNegX => (Direction::NegZ, Direction::NegY, Direction::NegX),
            Self::PosZNegYPosX => (Direction::PosZ, Direction::NegY, Direction::PosX),
            Self::NegZPosYPosX => (Direction::NegZ, Direction::PosY, Direction::PosX),
        }
    }

    pub fn into_mat3(self) -> Mat3 {
        let (x_direction, y_direction, z_direction) = self.into_triplet();
        Mat3::from_cols(
            x_direction.into_vec3(),
            y_direction.into_vec3(),
            z_direction.into_vec3(),
        )
    }
}
