use coordinate_systems::{Camera, Robot};
use nalgebra::{
    allocator::Allocator, vector, Const, DefaultAllocator, Dim, DimName, Matrix, Owned, SVector,
    UnitQuaternion, U1,
};
use serde::{Deserialize, Serialize};

use linear_algebra::{IntoTransform, Rotation3};
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use projection::camera_matrix::CameraMatrix;
use types::camera_position::CameraPosition;

pub type ParametersStorage<N> = Owned<f32, N, U1>;
pub type Parameters<N> = Matrix<f32, N, U1, ParametersStorage<N>>;

pub trait CorrectionsTrait
where
    Self::ParameterCount: Dim + DimName,
    DefaultAllocator: Allocator<Self::ParameterCount>,
{
    type ParameterCount: Dim + DimName;

    fn to_svector(&self) -> Parameters<Self::ParameterCount>;
    fn from_svector(vector: &Parameters<Self::ParameterCount>) -> Self;

    fn extrinsic_corrections(self) -> ExtrinsicCorrections;
}

pub const AMOUNT_OF_EXTRINSIC_PARAMETERS: usize = 9;

#[derive(
    Clone,
    Copy,
    Debug,
    Default,
    Serialize,
    Deserialize,
    PathDeserialize,
    PathSerialize,
    PathIntrospect,
)]
pub struct ExtrinsicCorrections {
    pub correction_in_robot: Rotation3<Robot, Robot, f32>,
    pub correction_in_camera_top: Rotation3<Camera, Camera, f32>,
    pub correction_in_camera_bottom: Rotation3<Camera, Camera, f32>,
}

impl CorrectionsTrait for ExtrinsicCorrections {
    fn to_svector(&self) -> SVector<f32, AMOUNT_OF_EXTRINSIC_PARAMETERS> {
        let (robot_roll, robot_pitch, robot_yaw) = self.correction_in_robot.inner.euler_angles();
        let (camera_top_roll, camera_top_pitch, camera_top_yaw) =
            self.correction_in_camera_top.inner.euler_angles();
        let (camera_bottom_roll, camera_bottom_pitch, camera_bottom_yaw) =
            self.correction_in_camera_bottom.inner.euler_angles();
        vector![
            robot_roll,
            robot_pitch,
            robot_yaw,
            camera_top_roll,
            camera_top_pitch,
            camera_top_yaw,
            camera_bottom_roll,
            camera_bottom_pitch,
            camera_bottom_yaw
        ]
    }

    fn from_svector(parameters: &SVector<f32, AMOUNT_OF_EXTRINSIC_PARAMETERS>) -> Self {
        Self {
            // correction_in_robot: UnitQuaternion::from_euler_angles(
            //     parameters[0],
            //     parameters[1],
            //     parameters[2],
            // )
            // .framed_transform(),
            correction_in_robot: UnitQuaternion::identity().framed_transform(),
            correction_in_camera_top: UnitQuaternion::from_euler_angles(
                parameters[3],
                parameters[4],
                parameters[5],
            )
            .framed_transform(),
            correction_in_camera_bottom: UnitQuaternion::from_euler_angles(
                parameters[6],
                parameters[7],
                parameters[8],
            )
            .framed_transform(),
        }
    }

    fn extrinsic_corrections(self) -> ExtrinsicCorrections {
        self
    }

    type ParameterCount = Const<AMOUNT_OF_EXTRINSIC_PARAMETERS>;
}

pub(crate) fn get_corrected_camera_matrix(
    input_matrix: &CameraMatrix,
    position: CameraPosition,
    parameters: &ExtrinsicCorrections,
) -> CameraMatrix {
    input_matrix.to_corrected(
        parameters.correction_in_robot,
        match position {
            CameraPosition::Top => parameters.correction_in_camera_top,
            CameraPosition::Bottom => parameters.correction_in_camera_bottom,
        },
    )
}
