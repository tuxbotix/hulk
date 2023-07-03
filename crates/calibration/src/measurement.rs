use coordinate_systems::Pixel;
use projection::camera_matrix::CameraMatrix;
use types::camera_position::CameraPosition;

use crate::lines::GoalBoxCalibrationLines;

#[derive(Clone)]
pub struct Measurement {
    pub position: CameraPosition,
    pub matrix: CameraMatrix,
    pub lines: GoalBoxCalibrationLines<Pixel>,
}
