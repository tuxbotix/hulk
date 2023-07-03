use types::{CameraMatrix, CameraPosition};

use crate::lines::GoalBoxCalibrationLines;

#[derive(Clone)]
pub struct Measurement {
    pub position: CameraPosition,
    pub matrix: CameraMatrix,
    pub lines: GoalBoxCalibrationLines,
}
