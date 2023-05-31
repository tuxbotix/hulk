use std::time::SystemTime;

use color_eyre::Result;
use context_attribute::context;
use framework::MainOutput;
use nalgebra::{Point, Point3};
use types::{CameraPosition, RobotKinematics, RobotMass};

pub struct CameraCalibrationController {}

#[context]
pub struct CreationContext {}

#[context]
pub struct CycleContext {
    pub robot_kinematics: Input<RobotKinematics, "robot_kinematics">,
    pub camera_matrices: Input<CameraMatrices, "camera_matrices">,
}

enum CaptureCommands {
    Capture {
        pub request_time: SystemTime,
        pub camera: CameraPosition,
    },
    Clear,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    pub capture_command: Option<CaptureCommand>,
}

impl CameraCalibrationController {
    pub fn new(_context: CreationContext) -> Result<Self> {
        Ok(Self {})
    }

    pub fn cycle(&mut self, context: CycleContext) -> Result<MainOutputs> {
        let capture_command = None;
        Ok(MainOutputs {
            capture_command: capture_command.into(),
        })
    }
}
