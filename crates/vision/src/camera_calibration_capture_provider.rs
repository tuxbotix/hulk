use color_eyre::Result;
use context_attribute::context;
use framework::MainOutput;
use nalgebra::{Point, Point3};
use types::{RobotKinematics, RobotMass};

pub struct CameraCalibrationController {
    current_calibration_captures: Vec<CameraCalibrationCapture>,
}

#[context]
pub struct CreationContext {}

#[context]
pub struct CycleContext {
    pub camera_matrices: RequiredInput<Option<CameraMatrices>, "Control", "camera_matrices?">,
    pub image: Input<YCbCr422Image, "image">,
    pub capture_command: Input<Option<CaptureCommand>, "capture_command">,
    pub sensor_data: Input<SensorData, "sensor_data">,
}

pub struct CameraCalibrationCapture {
    pub camer_matrix: CameraMatrix,
    pub image: GrayscaleImage,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    pub camera_calibration_captures: Option<Vec<CameraCalibrationCapture>>,
}

impl CameraCalibrationController {
    pub fn new(_context: CreationContext) -> Result<Self> {
        Ok(Self {
            current_calibration_captures: vec![],
        })
    }

    pub fn cycle(&mut self, context: CycleContext) -> Result<MainOutputs> {
        if let Some(command) = context.capture_command {
            match command {
                CaptureCommands::Clear => {
                    camera_calibration_captures.clear();
                }
                _ => {}
            }
        }

        Ok(MainOutputs {
            current_calibration_captures,
        })
    }
}
