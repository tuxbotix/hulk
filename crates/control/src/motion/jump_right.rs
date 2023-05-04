use color_eyre::Result;
use context_attribute::context;
use framework::MainOutput;
use motionfile::{MotionFile, MotionInterpolator};
use types::{
    ConditionInput, CycleTime, Joints, JointsCommand, MotionFinished, MotionSelection, MotionType,
    SensorData,
};

pub struct JumpRight {
    interpolator: MotionInterpolator<Joints<f32>>,
}

#[context]
pub struct CreationContext {
    pub motion_finished: PersistentState<MotionFinished, "motion_finished">,
}

#[context]
pub struct CycleContext {
    pub motion_finished: PersistentState<MotionFinished, "motion_finished">,

    pub condition_input: Input<ConditionInput, "condition_input">,
    pub cycle_time: Input<CycleTime, "cycle_time">,
    pub motion_selection: Input<MotionSelection, "motion_selection">,
    pub sensor_data: Input<SensorData, "sensor_data">,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    pub jump_right_joints_command: MainOutput<JointsCommand<f32>>,
}

impl JumpRight {
    pub fn new(_context: CreationContext) -> Result<Self> {
        Ok(Self {
            interpolator: MotionFile::from_path("etc/motions/jump_left.json")?.try_into()?,
        })
    }

    pub fn cycle(&mut self, context: CycleContext) -> Result<MainOutputs> {
        let last_cycle_duration = context.cycle_time.last_cycle_duration;
        if context.motion_selection.current_motion == MotionType::JumpRight {
            self.interpolator
                .advance_by(last_cycle_duration, context.condition_input);
        } else {
            self.interpolator.reset();
        }

        context.motion_finished[MotionType::JumpRight] = self.interpolator.is_finished();

        Ok(MainOutputs {
            jump_right_joints_command: JointsCommand {
                positions: self.interpolator.value().mirrored(),
                stiffnesses: Joints::fill(if self.interpolator.is_finished() {
                    0.0
                } else {
                    0.9
                }),
            }
            .into(),
        })
    }
}
