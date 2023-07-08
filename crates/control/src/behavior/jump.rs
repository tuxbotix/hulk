use nalgebra::point;
use types::{HeadMotion, JumpDirection, MotionCommand, PenaltyShotDirection, WorldState};

pub fn execute(
    world_state: &WorldState,
    testing_mode_without_jumping: bool,
) -> Option<MotionCommand> {
    let look_at_left = point![4.0, 1.5];
    let look_at_right = point![look_at_left.x, -look_at_left.y];

    world_state
        .ball
        .and_then(|ball| match ball.penalty_shot_direction {
            Some(PenaltyShotDirection::Left) => {
                if testing_mode_without_jumping {
                    Some(MotionCommand::ArmsUpSquat {
                        head: Some(HeadMotion::LookAt {
                            target: look_at_left,
                            camera: None,
                        }),
                    })
                } else {
                    Some(MotionCommand::Jump {
                        direction: JumpDirection::Left,
                    })
                }
            }
            Some(PenaltyShotDirection::Right) => {
                if testing_mode_without_jumping {
                    Some(MotionCommand::ArmsUpSquat {
                        head: Some(HeadMotion::LookAt {
                            target: look_at_right,
                            camera: None,
                        }),
                    })
                } else {
                    Some(MotionCommand::Jump {
                        direction: JumpDirection::Right,
                    })
                }
            }
            Some(PenaltyShotDirection::NotMoving) | None => None,
        })
}
