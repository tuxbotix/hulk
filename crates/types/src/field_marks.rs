use geometry::line::{Line, Line2};
use ordered_float::NotNan;
use serde::{Deserialize, Serialize};

use coordinate_systems::Field;

use crate::field_dimensions::FieldDimensions;
use linear_algebra::{distance, point, vector, Point2, Vector2};

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub enum FieldMark {
    Line {
        line: Line2<Field>,
        direction: Direction,
    },
    Circle {
        center: Point2<Field>,
        radius: f32,
    },
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub enum Direction {
    PositiveX,
    PositiveY,
}

impl FieldMark {
    pub fn to_correspondence_points(self, measured_line: Line2<Field>) -> Correspondences {
        match self {
            FieldMark::Line {
                line: reference_line,
                direction: _,
            } => {
                let measured_line = match [
                    distance(measured_line.first, reference_line.first),
                    distance(measured_line.first, reference_line.second),
                    distance(measured_line.second, reference_line.first),
                    distance(measured_line.second, reference_line.second),
                ]
                .iter()
                .enumerate()
                .min_by_key(|(_index, distance)| NotNan::new(**distance).unwrap())
                .unwrap()
                .0
                {
                    1 | 2 => Line::new(measured_line.second, measured_line.first),
                    _ => measured_line,
                };

                let measured_direction = (measured_line.first - measured_line.second).normalize();
                let reference_direction =
                    (reference_line.first - reference_line.second).normalize();

                let projected_point_on_measured_line =
                    measured_line.project_onto_segment(reference_line.first);
                let projected_point_on_reference_line =
                    reference_line.project_onto_segment(measured_line.first);

                let measured_distance =
                    distance(projected_point_on_measured_line, reference_line.first);
                let reference_distance =
                    distance(measured_line.first, projected_point_on_reference_line);
                let correspondence_0 = if measured_distance < reference_distance {
                    CorrespondencePoints {
                        measured: projected_point_on_measured_line,
                        reference: reference_line.first,
                    }
                } else {
                    CorrespondencePoints {
                        measured: measured_line.first,
                        reference: projected_point_on_reference_line,
                    }
                };

                let projected_point_on_measured_line =
                    measured_line.project_onto_segment(reference_line.second);
                let projected_point_on_reference_line =
                    reference_line.project_onto_segment(measured_line.second);

                let measured_distance =
                    distance(projected_point_on_measured_line, reference_line.second);
                let reference_distance =
                    distance(measured_line.second, projected_point_on_reference_line);
                let correspondence_1 = if measured_distance < reference_distance {
                    CorrespondencePoints {
                        measured: projected_point_on_measured_line,
                        reference: reference_line.second,
                    }
                } else {
                    CorrespondencePoints {
                        measured: measured_line.second,
                        reference: projected_point_on_reference_line,
                    }
                };

                Correspondences {
                    correspondence_points: (correspondence_0, correspondence_1),
                    measured_direction,
                    reference_direction,
                }
            }
            FieldMark::Circle { center, radius } => {
                let center_to_0 = measured_line.first - center;
                let center_to_1 = measured_line.second - center;

                let correspondence_0_measured = measured_line.first;
                let correspondence_0_reference = if center_to_0 == Vector2::zeros() {
                    point![center.x() + radius, center.y()]
                } else {
                    center + center_to_0.normalize() * radius
                };

                let correspondence_1_measured = measured_line.second;
                let correspondence_1_reference = if center_to_1 == Vector2::zeros() {
                    point![center.x() + radius, center.y()]
                } else {
                    center + center_to_1.normalize() * radius
                };

                let measured_direction = (measured_line.first - measured_line.second).normalize();
                let center_vector =
                    (correspondence_0_reference - center) + (correspondence_1_reference - center);
                let center_vector_rotated_by_90_degree =
                    vector![-center_vector.y(), center_vector.x()];
                let reference_direction = center_vector_rotated_by_90_degree.normalize();

                Correspondences {
                    correspondence_points: (
                        CorrespondencePoints {
                            measured: correspondence_0_measured,
                            reference: correspondence_0_reference,
                        },
                        CorrespondencePoints {
                            measured: correspondence_1_measured,
                            reference: correspondence_1_reference,
                        },
                    ),
                    measured_direction,
                    reference_direction,
                }
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct Correspondences {
    pub correspondence_points: (CorrespondencePoints, CorrespondencePoints),
    pub measured_direction: Vector2<Field>,
    pub reference_direction: Vector2<Field>,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct CorrespondencePoints {
    pub measured: Point2<Field>,
    pub reference: Point2<Field>,
}

pub fn field_marks_from_field_dimensions(field_dimensions: &FieldDimensions) -> Vec<FieldMark> {
    vec![
        FieldMark::Line {
            line: Line::new(
                point![-field_dimensions.length / 2.0, field_dimensions.width / 2.0],
                point![field_dimensions.length / 2.0, field_dimensions.width / 2.0],
            ),
            direction: Direction::PositiveX,
        },
        FieldMark::Line {
            line: Line::new(
                point![
                    -field_dimensions.length / 2.0,
                    -field_dimensions.width / 2.0
                ],
                point![field_dimensions.length / 2.0, -field_dimensions.width / 2.0],
            ),
            direction: Direction::PositiveX,
        },
        FieldMark::Line {
            line: Line::new(
                point![
                    -field_dimensions.length / 2.0,
                    -field_dimensions.width / 2.0
                ],
                point![-field_dimensions.length / 2.0, field_dimensions.width / 2.0],
            ),
            direction: Direction::PositiveY,
        },
        FieldMark::Line {
            line: Line::new(
                point![field_dimensions.length / 2.0, -field_dimensions.width / 2.0],
                point![field_dimensions.length / 2.0, field_dimensions.width / 2.0],
            ),
            direction: Direction::PositiveY,
        },
        FieldMark::Line {
            line: Line::new(
                point![
                    -field_dimensions.length / 2.0,
                    field_dimensions.penalty_area_width / 2.0
                ],
                point![
                    -field_dimensions.length / 2.0 + field_dimensions.penalty_area_length,
                    field_dimensions.penalty_area_width / 2.0
                ],
            ),
            direction: Direction::PositiveX,
        },
        FieldMark::Line {
            line: Line::new(
                point![
                    -field_dimensions.length / 2.0,
                    -field_dimensions.penalty_area_width / 2.0
                ],
                point![
                    -field_dimensions.length / 2.0 + field_dimensions.penalty_area_length,
                    -field_dimensions.penalty_area_width / 2.0
                ],
            ),
            direction: Direction::PositiveX,
        },
        FieldMark::Line {
            line: Line::new(
                point![
                    -field_dimensions.length / 2.0 + field_dimensions.penalty_area_length,
                    -field_dimensions.penalty_area_width / 2.0
                ],
                point![
                    -field_dimensions.length / 2.0 + field_dimensions.penalty_area_length,
                    field_dimensions.penalty_area_width / 2.0
                ],
            ),
            direction: Direction::PositiveY,
        },
        FieldMark::Line {
            line: Line::new(
                point![
                    -field_dimensions.length / 2.0,
                    field_dimensions.goal_box_area_width / 2.0
                ],
                point![
                    -field_dimensions.length / 2.0 + field_dimensions.goal_box_area_length,
                    field_dimensions.goal_box_area_width / 2.0
                ],
            ),
            direction: Direction::PositiveX,
        },
        FieldMark::Line {
            line: Line::new(
                point![
                    -field_dimensions.length / 2.0,
                    -field_dimensions.goal_box_area_width / 2.0
                ],
                point![
                    -field_dimensions.length / 2.0 + field_dimensions.goal_box_area_length,
                    -field_dimensions.goal_box_area_width / 2.0
                ],
            ),
            direction: Direction::PositiveX,
        },
        FieldMark::Line {
            line: Line::new(
                point![
                    -field_dimensions.length / 2.0 + field_dimensions.goal_box_area_length,
                    -field_dimensions.goal_box_area_width / 2.0
                ],
                point![
                    -field_dimensions.length / 2.0 + field_dimensions.goal_box_area_length,
                    field_dimensions.goal_box_area_width / 2.0
                ],
            ),
            direction: Direction::PositiveY,
        },
        FieldMark::Line {
            line: Line::new(
                point![
                    field_dimensions.length / 2.0 - field_dimensions.penalty_area_length,
                    field_dimensions.penalty_area_width / 2.0
                ],
                point![
                    field_dimensions.length / 2.0,
                    field_dimensions.penalty_area_width / 2.0
                ],
            ),
            direction: Direction::PositiveX,
        },
        FieldMark::Line {
            line: Line::new(
                point![
                    field_dimensions.length / 2.0 - field_dimensions.penalty_area_length,
                    -field_dimensions.penalty_area_width / 2.0
                ],
                point![
                    field_dimensions.length / 2.0,
                    -field_dimensions.penalty_area_width / 2.0
                ],
            ),
            direction: Direction::PositiveX,
        },
        FieldMark::Line {
            line: Line::new(
                point![
                    field_dimensions.length / 2.0 - field_dimensions.penalty_area_length,
                    -field_dimensions.penalty_area_width / 2.0
                ],
                point![
                    field_dimensions.length / 2.0 - field_dimensions.penalty_area_length,
                    field_dimensions.penalty_area_width / 2.0
                ],
            ),
            direction: Direction::PositiveY,
        },
        FieldMark::Line {
            line: Line::new(
                point![
                    field_dimensions.length / 2.0 - field_dimensions.goal_box_area_length,
                    field_dimensions.goal_box_area_width / 2.0
                ],
                point![
                    field_dimensions.length / 2.0,
                    field_dimensions.goal_box_area_width / 2.0
                ],
            ),
            direction: Direction::PositiveX,
        },
        FieldMark::Line {
            line: Line::new(
                point![
                    field_dimensions.length / 2.0 - field_dimensions.goal_box_area_length,
                    -field_dimensions.goal_box_area_width / 2.0
                ],
                point![
                    field_dimensions.length / 2.0,
                    -field_dimensions.goal_box_area_width / 2.0
                ],
            ),
            direction: Direction::PositiveX,
        },
        FieldMark::Line {
            line: Line::new(
                point![
                    field_dimensions.length / 2.0 - field_dimensions.goal_box_area_length,
                    -field_dimensions.goal_box_area_width / 2.0
                ],
                point![
                    field_dimensions.length / 2.0 - field_dimensions.goal_box_area_length,
                    field_dimensions.goal_box_area_width / 2.0
                ],
            ),
            direction: Direction::PositiveY,
        },
        FieldMark::Line {
            line: Line::new(
                point![0.0, -field_dimensions.width / 2.0],
                point![0.0, field_dimensions.width / 2.0],
            ),
            direction: Direction::PositiveY,
        },
        FieldMark::Circle {
            center: Point2::origin(),
            radius: field_dimensions.center_circle_diameter / 2.0,
        },
        FieldMark::Line {
            line: Line::new(
                point![
                    -field_dimensions.length / 2.0 + field_dimensions.penalty_marker_distance
                        - field_dimensions.penalty_marker_size / 2.0,
                    0.0
                ],
                point![
                    -field_dimensions.length / 2.0
                        + field_dimensions.penalty_marker_distance
                        + field_dimensions.penalty_marker_size / 2.0,
                    0.0
                ],
            ),
            direction: Direction::PositiveX,
        },
        FieldMark::Line {
            line: Line::new(
                point![
                    -field_dimensions.length / 2.0 + field_dimensions.penalty_marker_distance,
                    -field_dimensions.penalty_marker_size / 2.0
                ],
                point![
                    -field_dimensions.length / 2.0 + field_dimensions.penalty_marker_distance,
                    field_dimensions.penalty_marker_size / 2.0
                ],
            ),
            direction: Direction::PositiveY,
        },
        FieldMark::Line {
            line: Line::new(
                point![
                    field_dimensions.length / 2.0
                        - field_dimensions.penalty_marker_distance
                        - field_dimensions.penalty_marker_size / 2.0,
                    0.0
                ],
                point![
                    field_dimensions.length / 2.0 - field_dimensions.penalty_marker_distance
                        + field_dimensions.penalty_marker_size / 2.0,
                    0.0
                ],
            ),
            direction: Direction::PositiveX,
        },
        FieldMark::Line {
            line: Line::new(
                point![
                    field_dimensions.length / 2.0 - field_dimensions.penalty_marker_distance,
                    -field_dimensions.penalty_marker_size / 2.0
                ],
                point![
                    field_dimensions.length / 2.0 - field_dimensions.penalty_marker_distance,
                    field_dimensions.penalty_marker_size / 2.0
                ],
            ),
            direction: Direction::PositiveY,
        },
    ]
}
