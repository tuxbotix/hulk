use geometry::Distance;
use nalgebra::Dim;
use types::field_dimensions::FieldDimensions;

use crate::{
    corrections::{get_corrected_camera_matrix, CorrectionsTrait, ExtrinsicCorrections},
    jacobian::{jacobian_central_difference_mut, JacobianViewMut},
    residuals::CalculateDifferentiableResiduals,
};

use super::{lines::LinesError, measurement::Measurement};

#[derive(Debug, Copy, Clone)]
pub struct GoalBoxResiduals {
    pub border_to_connecting_angle: f32,
    pub connecting_to_goal_box_angle: f32,
    pub distance_between_parallel_line_start_points: f32,
    pub distance_between_parallel_line_center_points: f32,
    pub distance_between_parallel_line_end_points: f32,
}

impl CalculateDifferentiableResiduals for GoalBoxResiduals {
    type Error = ResidualsError;
    type Measurement = Measurement;
    type Corrections = ExtrinsicCorrections;

    fn calculate_from(
        parameters: &ExtrinsicCorrections,
        measurement: &Measurement,
        field_dimensions: &FieldDimensions,
    ) -> Result<Self, Self::Error> {
        let corrected =
            get_corrected_camera_matrix(&measurement.matrix, measurement.position, parameters);

        let projected_lines = measurement
            .lines
            .project_to_ground(&corrected)
            .map_err(ResidualsError::NotProjected)?;

        let border_to_connecting_angle = projected_lines
            .border_line
            .signed_acute_angle_to_orthogonal(projected_lines.connecting_line);
        let connecting_to_goal_box_angle = projected_lines
            .border_line
            .signed_acute_angle_to_orthogonal(projected_lines.connecting_line);
        let distance_between_parallel_line_start_points = projected_lines
            .border_line
            .distance_to(projected_lines.goal_box_line.0);
        let distance_between_parallel_line_center_points = projected_lines
            .border_line
            .distance_to(projected_lines.goal_box_line.center());
        let distance_between_parallel_line_end_points = projected_lines
            .border_line
            .distance_to(projected_lines.goal_box_line.1);

        Ok(GoalBoxResiduals {
            border_to_connecting_angle,
            connecting_to_goal_box_angle,
            distance_between_parallel_line_start_points: distance_between_parallel_line_start_points
                - field_dimensions.goal_box_area_length,
            distance_between_parallel_line_center_points:
                distance_between_parallel_line_center_points - field_dimensions.goal_box_area_length,
            distance_between_parallel_line_end_points: distance_between_parallel_line_end_points
                - field_dimensions.goal_box_area_length,
        })
    }

    fn residual_count(_measurement: &Self::Measurement) -> usize {
        5
    }

    fn calculate_as_vector(
        parameters: &Self::Corrections,
        measurement: &Self::Measurement,
        field_dimensions: &FieldDimensions,
    ) -> Result<Vec<f32>, Self::Error>
    where
        Self: Sized,
    {
        let residuals = Self::calculate_from(parameters, measurement, field_dimensions)?;
        Ok(residuals.into())
    }

    fn jacobian(
        parameters: &ExtrinsicCorrections,
        measurement: &Measurement,
        field_dimensions: &FieldDimensions,
        out: JacobianViewMut<<ExtrinsicCorrections as CorrectionsTrait>::ParameterCount>,
    ) -> Result<(), ResidualsError> {
        jacobian_central_difference_mut::<Self>(parameters, measurement, field_dimensions, out)
    }
}

impl From<GoalBoxResiduals> for Vec<f32> {
    fn from(residuals: GoalBoxResiduals) -> Self {
        vec![
            residuals.border_to_connecting_angle,
            residuals.connecting_to_goal_box_angle,
            residuals.distance_between_parallel_line_start_points,
            residuals.distance_between_parallel_line_center_points,
            residuals.distance_between_parallel_line_end_points,
        ]
    }
}

#[derive(Debug, thiserror::Error)]
pub enum ResidualsError {
    #[error("failed to project measurements to ground")]
    NotProjected(#[source] LinesError),
}
