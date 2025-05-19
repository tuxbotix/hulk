use nalgebra::{DVector, Dyn, Owned, Vector};

use types::field_dimensions::FieldDimensions;

pub type ResidualVector = Vector<f32, Dyn, ResidualVectorStorage>;
pub type ResidualVectorStorage = Owned<f32, Dyn>;

pub fn calculate_residuals_from_parameters<ResidualsFromMeasurement>(
    parameters: &ResidualsFromMeasurement::Corrections,
    measurements: &[ResidualsFromMeasurement::Measurement],
    field_dimensions: &FieldDimensions,
) -> Option<ResidualVector>
where
    ResidualsFromMeasurement: CalculateResiduals,
{
    let count = measurements.iter().fold(0, |acc, measurement| {
        acc + ResidualsFromMeasurement::residual_count(&measurement)
    });

    let mut residuals = DVector::zeros(count);
    let mut residual_slice = residuals.as_mut_slice();
    let mut offset = 0;
    for measurement in measurements {
        let residuals_part =
            ResidualsFromMeasurement::calculate_from(parameters, measurement, field_dimensions)
                .ok()?;
        // assert!(residuals_part.len() == measurement.residual_count());

        let residual_count = ResidualsFromMeasurement::residual_count(&measurement);
        residuals_part.copy_to_slice(&mut residual_slice[offset..offset + residual_count])?;
        offset += residual_count;
    }

    Some(residuals)
}

pub trait CalculateResiduals {
    type Error;
    type Measurement;
    type Corrections;

    fn calculate_from(
        parameters: &Self::Corrections,
        measurement: &Self::Measurement,
        field_dimensions: &FieldDimensions,
    ) -> Result<Self, Self::Error>
    where
        Self: Sized;

    // fn calculate_from_in_place(
    //     parameters: &Self::Corrections,
    //     measurement: &Self::Measurement,
    //     field_dimensions: &FieldDimensions,
    //     residuals: &mut ResidualVector,
    // ) -> Result<usize, Self::Error>
    // where
    //     Self: Sized;

    fn copy_to_slice(&self, out: &mut [f32]) -> Option<usize>;

    fn residual_count(measurement: &Self::Measurement) -> usize;
}
