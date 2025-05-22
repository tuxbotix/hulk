use nalgebra::{allocator::Allocator, DVector, DefaultAllocator, Dyn, Owned, Vector};

use types::field_dimensions::FieldDimensions;

use crate::CorrectionsTrait;

pub type ResidualVector = Vector<f32, Dyn, ResidualVectorStorage>;
pub type ResidualVectorStorage = Owned<f32, Dyn>;

pub fn calculate_residuals_from_parameters<ResidualsFromMeasurement>(
    parameters: &ResidualsFromMeasurement::Corrections,
    measurements: &[ResidualsFromMeasurement::Measurement],
    field_dimensions: &FieldDimensions,
) -> Option<ResidualVector>
where
    ResidualsFromMeasurement: CalculateResiduals,
    DefaultAllocator:
        Allocator<<ResidualsFromMeasurement::Corrections as CorrectionsTrait>::ParameterCount>,
{
    let count = measurements.iter().fold(0, |acc, measurement| {
        acc + ResidualsFromMeasurement::residual_count(measurement)
    });

    let mut residuals = DVector::zeros(count);
    let residual_slice = residuals.as_mut_slice();
    let mut offset = 0;
    for measurement in measurements {
        let residuals_part = ResidualsFromMeasurement::calculate_as_vector(
            parameters,
            measurement,
            field_dimensions,
        )
        .ok()?;
        let residual_count = ResidualsFromMeasurement::residual_count(measurement);
        residual_slice[offset..offset + residual_count].copy_from_slice(&residuals_part);
        offset += residual_count;
    }

    Some(residuals)
}

pub trait CalculateResiduals
where
    DefaultAllocator: Allocator<<Self::Corrections as CorrectionsTrait>::ParameterCount>,
{
    type Error;
    type Measurement;
    type Corrections: CorrectionsTrait;

    fn calculate_from(
        parameters: &Self::Corrections,
        measurement: &Self::Measurement,
        field_dimensions: &FieldDimensions,
    ) -> Result<Self, Self::Error>
    where
        Self: Sized;

    fn calculate_as_vector(
        parameters: &Self::Corrections,
        measurement: &Self::Measurement,
        field_dimensions: &FieldDimensions,
    ) -> Result<Vec<f32>, Self::Error>
    where
        Self: Sized;

    // fn jacobian(
    //     corrections: &Self::Corrections,
    // ) -> Jacobian<<Self::Corrections as CorrectionsTrait>::ParameterCount>;

    fn residual_count(measurement: &Self::Measurement) -> usize;
}
