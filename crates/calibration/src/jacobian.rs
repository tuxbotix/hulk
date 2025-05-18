use nalgebra::{Const, Dyn, Matrix, Owned, SVector};

use types::field_dimensions::FieldDimensions;

use crate::{
    corrections::CorrectionsTrait,
    residuals::{calculate_residuals_from_parameters, CalculateResiduals},
};

pub type JacobianStorage<const PARAMETER_COUNT: usize> = Owned<f32, Dyn, Const<PARAMETER_COUNT>>;
pub type Jacobian<const PARAMETER_COUNT: usize> =
    Matrix<f32, Dyn, Const<PARAMETER_COUNT>, JacobianStorage<PARAMETER_COUNT>>;

const EPSILON: f32 = 0.000001;

pub fn calculate_jacobian_from_parameters<MeasurementResidualsType, const PARAMETER_COUNT: usize>(
    parameters: &MeasurementResidualsType::Corrections,
    measurements: &[MeasurementResidualsType::Measurement],
    field_dimensions: &FieldDimensions,
) -> Option<Jacobian<PARAMETER_COUNT>>
where
    MeasurementResidualsType: CalculateResiduals,
    MeasurementResidualsType::Corrections: CorrectionsTrait<PARAMETER_COUNT>,
    Vec<f32>: From<MeasurementResidualsType>,
{
    let parameter_vector: SVector<f32, PARAMETER_COUNT> = parameters.to_nalgebra_vector();
    let columns = (0..PARAMETER_COUNT)
        .map(|index| {
            let mut epsilon_vector = SVector::<f32, PARAMETER_COUNT>::zeros();
            epsilon_vector[index] = EPSILON;
            let upper_support_parameters =
                MeasurementResidualsType::Corrections::from_nalgebra_vector(
                    &(parameter_vector + epsilon_vector),
                );
            let lower_support_parameters =
                MeasurementResidualsType::Corrections::from_nalgebra_vector(
                    &(parameter_vector - epsilon_vector),
                );

            Some(
                (calculate_residuals_from_parameters::<MeasurementResidualsType>(
                    &upper_support_parameters,
                    measurements,
                    field_dimensions,
                )? - calculate_residuals_from_parameters::<MeasurementResidualsType>(
                    &lower_support_parameters,
                    measurements,
                    field_dimensions,
                )?) / (2.0 * EPSILON),
            )
        })
        .collect::<Option<Vec<_>>>()?;
    Some(Matrix::from_columns(&columns))
}
