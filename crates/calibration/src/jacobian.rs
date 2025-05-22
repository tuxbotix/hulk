use nalgebra::{allocator::Allocator, DefaultAllocator, Dim, Dyn, Matrix, Owned};

use types::field_dimensions::FieldDimensions;

use crate::{
    corrections::{CorrectionsTrait, Parameters},
    residuals::{calculate_residuals_from_parameters, CalculateResiduals},
};

pub type JacobianStorage<N> = Owned<f32, Dyn, N>;
pub type Jacobian<N> = Matrix<f32, Dyn, N, JacobianStorage<N>>;

pub trait ResidualJacobian
where
    Self::Residuals: CalculateResiduals,
    DefaultAllocator: Allocator<
        <<Self::Residuals as CalculateResiduals>::Corrections as CorrectionsTrait>::ParameterCount,
    >,
{
    type Residuals: CalculateResiduals;

    fn jacobian(
        corrections: &<Self::Residuals as CalculateResiduals>::Corrections,
        measurements: &[<Self::Residuals as CalculateResiduals>::Measurement],
        field_dimensions: &FieldDimensions,
    ) -> Jacobian<
        <<Self::Residuals as CalculateResiduals>::Corrections as CorrectionsTrait>::ParameterCount,
    >;
}

const EPSILON: f32 = f32::EPSILON;

pub fn calculate_jacobian_from_parameters<MeasurementResidualsType>(
    parameters: &MeasurementResidualsType::Corrections,
    measurements: &[MeasurementResidualsType::Measurement],
    field_dimensions: &FieldDimensions,
) -> Option<Jacobian<<MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount>>
where
    MeasurementResidualsType: CalculateResiduals,
    MeasurementResidualsType::Corrections: CorrectionsTrait,
    Vec<f32>: From<MeasurementResidualsType>,
    DefaultAllocator:
        Allocator<<MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount>,
{
    let parameter_vector = parameters.to_svector();

    let count = measurements.iter().fold(0, |acc, measurement| {
        acc + MeasurementResidualsType::residual_count(&measurement)
    });

    let mut output = Jacobian::<
        <MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount,
    >::zeros(count);

    // for (index, &mut column) in output.columns_mut().enumerate() {
    for index in 0
        ..<MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount::try_to_usize(
        )
        .unwrap()
    {
        let mut epsilon_vector = Parameters::<
            <MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount,
        >::zeros();
        epsilon_vector[index] = EPSILON;
        let upper_support_parameters = MeasurementResidualsType::Corrections::from_svector(
            &(parameter_vector.clone() + epsilon_vector.clone()),
        );
        let lower_support_parameters = MeasurementResidualsType::Corrections::from_svector(
            &(parameter_vector.clone() - epsilon_vector),
        );

        output.column_mut(index).copy_from(
            &((calculate_residuals_from_parameters::<MeasurementResidualsType>(
                &upper_support_parameters,
                measurements,
                field_dimensions,
            )? - calculate_residuals_from_parameters::<MeasurementResidualsType>(
                &lower_support_parameters,
                measurements,
                field_dimensions,
            )?) / (2.0 * EPSILON)),
        );
    }

    // let columns = (0..<MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount::try_to_usize().unwrap())
    //     .map(|index| {
    //         let mut epsilon_vector = Parameters::<
    //             <MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount,
    //         >::zeros();
    //         epsilon_vector[index] = EPSILON;
    //         let upper_support_parameters = MeasurementResidualsType::Corrections::from_svector(
    //             &(parameter_vector + epsilon_vector),
    //         );
    //         let lower_support_parameters = MeasurementResidualsType::Corrections::from_svector(
    //             &(parameter_vector - epsilon_vector),
    //         );

    //         Some(
    //             (calculate_residuals_from_parameters::<MeasurementResidualsType>(
    //                 &upper_support_parameters,
    //                 measurements,
    //                 field_dimensions,
    //             )? - calculate_residuals_from_parameters::<MeasurementResidualsType>(
    //                 &lower_support_parameters,
    //                 measurements,
    //                 field_dimensions,
    //             )?) / (2.0 * EPSILON),
    //         )
    //     })
    //     .collect::<Option<Vec<_>>>()?;
    // Some(Matrix::from_columns(&columns))
    Some(output)
}
