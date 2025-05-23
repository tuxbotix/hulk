use nalgebra::{
    allocator::Allocator, DefaultAllocator, Dim, Dyn, Matrix, MatrixViewMut, MatrixXx1, Owned,
};

use types::field_dimensions::FieldDimensions;

use crate::{
    corrections::{CorrectionsTrait, Parameters},
    residuals::{
        calculate_residuals_from_parameters, CalculateDifferentiableResiduals, ResidualVector,
    },
};

pub type JacobianStorage<N> = Owned<f32, Dyn, N>;
pub type Jacobian<N> = Matrix<f32, Dyn, N, JacobianStorage<N>>;
pub type JacobianViewMut<'a, N> = MatrixViewMut<'a, f32, Dyn, N>;

pub(crate) fn calculate_jacobian_from_parameters<MeasurementResidualsType>(
    corrections: &MeasurementResidualsType::Corrections,
    measurements: &[MeasurementResidualsType::Measurement],
    field_dimensions: &FieldDimensions,
) -> Option<Jacobian<<MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount>>
where
    MeasurementResidualsType: CalculateDifferentiableResiduals,
    DefaultAllocator:
        Allocator<<MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount>,
{
    let count = measurements.into_iter().fold(0, |acc, measurement| {
        acc + MeasurementResidualsType::residual_count(measurement)
    });
    let mut out = Jacobian::<
        <MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount,
    >::zeros(count);
    let param_count =
        <MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount::try_to_usize()
            .unwrap();

    let mut offset = 0;
    for measurement in measurements {
        let residual_count = MeasurementResidualsType::residual_count(measurement);
        let out_chunk = out.rows_range_mut((offset..offset + residual_count));
        MeasurementResidualsType::jacobian(corrections, measurement, field_dimensions, out_chunk)
            .ok()?;

        offset += residual_count;
    }
    assert!(offset == count);

    Some(out)
}

const EPSILON: f32 = f32::EPSILON;

pub fn jacobian_central_difference_mut<MeasurementResidualsType>(
    parameters: &MeasurementResidualsType::Corrections,
    measurement: &MeasurementResidualsType::Measurement,
    field_dimensions: &FieldDimensions,
    mut out: JacobianViewMut<
        <MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount,
    >,
) -> Result<(), MeasurementResidualsType::Error>
where
    MeasurementResidualsType: CalculateDifferentiableResiduals,
    MeasurementResidualsType::Corrections: CorrectionsTrait,
    DefaultAllocator:
        Allocator<<MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount>,
{
    let parameter_vector = parameters.to_svector();

    assert!(out.nrows() == MeasurementResidualsType::residual_count(measurement));
    assert!(out.ncols() == <MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount::try_to_usize().unwrap());

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

        let upper = ResidualVector::from_vec(MeasurementResidualsType::calculate_as_vector(
            &upper_support_parameters,
            measurement,
            field_dimensions,
        )?);
        let lower = ResidualVector::from_vec(MeasurementResidualsType::calculate_as_vector(
            &lower_support_parameters,
            measurement,
            field_dimensions,
        )?);
        out.column_mut(index)
            .copy_from(&((upper - lower) / (2.0 * EPSILON)));
        // let mut out_slice = out.column_mut(index);
        // for (index, out_element) in out_slice.iter_mut().enumerate() {
        //     *out_element = (upper[index] - upper[index]) / (2.0 * EPSILON);
        // }
    }

    Ok(())
}

// TODO remove
pub fn jacobian_central_difference<MeasurementResidualsType>(
    parameters: &MeasurementResidualsType::Corrections,
    measurements: &[MeasurementResidualsType::Measurement],
    field_dimensions: &FieldDimensions,
) -> Option<Jacobian<<MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount>>
where
    MeasurementResidualsType: CalculateDifferentiableResiduals,
    MeasurementResidualsType::Corrections: CorrectionsTrait,
    DefaultAllocator:
        Allocator<<MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount>,
{
    let parameter_vector = parameters.to_svector();

    let count = measurements.iter().fold(0, |acc, measurement| {
        acc + MeasurementResidualsType::residual_count(measurement)
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

    Some(output)
}
