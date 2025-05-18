use levenberg_marquardt::LeastSquaresProblem;
use nalgebra::{Const, Dyn, Owned, SVector};

use types::field_dimensions::FieldDimensions;

use crate::{
    corrections::{Corrections, CorrectionsTrait},
    jacobian::{calculate_jacobian_from_parameters, Jacobian, JacobianStorage},
    residuals::{
        calculate_residuals_from_parameters, CalculateResiduals, ResidualVector,
        ResidualVectorStorage,
    },
};

pub struct CalibrationProblem<MeasurementResidualsType, const PARAMETER_COUNT: usize>
where
    MeasurementResidualsType: CalculateResiduals,
    MeasurementResidualsType::Corrections: CorrectionsTrait<PARAMETER_COUNT>,
{
    parameters: MeasurementResidualsType::Corrections,
    measurements: Vec<MeasurementResidualsType::Measurement>,
    field_dimensions: FieldDimensions,
}

impl<MeasurementResidualsType, const PARAMETER_COUNT: usize>
    CalibrationProblem<MeasurementResidualsType, PARAMETER_COUNT>
where
    MeasurementResidualsType: CalculateResiduals,
    MeasurementResidualsType::Corrections: Copy + CorrectionsTrait<PARAMETER_COUNT>,
{
    pub fn new(
        initial_corrections: MeasurementResidualsType::Corrections,
        measurements: Vec<MeasurementResidualsType::Measurement>,
        field_dimensions: FieldDimensions,
    ) -> Self {
        Self {
            parameters: initial_corrections,
            measurements,
            field_dimensions,
        }
    }

    pub fn get_corrections(&self) -> Corrections {
        self.parameters.base_corrections()
    }
}

impl<MeasurementResidualsType, const PARAMETER_COUNT: usize>
    LeastSquaresProblem<f32, Dyn, Const<PARAMETER_COUNT>>
    for CalibrationProblem<MeasurementResidualsType, PARAMETER_COUNT>
where
    MeasurementResidualsType: CalculateResiduals,
    Vec<f32>: From<MeasurementResidualsType>,
    MeasurementResidualsType::Corrections: CorrectionsTrait<PARAMETER_COUNT>,
{
    type ResidualStorage = ResidualVectorStorage;
    type JacobianStorage = JacobianStorage<PARAMETER_COUNT>;
    type ParameterStorage = Owned<f32, Const<PARAMETER_COUNT>>;

    fn set_params(&mut self, parameters: &SVector<f32, PARAMETER_COUNT>) {
        self.parameters = MeasurementResidualsType::Corrections::from_nalgebra_vector(parameters);
    }

    fn params(&self) -> SVector<f32, PARAMETER_COUNT> {
        self.parameters.to_nalgebra_vector()
    }

    fn residuals(&self) -> Option<ResidualVector> {
        calculate_residuals_from_parameters::<MeasurementResidualsType>(
            &self.parameters,
            &self.measurements,
            &self.field_dimensions,
        )
    }

    fn jacobian(&self) -> Option<Jacobian<PARAMETER_COUNT>> {
        calculate_jacobian_from_parameters::<MeasurementResidualsType, PARAMETER_COUNT>(
            &self.parameters,
            &self.measurements,
            &self.field_dimensions,
        )
    }
}
