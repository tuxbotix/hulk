use levenberg_marquardt::LeastSquaresProblem;
use nalgebra::{allocator::Allocator, DefaultAllocator, Dyn, Owned};

use types::field_dimensions::FieldDimensions;

use crate::{
    corrections::{CorrectionsTrait, ExtrinsicCorrections, Parameters},
    jacobian::{jacobian_central_difference, Jacobian, JacobianStorage},
    residuals::{
        calculate_residuals_from_parameters, CalculateResiduals, ResidualVector,
        ResidualVectorStorage,
    },
};

pub struct CalibrationProblem<MeasurementResidualsType>
where
    MeasurementResidualsType: CalculateResiduals,
    MeasurementResidualsType::Corrections: CorrectionsTrait,
    DefaultAllocator:
        Allocator<<MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount>,
{
    parameters: MeasurementResidualsType::Corrections,
    measurements: Vec<MeasurementResidualsType::Measurement>,
    field_dimensions: FieldDimensions,
}

impl<MeasurementResidualsType> CalibrationProblem<MeasurementResidualsType>
where
    MeasurementResidualsType: CalculateResiduals,
    MeasurementResidualsType::Corrections: Copy + CorrectionsTrait,
    DefaultAllocator:
        Allocator<<MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount>,
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

    pub fn get_corrections(&self) -> ExtrinsicCorrections {
        self.parameters.extrinsic_corrections()
    }

    pub fn get_all_corrections(&self) -> MeasurementResidualsType::Corrections {
        self.parameters
    }
}

impl<MeasurementResidualsType>
    LeastSquaresProblem<
        f32,
        Dyn,
        <MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount,
    > for CalibrationProblem<MeasurementResidualsType>
where
    MeasurementResidualsType: CalculateResiduals,
    Vec<f32>: From<MeasurementResidualsType>,
    MeasurementResidualsType::Corrections: CorrectionsTrait,
    DefaultAllocator:
        Allocator<<MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount>,
{
    type ResidualStorage = ResidualVectorStorage;
    type JacobianStorage = JacobianStorage<
        <MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount,
    >;
    type ParameterStorage =
        Owned<f32, <MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount>;

    fn set_params(
        &mut self,
        parameters: &Parameters<
            <MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount,
        >,
    ) {
        self.parameters = MeasurementResidualsType::Corrections::from_svector(parameters);
    }

    fn params(
        &self,
    ) -> Parameters<<MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount>
    {
        self.parameters.to_svector()
    }

    fn residuals(&self) -> Option<ResidualVector> {
        calculate_residuals_from_parameters::<MeasurementResidualsType>(
            &self.parameters,
            &self.measurements,
            &self.field_dimensions,
        )
    }

    fn jacobian(
        &self,
    ) -> Option<Jacobian<<MeasurementResidualsType::Corrections as CorrectionsTrait>::ParameterCount>>
    {
        jacobian_central_difference::<MeasurementResidualsType>(
            &self.parameters,
            &self.measurements,
            &self.field_dimensions,
        )
    }
}

// pub trait ProblemWrapper{

//     type MeasurementResidualsType;
//     // type PARAMETER_COUNT;
//     type

//     fn new_problem<MeasurementResidualsType>(
//         initial_corrections: MeasurementResidualsType::Corrections,
//         measurements: Vec<MeasurementResidualsType::Measurement>,
//         field_dimensions: FieldDimensions,
//     ){

//             CalibrationProblem::<MeasurementResidualsType, PARAMETER_COUNT>::new(
//                 initial_corrections,
//                 measurements.clone(),
//                 field_dimensions,
//             )
//         }

// }
