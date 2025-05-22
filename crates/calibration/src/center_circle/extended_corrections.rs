use nalgebra::{Const, SVector};

use crate::corrections::{CorrectionsTrait, ExtrinsicCorrections, AMOUNT_OF_EXTRINSIC_PARAMETERS};

pub const EXTENDED_AMOUNT_OF_PARAMETERS: usize = AMOUNT_OF_EXTRINSIC_PARAMETERS + 1;

#[derive(Clone, Copy, Debug, Default)]
pub struct ExtendedCorrections {
    pub primary_corrections: ExtrinsicCorrections,
    pub radius_compensation: f32,
}

impl CorrectionsTrait for ExtendedCorrections {
    fn to_svector(&self) -> SVector<f32, EXTENDED_AMOUNT_OF_PARAMETERS> {
        let mut vector = SVector::<f32, EXTENDED_AMOUNT_OF_PARAMETERS>::zeros();
        vector
            .fixed_view_mut::<AMOUNT_OF_EXTRINSIC_PARAMETERS, 1>(0, 0)
            .copy_from(&self.primary_corrections.to_svector());
        vector[EXTENDED_AMOUNT_OF_PARAMETERS - 1] = self.radius_compensation;
        vector
    }
    fn from_svector(vector: &SVector<f32, EXTENDED_AMOUNT_OF_PARAMETERS>) -> Self {
        Self {
            primary_corrections: ExtrinsicCorrections::from_svector(
                &vector
                    .fixed_view::<AMOUNT_OF_EXTRINSIC_PARAMETERS, 1>(0, 0)
                    .clone_owned(),
            ),
            radius_compensation: vector[EXTENDED_AMOUNT_OF_PARAMETERS - 1],
        }
    }

    fn extrinsic_corrections(self) -> ExtrinsicCorrections {
        self.primary_corrections.extrinsic_corrections()
    }

    type ParameterCount = Const<EXTENDED_AMOUNT_OF_PARAMETERS>;
}
