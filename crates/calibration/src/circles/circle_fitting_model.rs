use itertools::Itertools;
use nalgebra::{point, ComplexField, Dyn, OVector, Point2, RealField};
use rand::{prelude::StdRng, SeedableRng};
use rand_distr::{Distribution, Uniform};
use types::Circle;

// TODO switch to types::geometry::Circle
#[derive(Default, Debug, Copy, Clone, PartialEq)]
pub struct GenericCircle<T>
where
    T: ComplexField + Copy + Clone,
{
    pub radius: T,
    pub centre: Point2<T>,
}

impl From<GenericCircle<f32>> for Circle {
    fn from(value: GenericCircle<f32>) -> Self {
        Circle {
            center: value.centre,
            radius: value.radius,
        }
    }
}

impl From<Circle> for GenericCircle<f32> {
    fn from(value: Circle) -> Self {
        GenericCircle::<f32> {
            centre: value.center,
            radius: value.radius,
        }
    }
}

#[derive(Default, Debug, Copy, Clone)]
pub struct CircleFittingModel<T>
where
    T: ComplexField + Copy + Clone,
{
    pub candidate_circle: GenericCircle<T>,
    pub centre_distance_penalty_threshold: T,
}

impl<T> CircleFittingModel<T>
where
    T: ComplexField + Copy + RealField,
{
    // const CENTRE_DISTANCE_PENALTY_THRESHOLD: T = 10.0.into();
    pub fn circle_fit_residual(&self, points: &Vec<Point2<T>>) -> OVector<T, Dyn> {
        let centre_distance_to_origin = self.candidate_circle.centre.coords.norm();
        let radius_threshold = self.centre_distance_penalty_threshold.powi(2);
        let distance_penalty = (centre_distance_to_origin - radius_threshold).max(T::zero());

        let mut output_residual = OVector::<T, Dyn>::zeros(points.len());

        for (out_elem, point) in output_residual.iter_mut().zip(points.iter()) {
            let difference =
                (point - self.candidate_circle.centre).norm() - self.candidate_circle.radius;

            *out_elem = difference + difference.signum() * distance_penalty;
        }
        output_residual
    }

    // pub fn circle_fit_residual_derivative(
    //     &self,
    //     centre_coordinate_index: usize,
    //     points: Vec<Point2<T>>,
    // ) -> OVector<T, Dyn> {
    //     let centre_distance_to_origin = self.candidate_circle.centre.coords.norm();
    //     let distance_penalty_derivative: T = if centre_distance_to_origin
    //         < (self.candidate_circle.radius + self.centre_distance_penalty_threshold).powi(2)
    //     {
    //         T::zero()
    //     } else {
    //         // coords.norm(); -> sqrt( coord_x^2 + coord_y^2 ) -> ay/ax = 0.5(coord_x^2 + coord_y^2 )(2 coord_x)
    //         let centre_coords = &self.candidate_circle.centre.coords;

    //         T::from_f64(0.5).unwrap()
    //             * centre_coords.norm()
    //             * (T::from_f64(2.0).unwrap() * centre_coords[centre_coordinate_index])
    //     };

    //     OVector::<T, Dyn>::from_iterator(
    //         points.len(),
    //         points.iter().map(|point| {
    //             point[centre_coordinate_index]
    //                 - self.candidate_circle.centre[centre_coordinate_index]
    //         }),
    //     ) * T::from_f64(2.0).unwrap()
    // }

    pub fn get_inlier_count(&self, residuals: &OVector<T, Dyn>, radius_variance: T) -> usize {
        residuals.iter().fold(0, |accum, residual| {
            if residual.abs() <= radius_variance {
                accum + 1
            } else {
                accum
            }
        })
    }

    #[inline]
    pub fn is_inlier(&self, single_residual_value: T, radius_variance: T) -> bool {
        single_residual_value.abs() <= radius_variance
    }
}

pub fn generate_circle<T>(
    circle_centre: &Point2<T>,
    point_count: usize,
    circle_radius: T,
    circle_radius_variance: T,
    random_seed: u64,
) -> Vec<Point2<T>>
where
    T: ComplexField + Copy + RealField + rand_distr::uniform::SampleUniform,
{
    let angle_range = Uniform::from(-T::pi()..T::pi());

    let random_number_generator = StdRng::seed_from_u64(random_seed);

    let randomized_angles_iter = angle_range
        .sample_iter(random_number_generator.clone())
        .take(point_count);

    let randomized_radiuses = if circle_radius_variance.abs() <= T::default_epsilon() {
        vec![circle_radius; point_count]
    } else {
        let radius_range = Uniform::from(
            (circle_radius - circle_radius_variance)..(circle_radius + circle_radius_variance),
        );

        radius_range
            .sample_iter(random_number_generator)
            .take(point_count)
            .collect_vec()
    };

    let circle_points_iter =
        randomized_angles_iter
            .zip(randomized_radiuses.iter())
            .map(|(angle, radius)| {
                point![
                    (angle.cos() * *radius) + circle_centre.coords.x,
                    (angle.sin() * *radius) + circle_centre.coords.y
                ]
            });

    let out_vec = circle_points_iter.collect_vec();

    for point in &out_vec {
        let percieved_radius = (circle_centre.coords - point.coords).norm();
        assert!(
            (percieved_radius - circle_radius).abs()
                <= circle_radius_variance + T::from_f64(1e-5).unwrap()
        );
    }

    out_vec
}

#[cfg(test)]
mod tests {
    use crate::circles::circle_fitting_model::{
        generate_circle, CircleFittingModel, GenericCircle,
    };
    use nalgebra::point;

    type T = f64;
    const RADIUS: T = 0.75;
    const CENTRE_DISTANCE_PENALTY_THRESHOLD: T = 10.0;
    const SEED: u64 = 0;

    #[test]
    fn test_residual_calculation() {
        const POINT_COUNT: usize = 20;
        let radius_variance = 0.0;

        let circle_fiting_model = CircleFittingModel::<T> {
            candidate_circle: GenericCircle::<T> {
                centre: point![2.0, 4.0],
                radius: RADIUS,
            },
            centre_distance_penalty_threshold: CENTRE_DISTANCE_PENALTY_THRESHOLD,
        };

        let circle_points_ground = generate_circle(
            &circle_fiting_model.candidate_circle.centre,
            POINT_COUNT,
            circle_fiting_model.candidate_circle.radius,
            radius_variance,
            SEED,
        );

        // Since we give points fitting perfectly to a circle of the given radius, this residual should be nearly zero!
        let residual = circle_fiting_model.circle_fit_residual(&circle_points_ground);

        println!(
            "Circle centre: {:?}, circle radius: {:?} circle points: {:?}",
            circle_fiting_model.candidate_circle.centre,
            circle_fiting_model.candidate_circle.radius,
            circle_points_ground
        );
        println!("Residual -> should be close to 0 {:?}", residual);
        assert!(residual.norm() < 1e-6);
    }

    #[test]
    fn test_residual_calculation_with_variance() {
        const POINT_COUNT: usize = 20;
        let radius_variance = 0.1;

        let circle_fiting_model = CircleFittingModel::<T> {
            candidate_circle: GenericCircle::<T> {
                centre: point![2.0, 4.0],
                radius: RADIUS,
            },
            centre_distance_penalty_threshold: CENTRE_DISTANCE_PENALTY_THRESHOLD,
        };

        let circle_points_ground = generate_circle(
            &circle_fiting_model.candidate_circle.centre,
            POINT_COUNT,
            circle_fiting_model.candidate_circle.radius,
            radius_variance,
            SEED,
        );

        let residuals = circle_fiting_model.circle_fit_residual(&circle_points_ground);

        println!(
            "Circle centre: {:?}, circle radius: {:?} circle points: {:?}",
            circle_fiting_model.candidate_circle.centre,
            circle_fiting_model.candidate_circle.radius,
            circle_points_ground
        );
        println!(
            "Residuals should be within {:?}  residual {:?}",
            radius_variance, residuals
        );

        // Residual is square of the distance difference!

        let count = circle_fiting_model.get_inlier_count(&residuals, radius_variance);

        println!(
            "Residual and inlier count: {:?}, {:?}, {:?}%",
            count,
            radius_variance,
            (count as T / residuals.len() as T * 100.0)
        );

        assert_eq!(count, residuals.len());

        // assert!(residual.norm() < 1e-6);
    }
}
