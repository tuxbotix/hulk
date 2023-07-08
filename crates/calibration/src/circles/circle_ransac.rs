use itertools::Itertools;
use nalgebra::{point, ComplexField, Point2, RealField};
use rand::{rngs::StdRng, seq::SliceRandom, thread_rng, SeedableRng};

use super::circle_fitting_model::{CircleFittingModel, GenericCircle};

#[derive(Default, Debug, PartialEq)]
pub struct RansacResultCircle<T>
where
    T: ComplexField + Copy + RealField,
{
    pub output: Option<super::circle_fitting_model::GenericCircle<T>>,
    pub used_points: Vec<Point2<T>>,
}

pub struct RansacCircleWithRadius<T>
where
    T: ComplexField + Copy + RealField,
{
    circle_fitting_model: CircleFittingModel<T>,
    radius: T,
    pub unused_points: Vec<Point2<T>>,
    random_number_generator: StdRng,
}

impl<T> RansacCircleWithRadius<T>
where
    T: ComplexField + Copy + RealField,
{
    pub fn new(circle_fitting_model: CircleFittingModel<T>, unused_points: Vec<Point2<T>>) -> Self {
        Self {
            circle_fitting_model,
            radius: circle_fitting_model.candidate_circle.radius,
            unused_points,
            random_number_generator: StdRng::from_rng(thread_rng())
                .expect("Failed to create random number generator"),
        }
    }
}

impl<T> RansacCircleWithRadius<T>
where
    T: ComplexField + Copy + RealField,
{
    pub fn next_candidate(
        &mut self,
        iterations: usize,
        radius_variance: T,
    ) -> RansacResultCircle<T> {
        if self.unused_points.len() < 2 {
            return RansacResultCircle::<T> {
                output: None,
                used_points: vec![],
            };
        }
        let (best_candidate_model, inlier_count) = (0..iterations)
            .map(|_| {
                let three_points = self
                    .unused_points
                    .choose_multiple(&mut self.random_number_generator, 3)
                    .collect_vec();

                // TODO discard bad circles early?
                let model = CircleFittingModel {
                    candidate_circle: Self::circle_from_three_points(
                        three_points[0],
                        three_points[1],
                        three_points[2],
                    ),
                    centre_distance_penalty_threshold: self
                        .circle_fitting_model
                        .centre_distance_penalty_threshold,
                };

                // If the radius isn't within 30% of the radius, this is bad!
                if model.candidate_circle.radius - self.radius
                    > self.radius * T::from_f64(0.3).unwrap()
                {
                    return (model, 0);
                }

                let score = model.get_inlier_count(
                    &model.circle_fit_residual(&self.unused_points),
                    radius_variance,
                );

                (model, score)
            })
            .max_by_key(|scored_circle| scored_circle.1)
            .expect("max_by_key erroneously returned no result");

        let best_candidate_residual = best_candidate_model.circle_fit_residual(&self.unused_points);

        let (used_points, unused_points): (Vec<Point2<T>>, Vec<Point2<T>>) =
            best_candidate_residual
                .iter()
                .zip(&self.unused_points)
                .partition_map(|(residual_value, point)| {
                    if best_candidate_model.is_inlier(*residual_value, radius_variance) {
                        itertools::Either::Left(point)
                    } else {
                        itertools::Either::Right(point)
                    }
                });

        self.unused_points = unused_points;
        RansacResultCircle::<_> {
            output: Some(best_candidate_model.candidate_circle),
            used_points,
        }
    }

    fn circle_from_three_points(a: &Point2<T>, b: &Point2<T>, c: &Point2<T>) -> GenericCircle<T> {
        let two_t = T::from_f64(2.0).unwrap();

        // Let points be a, b, c
        let ba_diff = b - a;
        let cb_diff = c - b;
        let ab_mid = (a.coords + b.coords) / two_t;
        let bc_mid = (b.coords + c.coords) / two_t;

        let ab_perpendicular_slope = -(ba_diff.x / ba_diff.y);
        let bc_perpendicular_slope = -(cb_diff.x / cb_diff.y);

        // using y - y1 = m (x - x1) form, get x and y where centre is intersection of ab & bc perpendicular bisectors!
        let centre_x = ((bc_mid.y - ab_mid.y) + (ab_perpendicular_slope * ab_mid.x)
            - (bc_perpendicular_slope * bc_mid.x))
            / (ab_perpendicular_slope - bc_perpendicular_slope);

        let centre_y = ab_perpendicular_slope * (centre_x - ab_mid.x) + ab_mid.y;
        let centre = point![centre_x, centre_y];
        let radius = (a - centre).norm();

        GenericCircle { centre, radius }
    }
}

#[cfg(test)]
mod test {

    use approx::assert_relative_eq;
    use nalgebra::{point, Point2};
    use rand::{rngs::StdRng, SeedableRng};

    use crate::circles::{
        circle_fitting_model::{generate_circle, CircleFittingModel, GenericCircle},
        circle_ransac::RansacResultCircle,
    };

    use super::RansacCircleWithRadius;

    type T = f64;

    const TYPICAL_RADIUS: T = 0.75;
    const PENALTY_THRESHOLD: T = 10.0;

    fn ransac_circle_with_seed(
        unused_points: Vec<Point2<T>>,
        seed: u64,
        radius: T,
        distance_penalty_threshold: T,
    ) -> RansacCircleWithRadius<T> {
        RansacCircleWithRadius::<T> {
            unused_points,
            random_number_generator: StdRng::seed_from_u64(seed),
            circle_fitting_model: CircleFittingModel {
                candidate_circle: GenericCircle {
                    radius: radius,
                    centre: point![1.0, 0.0],
                },
                centre_distance_penalty_threshold: distance_penalty_threshold,
            },
            radius: radius,
        }
    }

    #[test]
    fn ransac_empty_input() {
        let mut ransac = ransac_circle_with_seed(vec![], 0, TYPICAL_RADIUS, PENALTY_THRESHOLD);
        assert_eq!(
            ransac.next_candidate(10, 5.0),
            RansacResultCircle::<T>::default()
        );
    }

    #[test]
    fn ransac_single_point() {
        let mut ransac =
            ransac_circle_with_seed(vec![point![5.0, 5.0]], 0, TYPICAL_RADIUS, PENALTY_THRESHOLD);
        assert_eq!(
            ransac.next_candidate(10, 5.0),
            RansacResultCircle::<T>::default()
        );
    }

    #[test]
    fn three_point_circle_equation_test() {
        let centre = point![2.0, 1.5];
        let radius = TYPICAL_RADIUS;
        let angles = [10.0, 45.0, 240.0];

        let points: Vec<_> = angles
            .iter()
            .map(|a: &T| {
                let angle_radian = a.to_radians();
                point![
                    radius * a.cos() + centre.coords.x,
                    radius * a.sin() + centre.coords.y
                ]
            })
            .collect();

        let circle = RansacCircleWithRadius::<T>::circle_from_three_points(
            &points[0], &points[1], &points[2],
        );
        assert_relative_eq!(circle.centre, centre, epsilon = 1e-10);
    }

    #[test]
    fn ransac_circle_three_points() {
        let centre = point![2.0, 1.5];
        let radius = TYPICAL_RADIUS;
        let angles = [10.0, 45.0, 240.0];

        let points: Vec<_> = angles
            .iter()
            .map(|a: &T| {
                let angle_radian = a.to_radians();
                point![
                    radius * a.cos() + centre.coords.x,
                    radius * a.sin() + centre.coords.y
                ]
            })
            .collect();

        let mut ransac =
            ransac_circle_with_seed(points.clone(), 0, TYPICAL_RADIUS, PENALTY_THRESHOLD);
        let result = ransac.next_candidate(10, 5.0);

        let out_circle = result.output.expect("No circle found");

        assert_eq!(points.len(), result.used_points.len());
        assert_relative_eq!(out_circle.centre, centre, epsilon = 1e-10);
        assert_relative_eq!(out_circle.radius, TYPICAL_RADIUS, epsilon = 1e-10);
        assert_relative_eq!(result.used_points[0], points[0]);
        assert_relative_eq!(result.used_points[1], points[1]);
    }

    #[test]
    fn ransac_perfect_circle() {
        let slope = 5.3;
        let y_intercept = -83.1;

        let centre = point![2.0, 1.5];
        let radius = TYPICAL_RADIUS;
        let points: Vec<Point2<T>> = generate_circle(&centre, 100, radius, 0.0, 0);

        let mut ransac =
            ransac_circle_with_seed(points.clone(), 0, TYPICAL_RADIUS, PENALTY_THRESHOLD);
        let result = ransac.next_candidate(15, 0.1);
        let output = result.output.expect("No circle was found");
        assert_relative_eq!(output.centre, centre, epsilon = 0.0001);
        assert_relative_eq!(output.radius, radius, epsilon = 0.0001);
        assert_eq!(result.used_points, points);
    }
}
