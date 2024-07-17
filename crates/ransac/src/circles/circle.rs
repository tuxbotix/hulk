use std::f32::consts::FRAC_PI_4;

use itertools::Itertools;
use ordered_float::NotNan;
use rand::{seq::SliceRandom, Rng};

use geometry::circle::Circle;
use linear_algebra::{point, Point2};

struct Parameters {
    radius: f32,
    inlier_threshold_on_residual: f32,
    minimum_furthest_points_distance_squared: f32,
}

#[derive(Default, Debug, PartialEq)]
pub struct RansacResultCircle<Frame> {
    pub circle: Circle<Frame>,
    pub used_points: Vec<Point2<Frame>>,
    pub score: f32,
}

pub struct RansacCircle<Frame> {
    pub unused_points: Vec<Point2<Frame>>,
    parameters: Parameters,
}

impl<Frame> RansacCircle<Frame> {
    pub fn new(
        radius: f32,
        accepted_radius_variance: f32,
        unused_points: Vec<Point2<Frame>>,
    ) -> Self {
        const MINIMUM_ANGLE_OF_ARC: f32 = FRAC_PI_4;
        let minimum_furthest_points_distance =
            compute_minimum_point_distance(MINIMUM_ANGLE_OF_ARC, radius);
        Self {
            unused_points,
            parameters: Parameters {
                radius,
                inlier_threshold_on_residual: accepted_radius_variance,
                minimum_furthest_points_distance_squared: minimum_furthest_points_distance.powi(2),
            },
        }
    }

    pub fn next_candidate(
        &mut self,
        random_number_generator: &mut impl Rng,
        iterations: usize,
    ) -> Option<RansacResultCircle<Frame>> {
        let best_candidate_model_option = get_best_candidate(
            &self.unused_points,
            iterations,
            &self.parameters,
            random_number_generator,
        );

        if let Some((candidate_circle, inliers_mask, score)) = best_candidate_model_option {
            let (used_points, unused_points) = inliers_mask
                .into_iter()
                .zip(&self.unused_points)
                .partition_map(|(is_inlier, point)| {
                    if is_inlier {
                        itertools::Either::Left(point)
                    } else {
                        itertools::Either::Right(point)
                    }
                });

            self.unused_points = unused_points;

            Some(RansacResultCircle::<Frame> {
                circle: candidate_circle,
                used_points,
                score,
            })
        } else {
            None
        }
    }
}

/// This method allows to find circles by transforming to another frame (i.e. Ground).
/// Otherwise, an ellipse fitting method should be used.
pub struct RansacCircleWithTransformation<OriginalFrame, SearchFrame> {
    parameters: Parameters,
    pub unused_points_original: Vec<Point2<OriginalFrame>>,
    unused_points_transformed: Vec<Point2<SearchFrame>>,
}

#[derive(Default, Debug, PartialEq, Clone)]
pub struct RansacResultCircleWithTransformation<OriginalFrame, SearchFrame> {
    pub circle: Circle<SearchFrame>,
    pub used_points_original: Vec<Point2<OriginalFrame>>,
    pub used_points_transformed: Vec<Point2<SearchFrame>>,
    pub score: f32,
}

impl<OriginalFrame, SearchFrame> RansacCircleWithTransformation<OriginalFrame, SearchFrame> {
    pub fn new(
        radius: f32,
        accepted_radius_variance: f32,
        points: Vec<Point2<OriginalFrame>>,
        transformer_function: impl Fn(&Point2<OriginalFrame>) -> Option<Point2<SearchFrame>>,
    ) -> Self {
        const MINIMUM_ANGLE_OF_ARC: f32 = FRAC_PI_4;
        let minimum_furthest_points_distance =
            compute_minimum_point_distance(MINIMUM_ANGLE_OF_ARC, radius);

        let (unused_points_original, unused_points_transformed) = points
            .iter()
            .filter_map(|point| {
                let output = transformer_function(point);
                output.map(|transformed| (point, transformed))
            })
            .unzip();

        Self {
            unused_points_original,
            unused_points_transformed,

            parameters: Parameters {
                radius,
                inlier_threshold_on_residual: accepted_radius_variance,
                minimum_furthest_points_distance_squared: minimum_furthest_points_distance.powi(2),
            },
        }
    }

    pub fn next_candidate(
        &mut self,
        random_number_generator: &mut impl Rng,
        iterations: usize,
    ) -> Option<RansacResultCircleWithTransformation<OriginalFrame, SearchFrame>> {
        let best_candidate_model_option = get_best_candidate::<SearchFrame>(
            &self.unused_points_transformed,
            iterations,
            &self.parameters,
            random_number_generator,
        );
        best_candidate_model_option.map(|(candidate_circle, inliers_mask, score)| {
            let (used_points_transformed, unused_points_transformed) = inliers_mask
                .iter()
                .zip(&self.unused_points_transformed)
                .partition_map(|(&is_inlier, point)| {
                    if is_inlier {
                        itertools::Either::Left(point)
                    } else {
                        itertools::Either::Right(point)
                    }
                });

            let (used_points_original, unused_points_original) = inliers_mask
                .into_iter()
                .zip(&self.unused_points_original)
                .partition_map(|(is_inlier, point)| {
                    if is_inlier {
                        itertools::Either::Left(point)
                    } else {
                        itertools::Either::Right(point)
                    }
                });

            self.unused_points_original = unused_points_original;
            self.unused_points_transformed = unused_points_transformed;

            RansacResultCircleWithTransformation {
                circle: candidate_circle,
                used_points_original,
                used_points_transformed,
                score,
            }
        })
    }
}

fn get_best_candidate<Frame>(
    src_unused_points: &[Point2<Frame>],
    iterations: usize,
    parameters: &Parameters,
    random_number_generator: &mut impl Rng,
) -> Option<(Circle<Frame>, Vec<bool>, f32)> {
    let src_point_count = src_unused_points.len();
    if src_point_count < 3 {
        return None;
    }

    let sampled_population_size = {
        let min_sampled_population_size = 100;
        let sample_size_fraction = 0.15;
        let candidate_sample_size = src_unused_points.len() as f32 * sample_size_fraction;
        if candidate_sample_size > min_sampled_population_size as f32 {
            candidate_sample_size as usize
        } else {
            src_point_count
        }
    };

    let radius_squared = parameters.radius.powi(2);

    let best = (0..iterations)
        .filter_map(|_| {
            let unused_points = src_unused_points
                .choose_multiple(random_number_generator, sampled_population_size)
                .collect_vec();
            let three_points = &unused_points[0..3];

            let point1 = three_points[0];
            let point2 = three_points[1];
            let point3 = three_points[2];
            let ab_squared = (*point1 - *point2).norm_squared();
            let bc_squared = (*point2 - *point3).norm_squared();
            let ca_squared = (*point3 - *point1).norm_squared();
            if ab_squared < parameters.minimum_furthest_points_distance_squared
                && bc_squared < parameters.minimum_furthest_points_distance_squared
                && ca_squared < parameters.minimum_furthest_points_distance_squared
            {
                return None;
            }
            let candidate_circle = circle_from_three_points(point1, point2, point3);
            let initial_max_variance = 1.5 * parameters.inlier_threshold_on_residual;
            if (candidate_circle.radius - parameters.radius).powi(2) > initial_max_variance {
                return None;
            }

            let score = unused_points
                .iter()
                .filter_map(|&&point| {
                    let distance_squared = (point - candidate_circle.center).norm_squared();
                    let residual_abs = (distance_squared - radius_squared).abs();
                    let is_inlier = residual_abs <= parameters.inlier_threshold_on_residual;
                    if is_inlier {
                        Some(1.0 - (residual_abs / parameters.inlier_threshold_on_residual))
                    } else {
                        None
                    }
                })
                .sum::<f32>();

            Some((candidate_circle, score as f32))
        })
        .max_by_key(|scored_circle| NotNan::new(scored_circle.1).unwrap_or_default());

    best.map(|(circle, _score)| {
        let mut score = 0.0;
        let center = circle.center;

        let inlier_points_mask = src_unused_points
            .iter()
            .map(|&point| {
                let distance_squared = (point - center).norm_squared();
                let residual_abs = (distance_squared - radius_squared).abs();
                let is_inlier = residual_abs <= parameters.inlier_threshold_on_residual;
                if is_inlier {
                    score += 1.0 - (residual_abs / parameters.inlier_threshold_on_residual);
                }
                is_inlier
            })
            .collect_vec();

        (circle, inlier_points_mask, score / src_point_count as f32)
    })
}

fn circle_from_three_points<Frame>(
    a: &Point2<Frame>,
    b: &Point2<Frame>,
    c: &Point2<Frame>,
) -> Circle<Frame> {
    let ba_diff = *b - *a;
    let cb_diff = *c - *b;
    let ab_mid = (a.coords() + b.coords()) / 2.0;
    let bc_mid = (b.coords() + c.coords()) / 2.0;

    let ab_perpendicular_slope = -(ba_diff.x() / ba_diff.y());
    let bc_perpendicular_slope = -(cb_diff.x() / cb_diff.y());

    // Center is the intersection point of perpendicular bisectors of lines ab, bc lines.
    let center_x = ((bc_mid.y() - ab_mid.y()) + (ab_perpendicular_slope * ab_mid.x())
        - (bc_perpendicular_slope * bc_mid.x()))
        / (ab_perpendicular_slope - bc_perpendicular_slope);

    let center_y = ab_perpendicular_slope * (center_x - ab_mid.x()) + ab_mid.y();
    let center = point![center_x, center_y];
    let radius = (a.coords() - center.coords()).norm();

    Circle { center, radius }
}

/// Calculates distance between two points a, b on a circle,
/// based on the angle `alpha` (`angle_at_center_to_points`) between ac & cb lines where c is the center.
/// For radius r, length of line ab = sin(alpha/2) * 2 * r
fn compute_minimum_point_distance(angle_at_center_to_points: f32, radius: f32) -> f32 {
    (angle_at_center_to_points / 2.0).sin() * 2.0 * radius
}

#[cfg(test)]
mod test {

    use super::RansacCircle;
    use crate::circles::{circle::circle_from_three_points, test_utilities::generate_circle};
    use approx::assert_relative_eq;
    use linear_algebra::{point, Point2};
    use rand::SeedableRng;
    use rand_chacha::ChaChaRng;

    const TYPICAL_RADIUS: f32 = 0.75;
    const ACCEPTED_RADIUS_VARIANCE: f32 = 0.1;
    const REL_ASSERT_EPSILON: f32 = 1e-5;

    #[derive(Debug, Clone, PartialEq, Eq, Default)]
    struct SomeFrame;

    #[test]
    fn ransac_empty_input() {
        let mut rng = ChaChaRng::from_entropy();
        let mut ransac =
            RansacCircle::<SomeFrame>::new(TYPICAL_RADIUS, ACCEPTED_RADIUS_VARIANCE, vec![]);
        assert_eq!(ransac.next_candidate(&mut rng, 10), None);
    }

    #[test]
    fn ransac_single_point() {
        let mut rng = ChaChaRng::from_entropy();
        let mut ransac = RansacCircle::<SomeFrame>::new(
            TYPICAL_RADIUS,
            ACCEPTED_RADIUS_VARIANCE,
            vec![point![5.0, 5.0]],
        );
        assert_eq!(ransac.next_candidate(&mut rng, 10), None);
    }

    #[test]
    fn three_point_circle_equation_test() {
        let center = point![2.0, 1.5];
        let radius = TYPICAL_RADIUS;
        let angles = [10.0, 45.0, 240.0];

        let points: Vec<_> = angles
            .iter()
            .map(|a: &f32| point![radius * a.cos() + center.x(), radius * a.sin() + center.y()])
            .collect();

        let circle = circle_from_three_points::<SomeFrame>(&points[0], &points[1], &points[2]);
        assert_relative_eq!(circle.center, center, epsilon = REL_ASSERT_EPSILON);
    }

    #[test]
    fn ransac_circle_three_points() {
        let center = point![2.0, 1.5];
        let radius = TYPICAL_RADIUS;
        let angles = [10.0, 45.0, 240.0];

        let points: Vec<_> = angles
            .iter()
            .map(|a: &f32| point![radius * a.cos() + center.x(), radius * a.sin() + center.y()])
            .collect();
        let mut rng = ChaChaRng::from_entropy();
        let mut ransac = RansacCircle::<SomeFrame>::new(
            TYPICAL_RADIUS,
            ACCEPTED_RADIUS_VARIANCE,
            points.clone(),
        );
        let result = ransac
            .next_candidate(&mut rng, 10)
            .expect("No circle found");

        let detected_circle = result.circle;

        assert_eq!(points.len(), result.used_points.len());
        assert_relative_eq!(detected_circle.center, center, epsilon = REL_ASSERT_EPSILON);
        assert_relative_eq!(
            detected_circle.radius,
            TYPICAL_RADIUS,
            epsilon = REL_ASSERT_EPSILON
        );

        assert_relative_eq!(result.used_points[0], points[0]);
        assert_relative_eq!(result.used_points[1], points[1]);
    }

    #[test]
    fn ransac_perfect_circle() {
        let center = point![2.0, 1.5];
        let radius = TYPICAL_RADIUS;
        let points: Vec<Point2<SomeFrame>> = generate_circle(&center, 100, radius, 0.0, 0);
        let mut rng = ChaChaRng::from_entropy();
        let mut ransac = RansacCircle::<SomeFrame>::new(
            TYPICAL_RADIUS,
            ACCEPTED_RADIUS_VARIANCE,
            points.clone(),
        );
        let result = ransac
            .next_candidate(&mut rng, 15)
            .expect("No circle was found");
        let detected_circle = result.circle;
        assert_relative_eq!(detected_circle.center, center, epsilon = 0.0001);
        assert_relative_eq!(detected_circle.radius, radius, epsilon = 0.0001);
        assert_eq!(result.used_points, points);
    }
}
