pub mod circle_points;
pub mod extended_corrections;
pub mod measurement;
pub mod residuals;

#[cfg(test)]
mod tests {
    use coordinate_systems::{Ground, Pixel};
    use geometry::rectangle::Rectangle;
    use itertools::Itertools;
    use linear_algebra::{point, vector, IntoTransform, Point2, Rotation3};
    use nalgebra::{Translation, UnitQuaternion};
    use projection::{camera_matrix::CameraMatrix, Projection};
    use types::{camera_position::CameraPosition, field_dimensions::FieldDimensions};

    use crate::{
        center_circle::extended_corrections::ExtendedCorrections, residuals::CalculateResiduals,
        solve,
    };

    use super::{
        circle_points::CenterCirclePoints, 
        measurement::Measurement, residuals::CenterCircleResiduals,
    };

    fn get_matrix() -> CameraMatrix {
        let focal_length = nalgebra::vector![0.95, 1.27];
        let optical_center = nalgebra::point![0.5, 0.5];

        // head to camera
        // [ 3.14159265 -1.54985242 -1.57079633]
        // -7.45058060e-09
        //  6.48555607e-02
        // -5.73643520e-02
        // robot to head
        // [ 0.00926836 -0.17876333 -0.05207865]
        //  3.76568325e-02
        //  5.82076609e-11
        // -2.08120674e-01
        // ground to robot
        // [ 1.41449154e-05 -5.48669267e-02 -7.75698686e-07]

        CameraMatrix::from_normalized_focal_and_center(
            focal_length,
            optical_center,
            vector![640.0, 480.0],
            nalgebra::Isometry3 {
                rotation: UnitQuaternion::from_euler_angles(
                    1.41449154e-05,
                    -5.48669267e-02,
                    -7.75698686e-07,
                ),
                translation: Translation::from(nalgebra::point![
                    1.26435421e-02,
                    2.27373675e-13,
                    -2.29651436e-01,
                ]),
            }
            .framed_transform(),
            nalgebra::Isometry3 {
                rotation: UnitQuaternion::from_euler_angles(0.00926836, -0.17876333, -0.05207865),
                translation: Translation::from(nalgebra::point![
                    3.76568325e-02,
                    5.82076609e-11,
                    -2.08120674e-01,
                ]),
            }
            .framed_transform(),
            nalgebra::Isometry3 {
                rotation: UnitQuaternion::from_euler_angles(3.14159265, -1.54985242, -1.57079633),
                translation: Translation::from(nalgebra::point![
                    -7.45058060e-09,
                    6.48555607e-02,
                    -5.73643520e-02
                ]),
            }
            .framed_transform(),
        )
    }

    #[test]
    fn calibrate_with_circle() {
        // let robot_distortion_angles = [1.5f32, -2.0f32, 1.0f32].map(|a| a.to_radians());
        let top_distortion_angles = [-4.0f32, 2.5f32, 1.2f32].map(|a| a.to_radians());

        let matrix = get_matrix();
        // println!("hoei {}", matrix.horizon);

        let distorted_matrix = get_matrix().to_corrected(
            Rotation3::from_euler_angles(
                0.0, 0.0,
                0.0, // robot_distortion_angles[0],
                    // robot_distortion_angles[1],
                    // robot_distortion_angles[2],
            ),
            Rotation3::from_euler_angles(
                top_distortion_angles[0],
                top_distortion_angles[1],
                top_distortion_angles[2],
                // 0.0, 0.0, 0.0,
            ),
        );

        let radius = 0.75;
        let field_dims = FieldDimensions {
            center_circle_diameter: radius * 2.0,
            line_width: 0.05,
            ..Default::default()
        };

        let center_ground: Point2<Ground> = point![1.5, 0.0];
        let center_pixel: Point2<Pixel> = matrix.ground_to_pixel(center_ground).unwrap();
        // let center_pixel: Point2<Pixel> =
        // point![matrix.image_size.x() / 2.0, matrix.image_size.y() * 0.65];
        // let center_ground: Point2<Ground> = matrix.pixel_to_ground(center_pixel).unwrap();

        println!("centers  {}, \n{}", center_pixel.inner, center_ground.inner);

        let circle_points_ground: Vec<Point2<Ground>> = (0..360)
            .step_by(10)
            .map(|angle_deg| {
                let (sin, cos) = (angle_deg as f32).to_radians().sin_cos();
                point![
                    center_ground.x() + radius * cos,
                    center_ground.y() + radius * sin
                ]
            })
            .collect();

        assert!(circle_points_ground.len() > 10);

        print!(
            "proj horz {:?}",
            matrix.ground_to_pixel(point![10000.0, 0.0])
        );

        let circle_points_pixel: Vec<_> = circle_points_ground
            .iter()
            .filter_map(|p| {
                assert!(p.x() < 10.0);
                let projected = matrix.ground_to_pixel(*p).ok()?;

                if projected.x().is_sign_negative()
                    || projected.y().is_sign_negative()
                    || projected.x() > matrix.image_size.x()
                    || projected.y() > matrix.image_size.y()
                {
                    return None;
                }
                // Rounding off error to emulate pixel noise
                Some(point![projected.x().round(), projected.y().round()])
            })
            .collect();
        let minmax_x = circle_points_pixel
            .iter()
            .minmax_by_key(|p| p.x())
            .into_option()
            .unwrap();
        let minmax_y = circle_points_pixel
            .iter()
            .minmax_by_key(|p| p.y())
            .into_option()
            .unwrap();
        let bounding_box = Rectangle::<Pixel> {
            min: point![minmax_x.0.x(), minmax_y.0.y()],
            max: point![minmax_x.1.x(), minmax_y.1.y()],
        };

        println!("BBOX {:?},", bounding_box);
        println!("field dims {:?}", field_dims);
        let measurements = vec![Measurement {
            matrix: distorted_matrix,
            position: CameraPosition::Top,
            circle_and_points: CenterCirclePoints {
                bounding_box,
                center: center_pixel,
                points: circle_points_pixel,
            },
        }];

        let corrections =
            solve::<CenterCircleResiduals>(Default::default(), measurements.clone(), field_dims);

        let center_circle_residuals: Vec<f32> = CenterCircleResiduals::calculate_from(
            &ExtendedCorrections {
                primary_corrections: corrections,
                radius_compensation: 0.0,
            },
            &measurements[0],
            &field_dims,
        )
        .unwrap()
        .into();

        let average_norm = nalgebra::DVectorView::from_slice(
            &center_circle_residuals,
            center_circle_residuals.len(),
        )
        .norm_squared()
            / 2.0;

        assert!(average_norm < 4e-6, "objective_func: {average_norm}");
        assert!(false);
    }
}
