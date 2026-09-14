#include "insta360/equirect.hpp"

#include <cmath>

#include "test_util.hpp"

using namespace insta360;

namespace {

// A lens with no distortion and no tilt, so the Mei/UCM projection reduces to the
// bare sphere-then-perspective step and every result is hand-computable.
LensCalibration bare_lens(double xi, double fx, double fy, double cx, double cy) {
    LensCalibration lens;
    lens.xi = xi;
    lens.fx = fx;
    lens.fy = fy;
    lens.cx = cx;
    lens.cy = cy;
    lens.k1 = lens.k2 = lens.k3 = lens.p1 = lens.p2 = 0;
    lens.rotation_deg = {0.0, 0.0, 0.0};
    lens.width = lens.height = 100000;  // large enough that bounds-checks never trigger
    return lens;
}

}  // namespace

TEST(center_ray_maps_to_the_principal_point_with_full_weight) {
    // A 7x7 canvas puts pixel (3, 3)'s center exactly at longitude 0, latitude 0:
    // (3 + 0.5) / 7 * 2*pi - pi == 0, and symmetrically for latitude. That ray is
    // (0, 0, 1) -- straight down the lens axis -- for any xi, so it must land
    // exactly on (cx, cy) with theta = 0 (full weight).
    LensCalibration lens = bare_lens(/*xi=*/2.0, /*fx=*/100, /*fy=*/100, /*cx=*/50, /*cy=*/50);
    EquirectMaps maps = build_equirect_maps(lens, 7, 7, /*is_back_lens=*/false);

    CHECK_NEAR(maps.map_x.at<float>(3, 3), 50.0, 1e-4);
    CHECK_NEAR(maps.map_y.at<float>(3, 3), 50.0, 1e-4);
    CHECK_NEAR(maps.weight.at<float>(3, 3), 1.0, 1e-6);
}

TEST(oblique_ray_matches_hand_computed_mei_projection) {
    // A 2x2 canvas puts pixel (u=1, v=0)'s center at longitude 90 deg, latitude 45
    // deg exactly: (1.5/2)*360-180 = 90, and 90-(0.5/2)*180 = 45. With x-right,
    // y-down, z-forward rays, that ray is (sin90*cos45, -sin45, cos90*cos45) =
    // (c, -c, 0) where c = sqrt(2)/2. For the front lens (no rotation), Xc == ray,
    // so this is a direct check of the Mei formula: x = Xc.x/(Xc.z+xi), scaled by
    // fx/fy and offset by cx/cy (no distortion terms to complicate it).
    const double xi = 2.0, fx = 100, fy = 100, cx = 50, cy = 50;
    const double c = std::sqrt(2.0) / 2.0;
    LensCalibration lens = bare_lens(xi, fx, fy, cx, cy);

    EquirectMaps front = build_equirect_maps(lens, 2, 2, /*is_back_lens=*/false);
    double expected_px = fx * (c / xi) + cx;
    double expected_py = fy * (-c / xi) + cy;
    CHECK_NEAR(front.map_x.at<float>(0, 1), expected_px, 1e-3);
    CHECK_NEAR(front.map_y.at<float>(0, 1), expected_py, 1e-3);
    // theta = angle from the lens axis to (c, -c, 0) is exactly 90 degrees (its Z
    // component is 0), landing exactly on the 90-degree midpoint of the [83, 97]
    // feather band.
    CHECK_NEAR(front.weight.at<float>(0, 1), 0.5, 1e-4);

    // The back lens rotates the incoming ray 180 degrees about its local X axis
    // before projecting: (x, y, z) -> (x, -y, -z). For this ray (z=0), that flips
    // the sign of y but leaves x and theta unchanged -- a real, checkable
    // difference from the front lens, not just "some other number".
    EquirectMaps back = build_equirect_maps(lens, 2, 2, /*is_back_lens=*/true);
    double expected_py_back = fy * (c / xi) + cy;
    CHECK_NEAR(back.map_x.at<float>(0, 1), expected_px, 1e-3);
    CHECK_NEAR(back.map_y.at<float>(0, 1), expected_py_back, 1e-3);
    CHECK_NEAR(back.weight.at<float>(0, 1), 0.5, 1e-4);
}

TEST(rays_far_outside_the_lens_fov_are_marked_invalid) {
    // longitude 157.5, latitude 11.25 (an arbitrary off-axis point on an 8x8 grid)
    // is well over 90 degrees from the front lens' forward axis -- outside even a
    // generous fisheye's usable range -- so it must come back with zero weight and
    // no source pixel.
    LensCalibration lens = bare_lens(2.0, 100, 100, 50, 50);
    EquirectMaps maps = build_equirect_maps(lens, 8, 8, /*is_back_lens=*/false);

    CHECK_NEAR(maps.weight.at<float>(3, 7), 0.0, 1e-6);
    CHECK(maps.map_x.at<float>(3, 7) < 0);
    CHECK(maps.map_y.at<float>(3, 7) < 0);
}

TEST(blend_equirect_is_a_weight_normalized_average) {
    cv::Mat a(1, 1, CV_8UC3, cv::Scalar(200, 0, 0));
    cv::Mat b(1, 1, CV_8UC3, cv::Scalar(0, 100, 0));
    cv::Mat weight_a(1, 1, CV_32FC1, cv::Scalar(0.75f));
    cv::Mat weight_b(1, 1, CV_32FC1, cv::Scalar(0.25f));

    cv::Mat blended = blend_equirect(a, weight_a, b, weight_b);
    // 0.75 * 200 + 0.25 * 0 = 150 on the first channel; 0.75*0 + 0.25*100 = 25 on
    // the second.
    cv::Vec3b pixel = blended.at<cv::Vec3b>(0, 0);
    CHECK_EQ(static_cast<int>(pixel[0]), 150);
    CHECK_EQ(static_cast<int>(pixel[1]), 25);
    CHECK_EQ(static_cast<int>(pixel[2]), 0);
}

TEST(blend_equirect_returns_black_where_neither_lens_is_valid) {
    cv::Mat a(1, 1, CV_8UC3, cv::Scalar(200, 200, 200));
    cv::Mat b(1, 1, CV_8UC3, cv::Scalar(200, 200, 200));
    cv::Mat zero(1, 1, CV_32FC1, cv::Scalar(0.f));

    cv::Mat blended = blend_equirect(a, zero, b, zero);
    cv::Vec3b pixel = blended.at<cv::Vec3b>(0, 0);
    CHECK_EQ(static_cast<int>(pixel[0]), 0);
    CHECK_EQ(static_cast<int>(pixel[1]), 0);
    CHECK_EQ(static_cast<int>(pixel[2]), 0);
}
