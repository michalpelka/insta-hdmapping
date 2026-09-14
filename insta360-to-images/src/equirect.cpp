#include "insta360/equirect.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace insta360 {

namespace {

// Theta (angle from the lens' own optical axis) below which a lens gets full
// blending weight, and above which it contributes nothing. Symmetric around the
// nominal 90-degree seam between two opposed lenses: since the back lens' ray is
// (to within the ~0.1-0.2 degree tilts in the calibration) exactly the negation of
// the front lens' ray for the same world direction, theta_front + theta_back = 180
// degrees, so these two thresholds make the two lenses' weights sum to exactly 1
// throughout the overlap band and each lens' own native hemisphere gets full
// weight. 97 degrees matches the FOV limit used by the reference implementation
// this model was cross-checked against; 83 = 180 - 97 keeps the band symmetric.
constexpr double kFullWeightThetaDeg = 83.0;
constexpr double kMaxThetaDeg = 97.0;

cv::Matx33d rotation_x(double radians) {
    double c = std::cos(radians), s = std::sin(radians);
    return {1, 0, 0, 0, c, -s, 0, s, c};
}
cv::Matx33d rotation_y(double radians) {
    double c = std::cos(radians), s = std::sin(radians);
    return {c, 0, s, 0, 1, 0, -s, 0, c};
}
cv::Matx33d rotation_z(double radians) {
    double c = std::cos(radians), s = std::sin(radians);
    return {c, -s, 0, s, c, 0, 0, 0, 1};
}

constexpr double kDeg2Rad = CV_PI / 180.0;

}  // namespace

EquirectMaps build_equirect_maps(const LensCalibration& lens, int eq_width, int eq_height,
                                  bool is_back_lens) {
    const double full_weight_theta = kFullWeightThetaDeg * kDeg2Rad;
    const double max_theta = kMaxThetaDeg * kDeg2Rad;

    // World ray -> this lens' local frame: undo the calibration's small per-lens
    // tilt (rx, ry, rz), then undo the lens' own mounting direction. The front lens
    // faces the equirect's longitude-0 axis directly (B = identity); the back lens
    // is rotated 180 degrees about its local X axis, not Y -- both the axis choice
    // and the sign/order of this composition (rz applied directly, rx/ry negated)
    // were pinned down empirically, not assumed: verified against a real X5
    // capture by geometrically stitching isolated single-lens frames at a moment
    // with visibly non-trivial camera roll (a tilted handheld frame partway through
    // a clip, not the near-level first frame) and confirming vertical scene
    // features (a door frame, a balcony door) come out vertical in the output, for
    // both lenses independently and in the blended result. See calibration.hpp for
    // the calibration-format cross-references.
    double rx = lens.rotation_deg[0] * kDeg2Rad;
    double ry = lens.rotation_deg[1] * kDeg2Rad;
    double rz = lens.rotation_deg[2] * kDeg2Rad;
    cv::Matx33d B = is_back_lens ? rotation_x(CV_PI) : cv::Matx33d::eye();
    cv::Matx33d R = rotation_z(rz) * rotation_y(-ry) * rotation_x(-rx) * B;

    EquirectMaps maps;
    maps.map_x.create(eq_height, eq_width, CV_32FC1);
    maps.map_y.create(eq_height, eq_width, CV_32FC1);
    maps.weight.create(eq_height, eq_width, CV_32FC1);

    for (int v = 0; v < eq_height; ++v) {
        // Row 0 is the top of the image (looking straight up); row eq_height is the
        // bottom (straight down).
        double lat = CV_PI / 2.0 - (v + 0.5) / eq_height * CV_PI;
        float* row_x = maps.map_x.ptr<float>(v);
        float* row_y = maps.map_y.ptr<float>(v);
        float* row_w = maps.weight.ptr<float>(v);

        for (int u = 0; u < eq_width; ++u) {
            // Column 0 is longitude -180; the center column is longitude 0, the
            // front lens' forward direction.
            double lon = (u + 0.5) / eq_width * 2.0 * CV_PI - CV_PI;

            // x right, y down, z forward -- the world frame this model was
            // validated in (see calibration.hpp).
            cv::Vec3d ray(std::sin(lon) * std::cos(lat), -std::sin(lat), std::cos(lon) * std::cos(lat));
            cv::Vec3d Xc = R * ray;

            double theta = std::acos(std::clamp(Xc[2], -1.0, 1.0));
            double denom = Xc[2] + lens.xi;

            float px = -1.f, py = -1.f, weight = 0.f;
            if (denom > 1e-6 && theta <= max_theta) {
                // Mei/UCM unified-sphere projection, then radial + tangential
                // distortion on the projected plane (see equirect.hpp).
                double x = Xc[0] / denom;
                double y = Xc[1] / denom;
                double r2 = x * x + y * y;
                double radial = 1.0 + lens.k1 * r2 + lens.k2 * r2 * r2 + lens.k3 * r2 * r2 * r2;
                double xd = x * radial + 2.0 * lens.p1 * x * y + lens.p2 * (r2 + 2.0 * x * x);
                double yd = y * radial + lens.p1 * (r2 + 2.0 * y * y) + 2.0 * lens.p2 * x * y;
                double src_x = lens.fx * xd + lens.cx;
                double src_y = lens.fy * yd + lens.cy;

                if (src_x >= 0 && src_x <= lens.width - 1 && src_y >= 0 && src_y <= lens.height - 1) {
                    px = static_cast<float>(src_x);
                    py = static_cast<float>(src_y);
                    if (theta <= full_weight_theta) {
                        weight = 1.f;
                    } else {
                        weight = static_cast<float>((max_theta - theta) /
                                                     (max_theta - full_weight_theta));
                    }
                }
            }
            row_x[u] = px;
            row_y[u] = py;
            row_w[u] = weight;
        }
    }
    return maps;
}

cv::Mat remap_to_equirect(const cv::Mat& fisheye, const EquirectMaps& maps) {
    cv::Mat out;
    cv::remap(fisheye, out, maps.map_x, maps.map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
              cv::Scalar(0, 0, 0));
    return out;
}

cv::Mat blend_equirect(const cv::Mat& a, const cv::Mat& weight_a, const cv::Mat& b,
                        const cv::Mat& weight_b) {
    CV_Assert(a.size() == b.size() && a.type() == CV_8UC3 && b.type() == CV_8UC3);
    CV_Assert(weight_a.size() == a.size() && weight_b.size() == a.size());

    cv::Mat out(a.size(), CV_8UC3);
    for (int v = 0; v < a.rows; ++v) {
        const cv::Vec3b* row_a = a.ptr<cv::Vec3b>(v);
        const cv::Vec3b* row_b = b.ptr<cv::Vec3b>(v);
        const float* wa = weight_a.ptr<float>(v);
        const float* wb = weight_b.ptr<float>(v);
        cv::Vec3b* row_out = out.ptr<cv::Vec3b>(v);

        for (int u = 0; u < a.cols; ++u) {
            double sum = static_cast<double>(wa[u]) + wb[u];
            if (sum < 1e-6) {
                row_out[u] = cv::Vec3b(0, 0, 0);
                continue;
            }
            double alpha_a = wa[u] / sum;
            double alpha_b = 1.0 - alpha_a;
            for (int c = 0; c < 3; ++c) {
                double value = alpha_a * row_a[u][c] + alpha_b * row_b[u][c];
                row_out[u][c] = static_cast<uchar>(std::clamp(value, 0.0, 255.0));
            }
        }
    }
    return out;
}

}  // namespace insta360
