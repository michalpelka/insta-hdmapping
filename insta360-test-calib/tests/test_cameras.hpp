// Reference cameras and synthetic board captures shared by the tests.
#pragma once

#include "Calibration.h"
#include "Camera.h"

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <vector>

namespace test_cameras {

inline CameraIntrinsics Make(CameraModel model, int w, int h, std::vector<double> params) {
    CameraIntrinsics cam;
    cam.model = model;
    cam.frameId = "test_optical_frame";
    cam.width = w;
    cam.height = h;
    for (size_t k = 0; k < params.size(); ++k) cam.params[k] = params[k];
    cam.loaded = true;
    return cam;
}

// The rig's factory calibration (data/camera_info.yaml).
inline CameraIntrinsics Rig() {
    return Make(CameraModel::Mei, 2880, 2880,
                {2303.378571, 2303.073214, 1444.832143, 1436.678571, 2.0, 0.18967018, 2.06612277, -3.31555128,
                 0.00052395, 3.87e-05});
}

// Roughly what the same ~190 deg lens looks like in cv::fisheye's model.
inline CameraIntrinsics Fisheye() {
    return Make(CameraModel::Equidistant, 2880, 2880,
                {780.0, 779.5, 1442.0, 1438.0, 0.02, -0.005, 0.001, -0.0002});
}

// An ordinary ~80 deg HFOV machine-vision camera with barrel distortion.
inline CameraIntrinsics Pinhole() {
    return Make(CameraModel::PlumbBob, 1920, 1200,
                {1100.0, 1098.0, 955.0, 605.0, -0.28, 0.09, 0.0008, -0.0005, -0.012});
}

inline CameraIntrinsics Rational() {
    return Make(CameraModel::RationalPolynomial, 1920, 1200,
                {900.0, 901.0, 962.0, 598.0, 0.4, -0.05, 0.0006, -0.0004, 0.001, 0.7, -0.02, 0.005});
}

// Same layout as the app's default board (14x8 squares of 260 units): 91
// interior corners.
inline std::vector<cv::Point3f> BoardGrid() {
    std::vector<cv::Point3f> pts;
    for (int y = 1; y < 8; ++y)
        for (int x = 1; x < 14; ++x) pts.emplace_back(x * 260.f, y * 260.f, 0.f);
    return pts;
}

inline cv::Matx33d Rot(const cv::Vec3d& w) {
    cv::Matx33d R;
    cv::Rodrigues(w, R);
    return R;
}

// A board centered `offAxisDeg` off the optical axis in direction
// `azimuthDeg`, `distance` away, roughly facing the camera but tilted by
// `tilt` — i.e. the kind of spread a calibration capture sweeps through.
inline BoardPose PoseAt(double offAxisDeg, double azimuthDeg, double distance, const cv::Vec3d& tilt) {
    const double a = offAxisDeg * CV_PI / 180.0, phi = azimuthDeg * CV_PI / 180.0;
    const cv::Vec3d dir(std::sin(a) * std::cos(phi), std::sin(a) * std::sin(phi), std::cos(a));
    const cv::Vec3d z(0, 0, 1);
    cv::Vec3d axis = z.cross(dir);
    const double s = cv::norm(axis);
    const cv::Matx33d face = s > 1e-12 ? Rot(axis * (std::asin(std::min(1.0, s)) / s)) : cv::Matx33d::eye();
    BoardPose pose;
    pose.R = face * Rot(tilt);
    const cv::Vec3d center(7 * 260.0, 4 * 260.0, 0.0); // board-frame center of the grid
    pose.t = dir * distance - pose.R * center;
    return pose;
}

// Projects the grid through `cam` at `pose`, keeping the corners that land
// inside the image, with Gaussian pixel noise of `sigma`.
inline BoardView Observe(const CameraIntrinsics& cam, const BoardPose& pose, double sigma, cv::RNG& rng) {
    BoardView v;
    for (const cv::Point3f& P : BoardGrid()) {
        const cv::Vec3d Pc = pose.R * cv::Vec3d(P.x, P.y, P.z) + pose.t;
        const cv::Point2d uv = cam.Project(cv::Point3d(Pc[0], Pc[1], Pc[2]));
        if (!(uv.x >= 0 && uv.y >= 0 && uv.x < cam.width && uv.y < cam.height)) continue;
        // One draw per statement: argument evaluation order is unspecified,
        // and the noise has to be the same whatever the compiler picks.
        const double nx = rng.gaussian(sigma);
        const double ny = rng.gaussian(sigma);
        v.objectPoints.push_back(P);
        v.imagePoints.emplace_back((float)(uv.x + nx), (float)(uv.y + ny));
    }
    return v;
}

// Boards swept over the field of view: one straight ahead, then six around
// each ring of `offAxisDeg`, at distances in [dMin, dMax].
inline std::vector<BoardView> Capture(const CameraIntrinsics& cam, double sigma, const std::vector<double>& offAxisDeg,
                                      double dMin, double dMax) {
    cv::RNG rng(12345);
    std::vector<BoardView> views;
    for (double off : offAxisDeg) {
        const int nAzimuth = off == 0 ? 1 : 6;
        for (int k = 0; k < nAzimuth; ++k) {
            cv::Vec3d tilt;
            tilt[0] = rng.uniform(-0.4, 0.4);
            tilt[1] = rng.uniform(-0.4, 0.4);
            tilt[2] = rng.uniform(-0.3, 0.3);
            const double distance = rng.uniform(dMin, dMax);
            const BoardPose pose = PoseAt(off, k * 60.0 + off, distance, tilt);
            BoardView v = Observe(cam, pose, sigma, rng);
            if (v.objectPoints.size() >= 20) views.push_back(std::move(v));
        }
    }
    return views;
}

// Fisheye sweep out past 90 deg off-axis at the corners.
inline std::vector<BoardView> FisheyeCapture(const CameraIntrinsics& cam, double sigma) {
    return Capture(cam, sigma, {0, 25, 45, 60, 72}, 2500.0, 4000.0);
}

// Pinhole sweep inside a ~80 deg field of view.
inline std::vector<BoardView> PinholeCapture(const CameraIntrinsics& cam, double sigma) {
    return Capture(cam, sigma, {0, 12, 22, 30}, 5000.0, 8000.0);
}

} // namespace test_cameras