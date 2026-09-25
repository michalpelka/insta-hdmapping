#pragma once

// Camera intrinsics for every projection model this app can verify and
// calibrate, read from / written to the flat camera_info.yaml layout
// insta360-to-images produces (fx, fy, cx, cy, [xi,] distortion, k, r, p):
//
//   insta360_mei_v2      Mei/unified-sphere model of the Insta360 rig's own
//                        calibration: xi, then distortion (k1, k2, k3, p1, p2).
//   plumb_bob            OpenCV's standard pinhole model (cv::calibrateCamera,
//                        cv::projectPoints): distortion (k1, k2, p1, p2, k3).
//   rational_polynomial  The same with CALIB_RATIONAL_MODEL's denominator
//                        terms: (k1, k2, p1, p2, k3, k4, k5, k6).
//   equidistant          OpenCV's fisheye model (cv::fisheye): (k1, k2, k3, k4).
//
// The OpenCV models go by the names ROS' CameraInfo uses for them, with
// OpenCV's coefficient order; tests/camera_test.cpp checks their projections
// against cv::projectPoints / cv::fisheye::projectPoints themselves.
//
// insta360_mei_v2: field meanings and the forward projection formula are
// taken from /home/michal/code/insta3360-to-images
// (include/insta360/calibration.hpp, src/equirect.cpp), which cross-checked
// them against an independent reverse-engineering effort and validated them
// visually against Insta360's own equirect export — not re-derived from
// scratch here. IMPORTANT: its `distortion` array is ordered
// (k1, k2, k3, p1, p2) — NOT OpenCV's pinhole order (k1, k2, p1, p2, k3). The
// two conventions are trivially easy to mix up (both are just five numbers in
// a row) and doing so produces a plausible-looking but badly wrong
// reprojection with no crash — see calibration.hpp's own comment on this.

#include <opencv2/core.hpp>

#include <array>
#include <cmath>
#include <string>

// Every switch over this is exhaustive (no default), so -Wswitch flags any
// that a new model is missing from; kCameraModelInfo in Camera.cpp is
// static_assert'ed to have one row per model.
enum class CameraModel { Mei, PlumbBob, RationalPolynomial, Equidistant };
inline constexpr int kCameraModelCount = 4;

// Intrinsics as one flat array: fx, fy, cx, cy for every model, then the
// model's own terms — xi first for Mei, then the camera_info.yaml
// `distortion` coefficients in that file's order. Sized for the largest
// model; entries past CameraModelInfo::numParams are unused (and 0).
inline constexpr int kMaxCameraParams = 12;
using CameraParams = std::array<double, kMaxCameraParams>;
enum CameraParamIndex { kFx = 0, kFy = 1, kCx = 2, kCy = 3, kMeiXi = 4 };

struct CameraModelInfo {
    const char* name;     // distortion_model in camera_info.yaml
    const char* label;    // for the UI
    int numParams;        // including fx, fy, cx, cy
    int distortionOffset; // params[] index of the yaml `distortion` array's first entry
    std::array<const char*, kMaxCameraParams> paramNames;

    int NumDistortion() const { return numParams - distortionOffset; }
};

const CameraModelInfo& ModelInfo(CameraModel model);

// ModelInfo(m).name for any model, plus "fisheye" as an alias of equidistant.
bool ParseCameraModel(const std::string& name, CameraModel& out);

struct CameraIntrinsics {
    CameraModel model = CameraModel::Mei;
    std::string frameId;
    int width = 0, height = 0;
    CameraParams params{};
    bool loaded = false;

    const CameraModelInfo& Info() const { return ModelInfo(model); }
    int NumParams() const { return Info().numParams; }

    // Forward: camera-frame 3D point (any positive scale, need not be unit
    // length) -> pixel coordinates. NaN if the model can't project it (see
    // ProjectPoint).
    cv::Point2d Project(cv::Point3d P) const;

    // Inverse: pixel -> unit-length ray direction in camera frame (closed
    // form where the model has one, iterative for the distortion terms). If
    // thetaDeg is given, receives the angle from the optical axis (0 = dead
    // center, useful as a "how far into the fisheye edge is this point"
    // sanity check).
    cv::Point3d Unproject(cv::Point2d uv, double* thetaDeg = nullptr) const;
};

namespace camera_detail {

template <typename T>
bool ProjectMei(const T* a, const T* P, T* uv) {
    using std::sqrt;
    const T n = sqrt(P[0] * P[0] + P[1] * P[1] + P[2] * P[2]);
    if (!(n > T(0.0))) return false;
    // Onto the unit sphere, then through the xi-shifted projection center.
    const T denom = P[2] / n + a[kMeiXi];
    if (!(denom > T(1e-9))) return false;
    const T x = P[0] / n / denom, y = P[1] / n / denom;

    const T &k1 = a[5], &k2 = a[6], &k3 = a[7], &p1 = a[8], &p2 = a[9];
    const T r2 = x * x + y * y;
    const T radial = T(1.0) + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    const T xd = x * radial + T(2.0) * p1 * x * y + p2 * (r2 + T(2.0) * x * x);
    const T yd = y * radial + p1 * (r2 + T(2.0) * y * y) + T(2.0) * p2 * x * y;
    uv[0] = a[kFx] * xd + a[kCx];
    uv[1] = a[kFy] * yd + a[kCy];
    return true;
}

// cv::projectPoints' model: plumb_bob, plus the k4..k6 denominator when
// `rational` (CALIB_RATIONAL_MODEL). Only points in front of the camera.
template <typename T>
bool ProjectPinhole(const T* a, const T* P, T* uv, bool rational) {
    if (!(P[2] > T(0.0))) return false;
    const T x = P[0] / P[2], y = P[1] / P[2];
    const T &k1 = a[4], &k2 = a[5], &p1 = a[6], &p2 = a[7], &k3 = a[8];
    const T r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
    T radial = T(1.0) + k1 * r2 + k2 * r4 + k3 * r6;
    if (rational) radial = radial / (T(1.0) + a[9] * r2 + a[10] * r4 + a[11] * r6);
    const T xd = x * radial + T(2.0) * p1 * x * y + p2 * (r2 + T(2.0) * x * x);
    const T yd = y * radial + p1 * (r2 + T(2.0) * y * y) + T(2.0) * p2 * x * y;
    uv[0] = a[kFx] * xd + a[kCx];
    uv[1] = a[kFy] * yd + a[kCy];
    return true;
}

// cv::fisheye's model, theta_d = theta * (1 + k1 theta^2 + ... + k4 theta^8).
// theta comes from atan2 rather than cv::fisheye's atan(r) — identical in
// front of the camera, and it keeps working past 90 deg off-axis, which a
// ~190 deg lens needs.
template <typename T>
bool ProjectEquidistant(const T* a, const T* P, T* uv) {
    using std::atan2;
    using std::sqrt;
    const T rho2 = P[0] * P[0] + P[1] * P[1];
    T scale; // theta_d / rho
    if (rho2 < T(1e-18) * P[2] * P[2]) {
        // Within ~1e-9 rad of the optical axis: theta ~ rho / z and the
        // distortion terms vanish (also avoids 0/0 on the axis itself).
        if (!(P[2] > T(0.0))) return false;
        scale = T(1.0) / P[2];
    } else {
        const T rho = sqrt(rho2);
        const T theta = atan2(rho, P[2]);
        const T t2 = theta * theta, t4 = t2 * t2;
        const T thetaD = theta * (T(1.0) + a[4] * t2 + a[5] * t4 + a[6] * t4 * t2 + a[7] * t4 * t4);
        scale = thetaD / rho;
    }
    uv[0] = a[kFx] * scale * P[0] + a[kCx];
    uv[1] = a[kFy] * scale * P[1] + a[kCy];
    return true;
}

} // namespace camera_detail

// The forward projection of `model` on a flat parameter array: camera-frame
// point P[3] -> pixel uv[2]. Templated so the Ceres calibration can autodiff
// it; CameraIntrinsics::Project goes through it too, so each model's formula
// exists exactly once. Returns false (leaving uv untouched) for points the model
// can't image: the origin, anything behind a pinhole camera, or through the
// Mei model's singularity (Xs.z + xi <= 0, reachable only with xi < 1).
template <typename T>
bool ProjectPoint(CameraModel model, const T* params, const T* P, T* uv) {
    switch (model) {
    case CameraModel::Mei: return camera_detail::ProjectMei(params, P, uv);
    case CameraModel::PlumbBob: return camera_detail::ProjectPinhole(params, P, uv, false);
    case CameraModel::RationalPolynomial: return camera_detail::ProjectPinhole(params, P, uv, true);
    case CameraModel::Equidistant: return camera_detail::ProjectEquidistant(params, P, uv);
    }
    return false;
}

// Loads intrinsics from a camera_info.yaml in this rig's flat format (see
// data/camera_info.yaml for a sample), for any model above. On any failure
// (missing file, missing field, unknown distortion_model) prints a message to
// stderr and returns a CameraIntrinsics with loaded=false — a missing/malformed
// file should degrade the app to "no reprojection available", not crash it.
CameraIntrinsics LoadCamera(const std::string& path);

// Writes `cam` as a camera_info.yaml in the format LoadCamera reads. When
// `templatePath` is non-empty, that file (normally the camera_info.yaml the
// calibration started from) is copied line by line with only the intrinsic
// keys (width, height, distortion_model, xi, fx, fy, cx, cy, distortion, k, p)
// replaced — so the fields CameraIntrinsics doesn't model (serial, model,
// firmware, video_stream_index, r, rotation_deg, translation, ...) survive verbatim,
// quoting included, instead of being dropped. A template's xi line is dropped
// when `cam` isn't a Mei camera. `comment` becomes a leading
// "# calib_app: ..." line. Returns false with a reason in *error on failure.
bool SaveCamera(const std::string& path, const CameraIntrinsics& cam, const std::string& templatePath,
                const std::string& comment, std::string* error);