#pragma once

// Intrinsic calibration of any camera model from several views of a planar
// board, plus the pose-only special case the verification view uses. Both
// minimize the same thing — pixel reprojection error through ProjectPoint —
// with Ceres.

#include "Camera.h"

#include <opencv2/core.hpp>

#include <array>
#include <string>
#include <vector>

// One image's worth of board observations: each detected corner's pixel
// position paired with where it sits on the board (board frame, z = 0).
struct BoardView {
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;
};

// Board frame -> camera frame: X_cam = R * X_board + t.
struct BoardPose {
    cv::Matx33d R = cv::Matx33d::eye();
    cv::Vec3d t;
};

// Pose-only fit with `cam` held fixed: a bearing-vector PnP (which undoes the
// camera's own distortion first, so it works for wide-FOV models instead of
// assuming a pinhole) for the initial guess, then refinement of the pixel
// reprojection error over every corner. Corners too far off-axis for the PnP
// step (> 80 deg) are left out of the initial guess only. Returns false with
// a reason in *message when no pose can be found.
bool EstimateBoardPose(const CameraIntrinsics& cam, const BoardView& view, BoardPose& pose,
                       std::string* message = nullptr);

struct CalibrationOptions {
    // Intrinsics held at their starting value, by CameraIntrinsics::params
    // index. Entries past the model's parameter count are always held.
    std::array<bool, kMaxCameraParams> fixed{};
    int maxIterations = 100;
};

// Mei: xi held — it trades off almost one-for-one against fx/fy and the
// radial terms, so a planar board constrains it poorly unless the views reach
// far into the fisheye's edge, and Insta360's own calibration pins it at
// exactly 2. The OpenCV models: everything free, as cv::calibrateCamera /
// cv::fisheye::calibrate do by default.
CalibrationOptions DefaultCalibrationOptions(CameraModel model);

struct CalibrationResult {
    bool ok = false;
    std::string message; // why it failed, or a one-line summary on success

    CameraIntrinsics camera;        // refined intrinsics (model, size and frame
                                    // id carried over from the initial guess)
    CameraParams stdDev{};          // 1-sigma per intrinsic, 0 for fixed ones
    std::vector<bool> viewUsed;     // per input view: false if no initial pose
    std::vector<BoardPose> poses;   // per input view (identity if unused)
    std::vector<double> viewRmsPx;  // per input view (NaN if unused)
    double initialRmsPx = 0.0;      // before the joint refinement
    double rmsPx = 0.0;             // over every corner of every used view
    int usedViews = 0;
    int usedCorners = 0;
    int iterations = 0;
};

// A starting point for `model` when there's no camera_info.yaml for it:
// principal point at the image center and no distortion (xi = 2 for Mei).
// The focal length comes from
//   - pinhole models: cv::initCameraMatrix2D over `views` (Zhang's closed
//     form from the board homographies — what cv::calibrateCamera starts
//     from), or a 60 deg horizontal field of view without views;
//   - fisheye models: putting 95 deg off-axis on the image's inscribed
//     circle (these lenses cover ~190 deg).
CameraIntrinsics GenericGuess(CameraModel model, int width, int height,
                              const std::vector<BoardView>& views = {});

// Joint refinement of the intrinsics and every view's pose, starting from
// `initial` (whose model is the one fitted). Needs at least
// kMinCalibrationViews views that each give an initial pose.
constexpr int kMinCalibrationViews = 3;
CalibrationResult Calibrate(const std::vector<BoardView>& views, const CameraIntrinsics& initial,
                            const CalibrationOptions& options);