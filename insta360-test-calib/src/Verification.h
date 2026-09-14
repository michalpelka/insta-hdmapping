#pragma once

#include "MeiCamera.h"

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core.hpp>

#include <string>
#include <vector>

// Per-corner outcome for one *detected* charuco corner: where it was
// detected vs. where the loaded intrinsics + estimated board pose predict it
// should be.
struct CornerResidual {
    int id = -1;
    cv::Point2f detected;
    cv::Point2f reprojected;
    double errorPx = 0.0;
};

struct VerificationResult {
    bool ok = false;
    std::string message; // set when !ok, explains why (shown in the UI)

    cv::Mat rvec, tvec; // estimated board pose (board frame -> camera frame)

    std::vector<CornerResidual> residuals;     // one per detected corner
    std::vector<cv::Point2f> allReprojected;   // one per *board* corner (id-indexed,
                                                // 0..totalCorners-1), detected or not —
                                                // lets the overlay show the whole
                                                // expected grid, not just hits.
    double meanErrorPx = 0.0, rmsErrorPx = 0.0, maxErrorPx = 0.0;
};

// Estimates the board's pose from its detected corners (via a bearing-vector
// PnP solve that undoes the camera's own distortion first, so it works for
// this wide-FOV model instead of assuming a pinhole) and reprojects every
// board corner through `cam` for comparison against what was actually
// detected. Requires `cam.loaded` and at least kMinCorners detected corners;
// otherwise returns a result with ok=false and an explanatory message.
VerificationResult VerifyPose(const cv::Mat& charucoCorners, const cv::Mat& charucoIds,
                               const cv::Ptr<cv::aruco::CharucoBoard>& board, const MeiCamera& cam);
