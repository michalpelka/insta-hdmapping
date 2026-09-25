#include "Verification.h"

#include "Board.h"
#include "Calibration.h"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>

VerificationResult VerifyPose(const cv::Mat& charucoCorners, const cv::Mat& charucoIds,
                               const cv::Ptr<cv::aruco::CharucoBoard>& board, const CameraIntrinsics& cam) {
    VerificationResult r;

    if (!cam.loaded) {
        r.message = "no camera_info.yaml loaded";
        return r;
    }
    const int n = charucoIds.empty() ? 0 : charucoIds.rows;
    if (n < kMinVerifyCorners) {
        r.message = "need >= " + std::to_string(kMinVerifyCorners) + " detected corners, have " + std::to_string(n);
        return r;
    }

    const std::vector<cv::Point3f> chessboardCorners = BoardCorners(*board);
    BoardView view;
    view.objectPoints.reserve(n);
    view.imagePoints.reserve(n);
    for (int i = 0; i < n; ++i) {
        view.objectPoints.push_back(chessboardCorners[charucoIds.at<int>(i)]);
        view.imagePoints.push_back(charucoCorners.at<cv::Point2f>(i));
    }

    BoardPose pose;
    if (!EstimateBoardPose(cam, view, pose, &r.message)) return r;
    cv::Rodrigues(cv::Mat(pose.R), r.rvec);
    r.tvec = cv::Mat(pose.t, true);

    auto reproject = [&](const cv::Point3f& P) {
        const cv::Vec3d Pc = pose.R * cv::Vec3d(P.x, P.y, P.z) + pose.t;
        return cam.Project(cv::Point3d(Pc[0], Pc[1], Pc[2]));
    };

    // Full board grid, so the overlay can show where every corner is
    // predicted to be — including ones that weren't detected.
    r.allReprojected.resize(chessboardCorners.size());
    for (size_t id = 0; id < chessboardCorners.size(); ++id) {
        const cv::Point2d pr = reproject(chessboardCorners[id]);
        r.allReprojected[id] = cv::Point2f((float)pr.x, (float)pr.y);
    }

    r.residuals.reserve(n);
    double sumErr = 0.0, sumSq = 0.0, maxErr = 0.0;
    for (int i = 0; i < n; ++i) {
        const int id = charucoIds.at<int>(i);
        const cv::Point2f detected = charucoCorners.at<cv::Point2f>(i);
        const cv::Point2f reprojected = r.allReprojected[id];
        const double e = cv::norm(reprojected - detected);
        r.residuals.push_back({id, detected, reprojected, e});
        sumErr += e;
        sumSq += e * e;
        maxErr = std::max(maxErr, e);
    }
    r.meanErrorPx = sumErr / n;
    r.rmsErrorPx = std::sqrt(sumSq / n);
    r.maxErrorPx = maxErr;
    r.ok = true;
    return r;
}
