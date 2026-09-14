#include "Verification.h"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>

namespace {
constexpr int kMinCorners = 6; // solvePnP is happy with 4; a few more for stability
}

VerificationResult VerifyPose(const cv::Mat& charucoCorners, const cv::Mat& charucoIds,
                               const cv::Ptr<cv::aruco::CharucoBoard>& board, const MeiCamera& cam) {
    VerificationResult r;

    if (!cam.loaded) {
        r.message = "no camera_info.yaml loaded";
        return r;
    }
    const int n = charucoIds.empty() ? 0 : charucoIds.rows;
    if (n < kMinCorners) {
        r.message = "need >= " + std::to_string(kMinCorners) + " detected corners, have " + std::to_string(n);
        return r;
    }

    // Bearing-vector PnP: undo this camera's own (wide-FOV, non-pinhole)
    // distortion to get each detected corner's ray direction, then hand
    // solvePnP the ray's (x/z, y/z) as if K were the identity — a point at
    // camera-frame direction Xs projects to exactly (Xs.x/Xs.z, Xs.y/Xs.z)
    // under an ideal unit-focal-length pinhole, so this is exact, not an
    // approximation, as long as those ratios stay finite (i.e. no detected
    // corner sits ~90 degrees off the optical axis — true here, see below).
    std::vector<cv::Point3f> objPts;
    std::vector<cv::Point2f> bearing2d;
    objPts.reserve(n);
    bearing2d.reserve(n);
    double maxThetaDeg = 0.0;
    for (int i = 0; i < n; ++i) {
        const int id = charucoIds.at<int>(i);
        const cv::Point2f p = charucoCorners.at<cv::Point2f>(i);
        double thetaDeg = 0.0;
        const cv::Point3d ray = cam.Unproject(cv::Point2d(p.x, p.y), &thetaDeg);
        maxThetaDeg = std::max(maxThetaDeg, thetaDeg);
        objPts.push_back(board->chessboardCorners[id]);
        bearing2d.emplace_back((float)(ray.x / ray.z), (float)(ray.y / ray.z));
    }
    if (maxThetaDeg > 80.0) {
        // Ratios blow up near 90 degrees off-axis; ITERATIVE would become
        // numerically unstable there. Not expected for this rig/board, but
        // fail loudly instead of silently returning a bad pose.
        r.message = "detected corner " + std::to_string((int)maxThetaDeg) +
                    " deg off-axis, too close to the fisheye's 90 deg limit for this solver";
        return r;
    }

    cv::Mat rvec, tvec;
    const bool ok = cv::solvePnP(objPts, bearing2d, cv::Mat::eye(3, 3, CV_64F), cv::Mat(), rvec, tvec,
                                  false, cv::SOLVEPNP_ITERATIVE);
    if (!ok) {
        r.message = "solvePnP failed to converge";
        return r;
    }
    r.rvec = rvec;
    r.tvec = tvec;

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    auto reproject = [&](const cv::Point3f& P) {
        const cv::Mat Pw = (cv::Mat_<double>(3, 1) << P.x, P.y, P.z);
        const cv::Mat Pc = R * Pw + tvec;
        return cam.Project(cv::Point3d(Pc.at<double>(0), Pc.at<double>(1), Pc.at<double>(2)));
    };

    // Full board grid, so the overlay can show where every corner is
    // predicted to be — including ones that weren't detected.
    r.allReprojected.resize(board->chessboardCorners.size());
    for (size_t id = 0; id < board->chessboardCorners.size(); ++id) {
        const cv::Point2d pr = reproject(board->chessboardCorners[id]);
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
