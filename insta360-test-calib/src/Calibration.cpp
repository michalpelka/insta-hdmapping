#include "Calibration.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <thread>

namespace {

constexpr int kNp = kMaxCameraParams;
// Ceres pose block: angle-axis rotation (3) then translation (3).
using PoseBlock = std::array<double, 6>;

constexpr double kInf = std::numeric_limits<double>::infinity();

// The bearing-vector PnP used for initial poses divides by the ray's z, which
// blows up towards 90 deg off-axis; corners past this are left out of the
// initial guess (the refinement that follows still uses them).
constexpr double kMaxInitThetaDeg = 80.0;
constexpr int kMinInitCorners = 4; // solvePnP's minimum

// One detected corner: pixel residual of the board point projected through
// the intrinsics block at the pose block. The intrinsics block is always
// kMaxCameraParams long whatever the model; the entries a model doesn't use
// are held constant by Calibrate().
struct CornerReprojection {
    CornerReprojection(CameraModel model, const cv::Point3f& P, const cv::Point2f& uv)
        : model(model), X{P.x, P.y, P.z}, u(uv.x), v(uv.y) {}

    template <typename T>
    bool operator()(const T* intrinsics, const T* pose, T* residual) const {
        const T Pb[3] = {T(X[0]), T(X[1]), T(X[2])};
        T Pc[3];
        ceres::AngleAxisRotatePoint(pose, Pb, Pc);
        for (int k = 0; k < 3; ++k) Pc[k] += pose[3 + k];
        T uv[2];
        if (!ProjectPoint(model, intrinsics, Pc, uv)) return false; // rejects the step
        residual[0] = uv[0] - T(u);
        residual[1] = uv[1] - T(v);
        return true;
    }

    static ceres::CostFunction* Create(CameraModel model, const cv::Point3f& P, const cv::Point2f& uv) {
        return new ceres::AutoDiffCostFunction<CornerReprojection, 2, kNp, 6>(new CornerReprojection(model, P, uv));
    }

    CameraModel model;
    double X[3];
    double u, v;
};

PoseBlock ToBlock(const BoardPose& pose) {
    cv::Vec3d rvec;
    cv::Rodrigues(pose.R, rvec);
    return {rvec[0], rvec[1], rvec[2], pose.t[0], pose.t[1], pose.t[2]};
}

BoardPose FromBlock(const PoseBlock& b) {
    BoardPose pose;
    cv::Rodrigues(cv::Vec3d(b[0], b[1], b[2]), pose.R);
    pose.t = cv::Vec3d(b[3], b[4], b[5]);
    return pose;
}

// Sum of squared pixel residuals over one view; +inf if any corner can't be
// projected.
double ViewCost(const CameraIntrinsics& cam, const BoardView& view, const BoardPose& pose) {
    double cost = 0.0;
    for (size_t i = 0; i < view.objectPoints.size(); ++i) {
        const cv::Point3f& P = view.objectPoints[i];
        const cv::Vec3d Pc = pose.R * cv::Vec3d(P.x, P.y, P.z) + pose.t;
        double uv[2];
        if (!ProjectPoint(cam.model, cam.params.data(), Pc.val, uv)) return kInf;
        const double dx = uv[0] - view.imagePoints[i].x, dy = uv[1] - view.imagePoints[i].y;
        cost += dx * dx + dy * dy;
    }
    return cost;
}

void AddView(ceres::Problem& problem, CameraModel model, const BoardView& view, double* intrinsics, double* pose) {
    for (size_t i = 0; i < view.objectPoints.size(); ++i)
        problem.AddResidualBlock(CornerReprojection::Create(model, view.objectPoints[i], view.imagePoints[i]),
                                 nullptr, intrinsics, pose);
}

bool InitialPose(const CameraIntrinsics& cam, const BoardView& view, BoardPose& pose, std::string& why) {
    std::vector<cv::Point3f> objPts;
    std::vector<cv::Point2f> bearing2d;
    for (size_t i = 0; i < view.objectPoints.size(); ++i) {
        const cv::Point2f& p = view.imagePoints[i];
        double thetaDeg = 0.0;
        const cv::Point3d ray = cam.Unproject(cv::Point2d(p.x, p.y), &thetaDeg);
        if (thetaDeg > kMaxInitThetaDeg) continue;
        // Unproject inverts the distortion terms iteratively, which can stall
        // far out in a fisheye; drop corners it didn't actually invert.
        const cv::Point2d back = cam.Project(ray);
        if (!(std::hypot(back.x - p.x, back.y - p.y) < 1.0)) continue;
        // Bearing-vector PnP: a camera-frame direction Xs projects to
        // exactly (Xs.x/Xs.z, Xs.y/Xs.z) under an ideal unit-focal pinhole,
        // so solving PnP against those ratios with K = I is exact, not an
        // approximation, as long as they stay finite.
        objPts.push_back(view.objectPoints[i]);
        bearing2d.emplace_back((float)(ray.x / ray.z), (float)(ray.y / ray.z));
    }
    if ((int)objPts.size() < kMinInitCorners) {
        why = "only " + std::to_string(objPts.size()) + " corners within " +
              std::to_string((int)kMaxInitThetaDeg) + " deg of the optical axis for the initial pose";
        return false;
    }
    cv::Mat rvec, tvec;
    if (!cv::solvePnP(objPts, bearing2d, cv::Mat::eye(3, 3, CV_64F), cv::Mat(), rvec, tvec, false,
                      cv::SOLVEPNP_ITERATIVE)) {
        why = "solvePnP failed to converge";
        return false;
    }
    cv::Rodrigues(rvec, pose.R);
    pose.t = cv::Vec3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    return true;
}

} // namespace

bool EstimateBoardPose(const CameraIntrinsics& cam, const BoardView& view, BoardPose& pose, std::string* message) {
    std::string why;
    if (!InitialPose(cam, view, pose, why)) {
        if (message) *message = why;
        return false;
    }

    CameraParams a = cam.params;
    PoseBlock block = ToBlock(pose);
    ceres::Problem problem;
    AddView(problem, cam.model, view, a.data(), block.data());
    problem.SetParameterBlockConstant(a.data());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 50;
    options.logging_type = ceres::SILENT;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (!summary.IsSolutionUsable()) {
        if (message) *message = "pose refinement failed: " + summary.message;
        return false;
    }
    pose = FromBlock(block);
    return true;
}

CalibrationOptions DefaultCalibrationOptions(CameraModel model) {
    CalibrationOptions o;
    switch (model) {
    case CameraModel::Mei: o.fixed[kMeiXi] = true; break;
    case CameraModel::PlumbBob:
    case CameraModel::RationalPolynomial:
    case CameraModel::Equidistant: break;
    }
    return o;
}

CameraIntrinsics GenericGuess(CameraModel model, int width, int height, const std::vector<BoardView>& views) {
    CameraIntrinsics cam;
    cam.model = model;
    cam.width = width;
    cam.height = height;
    cam.params[kCx] = width * 0.5;
    cam.params[kCy] = height * 0.5;
    cam.loaded = true;

    const double edgeTheta = 95.0 * CV_PI / 180.0;
    const double halfMin = 0.5 * std::min(width, height);
    double f = 0.0;
    switch (model) {
    case CameraModel::Mei: {
        cam.params[kMeiXi] = 2.0;
        // Undistorted Mei radius of a ray `theta` off-axis: sin/(cos + xi).
        f = halfMin / (std::sin(edgeTheta) / (std::cos(edgeTheta) + 2.0));
        break;
    }
    case CameraModel::Equidistant: f = halfMin / edgeTheta; break; // r = f * theta
    case CameraModel::PlumbBob:
    case CameraModel::RationalPolynomial: {
        f = 0.5 * width / std::tan(30.0 * CV_PI / 180.0);
        std::vector<std::vector<cv::Point3f>> obj;
        std::vector<std::vector<cv::Point2f>> img;
        for (const BoardView& v : views) {
            if (v.objectPoints.size() < 4) continue;
            obj.push_back(v.objectPoints);
            img.push_back(v.imagePoints);
        }
        if (!obj.empty()) {
            try {
                const cv::Mat K = cv::initCameraMatrix2D(obj, img, cv::Size(width, height), 1.0);
                const double fk = K.at<double>(0, 0);
                if (std::isfinite(fk) && fk > 0.0) f = fk;
            } catch (const cv::Exception& e) {
                std::fprintf(stderr, "calib_app: initCameraMatrix2D failed, using a 60 deg FOV guess: %s\n", e.what());
            }
        }
        break;
    }
    }
    cam.params[kFx] = cam.params[kFy] = f;
    return cam;
}

CalibrationResult Calibrate(const std::vector<BoardView>& views, const CameraIntrinsics& initial,
                            const CalibrationOptions& options) {
    CalibrationResult res;
    res.camera = initial;
    res.viewUsed.assign(views.size(), false);
    res.poses.assign(views.size(), BoardPose{});
    res.viewRmsPx.assign(views.size(), std::numeric_limits<double>::quiet_NaN());

    std::vector<int> fixed;
    for (int k = 0; k < kNp; ++k)
        if (k >= initial.NumParams() || options.fixed[k]) fixed.push_back(k);
    const int nFree = kNp - (int)fixed.size();

    // Initial poses from the starting intrinsics; views that don't give one
    // are left out rather than failing the whole calibration.
    std::vector<size_t> used;
    std::vector<PoseBlock> blocks;
    for (size_t v = 0; v < views.size(); ++v) {
        BoardPose pose;
        std::string why;
        if (!EstimateBoardPose(initial, views[v], pose, &why)) {
            std::fprintf(stderr, "calib_app: calibration skips view %zu: %s\n", v, why.c_str());
            continue;
        }
        used.push_back(v);
        blocks.push_back(ToBlock(pose));
        res.usedCorners += (int)views[v].objectPoints.size();
    }
    res.usedViews = (int)used.size();
    if (res.usedViews < kMinCalibrationViews) {
        res.message = "need >= " + std::to_string(kMinCalibrationViews) + " views with an initial pose, have " +
                      std::to_string(res.usedViews);
        return res;
    }

    double initialCost = 0.0;
    for (size_t u = 0; u < used.size(); ++u) initialCost += ViewCost(initial, views[used[u]], FromBlock(blocks[u]));
    res.initialRmsPx = std::sqrt(initialCost / res.usedCorners);

    CameraParams a = initial.params;
    ceres::Problem problem;
    for (size_t u = 0; u < used.size(); ++u)
        AddView(problem, initial.model, views[used[u]], a.data(), blocks[u].data());
    if (nFree == 0) problem.SetParameterBlockConstant(a.data());
    else if (!fixed.empty()) problem.SetManifold(a.data(), new ceres::SubsetManifold(kNp, fixed));

    ceres::Solver::Options solverOptions;
    // Poses only couple through the intrinsics, so eliminate them (Schur
    // complement) and leave a small dense intrinsics system per iteration.
    solverOptions.linear_solver_type = ceres::DENSE_SCHUR;
    solverOptions.max_num_iterations = options.maxIterations;
    solverOptions.function_tolerance = 1e-12;
    solverOptions.parameter_tolerance = 1e-12;
    solverOptions.num_threads = (int)std::max(1u, std::thread::hardware_concurrency());
    solverOptions.logging_type = ceres::SILENT;
    ceres::Solver::Summary summary;
    ceres::Solve(solverOptions, &problem, &summary);
    res.iterations = (int)summary.iterations.size();
    if (!summary.IsSolutionUsable() || !(a[kFx] > 0.0) || !(a[kFy] > 0.0)) {
        res.message = "optimization failed: " + summary.message;
        return res;
    }

    res.camera.params = a;
    res.camera.loaded = true;
    const double cost = 2.0 * summary.final_cost; // Ceres reports 0.5 * sum of squares
    res.rmsPx = std::sqrt(cost / res.usedCorners);

    // 1-sigma per intrinsic: Ceres' covariance assumes unit-variance
    // residuals, so scale by the variance the fit actually left behind.
    const int dof = 2 * res.usedCorners - nFree - 6 * res.usedViews;
    ceres::Covariance::Options covOptions;
    covOptions.num_threads = solverOptions.num_threads;
    ceres::Covariance covariance(covOptions);
    const std::vector<std::pair<const double*, const double*>> covBlocks = {{a.data(), a.data()}};
    if (nFree > 0 && dof > 0 && covariance.Compute(covBlocks, &problem)) {
        double C[kNp * kNp];
        covariance.GetCovarianceBlock(a.data(), a.data(), C);
        const double sigma2 = cost / dof;
        for (int k = 0; k < kNp; ++k) res.stdDev[k] = std::sqrt(std::max(0.0, sigma2 * C[k * kNp + k]));
    }

    for (size_t u = 0; u < used.size(); ++u) {
        const size_t v = used[u];
        res.viewUsed[v] = true;
        res.poses[v] = FromBlock(blocks[u]);
        res.viewRmsPx[v] = std::sqrt(ViewCost(res.camera, views[v], res.poses[v]) / views[v].objectPoints.size());
    }
    char buf[200];
    std::snprintf(buf, sizeof(buf), "%s, %d views, %d corners: rms %.3f px (was %.3f), %d iterations",
                  initial.Info().name, res.usedViews, res.usedCorners, res.rmsPx, res.initialRmsPx, res.iterations);
    res.message = buf;
    res.ok = true;
    return res;
}