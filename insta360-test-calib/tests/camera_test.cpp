#include "Camera.h"

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <string>
#include <vector>

#include "test_cameras.hpp"
#include "test_util.hpp"

using namespace test_cameras;

namespace {

// Random camera-frame points in front of the camera, up to `maxThetaDeg`
// off-axis, at assorted distances.
std::vector<cv::Point3d> PointsInFront(double maxThetaDeg, int n) {
    cv::RNG rng(7);
    std::vector<cv::Point3d> pts;
    for (int i = 0; i < n; ++i) {
        const double t = rng.uniform(0.0, maxThetaDeg) * CV_PI / 180.0, p = rng.uniform(0.0, 2 * CV_PI);
        const double d = rng.uniform(0.5, 20.0);
        pts.emplace_back(d * std::sin(t) * std::cos(p), d * std::sin(t) * std::sin(p), d * std::cos(t));
    }
    return pts;
}

cv::Matx33d K(const CameraIntrinsics& cam) {
    return {cam.params[kFx], 0, cam.params[kCx], 0, cam.params[kFy], cam.params[kCy], 0, 0, 1};
}

std::vector<double> Distortion(const CameraIntrinsics& cam) {
    return std::vector<double>(cam.params.begin() + cam.Info().distortionOffset,
                               cam.params.begin() + cam.NumParams());
}

void CheckMatchesOpenCv(const CameraIntrinsics& cam, const std::vector<cv::Point3d>& pts,
                        const std::vector<cv::Point2d>& opencv) {
    for (size_t i = 0; i < pts.size(); ++i) {
        const cv::Point2d ours = cam.Project(pts[i]);
        CHECK_NEAR(ours.x, opencv[i].x, 1e-6);
        CHECK_NEAR(ours.y, opencv[i].y, 1e-6);
    }
}

void CheckRoundTrip(const CameraIntrinsics& cam, double maxThetaDeg) {
    for (double th = 0.0; th <= maxThetaDeg; th += 5.0) {
        const double t = th * CV_PI / 180.0;
        const cv::Point3d ray(std::sin(t) * 0.6, std::sin(t) * 0.8, std::cos(t));
        const cv::Point2d uv = cam.Project(ray * 3.0); // scale must not matter
        double thetaDeg = 0.0;
        const cv::Point3d back = cam.Unproject(uv, &thetaDeg);
        CHECK_NEAR(thetaDeg, th, 1e-6);
        CHECK_NEAR(cv::norm(back - ray), 0.0, 1e-9);
    }
}

} // namespace

TEST(model_table_names_round_trip_and_match_the_parameter_layout) {
    for (int m = 0; m < kCameraModelCount; ++m) {
        const CameraModel model = static_cast<CameraModel>(m);
        const CameraModelInfo& info = ModelInfo(model);
        CameraModel parsed;
        CHECK(ParseCameraModel(info.name, parsed));
        CHECK(parsed == model);
        CHECK(info.numParams <= kMaxCameraParams);
        for (int k = 0; k < kMaxCameraParams; ++k) CHECK((info.paramNames[k] != nullptr) == (k < info.numParams));
        CHECK_EQ(std::string(info.paramNames[kFx]), std::string("fx"));
        CHECK_EQ(std::string(info.paramNames[kCy]), std::string("cy"));
        CHECK_EQ(std::string(info.paramNames[info.distortionOffset]), std::string("k1"));
    }
    CHECK_EQ(std::string(ModelInfo(CameraModel::Mei).paramNames[kMeiXi]), std::string("xi"));
    CameraModel alias;
    CHECK(ParseCameraModel("fisheye", alias));
    CHECK(alias == CameraModel::Equidistant);
    CHECK(!ParseCameraModel("kannala_brandt_but_not_really", alias));
}

TEST(plumb_bob_projects_exactly_like_cv_projectPoints) {
    const CameraIntrinsics cam = Pinhole();
    const std::vector<cv::Point3d> pts = PointsInFront(45.0, 200);
    std::vector<cv::Point2d> opencv;
    cv::projectPoints(pts, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K(cam), Distortion(cam), opencv);
    CheckMatchesOpenCv(cam, pts, opencv);
}

TEST(rational_polynomial_projects_exactly_like_cv_projectPoints) {
    const CameraIntrinsics cam = Rational();
    const std::vector<cv::Point3d> pts = PointsInFront(50.0, 200);
    std::vector<cv::Point2d> opencv;
    cv::projectPoints(pts, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K(cam), Distortion(cam), opencv);
    CheckMatchesOpenCv(cam, pts, opencv);
}

TEST(equidistant_projects_exactly_like_cv_fisheye_projectPoints) {
    const CameraIntrinsics cam = Fisheye();
    const std::vector<cv::Point3d> pts = PointsInFront(85.0, 200);
    std::vector<cv::Point2d> opencv;
    cv::fisheye::projectPoints(pts, opencv, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K(cam), Distortion(cam));
    CheckMatchesOpenCv(cam, pts, opencv);
}

TEST(project_and_unproject_round_trip_for_every_model) {
    CheckRoundTrip(Rig(), 90.0);
    CheckRoundTrip(Fisheye(), 100.0); // past 90 deg, where cv::fisheye's atan(r) would stop
    CheckRoundTrip(Pinhole(), 40.0);
    CheckRoundTrip(Rational(), 45.0);
}

TEST(pinhole_models_refuse_points_behind_the_camera_and_fisheye_takes_them) {
    const cv::Point3d sideways(1.0, 0.2, -0.05); // ~93 deg off-axis
    for (const CameraIntrinsics& cam : {Pinhole(), Rational()}) {
        const cv::Point2d uv = cam.Project(sideways);
        CHECK(std::isnan(uv.x) && std::isnan(uv.y));
    }
    const cv::Point2d uv = Fisheye().Project(sideways);
    CHECK(std::isfinite(uv.x) && std::isfinite(uv.y));
    // Straight down the optical axis must not divide by zero.
    const cv::Point2d center = Fisheye().Project(cv::Point3d(0, 0, 2));
    CHECK_NEAR(center.x, Fisheye().params[kCx], 1e-12);
    CHECK_NEAR(center.y, Fisheye().params[kCy], 1e-12);
}