#include "Calibration.h"
#include "Camera.h"

#include <cmath>
#include <vector>

#include "test_cameras.hpp"
#include "test_util.hpp"

using namespace test_cameras;

namespace {

// With per-axis pixel noise sigma, the per-corner error magnitude has RMS
// sigma*sqrt(2); a fit that has absorbed the model fully sits right there.
constexpr double kNoise = 0.3;
constexpr double kExpectedRms = kNoise * 1.41421356;

// Checks every free intrinsic lands within 4 of its own reported sigmas of
// the truth — which tests the estimate and its uncertainty together. (Over
// 40 noise seeds the normalized error has RMS ~1.0 for every parameter, so
// the reported sigmas are honest and 4 of them is a safe bound.)
void CheckWithinReportedSigma(const CalibrationResult& res, const CameraIntrinsics& truth) {
    // ...and that those sigmas are small, so the bound means something: a
    // couple of dozen views of up to 91 corners pin the focal length to a
    // fraction of a percent and the principal point to a few pixels (the
    // fisheye sweeps do ~5x better than the pinhole ones, which see less of
    // the distortion).
    CHECK(res.stdDev[kFx] > 0.0 && res.stdDev[kFx] < 0.003 * truth.params[kFx]);
    CHECK(res.stdDev[kCx] > 0.0 && res.stdDev[kCx] < 0.0015 * truth.width);
    for (int k = 0; k < truth.NumParams(); ++k) {
        if (res.stdDev[k] == 0.0) continue; // fixed
        CHECK(std::abs(res.camera.params[k] - truth.params[k]) < 4.0 * res.stdDev[k]);
    }
}

// Moves every intrinsic off the truth: focal lengths by a few percent, the
// principal point by tens of pixels, distortion to zero.
CameraIntrinsics Perturbed(CameraIntrinsics cam) {
    cam.params[kFx] *= 1.03;
    cam.params[kFy] *= 0.98;
    cam.params[kCx] += 25.0;
    cam.params[kCy] -= 20.0;
    for (int k = cam.Info().distortionOffset; k < cam.NumParams(); ++k) cam.params[k] = 0.0;
    return cam;
}

void CheckRecovers(const CameraIntrinsics& truth, const std::vector<BoardView>& views, const CameraIntrinsics& start) {
    const CalibrationResult res = Calibrate(views, start, DefaultCalibrationOptions(truth.model));
    CHECK(res.ok);
    CHECK_EQ(res.usedViews, (int)views.size());
    CHECK(res.camera.model == truth.model);
    CHECK_NEAR(res.rmsPx, kExpectedRms, 0.03);
    CheckWithinReportedSigma(res, truth);
    // Unused tail of the parameter array stays untouched.
    for (int k = truth.NumParams(); k < kMaxCameraParams; ++k) CHECK_EQ(res.camera.params[k], 0.0);
}

} // namespace

TEST(estimate_board_pose_recovers_an_exact_pose_including_far_off_axis_corners) {
    const CameraIntrinsics cam = Rig();
    cv::RNG rng(1);
    // Centered 75 deg off-axis: part of the board lies past the 80 deg the
    // initial bearing PnP can take, which the pixel refinement still uses.
    const BoardPose truth = PoseAt(75.0, 30.0, 3000.0, cv::Vec3d(0.2, -0.1, 0.3));
    const BoardView view = Observe(cam, truth, 0.0, rng);
    CHECK(view.objectPoints.size() >= 20);

    BoardPose pose;
    std::string why;
    CHECK(EstimateBoardPose(cam, view, pose, &why));
    CHECK_NEAR(cv::norm(pose.R - truth.R), 0.0, 1e-6);
    CHECK_NEAR(cv::norm(pose.t - truth.t), 0.0, 1e-3);
}

TEST(calibrate_recovers_the_rig_from_a_perturbed_guess) {
    const CameraIntrinsics truth = Rig();
    const std::vector<BoardView> views = FisheyeCapture(truth, kNoise);
    CHECK(views.size() >= 15);

    CameraIntrinsics guess = Perturbed(truth);
    guess.params[6] = 1.0; // k2: the rig's is large, start it somewhere non-zero
    const CalibrationResult res = Calibrate(views, guess, DefaultCalibrationOptions(CameraModel::Mei));
    CHECK(res.ok);
    CHECK(res.initialRmsPx > 5.0);
    CHECK_NEAR(res.rmsPx, kExpectedRms, 0.03);
    CHECK_EQ(res.camera.params[kMeiXi], truth.params[kMeiXi]); // fixed by default
    CHECK_EQ(res.stdDev[kMeiXi], 0.0);
    for (int k = 0; k < truth.NumParams(); ++k)
        if (k != kMeiXi) CHECK(res.stdDev[k] > 0.0);
    CheckWithinReportedSigma(res, truth);
    // Size / identity fields are carried through from the guess.
    CHECK_EQ(res.camera.width, truth.width);
    CHECK_EQ(res.camera.frameId, truth.frameId);
}

TEST(calibrate_converges_from_the_generic_guess_for_the_rig) {
    const CameraIntrinsics truth = Rig();
    CheckRecovers(truth, FisheyeCapture(truth, kNoise), GenericGuess(CameraModel::Mei, 2880, 2880));
}

TEST(calibrate_recovers_an_opencv_fisheye_past_90_degrees) {
    const CameraIntrinsics truth = Fisheye();
    const std::vector<BoardView> views = FisheyeCapture(truth, kNoise);
    CheckRecovers(truth, views, Perturbed(truth));
    CheckRecovers(truth, views, GenericGuess(CameraModel::Equidistant, 2880, 2880));
}

TEST(calibrate_recovers_an_opencv_pinhole) {
    const CameraIntrinsics truth = Pinhole();
    const std::vector<BoardView> views = PinholeCapture(truth, kNoise);
    CHECK(views.size() >= 15);
    CheckRecovers(truth, views, Perturbed(truth));
    // No camera_info at all: the focal length comes from initCameraMatrix2D.
    CheckRecovers(truth, views, GenericGuess(CameraModel::PlumbBob, 1920, 1200, views));
}

TEST(calibrate_fits_the_opencv_rational_model) {
    const CameraIntrinsics truth = Rational();
    const std::vector<BoardView> views = PinholeCapture(truth, kNoise);
    CheckRecovers(truth, views, Perturbed(truth));
}

TEST(calibrate_keeps_fixed_parameters_at_their_initial_values) {
    const CameraIntrinsics truth = Rig();
    const std::vector<BoardView> views = FisheyeCapture(truth, kNoise);
    CameraIntrinsics guess = truth;
    guess.params[7] = -3.0;   // k3
    guess.params[8] = 0.001;  // p1
    guess.params[9] = -0.001; // p2
    CalibrationOptions opts = DefaultCalibrationOptions(CameraModel::Mei);
    opts.fixed[7] = opts.fixed[8] = opts.fixed[9] = true;
    const CalibrationResult res = Calibrate(views, guess, opts);
    CHECK(res.ok);
    CHECK_EQ(res.camera.params[7], -3.0);
    CHECK_EQ(res.camera.params[8], 0.001);
    CHECK_EQ(res.camera.params[9], -0.001);
    CHECK(res.camera.params[5] != guess.params[5]); // k1 still moves
}

TEST(default_options_hold_xi_for_mei_only) {
    CHECK(DefaultCalibrationOptions(CameraModel::Mei).fixed[kMeiXi]);
    for (CameraModel m : {CameraModel::PlumbBob, CameraModel::RationalPolynomial, CameraModel::Equidistant})
        for (bool f : DefaultCalibrationOptions(m).fixed) CHECK(!f);
}

TEST(calibrate_refuses_too_few_views) {
    const CameraIntrinsics truth = Rig();
    std::vector<BoardView> views = FisheyeCapture(truth, kNoise);
    views.resize(kMinCalibrationViews - 1);
    const CalibrationResult res = Calibrate(views, truth, DefaultCalibrationOptions(CameraModel::Mei));
    CHECK(!res.ok);
    CHECK(!res.message.empty());
}