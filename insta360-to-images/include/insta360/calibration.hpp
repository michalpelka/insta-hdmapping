// Parsing of the Insta360 lens calibration strings found in the metadata record.
//
// The camera stores calibration as underscore-joined decimal text. The variant that
// matters here is "offset_v2" (metadata field 54/56), one group of 19 values per lens:
//
//   <n>_ [ <xi> <fx> <fy> <cx> <cy> <rx> <ry> <rz> <tx> <ty> <tz>
//          <k1> <k2> <k3> <p1> <p2> <canvas_w> <canvas_h> <crop> ] * n _<trailer>
//
// This is the Mei/Barreto unified spherical camera model, not a plain
// theta-polynomial fisheye model. A ray X in the lens frame (z along the optical
// axis) is normalized onto the unit sphere, then
//
//   x = Xs / (Zs + xi),  y = Ys / (Zs + xi),  r2 = x^2 + y^2
//   xd = x (1 + k1 r2 + k2 r2^2 + k3 r2^3) + 2 p1 x y + p2 (r2 + 2 x^2)
//   yd = y (1 + k1 r2 + k2 r2^2 + k3 r2^3) + p1 (r2 + 2 y^2) + 2 p2 x y
//   u = fx xd + cx,  v = fy yd + cy
//
// On an Insta360 X5, xi is 2.0 for both lenses. This layout was cross-checked
// against an independent reverse-engineering effort
// (github.com/BenjaminHenriksson/insv-stitch) that arrived at the same model and
// field order from a different capture, and against a stitcher for this same camera
// validated visually against Insta360's own equirect export. Confidence: high for
// xi/fx/fy/cx/cy and which fields are radial vs. tangential; lower for the exact
// sign convention of the small (~0.1-0.2 degree) rx/ry tilts.
//
// All pixel coordinates are expressed on a *stitched dual-fisheye canvas* of
// canvas_w x canvas_h in which each lens occupies a canvas_w/2 wide square, so lens i
// has its principal point offset by i * canvas_w/2. Frames stored in the file are
// usually smaller than that canvas, so the intrinsics are rescaled to the actual frame
// size on the way out.
//
// "offset_v3" (field 111/112) is the only calibration an Insta360 X6 writes (the X5
// writes it too, beside offset_v2). Same model and framing, 27 values per lens:
//
//   <n>_ [ <xi> <fx> <fy> <cx> <cy> <rx> <ry> <rz> <tx> <ty> <tz>
//          <k1> <k2> <k3> <k4> <zero> <p1> <p2> <c0> .. <c5>
//          <canvas_w> <canvas_h> <crop> ] * n _<trailer>
//
// i.e. offset_v2 with a fourth radial term (k4 * r^8) and seven more coefficients.
// Established empirically, against ChArUco corners detected in real footage with a
// per-frame board pose fitted and every intrinsic held at the factory value:
//   - X6 (xi = 2.455, both lenses, board within 52 degrees of the axis): 0.43-0.88 px
//     RMS for k1-k4 + p1/p2.
//   - X5 back lens (board out to 120 degrees): k1-k4 + p1/p2 is as good as or
//     better than the X5's own offset_v2 in every 10-15 degree band.
// <zero> has read 0 on every lens seen so far (presumably a k5 slot). <c0..c5> are
// NOT understood: read as OpenCV thin-prism (s1-s4) and/or sensor-tilt (taux, tauy)
// terms they make the X5 fit clearly worse (up to 2-3x), so they are kept verbatim
// in LensCalibration::source_distortion but not applied.
//
// Consumers of camera_info.yaml only know offset_v2's five coefficients, so a v3
// lens is handed out in that form: p1/p2 copied, and k1-k3 refitted by least squares
// to v3's four-term radial curve over the lens' whole image circle (see
// parse_offset_v3). On an X6 that refit stays within ~0.5 px of the k1-k4 curve.
#pragma once

#include <array>
#include <string>
#include <vector>

namespace insta360 {

constexpr const char* DISTORTION_MODEL = "insta360_mei_v2";

// Intrinsics for one fisheye lens, rescaled to the stored frame size.
struct LensCalibration {
    int index = 0;
    int width = 0;
    int height = 0;
    double fx = 0, fy = 0, cx = 0, cy = 0;
    double xi = 0;                           // Mei unified-sphere mirror offset
    double k1 = 0, k2 = 0, k3 = 0;            // radial distortion, on the unit-sphere-projected plane
    double p1 = 0, p2 = 0;                    // tangential distortion
    std::vector<double> distortion;           // the same 5 (k1,k2,k3,p1,p2), for passthrough/display
    std::array<double, 3> rotation_deg{};     // lens yaw/pitch/roll, as reported
    std::array<double, 3> translation{};      // lens translation, in canvas pixels
    std::array<int, 2> canvas{};              // canvas the raw values were expressed on
    std::array<double, 2> scale{};            // rescaling applied to reach width x height
    // Which metadata string this came from ("offset_v2" or "offset_v3"), and that
    // string's distortion coefficients verbatim: 5 for v2 (equal to `distortion`),
    // 13 for v3 (k1-k4, zero, p1, p2, c0-c5).
    std::string source = "offset_v2";
    std::vector<double> source_distortion;
    // offset_v3 only: k1-k3 above are a refit of v3's k1-k4. Worst disagreement
    // between the two, in output pixels, out to refit_max_theta_deg off-axis.
    double refit_max_error_px = 0;
    double refit_max_theta_deg = 0;

    std::array<double, 9> k() const { return {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0}; }
    // No stereo rectification is defined for a back-to-back fisheye pair.
    std::array<double, 9> r() const { return {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}; }
    std::array<double, 12> p() const {
        return {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    }
};

// Parses a field-54/56 calibration string, rescaled to width x height. Returns an
// empty vector if the string does not have the expected shape, so that a firmware
// revision with a different layout degrades to "no calibration" rather than to
// silently wrong intrinsics.
std::vector<LensCalibration> parse_offset_v2(const std::string& text, int width, int height);

// Parses a field-111/112 calibration string the same way. k1-k3 are refitted to the
// string's k1-k4 (see the header comment), out to the angle at which the lens'
// image circle meets the edge of its square, capped at 120 degrees.
std::vector<LensCalibration> parse_offset_v3(const std::string& text, int width, int height);

std::string summarise(const LensCalibration& lens);

}  // namespace insta360
