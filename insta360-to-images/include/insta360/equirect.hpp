// Geometric dual-fisheye -> equirectangular stitching using the camera's own Mei/UCM
// calibration (see calibration.hpp).
//
// This performs the actual projection math -- no feature matching, no per-frame
// solving. It is only as accurate as the calibration and the rotation convention
// below, both cross-checked against an independent reverse-engineering effort and
// against a from-scratch stitcher for this camera that validated the same model
// visually against Insta360's own equirect export (see calibration.hpp for
// references). What it deliberately does *not* do, matching that reference
// implementation's current state: IMU-based stabilization/horizon-locking, rolling-
// shutter correction, or per-channel chromatic-aberration correction. The seam
// blend is a plain linear feather by view angle, reported to work as well as
// optical-flow-based blending for this camera's ~3cm lens baseline.
#pragma once

#include <opencv2/core.hpp>

#include "insta360/calibration.hpp"

namespace insta360 {

// Precomputed backward-mapping tables for one lens onto a fixed equirectangular
// canvas: CV_32FC1 map_x/map_y for cv::remap, and a CV_32FC1 weight in [0, 1] used
// to feather the two lenses together across the seams.
struct EquirectMaps {
    cv::Mat map_x;
    cv::Mat map_y;
    cv::Mat weight;
};

// `is_back_lens` selects the base rotation: identity for the front lens (its axis is
// the equirect's longitude-0 forward direction), a 180-degree rotation about the
// lens' local X axis for the back lens. That specific axis (not a yaw flip about Y)
// was confirmed by checking which one reproduces the observed image-relative
// "up" direction on both raw fisheye frames (see calibration.hpp).
EquirectMaps build_equirect_maps(const LensCalibration& lens, int eq_width, int eq_height,
                                  bool is_back_lens);

// Remaps `fisheye` through `maps` (cv::remap with the given interpolation) and
// returns the result alongside its (already-included) weight map, ready to combine
// with the other lens via blend_equirect.
cv::Mat remap_to_equirect(const cv::Mat& fisheye, const EquirectMaps& maps);

// Combines two already-remapped equirect images using their weight maps: a plain
// weighted average, renormalized where only one lens is valid. `weight_a`/`weight_b`
// are the `weight` fields from the maps used to produce `a`/`b`.
cv::Mat blend_equirect(const cv::Mat& a, const cv::Mat& weight_a, const cv::Mat& b,
                        const cv::Mat& weight_b);

}  // namespace insta360
