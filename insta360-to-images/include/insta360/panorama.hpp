// Writes out the camera's own firmware-stitched equirectangular preview.
//
// This is *not* a panorama this tool computes: record 0x0002 in the trailer already
// holds a low-resolution (typically 1280x640) equirectangular thumbnail that the
// camera's own hardware stitched from both lenses at capture time. The embedded
// per-lens calibration does not fit any documented fisheye projection model closely
// enough to re-derive that stitch ourselves at full resolution -- see the README --
// so this module's only job is decoding the pixel format (NV12) and writing a JPEG,
// which is exactly the kind of thing OpenCV is for.
#pragma once

#include <string>

#include "insta360/sensors.hpp"

namespace insta360 {

// Converts a decoded preview record to BGR and writes it as a JPEG. Throws
// std::runtime_error if OpenCV fails to encode or write the file.
void write_panorama_preview(const PreviewImage& preview, const std::string& output_path);

}  // namespace insta360
