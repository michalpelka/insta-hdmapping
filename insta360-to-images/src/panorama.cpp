#include "insta360/panorama.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace insta360 {

void write_panorama_preview(const PreviewImage& preview, const std::string& output_path) {
    // NV12: a full-size luma plane (height rows) followed by a half-height,
    // full-width interleaved U/V plane -- exactly cv::COLOR_YUV2BGR_NV12's input
    // layout, so this is a format conversion, not a projection.
    cv::Mat nv12(preview.height + preview.height / 2, preview.width, CV_8UC1,
                 const_cast<uint8_t*>(preview.nv12.data()));
    cv::Mat bgr;
    cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);

    if (!cv::imwrite(output_path, bgr)) {
        throw std::runtime_error("OpenCV could not write " + output_path);
    }
}

}  // namespace insta360
