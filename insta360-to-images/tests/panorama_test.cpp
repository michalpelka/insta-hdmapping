#include "insta360/panorama.hpp"

#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <cstdio>
#include <filesystem>

#include "test_util.hpp"

using namespace insta360;

TEST(write_panorama_preview_produces_a_readable_bgr_image_of_the_right_size) {
    // Flat mid-gray NV12: Y=128 everywhere, U=V=128 (no color) -- exercises the
    // format conversion without depending on any particular test image.
    const int width = 16, height = 8;
    PreviewImage preview;
    preview.width = width;
    preview.height = height;
    preview.nv12.assign(static_cast<size_t>(width) * height * 3 / 2, 128);

    std::string path = (std::filesystem::temp_directory_path() / "insta360_panorama_test.jpg").string();
    write_panorama_preview(preview, path);

    cv::Mat readback = cv::imread(path, cv::IMREAD_COLOR);
    CHECK(!readback.empty());
    CHECK_EQ(readback.cols, width);
    CHECK_EQ(readback.rows, height);
    // Mid-gray in, mid-gray out (loose tolerance: JPEG is lossy).
    cv::Vec3b center = readback.at<cv::Vec3b>(height / 2, width / 2);
    for (int c = 0; c < 3; ++c) CHECK(std::abs(static_cast<int>(center[c]) - 128) < 20);

    std::remove(path.c_str());
}
