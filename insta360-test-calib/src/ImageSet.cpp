#include "ImageSet.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <system_error>

namespace fs = std::filesystem;

std::string ExtLower(const std::string& path) {
    const auto pos = path.find_last_of('.');
    if (pos == std::string::npos) return "";
    std::string e = path.substr(pos + 1);
    std::transform(e.begin(), e.end(), e.begin(), [](unsigned char c) { return std::tolower(c); });
    return e;
}

bool IsImageExt(const std::string& e) { return e == "jpg" || e == "jpeg" || e == "png" || e == "bmp"; }

namespace {

// Absolute and normalized, so the same file named two ways (relative on the
// command line, absolute from a drop) is recognized as already loaded.
fs::path Normalized(const std::string& path) {
    std::error_code ec;
    const fs::path abs = fs::absolute(path, ec);
    return (ec ? fs::path(path) : abs).lexically_normal();
}

} // namespace

std::vector<std::string> ExpandInputPath(const std::string& input) {
    const fs::path path = Normalized(input);
    std::error_code ec;
    if (!fs::is_directory(path, ec)) return {path.string()};

    std::vector<std::string> images;
    std::string cameraInfo;
    for (const fs::directory_entry& e : fs::directory_iterator(path, ec)) {
        if (!e.is_regular_file(ec)) continue;
        const std::string p = e.path().string();
        if (IsImageExt(ExtLower(p))) images.push_back(p);
        else if (e.path().filename() == "camera_info.yaml") cameraInfo = p;
    }
    std::sort(images.begin(), images.end());
    // Intrinsics first, so the images that follow are verified against them
    // straight away.
    if (!cameraInfo.empty()) images.insert(images.begin(), cameraInfo);
    return images;
}
