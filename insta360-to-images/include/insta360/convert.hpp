// Turns an Insta360 .insv capture into per-camera folders of timestamped JPEG
// frames, plus an optional IMU CSV and per-camera intrinsics sidecar.
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace insta360 {

struct Options {
    std::string input_path;
    std::string output_dir;
    int jpeg_quality = 3;
    std::optional<std::string> scale;    // "0.5" or "1440x1440"
    std::optional<int64_t> max_frames;
    bool swap_lenses = false;
    bool relative_time = false;
    double time_offset_s = 0.0;  // added to every timestamp written (frames and imu.csv)
    bool include_imu = true;
    bool include_camera_info = true;
    bool include_panorama = true;
    bool include_equirect = false;  // opt-in: geometric per-frame stitch, compute-heavy
    int equirect_width = 3840;      // height is always half this
    bool equirect_rotate_180 = false;  // for consumers expecting row 0 = south pole
    bool force = false;
};

struct Summary {
    std::string output_dir;
    int64_t start_ns = 0;
    int64_t end_ns = 0;
    std::map<std::string, int64_t> counts;  // e.g. "cam_front" -> frames written, "imu" -> samples
    std::vector<std::string> warnings;

    int64_t total() const {
        int64_t sum = 0;
        for (const auto& [_, n] : counts) sum += n;
        return sum;
    }
    double duration_s() const { return end_ns > start_ns ? (end_ns - start_ns) / 1e9 : 0.0; }
};

// Turns a --scale spec into explicit output dimensions. Accepts either a factor
// (0.5) or explicit dimensions (1440x1440). Throws std::invalid_argument on a bad
// spec.
std::pair<int, int> resolve_scale(const std::optional<std::string>& spec, int width, int height);

Summary convert(const Options& options, const std::function<void(const std::string&)>& log = [](const std::string&) {});

}  // namespace insta360
