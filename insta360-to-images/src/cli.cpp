// Command line interface for insta360-to-images.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "insta360/calibration.hpp"
#include "insta360/convert.hpp"
#include "insta360/media.hpp"
#include "insta360/metadata.hpp"
#include "insta360/sensors.hpp"
#include "insta360/trailer.hpp"

namespace fs = std::filesystem;
using namespace insta360;

namespace {

const char* kDescription =
    "Convert an Insta360 .insv capture into per-camera folders of timestamped JPEG\n"
    "frames, on the time base recovered from the camera's own clock.\n";

std::string human_bytes(double size) {
    const char* units[] = {"B", "KiB", "MiB", "GiB"};
    for (int i = 0; i < 3; ++i) {
        if (size < 1024 || i == 3) {
            std::ostringstream out;
            if (i == 0) {
                out << static_cast<int64_t>(size) << " B";
            } else {
                out.precision(1);
                out << std::fixed << size << " " << units[i];
            }
            return out.str();
        }
        size /= 1024;
    }
    std::ostringstream out;
    out.precision(1);
    out << std::fixed << size << " GiB";
    return out.str();
}

// std::stod stops at the first character it cannot use, so "1,5" would quietly parse
// as 1. This insists the whole value was consumed.
double parse_seconds(const std::string& text, const std::string& flag) {
    try {
        size_t consumed = 0;
        double value = std::stod(text, &consumed);
        if (consumed == text.size() && std::isfinite(value)) return value;
    } catch (const std::exception&) {
    }
    throw std::invalid_argument(flag + " '" + text + "' is not a number of seconds");
}

void print_usage() {
    std::cout << kDescription << "\n"
              << "Usage: insta360-to-images <input.insv> -o <output_dir> [options]\n\n"
              << "  -o, --output DIR        output directory (default: input filename without\n"
              << "                          extension, beside the input)\n"
              << "  --inspect               print what the file contains and exit\n"
              << "  --jpeg-quality N        ffmpeg MJPEG quality, 2 (best) to 31 (worst); "
                 "default 3\n"
              << "  --scale SPEC            downscale frames: a factor (0.5) or WIDTHxHEIGHT\n"
              << "  --max-frames N          stop after N video frames\n"
              << "  --swap-lenses           map the second video track to cam_front\n"
              << "  --no-imu                leave out imu.csv\n"
              << "  --no-camera-info        leave out the per-camera intrinsics sidecar\n"
              << "  --no-panorama           leave out panorama.jpg (the camera's own\n"
              << "                          firmware-stitched preview)\n"
              << "  --no-equirect           skip stitching the geometric per-frame equirect\n"
              << "                          panorama video (compute-heavy; see README)\n"
              << "  --equirect-width N      equirect output width, height is N/2; default 3840\n"
              << "  --no-equirect-flip      don't rotate the equirect output 180 degrees "
                 "(row 0 =\n"
              << "                          north pole instead of south)\n"
              << "  --relative-time         start timestamps at zero instead of the camera's "
                 "wall clock\n"
              << "  --time-offset SECONDS   shift every written timestamp by SECONDS; may be\n"
              << "                          fractional or negative\n"
              << "  -f, --force             write into a non-empty output directory\n"
              << "  -q, --quiet             only report warnings and errors\n"
              << "  -h, --help              show this help\n";
}

struct ParsedArgs {
    std::string input;
    std::optional<std::string> output;
    bool inspect = false;
    int jpeg_quality = 3;
    std::optional<std::string> scale;
    std::optional<int64_t> max_frames;
    bool swap_lenses = false;
    bool include_imu = true;
    bool include_camera_info = true;
    bool include_panorama = true;
    bool include_equirect = true;
    int equirect_width = 3840;
    bool equirect_flip = true;
    bool relative_time = false;
    double time_offset_s = 0.0;
    bool force = false;
    bool quiet = false;
};

// Returns nullopt (and prints usage) for --help, throws std::invalid_argument on a
// bad command line.
std::optional<ParsedArgs> parse_args(int argc, char** argv) {
    ParsedArgs args;
    bool have_input = false;
    std::vector<std::string> tokens(argv + 1, argv + argc);

    auto next_value = [&](size_t& i, const std::string& flag) -> std::string {
        if (i + 1 >= tokens.size()) throw std::invalid_argument(flag + " requires a value");
        return tokens[++i];
    };

    for (size_t i = 0; i < tokens.size(); ++i) {
        const std::string& token = tokens[i];
        if (token == "-h" || token == "--help") {
            print_usage();
            return std::nullopt;
        } else if (token == "-o" || token == "--output") {
            args.output = next_value(i, token);
        } else if (token == "--inspect") {
            args.inspect = true;
        } else if (token == "--jpeg-quality") {
            args.jpeg_quality = std::stoi(next_value(i, token));
        } else if (token == "--scale") {
            args.scale = next_value(i, token);
        } else if (token == "--max-frames") {
            args.max_frames = std::stoll(next_value(i, token));
        } else if (token == "--swap-lenses") {
            args.swap_lenses = true;
        } else if (token == "--no-imu") {
            args.include_imu = false;
        } else if (token == "--no-camera-info") {
            args.include_camera_info = false;
        } else if (token == "--no-panorama") {
            args.include_panorama = false;
        } else if (token == "--equirect") {
            args.include_equirect = true;
        } else if (token == "--no-equirect") {
            args.include_equirect = false;
        } else if (token == "--equirect-width") {
            args.equirect_width = std::stoi(next_value(i, token));
        } else if (token == "--equirect-flip") {
            args.equirect_flip = true;
        } else if (token == "--no-equirect-flip") {
            args.equirect_flip = false;
        } else if (token == "--relative-time") {
            args.relative_time = true;
        } else if (token == "--time-offset") {
            args.time_offset_s = parse_seconds(next_value(i, token), token);
        } else if (token == "-f" || token == "--force") {
            args.force = true;
        } else if (token == "-q" || token == "--quiet") {
            args.quiet = true;
        } else if (!token.empty() && token[0] == '-' && token != "-") {
            throw std::invalid_argument("unrecognized option: " + token);
        } else if (!have_input) {
            args.input = token;
            have_input = true;
        } else {
            throw std::invalid_argument("unexpected extra argument: " + token);
        }
    }
    if (!have_input) throw std::invalid_argument("input file is required");
    return args;
}

int run_inspect(const std::string& path) {
    Trailer trailer(path);
    Metadata meta = parse_metadata(trailer.read(REC_METADATA));
    Probe container_probe = probe(path);

    std::cout << "file          " << path << "\n";
    std::cout << "size          " << human_bytes(static_cast<double>(trailer.file_size())) << "\n";
    std::cout << "trailer       version " << trailer.version() << ", "
              << human_bytes(static_cast<double>(trailer.file_size() - trailer.base_offset()))
              << " at offset " << trailer.base_offset() << "\n\n";

    std::cout << "camera        " << (meta.model ? *meta.model : "?") << "  serial "
              << (meta.serial ? *meta.serial : "?") << "\n";
    std::cout << "firmware      " << (meta.firmware ? *meta.firmware : "?") << "\n";
    std::cout << "captured      " << (meta.capture_datetime ? *meta.capture_datetime : "?")
              << " (camera local time)\n";
    if (meta.start_epoch_ms) {
        time_t seconds = static_cast<time_t>(*meta.start_epoch_ms / 1000);
        char buf[64];
        struct tm tm_utc;
        gmtime_r(&seconds, &tm_utc);
        strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
        std::cout << "first frame   " << buf << " (utc)\n";
    }
    std::cout << "profile       " << (meta.profile ? *meta.profile : "?") << "\n";
    std::cout << "source        " << (meta.source_path ? *meta.source_path : "?") << "\n\n";

    std::cout << "frames        " << (meta.frame_count ? std::to_string(*meta.frame_count) : "?")
              << " at " << (meta.fps ? std::to_string(*meta.fps) : "?") << " fps, "
              << (meta.width ? std::to_string(*meta.width) : "?") << "x"
              << (meta.height ? std::to_string(*meta.height) : "?") << " per lens\n";
    std::cout << "duration      " << (meta.duration_s ? std::to_string(*meta.duration_s) : "?")
              << " s\n";
    std::cout << "imu ranges    +/-" << meta.accel_range_g << " g, +/-" << meta.gyro_range_dps
              << " dps" << (meta.sensor_ranges_known ? "" : "  (assumed; not in metadata)") << "\n\n";

    std::cout << "container streams\n";
    for (const auto& stream : container_probe.video) {
        std::cout << "  video " << stream.index << "  " << stream.codec << " "
                  << (stream.width ? std::to_string(*stream.width) : "?") << "x"
                  << (stream.height ? std::to_string(*stream.height) : "?") << " @ "
                  << (stream.frame_rate ? std::to_string(*stream.frame_rate) : "?") << " fps\n";
    }
    for (const auto& stream : container_probe.audio) {
        std::cout << "  audio " << stream.index << "  " << stream.codec << " "
                  << (stream.sample_rate ? std::to_string(*stream.sample_rate) : "?") << " Hz, "
                  << (stream.channels ? std::to_string(*stream.channels) : "?") << " ch\n";
    }
    std::cout << "\n";

    std::cout << "trailer records\n";
    for (const auto& record : trailer.records()) {
        char id_buf[16];
        std::snprintf(id_buf, sizeof(id_buf), "0x%04x", record.id);
        std::cout << "  " << id_buf << "  " << human_bytes(record.length) << "  " << record.name()
                  << "\n";
    }
    std::cout << "\n";

    if (trailer.contains(REC_IMU)) {
        auto samples = read_imu(trailer.read(REC_IMU), meta.accel_range_g, meta.gyro_range_dps);
        if (!samples.empty()) {
            double span = (samples.back().device_us - samples.front().device_us) / 1e6;
            double rate = span > 0 ? (samples.size() - 1) / span : 0.0;
            double sum_mag = 0;
            for (const auto& s : samples) {
                sum_mag += std::sqrt(s.accel[0] * s.accel[0] + s.accel[1] * s.accel[1] +
                                      s.accel[2] * s.accel[2]);
            }
            double mean_g = (sum_mag / samples.size()) / STANDARD_GRAVITY;
            std::cout << "imu           " << samples.size() << " samples over " << span << " s ("
                      << rate << " Hz)\n";
            std::cout << "              mean |accel| = " << mean_g
                      << " g (should sit near 1.0 for a handheld clip)\n";
        }
    }

    if (trailer.contains(REC_EXPOSURE)) {
        auto exposures = read_exposure(trailer.read(REC_EXPOSURE));
        if (!exposures.empty()) {
            double min_ms = exposures.front().exposure_s * 1000, max_ms = min_ms;
            for (const auto& e : exposures) {
                min_ms = std::min(min_ms, e.exposure_s * 1000);
                max_ms = std::max(max_ms, e.exposure_s * 1000);
            }
            std::cout << "exposure      " << exposures.size() << " entries, " << min_ms << " to "
                      << max_ms << " ms\n";
        }
    }

    if (!meta.lenses.empty()) {
        std::cout << "\ncalibration (rescaled to the stored frame size)\n";
        for (const auto& lens : meta.lenses) {
            std::cout << "  " << summarise(lens) << "\n";
            std::cout << "    distortion [";
            for (size_t i = 0; i < lens.distortion.size(); ++i) {
                if (i) std::cout << ", ";
                std::cout << lens.distortion[i];
            }
            std::cout << "]\n";
        }
    }

    auto problems = trailer.footer_mismatches();
    if (!problems.empty()) {
        std::cout << "\nwarnings\n";
        for (const auto& problem : problems) std::cout << "  " << problem << "\n";
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    std::optional<ParsedArgs> maybe_args;
    try {
        maybe_args = parse_args(argc, argv);
    } catch (const std::exception& exc) {
        std::cerr << "error: " << exc.what() << "\n";
        print_usage();
        return 2;
    }
    if (!maybe_args) return 0;  // --help
    ParsedArgs args = *maybe_args;

    if (!fs::exists(args.input)) {
        std::cerr << "error: input file not found: " << args.input << "\n";
        return 2;
    }

    try {
        require_tools();
    } catch (const FFmpegError& exc) {
        std::cerr << "error: " << exc.what() << "\n";
        return 2;
    }

    if (args.inspect) {
        try {
            return run_inspect(args.input);
        } catch (const std::exception& exc) {
            std::cerr << "error: " << exc.what() << "\n";
            return 1;
        }
    }

    std::string output_dir = args.output.value_or(fs::path(args.input).replace_extension("").string());

    if (fs::exists(output_dir) && !fs::is_empty(output_dir) && !args.force) {
        std::cerr << "error: " << output_dir << " already exists and is not empty; pass --force to "
                                                 "write into it\n";
        return 2;
    }

    if (args.scale) {
        try {
            resolve_scale(args.scale, 100, 100);
        } catch (const std::invalid_argument& exc) {
            std::cerr << "error: " << exc.what() << "\n";
            return 2;
        }
    }

    Options options;
    options.input_path = args.input;
    options.output_dir = output_dir;
    options.jpeg_quality = args.jpeg_quality;
    options.scale = args.scale;
    options.max_frames = args.max_frames;
    options.swap_lenses = args.swap_lenses;
    options.relative_time = args.relative_time;
    options.time_offset_s = args.time_offset_s;
    options.include_imu = args.include_imu;
    options.include_camera_info = args.include_camera_info;
    options.include_panorama = args.include_panorama;
    options.include_equirect = args.include_equirect;
    options.equirect_width = args.equirect_width;
    options.equirect_rotate_180 = args.equirect_flip;
    options.force = args.force;

    auto log = args.quiet ? std::function<void(const std::string&)>([](const std::string&) {})
                           : std::function<void(const std::string&)>([](const std::string& message) {
                                 std::cout << message << std::endl;
                             });

    Summary summary;
    try {
        summary = convert(options, log);
    } catch (const std::exception& exc) {
        std::cerr << "error: " << exc.what() << "\n";
        return 1;
    }

    if (!args.quiet) {
        std::cout << "\nwrote " << summary.output_dir << " (" << summary.total() << " files, "
                  << summary.duration_s() << " s)\n";
        for (const auto& [name, count] : summary.counts) {
            std::cout << "  " << count << "  " << name << "\n";
        }
    }
    for (const auto& warning : summary.warnings) {
        std::cerr << "warning: " << warning << "\n";
    }
    return 0;
}
