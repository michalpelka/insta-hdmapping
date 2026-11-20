#include "insta360/convert.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "insta360/calibration.hpp"
#include "insta360/imu_align.hpp"
#include "insta360/media.hpp"
#include "insta360/metadata.hpp"
#include "insta360/sensors.hpp"
#include "insta360/trailer.hpp"

namespace fs = std::filesystem;

namespace insta360 {

namespace {

//: (folder name, frame id suffix) for the two fisheye lenses, in file order.
const std::vector<std::string> kLensNames = {"cam_front", "cam_back"};

// Rounds to a positive even integer; MJPEG's 4:2:0 chroma needs even dimensions.
int round_even(double value) {
    int rounded = static_cast<int>(std::llround(value / 2.0)) * 2;
    return std::max(2, rounded);
}

std::string lower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                    [](unsigned char c) { return std::tolower(c); });
    return text;
}

// Maps camera device microseconds to the nanosecond stamps written to disk.
class TimeBase {
public:
    TimeBase(std::optional<Clock> clock, int64_t origin_us, bool relative, double offset_s)
        : clock_(relative ? std::nullopt : clock),
          origin_us_(origin_us),
          offset_ns_(static_cast<int64_t>(std::llround(offset_s * 1e9))) {}

    int64_t operator()(int64_t device_us) const {
        if (clock_) return clock_->to_epoch_ns(device_us) + offset_ns_;
        return (device_us - origin_us_) * 1000 + offset_ns_;
    }

private:
    std::optional<Clock> clock_;
    int64_t origin_us_;
    int64_t offset_ns_;  // whole-timeline shift from Options::time_offset_s
};

struct Camera {
    std::string name;
    int video_index;
    fs::path dir;
    int width, height;
    // Camera body identity, from the metadata record; any of these can be absent.
    std::optional<std::string> serial;
    std::optional<std::string> model;
    std::optional<std::string> firmware;
    std::optional<LensCalibration> calibration;
};

std::map<int, LensCalibration> lens_calibration(const Metadata& meta, int width, int height,
                                                 Summary& summary) {
    if (!meta.calibration_string("offset_v2") && !meta.calibration_string("offset_v3")) {
        summary.warnings.push_back("no lens calibration in metadata; intrinsics sidecar omitted");
        return {};
    }
    auto lenses = parse_lenses(meta, width, height);
    if (lenses.empty()) {
        summary.warnings.push_back(
            "lens calibration string had an unexpected layout; intrinsics sidecar omitted");
        return {};
    }
    std::map<int, LensCalibration> byIndex;
    for (auto& lens : lenses) byIndex[lens.index] = lens;
    return byIndex;
}

std::string yaml_number(double value) {
    std::ostringstream out;
    out.precision(10);
    out << value;
    return out.str();
}

// Double-quoted scalar: a serial like "0123" must not come back as a number, and
// the camera is free to put anything in that field.
std::string yaml_string(const std::string& text) {
    std::ostringstream out;
    out << '"';
    for (char c : text) {
        if (c == '"' || c == '\\') out << '\\';
        out << c;
    }
    out << '"';
    return out.str();
}

std::string yaml_array(const std::vector<double>& values) {
    std::ostringstream out;
    out << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i) out << ", ";
        out << yaml_number(values[i]);
    }
    out << "]";
    return out.str();
}

void write_camera_info(const fs::path& path, const Camera& camera) {
    const LensCalibration& lens = *camera.calibration;
    std::ofstream out(path);
    out << "frame_id: insta360_" << camera.name << "_optical_frame\n";
    out << "serial: " << (camera.serial ? yaml_string(*camera.serial) : "null") << "\n";
    out << "model: " << (camera.model ? yaml_string(*camera.model) : "null") << "\n";
    out << "firmware: " << (camera.firmware ? yaml_string(*camera.firmware) : "null") << "\n";
    out << "video_stream_index: " << camera.video_index << "\n";
    out << "width: " << camera.width << "\n";
    out << "height: " << camera.height << "\n";
    out << "distortion_model: " << DISTORTION_MODEL << "\n";
    out << "xi: " << yaml_number(lens.xi) << "\n";
    out << "fx: " << yaml_number(lens.fx) << "\n";
    out << "fy: " << yaml_number(lens.fy) << "\n";
    out << "cx: " << yaml_number(lens.cx) << "\n";
    out << "cy: " << yaml_number(lens.cy) << "\n";
    out << "distortion: " << yaml_array(lens.distortion) << "\n";
    // Store each matrix before iterating: lens.k().begin() and lens.k().end() would
    // otherwise be iterators into two different temporaries.
    std::array<double, 9> k = lens.k();
    std::array<double, 9> r = lens.r();
    std::array<double, 12> p = lens.p();
    out << "k: " << yaml_array(std::vector<double>(k.begin(), k.end())) << "\n";
    out << "r: " << yaml_array(std::vector<double>(r.begin(), r.end())) << "\n";
    out << "p: " << yaml_array(std::vector<double>(p.begin(), p.end())) << "\n";
    out << "rotation_deg: "
        << yaml_array(std::vector<double>(lens.rotation_deg.begin(), lens.rotation_deg.end())) << "\n";
    out << "translation: "
        << yaml_array(std::vector<double>(lens.translation.begin(), lens.translation.end())) << "\n";
    out << "calibration_source: " << lens.source << "\n";
    if (lens.source != "offset_v2") {
        // `distortion` above is a refit (see calibration.hpp); keep what the camera
        // actually wrote alongside it, and how far the refit strays from it.
        out << "source_distortion: " << yaml_array(lens.source_distortion) << "\n";
        out << "refit_max_error_px: " << yaml_number(lens.refit_max_error_px) << "\n";
        out << "refit_max_theta_deg: " << yaml_number(lens.refit_max_theta_deg) << "\n";
    }
    if (!out) throw std::runtime_error("could not write " + path.string());
}

void write_imu_csv(const fs::path& path, const std::vector<ImuSample>& samples,
                    const TimeBase& time_of) {
    std::ofstream out(path);
    out << "timestamp_ns,accel_x_mps2,accel_y_mps2,accel_z_mps2,gyro_x_radps,gyro_y_radps,"
           "gyro_z_radps\n";
    for (const auto& sample : samples) {
        out << time_of(sample.device_us) << ',' << sample.accel[0] << ',' << sample.accel[1] << ','
            << sample.accel[2] << ',' << sample.gyro[0] << ',' << sample.gyro[1] << ','
            << sample.gyro[2] << '\n';
    }
    if (!out) throw std::runtime_error("could not write " + path.string());
}

}  // namespace

std::pair<int, int> resolve_scale(const std::optional<std::string>& spec, int width, int height) {
    if (!spec) return {width, height};
    std::string text = lower(*spec);
    // Trim whitespace.
    size_t begin = text.find_first_not_of(" \t");
    size_t last = text.find_last_not_of(" \t");
    text = (begin == std::string::npos) ? "" : text.substr(begin, last - begin + 1);

    auto x_pos = text.find('x');
    if (x_pos != std::string::npos) {
        try {
            size_t consumed = 0;
            int out_width = std::stoi(text.substr(0, x_pos), &consumed);
            if (consumed != x_pos) throw std::invalid_argument("");
            std::string right = text.substr(x_pos + 1);
            size_t consumed2 = 0;
            int out_height = std::stoi(right, &consumed2);
            if (consumed2 != right.size()) throw std::invalid_argument("");
            if (out_width <= 0 || out_height <= 0) {
                throw std::invalid_argument("--scale '" + *spec + "' must be positive");
            }
            return {out_width, out_height};
        } catch (const std::invalid_argument&) {
            throw std::invalid_argument("could not parse --scale '" + *spec + "' as WIDTHxHEIGHT");
        } catch (const std::out_of_range&) {
            throw std::invalid_argument("could not parse --scale '" + *spec + "' as WIDTHxHEIGHT");
        }
    }
    try {
        size_t consumed = 0;
        double factor = std::stod(text, &consumed);
        if (consumed != text.size()) throw std::invalid_argument("");
        if (!(factor > 0) || factor > 1) {
            throw std::invalid_argument("--scale factor must be greater than 0 and at most 1");
        }
        return {round_even(width * factor), round_even(height * factor)};
    } catch (const std::invalid_argument&) {
        throw std::invalid_argument("could not parse --scale '" + *spec + "' as a factor or WIDTHxHEIGHT");
    } catch (const std::out_of_range&) {
        throw std::invalid_argument("could not parse --scale '" + *spec + "' as a factor or WIDTHxHEIGHT");
    }
}

Summary convert(const Options& options, const std::function<void(const std::string&)>& log) {
    require_tools();
    if (options.frame_step < 1) throw std::invalid_argument("frame step must be at least 1");
    Summary summary;
    summary.output_dir = options.output_dir;

    Trailer trailer(options.input_path);
    for (const auto& problem : trailer.footer_mismatches()) {
        summary.warnings.push_back("trailer inconsistency: " + problem);
    }

    Metadata meta = parse_metadata(trailer.read(REC_METADATA));
    Probe container_probe = probe(options.input_path);
    log((meta.model ? *meta.model : "unknown camera") + " (serial " +
        (meta.serial ? *meta.serial : "?") + ", firmware " + (meta.firmware ? *meta.firmware : "?") +
        ")");

    if (!meta.sensor_ranges_known) {
        std::ostringstream warning;
        warning << "metadata did not report IMU full-scale ranges; assuming +/-" << meta.accel_range_g
                << " g and +/-" << meta.gyro_range_dps << " dps";
        summary.warnings.push_back(warning.str());
    }

    // -- geometry and frame timing --------------------------------------------------
    std::optional<int> source_width = meta.width;
    std::optional<int> source_height = meta.height;
    if (!source_width && !container_probe.video.empty()) source_width = container_probe.video[0].width;
    if (!source_height && !container_probe.video.empty()) source_height = container_probe.video[0].height;
    if (!source_width || !source_height) {
        throw std::runtime_error("could not determine the video frame size");
    }
    auto [out_width, out_height] = resolve_scale(options.scale, *source_width, *source_height);
    std::optional<std::string> scale_filter;
    if (out_width != *source_width || out_height != *source_height) {
        scale_filter = std::to_string(out_width) + ":" + std::to_string(out_height);
    }

    int64_t frame_count = meta.frame_count.value_or(0);
    if (frame_count <= 0 && container_probe.duration_s && meta.fps) {
        frame_count = static_cast<int64_t>(std::llround(*container_probe.duration_s * *meta.fps));
    }
    if (options.max_frames) frame_count = std::min(frame_count, *options.max_frames);
    if (frame_count <= 0) {
        throw std::runtime_error("could not determine how many video frames the file holds");
    }

    std::vector<ExposureSample> exposures;
    if (trailer.contains(REC_EXPOSURE)) exposures = read_exposure(trailer.read(REC_EXPOSURE));

    FrameTimestamps frames =
        frame_timestamps(exposures, meta.first_frame_us, frame_count, meta.frame_interval_us());
    bool any_exposure = std::any_of(frames.exposure_s.begin(), frames.exposure_s.end(),
                                     [](const auto& v) { return v.has_value(); });
    if (!any_exposure) {
        summary.warnings.push_back(
            "no exposure record covering the clip; frame timestamps fall back to the nominal "
            "frame rate");
    }

    std::vector<ImuSample> imu_samples;
    if ((options.include_imu || options.align_imu_dir) && trailer.contains(REC_IMU)) {
        imu_samples = read_imu(trailer.read(REC_IMU), meta.accel_range_g, meta.gyro_range_dps);
    } else if (options.align_imu_dir) {
        throw std::runtime_error("file carries no IMU record, so it cannot be aligned by IMU");
    } else if (options.include_imu) {
        summary.warnings.push_back("file carries no IMU record");
    }

    // The output starts at whichever stream begins first; the IMU typically runs for
    // about a second before the first encoded frame.
    std::vector<int64_t> candidate_starts;
    if (!frames.device_us.empty()) candidate_starts.push_back(frames.device_us.front());
    if (!imu_samples.empty()) candidate_starts.push_back(imu_samples.front().device_us);
    if (meta.first_frame_us) candidate_starts.push_back(*meta.first_frame_us);
    if (candidate_starts.empty()) {
        throw std::runtime_error("nothing in this file carries a usable timestamp");
    }
    int64_t origin_us = *std::min_element(candidate_starts.begin(), candidate_starts.end());

    std::optional<Clock> clock = meta.clock();
    if (!clock && !options.relative_time) {
        summary.warnings.push_back("metadata carries no wall-clock reference; timestamps start at zero");
    }
    double offset_s = options.time_offset_s;
    if (options.align_imu_dir) {
        if (!clock || options.relative_time) {
            throw std::runtime_error(
                "IMU alignment needs the camera's wall clock (not --relative-time)");
        }
        log("aligning camera IMU against " + *options.align_imu_dir);
        GyroSeries reference = read_gyro_csv_dir(*options.align_imu_dir);
        {
            std::ostringstream message;
            message.precision(1);
            message << std::fixed << "reference IMU: " << reference.source << ", "
                    << reference.t_ns.size() << " samples over "
                    << (reference.t_ns.back() - reference.t_ns.front()) / 1e9 << " s";
            log(message.str());
        }
        TimeBase unshifted(clock, origin_us, false, 0.0);
        GyroSeries camera_gyro;
        for (const auto& sample : imu_samples) {
            camera_gyro.t_ns.push_back(unshifted(sample.device_us));
            camera_gyro.magnitude_radps.push_back(std::sqrt(sample.gyro[0] * sample.gyro[0] +
                                                            sample.gyro[1] * sample.gyro[1] +
                                                            sample.gyro[2] * sample.gyro[2]));
        }
        TimeAlignment alignment =
            align_gyro_magnitudes(reference, camera_gyro, options.align_max_lag_s);
        if (alignment.correlation < 0.5) {
            throw std::runtime_error("IMU alignment found no convincing match (best r = " +
                                     std::to_string(alignment.correlation) + ")");
        }
        if (alignment.correlation < 0.9) {
            summary.warnings.push_back("IMU alignment correlation is only " +
                                       std::to_string(alignment.correlation) +
                                       "; check the offset before trusting it");
        }
        if (alignment.at_search_edge) {
            summary.warnings.push_back(
                "IMU alignment peak sits at the edge of the search window; the real offset may "
                "be larger");
        }
        std::ostringstream message;
        message.precision(6);
        message << std::fixed << "IMU alignment: offset " << alignment.offset_s << " s, r = "
                << alignment.correlation << " over " << alignment.overlap_s << " s";
        log(message.str());
        summary.aligned_offset_s = alignment.offset_s;
        summary.aligned_correlation = alignment.correlation;
        offset_s += alignment.offset_s;
    }
    TimeBase time_of(clock, origin_us, options.relative_time || !clock, offset_s);

    // -- lenses -----------------------------------------------------------------
    std::vector<int> lens_order(container_probe.video.size());
    for (size_t i = 0; i < lens_order.size(); ++i) lens_order[i] = static_cast<int>(i);
    if (options.swap_lenses) std::reverse(lens_order.begin(), lens_order.end());
    auto lenses = lens_calibration(meta, out_width, out_height, summary);

    fs::path output_dir(options.output_dir);
    if (fs::exists(output_dir) && !fs::is_empty(output_dir) && !options.force) {
        throw std::runtime_error(output_dir.string() +
                                  " already exists and is not empty; pass --force to write into it");
    }
    fs::create_directories(output_dir);

    std::vector<Camera> cameras;
    for (size_t slot = 0; slot < std::min(lens_order.size(), kLensNames.size()); ++slot) {
        int video_index = lens_order[slot];
        Camera camera;
        camera.name = kLensNames[slot];
        camera.video_index = video_index;
        camera.dir = output_dir / camera.name;
        camera.width = out_width;
        camera.height = out_height;
        camera.serial = meta.serial;
        camera.model = meta.model;
        camera.firmware = meta.firmware;
        auto it = lenses.find(video_index);
        if (it != lenses.end()) camera.calibration = it->second;
        fs::create_directories(camera.dir);
        cameras.push_back(camera);
    }

    if (options.include_camera_info) {
        for (const auto& camera : cameras) {
            if (!camera.calibration) continue;
            write_camera_info(camera.dir / "camera_info.yaml", camera);
        }
    }

    if (options.include_imu && !imu_samples.empty()) {
        write_imu_csv(output_dir / "imu.csv", imu_samples, time_of);
        summary.counts["imu"] = static_cast<int64_t>(imu_samples.size());
    }

    // -- video ---------------------------------------------------------------------
    int64_t first_ns = time_of(origin_us);
    int64_t last_ns = first_ns;
    if (!cameras.empty()) {
        std::vector<std::unique_ptr<FramePipe>> pipes;
        for (const auto& camera : cameras) {
            pipes.push_back(std::make_unique<FramePipe>(options.input_path, camera.video_index,
                                                          options.jpeg_quality, scale_filter,
                                                          frame_count));
        }
        int64_t total = static_cast<int64_t>(frames.device_us.size());
        int64_t step = std::max<int64_t>(1, total / 10);
        // Progress counts frames read off the pipes, which with --frame-step is more than
        // the number written.
        auto report_progress = [&](int64_t done) {
            if (done % step != 0 && done != total) return;
            std::ostringstream progress;
            progress << "  video " << done << "/" << total << " frames ("
                     << (100.0 * done / total) << "%)";
            log(progress.str());
        };
        for (int64_t index = 0; index < total; ++index) {
            std::vector<std::optional<std::vector<uint8_t>>> frame_data;
            bool any_missing = false;
            for (auto& pipe : pipes) {
                auto frame = pipe->next();
                if (!frame) any_missing = true;
                frame_data.push_back(std::move(frame));
            }
            if (any_missing) {
                std::string missing_names;
                for (size_t i = 0; i < cameras.size(); ++i) {
                    if (!frame_data[i]) {
                        if (!missing_names.empty()) missing_names += ", ";
                        missing_names += cameras[i].name;
                    }
                }
                summary.warnings.push_back("video ended after " + std::to_string(index) + " of " +
                                            std::to_string(total) + " frames (no more data from " +
                                            missing_names + ")");
                break;
            }
            int64_t log_time = time_of(frames.device_us[index]);
            // Every frame still has to be pulled off both pipes to keep the two lenses in
            // lockstep; --frame-step only decides which of them reach the disk.
            if (index % options.frame_step != 0) {
                last_ns = std::max(last_ns, log_time);
                report_progress(index + 1);
                continue;
            }
            for (size_t i = 0; i < cameras.size(); ++i) {
                // camera.name is "cam_front"/"cam_back"; the exported file itself is
                // prefixed with just "front_"/"back_" so it reads well on its own once
                // pulled out of the per-camera directory.
                std::string prefix = cameras[i].name.substr(cameras[i].name.find('_') + 1) + "_";
                fs::path filename = cameras[i].dir / (prefix + std::to_string(log_time) + ".jpg");
                std::ofstream out(filename, std::ios::binary);
                out.write(reinterpret_cast<const char*>(frame_data[i]->data()),
                          static_cast<std::streamsize>(frame_data[i]->size()));
                if (!out) throw std::runtime_error("could not write " + filename.string());
                summary.counts[cameras[i].name]++;
            }
            last_ns = std::max(last_ns, log_time);
            report_progress(index + 1);
        }
    }

    if (!imu_samples.empty()) {
        last_ns = std::max(last_ns, time_of(imu_samples.back().device_us));
        first_ns = std::min(first_ns, time_of(imu_samples.front().device_us));
    }
    if (!frames.device_us.empty()) {
        first_ns = std::min(first_ns, time_of(frames.device_us.front()));
    }

    summary.start_ns = first_ns;
    summary.end_ns = last_ns;
    return summary;
}

}  // namespace insta360
