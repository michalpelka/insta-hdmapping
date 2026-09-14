// Typed view over the Insta360 protobuf metadata record (trailer record 0x0101).
//
// Field numbers were recovered by decoding the wire format of real captures; the
// camera ships no schema, so only the fields below are interpreted and everything
// else is left alone. Anything absent comes back empty and the caller degrades
// gracefully.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "insta360/calibration.hpp"

namespace insta360 {

constexpr double DEFAULT_ACCEL_RANGE_G = 32.0;
constexpr double DEFAULT_GYRO_RANGE_DPS = 2000.0;

// Maps the camera's monotonic microsecond clock onto wall-clock epoch time.
//
// Every timestamp in the trailer (IMU samples, exposure entries) is in microseconds
// since the camera booted. The metadata pins one point of that clock -- the first
// encoded video frame -- to a UTC epoch value, which is what lets the output carry
// real timestamps rather than an arbitrary origin.
struct Clock {
    int64_t first_frame_us = 0;
    int64_t first_frame_epoch_ns = 0;

    int64_t to_epoch_ns(int64_t device_us) const {
        return first_frame_epoch_ns + (device_us - first_frame_us) * 1000;
    }
};

struct Metadata {
    std::optional<std::string> serial;
    std::optional<std::string> model;
    std::optional<std::string> firmware;
    std::optional<std::string> profile;
    std::optional<std::string> source_path;
    std::optional<std::string> capture_datetime;
    std::optional<int> width;
    std::optional<int> height;
    std::optional<double> fps;
    std::optional<int64_t> frame_count;
    std::optional<double> duration_s;
    std::optional<int64_t> first_frame_us;
    std::optional<int64_t> last_frame_us;
    std::optional<int64_t> start_epoch_ms;
    std::optional<int64_t> end_epoch_ms;
    double accel_range_g = DEFAULT_ACCEL_RANGE_G;
    double gyro_range_dps = DEFAULT_GYRO_RANGE_DPS;
    bool sensor_ranges_known = false;
    std::vector<std::pair<std::string, std::string>> calibration_strings;  // name -> raw text
    std::vector<LensCalibration> lenses;

    std::optional<std::string> calibration_string(const std::string& name) const {
        for (const auto& [key, value] : calibration_strings) {
            if (key == name) return value;
        }
        return std::nullopt;
    }

    std::optional<Clock> clock() const {
        if (!first_frame_us || !start_epoch_ms) return std::nullopt;
        Clock clock;
        clock.first_frame_us = *first_frame_us;
        clock.first_frame_epoch_ns = *start_epoch_ms * 1'000'000;
        return clock;
    }

    std::optional<double> frame_interval_us() const {
        if (fps && *fps > 0) return 1'000'000.0 / *fps;
        return std::nullopt;
    }
};

// Decodes trailer record 0x0101 into a Metadata.
Metadata parse_metadata(const std::vector<uint8_t>& raw);

// Renders the packed YYYYMMDDhhmmss integer as an ISO-8601-ish local timestamp.
std::optional<std::string> format_capture_datetime(std::optional<int64_t> value);

}  // namespace insta360
