#include "insta360/metadata.hpp"

#include "insta360/protobuf.hpp"

namespace insta360 {

namespace {

// -- metadata field numbers --------------------------------------------------
constexpr int F_SERIAL = 1;
constexpr int F_MODEL = 2;
constexpr int F_FIRMWARE = 3;
constexpr int F_CAPTURE_DATETIME = 7;   // YYYYMMDDhhmmss as a single integer, camera local time
constexpr int F_FRAME_SIZE = 19;        // sub-message {1: width, 2: height}
constexpr int F_FPS = 20;
constexpr int F_PROFILE = 22;
constexpr int F_FIRST_FRAME_US = 24;
constexpr int F_SOURCE = 26;            // sub-message, {3: original path on the SD card}
constexpr int F_OFFSET = 5;
constexpr int F_START_EPOCH_MS = 36;
constexpr int F_OFFSET_V2 = 54;
constexpr int F_SENSOR_RANGES = 65;     // sub-message {1: accel full scale g, 2: gyro full scale dps}
constexpr int F_DURATION_MS = 93;
constexpr int F_CLOCK = 98;
constexpr int F_OFFSET_V3 = 111;

// -- F_CLOCK sub-message ------------------------------------------------------
constexpr int C_FIRST_FRAME_US = 1;
constexpr int C_LAST_FRAME_US = 2;
constexpr int C_START_EPOCH_MS = 3;
constexpr int C_END_EPOCH_MS = 4;
constexpr int C_FRAME_COUNT = 5;

}  // namespace

std::optional<std::string> format_capture_datetime(std::optional<int64_t> value) {
    if (!value) return std::nullopt;
    std::string text = std::to_string(*value);
    if (text.size() != 14) return text;
    return text.substr(0, 4) + "-" + text.substr(4, 2) + "-" + text.substr(6, 2) + "T" +
           text.substr(8, 2) + ":" + text.substr(10, 2) + ":" + text.substr(12, 2);
}

Metadata parse_metadata(const std::vector<uint8_t>& raw) {
    Message message = Message::parse(raw);
    Metadata meta;

    meta.serial = message.text(F_SERIAL);
    meta.model = message.text(F_MODEL);
    meta.firmware = message.text(F_FIRMWARE);
    meta.profile = message.text(F_PROFILE);
    if (auto capture = message.uint(F_CAPTURE_DATETIME)) {
        meta.capture_datetime = format_capture_datetime(static_cast<int64_t>(*capture));
    }
    if (auto fps = message.uint(F_FPS)) {
        meta.fps = static_cast<double>(*fps);
    }
    if (auto first = message.uint(F_FIRST_FRAME_US)) {
        meta.first_frame_us = static_cast<int64_t>(*first);
    }
    if (auto start = message.uint(F_START_EPOCH_MS)) {
        meta.start_epoch_ms = static_cast<int64_t>(*start);
    }

    if (auto source = message.message(F_SOURCE)) {
        meta.source_path = source->text(3);
    }

    if (auto size = message.message(F_FRAME_SIZE)) {
        if (auto width = size->uint(1)) meta.width = static_cast<int>(*width);
        if (auto height = size->uint(2)) meta.height = static_cast<int>(*height);
    }

    if (auto duration_ms = message.uint(F_DURATION_MS)) {
        meta.duration_s = static_cast<double>(*duration_ms) / 1000.0;
    }

    // The clock sub-message is the most precise source for all four of these, so it
    // overrides the standalone fields above where present.
    if (auto clock = message.message(F_CLOCK)) {
        if (auto v = clock->uint(C_FIRST_FRAME_US)) meta.first_frame_us = static_cast<int64_t>(*v);
        if (auto v = clock->uint(C_LAST_FRAME_US)) meta.last_frame_us = static_cast<int64_t>(*v);
        if (auto v = clock->uint(C_START_EPOCH_MS)) meta.start_epoch_ms = static_cast<int64_t>(*v);
        if (auto v = clock->uint(C_END_EPOCH_MS)) meta.end_epoch_ms = static_cast<int64_t>(*v);
        if (auto v = clock->uint(C_FRAME_COUNT)) meta.frame_count = static_cast<int64_t>(*v);
    }

    if (auto ranges = message.message(F_SENSOR_RANGES)) {
        auto accel = ranges->uint(1);
        auto gyro = ranges->uint(2);
        if (accel && gyro && *accel && *gyro) {
            meta.accel_range_g = static_cast<double>(*accel);
            meta.gyro_range_dps = static_cast<double>(*gyro);
            meta.sensor_ranges_known = true;
        }
    }

    for (auto [name, number] : {std::pair{"offset", F_OFFSET}, std::pair{"offset_v2", F_OFFSET_V2},
                                 std::pair{"offset_v3", F_OFFSET_V3}}) {
        if (auto text = message.text(number)) {
            meta.calibration_strings.emplace_back(name, *text);
        }
    }

    if (auto v2 = meta.calibration_string("offset_v2"); v2 && meta.width && meta.height) {
        meta.lenses = parse_offset_v2(*v2, *meta.width, *meta.height);
    }

    return meta;
}

}  // namespace insta360
