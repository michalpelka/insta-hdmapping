#include "insta360/sensors.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace insta360 {

namespace {

uint64_t read_u64le(const uint8_t* p) {
    uint64_t v;
    std::memcpy(&v, p, sizeof(v));
    return v;  // little-endian host assumed (x86_64 / arm64)
}

uint16_t read_u16le(const uint8_t* p) {
    uint16_t v;
    std::memcpy(&v, p, sizeof(v));
    return v;
}

double read_f64le(const uint8_t* p) {
    double v;
    std::memcpy(&v, p, sizeof(v));
    return v;
}

uint32_t read_u32le(const uint8_t* p) {
    uint32_t v;
    std::memcpy(&v, p, sizeof(v));
    return v;
}

}  // namespace

std::vector<ImuSample> read_imu(const std::vector<uint8_t>& raw, double accel_range_g,
                                 double gyro_range_dps) {
    double accel_scale = (accel_range_g * STANDARD_GRAVITY) / IMU_ZERO;
    double gyro_scale = (gyro_range_dps * M_PI / 180.0) / IMU_ZERO;

    size_t count = raw.size() / IMU_STRIDE;
    std::vector<ImuSample> samples;
    samples.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        const uint8_t* p = raw.data() + i * IMU_STRIDE;
        int64_t device_us = static_cast<int64_t>(read_u64le(p));
        int ax = read_u16le(p + 8), ay = read_u16le(p + 10), az = read_u16le(p + 12);
        int gx = read_u16le(p + 14), gy = read_u16le(p + 16), gz = read_u16le(p + 18);
        samples.push_back(ImuSample{
            device_us,
            {(ax - IMU_ZERO) * accel_scale, (ay - IMU_ZERO) * accel_scale, (az - IMU_ZERO) * accel_scale},
            {(gx - IMU_ZERO) * gyro_scale, (gy - IMU_ZERO) * gyro_scale, (gz - IMU_ZERO) * gyro_scale},
        });
    }
    return samples;
}

std::vector<ExposureSample> read_exposure(const std::vector<uint8_t>& raw) {
    size_t count = raw.size() / EXPOSURE_STRIDE;
    std::vector<ExposureSample> samples;
    samples.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        const uint8_t* p = raw.data() + i * EXPOSURE_STRIDE;
        samples.push_back(ExposureSample{static_cast<int64_t>(read_u64le(p)), read_f64le(p + 8)});
    }
    return samples;
}

PreviewImage read_preview(const std::vector<uint8_t>& raw) {
    if (raw.size() < PREVIEW_HEADER_LEN) {
        throw std::runtime_error("preview record is too short to hold a header");
    }
    int width = static_cast<int>(read_u32le(raw.data() + 4 * 4));
    int height = static_cast<int>(read_u32le(raw.data() + 4 * 5));
    size_t expected = static_cast<size_t>(width) * height * 3 / 2;
    size_t payload_len = raw.size() - PREVIEW_HEADER_LEN;
    if (width <= 0 || height <= 0 || payload_len < expected) {
        throw std::runtime_error("preview record claims " + std::to_string(width) + "x" +
                                  std::to_string(height) + " NV12 (" + std::to_string(expected) +
                                  " bytes) but carries " + std::to_string(payload_len));
    }
    PreviewImage preview;
    preview.width = width;
    preview.height = height;
    preview.nv12.assign(raw.begin() + PREVIEW_HEADER_LEN, raw.begin() + PREVIEW_HEADER_LEN + expected);
    return preview;
}

FrameTimestamps frame_timestamps(const std::vector<ExposureSample>& exposures,
                                  std::optional<int64_t> first_frame_us, int64_t frame_count,
                                  std::optional<double> frame_interval_us) {
    if (first_frame_us && !exposures.empty()) {
        std::vector<const ExposureSample*> aligned;
        for (const auto& sample : exposures) {
            if (sample.device_us >= *first_frame_us) aligned.push_back(&sample);
        }
        if (static_cast<int64_t>(aligned.size()) >= frame_count) {
            FrameTimestamps result;
            result.device_us.reserve(frame_count);
            result.exposure_s.reserve(frame_count);
            for (int64_t i = 0; i < frame_count; ++i) {
                result.device_us.push_back(aligned[i]->device_us);
                result.exposure_s.push_back(aligned[i]->exposure_s);
            }
            return result;
        }
    }

    if (!first_frame_us || !frame_interval_us || *frame_interval_us <= 0) {
        throw std::runtime_error(
            "cannot establish frame timestamps: no exposure record and no frame rate "
            "in the metadata");
    }
    FrameTimestamps result;
    result.device_us.reserve(frame_count);
    result.exposure_s.assign(frame_count, std::nullopt);
    for (int64_t i = 0; i < frame_count; ++i) {
        result.device_us.push_back(*first_frame_us +
                                    static_cast<int64_t>(std::llround(i * (*frame_interval_us))));
    }
    return result;
}

}  // namespace insta360
