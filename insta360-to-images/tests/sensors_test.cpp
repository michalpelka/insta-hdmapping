#include "insta360/sensors.hpp"

#include <cmath>
#include <cstring>

#include "test_util.hpp"

using namespace insta360;

namespace {

void push_u64le(std::vector<uint8_t>& buf, uint64_t v) {
    for (int i = 0; i < 8; ++i) buf.push_back((v >> (8 * i)) & 0xFF);
}
void push_u16le(std::vector<uint8_t>& buf, uint16_t v) {
    buf.push_back(v & 0xFF);
    buf.push_back((v >> 8) & 0xFF);
}
void push_f64le(std::vector<uint8_t>& buf, double v) {
    uint8_t bytes[8];
    std::memcpy(bytes, &v, 8);
    buf.insert(buf.end(), bytes, bytes + 8);
}

}  // namespace

TEST(read_imu_decodes_offset_binary_samples) {
    std::vector<uint8_t> raw;
    push_u64le(raw, 123456789);
    push_u16le(raw, 32768);          // ax = 0
    push_u16le(raw, 32768 + 16384);  // ay = +0.5 full scale
    push_u16le(raw, 32768 - 16384);  // az = -0.5 full scale
    push_u16le(raw, 32768);          // gx = 0
    push_u16le(raw, 32768);          // gy = 0
    push_u16le(raw, 32768 + 8192);   // gz = +0.25 full scale

    auto samples = read_imu(raw, /*accel_range_g=*/32.0, /*gyro_range_dps=*/2000.0);
    CHECK_EQ(samples.size(), 1u);
    const auto& s = samples[0];
    CHECK_EQ(s.device_us, 123456789);
    CHECK_NEAR(s.accel[0], 0.0, 1e-9);
    CHECK_NEAR(s.accel[1], 0.5 * 32.0 * STANDARD_GRAVITY, 1e-6);
    CHECK_NEAR(s.accel[2], -0.5 * 32.0 * STANDARD_GRAVITY, 1e-6);
    CHECK_NEAR(s.gyro[0], 0.0, 1e-9);
    CHECK_NEAR(s.gyro[2], 0.25 * 2000.0 * M_PI / 180.0, 1e-6);
}

TEST(read_exposure_decodes_timestamp_and_seconds) {
    std::vector<uint8_t> raw;
    push_u64le(raw, 1000);
    push_f64le(raw, 0.008);
    push_u64le(raw, 1042);
    push_f64le(raw, 0.0082);

    auto samples = read_exposure(raw);
    CHECK_EQ(samples.size(), 2u);
    CHECK_EQ(samples[0].device_us, 1000);
    CHECK_NEAR(samples[0].exposure_s, 0.008, 1e-12);
    CHECK_EQ(samples[1].device_us, 1042);
}

TEST(frame_timestamps_prefers_exposure_record_when_it_covers_the_clip) {
    std::vector<ExposureSample> exposures = {
        {100, 0.010}, {141, 0.011}, {183, 0.009}, {225, 0.010},
    };
    auto result = frame_timestamps(exposures, /*first_frame_us=*/141, /*frame_count=*/2,
                                    /*frame_interval_us=*/std::nullopt);
    CHECK_EQ(result.device_us.size(), 2u);
    CHECK_EQ(result.device_us[0], 141);
    CHECK_EQ(result.device_us[1], 183);
    CHECK(result.exposure_s[0].has_value());
    CHECK_NEAR(*result.exposure_s[0], 0.011, 1e-12);
}

TEST(frame_timestamps_falls_back_to_nominal_rate) {
    std::vector<ExposureSample> exposures;  // none at all
    auto result = frame_timestamps(exposures, /*first_frame_us=*/1000, /*frame_count=*/3,
                                    /*frame_interval_us=*/1'000'000.0 / 24.0);
    CHECK_EQ(result.device_us.size(), 3u);
    CHECK_EQ(result.device_us[0], 1000);
    CHECK(!result.exposure_s[0].has_value());
    // Uniform spacing at the nominal interval.
    int64_t interval = result.device_us[1] - result.device_us[0];
    CHECK_NEAR(static_cast<double>(interval), 1'000'000.0 / 24.0, 1.0);
}

TEST(frame_timestamps_throws_with_no_usable_timing_source) {
    std::vector<ExposureSample> exposures;
    CHECK_THROWS(frame_timestamps(exposures, std::nullopt, 3, std::nullopt));
}

TEST(read_preview_decodes_header_and_trims_payload_to_nv12_size) {
    const int width = 4, height = 2;
    std::vector<uint8_t> raw(PREVIEW_HEADER_LEN, 0);
    // Header is 10 little-endian uint32s; words 4 and 5 are width/height.
    std::memcpy(raw.data() + 4 * 4, &width, 4);
    std::memcpy(raw.data() + 4 * 5, &height, 4);
    size_t nv12_size = static_cast<size_t>(width) * height * 3 / 2;  // 12 bytes
    std::vector<uint8_t> payload(nv12_size + 5, 0xAB);               // 5 bytes of trailing junk
    for (size_t i = 0; i < nv12_size; ++i) payload[i] = static_cast<uint8_t>(i);
    raw.insert(raw.end(), payload.begin(), payload.end());

    PreviewImage preview = read_preview(raw);
    CHECK_EQ(preview.width, width);
    CHECK_EQ(preview.height, height);
    CHECK_EQ(preview.nv12.size(), nv12_size);
    CHECK_EQ(static_cast<int>(preview.nv12[0]), 0);
    CHECK_EQ(static_cast<int>(preview.nv12[nv12_size - 1]), static_cast<int>(nv12_size - 1));
}

TEST(read_preview_rejects_short_or_truncated_records) {
    CHECK_THROWS(read_preview(std::vector<uint8_t>(10, 0)));  // shorter than the header

    std::vector<uint8_t> raw(PREVIEW_HEADER_LEN, 0);
    int width = 100, height = 100;
    std::memcpy(raw.data() + 4 * 4, &width, 4);
    std::memcpy(raw.data() + 4 * 5, &height, 4);
    // Claims 100x100 NV12 (15000 bytes) but carries none.
    CHECK_THROWS(read_preview(raw));
}
