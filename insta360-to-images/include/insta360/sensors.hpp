// Decoders for the fixed-layout sensor records in the Insta360 trailer.
//
// Layouts here were recovered by inspection and then validated physically rather than
// just structurally -- see the notes on read_imu().
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace insta360 {

// Standard gravity, used to convert the accelerometer from g to m/s^2.
constexpr double STANDARD_GRAVITY = 9.80665;

constexpr size_t IMU_STRIDE = 20;       // uint64 timestamp + 6 x uint16 channels
constexpr size_t EXPOSURE_STRIDE = 16;  // uint64 timestamp + float64 seconds
constexpr size_t PREVIEW_HEADER_LEN = 40;

// The six IMU channels are offset-binary, i.e. a reading of 0 sits at 0x8000 rather
// than being two's complement. Interpreting them as int16 makes four of the six
// channels wrap through +/-32768, which is how this was caught.
constexpr int IMU_ZERO = 32768;

// One IMU sample in SI units: m/s^2 and rad/s, in the raw sensor frame.
struct ImuSample {
    int64_t device_us;
    std::array<double, 3> accel;
    std::array<double, 3> gyro;
};

struct ExposureSample {
    int64_t device_us;
    double exposure_s;
};

// The camera's own equirectangular preview thumbnail, NV12-encoded (record 0x0002):
// firmware-stitched, not something this tool computes.
struct PreviewImage {
    int width;
    int height;
    std::vector<uint8_t> nv12;
};

// Decodes trailer record 0x0002: a 40-byte header then an NV12 equirect thumbnail.
//
// Header is a packed array of 10 little-endian uint32s; words 4 and 5 are the width
// and height. The pixel format is NV12 (a full-size luma plane followed by
// interleaved Cb/Cr at half resolution) -- confirmed by rendering, since NV21 comes
// out with the chroma channels swapped and I420 comes out desaturated. Throws
// std::runtime_error if the record is too short or the claimed dimensions don't fit
// the payload.
PreviewImage read_preview(const std::vector<uint8_t>& raw);

// Decodes trailer record 0x0003 into calibrated IMU samples.
//
// Layout is a packed array of <Q6H>: a microsecond device timestamp followed by
// accelerometer x/y/z then gyroscope x/y/z, each an offset-binary 16-bit count over
// the full-scale range reported in the metadata (typically +/-32 g and +/-2000 dps).
//
// That assignment is not guesswork: with these scales the accelerometer magnitude
// holds at 1.008 g across a whole handheld clip, and integrating the gyroscope over
// the same clip yields 1863 deg of total rotation against 1828 deg of gravity-vector
// travel measured independently from the accelerometer. Swapping the two triples, or
// reading the counts as two's complement, breaks both checks.
//
// The axes are the sensor's own and are *not* rotated into REP 103 (x forward, y
// left, z up); the mapping from the X-series sensor frame to the camera body frame is
// undocumented, so the raw frame is preserved rather than being silently
// reinterpreted.
std::vector<ImuSample> read_imu(const std::vector<uint8_t>& raw, double accel_range_g,
                                 double gyro_range_dps);

// Decodes trailer record 0x0004: <Qd> of device timestamp and exposure seconds.
//
// The entries are emitted one per captured frame and start a little before the first
// *encoded* frame, so the caller aligns them by timestamp rather than by index.
std::vector<ExposureSample> read_exposure(const std::vector<uint8_t>& raw);

// Works out a device timestamp for every encoded video frame.
//
// Prefers the exposure record, whose entries are stamped with the sensor's real
// (slightly non-nominal) frame interval and whose first in-range entry lands exactly
// on the metadata's first-frame timestamp. Falls back to a uniform nominal interval
// when that record is missing or does not cover the clip. Throws std::runtime_error
// when neither source is usable.
struct FrameTimestamps {
    std::vector<int64_t> device_us;
    std::vector<std::optional<double>> exposure_s;
};

FrameTimestamps frame_timestamps(const std::vector<ExposureSample>& exposures,
                                  std::optional<int64_t> first_frame_us, int64_t frame_count,
                                  std::optional<double> frame_interval_us);

}  // namespace insta360
