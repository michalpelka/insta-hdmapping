// Estimates the clock offset between the camera and an external IMU (e.g. a lidar
// scanner's) by cross-correlating gyroscope magnitudes.
//
// The magnitude |omega| is independent of how either sensor is mounted, so no
// extrinsic rotation is needed: both rigidly attached sensors see the same angular
// speed at the same physical instant, and the lag that best lines the two curves up
// is the offset between their clocks.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace insta360 {

// A gyroscope magnitude time series, sorted by time.
struct GyroSeries {
    std::vector<int64_t> t_ns;
    std::vector<double> magnitude_radps;
    std::string source;  // the files the samples came from, for logging
};

// Reads one IMU's logs from `dir` (sorted by name, then merged by timestamp) into one
// series. The first of these that holds samples is used, never two mixed:
//   1. dir/imu*.csv -- a Mandeye session's lidar IMU, or this tool's own imu.csv;
//   2. dir/MURATA_IMU/murata_imu*.csv -- a Mandeye logger's separate Murata IMU (the
//      only gyro in a Hesai-based session, whose imu*.csv files are header-only);
//   3. dir/murata_imu*.csv -- `dir` is that MURATA_IMU folder itself.
// The header picks the columns, so both a Mandeye-style space-separated log
// (`timestamp gyroX gyroY gyroZ ...`, nanoseconds, rad/s) and this tool's own imu.csv
// (`timestamp_ns,...,gyro_x_radps,...`) are accepted. Throws std::runtime_error when
// none of them holds samples.
GyroSeries read_gyro_csv_dir(const std::string& dir);

struct TimeAlignment {
    double offset_s = 0.0;     // add to camera timestamps to land on the reference clock
    double correlation = 0.0;  // Pearson r of the magnitudes at that offset
    double overlap_s = 0.0;    // length of the stretch both series cover at that offset
    bool at_search_edge = false;  // the best lag sits at +/-max_lag_s: likely out of range
};

// Finds the offset, within +/-max_lag_s, that maximises the correlation between the
// camera's gyro magnitude (shifted by the offset) and the reference's. A coarse
// 10 ms search is refined on a 1 ms grid and then to sub-millisecond by fitting a
// parabola through the peak. Only lags where the two series overlap for at least
// half of the shorter one are considered. Throws std::runtime_error if the series
// are too short or never overlap enough.
TimeAlignment align_gyro_magnitudes(const GyroSeries& reference, const GyroSeries& camera,
                                    double max_lag_s);

}  // namespace insta360
