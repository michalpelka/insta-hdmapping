#include "insta360/imu_align.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>
#include <string_view>

#include "test_util.hpp"

using namespace insta360;
namespace fs = std::filesystem;

namespace {

// A handheld-looking angular speed: a few smooth rotation bursts of varying size.
double angular_speed(double t) {
    double w = 0.05;
    const double centers[] = {3.0, 7.5, 11.0, 16.0, 21.5, 26.0};
    const double heights[] = {1.5, 0.8, 2.6, 1.1, 3.0, 0.6};
    for (int i = 0; i < 6; ++i) {
        double d = (t - centers[i]) / 0.6;
        w += heights[i] * std::exp(-d * d);
    }
    return w;
}

// Samples angular_speed on the true timeline, stamped by a clock that reads
// `clock_ahead_s` ahead of the truth, with a little sensor noise.
GyroSeries sample(double start_s, double end_s, double rate_hz, double clock_ahead_s,
                  unsigned seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, 0.01);
    GyroSeries series;
    const int64_t epoch_ns = 1'789'125'000'000'000'000;
    for (double t = start_s; t < end_s; t += 1.0 / rate_hz) {
        series.t_ns.push_back(epoch_ns + static_cast<int64_t>(std::llround((t + clock_ahead_s) * 1e9)));
        series.magnitude_radps.push_back(angular_speed(t) + noise(rng));
    }
    return series;
}

}  // namespace

TEST(align_gyro_magnitudes_recovers_a_known_clock_offset) {
    // Reference at 200 Hz on the true clock; camera at 1 kHz, clock 0.5237 s ahead
    // and covering a longer stretch, like the Insta360 against a lidar logger.
    GyroSeries reference = sample(1.0, 28.0, 200.0, 0.0, 1);
    GyroSeries camera = sample(0.0, 30.0, 1000.0, 0.5237, 2);
    TimeAlignment result = align_gyro_magnitudes(reference, camera, 10.0);
    CHECK_NEAR(result.offset_s, -0.5237, 0.001);
    CHECK(result.correlation > 0.99);
    CHECK(!result.at_search_edge);
}

TEST(align_gyro_magnitudes_handles_a_camera_clock_behind) {
    GyroSeries reference = sample(1.0, 28.0, 200.0, 0.0, 3);
    GyroSeries camera = sample(0.0, 30.0, 1000.0, -2.25, 4);
    CHECK_NEAR(align_gyro_magnitudes(reference, camera, 10.0).offset_s, 2.25, 0.001);
}

TEST(align_gyro_magnitudes_rejects_series_that_never_overlap) {
    GyroSeries reference = sample(0.0, 10.0, 200.0, 0.0, 5);
    GyroSeries camera = sample(0.0, 10.0, 1000.0, 100.0, 6);
    CHECK_THROWS(align_gyro_magnitudes(reference, camera, 5.0));
}

TEST(read_gyro_csv_dir_merges_mandeye_and_own_formats) {
    fs::path dir = fs::temp_directory_path() / "insta360_imu_align_test";
    fs::remove_all(dir);
    fs::create_directories(dir);
    {
        std::ofstream a(dir / "imu0001.csv");
        a << "timestamp gyroX gyroY gyroZ accX accY accZ imuId timestampUnix\n"
          << "3000 0 3 4 0 0 1 0 3\n"
          << "4000 1 0 0 0 0 1 0 4\n"
          << "5000 0.5";  // truncated by the logger stopping mid-line
        std::ofstream b(dir / "imu0000.csv");
        b << "timestamp_ns,accel_x_mps2,accel_y_mps2,accel_z_mps2,gyro_x_radps,gyro_y_radps,"
             "gyro_z_radps\n"
          << "1000,0,0,9.8,0,0,2\n"
          << "3000,0,0,9.8,0,3,4\n";  // duplicate of imu0001's first row
        std::ofstream c(dir / "status0000.json");
        c << "{}";
    }
    GyroSeries series = read_gyro_csv_dir(dir.string());
    CHECK_EQ(series.t_ns.size(), 3u);
    CHECK_EQ(series.t_ns[0], 1000);
    CHECK_NEAR(series.magnitude_radps[0], 2.0, 1e-12);
    CHECK_EQ(series.t_ns[1], 3000);
    CHECK_NEAR(series.magnitude_radps[1], 5.0, 1e-12);
    CHECK_EQ(series.t_ns[2], 4000);
    fs::remove_all(dir);
}

namespace {

constexpr std::string_view kMandeyeHeader =
    "timestamp gyroX gyroY gyroZ accX accY accZ imuId timestampUnix\n";
constexpr std::string_view kMurataHeader =
    "timestamp gyroX gyroY gyroZ accX accY accZ timestampUnix\n";

}  // namespace

TEST(read_gyro_csv_dir_falls_back_to_the_murata_imu_of_a_hesai_session) {
    // A Hesai-based Mandeye session: imu*.csv are header-only, the gyro lives in
    // MURATA_IMU/murata_imu_<ns>.csv. Both the session folder and MURATA_IMU itself work.
    fs::path dir = fs::temp_directory_path() / "insta360_imu_align_murata_test";
    fs::remove_all(dir);
    fs::create_directories(dir / "MURATA_IMU");
    {
        std::ofstream(dir / "imu0000.csv") << kMandeyeHeader;
        std::ofstream(dir / "imu0001.csv") << kMandeyeHeader;
        std::ofstream(dir / "MURATA_IMU" / "murata_imu_4000.csv")
            << kMurataHeader << "3000 0 3 4 0 0 1 3000\n"
            << "4000 2 0 0 0 0 1 4000\n";
        std::ofstream(dir / "MURATA_IMU" / "murata_imu_2000.csv")
            << kMurataHeader << "1000 0 0 1 0 0 1 1000\n";
    }
    for (const fs::path& target : {dir, dir / "MURATA_IMU"}) {
        GyroSeries series = read_gyro_csv_dir(target.string());
        CHECK_EQ(series.t_ns.size(), 3u);
        CHECK_EQ(series.t_ns[0], 1000);
        CHECK_NEAR(series.magnitude_radps[1], 5.0, 1e-12);
        CHECK_EQ(series.t_ns[2], 4000);
        CHECK(series.source.find("murata_imu*.csv (2 files)") != std::string::npos);
    }
    fs::remove_all(dir);
}

TEST(read_gyro_csv_dir_prefers_the_lidar_imu_and_never_mixes_sensors) {
    fs::path dir = fs::temp_directory_path() / "insta360_imu_align_both_test";
    fs::remove_all(dir);
    fs::create_directories(dir / "MURATA_IMU");
    {
        std::ofstream(dir / "imu0000.csv") << kMandeyeHeader << "1000 0 0 1 0 0 1 0 1\n"
                                           << "3000 0 0 1 0 0 1 0 3\n";
        std::ofstream(dir / "MURATA_IMU" / "murata_imu_2000.csv")
            << kMurataHeader << "2000 0 0 7 0 0 1 2000\n";
    }
    GyroSeries series = read_gyro_csv_dir(dir.string());
    CHECK_EQ(series.t_ns.size(), 2u);
    CHECK_EQ(series.t_ns[1], 3000);
    fs::remove_all(dir);
}

TEST(read_gyro_csv_dir_rejects_a_folder_without_imu_logs) {
    fs::path dir = fs::temp_directory_path() / "insta360_imu_align_empty_test";
    fs::remove_all(dir);
    fs::create_directories(dir);
    CHECK_THROWS(read_gyro_csv_dir(dir.string()));
    fs::remove_all(dir);
}
