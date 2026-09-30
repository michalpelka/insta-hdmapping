#include "insta360/imu_align.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace insta360 {

namespace {

// Splits on commas and/or whitespace, dropping empty fields.
std::vector<std::string> split_fields(const std::string& line) {
    std::vector<std::string> fields;
    std::string current;
    for (char c : line) {
        if (c == ',' || c == ' ' || c == '\t' || c == '\r') {
            if (!current.empty()) fields.push_back(current);
            current.clear();
        } else {
            current += c;
        }
    }
    if (!current.empty()) fields.push_back(current);
    return fields;
}

int find_column(const std::vector<std::string>& header, std::initializer_list<const char*> names) {
    for (const char* name : names) {
        auto it = std::find(header.begin(), header.end(), name);
        if (it != header.end()) return static_cast<int>(it - header.begin());
    }
    return -1;
}

void append_csv(const fs::path& path, std::vector<std::pair<int64_t, double>>& samples) {
    std::ifstream in(path);
    std::string line;
    if (!std::getline(in, line)) return;
    auto header = split_fields(line);
    int t_col = find_column(header, {"timestamp", "timestamp_ns"});
    int x_col = find_column(header, {"gyroX", "gyro_x_radps"});
    int y_col = find_column(header, {"gyroY", "gyro_y_radps"});
    int z_col = find_column(header, {"gyroZ", "gyro_z_radps"});
    if (t_col < 0 || x_col < 0 || y_col < 0 || z_col < 0) {
        throw std::runtime_error(path.string() +
                                  ": header has no timestamp/gyro x/y/z columns: " + line);
    }
    int needed = std::max({t_col, x_col, y_col, z_col});
    while (std::getline(in, line)) {
        auto fields = split_fields(line);
        if (static_cast<int>(fields.size()) <= needed) continue;
        try {
            int64_t t = std::stoll(fields[t_col]);
            double x = std::stod(fields[x_col]);
            double y = std::stod(fields[y_col]);
            double z = std::stod(fields[z_col]);
            samples.emplace_back(t, std::sqrt(x * x + y * y + z * z));
        } catch (const std::exception&) {
            // A truncated last line (logger stopped mid-write) is expected; skip it.
        }
    }
}

// A series resampled onto the absolute grid t = k * dt_ns; NaN marks grid points
// that fall into a gap of the source data.
struct Grid {
    int64_t k0 = 0;
    std::vector<double> values;
};

Grid resample(const GyroSeries& series, int64_t dt_ns) {
    // Don't interpolate across dropouts: anything longer than this is a hole.
    const int64_t max_gap_ns = 50'000'000;
    const auto& t = series.t_ns;
    const auto& v = series.magnitude_radps;
    Grid grid;
    int64_t k_first = (t.front() + dt_ns - 1) / dt_ns;
    int64_t k_last = t.back() / dt_ns;
    if (k_last < k_first) return grid;
    grid.k0 = k_first;
    grid.values.assign(static_cast<size_t>(k_last - k_first + 1),
                        std::numeric_limits<double>::quiet_NaN());
    size_t j = 0;
    for (int64_t k = k_first; k <= k_last; ++k) {
        int64_t tk = k * dt_ns;
        while (j + 1 < t.size() && t[j + 1] < tk) ++j;
        if (j + 1 >= t.size()) {
            if (t[j] == tk) grid.values[k - k_first] = v[j];
            continue;
        }
        int64_t span = t[j + 1] - t[j];
        if (span <= 0 || span > max_gap_ns) continue;
        double a = static_cast<double>(tk - t[j]) / static_cast<double>(span);
        grid.values[k - k_first] = v[j] + a * (v[j + 1] - v[j]);
    }
    return grid;
}

size_t valid_count(const Grid& grid) {
    return static_cast<size_t>(std::count_if(grid.values.begin(), grid.values.end(),
                                              [](double x) { return std::isfinite(x); }));
}

struct Score {
    double r = -2.0;
    size_t n = 0;
};

// Pearson r between reference(k) and camera(k + lag) over the grid points both cover.
Score correlate(const Grid& ref, const Grid& cam, int64_t lag) {
    int64_t k_begin = std::max(ref.k0, cam.k0 - lag);
    int64_t k_end = std::min(ref.k0 + static_cast<int64_t>(ref.values.size()),
                             cam.k0 + static_cast<int64_t>(cam.values.size()) - lag);
    double sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
    size_t n = 0;
    for (int64_t k = k_begin; k < k_end; ++k) {
        double a = ref.values[k - ref.k0];
        double b = cam.values[k + lag - cam.k0];
        if (!std::isfinite(a) || !std::isfinite(b)) continue;
        sa += a;
        sb += b;
        saa += a * a;
        sbb += b * b;
        sab += a * b;
        ++n;
    }
    Score score;
    score.n = n;
    if (n < 2) return score;
    double cov = sab - sa * sb / n;
    double va = saa - sa * sa / n;
    double vb = sbb - sb * sb / n;
    if (va <= 0 || vb <= 0) return score;
    score.r = cov / std::sqrt(va * vb);
    return score;
}

struct Peak {
    int64_t lag = 0;
    Score score;
};

Peak search(const Grid& ref, const Grid& cam, int64_t lag_min, int64_t lag_max, size_t min_n) {
    Peak best;
    for (int64_t lag = lag_min; lag <= lag_max; ++lag) {
        Score s = correlate(ref, cam, lag);
        if (s.n >= min_n && s.r > best.score.r) {
            best.lag = lag;
            best.score = s;
        }
    }
    return best;
}

}  // namespace

GyroSeries read_gyro_csv_dir(const std::string& dir) {
    if (!fs::is_directory(dir)) throw std::runtime_error(dir + " is not a directory");
    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(dir)) {
        std::string name = entry.path().filename().string();
        if (entry.is_regular_file() && name.rfind("imu", 0) == 0 && entry.path().extension() == ".csv") {
            files.push_back(entry.path());
        }
    }
    if (files.empty()) throw std::runtime_error("no imu*.csv files in " + dir);
    std::sort(files.begin(), files.end());

    std::vector<std::pair<int64_t, double>> samples;
    for (const auto& file : files) append_csv(file, samples);
    std::sort(samples.begin(), samples.end());
    samples.erase(std::unique(samples.begin(), samples.end(),
                              [](const auto& a, const auto& b) { return a.first == b.first; }),
                  samples.end());
    if (samples.size() < 2) throw std::runtime_error("no gyro samples in the imu*.csv files in " + dir);

    GyroSeries series;
    for (const auto& [t, m] : samples) {
        series.t_ns.push_back(t);
        series.magnitude_radps.push_back(m);
    }
    return series;
}

TimeAlignment align_gyro_magnitudes(const GyroSeries& reference, const GyroSeries& camera,
                                    double max_lag_s) {
    if (reference.t_ns.size() < 2 || camera.t_ns.size() < 2) {
        throw std::runtime_error("IMU alignment needs at least two samples in each series");
    }

    // Coarse pass over the whole window.
    const int64_t coarse_ns = 10'000'000;
    Grid ref = resample(reference, coarse_ns);
    Grid cam = resample(camera, coarse_ns);
    size_t min_n = std::min(valid_count(ref), valid_count(cam)) / 2;
    if (min_n < 100) throw std::runtime_error("IMU series too short to align (need > 2 s each)");
    int64_t max_lag = static_cast<int64_t>(std::ceil(max_lag_s * 1e9 / coarse_ns));
    Peak coarse = search(ref, cam, -max_lag, max_lag, min_n);
    if (coarse.score.n == 0) {
        throw std::runtime_error(
            "camera and reference IMU never overlap by half their length within +/-" +
            std::to_string(max_lag_s) + " s; are both on the same wall clock?");
    }

    // Fine pass: 1 ms steps around the coarse peak.
    const int64_t fine_ns = 1'000'000;
    const int64_t ratio = coarse_ns / fine_ns;
    ref = resample(reference, fine_ns);
    cam = resample(camera, fine_ns);
    min_n = std::min(valid_count(ref), valid_count(cam)) / 2;
    Peak fine = search(ref, cam, coarse.lag * ratio - 2 * ratio, coarse.lag * ratio + 2 * ratio, min_n);
    if (fine.score.n == 0) fine = {coarse.lag * ratio, coarse.score};

    // Sub-millisecond: vertex of the parabola through the peak and its neighbours.
    double lag_steps = static_cast<double>(fine.lag);
    Score left = correlate(ref, cam, fine.lag - 1);
    Score right = correlate(ref, cam, fine.lag + 1);
    double denom = left.r - 2 * fine.score.r + right.r;
    if (left.n > 0 && right.n > 0 && denom < 0) {
        double shift = 0.5 * (left.r - right.r) / denom;
        if (std::abs(shift) <= 1.0) lag_steps += shift;
    }

    // camera(t + lag) matches reference(t), so the camera clock reads `lag` ahead and
    // its timestamps need `-lag` to land on the reference clock.
    TimeAlignment result;
    result.offset_s = -lag_steps * fine_ns / 1e9;
    result.correlation = fine.score.r;
    result.overlap_s = fine.score.n * fine_ns / 1e9;
    result.at_search_edge = std::abs(coarse.lag) >= max_lag;
    return result;
}

}  // namespace insta360
