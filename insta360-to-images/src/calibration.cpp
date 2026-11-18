#include "insta360/calibration.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <utility>

namespace insta360 {

namespace {

constexpr int kFieldsPerLensV2 = 19;
constexpr int kFieldsPerLensV3 = 27;
constexpr double kRefitCapDeg = 120.0;
constexpr int kRefitSamples = 1000;

std::vector<std::string> split(const std::string& text, char sep) {
    std::vector<std::string> parts;
    std::stringstream stream(text);
    std::string part;
    while (std::getline(stream, part, sep)) parts.push_back(part);
    // std::getline drops a trailing empty field after a final separator; the Python
    // str.split keeps it, but the parser below only ever looks at bounded prefixes so
    // that difference never matters here.
    return parts;
}

bool parse_double(const std::string& text, double& out) {
    try {
        size_t consumed = 0;
        out = std::stod(text, &consumed);
        return consumed == text.size();
    } catch (const std::exception&) {
        return false;
    }
}

bool parse_int(const std::string& text, int& out) {
    double value;
    if (!parse_double(text, value)) return false;
    out = static_cast<int>(value);
    return true;
}

// Splits "<n>_<group>_<group>..._<trailer>" into n groups of `per_lens` numbers.
// Only the first `per_lens - 1` values of each group are required to be numeric:
// the last (a crop/flags field) is never read. Empty on any mismatch.
std::vector<std::vector<double>> split_groups(const std::string& text, int per_lens) {
    auto tokens = split(text, '_');
    if (tokens.size() < 2) return {};
    int lens_count = 0;
    if (!parse_int(tokens[0], lens_count) || lens_count <= 0) return {};
    if (static_cast<int>(tokens.size()) - 1 < lens_count * per_lens) return {};

    std::vector<std::vector<double>> groups;
    for (int index = 0; index < lens_count; ++index) {
        std::vector<double> values(per_lens - 1);
        for (int i = 0; i < per_lens - 1; ++i) {
            if (!parse_double(tokens[1 + index * per_lens + i], values[i])) return {};
        }
        groups.push_back(std::move(values));
    }
    return groups;
}

// Fills in everything but the distortion from one lens group. Both versions open
// with xi, fx, fy, cx, cy, rx, ry, rz, tx, ty, tz, and both close with canvas_w,
// canvas_h (then the unread crop field), so `canvas_at` is all that differs.
bool fill_geometry(LensCalibration& lens, const std::vector<double>& g, int canvas_at, int index,
                   int width, int height) {
    int canvas_w = static_cast<int>(g[canvas_at]);
    int canvas_h = static_cast<int>(g[canvas_at + 1]);
    if (canvas_w <= 0 || canvas_h <= 0) return false;

    // Each lens occupies the left or right square half of the stitched canvas.
    double half_width = canvas_w / 2.0;
    // Scale each axis independently so a non-square output (from --scale) still
    // gets consistent intrinsics.
    double scale_x = width / half_width;
    double scale_y = height / static_cast<double>(canvas_h);

    lens.index = index;
    lens.width = width;
    lens.height = height;
    lens.xi = g[0];  // dimensionless sphere-model parameter; not a pixel quantity
    lens.fx = g[1] * scale_x;
    lens.fy = g[2] * scale_y;
    lens.cx = (g[3] - index * half_width) * scale_x;
    lens.cy = g[4] * scale_y;
    lens.rotation_deg = {g[5], g[6], g[7]};
    lens.translation = {g[8], g[9], g[10]};
    lens.canvas = {canvas_w, canvas_h};
    lens.scale = {scale_x, scale_y};
    return true;
}

void set_distortion(LensCalibration& lens, double k1, double k2, double k3, double p1, double p2) {
    lens.k1 = k1;
    lens.k2 = k2;
    lens.k3 = k3;
    lens.p1 = p1;
    lens.p2 = p2;
    lens.distortion = {k1, k2, k3, p1, p2};
}

// Solves the 3x3 system a x = b by Gaussian elimination with partial pivoting.
bool solve3(double a[3][3], double b[3], double x[3]) {
    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        for (int row = col + 1; row < 3; ++row) {
            if (std::abs(a[row][col]) > std::abs(a[pivot][col])) pivot = row;
        }
        if (std::abs(a[pivot][col]) < 1e-300) return false;
        std::swap(a[col], a[pivot]);
        std::swap(b[col], b[pivot]);
        for (int row = col + 1; row < 3; ++row) {
            double f = a[row][col] / a[col][col];
            for (int k = col; k < 3; ++k) a[row][k] -= f * a[col][k];
            b[row] -= f * b[col];
        }
    }
    for (int row = 2; row >= 0; --row) {
        double sum = b[row];
        for (int k = row + 1; k < 3; ++k) sum -= a[row][k] * x[k];
        x[row] = sum / a[row][row];
    }
    return true;
}

// Distorted radius on the Mei plane for undistorted radius r, radial terms only.
double radial(const double* k, int count, double r) {
    double r2 = r * r, power = r2, factor = 1.0;
    for (int i = 0; i < count; ++i, power *= r2) factor += k[i] * power;
    return r * factor;
}

// Refits k1-k3 to the four-term radial curve `k4` (k1..k4) for a lens with mirror
// offset `xi`, sampled uniformly in off-axis angle from 0 out to where the distorted
// radius, times `focal` pixels, reaches `radius_px` (the image circle), or
// kRefitCapDeg. Fills the refit and its worst pixel error; false if there is no
// usable range or the system is singular.
bool refit_radial(double xi, const double k4[4], double focal, double radius_px,
                  LensCalibration& lens) {
    // Find the angular extent of the image: the Mei mapping stays monotonic well
    // past 90 degrees for xi > 1, but stop early if the distortion folds back.
    double max_theta = 0;
    double previous = 0;
    for (double deg = 0.05; deg <= kRefitCapDeg + 1e-9; deg += 0.05) {
        double theta = deg * M_PI / 180.0;
        double denom = std::cos(theta) + xi;
        if (denom <= 1e-9) break;
        double rd = radial(k4, 4, std::sin(theta) / denom);
        if (rd <= previous || rd * focal > radius_px) break;
        previous = rd;
        max_theta = theta;
    }
    if (max_theta < 10.0 * M_PI / 180.0) return false;

    // Least squares on r^3, r^5, r^7 against the residual r * (k1 r^2 + .. + k4 r^8),
    // with r normalized by its largest value so the normal equations stay well
    // conditioned (r^7 is ~1e-3 of r^3 at the image edge).
    double r_max = std::sin(max_theta) / (std::cos(max_theta) + xi);
    double ata[3][3] = {}, atb[3] = {};
    std::vector<std::pair<double, double>> samples;  // (r, target)
    samples.reserve(kRefitSamples + 1);
    for (int i = 0; i <= kRefitSamples; ++i) {
        double theta = max_theta * i / kRefitSamples;
        double r = std::sin(theta) / (std::cos(theta) + xi);
        double target = radial(k4, 4, r) - r;
        samples.emplace_back(r, target);
        double u = r / r_max;
        double basis[3] = {u * u * u, std::pow(u, 5), std::pow(u, 7)};
        for (int a = 0; a < 3; ++a) {
            atb[a] += basis[a] * target;
            for (int b = 0; b < 3; ++b) ata[a][b] += basis[a] * basis[b];
        }
    }
    double c[3];
    if (!solve3(ata, atb, c)) return false;
    // target = k1 r^3 + k2 r^5 + k3 r^7 and u = r / r_max, so k_j = c_j / r_max^(power).
    double k3[3] = {c[0] / std::pow(r_max, 3), c[1] / std::pow(r_max, 5), c[2] / std::pow(r_max, 7)};

    double worst = 0;
    for (const auto& [r, target] : samples) {
        double fitted = radial(k3, 3, r) - r;
        worst = std::max(worst, std::abs(fitted - target) * focal);
    }
    lens.k1 = k3[0];
    lens.k2 = k3[1];
    lens.k3 = k3[2];
    lens.refit_max_error_px = worst;
    lens.refit_max_theta_deg = max_theta * 180.0 / M_PI;
    return true;
}

}  // namespace

std::vector<LensCalibration> parse_offset_v2(const std::string& text, int width, int height) {
    auto groups = split_groups(text, kFieldsPerLensV2);
    std::vector<LensCalibration> lenses;
    for (size_t index = 0; index < groups.size(); ++index) {
        const auto& g = groups[index];
        LensCalibration lens;
        if (!fill_geometry(lens, g, 16, static_cast<int>(index), width, height)) return {};
        set_distortion(lens, g[11], g[12], g[13], g[14], g[15]);
        lens.source = "offset_v2";
        lens.source_distortion = lens.distortion;
        lenses.push_back(std::move(lens));
    }
    return lenses;
}

std::vector<LensCalibration> parse_offset_v3(const std::string& text, int width, int height) {
    auto groups = split_groups(text, kFieldsPerLensV3);
    std::vector<LensCalibration> lenses;
    for (size_t index = 0; index < groups.size(); ++index) {
        const auto& g = groups[index];
        LensCalibration lens;
        if (!fill_geometry(lens, g, 24, static_cast<int>(index), width, height)) return {};
        lens.source = "offset_v3";
        lens.source_distortion.assign(g.begin() + 11, g.begin() + 24);

        // The image circle is bounded by the lens' own square, so it ends at the
        // nearest edge to the principal point.
        double radius_px = std::min({lens.cx, width - lens.cx, lens.cy, height - lens.cy});
        double focal = 0.5 * (lens.fx + lens.fy);
        const double k4[4] = {g[11], g[12], g[13], g[14]};
        if (!refit_radial(lens.xi, k4, focal, radius_px, lens)) return {};
        set_distortion(lens, lens.k1, lens.k2, lens.k3, g[16], g[17]);
        lenses.push_back(std::move(lens));
    }
    return lenses;
}

std::string summarise(const LensCalibration& lens) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                   "lens %d: xi=%.3f fx=%.2f fy=%.2f cx=%.2f cy=%.2f "
                   "(%s, rescaled from a %dx%d canvas by %.5fx%.5fx)",
                   lens.index, lens.xi, lens.fx, lens.fy, lens.cx, lens.cy, lens.source.c_str(),
                   lens.canvas[0], lens.canvas[1], lens.scale[0], lens.scale[1]);
    return buf;
}

}  // namespace insta360
