#include "insta360/calibration.hpp"

#include <cstdio>
#include <sstream>

namespace insta360 {

namespace {

constexpr int kFieldsPerLens = 19;

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

}  // namespace

std::vector<LensCalibration> parse_offset_v2(const std::string& text, int width, int height) {
    auto tokens = split(text, '_');
    if (tokens.size() < 2) return {};
    int lens_count = 0;
    if (!parse_int(tokens[0], lens_count) || lens_count <= 0) return {};

    std::vector<std::string> body(tokens.begin() + 1, tokens.end());
    if (static_cast<int>(body.size()) < lens_count * kFieldsPerLens) return {};

    std::vector<LensCalibration> lenses;
    for (int index = 0; index < lens_count; ++index) {
        const std::string* group = body.data() + index * kFieldsPerLens;

        double xi, fx, fy, cx, cy;
        double rx, ry, rz, tx, ty, tz;
        double k1, k2, k3, p1, p2;
        double canvas_w_d, canvas_h_d;
        bool ok = parse_double(group[0], xi) && parse_double(group[1], fx) &&
                  parse_double(group[2], fy) && parse_double(group[3], cx) &&
                  parse_double(group[4], cy) && parse_double(group[5], rx) &&
                  parse_double(group[6], ry) && parse_double(group[7], rz) &&
                  parse_double(group[8], tx) && parse_double(group[9], ty) &&
                  parse_double(group[10], tz) && parse_double(group[11], k1) &&
                  parse_double(group[12], k2) && parse_double(group[13], k3) &&
                  parse_double(group[14], p1) && parse_double(group[15], p2) &&
                  parse_double(group[16], canvas_w_d) && parse_double(group[17], canvas_h_d);
        if (!ok) return {};
        int canvas_w = static_cast<int>(canvas_w_d);
        int canvas_h = static_cast<int>(canvas_h_d);
        if (canvas_w <= 0 || canvas_h <= 0) return {};

        // Each lens occupies the left or right square half of the stitched canvas.
        double half_width = canvas_w / 2.0;
        // Scale each axis independently so a non-square output (from --scale) still
        // gets consistent intrinsics.
        double scale_x = width / half_width;
        double scale_y = height / static_cast<double>(canvas_h);

        LensCalibration lens;
        lens.index = index;
        lens.width = width;
        lens.height = height;
        lens.fx = fx * scale_x;
        lens.fy = fy * scale_y;
        lens.cx = (cx - index * half_width) * scale_x;
        lens.cy = cy * scale_y;
        lens.xi = xi;  // dimensionless sphere-model parameter; not a pixel quantity
        lens.k1 = k1;
        lens.k2 = k2;
        lens.k3 = k3;
        lens.p1 = p1;
        lens.p2 = p2;
        lens.distortion = {k1, k2, k3, p1, p2};
        lens.rotation_deg = {rx, ry, rz};
        lens.translation = {tx, ty, tz};
        lens.canvas = {canvas_w, canvas_h};
        lens.scale = {scale_x, scale_y};
        lenses.push_back(std::move(lens));
    }
    return lenses;
}

std::string summarise(const LensCalibration& lens) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                   "lens %d: xi=%.3f fx=%.2f fy=%.2f cx=%.2f cy=%.2f "
                   "(rescaled from a %dx%d canvas by %.5fx%.5fx)",
                   lens.index, lens.xi, lens.fx, lens.fy, lens.cx, lens.cy, lens.canvas[0],
                   lens.canvas[1], lens.scale[0], lens.scale[1]);
    return buf;
}

}  // namespace insta360
