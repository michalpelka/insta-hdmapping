#include "Camera.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <vector>

namespace {

// One row per CameraModel, in enum order. paramNames lists params[] in the
// layout the ProjectPoint/Unproject implementations index it with.
constexpr CameraModelInfo kCameraModelInfo[] = {
    {"insta360_mei_v2", "Insta360 Mei (insta360_mei_v2)", 10, 5,
     {"fx", "fy", "cx", "cy", "xi", "k1", "k2", "k3", "p1", "p2"}},
    {"plumb_bob", "OpenCV pinhole (plumb_bob)", 9, 4, {"fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3"}},
    {"rational_polynomial", "OpenCV rational (rational_polynomial)", 12, 4,
     {"fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"}},
    {"equidistant", "OpenCV fisheye (equidistant)", 8, 4, {"fx", "fy", "cx", "cy", "k1", "k2", "k3", "k4"}},
};
static_assert(sizeof(kCameraModelInfo) / sizeof(kCameraModelInfo[0]) == kCameraModelCount,
              "kCameraModelInfo needs exactly one row per CameraModel");

cv::Point3d Normalized(double x, double y, double z) {
    const double n = std::sqrt(x * x + y * y + z * z);
    return {x / n, y / n, z / n};
}

cv::Point3d UnprojectMei(const double* a, double xd, double yd) {
    const double xi = a[kMeiXi], k1 = a[5], k2 = a[6], k3 = a[7], p1 = a[8], p2 = a[9];
    // Invert the radial/tangential distortion by fixed-point (Newton-style)
    // iteration, same scheme cv::undistortPoints uses for the pinhole model.
    double x = xd, y = yd;
    for (int it = 0; it < 30; ++it) {
        const double r2 = x * x + y * y;
        const double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
        const double dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
        const double dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
        x = (xd - dx) / radial;
        y = (yd - dy) / radial;
    }

    // Closed-form inverse of the unit-sphere/xi projection: solve
    // (rho2+1)*s^2 - 2*xi*s + (xi^2-1) = 0 for s = Xs.z + xi, where
    // Xs.x = x*s, Xs.y = y*s, Xs.z = s - xi, subject to |Xs| = 1.
    const double rho2 = x * x + y * y;
    const double s = (xi + std::sqrt(std::max(0.0, 1.0 + rho2 * (1.0 - xi * xi)))) / (rho2 + 1.0);
    return {x * s, y * s, s - xi};
}

// cv::undistortPoints' fixed-point iteration, run to convergence.
cv::Point3d UnprojectPinhole(const double* a, double xd, double yd, bool rational) {
    const double k1 = a[4], k2 = a[5], p1 = a[6], p2 = a[7], k3 = a[8];
    const double k4 = rational ? a[9] : 0.0, k5 = rational ? a[10] : 0.0, k6 = rational ? a[11] : 0.0;
    double x = xd, y = yd;
    for (int it = 0; it < 30; ++it) {
        const double r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
        const double icdist = (1.0 + k4 * r2 + k5 * r4 + k6 * r6) / (1.0 + k1 * r2 + k2 * r4 + k3 * r6);
        const double dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
        const double dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
        x = (xd - dx) * icdist;
        y = (yd - dy) * icdist;
    }
    return Normalized(x, y, 1.0);
}

// Newton on theta_d = theta * (1 + k1 theta^2 + ... + k4 theta^8), as
// cv::fisheye::undistortPoints does.
cv::Point3d UnprojectEquidistant(const double* a, double xd, double yd) {
    const double thetaD = std::hypot(xd, yd);
    if (thetaD < 1e-12) return {0.0, 0.0, 1.0};
    const double k1 = a[4], k2 = a[5], k3 = a[6], k4 = a[7];
    double theta = thetaD;
    for (int it = 0; it < 30; ++it) {
        const double t2 = theta * theta, t4 = t2 * t2, t6 = t4 * t2, t8 = t4 * t4;
        const double f = theta * (1.0 + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8) - thetaD;
        const double df = 1.0 + 3 * k1 * t2 + 5 * k2 * t4 + 7 * k3 * t6 + 9 * k4 * t8;
        theta -= f / df;
    }
    const double s = std::sin(theta) / thetaD;
    return {xd * s, yd * s, std::cos(theta)};
}

std::string ParamsSummary(const CameraIntrinsics& cam) {
    std::string s;
    char buf[64];
    for (int k = 0; k < cam.NumParams(); ++k) {
        std::snprintf(buf, sizeof(buf), "%s%s=%.6g", k ? " " : "", cam.Info().paramNames[k], cam.params[k]);
        s += buf;
    }
    return s;
}

} // namespace

const CameraModelInfo& ModelInfo(CameraModel model) { return kCameraModelInfo[static_cast<int>(model)]; }

bool ParseCameraModel(const std::string& name, CameraModel& out) {
    for (int m = 0; m < kCameraModelCount; ++m) {
        if (name == kCameraModelInfo[m].name) {
            out = static_cast<CameraModel>(m);
            return true;
        }
    }
    if (name == "fisheye") {
        out = CameraModel::Equidistant;
        return true;
    }
    return false;
}

cv::Point2d CameraIntrinsics::Project(cv::Point3d P) const {
    const double Pc[3] = {P.x, P.y, P.z};
    double uv[2];
    if (!ProjectPoint(model, params.data(), Pc, uv)) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        return {nan, nan};
    }
    return {uv[0], uv[1]};
}

cv::Point3d CameraIntrinsics::Unproject(cv::Point2d uv, double* thetaDeg) const {
    const double* a = params.data();
    const double xd = (uv.x - a[kCx]) / a[kFx], yd = (uv.y - a[kCy]) / a[kFy];
    cv::Point3d ray;
    switch (model) {
    case CameraModel::Mei: ray = UnprojectMei(a, xd, yd); break;
    case CameraModel::PlumbBob: ray = UnprojectPinhole(a, xd, yd, false); break;
    case CameraModel::RationalPolynomial: ray = UnprojectPinhole(a, xd, yd, true); break;
    case CameraModel::Equidistant: ray = UnprojectEquidistant(a, xd, yd); break;
    }
    if (thetaDeg) *thetaDeg = std::acos(std::clamp(ray.z, -1.0, 1.0)) * 180.0 / CV_PI;
    return ray;
}

CameraIntrinsics LoadCamera(const std::string& path) {
    CameraIntrinsics cam;
    YAML::Node node;
    try {
        node = YAML::LoadFile(path);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "calib_app: failed to load '%s': %s\n", path.c_str(), e.what());
        return cam;
    }

    try {
        const std::string modelName = node["distortion_model"] ? node["distortion_model"].as<std::string>() : "";
        if (modelName.empty()) {
            if (!node["xi"]) {
                std::fprintf(stderr, "calib_app: '%s' has no distortion_model\n", path.c_str());
                return cam;
            }
            std::fprintf(stderr, "calib_app: WARNING '%s' has no distortion_model but has xi, assuming %s\n",
                         path.c_str(), ModelInfo(CameraModel::Mei).name);
            cam.model = CameraModel::Mei;
        } else if (!ParseCameraModel(modelName, cam.model)) {
            std::string known;
            for (int m = 0; m < kCameraModelCount; ++m)
                known += std::string(m ? ", " : "") + ModelInfo(static_cast<CameraModel>(m)).name;
            std::fprintf(stderr, "calib_app: '%s' has distortion_model '%s', which this app doesn't know (%s)\n",
                         path.c_str(), modelName.c_str(), known.c_str());
            return cam;
        }

        cam.frameId = node["frame_id"] ? node["frame_id"].as<std::string>() : "";
        cam.width  = node["width"].as<int>();
        cam.height = node["height"].as<int>();
        cam.params[kFx] = node["fx"].as<double>();
        cam.params[kFy] = node["fy"].as<double>();
        cam.params[kCx] = node["cx"].as<double>();
        cam.params[kCy] = node["cy"].as<double>();
        if (cam.model == CameraModel::Mei) cam.params[kMeiXi] = node["xi"].as<double>();

        // Read defensively: warn (don't silently drop data) if the array
        // isn't exactly as long as the model's coefficient list.
        const CameraModelInfo& info = cam.Info();
        const YAML::Node d = node["distortion"];
        const int n = (int)d.size();
        if (n != info.NumDistortion()) {
            std::string names;
            for (int k = info.distortionOffset; k < info.numParams; ++k)
                names += std::string(k > info.distortionOffset ? "," : "") + info.paramNames[k];
            std::fprintf(stderr,
                         "calib_app: WARNING '%s' distortion has %d elements, expected %d (%s for %s) — missing "
                         "ones default to 0, extras are ignored\n",
                         path.c_str(), n, info.NumDistortion(), names.c_str(), info.name);
        }
        for (int i = 0; i < std::min(n, info.NumDistortion()); ++i)
            cam.params[info.distortionOffset + i] = d[i].as<double>();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "calib_app: '%s' is missing an expected field: %s\n", path.c_str(), e.what());
        return cam;
    }

    cam.loaded = true;
    std::printf("calib_app: loaded %s intrinsics from %s (frame '%s', %dx%d, %s)\n", cam.Info().name, path.c_str(),
                cam.frameId.c_str(), cam.width, cam.height, ParamsSummary(cam).c_str());
    return cam;
}

namespace {

std::string YamlNumber(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.10g", v);
    return buf;
}

std::string YamlArray(const std::vector<double>& values) {
    std::string s = "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i) s += ", ";
        s += YamlNumber(values[i]);
    }
    return s + "]";
}

// "key" for a top-level "key: value" line, "" for anything else (comments,
// blank lines, indented continuation lines of a block-style value).
std::string TopLevelKey(const std::string& line) {
    if (line.empty() || line[0] == ' ' || line[0] == '\t' || line[0] == '#' || line[0] == '-') return "";
    const auto colon = line.find(':');
    if (colon == std::string::npos) return "";
    return line.substr(0, colon);
}

} // namespace

bool SaveCamera(const std::string& path, const CameraIntrinsics& cam, const std::string& templatePath,
                const std::string& comment, std::string* error) {
    auto fail = [&](const std::string& why) {
        if (error) *error = why;
        std::fprintf(stderr, "calib_app: %s\n", why.c_str());
        return false;
    };

    const CameraModelInfo& info = cam.Info();
    const double fx = cam.params[kFx], fy = cam.params[kFy], cx = cam.params[kCx], cy = cam.params[kCy];
    const std::vector<double> distortion(cam.params.begin() + info.distortionOffset,
                                         cam.params.begin() + info.numParams);

    // Everything CameraIntrinsics models, in the order insta360-to-images'
    // write_camera_info() emits it (so a file written from scratch reads the
    // same as one it produced). Keys it writes that aren't listed here
    // (serial, model, firmware, video_stream_index, rotation_deg,
    // translation) aren't known to this app: they're carried over from the
    // template when there is one and otherwise left out. `drop` removes a
    // key the template has but this model doesn't (xi, for non-Mei models).
    struct Owned {
        std::string key, value;
        bool drop = false;
    };
    const bool mei = cam.model == CameraModel::Mei;
    const std::vector<Owned> owned = {
        {"width", std::to_string(cam.width)},
        {"height", std::to_string(cam.height)},
        {"distortion_model", info.name},
        {"xi", mei ? YamlNumber(cam.params[kMeiXi]) : "", !mei},
        {"fx", YamlNumber(fx)},
        {"fy", YamlNumber(fy)},
        {"cx", YamlNumber(cx)},
        {"cy", YamlNumber(cy)},
        {"distortion", YamlArray(distortion)},
        {"k", YamlArray({fx, 0, cx, 0, fy, cy, 0, 0, 1})},
        {"p", YamlArray({fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0})},
    };
    auto findOwned = [&](const std::string& key) -> const Owned* {
        for (const Owned& o : owned)
            if (o.key == key) return &o;
        return nullptr;
    };

    std::vector<std::string> lines;
    if (!comment.empty()) lines.push_back("# calib_app: " + comment);

    std::set<std::string> written;
    if (!templatePath.empty()) {
        std::ifstream in(templatePath);
        if (!in) return fail("can't read template '" + templatePath + "'");
        std::string line;
        bool skippingContinuation = false; // inside a replaced key's block-style value
        while (std::getline(in, line)) {
            if (line.rfind("# calib_app:", 0) == 0) continue; // superseded by `comment`
            const std::string key = TopLevelKey(line);
            if (key.empty()) {
                if (skippingContinuation && !line.empty() && line[0] != '#') continue;
                lines.push_back(line);
                continue;
            }
            skippingContinuation = false;
            written.insert(key);
            if (const Owned* o = findOwned(key)) {
                if (!o->drop) lines.push_back(key + ": " + o->value);
                skippingContinuation = true;
            } else {
                lines.push_back(line);
            }
        }
    }
    if (!written.count("frame_id") && !cam.frameId.empty()) lines.push_back("frame_id: " + cam.frameId);
    for (const Owned& o : owned) {
        if (written.count(o.key) || o.drop) continue;
        lines.push_back(o.key + ": " + o.value);
        if (o.key == "k" && !written.count("r")) lines.push_back("r: [1, 0, 0, 0, 1, 0, 0, 0, 1]");
    }

    std::ostringstream text;
    for (const std::string& l : lines) text << l << '\n';

    // Parse what's about to be written before touching the file, so a
    // template in some layout the line rewrite above mangles fails here
    // instead of leaving a broken camera_info.yaml on disk.
    try {
        const YAML::Node check = YAML::Load(text.str());
        if (std::abs(check["fx"].as<double>() - fx) > 1e-6 * std::max(1.0, std::abs(fx)) ||
            (int)check["distortion"].size() != info.NumDistortion() ||
            check["distortion_model"].as<std::string>() != info.name)
            return fail("rewritten camera_info doesn't read back correctly (unexpected template layout?)");
    } catch (const std::exception& e) {
        return fail(std::string("rewritten camera_info isn't valid YAML: ") + e.what());
    }

    std::ofstream out(path);
    out << text.str();
    out.close();
    if (!out) return fail("can't write '" + path + "'");
    std::printf("calib_app: wrote %s\n", path.c_str());
    return true;
}