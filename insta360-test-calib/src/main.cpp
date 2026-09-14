// calib_app — fisheye calibration verification viewer.
//
// Loads one image, detects the ChArUco board in it with OpenCV, loads the
// camera's Mei/omnidirectional intrinsics from data/camera_info.yaml,
// estimates the board's pose from the detected corners and reprojects the
// whole board grid through those intrinsics — so miscalibration shows up as
// detected (green) and reprojected (red) corners visibly disagreeing. Board
// layout and everything else is editable at runtime via ImGui panels.
//
// Usage:
//   calib_app [image_path] [camera_info_yaml_path]
// With no arguments, starts with nothing loaded — drop an image, a
// camera_info.yaml, and/or a charuco meta.txt onto the window (or pass paths
// above / drop them in later; each can be provided independently).

#include "raylib.h"

#include "imgui.h"
#include "rlImGui.h"

#include "MeiCamera.h"
#include "Verification.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace {

// Board dictionaries offered in the UI, same set as
// /home/michal/fisheye/files(1)/show_charuco_4k.py's DICT_NAMES.
constexpr const char* kDictNames[] = {"4X4_50", "5X5_100", "6X6_250", "7X7_1000", "APRILTAG_36h11"};
constexpr cv::aruco::PREDEFINED_DICTIONARY_NAME kDictEnums[] = {
    cv::aruco::DICT_4X4_50, cv::aruco::DICT_5X5_100, cv::aruco::DICT_6X6_250,
    cv::aruco::DICT_7X7_1000, cv::aruco::DICT_APRILTAG_36h11,
};
constexpr int kDictCount = 5;

// Editable ChArUco board layout. Defaults mirror
// /home/michal/fisheye/files(1)/charuco_4k_meta.txt, the metadata written
// alongside the board image (charuco_4k.png) by show_charuco_4k.py when the
// target was generated — update the UI values if the board is regenerated
// with different settings.
struct BoardSettings {
    int dictIndex = 1; // 5X5_100
    int cols      = 14;
    int rows      = 8;
    int squarePx  = 260;
    int markerPx  = 182;
};

// Keeps sanitized values within workable ranges (CharucoBoard::create()
// requires markerLength < squareLength, and degenerate small boards have no
// interior corners to detect).
BoardSettings Sanitize(BoardSettings s) {
    s.dictIndex = std::clamp(s.dictIndex, 0, kDictCount - 1);
    s.cols      = std::clamp(s.cols, 3, 30);
    s.rows      = std::clamp(s.rows, 3, 30);
    s.squarePx  = std::clamp(s.squarePx, 20, 4000);
    s.markerPx  = std::clamp(s.markerPx, 10, s.squarePx - 1);
    return s;
}

std::string ExtLower(const std::string& path) {
    const auto pos = path.find_last_of('.');
    if (pos == std::string::npos) return "";
    std::string e = path.substr(pos + 1);
    std::transform(e.begin(), e.end(), e.begin(), [](unsigned char c) { return std::tolower(c); });
    return e;
}

bool IsImageExt(const std::string& e) { return e == "jpg" || e == "jpeg" || e == "png" || e == "bmp"; }

// Parses a show_charuco_4k.py-style meta.txt (key=value lines: cols, rows,
// dict, square_px, marker_px, plus screen_w/screen_h/margin which this app
// has no use for). Starts from the board's current settings and only
// overwrites keys actually present, so a partial/hand-edited file degrades
// gracefully instead of zeroing everything else out.
bool LoadBoardMetaTxt(const std::string& path, BoardSettings& out) {
    std::ifstream f(path);
    if (!f) {
        std::fprintf(stderr, "calib_app: failed to open '%s'\n", path.c_str());
        return false;
    }
    BoardSettings s = out;
    bool any = false;
    std::string line;
    while (std::getline(f, line)) {
        const auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = line.substr(0, eq);
        const std::string val = line.substr(eq + 1);
        try {
            if (key == "cols") { s.cols = std::stoi(val); any = true; }
            else if (key == "rows") { s.rows = std::stoi(val); any = true; }
            else if (key == "square_px") { s.squarePx = std::stoi(val); any = true; }
            else if (key == "marker_px") { s.markerPx = std::stoi(val); any = true; }
            else if (key == "dict") {
                int idx = -1;
                for (int i = 0; i < kDictCount; ++i)
                    if (val == kDictNames[i]) idx = i;
                if (idx >= 0) { s.dictIndex = idx; any = true; }
                else std::fprintf(stderr, "calib_app: WARNING '%s' dict='%s' not recognized, keeping current\n",
                                   path.c_str(), val.c_str());
            }
        } catch (const std::exception&) {
            std::fprintf(stderr, "calib_app: '%s' has a malformed '%s' value\n", path.c_str(), key.c_str());
        }
    }
    if (!any) {
        std::fprintf(stderr, "calib_app: '%s' had none of cols/rows/dict/square_px/marker_px\n", path.c_str());
        return false;
    }
    out = Sanitize(s);
    return true;
}

struct BoardState {
    cv::Ptr<cv::aruco::Dictionary> dict;
    cv::Ptr<cv::aruco::CharucoBoard> board;
    int totalCorners = 0; // interior chessboard corners == (cols-1)*(rows-1)
};

BoardState BuildBoard(const BoardSettings& s) {
    BoardState b;
    b.dict = cv::aruco::getPredefinedDictionary(kDictEnums[s.dictIndex]);
    b.board = cv::aruco::CharucoBoard::create(s.cols, s.rows, (float)s.squarePx, (float)s.markerPx, b.dict);
    b.totalCorners = (s.cols - 1) * (s.rows - 1);
    return b;
}

struct Detection {
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::Mat charucoCorners; // Nx1 CV_32FC2, empty if none found
    cv::Mat charucoIds;     // Nx1 CV_32SC1
};

Detection DetectCharuco(const cv::Mat& bgr, const BoardState& b) {
    Detection d;
    auto params = cv::aruco::DetectorParameters::create();
    params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    // Fisheye-distorted squares curve a lot near the image edge; widen the
    // adaptive-threshold window range so markers there still binarize cleanly.
    params->adaptiveThreshWinSizeMin = 5;
    params->adaptiveThreshWinSizeMax = 35;
    params->adaptiveThreshWinSizeStep = 5;

    std::vector<std::vector<cv::Point2f>> rejected;
    cv::aruco::detectMarkers(bgr, b.dict, d.markerCorners, d.markerIds, params, rejected);

    if (!d.markerIds.empty()) {
        cv::aruco::interpolateCornersCharuco(d.markerCorners, d.markerIds, bgr, b.board,
                                              d.charucoCorners, d.charucoIds);
    }
    return d;
}

// Uploads a BGR cv::Mat to the GPU as a raylib Texture2D. Must be called
// after InitWindow() (needs a GL context). The conversion buffer only needs
// to outlive the LoadTextureFromImage() call, which uploads synchronously.
Texture2D MatToTexture(const cv::Mat& bgr) {
    cv::Mat rgba;
    cv::cvtColor(bgr, rgba, cv::COLOR_BGR2RGBA);

    Image img{};
    img.data    = rgba.data;
    img.width   = rgba.cols;
    img.height  = rgba.rows;
    img.mipmaps = 1;
    img.format  = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;
    return LoadTextureFromImage(img);
}

// Current image-space -> screen-space mapping (uniform scale, fit-to-window,
// with user pan/zoom applied on top).
struct View {
    float scale  = 1.f;
    Vector2 offset{0.f, 0.f}; // screen-space position of image (0,0)

    void ResetToFit(int imgW, int imgH, int screenW, int screenH) {
        scale = std::min(static_cast<float>(screenW) / imgW, static_cast<float>(screenH) / imgH);
        offset.x = (screenW - imgW * scale) * 0.5f;
        offset.y = (screenH - imgH * scale) * 0.5f;
    }

    Vector2 ToScreen(cv::Point2f p) const {
        return {offset.x + p.x * scale, offset.y + p.y * scale};
    }
};

// Green under 1px, yellow up to 3px, red beyond — thresholds chosen so a
// well-calibrated rig (sub-pixel to ~1px, as validated against this rig's
// sample capture) reads as green.
Color ErrorColor(double errorPx) {
    if (errorPx < 1.0) return LIME;
    if (errorPx < 3.0) return YELLOW;
    return RED;
}

// Transient banner reporting the outcome of the last drag-and-drop.
struct DropStatus {
    std::string text;
    double expiresAt = -1.0;
    Color color = RAYWHITE;
};

} // namespace

int main(int argc, char** argv) {
    std::string imagePath = (argc > 1) ? argv[1] : "";
    std::string cameraInfoPath = (argc > 2) ? argv[2] : "";

    cv::Mat bgr;
    bool imageLoaded = false;
    if (imagePath.empty()) {
        std::printf("calib_app: no image given — drop one onto the window, or pass a path as the first argument\n");
    } else {
        bgr = cv::imread(imagePath, cv::IMREAD_COLOR);
        if (bgr.empty()) {
            std::fprintf(stderr, "calib_app: failed to load image '%s', starting without one\n", imagePath.c_str());
            imagePath.clear();
        } else {
            imageLoaded = true;
            std::printf("calib_app: loaded %s (%dx%d)\n", imagePath.c_str(), bgr.cols, bgr.rows);
        }
    }

    MeiCamera cam;
    if (cameraInfoPath.empty())
        std::printf("calib_app: no camera_info.yaml given — drop one onto the window, or pass a path as the second argument\n");
    else
        cam = LoadMeiCamera(cameraInfoPath);

    BoardSettings appliedSettings; // last settings actually used to detect
    BoardSettings editSettings = appliedSettings; // live UI edit buffer
    BoardState boardState = BuildBoard(appliedSettings);

    Detection det;
    VerificationResult verify;
    auto runDetectAndVerify = [&]() {
        if (!imageLoaded) {
            det = Detection{};
            verify = VerificationResult{};
            verify.message = "no image loaded";
            return;
        }
        det = DetectCharuco(bgr, boardState);
        verify = VerifyPose(det.charucoCorners, det.charucoIds, boardState.board, cam);
        std::printf("calib_app: detected %zu markers, %d/%d charuco corners", det.markerIds.size(),
                    det.charucoCorners.empty() ? 0 : det.charucoCorners.rows, boardState.totalCorners);
        if (verify.ok)
            std::printf(" | reprojection mean=%.2f rms=%.2f max=%.2f px\n", verify.meanErrorPx,
                        verify.rmsErrorPx, verify.maxErrorPx);
        else
            std::printf(" | no reprojection (%s)\n", verify.message.c_str());
    };
    runDetectAndVerify();

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(1280, 960, "calib_app - charuco detection");
    SetTargetFPS(60);
    rlImGuiSetup(true);

    Texture2D tex{}; // left zero-valued (id=0, "no texture") until an image loads
    View view;
    if (imageLoaded) {
        tex = MatToTexture(bgr);
        view.ResetToFit(tex.width, tex.height, GetScreenWidth(), GetScreenHeight());
    }

    bool showMarkers     = true;
    bool showCharuco     = true;
    bool showReprojected = true;
    bool showHelp        = true;
    float markerScale    = 1.0f; // uniform size multiplier for all overlay markers

    DropStatus dropStatus;
    auto setDropStatus = [&](const std::string& text, Color color) {
        dropStatus = {text, GetTime() + 4.0, color};
        std::printf("calib_app: %s\n", text.c_str());
    };

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(DARKGRAY);

        rlImGuiBegin();
        const ImGuiIO& io = ImGui::GetIO();
        const bool blockMouse    = io.WantCaptureMouse;
        const bool blockKeyboard = io.WantCaptureKeyboard;

        // ── drag & drop: image / camera_info.yaml / charuco meta.txt ───────
        if (IsFileDropped()) {
            FilePathList files = LoadDroppedFiles();
            bool needBoardRebuild = false;
            bool needRedetect = false;
            for (unsigned int i = 0; i < files.count; ++i) {
                const std::string path = files.paths[i];
                const std::string name = GetFileName(path.c_str());
                const std::string ext = ExtLower(path);

                if (IsImageExt(ext)) {
                    cv::Mat newBgr = cv::imread(path, cv::IMREAD_COLOR);
                    if (newBgr.empty()) {
                        setDropStatus("failed to load image '" + name + "'", RED);
                    } else {
                        bgr = newBgr;
                        imagePath = path;
                        if (imageLoaded) UnloadTexture(tex);
                        tex = MatToTexture(bgr);
                        imageLoaded = true;
                        view.ResetToFit(tex.width, tex.height, GetScreenWidth(), GetScreenHeight());
                        needRedetect = true;
                        setDropStatus("loaded image '" + name + "'", LIME);
                    }
                } else if (ext == "yaml" || ext == "yml") {
                    MeiCamera newCam = LoadMeiCamera(path);
                    if (newCam.loaded) {
                        cam = newCam;
                        cameraInfoPath = path;
                        needRedetect = true;
                        setDropStatus("loaded camera intrinsics '" + name + "'", LIME);
                    } else {
                        setDropStatus("failed to load camera info '" + name + "'", RED);
                    }
                } else if (ext == "txt") {
                    if (LoadBoardMetaTxt(path, editSettings)) {
                        appliedSettings = editSettings;
                        needBoardRebuild = true;
                        needRedetect = true;
                        setDropStatus("loaded board layout '" + name + "'", LIME);
                    } else {
                        setDropStatus("failed to load board layout '" + name + "'", RED);
                    }
                } else {
                    setDropStatus("unrecognized file '" + name + "' (expected image/.yaml/.txt)", ORANGE);
                }
            }
            if (needBoardRebuild) boardState = BuildBoard(appliedSettings);
            if (needRedetect) runDetectAndVerify();
            UnloadDroppedFiles(files);
        }

        // ── input ────────────────────────────────────────────────────────
        if (!blockKeyboard) {
            if (IsKeyPressed(KEY_M)) showMarkers = !showMarkers;
            if (IsKeyPressed(KEY_C)) showCharuco = !showCharuco;
            if (IsKeyPressed(KEY_P)) showReprojected = !showReprojected;
            if (IsKeyPressed(KEY_H)) showHelp = !showHelp;
            if (IsKeyPressed(KEY_R) && imageLoaded)
                view.ResetToFit(tex.width, tex.height, GetScreenWidth(), GetScreenHeight());
        }
        if (IsWindowResized() && imageLoaded)
            view.ResetToFit(tex.width, tex.height, GetScreenWidth(), GetScreenHeight());

        if (!blockMouse) {
            const float wheel = GetMouseWheelMove();
            if (wheel != 0.f) {
                const Vector2 mouse = GetMousePosition();
                const float oldScale = view.scale;
                const float newScale = std::clamp(oldScale * (1.f + wheel * 0.1f), 0.05f, 40.f);
                // Keep the point under the cursor fixed while zooming.
                view.offset.x = mouse.x - (mouse.x - view.offset.x) * (newScale / oldScale);
                view.offset.y = mouse.y - (mouse.y - view.offset.y) * (newScale / oldScale);
                view.scale = newScale;
            }
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                const Vector2 d = GetMouseDelta();
                view.offset.x += d.x;
                view.offset.y += d.y;
            }
        }

        // ── image + overlays ─────────────────────────────────────────────
        if (!imageLoaded) {
            const char* msg = "Drop a fisheye image onto this window to begin";
            const char* msg2 = "(or pass a path as the first CLI argument)";
            const int fs = 22, fs2 = 15;
            DrawText(msg, (GetScreenWidth() - MeasureText(msg, fs)) / 2, GetScreenHeight() / 2 - 20, fs,
                      Fade(RAYWHITE, 0.85f));
            DrawText(msg2, (GetScreenWidth() - MeasureText(msg2, fs2)) / 2, GetScreenHeight() / 2 + 10, fs2,
                      Fade(RAYWHITE, 0.55f));
        }

        if (imageLoaded)
            DrawTexturePro(tex, {0, 0, (float)tex.width, (float)tex.height},
                           {view.offset.x, view.offset.y, tex.width * view.scale, tex.height * view.scale},
                           {0, 0}, 0.f, WHITE);

        if (showMarkers) {
            for (size_t i = 0; i < det.markerCorners.size(); ++i) {
                const auto& c = det.markerCorners[i];
                for (int k = 0; k < 4; ++k) {
                    Vector2 a = view.ToScreen(c[k]);
                    Vector2 b = view.ToScreen(c[(k + 1) % 4]);
                    DrawLineEx(a, b, 2.f * markerScale, YELLOW);
                }
                Vector2 label = view.ToScreen(c[0]);
                const int labelFs = std::max(1, (int)std::lround(14.f * markerScale));
                DrawText(TextFormat("%d", det.markerIds[i]), (int)label.x + (int)std::lround(4.f * markerScale),
                          (int)label.y - labelFs, labelFs, YELLOW);
            }
        }

        const int nCharuco = det.charucoCorners.empty() ? 0 : det.charucoCorners.rows;
        if (showCharuco && nCharuco > 0) {
            for (int i = 0; i < nCharuco; ++i) {
                cv::Point2f p = det.charucoCorners.at<cv::Point2f>(i);
                Vector2 s = view.ToScreen(p);
                DrawCircleV(s, 4.f * markerScale, GREEN);
            }
        }

        if (showReprojected && verify.ok) {
            // The whole predicted board grid, including corners that weren't
            // detected, so a systematic mismatch (wrong intrinsics, wrong
            // board settings) is visible even where detection failed.
            const float crossHalf = 5.f * markerScale;
            for (const cv::Point2f& p : verify.allReprojected) {
                Vector2 s = view.ToScreen(p);
                DrawLineEx({s.x - crossHalf, s.y}, {s.x + crossHalf, s.y}, 1.5f * markerScale, SKYBLUE);
                DrawLineEx({s.x, s.y - crossHalf}, {s.x, s.y + crossHalf}, 1.5f * markerScale, SKYBLUE);
            }
            // Detected <-> reprojected residual, colored by error magnitude.
            for (const CornerResidual& res : verify.residuals) {
                Vector2 a = view.ToScreen(res.detected);
                Vector2 b = view.ToScreen(res.reprojected);
                DrawLineEx(a, b, 2.f * markerScale, ErrorColor(res.errorPx));
            }
        }

        // ── HUD ──────────────────────────────────────────────────────────
        DrawRectangle(0, 0, GetScreenWidth(), 26, Fade(BLACK, 0.6f));
        if (imageLoaded)
            DrawText(TextFormat("%s  |  %d markers  |  %d/%d charuco corners  |  zoom %.0f%%",
                                 GetFileName(imagePath.c_str()), (int)det.markerIds.size(),
                                 nCharuco, boardState.totalCorners, view.scale * 100.f),
                      8, 6, 14, RAYWHITE);
        else
            DrawText("no image loaded", 8, 6, 14, RAYWHITE);

        if (showHelp) {
            const int x = 8, y = GetScreenHeight() - 132;
            DrawRectangle(x - 6, y - 6, 340, 126, Fade(BLACK, 0.6f));
            DrawText("[M] markers  [C] charuco corners", x, y, 14, RAYWHITE);
            DrawText("[P] reprojection  [R] reset view", x, y + 18, 14, RAYWHITE);
            DrawText("[H] hide this help", x, y + 36, 14, RAYWHITE);
            DrawText("scroll: zoom (at cursor)  drag: pan", x, y + 54, 14, RAYWHITE);
            DrawText("drop image / camera_info.yaml / meta.txt", x, y + 72, 14, RAYWHITE);
            if (imageLoaded)
                DrawText(nCharuco == boardState.totalCorners ? "full board detected" : "partial board",
                         x, y + 96, 14, nCharuco == boardState.totalCorners ? GREEN : YELLOW);
        }

        if (GetTime() < dropStatus.expiresAt) {
            const int w = MeasureText(dropStatus.text.c_str(), 16) + 20;
            const int x = (GetScreenWidth() - w) / 2, y = 34;
            DrawRectangle(x, y, w, 26, Fade(BLACK, 0.75f));
            DrawRectangleLines(x, y, w, 26, dropStatus.color);
            DrawText(dropStatus.text.c_str(), x + 10, y + 5, 16, dropStatus.color);
        }

        // ── board settings panel ─────────────────────────────────────────
        ImGui::SetNextWindowPos(ImVec2(10, 34), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(280, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("Board settings");

        if (!imageLoaded)
            ImGui::TextColored(ImVec4(1.f, 0.65f, 0.2f, 1.f), "No image loaded (drop one onto the window)");

        ImGui::Combo("Dictionary", &editSettings.dictIndex, kDictNames, kDictCount);
        ImGui::InputInt("Columns", &editSettings.cols);
        ImGui::InputInt("Rows", &editSettings.rows);
        ImGui::InputInt("Square (px)", &editSettings.squarePx);
        ImGui::InputInt("Marker (px)", &editSettings.markerPx);

        const BoardSettings sanitized = Sanitize(editSettings);
        if (sanitized.markerPx != editSettings.markerPx || sanitized.squarePx != editSettings.squarePx ||
            sanitized.cols != editSettings.cols || sanitized.rows != editSettings.rows) {
            ImGui::TextColored(ImVec4(1.f, 0.65f, 0.2f, 1.f), "Will clamp to %dx%d, square=%d, marker=%d",
                                sanitized.cols, sanitized.rows, sanitized.squarePx, sanitized.markerPx);
        }

        if (ImGui::Button("Detect")) {
            editSettings = sanitized;
            appliedSettings = sanitized;
            boardState = BuildBoard(appliedSettings);
            runDetectAndVerify();
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset to default")) {
            editSettings = BoardSettings{};
        }

        ImGui::Separator();
        ImGui::Text("Applied: %dx%d  %s  sq=%d mk=%d", appliedSettings.cols, appliedSettings.rows,
                    kDictNames[appliedSettings.dictIndex], appliedSettings.squarePx, appliedSettings.markerPx);
        ImGui::Text("Detected: %zu markers", det.markerIds.size());
        ImGui::TextColored(nCharuco == boardState.totalCorners ? ImVec4(0.3f, 1.f, 0.3f, 1.f)
                                                                 : ImVec4(1.f, 0.85f, 0.2f, 1.f),
                            "%d / %d charuco corners", nCharuco, boardState.totalCorners);

        ImGui::Separator();
        ImGui::SliderFloat("Overlay marker size", &markerScale, 0.25f, 4.0f, "%.2fx");

        ImGui::End();

        // ── calibration verification panel ────────────────────────────────
        ImGui::SetNextWindowPos(ImVec2(10, 260), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(280, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("Calibration verification");

        if (!cam.loaded) {
            ImGui::TextColored(ImVec4(1.f, 0.4f, 0.4f, 1.f), "camera_info.yaml not loaded");
            if (cameraInfoPath.empty())
                ImGui::TextDisabled("drop a camera_info.yaml onto the window");
            else
                ImGui::TextWrapped("%s", cameraInfoPath.c_str());
        } else {
            ImGui::Text("%s", cam.distortionModel.c_str());
            ImGui::Text("fx=%.2f fy=%.2f", cam.fx, cam.fy);
            ImGui::Text("cx=%.2f cy=%.2f", cam.cx, cam.cy);
            ImGui::Text("xi=%.4f", cam.xi);
            ImGui::Text("k1=%.5g k2=%.5g k3=%.5g", cam.k1, cam.k2, cam.k3);
            ImGui::Text("p1=%.5g p2=%.5g", cam.p1, cam.p2);
            ImGui::TextDisabled("(distortion order: k1,k2,k3,p1,p2)");

            ImGui::Separator();
            ImGui::Checkbox("Show reprojection overlay", &showReprojected);

            if (!verify.ok) {
                ImGui::TextColored(ImVec4(1.f, 0.65f, 0.2f, 1.f), "No pose: %s", verify.message.c_str());
            } else {
                ImGui::Text("Reprojection error (%zu corners):", verify.residuals.size());
                ImGui::TextColored(ImVec4(ErrorColor(verify.meanErrorPx).r / 255.f,
                                           ErrorColor(verify.meanErrorPx).g / 255.f,
                                           ErrorColor(verify.meanErrorPx).b / 255.f, 1.f),
                                    "mean %.2f px", verify.meanErrorPx);
                ImGui::TextColored(ImVec4(ErrorColor(verify.rmsErrorPx).r / 255.f,
                                           ErrorColor(verify.rmsErrorPx).g / 255.f,
                                           ErrorColor(verify.rmsErrorPx).b / 255.f, 1.f),
                                    "rms  %.2f px", verify.rmsErrorPx);
                ImGui::TextColored(ImVec4(ErrorColor(verify.maxErrorPx).r / 255.f,
                                           ErrorColor(verify.maxErrorPx).g / 255.f,
                                           ErrorColor(verify.maxErrorPx).b / 255.f, 1.f),
                                    "max  %.2f px", verify.maxErrorPx);
                ImGui::TextDisabled("blue crosshair = predicted corner");
                ImGui::TextDisabled("green/yellow/red = detected->predicted");
            }
        }

        ImGui::End();

        rlImGuiEnd();
        EndDrawing();
    }

    rlImGuiShutdown();
    if (imageLoaded) UnloadTexture(tex);
    CloseWindow();
    return 0;
}
