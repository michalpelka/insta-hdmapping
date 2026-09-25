// calib_app — camera calibration and calibration verification.
//
// Loads a set of images of a ChArUco board and detects the board in each with
// OpenCV. Against the camera's intrinsics (camera_info.yaml: the Insta360 rig's
// Mei model or one of OpenCV's standard models, see Camera.h) it then
//   - verifies them: each image's board pose is estimated from its detected
//     corners and the whole board grid is reprojected through the
//     intrinsics, so miscalibration shows up as detected (green) and
//     reprojected (blue) corners visibly disagreeing, and
//   - re-estimates them: a joint Ceres fit of the chosen model's intrinsics
//     over every image selected for calibration, starting from the loaded
//     camera_info.yaml (or a generic guess), whose result can be made the
//     active camera and saved as a new camera_info.yaml.
// Board layout and everything else is editable at runtime via ImGui panels.
//
// Usage:
//   calib_app [path ...]
// Each path may be an image, a folder (its images plus its camera_info.yaml —
// e.g. an insta360-to-images cam_front/ directory), a camera_info.yaml or a
// charuco meta.txt, in any order; files dropped onto the window are handled
// the same way. With no arguments, starts with nothing loaded.

#include "raylib.h"

#include "imgui.h"
#include "rlImGui.h"

#include "Board.h"
#include "ImageSet.h"
#include "Calibration.h"
#include "Camera.h"
#include "Verification.h"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

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

bool Finite(cv::Point2f p) { return std::isfinite(p.x) && std::isfinite(p.y); }

// Green under 1px, yellow up to 3px, red beyond — thresholds chosen so a
// well-calibrated rig (sub-pixel to ~1px, as validated against this rig's
// sample capture) reads as green.
Color ErrorColor(double errorPx) {
    if (errorPx < 1.0) return LIME;
    if (errorPx < 3.0) return YELLOW;
    return RED;
}

ImVec4 ToImVec4(Color c) { return ImVec4(c.r / 255.f, c.g / 255.f, c.b / 255.f, 1.f); }

const ImVec4 kWarnColor(1.f, 0.65f, 0.2f, 1.f);
const ImVec4 kBadColor(1.f, 0.4f, 0.4f, 1.f);
const ImVec4 kGoodColor(0.3f, 1.f, 0.3f, 1.f);
const ImVec4 kPartialColor(1.f, 0.85f, 0.2f, 1.f);

std::string FileName(const std::string& path) { return fs::path(path).filename().string(); }

// Transient banner reporting the outcome of the last load/save/calibration.
struct StatusBanner {
    std::string text;
    double expiresAt = -1.0;
    Color color = RAYWHITE;
};

struct App {
    // ── images ──
    std::vector<ImageEntry> images;
    int current = -1;       // index into images, -1 if none
    cv::Mat currentBgr;     // decoded pixels of images[current]; empty if it failed to load
    Texture2D tex{};        // left zero-valued (id=0, "no texture") until an image loads
    bool texLoaded = false;
    View view;
    bool detectAllRunning = false;
    bool scrollListToCurrent = false;

    // ── camera ──
    CameraIntrinsics cam;       // the active camera everything is verified against
    std::string cameraInfoPath; // last camera_info.yaml loaded (also the save template)
    CameraIntrinsics loadedCam; // what cameraInfoPath held, to revert to
    bool camIsCalibration = false;

    // ── board ──
    BoardSettings appliedSettings;                // last settings actually used to detect
    BoardSettings editSettings = appliedSettings; // live UI edit buffer
    BoardState boardState = BuildBoard(appliedSettings);

    // ── calibration ──
    CameraModel calibModel = CameraModel::Mei; // model the next calibration fits
    CalibrationOptions calibOptions = DefaultCalibrationOptions(calibModel);
    bool calibFromGeneric = false;
    bool haveCalib = false;
    CalibrationResult calib;
    CameraIntrinsics calibInitial;
    CalibrationOptions calibRunOptions; // what `calib` was actually run with
    std::string calibTemplatePath; // camera_info.yaml whose other fields a save keeps, "" if none
    std::string calibStartLabel;   // what the calibration started from, for display
    char savePath[1024] = "";
    bool overwriteArmed = false;

    // ── display ──
    bool showMarkers     = true;
    bool showCharuco     = true;
    bool showReprojected = true;
    bool showHelp        = true;
    float markerScale    = 1.0f; // uniform size multiplier for all overlay markers
    StatusBanner banner;

    void SetBanner(const std::string& text, Color color) {
        banner = {text, GetTime() + 5.0, color};
        std::printf("calib_app: %s\n", text.c_str());
    }

    ImageEntry* Current() { return current >= 0 ? &images[current] : nullptr; }

    // ── loading ───────────────────────────────────────────────────────────

    // Loads every path given on the command line or dropped on the window:
    // images (and folders of them) are appended to the set, a .yaml becomes
    // the active camera, a .txt the board layout.
    void LoadPaths(const std::vector<std::string>& inputs) {
        // One banner for the whole batch, colored by its worst outcome.
        enum Severity { kOk, kWarning, kError };
        std::vector<std::string> notes;
        Severity worst = kOk;
        auto note = [&](const std::string& text, Severity sev) {
            notes.push_back(text);
            worst = std::max(worst, sev);
        };

        int added = 0, alreadyLoaded = 0, firstNew = -1;
        bool boardChanged = false, camChanged = false;
        for (const std::string& input : inputs) {
            const std::vector<std::string> files = ExpandInputPath(input);
            if (files.empty()) note("nothing loadable in folder '" + FileName(input) + "'", kWarning);
            for (const std::string& path : files) {
                const std::string name = FileName(path);
                const std::string ext = ExtLower(path);
                if (IsImageExt(ext)) {
                    auto it = std::find_if(images.begin(), images.end(),
                                           [&](const ImageEntry& e) { return e.path == path; });
                    if (it != images.end()) {
                        ++alreadyLoaded;
                        if (firstNew < 0) firstNew = (int)(it - images.begin());
                        continue;
                    }
                    ImageEntry e;
                    e.path = path;
                    images.push_back(std::move(e));
                    if (firstNew < 0) firstNew = (int)images.size() - 1;
                    ++added;
                } else if (ext == "yaml" || ext == "yml") {
                    CameraIntrinsics newCam = LoadCamera(path);
                    if (newCam.loaded) {
                        cam = loadedCam = newCam;
                        cameraInfoPath = path;
                        camIsCalibration = false;
                        camChanged = true;
                        SetCalibrationModel(cam.model);
                        note("loaded " + std::string(cam.Info().name) + " intrinsics '" + name + "'", kOk);
                    } else {
                        note("failed to load camera info '" + name + "'", kError);
                    }
                } else if (ext == "txt") {
                    if (LoadBoardMetaTxt(path, editSettings)) {
                        appliedSettings = editSettings;
                        boardChanged = true;
                        note("loaded board layout '" + name + "'", kOk);
                    } else {
                        note("failed to load board layout '" + name + "'", kError);
                    }
                } else {
                    note("unrecognized file '" + name + "' (expected image/folder/.yaml/.txt)", kWarning);
                }
            }
        }
        if (added == 1 && alreadyLoaded == 0) note("added image '" + FileName(images.back().path) + "'", kOk);
        else if (added > 0) note("added " + std::to_string(added) + " images", kOk);
        if (alreadyLoaded > 0) note(std::to_string(alreadyLoaded) + " already loaded", kOk);

        if (boardChanged) {
            boardState = BuildBoard(appliedSettings);
            InvalidateDetections();
        } else if (camChanged) {
            ReverifyAll();
        }
        if (firstNew >= 0) Select(firstNew);

        if (!notes.empty()) {
            std::string text = notes[0];
            for (size_t i = 1; i < notes.size(); ++i) text += "  |  " + notes[i];
            SetBanner(text, worst == kError ? RED : worst == kWarning ? ORANGE : LIME);
        }
    }

    // Makes images[idx] the displayed image, decoding it (and detecting the
    // board in it, if that hasn't happened for the current board yet).
    void Select(int idx) {
        const int prevW = texLoaded ? tex.width : -1, prevH = texLoaded ? tex.height : -1;
        if (texLoaded) UnloadTexture(tex);
        texLoaded = false;
        currentBgr.release();
        if (images.empty()) {
            current = -1;
            return;
        }
        current = std::clamp(idx, 0, (int)images.size() - 1);
        scrollListToCurrent = true;

        ImageEntry& e = images[current];
        currentBgr = cv::imread(e.path, cv::IMREAD_COLOR);
        if (currentBgr.empty()) {
            e.loadFailed = true;
            SetBanner("failed to load image '" + FileName(e.path) + "'", RED);
            return;
        }
        e.loadFailed = false;
        tex = MatToTexture(currentBgr);
        texLoaded = true;
        // Same-size images keep the zoom/pan, so the same region can be
        // compared while stepping through a capture.
        if (tex.width != prevW || tex.height != prevH)
            view.ResetToFit(tex.width, tex.height, GetScreenWidth(), GetScreenHeight());
        if (!e.detected) Process(e, currentBgr);
    }

    void Step(int delta) {
        if (!images.empty()) Select(std::clamp(current + delta, 0, (int)images.size() - 1));
    }

    void Remove(int idx) {
        if (idx < 0 || idx >= (int)images.size()) return;
        images.erase(images.begin() + idx);
        if (idx < current) --current;
        else if (idx == current) Select(std::min(idx, (int)images.size() - 1));
    }

    void Clear() {
        images.clear();
        detectAllRunning = false;
        Select(-1);
    }

    // ── detection / verification ──────────────────────────────────────────

    void Process(ImageEntry& e, const cv::Mat& bgr) {
        e.width = bgr.cols;
        e.height = bgr.rows;
        e.det = DetectCharuco(bgr, boardState);
        e.verify = VerifyPose(e.det.charucoCorners, e.det.charucoIds, boardState.board, cam);
        e.detected = true;
        std::printf("calib_app: %s: %zu markers, %d/%d charuco corners", FileName(e.path).c_str(),
                    e.det.markerIds.size(), e.det.CornerCount(), boardState.totalCorners);
        if (e.verify.ok)
            std::printf(" | reprojection mean=%.2f rms=%.2f max=%.2f px\n", e.verify.meanErrorPx,
                        e.verify.rmsErrorPx, e.verify.maxErrorPx);
        else
            std::printf(" | no reprojection (%s)\n", e.verify.message.c_str());
    }

    // Board changed: every cached detection is stale. The displayed image is
    // re-detected straight away; the rest wait for a visit or "Detect all".
    void InvalidateDetections() {
        for (ImageEntry& e : images) {
            e.detected = false;
            e.det = Detection{};
            e.verify = VerificationResult{};
        }
        if (ImageEntry* e = Current(); e && !currentBgr.empty()) Process(*e, currentBgr);
    }

    // Camera changed: detections still hold, only the poses/reprojections
    // need redoing — cheap enough to do for every image at once.
    void ReverifyAll() {
        for (ImageEntry& e : images)
            if (e.detected) e.verify = VerifyPose(e.det.charucoCorners, e.det.charucoIds, boardState.board, cam);
    }

    int DetectedCount() const {
        return (int)std::count_if(images.begin(), images.end(),
                                  [](const ImageEntry& e) { return e.detected || e.loadFailed; });
    }

    // One image per frame, so the window stays responsive while a whole
    // capture folder is worked through.
    void DetectAllStep() {
        if (!detectAllRunning) return;
        for (int i = 0; i < (int)images.size(); ++i) {
            ImageEntry& e = images[i];
            if (e.detected || e.loadFailed) continue;
            if (i == current && !currentBgr.empty()) {
                Process(e, currentBgr);
                return;
            }
            const cv::Mat bgr = cv::imread(e.path, cv::IMREAD_COLOR);
            if (bgr.empty()) e.loadFailed = true;
            else Process(e, bgr);
            return;
        }
        detectAllRunning = false;
        SetBanner("detection finished: " + std::to_string(CalibrationCandidates().size()) + " of " +
                      std::to_string(images.size()) + " images usable for calibration",
                  LIME);
    }

    // ── calibration ───────────────────────────────────────────────────────

    // Switching models resets which parameters are held: the indices mean
    // different things per model.
    void SetCalibrationModel(CameraModel model) {
        if (model == calibModel) return;
        calibModel = model;
        calibOptions = DefaultCalibrationOptions(model);
    }

    std::vector<int> CalibrationCandidates() const {
        std::vector<int> idx;
        for (int i = 0; i < (int)images.size(); ++i) {
            const ImageEntry& e = images[i];
            if (e.detected && !e.loadFailed && e.useForCalibration && e.det.CornerCount() >= kMinVerifyCorners)
                idx.push_back(i);
        }
        return idx;
    }

    void RunCalibration() {
        const std::vector<int> idx = CalibrationCandidates();
        if ((int)idx.size() < kMinCalibrationViews) {
            SetBanner("calibration needs >= " + std::to_string(kMinCalibrationViews) +
                          " usable images, have " + std::to_string(idx.size()),
                      ORANGE);
            return;
        }
        const int w = images[idx[0]].width, h = images[idx[0]].height;
        for (int i : idx) {
            if (images[i].width != w || images[i].height != h) {
                SetBanner("images differ in size (" + FileName(images[i].path) + ") — calibrate one camera at a time",
                          RED);
                return;
            }
        }

        const std::vector<cv::Point3f> corners = BoardCorners(*boardState.board);
        std::vector<BoardView> views;
        views.reserve(idx.size());
        for (int i : idx) {
            const Detection& d = images[i].det;
            BoardView v;
            for (int k = 0; k < d.CornerCount(); ++k) {
                v.objectPoints.push_back(corners[d.charucoIds.at<int>(k)]);
                v.imagePoints.push_back(d.charucoCorners.at<cv::Point2f>(k));
            }
            views.push_back(std::move(v));
        }

        const bool sizeMatches = cam.loaded && cam.width == w && cam.height == h;
        const bool modelMatches = cam.loaded && cam.model == calibModel;
        if (modelMatches && !sizeMatches && !calibFromGeneric)
            SetBanner("camera_info.yaml is " + std::to_string(cam.width) + "x" + std::to_string(cam.height) +
                          " but the images are " + std::to_string(w) + "x" + std::to_string(h) +
                          " — starting from a generic guess",
                      ORANGE);
        const bool fromCamera = sizeMatches && modelMatches && !calibFromGeneric;
        calibInitial = fromCamera ? cam : GenericGuess(calibModel, w, h, views);
        if (!fromCamera && cam.loaded) calibInitial.frameId = cam.frameId;
        // A save keeps the loaded camera_info.yaml's other fields whenever the
        // images match it, even when the fit itself started elsewhere (or is
        // of another model).
        calibTemplatePath = sizeMatches ? cameraInfoPath : "";
        calibStartLabel = !fromCamera        ? "a generic guess"
                          : camIsCalibration ? "the previous calibration"
                                             : FileName(cameraInfoPath);

        calibRunOptions = calibOptions;
        calib = Calibrate(views, calibInitial, calibRunOptions);
        haveCalib = true;
        overwriteArmed = false;
        if (!calib.ok) {
            SetBanner("calibration failed: " + calib.message, RED);
            return;
        }
        if (savePath[0] == '\0') {
            const std::string dir = fs::path(calibTemplatePath.empty() ? images[idx[0]].path : calibTemplatePath)
                                        .parent_path()
                                        .string();
            const std::string defaultPath = (fs::path(dir) / "camera_info_calibrated.yaml").string();
            std::snprintf(savePath, sizeof(savePath), "%s", defaultPath.c_str());
        }
        std::printf("calib_app: calibration: %s\n", calib.message.c_str());
        for (int k = 0; k < calib.camera.NumParams(); ++k)
            std::printf("calib_app:   %-2s = %.8g +- %.3g\n", calib.camera.Info().paramNames[k],
                        calib.camera.params[k], calib.stdDev[k]);
        SetBanner("calibrated: " + calib.message, LIME);
    }

    void ApplyCalibration() {
        cam = calib.camera;
        camIsCalibration = true;
        ReverifyAll();
        SetBanner("verifying against the calibration result", LIME);
    }

    void RevertCamera() {
        cam = loadedCam;
        camIsCalibration = false;
        ReverifyAll();
        SetBanner("verifying against '" + FileName(cameraInfoPath) + "' again", LIME);
    }

    void SaveCalibration() {
        const std::string path = savePath;
        std::error_code ec;
        if (fs::exists(path, ec) && !overwriteArmed) {
            overwriteArmed = true;
            SetBanner("'" + FileName(path) + "' exists — press Save again to overwrite it", ORANGE);
            return;
        }
        overwriteArmed = false;

        std::string comment = "intrinsics re-estimated from " + calib.message;
        std::string fixed;
        for (int k = 0; k < calib.camera.NumParams(); ++k)
            if (calibRunOptions.fixed[k]) fixed += std::string(" ") + calib.camera.Info().paramNames[k];
        if (!fixed.empty()) comment += "; fixed:" + fixed;
        comment += "; started from " + calibStartLabel;
        std::string error;
        if (SaveCamera(path, calib.camera, calibTemplatePath, comment, &error))
            SetBanner("saved '" + path + "'", LIME);
        else
            SetBanner("save failed: " + error, RED);
    }

    // ── per-frame UI ──────────────────────────────────────────────────────

    void HandleInput(bool blockMouse, bool blockKeyboard) {
        if (!blockKeyboard) {
            if (IsKeyPressed(KEY_M)) showMarkers = !showMarkers;
            if (IsKeyPressed(KEY_C)) showCharuco = !showCharuco;
            if (IsKeyPressed(KEY_P)) showReprojected = !showReprojected;
            if (IsKeyPressed(KEY_H)) showHelp = !showHelp;
            if (IsKeyPressed(KEY_R) && texLoaded)
                view.ResetToFit(tex.width, tex.height, GetScreenWidth(), GetScreenHeight());
            if (IsKeyPressed(KEY_RIGHT) || IsKeyPressedRepeat(KEY_RIGHT)) Step(1);
            if (IsKeyPressed(KEY_LEFT) || IsKeyPressedRepeat(KEY_LEFT)) Step(-1);
        }
        if (IsWindowResized() && texLoaded)
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
    }

    void DrawImageAndOverlays() {
        const ImageEntry* e = Current();
        if (!e) {
            const char* msg = "Drop fisheye images or a capture folder onto this window to begin";
            const char* msg2 = "(or pass paths on the command line)";
            const int fs = 22, fs2 = 15;
            DrawText(msg, (GetScreenWidth() - MeasureText(msg, fs)) / 2, GetScreenHeight() / 2 - 20, fs,
                     Fade(RAYWHITE, 0.85f));
            DrawText(msg2, (GetScreenWidth() - MeasureText(msg2, fs2)) / 2, GetScreenHeight() / 2 + 10, fs2,
                     Fade(RAYWHITE, 0.55f));
            return;
        }
        if (!texLoaded) {
            const std::string msg = "failed to load " + FileName(e->path);
            DrawText(msg.c_str(), (GetScreenWidth() - MeasureText(msg.c_str(), 20)) / 2, GetScreenHeight() / 2, 20,
                     RED);
            return;
        }

        DrawTexturePro(tex, {0, 0, (float)tex.width, (float)tex.height},
                       {view.offset.x, view.offset.y, tex.width * view.scale, tex.height * view.scale}, {0, 0}, 0.f,
                       WHITE);

        const Detection& det = e->det;
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

        if (showCharuco) {
            for (int i = 0; i < det.CornerCount(); ++i)
                DrawCircleV(view.ToScreen(det.charucoCorners.at<cv::Point2f>(i)), 4.f * markerScale, GREEN);
        }

        if (showReprojected && e->verify.ok) {
            // The whole predicted board grid, including corners that weren't
            // detected, so a systematic mismatch (wrong intrinsics, wrong
            // board settings) is visible even where detection failed.
            const float crossHalf = 5.f * markerScale;
            for (const cv::Point2f& p : e->verify.allReprojected) {
                if (!Finite(p)) continue;
                Vector2 s = view.ToScreen(p);
                DrawLineEx({s.x - crossHalf, s.y}, {s.x + crossHalf, s.y}, 1.5f * markerScale, SKYBLUE);
                DrawLineEx({s.x, s.y - crossHalf}, {s.x, s.y + crossHalf}, 1.5f * markerScale, SKYBLUE);
            }
            // Detected <-> reprojected residual, colored by error magnitude.
            for (const CornerResidual& res : e->verify.residuals) {
                if (!Finite(res.reprojected)) continue;
                DrawLineEx(view.ToScreen(res.detected), view.ToScreen(res.reprojected), 2.f * markerScale,
                           ErrorColor(res.errorPx));
            }
        }
    }

    void DrawHud() {
        DrawRectangle(0, 0, GetScreenWidth(), 26, Fade(BLACK, 0.6f));
        if (const ImageEntry* e = Current())
            DrawText(TextFormat("[%d/%d] %s  |  %d markers  |  %d/%d charuco corners  |  zoom %.0f%%", current + 1,
                                (int)images.size(), FileName(e->path).c_str(), (int)e->det.markerIds.size(),
                                e->det.CornerCount(), boardState.totalCorners, view.scale * 100.f),
                     8, 6, 14, RAYWHITE);
        else
            DrawText("no image loaded", 8, 6, 14, RAYWHITE);

        if (showHelp) {
            const int x = 8, y = GetScreenHeight() - 150;
            DrawRectangle(x - 6, y - 6, 360, 144, Fade(BLACK, 0.6f));
            DrawText("[M] markers  [C] charuco corners", x, y, 14, RAYWHITE);
            DrawText("[P] reprojection  [R] reset view", x, y + 18, 14, RAYWHITE);
            DrawText("[<-] [->] previous / next image", x, y + 36, 14, RAYWHITE);
            DrawText("[H] hide this help", x, y + 54, 14, RAYWHITE);
            DrawText("scroll: zoom (at cursor)  drag: pan", x, y + 72, 14, RAYWHITE);
            DrawText("drop images / folder / camera_info.yaml / meta.txt", x, y + 90, 14, RAYWHITE);
            if (const ImageEntry* e = Current(); e && texLoaded) {
                const bool full = e->det.CornerCount() == boardState.totalCorners;
                DrawText(full ? "full board detected" : "partial board", x, y + 114, 14, full ? GREEN : YELLOW);
            }
        }

        if (GetTime() < banner.expiresAt) {
            const int w = MeasureText(banner.text.c_str(), 16) + 20;
            const int x = (GetScreenWidth() - w) / 2, y = 34;
            DrawRectangle(x, y, w, 26, Fade(BLACK, 0.75f));
            DrawRectangleLines(x, y, w, 26, banner.color);
            DrawText(banner.text.c_str(), x + 10, y + 5, 16, banner.color);
        }
    }

    void BoardPanel() {
        ImGui::SetNextWindowPos(ImVec2(10, 34), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(280, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("Board settings");

        ImGui::Combo("Dictionary", &editSettings.dictIndex, kDictNames, kDictCount);
        ImGui::InputInt("Columns", &editSettings.cols);
        ImGui::InputInt("Rows", &editSettings.rows);
        ImGui::InputInt("Square (px)", &editSettings.squarePx);
        ImGui::InputInt("Marker (px)", &editSettings.markerPx);

        const BoardSettings sanitized = Sanitize(editSettings);
        if (sanitized.markerPx != editSettings.markerPx || sanitized.squarePx != editSettings.squarePx ||
            sanitized.cols != editSettings.cols || sanitized.rows != editSettings.rows) {
            ImGui::TextColored(kWarnColor, "Will clamp to %dx%d, square=%d, marker=%d", sanitized.cols,
                               sanitized.rows, sanitized.squarePx, sanitized.markerPx);
        }

        if (ImGui::Button("Detect")) {
            editSettings = sanitized;
            appliedSettings = sanitized;
            boardState = BuildBoard(appliedSettings);
            InvalidateDetections();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Apply these settings; every image's detection is redone");
        ImGui::SameLine();
        if (ImGui::Button("Reset to default")) editSettings = BoardSettings{};

        ImGui::Separator();
        ImGui::Text("Applied: %dx%d  %s  sq=%d mk=%d", appliedSettings.cols, appliedSettings.rows,
                    kDictNames[appliedSettings.dictIndex], appliedSettings.squarePx, appliedSettings.markerPx);
        if (const ImageEntry* e = Current()) {
            const int n = e->det.CornerCount();
            ImGui::Text("This image: %zu markers", e->det.markerIds.size());
            ImGui::TextColored(n == boardState.totalCorners ? kGoodColor : kPartialColor, "%d / %d charuco corners",
                               n, boardState.totalCorners);
        } else {
            ImGui::TextColored(kWarnColor, "No image loaded (drop one onto the window)");
        }

        ImGui::Separator();
        ImGui::SliderFloat("Overlay marker size", &markerScale, 0.25f, 4.0f, "%.2fx");
        ImGui::End();
    }

    void VerificationPanel() {
        ImGui::SetNextWindowPos(ImVec2(10, 280), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(280, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("Calibration verification");

        if (!cam.loaded) {
            ImGui::TextColored(kBadColor, "camera_info.yaml not loaded");
            ImGui::TextDisabled("drop a camera_info.yaml onto the window");
        } else {
            if (camIsCalibration) {
                ImGui::TextColored(kPartialColor, "Active: calibration result");
                if (loadedCam.loaded) {
                    ImGui::SameLine();
                    if (ImGui::SmallButton("Revert")) RevertCamera();
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Back to %s", cameraInfoPath.c_str());
                }
            } else {
                ImGui::Text("Active: %s", FileName(cameraInfoPath).c_str());
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", cameraInfoPath.c_str());
            }
            const CameraModelInfo& info = cam.Info();
            ImGui::Text("%s  %dx%d", info.name, cam.width, cam.height);
            ImGui::Text("fx=%.2f fy=%.2f", cam.params[kFx], cam.params[kFy]);
            ImGui::Text("cx=%.2f cy=%.2f", cam.params[kCx], cam.params[kCy]);
            // The model's own terms, three to a line.
            for (int k = 4; k < info.numParams; ++k) {
                if ((k - 4) % 3 != 0) ImGui::SameLine();
                ImGui::Text("%s=%.5g", info.paramNames[k], cam.params[k]);
            }
            std::string order;
            for (int k = info.distortionOffset; k < info.numParams; ++k)
                order += std::string(k > info.distortionOffset ? "," : "") + info.paramNames[k];
            ImGui::TextDisabled("(distortion order: %s)", order.c_str());

            ImGui::Separator();
            ImGui::Checkbox("Show reprojection overlay", &showReprojected);

            const ImageEntry* e = Current();
            if (!e) {
                ImGui::TextDisabled("no image selected");
            } else if (!e->verify.ok) {
                ImGui::TextColored(kWarnColor, "No pose: %s",
                                   e->detected ? e->verify.message.c_str() : "not detected yet");
            } else {
                const VerificationResult& v = e->verify;
                ImGui::Text("Reprojection error (%zu corners):", v.residuals.size());
                ImGui::TextColored(ToImVec4(ErrorColor(v.meanErrorPx)), "mean %.2f px", v.meanErrorPx);
                ImGui::TextColored(ToImVec4(ErrorColor(v.rmsErrorPx)), "rms  %.2f px", v.rmsErrorPx);
                ImGui::TextColored(ToImVec4(ErrorColor(v.maxErrorPx)), "max  %.2f px", v.maxErrorPx);
                ImGui::TextDisabled("blue crosshair = predicted corner");
                ImGui::TextDisabled("green/yellow/red = detected->predicted");
            }
        }
        ImGui::End();
    }

    void ImagesPanel() {
        ImGui::SetNextWindowPos(ImVec2((float)GetScreenWidth() - 390, 34), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(380, 420), ImGuiCond_FirstUseEver);
        ImGui::Begin("Images");

        const int n = (int)images.size();
        const int done = DetectedCount();
        if (detectAllRunning) {
            ImGui::ProgressBar(n ? (float)done / n : 1.f, ImVec2(-80, 0),
                               TextFormat("detecting %d/%d", done, n));
            ImGui::SameLine();
            if (ImGui::Button("Stop", ImVec2(-1, 0))) detectAllRunning = false;
        } else {
            ImGui::BeginDisabled(done == n);
            if (ImGui::Button("Detect all")) detectAllRunning = true;
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::Text("%d image(s), %d detected", n, done);
        }

        ImGui::BeginDisabled(current <= 0);
        if (ImGui::ArrowButton("##prev", ImGuiDir_Left)) Step(-1);
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(current < 0 || current >= n - 1);
        if (ImGui::ArrowButton("##next", ImGuiDir_Right)) Step(1);
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(current < 0);
        const bool removeCurrent = ImGui::Button("Remove");
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(n == 0);
        const bool clearAll = ImGui::Button("Clear all");
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(n == 0);
        if (ImGui::Button("Use all"))
            for (ImageEntry& e : images) e.useForCalibration = true;
        ImGui::SameLine();
        if (ImGui::Button("Use none"))
            for (ImageEntry& e : images) e.useForCalibration = false;
        ImGui::EndDisabled();

        int selectRequest = -1;
        const ImGuiTableFlags flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                      ImGuiTableFlags_SizingFixedFit;
        if (ImGui::BeginTable("images", 4, flags, ImVec2(0, -1))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("image", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("corners");
            ImGui::TableSetupColumn("rms px");
            ImGui::TableSetupColumn("use");
            ImGui::TableHeadersRow();

            ImGuiListClipper clipper;
            clipper.Begin(n);
            if (scrollListToCurrent && current >= 0) clipper.IncludeItemByIndex(current);
            while (clipper.Step()) {
                for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
                    ImageEntry& e = images[i];
                    ImGui::TableNextRow();
                    ImGui::PushID(i);

                    ImGui::TableSetColumnIndex(0);
                    if (e.loadFailed) ImGui::PushStyleColor(ImGuiCol_Text, kBadColor);
                    if (ImGui::Selectable(FileName(e.path).c_str(), i == current,
                                          ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap))
                        selectRequest = i;
                    if (e.loadFailed) ImGui::PopStyleColor();
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", e.path.c_str());
                    if (scrollListToCurrent && i == current) ImGui::SetScrollHereY();

                    ImGui::TableSetColumnIndex(1);
                    if (e.loadFailed) ImGui::TextColored(kBadColor, "failed");
                    else if (!e.detected) ImGui::TextDisabled("-");
                    else if (e.det.CornerCount() < kMinVerifyCorners) ImGui::TextDisabled("%d", e.det.CornerCount());
                    else
                        ImGui::TextColored(e.det.CornerCount() == boardState.totalCorners ? kGoodColor : kPartialColor,
                                           "%d", e.det.CornerCount());

                    ImGui::TableSetColumnIndex(2);
                    if (e.verify.ok)
                        ImGui::TextColored(ToImVec4(ErrorColor(e.verify.rmsErrorPx)), "%.2f", e.verify.rmsErrorPx);
                    else
                        ImGui::TextDisabled("-");

                    ImGui::TableSetColumnIndex(3);
                    ImGui::Checkbox("##use", &e.useForCalibration);
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Include in calibration");

                    ImGui::PopID();
                }
            }
            ImGui::EndTable();
        }
        scrollListToCurrent = false;
        ImGui::End();

        if (selectRequest >= 0 && selectRequest != current) Select(selectRequest);
        if (removeCurrent) Remove(current);
        if (clearAll) Clear();
    }

    void CalibrationPanel() {
        ImGui::SetNextWindowPos(ImVec2((float)GetScreenWidth() - 390, 464), ImGuiCond_FirstUseEver);
        // Fixed width, height following the content: the results section
        // only appears after the first run and would otherwise be clipped.
        ImGui::SetNextWindowSizeConstraints(ImVec2(380, 0), ImVec2(380, FLT_MAX));
        ImGui::Begin("Calibration", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

        int modelIndex = static_cast<int>(calibModel);
        ImGui::SetNextItemWidth(-60);
        if (ImGui::BeginCombo("Model", ModelInfo(calibModel).label)) {
            for (int m = 0; m < kCameraModelCount; ++m)
                if (ImGui::Selectable(ModelInfo(static_cast<CameraModel>(m)).label, m == modelIndex)) modelIndex = m;
            ImGui::EndCombo();
        }
        SetCalibrationModel(static_cast<CameraModel>(modelIndex));
        const CameraModelInfo& info = ModelInfo(calibModel);

        const int usable = (int)CalibrationCandidates().size();
        const int done = DetectedCount();
        ImGui::Text("%d image(s) selected with >= %d corners", usable, kMinVerifyCorners);
        if (done < (int)images.size())
            ImGui::TextColored(kWarnColor, "%d of %zu images not detected yet (Detect all)", (int)images.size() - done,
                               images.size());

        // Starting from the active camera needs one of the same model; show
        // "generic" when there's no such camera without overwriting the
        // choice for when there is.
        const bool canStartFromCamera = cam.loaded && cam.model == calibModel;
        bool generic = calibFromGeneric || !canStartFromCamera;
        ImGui::BeginDisabled(!canStartFromCamera);
        if (ImGui::Checkbox("Start from a generic guess", &generic)) calibFromGeneric = generic;
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("%s", canStartFromCamera ? "Instead of the active camera's intrinsics"
                                    : cam.loaded      ? "The active camera is a different model"
                                                      : "No camera_info.yaml loaded to start from");

        // One row per parameter: whether it's held, and — once this model
        // has been calibrated — the result next to where it started.
        const bool showResult = haveCalib && calib.ok && calib.camera.model == calibModel;
        if (ImGui::BeginTable("params", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("param");
            ImGui::TableSetupColumn("fix");
            ImGui::TableSetupColumn("result");
            ImGui::TableSetupColumn("1 sigma");
            ImGui::TableSetupColumn("start");
            ImGui::TableHeadersRow();
            for (int k = 0; k < info.numParams; ++k) {
                ImGui::PushID(k);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(info.paramNames[k]);
                ImGui::TableSetColumnIndex(1);
                ImGui::Checkbox("##fix", &calibOptions.fixed[k]);
                if (calibModel == CameraModel::Mei && k == kMeiXi && ImGui::IsItemHovered())
                    ImGui::SetTooltip("xi trades off against fx/fy and k1..k3 and is poorly\n"
                                      "constrained by a flat board; Insta360 pins it at 2");
                if (showResult) {
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%.6g", calib.camera.params[k]);
                    ImGui::TableSetColumnIndex(3);
                    if (calib.stdDev[k] > 0.0) ImGui::Text("%.2g", calib.stdDev[k]);
                    else ImGui::TextDisabled("fixed");
                    ImGui::TableSetColumnIndex(4);
                    ImGui::TextDisabled("%.6g", calibInitial.params[k]);
                }
                ImGui::PopID();
            }
            ImGui::EndTable();
        }

        ImGui::BeginDisabled(usable < kMinCalibrationViews || detectAllRunning);
        if (ImGui::Button("Calibrate", ImVec2(-1, 0))) RunCalibration();
        ImGui::EndDisabled();

        if (haveCalib) {
            ImGui::Separator();
            if (!calib.ok) {
                ImGui::TextColored(kBadColor, "Failed: %s", calib.message.c_str());
            } else {
                ImGui::TextWrapped("%s", calib.message.c_str());
                ImGui::TextColored(ToImVec4(ErrorColor(calib.rmsPx)), "rms %.3f px", calib.rmsPx);
                ImGui::SameLine();
                ImGui::TextDisabled("(start: %s, rms %.3f px)", calibStartLabel.c_str(), calib.initialRmsPx);

                if (calib.camera.model != calibModel &&
                    ImGui::BeginTable("results", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
                    // Result of a model other than the one now selected above.
                    ImGui::TableSetupColumn("param");
                    ImGui::TableSetupColumn("value");
                    ImGui::TableSetupColumn("1 sigma");
                    ImGui::TableHeadersRow();
                    for (int k = 0; k < calib.camera.NumParams(); ++k) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextUnformatted(calib.camera.Info().paramNames[k]);
                        ImGui::TableSetColumnIndex(1);
                        ImGui::Text("%.6g", calib.camera.params[k]);
                        ImGui::TableSetColumnIndex(2);
                        if (calib.stdDev[k] > 0.0) ImGui::Text("%.2g", calib.stdDev[k]);
                        else ImGui::TextDisabled("fixed");
                    }
                    ImGui::EndTable();
                }

                if (ImGui::Button("Use as active camera")) ApplyCalibration();
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Verify every image against these intrinsics\n(the images list rms updates)");

                ImGui::SetNextItemWidth(-60);
                if (ImGui::InputText("##savepath", savePath, sizeof(savePath))) overwriteArmed = false;
                ImGui::SameLine();
                if (ImGui::Button(overwriteArmed ? "Overwrite" : "Save", ImVec2(-1, 0))) SaveCalibration();
                if (ImGui::IsItemHovered()) {
                    if (calibTemplatePath.empty())
                        ImGui::SetTooltip("Writes a new camera_info.yaml");
                    else
                        ImGui::SetTooltip("Writes a copy of %s with the intrinsics replaced\n"
                                          "(serial, extrinsics and other fields kept)",
                                          FileName(calibTemplatePath).c_str());
                }
            }
        }
        ImGui::End();
    }
};

} // namespace

int main(int argc, char** argv) {
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(1280, 960, "calib_app - camera calibration");
    SetTargetFPS(60);
    rlImGuiSetup(true);

    App app;
    std::vector<std::string> args(argv + 1, argv + argc);
    if (args.empty())
        std::printf("calib_app: nothing given — drop images, a capture folder, a camera_info.yaml and/or a meta.txt "
                    "onto the window, or pass them as arguments\n");
    else
        app.LoadPaths(args);

    while (!WindowShouldClose()) {
        app.DetectAllStep();

        BeginDrawing();
        ClearBackground(DARKGRAY);

        rlImGuiBegin();
        const ImGuiIO& io = ImGui::GetIO();
        const bool blockMouse    = io.WantCaptureMouse;
        const bool blockKeyboard = io.WantCaptureKeyboard;

        if (IsFileDropped()) {
            FilePathList files = LoadDroppedFiles();
            std::vector<std::string> paths(files.paths, files.paths + files.count);
            UnloadDroppedFiles(files);
            app.LoadPaths(paths);
        }

        app.HandleInput(blockMouse, blockKeyboard);
        app.DrawImageAndOverlays();
        app.DrawHud();
        app.BoardPanel();
        app.VerificationPanel();
        app.ImagesPanel();
        app.CalibrationPanel();

        rlImGuiEnd();
        EndDrawing();
    }

    rlImGuiShutdown();
    if (app.texLoaded) UnloadTexture(app.tex);
    CloseWindow();
    return 0;
}
