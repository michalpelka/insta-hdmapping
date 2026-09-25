#include "Board.h"

#include <algorithm>
#include <cstdio>
#include <fstream>

namespace {

// Parallel to kDictNames (Board.h) — keep the two in the same order.
constexpr cv::aruco::DictionaryName kDictEnums[] = {
    cv::aruco::DICT_4X4_50, cv::aruco::DICT_5X5_100, cv::aruco::DICT_6X6_250,
    cv::aruco::DICT_7X7_1000, cv::aruco::DICT_APRILTAG_36h11,
};
static_assert(sizeof(kDictEnums) / sizeof(kDictEnums[0]) == kDictCount, "kDictEnums out of sync with kDictNames");

} // namespace

BoardSettings Sanitize(BoardSettings s) {
    s.dictIndex = std::clamp(s.dictIndex, 0, kDictCount - 1);
    s.cols      = std::clamp(s.cols, 3, 30);
    s.rows      = std::clamp(s.rows, 3, 30);
    s.squarePx  = std::clamp(s.squarePx, 20, 4000);
    s.markerPx  = std::clamp(s.markerPx, 10, s.squarePx - 1);
    return s;
}

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

BoardState BuildBoard(const BoardSettings& s) {
    BoardState b;
#if INSTA360_CALIB_NEW_ARUCO_API
    b.dict = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(kDictEnums[s.dictIndex]));
    b.board = cv::makePtr<cv::aruco::CharucoBoard>(cv::Size(s.cols, s.rows), (float)s.squarePx,
                                                    (float)s.markerPx, *b.dict);
#else
    b.dict = cv::aruco::getPredefinedDictionary(kDictEnums[s.dictIndex]);
    b.board = cv::aruco::CharucoBoard::create(s.cols, s.rows, (float)s.squarePx, (float)s.markerPx, b.dict);
#endif
    b.totalCorners = (s.cols - 1) * (s.rows - 1);
    return b;
}

std::vector<cv::Point3f> BoardCorners(const cv::aruco::CharucoBoard& board) {
    // CharucoBoard::chessboardCorners is a public field pre-4.7, and the
    // getChessboardCorners() accessor from 4.7 on -- see ArucoCompat.h.
#if INSTA360_CALIB_NEW_ARUCO_API
    return board.getChessboardCorners();
#else
    return board.chessboardCorners;
#endif
}

Detection DetectCharuco(const cv::Mat& bgr, const BoardState& b) {
    Detection d;
#if INSTA360_CALIB_NEW_ARUCO_API
    cv::aruco::DetectorParameters params;
    params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    // Fisheye-distorted squares curve a lot near the image edge; widen the
    // adaptive-threshold window range so markers there still binarize cleanly.
    params.adaptiveThreshWinSizeMin = 5;
    params.adaptiveThreshWinSizeMax = 35;
    params.adaptiveThreshWinSizeStep = 5;

    cv::aruco::ArucoDetector detector(*b.dict, params);
    std::vector<std::vector<cv::Point2f>> rejected;
    detector.detectMarkers(bgr, d.markerCorners, d.markerIds, rejected);

    if (!d.markerIds.empty()) {
        cv::aruco::CharucoDetector charucoDetector(*b.board);
        charucoDetector.detectBoard(bgr, d.charucoCorners, d.charucoIds, d.markerCorners, d.markerIds);
    }
#else
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
#endif
    return d;
}
