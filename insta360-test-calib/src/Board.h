#pragma once

// ChArUco board description, construction and detection — everything the app
// needs to know about the calibration target, independent of the UI.

#include "ArucoCompat.h"

#include <opencv2/core.hpp>

#include <string>
#include <vector>

// Board dictionaries offered in the UI, same set as
// /home/michal/fisheye/files(1)/show_charuco_4k.py's DICT_NAMES.
inline constexpr const char* kDictNames[] = {"4X4_50", "5X5_100", "6X6_250", "7X7_1000", "APRILTAG_36h11"};
inline constexpr int kDictCount = 5;

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

// Keeps sanitized values within workable ranges (CharucoBoard construction
// requires markerLength < squareLength, and degenerate small boards have no
// interior corners to detect).
BoardSettings Sanitize(BoardSettings s);

// Parses a show_charuco_4k.py-style meta.txt (key=value lines: cols, rows,
// dict, square_px, marker_px, plus screen_w/screen_h/margin which this app
// has no use for). Starts from the board's current settings and only
// overwrites keys actually present, so a partial/hand-edited file degrades
// gracefully instead of zeroing everything else out.
bool LoadBoardMetaTxt(const std::string& path, BoardSettings& out);

struct BoardState {
    cv::Ptr<cv::aruco::Dictionary> dict;
    cv::Ptr<cv::aruco::CharucoBoard> board;
    int totalCorners = 0; // interior chessboard corners == (cols-1)*(rows-1)
};

BoardState BuildBoard(const BoardSettings& s);

// Board-frame positions of the interior chessboard corners, indexed by
// charuco corner id. Board units are the layout's pixels (squarePx), which is
// fine for calibration: intrinsics don't depend on the board's scale.
std::vector<cv::Point3f> BoardCorners(const cv::aruco::CharucoBoard& board);

struct Detection {
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::Mat charucoCorners; // Nx1 CV_32FC2, empty if none found
    cv::Mat charucoIds;     // Nx1 CV_32SC1

    int CornerCount() const { return charucoIds.empty() ? 0 : charucoIds.rows; }
};

Detection DetectCharuco(const cv::Mat& bgr, const BoardState& b);
