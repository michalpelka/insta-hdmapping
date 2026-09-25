#pragma once

// The set of images loaded into the app. Only paths and per-image results
// live here; decoding happens on demand, so a folder of hundreds of 2880x2880
// frames never has to fit in memory at once.

#include "Board.h"
#include "Verification.h"

#include <string>
#include <vector>

struct ImageEntry {
    std::string path;
    int width = 0, height = 0; // known once it has been decoded
    bool loadFailed = false;   // cv::imread couldn't decode it
    bool detected = false;     // det/verify are current for the applied board
    bool useForCalibration = true;

    Detection det;
    VerificationResult verify; // against the active camera
};

// Lower-cased extension without the dot ("" if none).
std::string ExtLower(const std::string& path);
bool IsImageExt(const std::string& ext);

// Expands one path given on the command line or dropped on the window into
// the files to load from it. A directory contributes its images (sorted by
// name, not recursive) and, if it has one, its camera_info.yaml — the layout
// insta360-to-images writes per camera. Anything else is returned as-is for
// the caller to classify. Paths come back absolute and normalized.
std::vector<std::string> ExpandInputPath(const std::string& input);
