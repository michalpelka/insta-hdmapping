#pragma once

// OpenCV's ArUco/ChArUco API changed shape twice, and the two CI runners land
// on opposite sides of the split:
//   - Ubuntu's apt-packaged OpenCV (4.6.0, as of this writing) predates the
//     4.7 rewrite: aruco lived in its own contrib module named "aruco", board
//     and dictionary factories were `cv::Ptr<T> T::create(...)` / free
//     functions returning `Ptr<Dictionary>`, and detection went through free
//     functions taking a `cv::Ptr<DetectorParameters>`.
//   - Homebrew's OpenCV (5.0+) is well past it: aruco was folded into the
//     main "objdetect" module (no separate "aruco" component to find_package
//     any more), factories became plain value-type constructors, and
//     detection is done via the ArucoDetector/CharucoDetector classes. The
//     old API is gone entirely -- there is no compat shim for it.
// Everything that differs is confined behind INSTA360_CALIB_NEW_ARUCO_API so
// one set of source files builds against either.
#include <opencv2/core/version.hpp>

#define INSTA360_CALIB_NEW_ARUCO_API \
    (CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7))

#if INSTA360_CALIB_NEW_ARUCO_API
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
namespace cv {
namespace aruco {
using DictionaryName = PredefinedDictionaryType;
} // namespace aruco
} // namespace cv
#else
#include <opencv2/aruco/charuco.hpp>
namespace cv {
namespace aruco {
using DictionaryName = PREDEFINED_DICTIONARY_NAME;
} // namespace aruco
} // namespace cv
#endif
