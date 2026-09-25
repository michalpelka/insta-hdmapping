#include "Camera.h"
#include "ImageSet.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "test_cameras.hpp"
#include "test_util.hpp"

namespace fs = std::filesystem;

namespace {

fs::path FreshDir(const std::string& name) {
    const fs::path dir = fs::temp_directory_path() / name;
    fs::remove_all(dir);
    fs::create_directories(dir);
    return dir;
}

std::string ReadAll(const fs::path& p) {
    std::ifstream f(p);
    std::stringstream s;
    s << f.rdbuf();
    return s.str();
}

void WriteText(const fs::path& p, const std::string& text) { std::ofstream(p) << text; }

// Shaped like insta360-to-images' write_camera_info() output, with a serial
// that only stays a string while it keeps its quotes.
const char* kTemplate =
    "frame_id: insta360_cam_front_optical_frame\n"
    "serial: \"0123\"\n"
    "model: null\n"
    "firmware: \"v1.2\"\n"
    "video_stream_index: 0\n"
    "width: 2880\n"
    "height: 2880\n"
    "distortion_model: insta360_mei_v2\n"
    "xi: 2\n"
    "fx: 2303.378571\n"
    "fy: 2303.073214\n"
    "cx: 1444.832143\n"
    "cy: 1436.678571\n"
    "distortion: [0.18967018, 2.06612277, -3.31555128, 0.00052395, 3.87e-05]\n"
    "k: [2303.378571, 0, 1444.832143, 0, 2303.073214, 1436.678571, 0, 0, 1]\n"
    "r: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n"
    "p: [2303.378571, 0, 1444.832143, 0, 0, 2303.073214, 1436.678571, 0, 0, 0, 1, 0]\n"
    "rotation_deg: [0.196, -0.027, 89.717]\n"
    "translation: [0, 0, 0]\n";

CameraIntrinsics Calibrated() {
    CameraIntrinsics cam = test_cameras::Make(CameraModel::Mei, 2880, 2880,
                                    {2301.25, 2300.5, 1443.125, 1437.75, 2.0, 0.2, 2.0, -3.25, 0.0005, -0.0001});
    cam.frameId = "insta360_cam_front_optical_frame";
    return cam;
}

void CheckIntrinsicsEqual(const CameraIntrinsics& a, const CameraIntrinsics& b) {
    CHECK(a.loaded);
    CHECK(a.model == b.model);
    CHECK_EQ(a.width, b.width);
    CHECK_EQ(a.height, b.height);
    for (int k = 0; k < kMaxCameraParams; ++k) CHECK_NEAR(a.params[k], b.params[k], 1e-9);
}

} // namespace

TEST(save_camera_info_replaces_intrinsics_and_keeps_every_other_template_line) {
    const fs::path dir = FreshDir("calib_app_io_test_template");
    WriteText(dir / "camera_info.yaml", kTemplate);
    const CameraIntrinsics cam = Calibrated();

    std::string error;
    CHECK(SaveCamera((dir / "out.yaml").string(), cam, (dir / "camera_info.yaml").string(), "test run",
                        &error));
    CheckIntrinsicsEqual(LoadCamera((dir / "out.yaml").string()), cam);

    const std::string text = ReadAll(dir / "out.yaml");
    CHECK(text.rfind("# calib_app: test run\n", 0) == 0);
    for (const char* kept : {"serial: \"0123\"\n", "model: null\n", "firmware: \"v1.2\"\n", "video_stream_index: 0\n",
                             "r: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n", "rotation_deg: [0.196, -0.027, 89.717]\n",
                             "translation: [0, 0, 0]\n"})
        CHECK(text.find(kept) != std::string::npos);
    CHECK(text.find("fx: 2301.25\n") != std::string::npos);
    CHECK(text.find("k: [2301.25, 0, 1443.125, 0, 2300.5, 1437.75, 0, 0, 1]\n") != std::string::npos);
    CHECK(text.find("2303.378571") == std::string::npos); // no stale intrinsics left behind

    // Saving over a file that already carries a calib_app comment replaces
    // it instead of stacking another.
    CHECK(SaveCamera((dir / "again.yaml").string(), cam, (dir / "out.yaml").string(), "second run", &error));
    const std::string again = ReadAll(dir / "again.yaml");
    CHECK(again.find("test run") == std::string::npos);
    CHECK(again.find("# calib_app: second run\n") != std::string::npos);
}

TEST(save_camera_info_without_a_template_writes_a_loadable_file) {
    const fs::path dir = FreshDir("calib_app_io_test_fresh");
    const CameraIntrinsics cam = Calibrated();
    std::string error;
    CHECK(SaveCamera((dir / "out.yaml").string(), cam, "", "", &error));
    const CameraIntrinsics back = LoadCamera((dir / "out.yaml").string());
    CheckIntrinsicsEqual(back, cam);
    CHECK_EQ(back.frameId, cam.frameId);
}

TEST(every_model_round_trips_through_camera_info_yaml) {
    const fs::path dir = FreshDir("calib_app_io_test_models");
    for (const CameraIntrinsics& cam :
         {test_cameras::Rig(), test_cameras::Pinhole(), test_cameras::Rational(), test_cameras::Fisheye()}) {
        const fs::path path = dir / (std::string(cam.Info().name) + ".yaml");
        std::string error;
        CHECK(SaveCamera(path.string(), cam, "", "", &error));
        CheckIntrinsicsEqual(LoadCamera(path.string()), cam);
        const std::string text = ReadAll(path);
        CHECK(text.find(std::string("distortion_model: ") + cam.Info().name + "\n") != std::string::npos);
        CHECK((text.find("xi:") != std::string::npos) == (cam.model == CameraModel::Mei));
    }
    // cv::calibrateCamera's distortion order, not the Mei file's.
    CHECK(ReadAll(dir / "plumb_bob.yaml").find("distortion: [-0.28, 0.09, 0.0008, -0.0005, -0.012]\n") !=
          std::string::npos);
}

TEST(saving_an_opencv_model_over_a_mei_template_switches_the_model_and_drops_xi) {
    const fs::path dir = FreshDir("calib_app_io_test_switch");
    WriteText(dir / "camera_info.yaml", kTemplate);
    CameraIntrinsics cam = test_cameras::Fisheye();
    std::string error;
    CHECK(SaveCamera((dir / "out.yaml").string(), cam, (dir / "camera_info.yaml").string(), "", &error));
    CheckIntrinsicsEqual(LoadCamera((dir / "out.yaml").string()), cam);
    const std::string text = ReadAll(dir / "out.yaml");
    CHECK(text.find("distortion_model: equidistant\n") != std::string::npos);
    CHECK(text.find("xi:") == std::string::npos);
    CHECK(text.find("serial: \"0123\"\n") != std::string::npos);
    CHECK(text.find("rotation_deg: [0.196, -0.027, 89.717]\n") != std::string::npos);
}

TEST(load_camera_info_rejects_an_unknown_model_and_accepts_the_fisheye_alias) {
    const fs::path dir = FreshDir("calib_app_io_test_load");
    const std::string body = "width: 640\nheight: 480\nfx: 500\nfy: 500\ncx: 320\ncy: 240\n";
    WriteText(dir / "unknown.yaml", "distortion_model: scaramuzza\n" + body + "distortion: [0, 0, 0, 0]\n");
    CHECK(!LoadCamera((dir / "unknown.yaml").string()).loaded);

    WriteText(dir / "fisheye.yaml", "distortion_model: fisheye\n" + body + "distortion: [0.1, 0.01, 0, 0]\n");
    const CameraIntrinsics fish = LoadCamera((dir / "fisheye.yaml").string());
    CHECK(fish.loaded);
    CHECK(fish.model == CameraModel::Equidistant);
    CHECK_EQ(fish.params[4], 0.1);

    // Older files without distortion_model but with xi are the rig's.
    WriteText(dir / "old.yaml", body + "xi: 2\ndistortion: [0.1, 0, 0, 0, 0]\n");
    const CameraIntrinsics old = LoadCamera((dir / "old.yaml").string());
    CHECK(old.loaded);
    CHECK(old.model == CameraModel::Mei);
    CHECK_EQ(old.params[kMeiXi], 2.0);
}

TEST(save_camera_info_fails_on_a_missing_template_without_writing) {
    const fs::path dir = FreshDir("calib_app_io_test_missing");
    std::string error;
    CHECK(!SaveCamera((dir / "out.yaml").string(), Calibrated(), (dir / "nope.yaml").string(), "", &error));
    CHECK(!error.empty());
    CHECK(!fs::exists(dir / "out.yaml"));
}

TEST(expand_input_path_lists_a_folders_images_sorted_after_its_camera_info) {
    const fs::path dir = FreshDir("calib_app_io_test_expand");
    for (const char* name : {"front_3.jpg", "front_1.jpg", "front_2.PNG", "camera_info.yaml",
                             "camera_info_calibrated.yaml", "notes.txt", "imu.csv"})
        WriteText(dir / name, "x");
    fs::create_directories(dir / "sub");
    WriteText(dir / "sub" / "front_0.jpg", "x");

    const std::vector<std::string> got = ExpandInputPath(dir.string());
    CHECK_EQ(got.size(), (size_t)4);
    if (got.size() == 4) {
        CHECK_EQ(fs::path(got[0]).filename().string(), std::string("camera_info.yaml"));
        CHECK_EQ(fs::path(got[1]).filename().string(), std::string("front_1.jpg"));
        CHECK_EQ(fs::path(got[2]).filename().string(), std::string("front_2.PNG"));
        CHECK_EQ(fs::path(got[3]).filename().string(), std::string("front_3.jpg"));
    }

    // A plain file passes through untouched, whatever it is.
    CHECK(ExpandInputPath((dir / "notes.txt").string()) == std::vector<std::string>{(dir / "notes.txt").string()});
}
