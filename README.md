# insta-hdmapping

Tools for turning Insta360 dual-fisheye captures into calibrated, per-camera image
data for HD mapping.

| Folder | What it is |
| --- | --- |
| [`insta360-to-images`](insta360-to-images/) | C++17 tool: unpacks an Insta360 `.insv` capture into timestamped per-camera JPEG frames, IMU CSV, lens intrinsics, and an optional geometric equirectangular stitch. |
| [`insta360-test-calib`](insta360-test-calib/) | C++ GUI app (raylib/Dear ImGui) for verifying the fisheye lens calibration against a captured ChArUco board. |
| [`cahruco`](cahruco/) | Python script that generates a pixel-exact ChArUco calibration board sized to fill a 4K screen, for use with `insta360-test-calib`. |

Each subproject has its own build instructions and README where applicable; see
`insta360-to-images/README.md` for the most detailed one.

## CI

Each subproject builds on Linux and macOS via GitHub Actions (`.github/workflows/`),
triggered only when its own files change.
