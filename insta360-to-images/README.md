# insta360-to-images

A C++17 rewrite of [InstaToBag](https://github.com/michalpelka/InstaToBag) that
converts an Insta360 `.insv` capture into **per-camera folders of timestamped JPEG
frames** instead of a ROS 2 MCAP bag: both fisheye tracks, each frame named by its
nanosecond timestamp on the camera's own clock, plus an optional 1 kHz IMU CSV and a
per-lens intrinsics sidecar. No third-party libraries: ffmpeg does the decoding, and
everything else is parsed by hand.

Developed and verified against an **Insta360 X5** (firmware `v1.10.11_build1`, 5.7K
dual-fisheye, 2880x2880 per lens at 24 fps) and an **Insta360 X6** (firmware
`v1.3.59`, 3840x3840 per lens at 29.97 fps) -- see [Insta360 X6](#insta360-x6). The
trailer format is shared across the X-series, so ONE X2/X3/X4 files should work; the
parsers degrade to "field not present" rather than to wrong values when a firmware
revision differs.

## Build

Requires a C++17 compiler, CMake >= 3.16, and `ffmpeg`/`ffprobe` on `PATH` at
*runtime* (not needed to build):

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

This produces `build/insta360-to-images` and, unless you pass
`-DINSTA360_BUILD_TESTS=OFF`, a unit test binary run via `ctest`:

```bash
ctest --test-dir build --output-on-failure
```

## Use

```bash
# See what's in a file without writing anything
insta360-to-images VID_20260909_132827_00_005.insv --inspect

# Convert (writes VID_20260909_132827_00_005/ beside the input)
insta360-to-images VID_20260909_132827_00_005.insv

# A quick look at a long clip, at a quarter resolution
insta360-to-images capture.insv -o small/ --max-frames 100 --scale 0.25
```

Run `insta360-to-images --help` for the full set. The useful ones:

| Flag | Effect |
| --- | --- |
| `--jpeg-quality N` | ffmpeg MJPEG quality, 2 (best) to 31. Default 3. |
| `--scale SPEC` | `0.5` or `1440x1440`. Intrinsics are rescaled to match. |
| `--max-frames N` | Stop after N frames. |
| `--frame-step N` | Export only every Nth frame (1 = all, the default). Counts source frames, so `--max-frames 100 --frame-step 5` writes 20 images per lens. |
| `--swap-lenses` | Map the second video track to `cam_front`. |
| `--no-imu` / `--no-camera-info` | Leave out `imu.csv` / the intrinsics sidecar. |
| `--relative-time` | Start timestamps at zero instead of the capture wall clock. |
| `--time-offset SECONDS` | Shift every written timestamp by SECONDS (fractional and negative both accepted), to align the clip against another clock. |
| `--align-imus DIR` | Shift every timestamp onto an external IMU's clock (e.g. a Mandeye lidar logger), found by cross-correlating gyro magnitudes against `DIR/imu*.csv`. Added to any `--time-offset`; needs the camera wall clock, so not with `--relative-time`. |
| `--align-max-lag SECONDS` | Largest clock offset `--align-imus` searches, either direction. Default 60. |
| `-f`, `--force` | Write into a non-empty output directory. |

## Output layout

```
<output>/
  cam_front/
    camera_info.yaml              # once, if lens calibration was present
    front_1736432112123456789.jpg # one per frame, named by its epoch nanosecond timestamp
    front_1736432112165123456.jpg
    ...
  cam_back/
    camera_info.yaml
    back_<timestamp_ns>.jpg
    ...
  imu.csv                         # timestamp_ns,accel_x_mps2,...,gyro_z_radps at ~1 kHz
```

With `--relative-time`, filenames and `imu.csv` timestamps count nanoseconds from the
start of the clip instead of epoch nanoseconds.

`--time-offset` adds a fixed shift on top of whichever of those two time bases is in
use, and applies to everything stamped with a time: the frame filenames and the
`imu.csv` rows move together, so the streams stay aligned with each other. Use it when the camera's own clock is known to be off
against whatever else you are logging.

`--frame-step N` thins the export to every Nth frame -- `--frame-step 5` on a 30 fps
clip leaves 6 images per second per lens. The kept frames keep their real timestamps, so the
filenames stay directly comparable to a full export; `imu.csv` is never thinned.

### `camera_info.yaml`

One per camera, written once, holding the same intrinsics the original tool put on
`sensor_msgs/CameraInfo` (`fx`/`fy`/`cx`/`cy`, the 3x3 `k`, 3x3 `r` -- identity, since
no stereo rectification is defined for a back-to-back fisheye pair -- and the 3x4
`p`), plus the fields a plain `CameraInfo` message has no room for: `xi` (the Mei
model's mirror parameter) and `distortion` (`k1`, `k2`, `k3`, `p1`, `p2`, under
`distortion_model: "insta360_mei_v2"`), and the per-lens `rotation_deg`/`translation`
the camera reports (not a calibrated IMU-to-camera extrinsic). It also carries the
camera body's identity as quoted strings -- `serial`, `model` and `firmware`, each
`null` on a file whose metadata record omits it -- the same three values the run
banner prints, repeated in every camera's sidecar so a directory of frames identifies
the body and firmware it came off without the source `.insv`. Note `k`/`p` are the
ordinary pinhole matrices built from `fx`/`fy`/`cx`/`cy` alone -- they do *not*
capture `xi` or the distortion, so they're only a rough pinhole approximation near
the lens center, not something to feed `cv::undistort`.

`calibration_source` names the metadata string the values came from: `offset_v2` on
an X5, `offset_v3` on an X6. For `offset_v3` three more keys follow --
`source_distortion` (that string's 13 distortion coefficients verbatim),
`refit_max_error_px` and `refit_max_theta_deg` -- see [Insta360 X6](#insta360-x6).

#### The lens model

The embedded calibration's `fx`/`fy`/`cx`/`cy` plus distortion coefficients do not
fit any *documented* fisheye model closely enough to project with -- plugging its
numbers into the equidistant, equisolid, stereographic, or a normalized-domain
polynomial model all give a field of view around 35-40 degrees for a lens that
visibly covers most of a hemisphere. The actual model is the **Mei/Barreto unified
spherical camera model**: project onto a unit sphere, offset by a mirror parameter
`xi`, perspective-project, then apply ordinary radial + tangential distortion on that
plane (`calibration.hpp` has the exact formula). `xi = 2.0` for both lenses on an X5,
`2.455` on an X6. Cross-referenced against an independent reverse-engineering effort
([BenjaminHenriksson/insv-stitch](https://github.com/BenjaminHenriksson/insv-stitch))
that landed on the same model and field layout from different footage.

## Insta360 X6

An X6 file has the same trailer and records as an X5; two things in its metadata
differ, and both are handled:

- **No clock sub-message (field 98).** The X5 keeps the frame count, first-frame
  time and its epoch there. The X6 writes the frame count as field 40 instead, and
  the first frame's device time and epoch only as the standalone fields 24 and 36,
  which the X5 also writes. On the X5, field 36 runs 64-93 ms *ahead* of the clock
  sub-message's epoch, and the gap varies per file; whether the X6's field 36 has
  the same lag can't be told from the file. So X6 filenames may sit up to ~0.1 s
  early against true UTC. `--align-imus` measures the real offset against your
  logger, so it absorbs this. Frame-to-frame and frame-to-IMU timing come from the
  exposure record as before and are unaffected.
- **Only `offset_v3` calibration (field 111).** The X5 writes the 19-value-per-lens
  `offset_v2` and this newer 27-value `offset_v3` side by side; the X6 writes only
  the latter. It is the same Mei model with a fourth radial term (`k4 * r^8`) and
  seven more coefficients. One of those has read 0 on every lens seen so far; the
  other six could not be identified -- read as OpenCV thin-prism or sensor-tilt terms
  they make the fit *worse* -- so they are not applied.

How `offset_v3` was pinned down: ChArUco corners detected in real footage, the board
pose fitted per frame, every intrinsic held at the factory value. On the X6 (board
within 52 degrees of the axis) that gives 0.43-0.88 px RMS. On the X5 back lens
(board out to 120 degrees) it matches or beats the X5's own `offset_v2` in every
angular band.

Everything downstream of `camera_info.yaml` reads `insta360_mei_v2`'s five
coefficients, so an `offset_v3` lens is written in that form: `p1`/`p2` copied
as-is, `k1`-`k3` refitted by least squares to the four-term radial curve over the
lens' whole image circle (out to where it meets the frame edge, ~101 degrees on an
X6). The refit stays within 0.42 px (front) / 0.13 px (back) of the k1-k4 curve at
3840x3840, recorded per lens as `refit_max_error_px`, and fits the X6 board corners
exactly as well as the k1-k4 original (0.866 vs 0.853 px RMS). `--inspect` prints
both the refit and the raw coefficients. Files carrying both strings (the X5) keep
using `offset_v2`, so X5 output is unchanged.

## Caveats — read before you use the data

Carried over unchanged from the original tool; nothing about the trailer format or
its ambiguities changed in this rewrite.

- **IMU axes are the sensor's own, not REP 103.** Accelerometer and gyroscope are
  converted to m/s² and rad/s, but they are *not* rotated into ROS's x-forward /
  y-left / z-up convention, because the mapping from the X-series sensor frame to the
  camera body frame is undocumented. Determine it for your rig before fusing.
- **No IMU-to-camera extrinsics.** The metadata carries per-lens rotations, but not a
  calibrated IMU-to-camera transform. The per-lens rotation and translation values are
  passed through verbatim in `camera_info.yaml`.
- **`camera_info.yaml`'s `k`/`p` are plain pinhole, not the real lens model.**
  `distortion_model` is `insta360_mei_v2` (Mei/Barreto unified spherical model, `xi`
  plus radial/tangential terms -- see [The lens model](#the-lens-model)). `k`/`p`
  in that same file are ordinary pinhole
  matrices built from `fx`/`fy`/`cx`/`cy` alone and must not be fed to
  `cv::undistort`/`cv::fisheye` as if they captured the real distortion -- they don't.
- **`cam_front` / `cam_back` is naming, not a determination.** They follow the order
  of the video tracks in the file, which is stable but has not been confirmed against
  which lens physically faces the screen. Use `--swap-lenses` if you need them the
  other way round.
- **JPEG re-encode is lossy.** Frames are decoded from HEVC and re-encoded as JPEG.
  Lower `--jpeg-quality` for more fidelity, or keep the original file for archival.
- **IMU covariances are unknown** and are simply not written; treat `imu.csv` as raw
  sensor readings, not a calibrated `sensor_msgs/Imu`-equivalent.
- **X6 timestamps may be up to ~0.1 s early against UTC** -- see
  [Insta360 X6](#insta360-x6). Use `--align-imus` when that matters.

## The `.insv` format

Unchanged from the original project; see its
[README](https://github.com/michalpelka/InstaToBag#the-insv-format) for the full
trailer layout, timestamp derivation, and how the IMU axis/scale assignment was
physically validated. This rewrite reads the identical byte layout.

## Layout

| File | Role |
| --- | --- |
| `include/insta360/trailer.hpp`, `src/trailer.cpp` | Trailer tail, record directory, random access to records |
| `include/insta360/protobuf.hpp`, `src/protobuf.cpp` | Dependency-free protobuf wire decoder |
| `include/insta360/metadata.hpp`, `src/metadata.cpp` | Typed view over record `0x0101`, including the wall-clock mapping |
| `include/insta360/calibration.hpp`, `src/calibration.cpp` | Lens calibration strings (`offset_v2`/`offset_v3`) -> rescaled intrinsics |
| `include/insta360/sensors.hpp`, `src/sensors.cpp` | IMU, exposure and frame-timestamp decoding |
| `include/insta360/subprocess.hpp`, `src/subprocess.cpp` | `posix_spawn`-based child process wrapper (no shell) |
| `include/insta360/media.hpp`, `src/media.cpp` | ffprobe/ffmpeg piping, JPEG frame splitting |
| `include/insta360/convert.hpp`, `src/convert.cpp` | Merges every stream into timestamped files on disk |
| `src/cli.cpp` | Argument parsing, `--inspect`, `main` |
| `tests/` | Unit tests against byte-exact synthetic trailers and protobuf messages |

## License

MIT

## Trademarks

This is an independent project, not affiliated with, endorsed by, or sponsored by
Arashi Vision Inc. "Insta360" and the product names above are trademarks of their
respective owners, used here only to identify the file format and hardware this tool
interoperates with. No Insta360 source code, SDK or firmware is included: the trailer
format was derived (in the original Python project this is ported from) by observing
captures from a camera its author owns.
