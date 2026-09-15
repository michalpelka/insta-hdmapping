# insta360-to-images

A C++17 rewrite of [InstaToBag](https://github.com/michalpelka/InstaToBag) that
converts an Insta360 `.insv` capture into **per-camera folders of timestamped JPEG
frames** instead of a ROS 2 MCAP bag: both fisheye tracks, each frame named by its
nanosecond timestamp on the camera's own clock, plus an optional 1 kHz IMU CSV,
per-lens intrinsics sidecar, the camera's own stitched equirectangular preview, and
a real per-frame geometric equirectangular stitch computed from the camera's own
lens calibration -- see [Equirect video](#equirect) below. Uses OpenCV for
exactly two things: decoding the firmware preview's pixel format, and the stitch's
projection/blending math.

Developed and verified against an **Insta360 X5** (firmware `v1.10.11_build1`, 5.7K
dual-fisheye, 2880x2880 per lens at 24 fps). The trailer format is shared across the
X-series, so ONE X2/X3/X4 files should work; the parsers degrade to "field not
present" rather than to wrong values when a firmware revision differs.

## Build

Requires a C++17 compiler, CMake >= 3.16, OpenCV (`core`, `imgproc`, `imgcodecs` --
e.g. `brew install opencv` or `apt install libopencv-dev`), and `ffmpeg`/`ffprobe` on
`PATH` at *runtime* (not needed to build):

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
| `--swap-lenses` | Map the second video track to `cam_front`. |
| `--no-imu` / `--no-camera-info` / `--no-panorama` | Leave out `imu.csv` / the intrinsics sidecar / `panorama.jpg`. |
| `--no-equirect` | Skip stitching the geometric per-frame equirect video (on by default, compute-heavy). |
| `--equirect-width N` | Equirect output width; height is always `N/2`. Default 3840. |
| `--no-equirect-flip` | Don't rotate the equirect output 180 degrees; row 0 becomes the north pole instead of the south pole. The flip is on by default and, when applied, is a full rotation, not a mirror flip -- see below. |
| `--relative-time` | Start timestamps at zero instead of the capture wall clock. |
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
  panorama.jpg                    # the camera's own firmware-stitched equirect preview
  equirect/                       # unless --no-equirect
    equirectangular_<timestamp_ns>.jpg # one geometrically stitched equirect frame per video frame
```

With `--relative-time`, filenames and `imu.csv` timestamps count nanoseconds from the
start of the clip instead of epoch nanoseconds.

### `panorama.jpg`

This one is **not** a panorama this tool computes. The trailer already carries a
low-resolution (typically 1280x640) equirectangular thumbnail that the camera's own
firmware stitched from both lenses at capture time (trailer record `0x0002`); this
tool just decodes its NV12 pixel format via `cv::cvtColor` and writes it out as a
JPEG via `cv::imwrite`.

### `equirect/`

By default this tool computes a real per-frame geometric equirectangular stitch, at
`--equirect-width` resolution (default 3840x1920), one JPEG per video frame, named
`equirectangular_<timestamp_ns>.jpg`; pass `--no-equirect` to skip it (it's
compute-heavy). Row 0 is the south pole by default (the 180-degree flip, see below,
is applied unless you pass `--no-equirect-flip`); pass `--no-equirect-flip` if your
consumer expects row 0 at the north pole (straight up) instead. The flip does a full
180-degree rotation (`cv::ROTATE_180`), not a single-axis flip: negating only
latitude (or only longitude) is a mirror reflection that reverses the scene's
handedness -- panning left vs. right would feel backwards in a viewer. Rotating both
axes swaps the pole correctly; the resulting longitude shift is invisible since the
image wraps.

Getting here took ruling out the obvious approach first: the embedded `offset_v2`
lens calibration's `fx`/`fy`/`cx`/`cy` plus five distortion coefficients do not fit
any *documented* fisheye model closely enough to project with -- plugging its numbers
into the equidistant, equisolid, stereographic, or a normalized-domain polynomial
model all give a field of view around 35-40 degrees for a lens that visibly covers
most of a hemisphere. The actual model turned out to be the **Mei/Barreto unified
spherical camera model** (project onto a unit sphere, offset by a mirror parameter
`xi`, then perspective-project, then apply ordinary radial + tangential distortion on
that plane) with `xi = 2.0` for both lenses on an X5 -- confirmed by cross-referencing
an independent reverse-engineering effort
([BenjaminHenriksson/insv-stitch](https://github.com/BenjaminHenriksson/insv-stitch))
that landed on the same model and field layout from different footage, and validated
here end-to-end against a real capture (see `equirect.hpp`/`equirect.cpp` for the
exact math and how the rotation convention was pinned down empirically).

What this does *not* do, matching the state of that reference implementation:
IMU-based stabilization or horizon-locking, rolling-shutter correction, or
per-channel chromatic-aberration correction. Concretely, that means the output
faithfully reproduces whatever roll/tilt the physical camera had at each instant --
a handheld clip's equirect video will visibly tilt as the camera does, exactly like
the raw fisheye footage does. Leveling that requires fusing the IMU stream (already
exported as `imu.csv`) into a per-frame stabilization rotation, which is future work.
The seam blend is a plain linear feather by view angle (weight 1 within 83 degrees of
a lens' own axis, fading to 0 by 97 degrees), which close-up testing against this
same reference showed performs as well as optical-flow-based blending for this
camera's ~3cm lens baseline -- distant structure aligns well; close objects (a hand
near the lens) show the parallax ghosting inherent to any dual-fisheye rig at that
range, not fixable by geometry alone.

This is compute-heavier than the rest of the tool: building the per-lens remap
tables is a one-time cost per run, but every frame still does two `cv::remap` calls
plus a per-pixel blend at the target resolution. Expect roughly the same order of
magnitude as decoding the source HEVC.

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
the lens center, not something to feed `cv::undistort`. The `equirect/` output (see
below) is where the full model is actually used; `--equirect` is the thing to reach
for if you want the geometry right, not this file.

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
  plus radial/tangential terms -- see [Equirect video](#equirect)), which is
  what `--equirect` actually uses. `k`/`p` in that same file are ordinary pinhole
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
- **`--equirect` has no horizon leveling.** Without IMU-based stabilization, the
  output tilts and rolls exactly as the physical camera did at each instant -- see
  [Equirect video](#equirect).

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
| `include/insta360/calibration.hpp`, `src/calibration.cpp` | Lens calibration strings -> rescaled intrinsics |
| `include/insta360/sensors.hpp`, `src/sensors.cpp` | IMU, exposure and frame-timestamp decoding |
| `include/insta360/subprocess.hpp`, `src/subprocess.cpp` | `posix_spawn`-based child process wrapper (no shell) |
| `include/insta360/media.hpp`, `src/media.cpp` | ffprobe/ffmpeg piping, JPEG frame splitting |
| `include/insta360/panorama.hpp`, `src/panorama.cpp` | Decodes the firmware-stitched preview |
| `include/insta360/equirect.hpp`, `src/equirect.cpp` | Geometric fisheye -> equirect stitch (Mei/UCM projection, seam blend) |
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
