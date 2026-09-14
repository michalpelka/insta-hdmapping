// ffmpeg-driven extraction of the video essence from an .insv file.
//
// The container is a plain MP4, so ffmpeg reads it directly; only the trailer needs
// bespoke parsing. Frames stream through a pipe so a long clip never has to be staged
// on disk or held in memory.
#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "insta360/subprocess.hpp"

namespace insta360 {

class FFmpegError : public std::runtime_error {
public:
    explicit FFmpegError(const std::string& what) : std::runtime_error(what) {}
};

struct StreamInfo {
    int index = 0;
    std::string codec;
    std::optional<int> width;
    std::optional<int> height;
    std::optional<double> frame_rate;
    std::optional<int> sample_rate;
    std::optional<int> channels;
};

struct Probe {
    std::optional<double> duration_s;
    std::vector<StreamInfo> video;
    std::vector<StreamInfo> audio;
};

// Throws FFmpegError if ffmpeg/ffprobe are not on PATH.
void require_tools();

// Enumerates the streams in the container via ffprobe.
Probe probe(const std::string& path);

// Streams one video track out of the file as a sequence of JPEG buffers.
class FramePipe {
public:
    FramePipe(const std::string& path, int video_index, int quality,
              const std::optional<std::string>& scale_filter, std::optional<int64_t> limit);
    ~FramePipe();

    FramePipe(const FramePipe&) = delete;
    FramePipe& operator=(const FramePipe&) = delete;

    // Returns the next JPEG frame, or nullopt at natural EOF. Throws FFmpegError on a
    // malformed stream or a non-zero ffmpeg exit after EOF.
    std::optional<std::vector<uint8_t>> next();

private:
    std::string path_;
    std::unique_ptr<Subprocess> process_;
    std::vector<uint8_t> buffer_;
    bool eof_ = false;
};

}  // namespace insta360
