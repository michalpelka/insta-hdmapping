#include "insta360/media.hpp"

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace insta360 {

namespace {

constexpr const char* FFMPEG = "ffmpeg";
constexpr const char* FFPROBE = "ffprobe";
constexpr size_t kReadChunk = 4 << 20;

bool tool_on_path(const char* name) {
    const char* path_env = std::getenv("PATH");
    if (!path_env) return false;
    std::string path(path_env);
    std::stringstream stream(path);
    std::string dir;
    while (std::getline(stream, dir, ':')) {
        std::string candidate = (dir.empty() ? "." : dir) + "/" + name;
        if (::access(candidate.c_str(), X_OK) == 0) return true;
    }
    return false;
}

std::vector<std::string> split_lines(const std::string& text) {
    std::vector<std::string> lines;
    std::stringstream stream(text);
    std::string line;
    while (std::getline(stream, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty()) lines.push_back(line);
    }
    return lines;
}

std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(field);
    return fields;
}

std::optional<double> parse_rate(const std::string& text) {
    auto slash = text.find('/');
    if (slash == std::string::npos) return std::nullopt;
    try {
        double numerator = std::stod(text.substr(0, slash));
        double denominator = std::stod(text.substr(slash + 1));
        if (denominator == 0) return std::nullopt;
        return numerator / denominator;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

std::optional<int> parse_int(const std::string& text) {
    if (text.empty() || text == "N/A") return std::nullopt;
    try {
        return std::stoi(text);
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

std::optional<double> parse_double(const std::string& text) {
    if (text.empty() || text == "N/A") return std::nullopt;
    try {
        return std::stod(text);
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

// Runs ffprobe to completion and returns its full stdout; throws FFmpegError on a
// non-zero exit.
std::string run_ffprobe(const std::vector<std::string>& args) {
    std::vector<std::string> argv = {FFPROBE, "-v", "error"};
    argv.insert(argv.end(), args.begin(), args.end());
    Subprocess process(argv);
    std::string output;
    char buf[65536];
    size_t n;
    while ((n = process.read_stdout(buf, sizeof(buf))) > 0) output.append(buf, n);
    process.mark_exhausted();
    process.close();
    std::string error = process.last_error("ffprobe");
    if (!error.empty()) throw FFmpegError(error);
    return output;
}

}  // namespace

void require_tools() {
    std::vector<std::string> missing;
    if (!tool_on_path(FFMPEG)) missing.push_back(FFMPEG);
    if (!tool_on_path(FFPROBE)) missing.push_back(FFPROBE);
    if (!missing.empty()) {
        std::string joined = missing[0];
        if (missing.size() > 1) joined += " and " + missing[1];
        throw FFmpegError(joined + " not found on PATH; install ffmpeg (e.g. 'brew install ffmpeg' "
                                    "or 'sudo apt install ffmpeg')");
    }
}

Probe probe(const std::string& path) {
    Probe result;

    // Video streams: index, codec, width, height, frame rate.
    for (const auto& line : split_lines(run_ffprobe(
             {"-select_streams", "v", "-show_entries",
              "stream=index,codec_name,width,height,avg_frame_rate", "-of", "csv=p=0", path}))) {
        auto fields = split_csv(line);
        if (fields.size() < 5) continue;
        StreamInfo stream;
        stream.index = std::stoi(fields[0]);
        stream.codec = fields[1];
        stream.width = parse_int(fields[2]);
        stream.height = parse_int(fields[3]);
        stream.frame_rate = parse_rate(fields[4]);
        result.video.push_back(stream);
    }

    // Audio streams: index, codec, sample rate, channels.
    for (const auto& line : split_lines(run_ffprobe(
             {"-select_streams", "a", "-show_entries", "stream=index,codec_name,sample_rate,channels",
              "-of", "csv=p=0", path}))) {
        auto fields = split_csv(line);
        if (fields.size() < 4) continue;
        StreamInfo stream;
        stream.index = std::stoi(fields[0]);
        stream.codec = fields[1];
        stream.sample_rate = parse_int(fields[2]);
        stream.channels = parse_int(fields[3]);
        result.audio.push_back(stream);
    }

    // Container duration.
    auto duration_lines =
        split_lines(run_ffprobe({"-show_entries", "format=duration", "-of", "csv=p=0", path}));
    if (!duration_lines.empty()) result.duration_s = parse_double(duration_lines[0]);

    return result;
}

// -- JPEG framing -------------------------------------------------------------------

namespace {

// Returns the index just past the EOI of the JPEG at `start`, or -1 if truncated.
//
// Walks the marker structure rather than searching for a bare FF D9: inside
// entropy-coded scan data a literal 0xFF is byte-stuffed as FF 00, so a naive search
// happens to work for ffmpeg's output, but restart markers and multi-scan images
// make that a coincidence rather than a guarantee.
long find_jpeg_end(const std::vector<uint8_t>& buf, size_t start) {
    size_t size = buf.size();
    size_t pos = start;
    if (size - pos < 2 || buf[pos] != 0xFF || buf[pos + 1] != 0xD8) {
        throw FFmpegError("ffmpeg output is not positioned at a JPEG SOI marker");
    }
    pos += 2;
    while (true) {
        while (pos < size && buf[pos] != 0xFF) pos++;
        while (pos < size && buf[pos] == 0xFF) pos++;  // 0xFF fill bytes before a marker
        if (pos >= size) return -1;
        uint8_t marker = buf[pos];
        pos++;
        if (marker == 0xD9) return static_cast<long>(pos);  // EOI
        if (marker == 0x01 || (marker >= 0xD0 && marker <= 0xD7)) continue;  // TEM, RSTn: no payload
        if (size - pos < 2) return -1;
        int segment_length = (buf[pos] << 8) | buf[pos + 1];
        if (segment_length < 2) throw FFmpegError("invalid JPEG segment length in ffmpeg output");
        pos += segment_length;
        if (marker != 0xDA) continue;  // not SOS, so the next marker follows immediately
        // Scan past entropy-coded data to the next real marker.
        while (true) {
            size_t found = pos;
            while (found < size && buf[found] != 0xFF) found++;
            if (found >= size || found + 1 >= size) return -1;
            uint8_t following = buf[found + 1];
            if (following == 0x00 || (following >= 0xD0 && following <= 0xD7) || following == 0xFF) {
                pos = following == 0xFF ? found + 1 : found + 2;
                continue;
            }
            pos = found;
            break;
        }
    }
}

}  // namespace

FramePipe::FramePipe(const std::string& path, int video_index, int quality,
                      const std::optional<std::string>& scale_filter, std::optional<int64_t> limit)
    : path_(path) {
    std::vector<std::string> argv = {FFMPEG,  "-nostdin", "-v", "error",
                                      "-i",    path,       "-map", "0:v:" + std::to_string(video_index)};
    if (limit) {
        argv.push_back("-frames:v");
        argv.push_back(std::to_string(*limit));
    }
    if (scale_filter) {
        argv.push_back("-vf");
        argv.push_back("scale=" + *scale_filter);
    }
    argv.insert(argv.end(), {"-c:v", "mjpeg", "-q:v", std::to_string(quality), "-f", "image2pipe", "-"});
    process_ = std::make_unique<Subprocess>(argv);
}

FramePipe::~FramePipe() {
    if (process_) process_->close();
}

std::optional<std::vector<uint8_t>> FramePipe::next() {
    while (true) {
        if (!buffer_.empty()) {
            long end = find_jpeg_end(buffer_, 0);
            if (end >= 0) {
                std::vector<uint8_t> frame(buffer_.begin(), buffer_.begin() + end);
                buffer_.erase(buffer_.begin(), buffer_.begin() + end);
                return frame;
            }
        }
        if (eof_) {
            bool all_zero = true;
            for (uint8_t b : buffer_) {
                if (b != 0) {
                    all_zero = false;
                    break;
                }
            }
            if (!buffer_.empty() && !all_zero) {
                throw FFmpegError(std::to_string(buffer_.size()) +
                                   " trailing bytes from ffmpeg did not form a complete JPEG");
            }
            process_->mark_exhausted();
            process_->close();
            std::string error = process_->last_error("ffmpeg exited with an error while decoding " + path_);
            if (!error.empty()) throw FFmpegError(error);
            return std::nullopt;
        }
        static thread_local std::vector<uint8_t> chunk(kReadChunk);
        size_t n = process_->read_stdout(chunk.data(), chunk.size());
        if (n == 0) {
            eof_ = true;
            continue;
        }
        buffer_.insert(buffer_.end(), chunk.data(), chunk.data() + n);
    }
}

}  // namespace insta360
