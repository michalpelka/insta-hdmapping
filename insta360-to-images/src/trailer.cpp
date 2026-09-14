#include "insta360/trailer.hpp"

#include <algorithm>
#include <cstring>

namespace insta360 {

namespace {

constexpr char kMagic[] = "8db42d694ccc418790edff439fe026bf";  // 32 bytes, no NUL
constexpr size_t kMagicLen = 32;
constexpr uint64_t kTailLen = 32 + 4 + 4 + kMagicLen;  // 72
constexpr uint64_t kFooterLen = 6;
constexpr uint64_t kDirectoryHeaderLen = 10;
constexpr uint64_t kDirectoryEntryLen = 10;

uint16_t read_u16le(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
uint16_t read_u16be(const uint8_t* p) { return static_cast<uint16_t>((p[0] << 8) | p[1]); }
uint32_t read_u32le(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

}  // namespace

std::string record_name(uint16_t id) {
    switch (id) {
        case REC_DIRECTORY: return "directory";
        case REC_PREVIEW: return "preview image (NV12)";
        case REC_IMU: return "imu (accel + gyro)";
        case REC_EXPOSURE: return "exposure times";
        case REC_ISP_STATS_A: return "isp stats a";
        case REC_SENSOR_MAP: return "sensor map";
        case REC_ISP_STATS_B: return "isp stats b";
        case REC_PREVIEW_SEQUENCE: return "preview sequence";
        case REC_LUT: return "lookup table";
        case REC_FRAME_MARKS: return "frame marks";
        case REC_METADATA: return "metadata (protobuf)";
        default: {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "unknown 0x%04x", id);
            return buf;
        }
    }
}

Trailer::Trailer(const std::string& path) : path_(path) {
    file_.open(path, std::ios::binary);
    if (!file_) {
        throw TrailerError("could not open " + path);
    }
    file_.seekg(0, std::ios::end);
    file_size_ = static_cast<uint64_t>(file_.tellg());

    auto [version, base_offset] = read_tail();
    version_ = version;
    base_offset_ = base_offset;
    records_ = read_directory();
}

std::vector<uint8_t> Trailer::read_at(uint64_t offset, uint64_t length) const {
    file_.seekg(static_cast<std::streamoff>(offset));
    std::vector<uint8_t> data(length);
    file_.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(length));
    if (static_cast<uint64_t>(file_.gcount()) != length) {
        throw TrailerError("short read of " + std::to_string(length) + " bytes at offset " +
                            std::to_string(offset));
    }
    return data;
}

std::pair<uint32_t, uint64_t> Trailer::read_tail() {
    if (file_size_ < kTailLen) {
        throw TrailerError("file is too small to contain an Insta360 trailer");
    }
    auto tail = read_at(file_size_ - kTailLen, kTailLen);
    if (std::memcmp(tail.data() + 40, kMagic, kMagicLen) != 0) {
        throw TrailerError(
            "Insta360 trailer magic not found at end of file -- "
            "is this an original .insv/.insp straight off the camera?");
    }
    uint32_t trailer_size = read_u32le(tail.data() + 32);
    uint32_t version = read_u32le(tail.data() + 36);
    if (trailer_size == 0 || trailer_size > file_size_) {
        throw TrailerError("implausible trailer size " + std::to_string(trailer_size));
    }
    return {version, file_size_ - trailer_size};
}

std::pair<uint16_t, uint32_t> Trailer::unpack_footer(uint64_t offset) const {
    auto raw = read_at(offset, kFooterLen);
    return {read_u16be(raw.data()), read_u32le(raw.data() + 2)};
}

std::map<uint16_t, Record> Trailer::read_directory() {
    // The directory is the last record, immediately before the 72-byte tail.
    uint64_t footer_at = file_size_ - kTailLen - kFooterLen;
    auto [dir_id, dir_len] = unpack_footer(footer_at);
    if (dir_id != REC_DIRECTORY) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "expected directory record, found id 0x%04x", dir_id);
        throw TrailerError(buf);
    }
    auto raw = read_at(footer_at - dir_len, dir_len);

    std::map<uint16_t, Record> records;
    if (raw.size() < kDirectoryHeaderLen) {
        throw TrailerError("trailer directory is shorter than its header");
    }
    const uint8_t* body = raw.data() + kDirectoryHeaderLen;
    size_t body_len = raw.size() - kDirectoryHeaderLen;
    for (size_t pos = 0; pos + kDirectoryEntryLen <= body_len; pos += kDirectoryEntryLen) {
        uint16_t rec_id = read_u16le(body + pos);
        uint32_t length = read_u32le(body + pos + 2);
        uint32_t offset = read_u32le(body + pos + 6);
        if (rec_id == 0 && length == 0) {
            continue;  // unused slot; the table is fixed-size and zero padded
        }
        uint64_t absolute = base_offset_ + offset;
        if (absolute + length > file_size_) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "directory entry 0x%04x points outside the file", rec_id);
            throw TrailerError(buf);
        }
        records[rec_id] = Record{rec_id, absolute, length};
    }
    if (records.empty()) {
        throw TrailerError("trailer directory is empty");
    }
    return records;
}

const Record* Trailer::find(uint16_t record_id) const {
    auto it = records_.find(record_id);
    return it == records_.end() ? nullptr : &it->second;
}

std::vector<uint8_t> Trailer::read(uint16_t record_id) const {
    const Record* record = find(record_id);
    if (!record) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "record 0x%04x is not present in this file", record_id);
        throw TrailerError(buf);
    }
    return read_at(record->offset, record->length);
}

std::vector<Record> Trailer::records() const {
    std::vector<Record> result;
    result.reserve(records_.size());
    for (const auto& [id, record] : records_) result.push_back(record);
    std::sort(result.begin(), result.end(),
              [](const Record& a, const Record& b) { return a.offset < b.offset; });
    return result;
}

std::vector<std::string> Trailer::footer_mismatches() const {
    std::vector<std::string> problems;
    for (const Record& record : records()) {
        if (record.id == REC_DIRECTORY) continue;
        uint64_t end = record.offset + record.length;
        if (end + kFooterLen > file_size_) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "0x%04x: no room for a footer", record.id);
            problems.push_back(buf);
            continue;
        }
        auto [footer_id, footer_len] = unpack_footer(end);
        if (footer_id != record.id || footer_len != record.length) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                           "0x%04x: directory says (id=0x%04x, len=%u) but footer says "
                           "(id=0x%04x, len=%u)",
                           record.id, record.id, record.length, footer_id, footer_len);
            problems.push_back(buf);
        }
    }
    return problems;
}

}  // namespace insta360
