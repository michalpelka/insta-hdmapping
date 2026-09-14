// Reader for the Insta360 metadata trailer appended to .insv / .insp files.
//
// An .insv file is an ordinary MP4 (two fisheye HEVC tracks plus AAC audio) with a
// proprietary blob glued onto the end. The layout, walking backwards from EOF, is:
//
//   ...MP4 boxes... | record | footer | record | footer | ... | directory | footer | tail
//
//   tail (72 bytes)   32 bytes reserved (zero)
//                     uint32 le  trailer size, in bytes, counted back from EOF
//                     uint32 le  trailer format version (3 on current firmware)
//                     char[32]   "8db42d694ccc418790edff439fe026bf"
//
//   footer (6 bytes)  uint16 be  record id
//                     uint32 le  record payload length
//
//   directory         10 bytes reserved, then 10-byte entries of
//                     uint16 le record id, uint32 le length, uint32 le offset
//                     where offset is relative to the start of the trailer
//
// Note the record id is big-endian in the per-record footer but little-endian in the
// directory -- verified against real captures. The directory is authoritative here and
// the footers are used only as a consistency check.
#pragma once

#include <cstdint>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace insta360 {

class TrailerError : public std::runtime_error {
public:
    explicit TrailerError(const std::string& what) : std::runtime_error(what) {}
};

// Record ids seen on Insta360 ONE X2 / X3 / X4 / X5 firmware.
constexpr uint16_t REC_DIRECTORY = 0x0000;
constexpr uint16_t REC_PREVIEW = 0x0002;         // NV12 equirectangular thumbnail
constexpr uint16_t REC_IMU = 0x0003;             // 1 kHz accelerometer + gyroscope
constexpr uint16_t REC_EXPOSURE = 0x0004;        // per-frame exposure time
constexpr uint16_t REC_ISP_STATS_A = 0x0009;
constexpr uint16_t REC_SENSOR_MAP = 0x000A;
constexpr uint16_t REC_ISP_STATS_B = 0x000B;
constexpr uint16_t REC_PREVIEW_SEQUENCE = 0x0016;
constexpr uint16_t REC_LUT = 0x001C;
constexpr uint16_t REC_FRAME_MARKS = 0x001D;
constexpr uint16_t REC_METADATA = 0x0101;        // protobuf; see metadata.hpp

std::string record_name(uint16_t id);

struct Record {
    uint16_t id;
    uint64_t offset;  // absolute file offset of the payload
    uint32_t length;

    std::string name() const { return record_name(id); }
};

// Random-access reader over the trailer records of a single file.
class Trailer {
public:
    explicit Trailer(const std::string& path);

    uint64_t file_size() const { return file_size_; }
    uint32_t version() const { return version_; }
    uint64_t base_offset() const { return base_offset_; }

    bool contains(uint16_t record_id) const { return records_.find(record_id) != records_.end(); }
    const Record* find(uint16_t record_id) const;
    std::vector<uint8_t> read(uint16_t record_id) const;

    // All records, sorted by file offset (matches Python's Trailer.__iter__).
    std::vector<Record> records() const;

    // Cross-checks every directory entry against the record's own footer. Returns a
    // list of human-readable complaints; empty means the trailer is internally
    // consistent. The directory alone is enough to read the data, so this is only a
    // soft warning.
    std::vector<std::string> footer_mismatches() const;

private:
    std::vector<uint8_t> read_at(uint64_t offset, uint64_t length) const;
    std::pair<uint32_t, uint64_t> read_tail();
    std::map<uint16_t, Record> read_directory();
    std::pair<uint16_t, uint32_t> unpack_footer(uint64_t offset) const;

    std::string path_;
    mutable std::ifstream file_;
    uint64_t file_size_ = 0;
    uint32_t version_ = 0;
    uint64_t base_offset_ = 0;
    std::map<uint16_t, Record> records_;
};

}  // namespace insta360
