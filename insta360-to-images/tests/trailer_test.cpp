#include "insta360/trailer.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>

#include "test_util.hpp"

using namespace insta360;

namespace {

void push_u16le(std::vector<uint8_t>& buf, uint16_t v) {
    buf.push_back(v & 0xFF);
    buf.push_back((v >> 8) & 0xFF);
}
void push_u16be(std::vector<uint8_t>& buf, uint16_t v) {
    buf.push_back((v >> 8) & 0xFF);
    buf.push_back(v & 0xFF);
}
void push_u32le(std::vector<uint8_t>& buf, uint32_t v) {
    for (int i = 0; i < 4; ++i) buf.push_back((v >> (8 * i)) & 0xFF);
}

// Builds a minimal but structurally valid Insta360 trailer: a fake "video" prefix,
// two records (each followed by its footer), a directory, and the 72-byte tail --
// exactly the layout described in trailer.hpp.
std::vector<uint8_t> build_file(const std::vector<uint8_t>& record_a,
                                 const std::vector<uint8_t>& record_b, bool corrupt_footer = false) {
    std::vector<uint8_t> file;
    const char* prefix = "fake mp4 boxes here";
    file.insert(file.end(), prefix, prefix + std::strlen(prefix));

    uint64_t trailer_start = file.size();

    uint64_t a_offset = file.size() - trailer_start;
    file.insert(file.end(), record_a.begin(), record_a.end());
    push_u16be(file, REC_IMU);
    push_u32le(file, corrupt_footer ? static_cast<uint32_t>(record_a.size() + 1)
                                     : static_cast<uint32_t>(record_a.size()));

    uint64_t b_offset = file.size() - trailer_start;
    file.insert(file.end(), record_b.begin(), record_b.end());
    push_u16be(file, REC_EXPOSURE);
    push_u32le(file, static_cast<uint32_t>(record_b.size()));

    std::vector<uint8_t> directory;
    directory.resize(10, 0);  // reserved header
    push_u16le(directory, REC_IMU);
    push_u32le(directory, static_cast<uint32_t>(record_a.size()));
    push_u32le(directory, static_cast<uint32_t>(a_offset));
    push_u16le(directory, REC_EXPOSURE);
    push_u32le(directory, static_cast<uint32_t>(record_b.size()));
    push_u32le(directory, static_cast<uint32_t>(b_offset));

    file.insert(file.end(), directory.begin(), directory.end());  // header + entries, on disk
    push_u16be(file, REC_DIRECTORY);
    push_u32le(file, static_cast<uint32_t>(directory.size()));

    std::vector<uint8_t> tail(32, 0);
    // The size field counts back from EOF, i.e. it includes the 72-byte tail itself.
    constexpr uint32_t kTailLen = 32 + 4 + 4 + 32;
    uint32_t trailer_size = static_cast<uint32_t>(file.size() - trailer_start) + kTailLen;
    push_u32le(tail, trailer_size);
    push_u32le(tail, 3);  // version
    const char* magic = "8db42d694ccc418790edff439fe026bf";
    tail.insert(tail.end(), magic, magic + 32);
    file.insert(file.end(), tail.begin(), tail.end());

    return file;
}

std::string write_temp_file(const std::vector<uint8_t>& bytes, const char* name) {
    std::string path = std::string("/tmp/") + name;
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    out.close();
    return path;
}

}  // namespace

TEST(trailer_reads_records_back) {
    std::vector<uint8_t> record_a(20, 0xAA);
    std::vector<uint8_t> record_b(16, 0xBB);
    auto bytes = build_file(record_a, record_b);
    std::string path = write_temp_file(bytes, "insta360_trailer_test.bin");

    Trailer trailer(path);
    CHECK_EQ(trailer.version(), 3u);
    CHECK(trailer.contains(REC_IMU));
    CHECK(trailer.contains(REC_EXPOSURE));
    CHECK(!trailer.contains(REC_PREVIEW));
    CHECK(trailer.read(REC_IMU) == record_a);
    CHECK(trailer.read(REC_EXPOSURE) == record_b);
    CHECK(trailer.footer_mismatches().empty());

    auto records = trailer.records();
    CHECK_EQ(records.size(), 2u);
    CHECK(records[0].offset < records[1].offset);  // sorted by file offset
}

TEST(trailer_flags_footer_mismatch) {
    std::vector<uint8_t> record_a(20, 0xAA);
    std::vector<uint8_t> record_b(16, 0xBB);
    auto bytes = build_file(record_a, record_b, /*corrupt_footer=*/true);
    std::string path = write_temp_file(bytes, "insta360_trailer_test_corrupt.bin");

    Trailer trailer(path);
    auto problems = trailer.footer_mismatches();
    CHECK_EQ(problems.size(), 1u);
}

TEST(trailer_rejects_file_without_magic) {
    std::vector<uint8_t> bytes(200, 0);
    std::string path = write_temp_file(bytes, "insta360_trailer_test_no_magic.bin");
    CHECK_THROWS(Trailer(path));
}

TEST(trailer_rejects_missing_record) {
    std::vector<uint8_t> record_a(20, 0xAA);
    std::vector<uint8_t> record_b(16, 0xBB);
    auto bytes = build_file(record_a, record_b);
    std::string path = write_temp_file(bytes, "insta360_trailer_test_missing.bin");
    Trailer trailer(path);
    CHECK_THROWS(trailer.read(REC_METADATA));
}
