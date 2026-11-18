#include "insta360/metadata.hpp"

#include "test_util.hpp"

using namespace insta360;

namespace {

void push_varint(std::vector<uint8_t>& buf, uint64_t value) {
    while (true) {
        uint8_t byte = value & 0x7F;
        value >>= 7;
        buf.push_back(value ? (byte | 0x80) : byte);
        if (!value) break;
    }
}

void push_uint(std::vector<uint8_t>& buf, int field, uint64_t value) {
    push_varint(buf, static_cast<uint64_t>(field) << 3);  // wire type 0
    push_varint(buf, value);
}

void push_bytes(std::vector<uint8_t>& buf, int field, const std::vector<uint8_t>& bytes) {
    push_varint(buf, (static_cast<uint64_t>(field) << 3) | 2);
    push_varint(buf, bytes.size());
    buf.insert(buf.end(), bytes.begin(), bytes.end());
}

void push_text(std::vector<uint8_t>& buf, int field, const std::string& text) {
    push_bytes(buf, field, std::vector<uint8_t>(text.begin(), text.end()));
}

// The fields every capture carries: 3840x3840 frames, first frame and its epoch.
std::vector<uint8_t> base_message() {
    std::vector<uint8_t> buf;
    std::vector<uint8_t> size;
    push_uint(size, 1, 3840);
    push_uint(size, 2, 3840);
    push_bytes(buf, 19, size);
    push_uint(buf, 20, 30);
    push_uint(buf, 24, 8724295018);
    push_uint(buf, 36, 1795015802335);
    return buf;
}

std::string lens_group(int fields, double cx) {
    std::string group = "2.0_1000_1000_" + std::to_string(cx) + "_1440_0_0_90_0_0_0";
    for (int i = 11; i < fields - 3; ++i) group += "_0";
    return group + "_5760_2880_113";
}

std::string calibration(int fields) {
    return "2_" + lens_group(fields, 1440) + "_" + lens_group(fields, 4320) + "_0";
}

}  // namespace

TEST(x6_metadata_gives_frame_count_and_offset_v3_lenses) {
    // An X6 writes no clock sub-message (field 98) and no offset_v2: the frame
    // count is field 40 and the only calibration is offset_v3.
    auto buf = base_message();
    push_uint(buf, 40, 251);
    push_text(buf, 111, calibration(27));
    Metadata meta = parse_metadata(buf);
    CHECK(meta.frame_count.has_value());
    CHECK_EQ(*meta.frame_count, 251);
    CHECK(meta.clock().has_value());
    CHECK_EQ(meta.lenses.size(), 2u);
    CHECK_EQ(meta.lenses[0].source, std::string("offset_v3"));
    CHECK_NEAR(meta.lenses[1].cx, 1920.0, 1e-6);  // lens-local, rescaled by 3840/2880
}

TEST(x5_metadata_prefers_the_clock_frame_count_and_offset_v2) {
    // An X5 writes field 40 as zero and both calibrations; offset_v2 is the one this
    // tool was validated on, so it wins.
    auto buf = base_message();
    push_uint(buf, 40, 0);
    std::vector<uint8_t> clock;
    push_uint(clock, 1, 8724295018);
    push_uint(clock, 3, 1795015802400);
    push_uint(clock, 5, 14872);
    push_bytes(buf, 98, clock);
    push_text(buf, 54, calibration(19));
    push_text(buf, 111, calibration(27));
    Metadata meta = parse_metadata(buf);
    CHECK(meta.frame_count.has_value());
    CHECK_EQ(*meta.frame_count, 14872);
    CHECK_EQ(*meta.start_epoch_ms, 1795015802400);  // clock overrides field 36
    CHECK_EQ(meta.lenses.size(), 2u);
    CHECK_EQ(meta.lenses[0].source, std::string("offset_v2"));
}

TEST(a_zero_field_40_leaves_the_frame_count_unknown) {
    auto buf = base_message();
    push_uint(buf, 40, 0);
    CHECK(!parse_metadata(buf).frame_count.has_value());
}

TEST(parse_lenses_falls_back_to_offset_v3_when_offset_v2_is_malformed) {
    auto buf = base_message();
    push_text(buf, 54, "2_garbage");
    push_text(buf, 111, calibration(27));
    Metadata meta = parse_metadata(buf);
    CHECK_EQ(meta.lenses.size(), 2u);
    CHECK_EQ(meta.lenses[0].source, std::string("offset_v3"));
}
