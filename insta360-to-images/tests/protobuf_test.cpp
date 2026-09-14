#include "insta360/protobuf.hpp"

#include <cstring>

#include "test_util.hpp"

using insta360::Message;
using insta360::ProtobufError;

namespace {

void push_varint(std::vector<uint8_t>& buf, uint64_t value) {
    while (true) {
        uint8_t byte = value & 0x7F;
        value >>= 7;
        if (value) {
            buf.push_back(byte | 0x80);
        } else {
            buf.push_back(byte);
            break;
        }
    }
}

void push_tag(std::vector<uint8_t>& buf, int field, int wire_type) {
    push_varint(buf, (static_cast<uint64_t>(field) << 3) | wire_type);
}

}  // namespace

TEST(varint_and_bytes_fields_round_trip) {
    std::vector<uint8_t> buf;
    // field 1, varint 150 (the canonical protobuf-docs example).
    push_tag(buf, 1, Message::kVarint);
    push_varint(buf, 150);
    // field 2, string "testing".
    push_tag(buf, 2, Message::kBytes);
    push_varint(buf, 7);
    const char* text = "testing";
    buf.insert(buf.end(), text, text + 7);
    // field 3, fixed64 double 2.5.
    push_tag(buf, 3, Message::kFixed64);
    double pi = 2.5;
    uint8_t pi_bytes[8];
    std::memcpy(pi_bytes, &pi, 8);
    buf.insert(buf.end(), pi_bytes, pi_bytes + 8);
    // field 4, nested message {field 1: varint 5}.
    std::vector<uint8_t> inner;
    push_tag(inner, 1, Message::kVarint);
    push_varint(inner, 5);
    push_tag(buf, 4, Message::kBytes);
    push_varint(buf, inner.size());
    buf.insert(buf.end(), inner.begin(), inner.end());
    // field 6, packed doubles [1.0, 2.0].
    push_tag(buf, 6, Message::kBytes);
    push_varint(buf, 16);
    double values[2] = {1.0, 2.0};
    uint8_t values_bytes[16];
    std::memcpy(values_bytes, values, 16);
    buf.insert(buf.end(), values_bytes, values_bytes + 16);

    Message message = Message::parse(buf);
    CHECK(message.contains(1));
    CHECK_EQ(*message.uint(1), 150u);
    CHECK_EQ(*message.text(2), std::string("testing"));
    CHECK_NEAR(*message.fixed64_double(3), 2.5, 1e-12);
    auto nested = message.message(4);
    CHECK(nested.has_value());
    CHECK_EQ(*nested->uint(1), 5u);
    auto doubles = message.doubles(6);
    CHECK_EQ(doubles.size(), 2u);
    CHECK_NEAR(doubles[0], 1.0, 1e-12);
    CHECK_NEAR(doubles[1], 2.0, 1e-12);

    CHECK(!message.contains(5));
    CHECK(!message.uint(5).has_value());
}

TEST(truncated_varint_throws) {
    std::vector<uint8_t> buf;
    push_tag(buf, 1, Message::kVarint);
    buf.push_back(0x80);  // continuation bit set, but nothing follows
    CHECK_THROWS(Message::parse(buf));
}

TEST(unsupported_wire_type_throws) {
    std::vector<uint8_t> buf;
    push_varint(buf, (1 << 3) | 3);  // wire type 3 (start group), removed in proto3
    CHECK_THROWS(Message::parse(buf));
}
