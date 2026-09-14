// Minimal protobuf wire-format reader.
//
// Insta360 stores its per-file metadata record as a serialised protobuf message but
// publishes no .proto schema, so field *names* are unknowable and fields have to be
// addressed by number. Rather than link the full protobuf runtime for one unknown
// message, we walk the wire format directly -- it is a few dozen lines and keeps the
// tool dependency-free.
//
// Wire format reference: https://protobuf.dev/programming-guides/encoding/
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace insta360 {

class ProtobufError : public std::runtime_error {
public:
    explicit ProtobufError(const std::string& what) : std::runtime_error(what) {}
};

// A decoded protobuf message, addressable by field number.
//
// Repeated fields keep every occurrence; the accessors below return the first
// occurrence of the requested wire type, which is what every field the metadata
// parser cares about happens to be.
class Message {
public:
    static constexpr int kVarint = 0;
    static constexpr int kFixed64 = 1;
    static constexpr int kBytes = 2;
    static constexpr int kFixed32 = 5;

    static Message parse(const uint8_t* data, size_t len);
    static Message parse(const std::vector<uint8_t>& data) { return parse(data.data(), data.size()); }

    bool contains(int field_number) const { return fields_.find(field_number) != fields_.end(); }

    std::optional<uint64_t> uint(int field_number) const;
    std::optional<double> fixed64_double(int field_number) const;
    std::optional<float> fixed32_float(int field_number) const;
    std::optional<std::vector<uint8_t>> raw(int field_number) const;
    std::optional<std::string> text(int field_number) const;
    std::optional<Message> message(int field_number) const;
    // Decodes a length-delimited field as a packed array of float64.
    std::vector<double> doubles(int field_number) const;

private:
    struct Value {
        int wire_type;
        uint64_t varint = 0;          // valid when wire_type == kVarint
        std::vector<uint8_t> bytes;   // valid for kFixed64 (8 bytes), kFixed32 (4 bytes), kBytes
    };

    const Value* first(int field_number, int wire_type) const;

    std::map<int, std::vector<Value>> fields_;
};

}  // namespace insta360
