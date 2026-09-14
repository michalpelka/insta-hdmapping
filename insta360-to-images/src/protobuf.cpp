#include "insta360/protobuf.hpp"

#include <cstring>

namespace insta360 {

namespace {

// Reads a base-128 varint starting at buf[pos]; returns the value and the position
// just past it.
uint64_t read_varint(const uint8_t* buf, size_t len, size_t& pos) {
    uint64_t value = 0;
    int shift = 0;
    while (true) {
        if (pos >= len) {
            throw ProtobufError("truncated varint");
        }
        uint8_t byte = buf[pos++];
        value |= static_cast<uint64_t>(byte & 0x7F) << shift;
        if (!(byte & 0x80)) {
            return value;
        }
        shift += 7;
        if (shift > 63) {
            throw ProtobufError("varint longer than 64 bits");
        }
    }
}

}  // namespace

Message Message::parse(const uint8_t* data, size_t len) {
    Message message;
    size_t pos = 0;
    while (pos < len) {
        uint64_t key = read_varint(data, len, pos);
        int field_number = static_cast<int>(key >> 3);
        int wire_type = static_cast<int>(key & 0x07);
        if (field_number == 0) {
            throw ProtobufError("field number 0 is not valid");
        }
        Value value;
        value.wire_type = wire_type;
        switch (wire_type) {
            case kVarint:
                value.varint = read_varint(data, len, pos);
                break;
            case kFixed64:
                if (pos + 8 > len) throw ProtobufError("truncated 64-bit field");
                value.bytes.assign(data + pos, data + pos + 8);
                pos += 8;
                break;
            case kFixed32:
                if (pos + 4 > len) throw ProtobufError("truncated 32-bit field");
                value.bytes.assign(data + pos, data + pos + 4);
                pos += 4;
                break;
            case kBytes: {
                uint64_t length = read_varint(data, len, pos);
                if (pos + length > len) throw ProtobufError("truncated length-delimited field");
                value.bytes.assign(data + pos, data + pos + length);
                pos += length;
                break;
            }
            default:
                // Groups (3, 4) were removed from proto3 and 6/7 are unassigned.
                throw ProtobufError("unsupported wire type " + std::to_string(wire_type));
        }
        message.fields_[field_number].push_back(std::move(value));
    }
    return message;
}

const Message::Value* Message::first(int field_number, int wire_type) const {
    auto it = fields_.find(field_number);
    if (it == fields_.end()) return nullptr;
    for (const auto& value : it->second) {
        if (value.wire_type == wire_type) return &value;
    }
    return nullptr;
}

std::optional<uint64_t> Message::uint(int field_number) const {
    const Value* value = first(field_number, kVarint);
    if (!value) return std::nullopt;
    return value->varint;
}

std::optional<double> Message::fixed64_double(int field_number) const {
    const Value* value = first(field_number, kFixed64);
    if (!value) return std::nullopt;
    double result;
    std::memcpy(&result, value->bytes.data(), sizeof(result));
    return result;
}

std::optional<float> Message::fixed32_float(int field_number) const {
    const Value* value = first(field_number, kFixed32);
    if (!value) return std::nullopt;
    float result;
    std::memcpy(&result, value->bytes.data(), sizeof(result));
    return result;
}

std::optional<std::vector<uint8_t>> Message::raw(int field_number) const {
    const Value* value = first(field_number, kBytes);
    if (!value) return std::nullopt;
    return value->bytes;
}

std::optional<std::string> Message::text(int field_number) const {
    auto raw_bytes = raw(field_number);
    if (!raw_bytes) return std::nullopt;
    return std::string(raw_bytes->begin(), raw_bytes->end());
}

std::optional<Message> Message::message(int field_number) const {
    auto raw_bytes = raw(field_number);
    if (!raw_bytes) return std::nullopt;
    try {
        return Message::parse(*raw_bytes);
    } catch (const ProtobufError&) {
        return std::nullopt;
    }
}

std::vector<double> Message::doubles(int field_number) const {
    auto raw_bytes = raw(field_number);
    if (!raw_bytes || raw_bytes->size() % 8 != 0) return {};
    std::vector<double> result(raw_bytes->size() / 8);
    std::memcpy(result.data(), raw_bytes->data(), raw_bytes->size());
    return result;
}

}  // namespace insta360
