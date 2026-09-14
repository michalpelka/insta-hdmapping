#include "insta360/convert.hpp"

#include "test_util.hpp"

using namespace insta360;

TEST(resolve_scale_with_no_spec_keeps_source_size) {
    auto [w, h] = resolve_scale(std::nullopt, 2880, 2880);
    CHECK_EQ(w, 2880);
    CHECK_EQ(h, 2880);
}

TEST(resolve_scale_explicit_dimensions) {
    auto [w, h] = resolve_scale(std::string("1440x1440"), 2880, 2880);
    CHECK_EQ(w, 1440);
    CHECK_EQ(h, 1440);
}

TEST(resolve_scale_factor_rounds_to_even) {
    auto [w, h] = resolve_scale(std::string("0.5"), 2881, 2881);
    // (2881 * 0.5) = 1440.5 -> round to nearest, then forced even.
    CHECK_EQ(w % 2, 0);
    CHECK_EQ(h % 2, 0);
    CHECK(w >= 2);
}

TEST(resolve_scale_rejects_factor_over_one) {
    CHECK_THROWS(resolve_scale(std::string("1.5"), 100, 100));
}

TEST(resolve_scale_rejects_nonpositive_dimensions) {
    CHECK_THROWS(resolve_scale(std::string("0x100"), 100, 100));
    CHECK_THROWS(resolve_scale(std::string("100x-1"), 100, 100));
}

TEST(resolve_scale_rejects_garbage) {
    CHECK_THROWS(resolve_scale(std::string("not-a-scale"), 100, 100));
}
