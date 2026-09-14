#include "insta360/calibration.hpp"

#include "test_util.hpp"

using namespace insta360;

TEST(parse_offset_v2_rescales_a_single_lens) {
    // n=1 lens; group: xi,fx,fy,cx,cy,rx,ry,rz,tx,ty,tz,k1,k2,k3,p1,p2,canvas_w,canvas_h,crop;
    // one trailing token that the parser does not need to look at.
    std::string text =
        "1_2.0_1000_1000_1440_1440_1_2_3_4_5_6_0.1_0.2_0.3_0.4_0.5_5760_2880_0_999";

    auto lenses = parse_offset_v2(text, 1440, 1440);
    CHECK_EQ(lenses.size(), 1u);
    const auto& lens = lenses[0];
    CHECK_EQ(lens.index, 0);
    CHECK_NEAR(lens.xi, 2.0, 1e-9);  // dimensionless: not rescaled with fx/fy/cx/cy
    CHECK_NEAR(lens.fx, 500.0, 1e-9);
    CHECK_NEAR(lens.fy, 500.0, 1e-9);
    CHECK_NEAR(lens.cx, 720.0, 1e-9);
    CHECK_NEAR(lens.cy, 720.0, 1e-9);
    CHECK_NEAR(lens.k1, 0.1, 1e-9);
    CHECK_NEAR(lens.k2, 0.2, 1e-9);
    CHECK_NEAR(lens.k3, 0.3, 1e-9);
    CHECK_NEAR(lens.p1, 0.4, 1e-9);
    CHECK_NEAR(lens.p2, 0.5, 1e-9);
    CHECK_EQ(lens.distortion.size(), 5u);
    CHECK_NEAR(lens.distortion[0], 0.1, 1e-9);
    CHECK_NEAR(lens.distortion[4], 0.5, 1e-9);
    CHECK_NEAR(lens.rotation_deg[0], 1.0, 1e-9);
    CHECK_NEAR(lens.translation[2], 6.0, 1e-9);
    CHECK_EQ(lens.canvas[0], 5760);
    CHECK_EQ(lens.canvas[1], 2880);
    CHECK_NEAR(lens.scale[0], 0.5, 1e-9);

    auto k = lens.k();
    CHECK_NEAR(k[0], 500.0, 1e-9);  // fx
    CHECK_NEAR(k[2], 720.0, 1e-9);  // cx
    CHECK_NEAR(k[4], 500.0, 1e-9);  // fy
}

TEST(parse_offset_v2_offsets_the_second_lens_by_half_canvas) {
    // Two lenses side by side on a 5760-wide canvas; the second lens' cx is expressed
    // in absolute canvas coordinates and must be brought back to lens-local before
    // rescaling. Leading field (xi) is irrelevant here, so held at 0.
    std::string lens0 = "0_1000_1000_1440_1440_0_0_0_0_0_0_0_0_0_0_0_5760_2880_0";
    std::string lens1 = "0_1000_1000_4320_1440_0_0_0_0_0_0_0_0_0_0_0_5760_2880_0";  // cx = 1440 + 2880
    std::string text = "2_" + lens0 + "_" + lens1 + "_999";

    auto lenses = parse_offset_v2(text, 1440, 1440);
    CHECK_EQ(lenses.size(), 2u);
    CHECK_NEAR(lenses[0].cx, 720.0, 1e-9);
    CHECK_NEAR(lenses[1].cx, 720.0, 1e-9);  // same lens-local position, on the right half
}

TEST(parse_offset_v2_rejects_malformed_input) {
    CHECK(parse_offset_v2("", 100, 100).empty());
    CHECK(parse_offset_v2("not_a_number_at_all", 100, 100).empty());
    CHECK(parse_offset_v2("1_too_few_fields", 100, 100).empty());
    // A full 19-field group but with canvas_w = canvas_h = 0.
    CHECK(parse_offset_v2("1_0_1_1_1_1_1_1_1_1_1_1_1_1_1_1_1_0_0_0", 100, 100).empty());
}
