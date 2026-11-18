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

namespace {

// One offset_v3 lens group: xi fx fy cx cy rx ry rz tx ty tz, k1-k4, zero, p1 p2,
// six unapplied coefficients, canvas_w canvas_h crop.
std::string v3_group(const std::string& cx, const std::string& k1_to_k4) {
    return "2.0_1000_1000_" + cx + "_1440_1_2_3_4_5_6_" + k1_to_k4 +
           "_0_0.01_0.02_0.1_0.2_0.3_0.4_0.5_0.6_5760_2880_193";
}

}  // namespace

TEST(parse_offset_v3_reads_the_27_field_layout) {
    std::string text = "2_" + v3_group("1440", "0.1_0.2_0.3_0") + "_" + v3_group("4320", "0.1_0.2_0.3_0") +
                       "_394240";
    auto lenses = parse_offset_v3(text, 1440, 1440);
    CHECK_EQ(lenses.size(), 2u);
    for (const auto& lens : lenses) {
        CHECK_EQ(lens.source, std::string("offset_v3"));
        CHECK_NEAR(lens.xi, 2.0, 1e-9);
        CHECK_NEAR(lens.fx, 500.0, 1e-9);
        CHECK_NEAR(lens.cx, 720.0, 1e-9);  // the second lens brought back to lens-local
        CHECK_NEAR(lens.cy, 720.0, 1e-9);
        CHECK_NEAR(lens.rotation_deg[2], 3.0, 1e-9);
        CHECK_NEAR(lens.translation[0], 4.0, 1e-9);
        CHECK_EQ(lens.canvas[0], 5760);
        // p1/p2 sit after k1-k4 and the zero slot.
        CHECK_NEAR(lens.p1, 0.01, 1e-12);
        CHECK_NEAR(lens.p2, 0.02, 1e-12);
        CHECK_EQ(lens.distortion.size(), 5u);
        CHECK_NEAR(lens.distortion[3], 0.01, 1e-12);
        // Everything the string carried, verbatim, applied or not.
        CHECK_EQ(lens.source_distortion.size(), 13u);
        CHECK_NEAR(lens.source_distortion[3], 0.0, 1e-12);
        CHECK_NEAR(lens.source_distortion[12], 0.6, 1e-12);
    }
}

TEST(parse_offset_v3_refit_is_exact_when_k4_is_zero) {
    // With no r^8 term the four-term curve is already three-term, so the refit must
    // hand k1-k3 back unchanged.
    auto lenses = parse_offset_v3("1_" + v3_group("1440", "0.1_0.2_0.3_0") + "_0", 1440, 1440);
    CHECK_EQ(lenses.size(), 1u);
    CHECK_NEAR(lenses[0].k1, 0.1, 1e-6);
    CHECK_NEAR(lenses[0].k2, 0.2, 1e-6);
    CHECK_NEAR(lenses[0].k3, 0.3, 1e-6);
    CHECK(lenses[0].refit_max_error_px < 1e-6);
    // This lens' image never reaches its square's edge, so the range is the cap.
    CHECK_NEAR(lenses[0].refit_max_theta_deg, 120.0, 0.1);
}

TEST(parse_offset_v3_refits_a_real_x6_lens_to_within_half_a_pixel) {
    // Lens 0 of an Insta360 X6 (firmware v1.3.59): k4 = 20 is far from negligible
    // (~30 px at the image edge), so this checks the refit absorbs it.
    std::string text =
        "1_2.455430_7201.820_7201.820_3858.260_3880.730_0.387_-0.026_89.745_0.000000_0.000000_"
        "0.000000_1.27328110_-0.70271325_-0.70118731_20.13775826_0.00000000_-0.00246698_"
        "0.00143700_0.00492177_-0.00308467_0.00314514_-0.00091293_0.01603204_-0.01369528_15488_"
        "7744_193_394240";
    auto lenses = parse_offset_v3(text, 3840, 3840);
    CHECK_EQ(lenses.size(), 1u);
    const auto& lens = lenses[0];
    CHECK_NEAR(lens.fx, 7201.82 * 3840 / 7744, 1e-6);
    CHECK(lens.refit_max_error_px > 0.0);
    CHECK(lens.refit_max_error_px < 0.5);
    // Its image circle meets the frame edge just past 100 degrees off-axis.
    CHECK(lens.refit_max_theta_deg > 98.0 && lens.refit_max_theta_deg < 104.0);
    CHECK_NEAR(lens.p1, -0.00246698, 1e-12);
    CHECK_NEAR(lens.p2, 0.00143700, 1e-12);
}

TEST(parse_offset_v3_rejects_malformed_input) {
    CHECK(parse_offset_v3("", 100, 100).empty());
    // A v2-sized (19-field) group is too short for v3.
    CHECK(parse_offset_v3("1_0_1_1_1_1_1_1_1_1_1_1_1_1_1_1_1_10_10_0", 100, 100).empty());
    // Full group, but canvas_w = canvas_h = 0.
    std::string group = v3_group("1440", "0.1_0.2_0.3_0");
    group.replace(group.find("5760_2880"), 9, "0_0");
    CHECK(parse_offset_v3("1_" + group, 100, 100).empty());
}
