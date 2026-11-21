#pragma once

#include "ik_equations/codegen/GeneratedIkRuntime.hpp"

namespace kr4_generated {

namespace rt = kinemaforge::ik::generated;
using Pose = rt::Pose;
using Result = rt::Result;
using Status = rt::Status;
using Branch = rt::Branch;
using CompactResult = rt::CompactResult;

inline constexpr rt::Parameters parameters{
    .axes = {
        rt::Axis{.point = {0, 0, 0.14940000000000001}, .direction = {0, -1.2246063538223773e-16, -1}},
        rt::Axis{.point = {0, 0.063600000000000032, 0.33000000000000002}, .direction = {0, 1, -1.8369095307335659e-16}},
        rt::Axis{.point = {0.28999999999999998, 0.059000000000000032, 0.33000000000000002}, .direction = {0, 1, -1.8369095307335659e-16}},
        rt::Axis{.point = {0.45939999999999998, 4.163336342344337e-17, 0.35000000000000003}, .direction = {-1, 6.1230317691118875e-17, 6.1230317691118851e-17}},
        rt::Axis{.point = {0.59999999999999998, 0.045000000000000033, 0.35000000000000003}, .direction = {0, 1, -1.2246063538223773e-16}},
        rt::Axis{.point = {0.64649999999999996, 3.4694469519536142e-17, 0.35000000000000003}, .direction = {-1, 6.1230317691118888e-17, 1.2246063538223773e-16}}
    },
    .wristCenterHome = {0.59999999999999998, 3.3859425137804098e-17, 0.35000000000000003},
    .wristInTargetFrame = {-3.0333993731961704e-22, 5.502293499298754e-18, -0.074999999999999956},
    .targetFrameHomeRotation = {{{4.0445324975948962e-21, -6.1230317691118851e-17, 1}, {-7.4985512571204294e-33, 1, 6.1230317691118838e-17}, {-1, -9.6552960699352445e-33, 4.0445324975948962e-21}}},
    .planarX = {1, 0, -2.9137185659911737e-18},
    .planarY = {-2.9137185659911737e-18, -1.8369095307335659e-16, -1},
    .link1 = 0.28999999999999998,
    .link2 = 0.31064449134018129,
    .link2HomeAngle = -0.064426839425553578,
    .axis3Sign = 1,
    .wristBasis = {{{-1, 0, -6.1230317691118863e-17}, {6.1230317691118875e-17, 1, -1.2246063538223773e-16}, {6.1230317691118851e-17, -1.2246063538223773e-16, -1}}},
    .axis6Sign = 1,
    .limits = {
        rt::JointLimit{.enabled = true, .lower = -2.9670597249999999, .upper = 2.9670597249999999},
        rt::JointLimit{.enabled = true, .lower = -3.4033920375000002, .upper = 0.69813170000000002},
        rt::JointLimit{.enabled = true, .lower = -2.0071286375000001, .upper = 2.6179938749999998},
        rt::JointLimit{.enabled = true, .lower = -3.2288591124999999, .upper = 3.2288591124999999},
        rt::JointLimit{.enabled = true, .lower = -2.0943950999999998, .upper = 2.0943950999999998},
        rt::JointLimit{.enabled = true, .lower = -6.1086523750000001, .upper = 6.1086523750000001}
    }
};

[[nodiscard]] inline Result solve(const Pose& target)
{
    return rt::solve(parameters, target);
}

// All solutions in compact form (per joint: its in-limit full turns); no
// branch budget. rt::expand() gives what solve() returns.
[[nodiscard]] inline CompactResult solveCompact(const Pose& target)
{
    return rt::solveCompact(parameters, target);
}

} // namespace kr4_generated
