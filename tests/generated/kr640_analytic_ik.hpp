#pragma once

#include "ik_equations/codegen/GeneratedIkRuntime.hpp"

namespace kr640_generated {

namespace rt = kinemaforge::ik::generated;
using Pose = rt::Pose;
using Result = rt::Result;
using Status = rt::Status;
using Branch = rt::Branch;
using CompactResult = rt::CompactResult;

inline constexpr rt::Parameters parameters{
    .axes = {
        rt::Axis{.point = {0, 0, 0.75}, .direction = {0, 0, 1}},
        rt::Axis{.point = {0.34999999999999998, 0, 0.75}, .direction = {0, 1, 0}},
        rt::Axis{.point = {0.34999999999999998, 0, 1.8999999999999999}, .direction = {0, 1, 0}},
        rt::Axis{.point = {1.6000000000000001, 0, 2.0449999999999999}, .direction = {1, 0, 0}},
        rt::Axis{.point = {1.6000000000000001, 0, 2.0449999999999999}, .direction = {0, 1, 0}},
        rt::Axis{.point = {1.6000000000000001, 0, 2.0449999999999999}, .direction = {1, 0, 0}}
    },
    .wristCenterHome = {1.6000000000000001, 0, 2.0449999999999999},
    .wristInTargetFrame = {0, 0, -0.29000000000000004},
    .targetFrameHomeRotation = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}},
    .planarX = {0, 0, 1},
    .planarY = {1, 0, 0},
    .link1 = 1.1499999999999999,
    .link2 = 1.2583818975176018,
    .link2HomeAngle = 1.4553124647348641,
    .axis3Sign = 1,
    .wristBasis = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}},
    .axis6Sign = 1,
    .limits = {
        rt::JointLimit{.enabled = true, .lower = -3.2288000000000001, .upper = 3.2288000000000001},
        rt::JointLimit{.enabled = true, .lower = -2.0943999999999998, .upper = 0.6109},
        rt::JointLimit{.enabled = true, .lower = -2.0943999999999998, .upper = 2.7576000000000001},
        rt::JointLimit{.enabled = true, .lower = -6.1086999999999998, .upper = 6.1086999999999998},
        rt::JointLimit{.enabled = true, .lower = -2.1817000000000002, .upper = 2.1817000000000002},
        rt::JointLimit{.enabled = true, .lower = -6.1086999999999998, .upper = 6.1086999999999998}
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

} // namespace kr640_generated
