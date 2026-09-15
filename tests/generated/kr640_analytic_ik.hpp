#pragma once

#include "ik_equations/codegen/GeneratedIkRuntime.hpp"

namespace kr640_generated {

namespace rt = kinemaforge::ik::generated;
using Pose = rt::Pose;
using Result = rt::Result;
using Status = rt::Status;
using Branch = rt::Branch;

inline constexpr rt::Parameters parameters{
    {
        rt::Axis{{0, 0, 0.75}, {0, 0, 1}},
        rt::Axis{{0.34999999999999998, 0, 0.75}, {0, 1, 0}},
        rt::Axis{{0.34999999999999998, 0, 1.8999999999999999}, {0, 1, 0}},
        rt::Axis{{1.6000000000000001, 0, 2.0449999999999999}, {1, 0, 0}},
        rt::Axis{{1.6000000000000001, 0, 2.0449999999999999}, {0, 1, 0}},
        rt::Axis{{1.6000000000000001, 0, 2.0449999999999999}, {1, 0, 0}}
    },
    {1.6000000000000001, 0, 2.0449999999999999},
    {0, 0, -0.29000000000000004},
    {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}},
    {0, 0, 1},
    {1, 0, 0},
    1.1499999999999999,
    1.2583818975176018,
    1.4553124647348641,
    1,
    {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}},
    1,
    {
        rt::JointLimit{true, -3.2288000000000001, 3.2288000000000001},
        rt::JointLimit{true, -2.0943999999999998, 0.6109},
        rt::JointLimit{true, -2.0943999999999998, 2.7576000000000001},
        rt::JointLimit{true, -6.1086999999999998, 6.1086999999999998},
        rt::JointLimit{true, -2.1817000000000002, 2.1817000000000002},
        rt::JointLimit{true, -6.1086999999999998, 6.1086999999999998}
    }
};

[[nodiscard]] inline Result solve(const Pose& target)
{
    return rt::solve(parameters, target);
}

} // namespace kr640_generated
