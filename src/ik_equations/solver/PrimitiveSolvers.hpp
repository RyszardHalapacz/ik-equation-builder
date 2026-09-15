#pragma once

#include "ik_equations/model/AnalyticIkSolution.hpp"

#include <vector>

namespace kinemaforge::ik {

inline constexpr double kPrimitiveSolverTolerance = 1e-12;

enum class ScalarSolutionKind
{
    Finite,
    None,
    Infinite
};

struct ScalarSolutions
{
    ScalarSolutionKind kind{ScalarSolutionKind::None};
    std::vector<double> values;
};

struct Planar2RBranch
{
    double firstJoint{};
    double secondJoint{};
    ElbowBranch elbow{ElbowBranch::Positive};
    bool singular{};
};

[[nodiscard]] double normalizeAngle(double angle) noexcept;

// a*x + b = 0
[[nodiscard]] ScalarSolutions solveLinear(double a, double b,
                                          double tolerance = kPrimitiveSolverTolerance);

// sin(x) = value / cos(x) = value, all distinct solutions modulo 2*pi.
[[nodiscard]] ScalarSolutions solveSine(double value,
                                        double tolerance = kPrimitiveSolverTolerance);
[[nodiscard]] ScalarSolutions solveCosine(double value,
                                          double tolerance = kPrimitiveSolverTolerance);

// a*sin(x) + b*cos(x) = c
[[nodiscard]] ScalarSolutions solveSinCos(double a, double b, double c,
                                          double tolerance = kPrimitiveSolverTolerance);

// A planar 2R arm written in its zero-configuration plane.  Link 1 starts on
// +X.  link2HomeAngle is the angle of link 2 relative to link 1 when q2 == 0.
// secondJointDirectionSign is +1 when positive q2 rotates around the same plane
// normal as q1 and -1 when the physical second axis is anti-parallel.
[[nodiscard]] std::vector<Planar2RBranch> solvePlanar2R(
    double targetX,
    double targetY,
    double link1Length,
    double link2Length,
    double link2HomeAngle,
    double secondJointDirectionSign = 1.0,
    double tolerance = kPrimitiveSolverTolerance);

} // namespace kinemaforge::ik
