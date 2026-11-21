#pragma once

#include <array>
#include <cstddef>

namespace kinemaforge::ik {

struct PositionalLimit
{
    bool enabled{};
    double lower{};
    double upper{};
};

// A positional singular family: one arm joint is free and the wrist centre
// does not move with it.
//
//   freeJoint == 0  wrist centre on the joint-1 axis (IkSingularityKind::ShoulderAxis)
//   freeJoint == 1  equal arm links, elbow fully folded (IkSingularityKind::ElbowFolded)
//
// The other two arm joints are fixed; q4..q6 follow from the wrist
// decomposition and depend on the free value NONLINEARLY, so the family is
// described by the data needed to evaluate a member, not by a formula.
// evaluate(family, value) (AnalyticIkSolver.hpp / GeneratedIkRuntime.hpp)
// returns the member for a value in [freeLower, freeUpper], or nothing where
// that member leaves a wrist limit. Pure data, shared by both solvers.
struct PositionalFamily
{
    std::size_t freeJoint{};
    double freeLower{};
    double freeUpper{};
    std::array<double, 3> arm{};     // q1, q2, q3; arm[freeJoint] is replaced by the free value
    std::size_t wristBranch{};       // 0: primary decomposition (q5 >= 0), 1: flipped
    std::array<std::array<double, 3>, 3> axisDirections{}; // unit axes of joints 1..3 (base frame)
    std::array<std::array<double, 3>, 3> targetInHome{};   // target rotation * home rotation^T
    std::array<std::array<double, 3>, 3> wristBasis{};     // columns: wrist basis vectors
    double axis6Sign{1.0};
    std::array<PositionalLimit, 6> limits{};
};

} // namespace kinemaforge::ik
