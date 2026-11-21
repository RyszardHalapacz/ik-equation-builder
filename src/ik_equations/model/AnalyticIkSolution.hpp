#pragma once

#include "ik_equations/model/PositionalFamily.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace kinemaforge::ik {

enum class ShoulderBranch
{
    Primary,
    Alternate
};

enum class ElbowBranch
{
    Positive,
    Negative
};

enum class WristBranch
{
    Primary,
    Flip,
    Singular
};

enum class IkSingularityKind
{
    ShoulderAxis,
    ElbowFolded,
    ElbowExtended,
    WristAligned,
    WristAntiAligned
};

struct JointSolutionValue
{
    std::size_t actuatedJointPosition{};
    std::string jointName;
    double value{}; // radians for the first supported 6R family
};

// A free parameter is not filled with an arbitrary number.  The interval is
// the set of values that remain admissible after the target has been applied;
// downstream policy may choose one, but the solver must preserve the fact that
// the family is infinite.
struct FreeJointParameter
{
    std::size_t actuatedJointPosition{};
    std::string jointName;
    double lower{};
    double upper{};
};

struct CoupledFreeParameter
{
    std::size_t freeActuatedJointPosition{};
    std::string freeJointName;
    std::size_t dependentActuatedJointPosition{};
    std::string dependentJointName;
    // dependent = offset + multiplier * free, modulo 2*pi for rotary joints.
    double multiplier{};
    double offset{};
};

// A positional singular family (q1 or q2 free) with its labels. Members come
// from evaluate(family, value) in AnalyticIkSolver.hpp.
struct PositionalFamilyBranch
{
    PositionalFamily family;
    std::string freeJointName;
    ElbowBranch elbow{ElbowBranch::Positive};
    WristBranch wrist{WristBranch::Primary};
    IkSingularityKind singularity{IkSingularityKind::ShoulderAxis};
};

struct SolutionBranch
{
    std::vector<JointSolutionValue> joints;
    ShoulderBranch shoulder{ShoulderBranch::Primary};
    ElbowBranch elbow{ElbowBranch::Positive};
    WristBranch wrist{WristBranch::Primary};
    std::optional<IkSingularityKind> singularity;
    std::vector<FreeJointParameter> freeParameters;
    std::vector<CoupledFreeParameter> coupledFreeParameters;
};

enum class AnalyticIkStatus
{
    Solutions,
    NoSolution,
    InfiniteFamily,
    UnsupportedKinematicStructure,
    InvalidTarget
};

struct AnalyticIkSolution
{
    AnalyticIkStatus status{AnalyticIkStatus::NoSolution};
    std::vector<SolutionBranch> branches;
    std::string message;
    // Every branch reproduces the target position and the rotation NEAREST to
    // the target orientation. This is the largest element-wise distance between
    // that rotation and the orientation the caller passed in: ~1e-16 for a
    // clean matrix, up to kMaxOrientationCorrection for one accepted at the
    // edge of the validation band.
    double orientationCorrection{};
    // Families with a free arm joint (wrist centre on the joint-1 axis, or
    // equal links with the elbow folded). status is InfiniteFamily when any
    // exist; `branches` may then be empty.
    std::vector<PositionalFamilyBranch> positionalFamilies;
    // Distance of the wrist centre from the joint-1 axis, metres: a property of
    // the target, the same for every branch. 0 is the shoulder-axis singularity
    // (q1 free, see positionalFamilies); the solver switches to a family at
    // kSingularLinear. A caller that wants a wider safety band reads this.
    double shoulderAxisDistance{};
};

// |sin q5| of a branch: how far its wrist is from the wrist singularity.
// Exactly 0 for a wrist family (q4 and q6 coupled).
[[nodiscard]] double wristPitchSine(const SolutionBranch& branch);

} // namespace kinemaforge::ik
