#pragma once

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
};

} // namespace kinemaforge::ik
