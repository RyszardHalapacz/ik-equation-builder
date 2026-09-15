#pragma once

#include "ik_equations/model/Vector3.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace kinemaforge::ik {

enum class IkPatternKind
{
    SphericalWrist,
    ParallelShoulderElbow,
    SixRotarySphericalWrist
};

struct PatternEvidence
{
    double maximumAxisResidual{};
    std::optional<Vector3> wristCenter;
};

struct DetectedIkPattern
{
    IkPatternKind kind{};
    std::vector<std::size_t> actuatedJointPositions;
    PatternEvidence evidence;
};

} // namespace kinemaforge::ik
