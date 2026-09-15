#pragma once

#include "ik_equations/model/IkPattern.hpp"
#include "ik_equations/model/KinematicGeometry.hpp"

#include <vector>

namespace kinemaforge::ik {

class IkPatternDetector
{
public:
    [[nodiscard]] std::vector<DetectedIkPattern> detect(const KinematicGeometry& geometry) const;
};

} // namespace kinemaforge::ik
