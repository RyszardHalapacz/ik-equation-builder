#pragma once

#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/model/KinematicChainError.hpp"
#include "ik_equations/model/RobotDescription.hpp"

#include <expected>
#include <string>

namespace kinemaforge::ik {

// Preserves topology only. Performs no symbolic or geometric computation.
class KinematicChainBuilder
{
public:
    std::expected<KinematicChain, KinematicChainError> build(
        const RobotDescription& robot,
        const std::string& baseLink,
        const std::string& toolLink
    ) const;
};

} // namespace kinemaforge::ik
