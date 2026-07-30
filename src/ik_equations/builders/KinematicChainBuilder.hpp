#pragma once

#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/model/RobotDescription.hpp"

#include <string>

namespace kinemaforge::ik {

class KinematicChainBuilder
{
public:
    KinematicChain build(
        const RobotDescription& robot,
        const std::string& baseLink,
        const std::string& toolLink
    ) const;
};

} // namespace kinemaforge::ik
