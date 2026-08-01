#pragma once

namespace kinemaforge::ik {

enum class KinematicChainError
{
    BaseLinkNotFound,
    ToolLinkNotFound,
    NoPathFound,
    InvalidRobotDescription
};

} // namespace kinemaforge::ik
