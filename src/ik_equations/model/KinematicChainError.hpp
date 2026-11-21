#pragma once

namespace kinemaforge::ik {

enum class KinematicChainError
{
    BaseLinkNotFound,
    ToolLinkNotFound,
    NoPathFound,
    InvalidRobotDescription,
    // The path from base to tool passes through a <mimic> joint. Every chain
    // consumer treats each joint as an independent axis, so the coupling would
    // be silently ignored.
    MimicJointInChain
};

} // namespace kinemaforge::ik
