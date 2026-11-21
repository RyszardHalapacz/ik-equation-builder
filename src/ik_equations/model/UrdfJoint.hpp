#pragma once

#include "ik_equations/model/Vector3.hpp"

#include <optional>
#include <string>

namespace kinemaforge::ik {

enum class JointType
{
    Fixed,
    Revolute,
    Continuous,
    Prismatic
};

struct JointOrigin
{
    Vector3 translation;
    Vector3 rpy;
};

struct JointLimits
{
    double lower{};
    double upper{};
    double velocity{};
    double effort{};

    bool hasPositionLimits{};
};

// URDF <mimic>: value = multiplier * value(joint) + offset. Such a joint is
// not an independent axis; KinematicChainBuilder refuses a chain through it.
struct JointMimic
{
    std::string joint;
    double multiplier{1.0};
    double offset{};
};

struct UrdfJoint
{
    std::string name;

    std::string parentLink;
    std::string childLink;

    JointType type{JointType::Fixed};

    JointOrigin origin;
    Vector3 axis;

    JointLimits limits;

    std::optional<JointMimic> mimic;
};

struct UrdfLink
{
    std::string name;
};

} // namespace kinemaforge::ik
