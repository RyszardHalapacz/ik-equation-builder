#pragma once

#include "ik_equations/model/Vector3.hpp"

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

struct UrdfJoint
{
    std::string name;

    std::string parentLink;
    std::string childLink;

    JointType type{JointType::Fixed};

    JointOrigin origin;
    Vector3 axis;

    JointLimits limits;
};

struct UrdfLink
{
    std::string name;
};

} // namespace kinemaforge::ik
