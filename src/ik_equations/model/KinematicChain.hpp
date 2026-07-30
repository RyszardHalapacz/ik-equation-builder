#pragma once

#include "ik_equations/model/JointVariable.hpp"
#include "ik_equations/model/UrdfJoint.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace kinemaforge::ik {

struct KinematicJoint
{
    std::size_t index{};

    std::string name;
    std::string parentLink;
    std::string childLink;

    JointType type{JointType::Fixed};

    JointOrigin origin;
    Vector3 axis;
    JointLimits limits;

    std::optional<JointVariable> variable;
};

struct KinematicChain
{
    std::string baseLink;
    std::string toolLink;

    std::vector<KinematicJoint> joints;
};

} // namespace kinemaforge::ik
