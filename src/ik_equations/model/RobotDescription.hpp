#pragma once

#include "ik_equations/model/UrdfJoint.hpp"

#include <string>
#include <vector>

namespace kinemaforge::ik {

struct RobotDescription
{
    std::string name;

    std::vector<UrdfLink> links;
    std::vector<UrdfJoint> joints;
};

} // namespace kinemaforge::ik
