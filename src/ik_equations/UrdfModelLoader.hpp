#pragma once

#include "ik_equations/model/RobotDescription.hpp"

#include <filesystem>

namespace kinemaforge::ik {

class UrdfModelLoader
{
public:
    RobotDescription load(const std::filesystem::path& urdfPath) const;
};

} // namespace kinemaforge::ik
