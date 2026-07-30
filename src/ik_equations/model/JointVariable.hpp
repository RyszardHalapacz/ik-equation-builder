#pragma once

#include <cstddef>
#include <string>

namespace kinemaforge::ik {

struct JointVariable
{
    std::string name;
    std::size_t index{};
};

} // namespace kinemaforge::ik
