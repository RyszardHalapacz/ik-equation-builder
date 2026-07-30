#pragma once
#include <cstdint>
#include <expected>
#include <filesystem>
#include "common/diagnostic_bag.hpp"
#include "kinematics/robot_model.hpp"

namespace mt::kinematics {

enum class LoadError : std::uint8_t {
    file_not_found,
    parse_failure,
    unsupported_joint_type,
    incomplete_kinematic_chain,
    invalid_limits,
};

struct LoadResult {
    RobotModel    model;
    DiagnosticBag diagnostics;
};

[[nodiscard]] std::expected<LoadResult, LoadError>
load_urdf(std::filesystem::path const& urdf_path);

}  // namespace mt::kinematics

// REVIEWED: modern C++ (C++23-first); deviations justified inline.
