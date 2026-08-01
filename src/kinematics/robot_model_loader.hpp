#pragma once
#include <cstdint>
#include <expected>
#include <filesystem>
#include <optional>
#include <string>
#include "common/diagnostic_bag.hpp"
#include "kinematics/robot_model.hpp"

namespace mt::kinematics {

enum class LoadErrorCode : std::uint8_t {
    file_not_found,
    parse_failure,
    unsupported_joint_type,
    incomplete_kinematic_chain,
    invalid_limits,
    malformed_vector,   // attribute present but not three finite numbers
    degenerate_axis,    // actuated joint whose axis has zero length
};

// Code plus diagnostic context. The code alone is not enough: "malformed
// vector" in a file with thirty joints tells the user nothing useful.
//
// rawValue is optional rather than a plain string because an empty
// attribute (xyz="") is itself a reportable value, and must stay
// distinguishable from "no raw value applies to this error".
struct LoadError {
    LoadErrorCode              code{};
    std::string                jointName;   // empty when not joint-specific
    std::string                attribute;   // e.g. "origin/xyz", "axis/xyz"
    std::optional<std::string> rawValue;    // raw attribute text, truncated
};

struct LoadResult {
    RobotModel    model;
    DiagnosticBag diagnostics;
};

// On success the geometry is canonical:
//   * every joint's origin_xyz and origin_rpy hold only finite values
//   * an actuated joint's axis is finite, non-zero and numerically
//     normalized — exactly +/-1 and 0 for axis-aligned input, 1.0 within a
//     few ULP otherwise
//   * a fixed joint's axis carries no meaning
[[nodiscard]] std::expected<LoadResult, LoadError>
load_urdf(std::filesystem::path const& urdf_path);

}  // namespace mt::kinematics

// REVIEWED: modern C++ (C++23-first); deviations justified inline.
