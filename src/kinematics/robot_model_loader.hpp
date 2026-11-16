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
    malformed_vector,   // attribute present but not three finite numbers
    degenerate_axis,    // actuated joint whose axis has zero length

    // Joint limits. See the strict profile documented on load_urdf below.
    missing_limit_element,     // revolute/prismatic without <limit>
    duplicate_limit_element,   // more than one <limit> in one joint
    missing_limit_attribute,   // a required <limit> attribute is absent
    malformed_limit_value,     // present, but not one finite number
    invalid_limit_constraint,  // lower > upper, or velocity/effort < 0

    // Topology. A URDF describes a tree; these are the ways it can fail to.
    invalid_link_declaration,  // <link> with an empty or duplicate name
    undeclared_parent_link,    // joint parent has no matching <link>
    undeclared_child_link,     // joint child has no matching <link>
    duplicate_child_link,      // one link is the child of two joints
    cyclic_topology,           // the graph contains a cycle
    disconnected_topology,     // not one connected component / not one root

    // Joint identity and semantics.
    invalid_joint_declaration, // <joint> with an empty or duplicate name
    unsupported_mimic_joint,   // <mimic>: the joint is not an independent axis
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
//
// ...and the topology is a well-formed tree:
//   * every <link> name is non-empty and unique — link names are node identity
//   * every <joint> name is non-empty and unique — joint names identify axes
//     in IK results
//   * every joint parent/child reference resolves to a declared <link>
//   * no link is the child of two joints
//   * there is exactly one root, the graph is connected, and it has no cycle
//
// The result therefore does NOT depend on the order of <joint> or <link>
// elements in the file. It used to: a previous version traced a single linear
// chain, so the same robot loaded or failed depending on which of two joints
// sharing a parent appeared last.
//
// ...and joint limits satisfy the KinemaForge strict profile:
//   * a revolute or prismatic joint carries EXACTLY ONE <limit> element
//   * that element has all four of lower, upper, velocity and effort
//   * each value is exactly one finite number — "1 2" and "nan" are rejected,
//     and a missing attribute is an error rather than a zero
//   * lower <= upper, velocity >= 0, effort >= 0
//   * a <limit> on a fixed joint is ignored, contents and all
//
// This is deliberately narrower than older URDF, where lower and upper default
// to 0, and it is NOT the URDF 1.2 rule, where effort and velocity are optional.
// It is this project's own profile: the loader does not read <robot version>
// and claims conformance to no particular release. A joint whose limits default
// to [0, 0] cannot be told apart from a mistake, and this project derives IK
// equations from those numbers.
//
// A joint carrying <mimic> is rejected (unsupported_mimic_joint): it is a
// function of another joint, not an independent axis, and the model has no
// representation for that coupling yet.
//
// Choosing a kinematic chain is deliberately NOT done here — that needs a base
// and a tool link, which only the caller knows. See KinematicChainBuilder.
[[nodiscard]] std::expected<LoadResult, LoadError>
load_urdf(std::filesystem::path const& urdf_path);

}  // namespace mt::kinematics

// REVIEWED: modern C++ (C++23-first); deviations justified inline.
