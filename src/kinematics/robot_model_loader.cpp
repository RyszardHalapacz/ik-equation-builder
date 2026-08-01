#include "kinematics/robot_model_loader.hpp"
#include <pugixml.hpp>
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstring>
#include <optional>
#include <string_view>
#include <unordered_map>

namespace mt::kinematics {
namespace {

// Longest untrusted text carried into an error. A well-formed vector of
// three full-precision doubles fits in ~70 characters, so this leaves room
// to see the context of a mistake without copying an arbitrarily long
// attribute — or joint name — into an exception message.
constexpr std::size_t kMaxDiagnosticLength = 256;

std::string truncated(std::string_view text) {
    if (text.size() <= kMaxDiagnosticLength) return std::string{text};
    return std::string{text.substr(0, kMaxDiagnosticLength)} + "...";
}

// rawValue is passed as an optional so that an empty attribute (xyz="")
// stays distinguishable from an error that carries no raw value at all.
LoadError makeError(LoadErrorCode code,
                    std::string_view jointName = {},
                    std::string_view attribute = {},
                    std::optional<std::string_view> rawValue = std::nullopt) {
    LoadError error;
    error.code      = code;
    error.jointName = truncated(jointName);   // joint names come from the
    error.attribute = attribute;              // same untrusted XML
    if (rawValue) error.rawValue = truncated(*rawValue);
    return error;
}

// Both parsers stay noexcept: they return plain values and never build a
// LoadError, so no allocation happens on their failure path. Context is
// attached by the caller, which is not noexcept.
std::optional<JointType> parse_joint_type(std::string_view s) noexcept {
    if (s == "revolute")  return JointType::revolute;
    if (s == "prismatic") return JointType::prismatic;
    if (s == "fixed")     return JointType::fixed;
    return std::nullopt;
}

constexpr bool is_space(char c) noexcept {
    return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

// Accepts exactly three finite numbers separated by whitespace:
//
//     whitespace* number whitespace+ number whitespace+ number whitespace*
//
// Rejects a wrong count, trailing garbage, non-finite values ("nan", "inf")
// and out-of-range literals ("1e400"). std::from_chars refuses a leading
// '+', so one is skipped by hand — but at most one, and it may not be
// followed by another sign, otherwise "+-1" would slip through as -1.
std::optional<Vec3> parse_vector3(std::string_view text) noexcept {
    if (text.empty()) return std::nullopt;   // present but empty attribute

    char const* p   = text.data();
    char const* end = p + text.size();

    double components[3]{};
    for (int i = 0; i < 3; ++i) {
        // Numbers must be separated by whitespace: "1-2 3" is not a vector.
        if (i > 0 && (p == end || !is_space(*p))) return std::nullopt;
        while (p < end && is_space(*p)) ++p;
        if (p == end) return std::nullopt;

        if (*p == '+') {
            ++p;
            if (p == end || *p == '+' || *p == '-') return std::nullopt;
        }

        auto const [next, ec] = std::from_chars(p, end, components[i]);
        if (ec != std::errc{})             return std::nullopt;
        if (!std::isfinite(components[i])) return std::nullopt;
        p = next;
    }

    while (p < end && is_space(*p)) ++p;
    if (p != end) return std::nullopt;

    return Vec3{components[0], components[1], components[2]};
}

// Scaling by the largest component first is required, not an optimisation:
// std::hypot alone loses the answer deep in the subnormal range, where the
// true length is not representable. For [denorm_min, denorm_min, 0] it
// returns denorm_min, and dividing by it yields [1, 1, 0] — length sqrt(2).
//
// After scaling, one component is exactly +/-1, so the second length lies
// in [1, sqrt(3)]: far from both ends of the double range.
std::optional<Vec3> normalize_axis(Vec3 axis) noexcept {
    double const scale = std::max({std::abs(axis.x), std::abs(axis.y), std::abs(axis.z)});
    if (scale == 0.0) return std::nullopt;

    double const sx = axis.x / scale;
    double const sy = axis.y / scale;
    double const sz = axis.z / scale;
    double const norm = std::hypot(sx, sy, sz);

    return Vec3{sx / norm, sy / norm, sz / norm};
}

// Traces a linear kinematic chain (parent->child). Collects actuated joint indices.
// Emits Info for each skipped fixed joint. Emits Warning if chain < 6 actuated joints.
std::expected<std::vector<std::size_t>, LoadError>
trace_chain(std::span<Joint const> joints, mt::DiagnosticBag& diag) {
    std::unordered_map<std::string, std::size_t> parent_to_idx;
    std::unordered_map<std::string, bool>        is_child;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        parent_to_idx[joints[i].parent_link] = i;
        is_child[joints[i].child_link]       = true;
    }

    // Root link: appears as a parent but never as a child of any joint.
    std::string current;
    for (auto const& [parent, _] : parent_to_idx) {
        if (!is_child.contains(parent)) { current = parent; break; }
    }
    if (current.empty())
        return std::unexpected(makeError(LoadErrorCode::incomplete_kinematic_chain));

    std::vector<std::size_t> chain;
    while (parent_to_idx.contains(current)) {
        auto const  idx = parent_to_idx.at(current);
        auto const& j   = joints[idx];
        if (j.type == JointType::fixed) {
            diag.add(mt::DiagnosticSeverity::Info,
                     mt::DiagnosticCode::Kinematics_InvalidRobotModel,
                     {}, "fixed joint skipped in kinematic chain: " + j.name);
        } else {
            chain.push_back(idx);
        }
        current = j.child_link;
    }

    if (chain.empty())
        return std::unexpected(makeError(LoadErrorCode::incomplete_kinematic_chain));

    if (chain.size() < 6) {
        diag.add(mt::DiagnosticSeverity::Warning,
                 mt::DiagnosticCode::Kinematics_InvalidRobotModel,
                 {}, "kinematic chain has " + std::to_string(chain.size()) +
                     " actuated joints (expected >=6)");
    }
    return chain;
}

}  // namespace

[[nodiscard]] std::expected<LoadResult, LoadError>
load_urdf(std::filesystem::path const& urdf_path) {
    if (!std::filesystem::exists(urdf_path))
        return std::unexpected(makeError(LoadErrorCode::file_not_found));

    pugi::xml_document doc;
    if (!doc.load_file(urdf_path.c_str()))
        return std::unexpected(makeError(LoadErrorCode::parse_failure));

    auto robot = doc.child("robot");
    if (!robot) return std::unexpected(makeError(LoadErrorCode::parse_failure));

    LoadResult out;
    out.model.set_name(robot.attribute("name").as_string());

    for (auto node : robot.children("link"))
        out.model.add_link(Link{.name = node.attribute("name").as_string()});

    for (auto node : robot.children("joint")) {
        Joint j;
        j.name = node.attribute("name").as_string();

        auto jtype = parse_joint_type(node.attribute("type").as_string());
        if (!jtype)
            return std::unexpected(makeError(LoadErrorCode::unsupported_joint_type,
                                             j.name, "type",
                                             node.attribute("type").as_string()));
        j.type = *jtype;

        // A missing element or a missing attribute means "use the default".
        // An attribute that is present but unparsable is an error — including
        // an empty one, which pugixml cannot distinguish from a missing
        // attribute through as_string(), hence the explicit truthiness test.
        if (auto origin = node.child("origin"); origin) {
            if (auto attribute = origin.attribute("xyz")) {
                auto parsed = parse_vector3(attribute.value());
                if (!parsed)
                    return std::unexpected(makeError(LoadErrorCode::malformed_vector,
                                                     j.name, "origin/xyz",
                                                     attribute.value()));
                j.origin_xyz = *parsed;
            }
            if (auto attribute = origin.attribute("rpy")) {
                auto parsed = parse_vector3(attribute.value());
                if (!parsed)
                    return std::unexpected(makeError(LoadErrorCode::malformed_vector,
                                                     j.name, "origin/rpy",
                                                     attribute.value()));
                j.origin_rpy = *parsed;
            }
        }

        // URDF defaults <axis> to (1,0,0). Syntax is validated for every
        // joint type; only the semantics (non-degenerate) are skipped for
        // fixed joints, whose axis takes no part in the kinematics.
        std::string_view rawAxis;
        j.axis = Vec3{1.0, 0.0, 0.0};
        if (auto axis = node.child("axis"); axis) {
            if (auto attribute = axis.attribute("xyz")) {
                rawAxis = attribute.value();
                auto parsed = parse_vector3(rawAxis);
                if (!parsed)
                    return std::unexpected(makeError(LoadErrorCode::malformed_vector,
                                                     j.name, "axis/xyz", rawAxis));
                j.axis = *parsed;
            }
        }
        if (j.type != JointType::fixed) {
            auto normalized = normalize_axis(j.axis);
            if (!normalized)
                return std::unexpected(makeError(LoadErrorCode::degenerate_axis,
                                                 j.name, "axis/xyz", rawAxis));
            j.axis = *normalized;
        }

        j.parent_link = node.child("parent").attribute("link").as_string();
        j.child_link  = node.child("child").attribute("link").as_string();

        if (j.type != JointType::fixed) {
            auto lim = node.child("limit");
            if (!lim)
                return std::unexpected(makeError(LoadErrorCode::invalid_limits, j.name, "limit"));
            j.limits.lower    = lim.attribute("lower").as_double();
            j.limits.upper    = lim.attribute("upper").as_double();
            j.limits.velocity = lim.attribute("velocity").as_double();
            j.limits.effort   = lim.attribute("effort").as_double();
            if (!std::isfinite(j.limits.lower) || !std::isfinite(j.limits.upper) ||
                j.limits.lower > j.limits.upper)
                return std::unexpected(makeError(LoadErrorCode::invalid_limits, j.name, "limit"));
        }
        out.model.add_joint(std::move(j));
    }

    auto chain = trace_chain(out.model.joints(), out.diagnostics);
    if (!chain) return std::unexpected(chain.error());
    out.model.set_chain(std::move(*chain));

    return out;
}

}  // namespace mt::kinematics
