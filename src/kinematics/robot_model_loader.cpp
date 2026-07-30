#include "kinematics/robot_model_loader.hpp"
#include <pugixml.hpp>
#include <charconv>
#include <cmath>
#include <cstring>
#include <unordered_map>

namespace mt::kinematics {
namespace {

std::expected<JointType, LoadError> parse_joint_type(std::string_view s) noexcept {
    if (s == "revolute")  return JointType::revolute;
    if (s == "prismatic") return JointType::prismatic;
    if (s == "fixed")     return JointType::fixed;
    return std::unexpected(LoadError::unsupported_joint_type);
}

// Parses space-separated "x y z" from a null-terminated URDF attribute value.
Vec3 parse_xyz(char const* attr) noexcept {
    Vec3 v{};
    if (!attr || !*attr) return v;
    char const* p   = attr;
    char const* end = p + std::strlen(p);
    double* dst[3]  = {&v.x, &v.y, &v.z};
    for (int i = 0; i < 3; ++i) {
        while (p < end && *p == ' ') ++p;
        auto [next, ec] = std::from_chars(p, end, *dst[i]);
        if (ec != std::errc{}) break;
        p = next;
    }
    return v;
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
    if (current.empty()) return std::unexpected(LoadError::incomplete_kinematic_chain);

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

    if (chain.empty()) return std::unexpected(LoadError::incomplete_kinematic_chain);

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
        return std::unexpected(LoadError::file_not_found);

    pugi::xml_document doc;
    if (!doc.load_file(urdf_path.c_str()))
        return std::unexpected(LoadError::parse_failure);

    auto robot = doc.child("robot");
    if (!robot) return std::unexpected(LoadError::parse_failure);

    LoadResult out;
    out.model.set_name(robot.attribute("name").as_string());

    for (auto node : robot.children("link"))
        out.model.add_link(Link{.name = node.attribute("name").as_string()});

    for (auto node : robot.children("joint")) {
        auto jtype = parse_joint_type(node.attribute("type").as_string());
        if (!jtype) return std::unexpected(LoadError::unsupported_joint_type);

        Joint j;
        j.name = node.attribute("name").as_string();
        j.type = *jtype;

        if (auto origin = node.child("origin"); origin) {
            j.origin_xyz = parse_xyz(origin.attribute("xyz").as_string());
            j.origin_rpy = parse_xyz(origin.attribute("rpy").as_string());
        }
        if (auto axis = node.child("axis"); axis)
            j.axis = parse_xyz(axis.attribute("xyz").as_string());

        j.parent_link = node.child("parent").attribute("link").as_string();
        j.child_link  = node.child("child").attribute("link").as_string();

        if (j.type != JointType::fixed) {
            auto lim = node.child("limit");
            if (!lim) return std::unexpected(LoadError::invalid_limits);
            j.limits.lower    = lim.attribute("lower").as_double();
            j.limits.upper    = lim.attribute("upper").as_double();
            j.limits.velocity = lim.attribute("velocity").as_double();
            j.limits.effort   = lim.attribute("effort").as_double();
            if (!std::isfinite(j.limits.lower) || !std::isfinite(j.limits.upper) ||
                j.limits.lower > j.limits.upper)
                return std::unexpected(LoadError::invalid_limits);
        }
        out.model.add_joint(std::move(j));
    }

    auto chain = trace_chain(out.model.joints(), out.diagnostics);
    if (!chain) return std::unexpected(chain.error());
    out.model.set_chain(std::move(*chain));

    return out;
}

}  // namespace mt::kinematics
