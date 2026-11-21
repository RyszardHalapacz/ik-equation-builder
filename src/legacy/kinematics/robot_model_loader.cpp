#include "legacy/kinematics/robot_model_loader.hpp"
#include <pugixml.hpp>
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstring>
#include <optional>
#include <span>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

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
    if (s == "continuous") return JointType::continuous;
    return std::nullopt;
}

constexpr bool is_space(char c) noexcept {
    return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

// Parses one finite number starting at `p`, advancing it past the token.
//
// Deliberately does NOT skip surrounding whitespace and does NOT require
// reaching the end of the input: those are the callers' rules, and they differ.
// parse_scalar demands the end, parse_vector3 must be able to stop mid-string
// and keep reading. Folding either rule in here would make one caller wrong.
//
// std::from_chars refuses a leading '+', so one is skipped by hand — but at
// most one, and it may not be followed by another sign, otherwise "+-1" would
// slip through as -1.
bool parse_finite_number(char const*& p, char const* end, double& out) noexcept {
    if (p == end) return false;

    if (*p == '+') {
        ++p;
        if (p == end || *p == '+' || *p == '-') return false;
    }

    auto const [next, ec] = std::from_chars(p, end, out);
    if (ec != std::errc{})   return false;   // garbage, or out of range ("1e400")
    if (!std::isfinite(out)) return false;   // "nan", "inf"
    p = next;
    return true;
}

// Accepts exactly three finite numbers separated by whitespace:
//
//     whitespace* number whitespace+ number whitespace+ number whitespace*
//
// Rejects a wrong count, trailing garbage, non-finite values ("nan", "inf")
// and out-of-range literals ("1e400").
std::optional<Vec3> parse_vector3(std::string_view text) noexcept {
    if (text.empty()) return std::nullopt;   // present but empty attribute

    char const* p   = text.data();
    char const* end = p + text.size();

    double components[3]{};
    for (int i = 0; i < 3; ++i) {
        // Numbers must be separated by whitespace: "1-2 3" is not a vector.
        if (i > 0 && (p == end || !is_space(*p))) return std::nullopt;
        while (p < end && is_space(*p)) ++p;
        if (!parse_finite_number(p, end, components[i])) return std::nullopt;
    }

    while (p < end && is_space(*p)) ++p;
    if (p != end) return std::nullopt;

    return Vec3{components[0], components[1], components[2]};
}

// Exactly one finite number, with optional surrounding whitespace:
//
//     whitespace* number whitespace*
//
// The END requirement lives here rather than in parse_finite_number, and it is
// the whole point: without it lower="1 2" would quietly become 1.
std::optional<double> parse_scalar(std::string_view text) noexcept {
    if (text.empty()) return std::nullopt;   // present but empty attribute

    char const* p   = text.data();
    char const* end = p + text.size();

    while (p < end && is_space(*p)) ++p;

    double value = 0.0;
    if (!parse_finite_number(p, end, value)) return std::nullopt;

    while (p < end && is_space(*p)) ++p;
    if (p != end) return std::nullopt;       // trailing garbage, or a second number

    return value;
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

// Reads one required <limit> attribute under the strict profile. Absence and
// unparsable text are DIFFERENT errors: pugixml's as_double() cannot tell them
// apart — it answers 0.0 to both — which is what let lower="banana" and a
// missing lower produce the same silent zero.
std::expected<double, LoadError>
read_limit_attribute(pugi::xml_node limit, std::string_view jointName,
                     char const* name) {
    std::string const attribute = std::string("limit/") + name;

    auto const node = limit.attribute(name);
    if (!node)
        return std::unexpected(makeError(LoadErrorCode::missing_limit_attribute,
                                         jointName, attribute));

    auto const value = parse_scalar(node.value());
    if (!value)
        return std::unexpected(makeError(LoadErrorCode::malformed_limit_value,
                                         jointName, attribute, node.value()));

    return *value;
}

// Validates that the declared links and joints form one well-formed tree.
//
// Replaces trace_chain(), which assumed URDF describes a linear chain. URDF
// describes a TREE: one link may be the parent of several joints, and both
// robots in data/urdf are like that. Deciding which branch is "the" kinematic
// chain is not this layer's business -- it needs a base and a tool link, which
// only the caller knows. KinematicChainBuilder makes that choice and
// understands branching. This function only guarantees the graph is a tree.
//
// Invariants, checked in this order so each code keeps one meaning when several
// are broken at once (same discipline as IkEquationSystem::create):
//
//   T0a  every <link> has a non-empty name
//   T0b  <link> names are unique   -- link.name IS node identity here, so
//                                     without it the graph is not well defined
//                                     and nothing below means anything
//   T0c  every <joint> has a non-empty, unique name -- joint names identify
//        axes in IK results, so an empty or repeated one makes that mapping
//        ambiguous
//   T1   every joint.parent_link resolves to a declared <link>
//   T2   every joint.child_link resolves to a declared <link>
//   T3   no link is the child of two joints -- this is what makes it a tree
//                                              rather than a DAG
//   T4   no cycle
//   T5   exactly one root, and the graph is connected
//
// This order is a contract WITHIN topology validation only. load_urdf validates
// joint type, origin, axis and limits before calling this, so a file that is
// both geometrically and topologically broken reports the geometry fault first.
// That is existing behaviour, not a choice made here.
//
// Diagnostic determinism: hash containers are used for membership tests only.
// Every link named in an error is chosen by walking `links` in DECLARATION
// order and taking the first match -- never by iterating an unordered
// container. Picking it from a hash container would make the same file produce
// different messages on a different standard library, which is precisely the
// class of defect this function was written to remove.
std::expected<void, LoadError>
validate_topology(std::span<Link const> links, std::span<Joint const> joints) {
    // T0a, T0b
    std::unordered_set<std::string_view> declared;
    declared.reserve(links.size());
    for (auto const& link : links) {
        // An empty name is reported with an engaged (empty) rawValue: the empty
        // string is itself the value worth quoting back, exactly as with xyz="".
        if (link.name.empty())
            return std::unexpected(makeError(LoadErrorCode::invalid_link_declaration,
                                             {}, "link/name", std::string_view{}));
        if (!declared.insert(link.name).second)
            return std::unexpected(makeError(LoadErrorCode::invalid_link_declaration,
                                             {}, "link/name", link.name));
    }

    // T0c. The second occurrence is the first point at which uniqueness is
    // observably broken, so that is the one reported.
    std::unordered_set<std::string_view> jointNames;
    jointNames.reserve(joints.size());
    for (auto const& joint : joints) {
        if (joint.name.empty())
            return std::unexpected(makeError(LoadErrorCode::invalid_joint_declaration,
                                             {}, "joint/name", std::string_view{}));
        if (!jointNames.insert(joint.name).second)
            return std::unexpected(makeError(LoadErrorCode::invalid_joint_declaration,
                                             joint.name, "joint/name", joint.name));
    }

    // T1, T2
    for (auto const& joint : joints) {
        if (!declared.contains(joint.parent_link))
            return std::unexpected(makeError(LoadErrorCode::undeclared_parent_link,
                                             joint.name, "parent/link", joint.parent_link));
        if (!declared.contains(joint.child_link))
            return std::unexpected(makeError(LoadErrorCode::undeclared_child_link,
                                             joint.name, "child/link", joint.child_link));
    }

    // T3, plus the adjacency the traversal below needs.
    std::unordered_set<std::string_view> hasParent;
    std::unordered_map<std::string_view, std::vector<std::string_view>> childrenOf;
    hasParent.reserve(joints.size());
    for (auto const& joint : joints) {
        // The SECOND joint claiming a child is the first point at which the
        // invariant is observably broken, so that is the one reported.
        if (!hasParent.insert(joint.child_link).second)
            return std::unexpected(makeError(LoadErrorCode::duplicate_child_link,
                                             joint.name, {}, joint.child_link));
        childrenOf[joint.parent_link].push_back(joint.child_link);
    }

    if (joints.empty()) {
        // Nothing to describe at all.
        if (links.empty())
            return std::unexpected(makeError(LoadErrorCode::parse_failure));
        // A single link is a legal one-node tree. It is also what makes
        // baseLink == toolLink -> empty KinematicChain reachable from a file.
        if (links.size() == 1)
            return {};
        // Several links and no joints are isolated nodes, not one tree.
        return std::unexpected(makeError(LoadErrorCode::disconnected_topology,
                                         {}, {}, links[1].name));
    }

    // Roots, in declaration order -- never taken from a hash container.
    std::vector<std::string_view> roots;
    for (auto const& link : links)
        if (!hasParent.contains(link.name))
            roots.push_back(link.name);

    // Every link has a parent. Walking up parents in a finite graph cannot
    // terminate, so a cycle is certain. This is an INFERENCE from T3, not
    // direct detection -- which is why the reported link is "a link in the
    // cyclic component", not "the link where the cycle closes".
    if (roots.empty())
        return std::unexpected(makeError(LoadErrorCode::cyclic_topology,
                                         {}, {}, links.front().name));

    if (roots.size() > 1)
        return std::unexpected(makeError(LoadErrorCode::disconnected_topology,
                                         {}, {}, roots[1]));

    // T4/T5. Iterative, not recursive: this walks untrusted file input, and the
    // project keeps recursion out of such walks (see KinematicChainBuilder).
    // Termination needs no step counter -- `visited` over a finite node set
    // means each link is pushed at most once.
    std::unordered_set<std::string_view> visited;
    visited.reserve(links.size());
    std::vector<std::string_view> stack{roots.front()};
    visited.insert(roots.front());
    while (!stack.empty()) {
        auto const current = stack.back();
        stack.pop_back();
        auto const it = childrenOf.find(current);
        if (it == childrenOf.end()) continue;
        for (auto const child : it->second)
            if (visited.insert(child).second)
                stack.push_back(child);
    }

    if (visited.size() != links.size()) {
        // An unvisited component cannot contain a root: there is exactly one
        // and it was visited. So every node there has a parent, and by the same
        // argument as above that component contains a cycle.
        for (auto const& link : links)
            if (!visited.contains(link.name))
                return std::unexpected(makeError(LoadErrorCode::cyclic_topology,
                                                 {}, {}, link.name));
    }

    return {};
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

        // <mimic> makes this joint a function of another one. Parsed here;
        // the reference itself is checked once every joint is known.
        if (auto mimic = node.child("mimic")) {
            if (j.type == JointType::fixed)
                return std::unexpected(makeError(LoadErrorCode::unsupported_mimic_joint,
                                                 j.name, "mimic",
                                                 mimic.attribute("joint").as_string()));
            if (mimic.next_sibling("mimic"))
                return std::unexpected(makeError(LoadErrorCode::invalid_mimic_reference,
                                                 j.name, "mimic"));
            Mimic parsed;
            parsed.joint = mimic.attribute("joint").as_string();
            if (parsed.joint.empty())
                return std::unexpected(makeError(LoadErrorCode::invalid_mimic_reference,
                                                 j.name, "mimic/joint", std::string_view{}));
            for (auto [name, target] : {std::pair{"multiplier", &parsed.multiplier},
                                        std::pair{"offset", &parsed.offset}}) {
                if (auto attribute = mimic.attribute(name)) {
                    auto const value = parse_scalar(attribute.value());
                    if (!value)
                        return std::unexpected(makeError(LoadErrorCode::invalid_mimic_reference,
                                                         j.name, std::string("mimic/") + name,
                                                         attribute.value()));
                    *target = *value;
                }
            }
            j.mimic = std::move(parsed);
        }

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

        // Fixed joints: <limit> is ignored entirely, contents and all. A limit
        // on a joint that cannot move has no meaning, and the domain model does
        // not consume it. Same spirit as <axis> on a fixed joint, though not the
        // same treatment: an axis is still parsed for syntax because the parser
        // runs for every type anyway, while validating a limit here would mean
        // running the whole path only to discard the result.
        // Continuous joints: <limit> is optional (URDF), lower/upper carry no
        // meaning and are ignored, but a <limit> that is present still follows
        // the strict profile for velocity and effort.
        if (j.type == JointType::continuous) {
            auto lim = node.child("limit");
            if (lim) {
                if (lim.next_sibling("limit"))
                    return std::unexpected(makeError(LoadErrorCode::duplicate_limit_element,
                                                     j.name, "limit"));
                auto const velocity = read_limit_attribute(lim, j.name, "velocity");
                if (!velocity) return std::unexpected(velocity.error());
                auto const effort = read_limit_attribute(lim, j.name, "effort");
                if (!effort) return std::unexpected(effort.error());
                if (*velocity < 0.0)
                    return std::unexpected(makeError(LoadErrorCode::invalid_limit_constraint,
                                                     j.name, "limit/velocity"));
                if (*effort < 0.0)
                    return std::unexpected(makeError(LoadErrorCode::invalid_limit_constraint,
                                                     j.name, "limit/effort"));
                j.limits.velocity = *velocity;
                j.limits.effort   = *effort;
            }
        } else if (j.type != JointType::fixed) {
            auto lim = node.child("limit");
            if (!lim)
                return std::unexpected(makeError(LoadErrorCode::missing_limit_element,
                                                 j.name, "limit"));

            // Exactly one, not "at least one". node.child() returns the first
            // and would silently ignore the rest — input that means two
            // different things, quietly resolved to one. Checked before the
            // attributes: until it is known which element applies, reporting a
            // fault inside one of them would point at an arbitrary choice.
            if (lim.next_sibling("limit"))
                return std::unexpected(makeError(LoadErrorCode::duplicate_limit_element,
                                                 j.name, "limit"));

            // Fixed order, so a file missing several attributes always reports
            // the same one. Never a loop over lim.attributes(), whose order
            // follows the file.
            auto const lower = read_limit_attribute(lim, j.name, "lower");
            if (!lower) return std::unexpected(lower.error());
            auto const upper = read_limit_attribute(lim, j.name, "upper");
            if (!upper) return std::unexpected(upper.error());
            auto const velocity = read_limit_attribute(lim, j.name, "velocity");
            if (!velocity) return std::unexpected(velocity.error());
            auto const effort = read_limit_attribute(lim, j.name, "effort");
            if (!effort) return std::unexpected(effort.error());

            // Relations last: they only mean something once all four are
            // numbers. lower == upper is allowed — it is what the author wrote
            // twice, not something the parser invented. Zero velocity/effort is
            // allowed too: this validates data, it does not judge whether a
            // robot with those numbers is useful.
            if (*lower > *upper)
                return std::unexpected(makeError(LoadErrorCode::invalid_limit_constraint,
                                                 j.name, "limit/lower"));
            if (*velocity < 0.0)
                return std::unexpected(makeError(LoadErrorCode::invalid_limit_constraint,
                                                 j.name, "limit/velocity"));
            if (*effort < 0.0)
                return std::unexpected(makeError(LoadErrorCode::invalid_limit_constraint,
                                                 j.name, "limit/effort"));

            // Committed only once everything has passed, so a rejected joint
            // never leaves half-written limits behind.
            j.limits.lower    = *lower;
            j.limits.upper    = *upper;
            j.limits.velocity = *velocity;
            j.limits.effort   = *effort;
        }
        out.model.add_joint(std::move(j));
    }

    // Last, deliberately: every joint must be parsed before the graph they form
    // can be judged. Nothing here selects a chain -- selecting one needs a base
    // and a tool link, which only the caller knows (see KinematicChainBuilder).
    if (auto valid = validate_topology(out.model.links(), out.model.joints()); !valid)
        return std::unexpected(valid.error());

    // Mimic references: to an existing, movable, independent, other joint.
    // Checked after topology so the joint names are known to be unique.
    for (auto const& joint : out.model.joints()) {
        if (!joint.mimic)
            continue;
        auto const& joints = out.model.joints();
        auto const target = std::find_if(joints.begin(), joints.end(), [&](Joint const& other) {
            return other.name == joint.mimic->joint;
        });
        if (target == joints.end() || target->name == joint.name ||
            target->type == JointType::fixed || target->mimic)
            return std::unexpected(makeError(LoadErrorCode::invalid_mimic_reference,
                                             joint.name, "mimic/joint", joint.mimic->joint));
    }

    // out.diagnostics is empty on success now. The Info-per-fixed-joint and the
    // "< 6 actuated joints" warning both described a traced chain, and there is
    // no traced chain any more: how many axes a robot has is a property of a
    // chosen path, not of the file, and requiring >= 6 was never the loader's
    // call. UrdfModelLoader discards the bag either way.
    return out;
}

}  // namespace mt::kinematics
