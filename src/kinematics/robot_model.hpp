#pragma once
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace mt::kinematics {

enum class JointType : std::uint8_t {
    revolute,
    prismatic,
    fixed,
    continuous,  // revolute without position limits
};

struct JointLimits {
    double lower    = 0.0;  // [rad] revolute, [m] prismatic
    double upper    = 0.0;
    double velocity = 0.0;  // [rad/s] / [m/s]
    double effort   = 0.0;
};

// Local to motion_kinematics (SI units: meters).
// Intentionally not shared with core::Vec3 — there they are mm.
struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// <mimic>: this joint's value is multiplier * value(joint) + offset.
struct Mimic {
    std::string joint;
    double      multiplier = 1.0;
    double      offset     = 0.0;
};

struct Joint {
    std::string name;
    JointType   type        = JointType::revolute;
    Vec3        axis        = {1.0, 0.0, 0.0};  // Unit vector in the joint frame.
    Vec3        origin_xyz  = {};               // [m]
    Vec3        origin_rpy  = {};               // [rad], roll-pitch-yaw
    std::string parent_link;
    std::string child_link;
    JointLimits limits;
    std::optional<Mimic> mimic;  // engaged: not an independent axis
};

struct Link {
    std::string name;
};

// Storage model used by the URDF loader.
//
// When produced by load_urdf(), links and joints correspond to the validated
// URDF declarations and preserve file order. The type itself guarantees neither:
// set_name(), add_link() and add_joint() are public, so a caller can build one
// by hand. Validation is the loader's contract, not this aggregate's.
//
// This type deliberately does NOT hold a kinematic chain. It used to: chain_
// carried the indices of one traced linear path, filled by trace_chain() in the
// loader. That was removed because URDF describes a tree, so selecting one path
// needs a base and a tool link, which only a caller knows -- see
// KinematicChainBuilder, which makes that choice and understands branching.
//
// within_limits() went with it. It checked a joint configuration against the
// chain, so with no chain it had nothing to check against; it also had no caller
// anywhere in this repository. A limits check belongs where joint order is
// meaningful -- the ik layer, against KinematicChain -- and whoever writes it
// must reject NaN explicitly: `v < lower || v > upper` is false for NaN, which
// is how the removed version accepted it.
class RobotModel {
public:
    [[nodiscard]] std::string_view name() const noexcept;

    [[nodiscard]] std::span<Link const>  links()  const noexcept;
    [[nodiscard]] std::span<Joint const> joints() const noexcept;

    // Builder API — used by the loader.
    void set_name(std::string n);
    void add_link(Link l);
    void add_joint(Joint j);

private:
    std::string        name_;
    std::vector<Link>  links_;
    std::vector<Joint> joints_;
};

}  // namespace mt::kinematics

// REVIEWED: modern C++ (C++23-first); deviations justified inline.
