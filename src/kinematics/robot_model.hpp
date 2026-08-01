#pragma once
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace mt::kinematics {

enum class JointType : std::uint8_t {
    revolute,
    prismatic,
    fixed,
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

struct Joint {
    std::string name;
    JointType   type        = JointType::revolute;
    Vec3        axis        = {1.0, 0.0, 0.0};  // Unit vector in the joint frame.
    Vec3        origin_xyz  = {};               // [m]
    Vec3        origin_rpy  = {};               // [rad], roll-pitch-yaw
    std::string parent_link;
    std::string child_link;
    JointLimits limits;
};

struct Link {
    std::string name;
};

class RobotModel {
public:
    [[nodiscard]] std::string_view name() const noexcept;

    [[nodiscard]] std::span<Link const>        links()  const noexcept;
    [[nodiscard]] std::span<Joint const>       joints() const noexcept;
    [[nodiscard]] std::span<std::size_t const> chain()  const noexcept;

    // True jeśli każdy joint_values[i] mieści się w limits actuated joint chain()[i].
    // Rozmiar joint_values musi być równy chain().size(); inaczej false.
    [[nodiscard]] bool within_limits(std::span<double const> joint_values) const noexcept;

    // Builder API — używany przez loader.
    void set_name(std::string n);
    void add_link(Link l);
    void add_joint(Joint j);
    void set_chain(std::vector<std::size_t> indices);

private:
    std::string              name_;
    std::vector<Link>        links_;
    std::vector<Joint>       joints_;
    std::vector<std::size_t> chain_;  // indeksy w joints_, tylko actuated
};

}  // namespace mt::kinematics

// REVIEWED: modern C++ (C++23-first); deviations justified inline.
