#pragma once

#include "ik_equations/model/UrdfJoint.hpp"
#include "ik_equations/model/Vector3.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace kinemaforge::ik {

inline constexpr double kGeometryAngularTolerance = 1e-10;
inline constexpr double kGeometryLinearTolerance = 1e-9;

struct AxisLine
{
    Vector3 point;
    Vector3 direction;
};

struct JointAxisGeometry
{
    std::size_t chainPosition{};
    std::size_t sourceJointIndex{};
    std::string name;
    JointType type{JointType::Fixed};
    AxisLine axis;
};

struct KinematicGeometry
{
    std::string baseLink;
    std::string toolLink;
    std::vector<JointAxisGeometry> actuatedJoints;
};

[[nodiscard]] bool axesParallel(const AxisLine& lhs, const AxisLine& rhs,
                                double angularTolerance = kGeometryAngularTolerance) noexcept;
[[nodiscard]] bool axesPerpendicular(const AxisLine& lhs, const AxisLine& rhs,
                                     double angularTolerance = kGeometryAngularTolerance) noexcept;
[[nodiscard]] double axisDistance(const AxisLine& lhs, const AxisLine& rhs) noexcept;
[[nodiscard]] double pointToAxisDistance(const Vector3& point, const AxisLine& axis) noexcept;
[[nodiscard]] bool axesCoincident(const AxisLine& lhs, const AxisLine& rhs,
                                  double angularTolerance = kGeometryAngularTolerance,
                                  double linearTolerance = kGeometryLinearTolerance) noexcept;
[[nodiscard]] std::optional<Vector3> axisIntersectionPoint(
    const AxisLine& lhs,
    const AxisLine& rhs,
    double angularTolerance = kGeometryAngularTolerance,
    double linearTolerance = kGeometryLinearTolerance) noexcept;

} // namespace kinemaforge::ik
