#include "ik_equations/builders/KinematicGeometryBuilder.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace kinemaforge::ik {
namespace {

using Matrix3 = std::array<std::array<double, 3>, 3>;

Vector3 add(const Vector3& a, const Vector3& b) noexcept
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vector3 subtract(const Vector3& a, const Vector3& b) noexcept
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vector3 scale(const Vector3& v, double factor) noexcept
{
    return {v.x * factor, v.y * factor, v.z * factor};
}

double dot(const Vector3& a, const Vector3& b) noexcept
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vector3 cross(const Vector3& a, const Vector3& b) noexcept
{
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

double norm(const Vector3& v) noexcept
{
    return std::hypot(v.x, v.y, v.z);
}

bool finite(const Vector3& v) noexcept
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

std::optional<Vector3> normalized(const Vector3& v) noexcept
{
    if (!finite(v))
        return std::nullopt;
    const double length = norm(v);
    if (!std::isfinite(length) || length == 0.0)
        return std::nullopt;
    return scale(v, 1.0 / length);
}

Matrix3 identity() noexcept
{
    return {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
}

Matrix3 multiply(const Matrix3& lhs, const Matrix3& rhs) noexcept
{
    Matrix3 result{};
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            for (std::size_t k = 0; k < 3; ++k)
                result[row][column] += lhs[row][k] * rhs[k][column];
    return result;
}

Vector3 multiply(const Matrix3& matrix, const Vector3& vector) noexcept
{
    return {
        matrix[0][0] * vector.x + matrix[0][1] * vector.y + matrix[0][2] * vector.z,
        matrix[1][0] * vector.x + matrix[1][1] * vector.y + matrix[1][2] * vector.z,
        matrix[2][0] * vector.x + matrix[2][1] * vector.y + matrix[2][2] * vector.z};
}

Matrix3 rpyRotation(const Vector3& rpy) noexcept
{
    const double cr = std::cos(rpy.x);
    const double sr = std::sin(rpy.x);
    const double cp = std::cos(rpy.y);
    const double sp = std::sin(rpy.y);
    const double cy = std::cos(rpy.z);
    const double sy = std::sin(rpy.z);

    // Rz(yaw) * Ry(pitch) * Rx(roll), identical to the FK convention.
    return {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
             {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
             {-sp, cp * sr, cp * cr}}};
}

bool isActuated(JointType type) noexcept
{
    switch (type)
    {
    case JointType::Fixed: return false;
    case JointType::Revolute:
    case JointType::Continuous:
    case JointType::Prismatic: return true;
    }
    return false;
}

KinematicGeometryError error(KinematicGeometryErrorCode code,
                             const KinematicJoint& joint,
                             std::string message)
{
    return {code, joint.name, std::move(message)};
}

} // namespace

bool axesParallel(const AxisLine& lhs, const AxisLine& rhs, double angularTolerance) noexcept
{
    return norm(cross(lhs.direction, rhs.direction)) <= angularTolerance;
}

bool axesPerpendicular(const AxisLine& lhs, const AxisLine& rhs,
                       double angularTolerance) noexcept
{
    return std::abs(dot(lhs.direction, rhs.direction)) <= angularTolerance;
}

double pointToAxisDistance(const Vector3& point, const AxisLine& axis) noexcept
{
    return norm(cross(subtract(point, axis.point), axis.direction));
}

double axisDistance(const AxisLine& lhs, const AxisLine& rhs) noexcept
{
    const Vector3 normal = cross(lhs.direction, rhs.direction);
    const double normalLength = norm(normal);
    const Vector3 delta = subtract(rhs.point, lhs.point);
    if (normalLength <= kGeometryAngularTolerance)
        return norm(cross(delta, lhs.direction));
    return std::abs(dot(delta, normal)) / normalLength;
}

bool axesCoincident(const AxisLine& lhs, const AxisLine& rhs,
                    double angularTolerance, double linearTolerance) noexcept
{
    return axesParallel(lhs, rhs, angularTolerance) &&
           pointToAxisDistance(rhs.point, lhs) <= linearTolerance;
}

std::optional<Vector3> axisIntersectionPoint(const AxisLine& lhs, const AxisLine& rhs,
                                             double angularTolerance,
                                             double linearTolerance) noexcept
{
    const Vector3 normal = cross(lhs.direction, rhs.direction);
    const double denominator = dot(normal, normal);
    if (denominator <= angularTolerance * angularTolerance)
        return std::nullopt;

    if (axisDistance(lhs, rhs) > linearTolerance)
        return std::nullopt;

    const Vector3 delta = subtract(rhs.point, lhs.point);
    const double parameter = dot(cross(delta, rhs.direction), normal) / denominator;
    const Vector3 first = add(lhs.point, scale(lhs.direction, parameter));

    // For nearly-intersecting lines use the midpoint of the closest points.
    const double secondParameter = dot(cross(delta, lhs.direction), normal) / denominator;
    const Vector3 second = add(rhs.point, scale(rhs.direction, secondParameter));
    return scale(add(first, second), 0.5);
}

std::expected<KinematicGeometry, KinematicGeometryError>
KinematicGeometryBuilder::build(const KinematicChain& chain) const
{
    KinematicGeometry geometry;
    geometry.baseLink = chain.baseLink;
    geometry.toolLink = chain.toolLink;

    Matrix3 baseRotation = identity();
    Vector3 baseTranslation{};

    for (std::size_t position = 0; position < chain.joints.size(); ++position)
    {
        const KinematicJoint& joint = chain.joints[position];
        if (!finite(joint.origin.translation) || !finite(joint.origin.rpy))
            return std::unexpected(error(KinematicGeometryErrorCode::NonFiniteOrigin, joint,
                                         "joint origin contains a non-finite value"));

        const Matrix3 originRotation = rpyRotation(joint.origin.rpy);
        const Vector3 jointPoint = add(baseTranslation,
                                       multiply(baseRotation, joint.origin.translation));
        const Matrix3 jointRotation = multiply(baseRotation, originRotation);

        if (isActuated(joint.type))
        {
            const auto axis = normalized(multiply(jointRotation, joint.axis));
            if (!axis)
                return std::unexpected(error(KinematicGeometryErrorCode::InvalidActuatedAxis,
                                             joint,
                                             "actuated joint axis is non-finite or zero"));

            geometry.actuatedJoints.push_back(JointAxisGeometry{
                position,
                joint.index,
                joint.name,
                joint.type,
                AxisLine{jointPoint, *axis}});
        }

        baseTranslation = jointPoint;
        baseRotation = jointRotation;
    }

    return geometry;
}

} // namespace kinemaforge::ik
