#include "ik_equations/model/TargetValidation.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <numbers>
#include <sstream>
#include <string>
#include <utility>
#include <variant>

namespace kinemaforge::ik {

namespace {

std::unexpected<TargetValidationError> makeError(TargetValidationErrorCode code,
                                                 std::string message)
{
    return std::unexpected(TargetValidationError{code, std::move(message)});
}

// std::to_string prints six decimals, so a determinant of 1 + 1.2e-8 -- the
// very value that caused the rejection -- would render as "1.000000". At these
// magnitudes the message has to carry full precision or it is worse than no
// message.
std::string format(double value)
{
    std::ostringstream stream;
    stream << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
    return stream.str();
}

const char* describeAxis(std::size_t index) noexcept
{
    switch (index)
    {
    case 0: return "x";
    case 1: return "y";
    default: return "z";
    }
}

const char* describeQuaternionComponent(std::size_t index) noexcept
{
    switch (index)
    {
    case 0: return "w";
    case 1: return "x";
    case 2: return "y";
    default: return "z";
    }
}

// The zero test runs on the largest component in magnitude, not on a norm: a
// vector is zero exactly when max(|x|,|y|,|z|) is zero. No intermediate
// arithmetic, so neither overflow nor underflow can happen here -- and the same
// value is what ConstraintBuilder uses as its scale.
std::expected<void, TargetValidationError> checkDirection(const Vector3& direction)
{
    const double components[3] = {direction.x, direction.y, direction.z};
    for (std::size_t index = 0; index < 3; ++index)
        if (!std::isfinite(components[index]))
            return makeError(TargetValidationErrorCode::NonFiniteDirection,
                             std::string("direction ") + describeAxis(index)
                                 + " is not finite");

    const double scale = std::max({std::abs(direction.x), std::abs(direction.y),
                                   std::abs(direction.z)});
    if (scale == 0.0)
        return makeError(TargetValidationErrorCode::ZeroDirection,
                         "direction is the zero vector and names no direction");

    return {};
}

std::expected<void, TargetValidationError> checkAngle(double angle)
{
    if (!std::isfinite(angle))
        return makeError(TargetValidationErrorCode::NonFiniteAngle, "angle is not finite");

    if (angle < 0.0 || angle > std::numbers::pi_v<double>)
        return makeError(TargetValidationErrorCode::AngleOutOfRange,
                         "angle must lie in [0, pi], got " + format(angle));

    // The mathematical endpoints first, so the rejection does not depend on the
    // implementation of std::cos.
    if (angle == 0.0 || angle == std::numbers::pi_v<double>)
        return makeError(TargetValidationErrorCode::DegenerateAngle,
                         "angle constrains the tool axis to a single direction; use "
                         "ToolDirectionTarget with direction = +/- reference");

    // ...then whatever has collapsed onto them numerically. This tests the very
    // value that becomes the right-hand side of the equation.
    if (std::abs(std::cos(angle)) == 1.0)
        return makeError(TargetValidationErrorCode::DegenerateAngle,
                         "angle is numerically degenerate: cos(angle) is indistinguishable "
                         "from +/-1; use ToolDirectionTarget with direction = +/- reference");

    return {};
}

std::expected<void, TargetValidationError> checkToolAxis(ToolAxis axis)
{
    if (!toolAxisColumn(axis))
        return makeError(TargetValidationErrorCode::InvalidToolAxis,
                         "tool axis is not one of X, Y, Z");
    return {};
}

// Rejected, never repaired. Re-orthonormalising would change the target the
// caller asked for and hide their mistake -- the same reason the quaternion
// reference checks its norm instead of fixing it.
std::expected<void, TargetValidationError> checkFinite(const RotationMatrix3& rotation)
{
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            if (!std::isfinite(rotation.values[row][column]))
                return makeError(TargetValidationErrorCode::NonFiniteOrientation,
                                 "orientation (" + std::to_string(row) + ", " +
                                     std::to_string(column) + ") is not finite");
    return {};
}

std::expected<void, TargetValidationError> checkOrthogonal(const RotationMatrix3& rotation)
{
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
        {
            double product = 0.0;
            for (std::size_t k = 0; k < 3; ++k)
                product += rotation.values[k][i] * rotation.values[k][j];

            const double expected = (i == j) ? 1.0 : 0.0;
            if (std::abs(product - expected) > kOrientationTolerance)
                return makeError(TargetValidationErrorCode::NonOrthogonalOrientation,
                                 "(R^T R)(" + std::to_string(i) + ", " + std::to_string(j) +
                                     ") = " + format(product));
        }
    return {};
}

double determinant(const RotationMatrix3& rotation) noexcept
{
    const auto& m = rotation.values;
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
           m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

// Explicit overload set, not a generic lambda: adding a new alternative to
// IkTarget must fail to compile until somebody decides how to validate it.
struct TargetVisitor
{
    std::expected<void, TargetValidationError> operator()(const PositionTarget& target) const
    {
        return validate(target);
    }

    std::expected<void, TargetValidationError> operator()(const PoseTarget& target) const
    {
        return validate(target);
    }

    std::expected<void, TargetValidationError> operator()(const ToolDirectionTarget& target) const
    {
        return validate(target);
    }

    std::expected<void, TargetValidationError>
    operator()(const PositionAndToolDirectionTarget& target) const
    {
        return validate(target);
    }

    std::expected<void, TargetValidationError>
    operator()(const DirectionAngleTarget& target) const
    {
        return validate(target);
    }

    std::expected<void, TargetValidationError>
    operator()(const PositionAndDirectionAngleTarget& target) const
    {
        return validate(target);
    }
};

} // namespace

std::expected<void, TargetValidationError> validate(const PositionTarget& target)
{
    const double components[3] = {target.position.x, target.position.y, target.position.z};
    for (std::size_t index = 0; index < 3; ++index)
        if (!std::isfinite(components[index]))
            return makeError(TargetValidationErrorCode::NonFinitePosition,
                             std::string("position ") + describeAxis(index) + " is not finite");
    return {};
}

std::expected<void, TargetValidationError> validate(const RotationMatrix3& rotation)
{
    if (auto checked = checkFinite(rotation); !checked)
        return checked;

    if (auto checked = checkOrthogonal(rotation); !checked)
        return checked;

    // Computed once, and NOT called a reflection: a matrix that passed the
    // tolerant orthogonality check can still be a slight uniform scaling with
    // a positive determinant.
    const double determinantValue = determinant(rotation);
    if (std::abs(determinantValue - 1.0) > kOrientationTolerance)
        return makeError(TargetValidationErrorCode::InvalidOrientationDeterminant,
                         "orientation determinant differs from +1: det = " +
                             format(determinantValue));

    return {};
}

std::expected<void, TargetValidationError> validate(const Quaternion& quaternion)
{
    const double components[4] = {quaternion.w, quaternion.x, quaternion.y, quaternion.z};
    for (std::size_t index = 0; index < 4; ++index)
        if (!std::isfinite(components[index]))
            return makeError(TargetValidationErrorCode::NonFiniteQuaternion,
                             std::string("quaternion ") + describeQuaternionComponent(index)
                                 + " is not finite");

    // The finiteness loop above is FIRST for two independent reasons, worth
    // separating because they guard against different things:
    //
    //   no finiteness check at all -> NaN gives a NaN norm, "NaN > tol" is
    //                                 false, so NaN passes as a unit quaternion;
    //   norm checked before it     -> inf gives an inf norm, "inf > tol" is
    //                                 true, so inf comes back as
    //                                 NonNormalizedQuaternion instead of
    //                                 NonFiniteQuaternion -- the code lies.
    //
    // Moving this sqrt up while keeping the loop before any return changes
    // nothing for NaN. It is the inf case that pins the order.
    //
    // Note also that finite components do not imply a finite sum of squares:
    // w = 1e200 squares to inf. That is not a hole -- |inf - 1| > tol, so such
    // a quaternion is rejected as non-normalised, which is the right answer.
    const double norm = std::sqrt(quaternion.w * quaternion.w + quaternion.x * quaternion.x
                                  + quaternion.y * quaternion.y + quaternion.z * quaternion.z);

    if (std::abs(norm - 1.0) > kQuaternionNormTolerance)
        return makeError(TargetValidationErrorCode::NonNormalizedQuaternion,
                         "quaternion norm differs from 1: ||q|| = " + format(norm));

    return {};
}

std::expected<void, TargetValidationError> validate(const PoseTarget& target)
{
    if (auto checked = validate(PositionTarget{target.position}); !checked)
        return checked;

    return validate(target.orientation);
}

std::expected<void, TargetValidationError> validate(const ToolDirectionTarget& target)
{
    if (auto checked = checkDirection(target.direction); !checked)
        return checked;

    return checkToolAxis(target.axis);
}

std::expected<void, TargetValidationError>
validate(const PositionAndToolDirectionTarget& target)
{
    if (auto checked = validate(PositionTarget{target.position}); !checked)
        return checked;

    return validate(ToolDirectionTarget{target.direction, target.axis});
}

std::expected<void, TargetValidationError> validate(const DirectionAngleTarget& target)
{
    if (auto checked = checkDirection(target.reference); !checked)
        return checked;

    if (auto checked = checkAngle(target.angle); !checked)
        return checked;

    return checkToolAxis(target.axis);
}

std::expected<void, TargetValidationError>
validate(const PositionAndDirectionAngleTarget& target)
{
    if (auto checked = validate(PositionTarget{target.position}); !checked)
        return checked;

    return validate(DirectionAngleTarget{target.reference, target.angle, target.axis});
}

std::expected<void, TargetValidationError> validate(const IkTarget& target)
{
    return std::visit(TargetVisitor{}, target);
}

} // namespace kinemaforge::ik
