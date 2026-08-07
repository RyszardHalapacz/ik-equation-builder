#include "ik_equations/model/TargetValidation.hpp"

#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
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

// Explicit overload set, not a generic lambda: adding a third alternative to
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

std::expected<void, TargetValidationError> validate(const PoseTarget& target)
{
    if (auto checked = validate(PositionTarget{target.position}); !checked)
        return checked;

    return validate(target.orientation);
}

std::expected<void, TargetValidationError> validate(const IkTarget& target)
{
    return std::visit(TargetVisitor{}, target);
}

} // namespace kinemaforge::ik
