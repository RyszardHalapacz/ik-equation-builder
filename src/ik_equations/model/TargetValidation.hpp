#pragma once

#include "ik_equations/model/IkTarget.hpp"

#include <expected>
#include <string>

namespace kinemaforge::ik {

enum class TargetValidationErrorCode
{
    NonFinitePosition,
    NonFiniteOrientation,
    NonOrthogonalOrientation,
    InvalidOrientationDeterminant
};

struct TargetValidationError
{
    TargetValidationErrorCode code{};
    std::string message;
};

// Absolute tolerance for accepting an input rotation matrix.
//
// NOT the 1e-12 used when comparing FK results: that bound was measured on
// error accumulated inside our own computation, this one accepts data that
// arrived from elsewhere. Measured on 500 000 random rotations rounded to nine
// decimal places, the worst deviations were |R^T R - I| = 1.68e-9 and
// |det - 1| = 1.85e-9 -- an independent run during review reached 1.94e-9 for
// the determinant, so 1e-8 leaves at least five times headroom either way.
//
// The supported input contract is double and text of reasonable precision.
// Data that has passed through float deviates by ~1e-6 and is rejected on
// purpose: the right fix is an explicit orthonormalisation on the caller's
// side, not a threshold loose enough to let a genuinely non-orthogonal target
// through and make the equation system quietly unsatisfiable.
inline constexpr double kOrientationTolerance = 1e-8;

// Checked in this order, and the order is observable:
//   1. all values finite      -> NonFiniteOrientation
//   2. |(R^T R - I)ij| <= tol -> NonOrthogonalOrientation
//   3. |det(R) - 1|    <= tol -> InvalidOrientationDeterminant
//
// Step 3 is NOT "therefore a reflection". That would hold for exactly
// orthogonal matrices, where det is +1 or -1 -- but step 2 accepts a
// tolerance, so a uniform scaling can slip through it and fail here with a
// positive determinant. diag(1+4e-9, 1+4e-9, 1+4e-9) deviates from
// orthogonality by 8e-9 (accepted) and from unit determinant by 1.2e-8
// (rejected), and is no reflection at all. Hence the neutral name.
[[nodiscard]] std::expected<void, TargetValidationError> validate(const PositionTarget& target);
[[nodiscard]] std::expected<void, TargetValidationError> validate(const RotationMatrix3& rotation);
[[nodiscard]] std::expected<void, TargetValidationError> validate(const PoseTarget& target);
[[nodiscard]] std::expected<void, TargetValidationError> validate(const IkTarget& target);

} // namespace kinemaforge::ik
