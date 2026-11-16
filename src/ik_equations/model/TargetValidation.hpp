#pragma once

#include "ik_equations/model/IkTarget.hpp"
#include "ik_equations/model/Quaternion.hpp"

#include <expected>
#include <string>

namespace kinemaforge::ik {

enum class TargetValidationErrorCode
{
    NonFinitePosition,
    NonFiniteOrientation,
    NonOrthogonalOrientation,
    InvalidOrientationDeterminant,
    NonFiniteQuaternion,
    NonNormalizedQuaternion,
    NonFiniteDirection,
    ZeroDirection,
    InvalidToolAxis,
    NonFiniteAngle,
    AngleOutOfRange,
    DegenerateAngle
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
//
// Inside the accepted band a matrix is still not exactly a rotation. Validation
// leaves it untouched; nearestRotation() below is the explicit, bounded and
// reported projection, and the analytic solvers apply it themselves.
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

// Norm tolerance for an input quaternion. NOT kOrientationTolerance and not
// derived from it: that one measures how far R^T R strays from I, this one
// measures the length of a four-vector. Measured over 200 000 random rotations:
// a unit quaternion written out with nine decimal places is off by at most
// 9.03e-10, so it clears this threshold with 11x to spare. The numeric equality
// with kOrientationTolerance is a coincidence of two measurements; tightening
// that constant must not drag this one along.
//
// What this threshold does NOT tell you: where the data came from. A unit
// quaternion that has been through float lands inside it 41.65% of the time
// (1 000 000 samples, median norm error 1.22e-8), and the identity and
// (.5,.5,.5,.5) survive float with exactly zero error. This is a measure of the
// quality of the representation handed to us, not a detector of the precision
// it was computed in -- and it has no way to be one. A float-derived quaternion
// that is accepted carries a rotation error of at most 9.9e-8 rad.
inline constexpr double kQuaternionNormTolerance = 1e-8;

// Checked in this order, and the order is observable:
//   1. all four components finite     -> NonFiniteQuaternion
//   2. | ||q|| - 1 | <= tolerance     -> NonNormalizedQuaternion
//
// Step 1 exists because NaN passes EVERY comparison in step 2: "NaN > tol" is
// false, so validation without it accepts NaN as a unit quaternion. The same
// shape of bug the 2026-08-17 review found in RobotModel::within_limits()
// (removed in R4).
//
// Step 1 comes FIRST for a different reason: for inf the norm is inf too, and
// "inf > tol" is already true -- so the reversed order would report
// NonNormalizedQuaternion about data that are not numbers. The error code must
// not lie, even where the rejection happens either way.
[[nodiscard]] std::expected<void, TargetValidationError> validate(const Quaternion& quaternion);
[[nodiscard]] std::expected<void, TargetValidationError> validate(const PoseTarget& target);

// Checked in this order, and the order is observable:
//   1. all three direction components finite -> NonFiniteDirection
//   2. the direction is not the zero vector   -> ZeroDirection
//   3. the tool axis is a real enumerator     -> InvalidToolAxis
//
// NO norm tolerance, and that is the difference from a quaternion. A direction
// is a ray: (0,0,1), (0,0,5) and (0,0,0.2) name the same direction, so rejecting
// the last two for not being unit length would invent a contract the caller
// never agreed to. A unit quaternion, by contrast, IS the caller's declaration,
// and a norm of 1.7 there proves a bug.
//
// Step 1 comes first because everything after it assumes numbers: the zero test
// takes max(|x|,|y|,|z|), and with NaN in a component every comparison inside
// max() is false, so its result depends on argument order -- and that same value
// is later used as the scale when ConstraintBuilder canonicalises the direction.
//
// Validation never rewrites the target. The direction stays exactly as the
// caller wrote it; making it a unit vector is the builder's job, and only the
// builder ever needs it.
[[nodiscard]] std::expected<void, TargetValidationError>
validate(const ToolDirectionTarget& target);

// Position first, exactly as in validate(PoseTarget): one order across the whole
// model, and observable -- a target broken in both halves reports the position.
[[nodiscard]] std::expected<void, TargetValidationError>
validate(const PositionAndToolDirectionTarget& target);

// Checked in this order, and the order is observable:
//   1. reference finite, non-zero  -> NonFiniteDirection / ZeroDirection
//   2. angle finite                -> NonFiniteAngle
//   3. angle outside [0, pi]       -> AngleOutOfRange
//   4. angle is 0 or pi            -> DegenerateAngle   (mathematics)
//   5. |cos(angle)| == 1           -> DegenerateAngle   (arithmetic)
//   6. tool axis is an enumerator  -> InvalidToolAxis
//
// Step 3 exists because cos() is what reaches the equation: angle = 3*pi would
// be accepted and silently mean pi, handing the caller a different task than the
// one they asked for.
//
// Steps 4 and 5 are separate on purpose. Step 4 rejects the MATHEMATICAL
// endpoints and is portable -- it does not depend on anyone's libm. Step 5
// catches what has numerically collapsed onto them, and the width of that band
// is a platform detail, not a published tolerance: no constant here names it.
//
// At both endpoints the cone about the reference direction degenerates to a
// line, so the constraint removes two rotational degrees of freedom instead of
// one and the scalar equation goes singular. That geometry is exactly what
// ToolDirectionTarget describes, which is where such a caller is pointed.
[[nodiscard]] std::expected<void, TargetValidationError>
validate(const DirectionAngleTarget& target);

// Position first, as everywhere else in this model.
[[nodiscard]] std::expected<void, TargetValidationError>
validate(const PositionAndDirectionAngleTarget& target);

[[nodiscard]] std::expected<void, TargetValidationError> validate(const IkTarget& target);

// Upper bound on OrientationProjection::correction that the analytic solvers
// accept. A matrix that passed validate(RotationMatrix3) stays below 1e-8 to
// first order; anything above this bound is rejected, never silently rewritten.
inline constexpr double kMaxOrientationCorrection = 2e-8;

struct OrientationProjection
{
    RotationMatrix3 rotation;   // rotation nearest to the input (Frobenius norm)
    double correction{};        // max |rotation_ij - input_ij|
};

// The rotation closest to a matrix that validate(RotationMatrix3) accepted.
//
// This is NOT the quaternion case. Dividing a quaternion by its norm does not
// change the rotation it represents; projecting a matrix CHOOSES a nearby
// rotation, because the input did not name one exactly. That is why the size of
// the change is returned instead of hidden: a caller can inspect `correction`,
// and the analytic solvers report it in their results and refuse a target whose
// correction exceeds kMaxOrientationCorrection.
//
// validate() itself still never rewrites anything, and ConstraintBuilder still
// receives the matrix exactly as written. A caller that wants symbolic
// equations for an exactly orthonormal target calls this first.
//
// Returns NonOrthogonalOrientation when the input is not close enough to a
// rotation for the projection to exist (non-finite values, non-positive
// determinant, no convergence). Inputs accepted by validate() never do.
[[nodiscard]] std::expected<OrientationProjection, TargetValidationError>
nearestRotation(const RotationMatrix3& rotation);

} // namespace kinemaforge::ik
