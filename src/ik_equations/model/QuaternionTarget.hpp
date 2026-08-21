#pragma once

#include "ik_equations/model/IkTarget.hpp"
#include "ik_equations/model/Quaternion.hpp"
#include "ik_equations/model/TargetValidation.hpp"
#include "ik_equations/model/Vector3.hpp"

#include <expected>

namespace kinemaforge::ik {

// The only door a quaternion enters this project through.
//
// The conversion sits on the boundary of the model rather than inside it: once
// a target exists there is exactly one representation of orientation and
// exactly one path to IkEquationSystem. That is what makes "the same equation
// system for a quaternion and for the equivalent matrix" true by construction
// instead of true by test.
//
// The returned PoseTarget passes validate(PoseTarget) ALWAYS. That is not a
// courtesy to ConstraintBuilder but a consistency condition on the model: a
// type whose own validator rejects its own output is broken. The guarantee
// comes from normalising before the formula -- a quaternion accepted at the
// edge of the tolerance band would otherwise produce a matrix 8e-8 away from
// orthogonal, eight times outside kOrientationTolerance.
//
// Position and orientation are validated in that order, the same order
// validate(PoseTarget) uses; the first failure wins.
//
// q and -q are the same rotation and produce the same PoseTarget, cell for
// cell: the formula is quadratic in the components, so the sign cancels in
// every product.
//
// The name says WHICH door is being used rather than relying on overloading by
// type: the input forms this stage deliberately leaves for later -- RPY and a
// rotation vector -- have identical signatures (Vector3, Vector3), so a family
// of makePoseTarget overloads would collapse exactly when it started being
// useful.
[[nodiscard]] std::expected<PoseTarget, TargetValidationError>
makePoseTargetFromQuaternion(const Vector3& position, const Quaternion& orientation);

} // namespace kinemaforge::ik
