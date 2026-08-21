#include "ik_equations/model/QuaternionTarget.hpp"

#include <cassert>
#include <cmath>
#include <utility>

namespace kinemaforge::ik {
namespace {

// Precondition: validate(quaternion) succeeded, so every component is finite
// and the norm lies in [1 - 1e-8, 1 + 1e-8]. In particular it is nowhere near
// zero and this division cannot blow up.
//
// Private for exactly that reason: a function carrying a precondition its
// caller cannot check would be a second door with a worse contract --
// {0,0,0,0} would divide by zero and hand back a matrix full of NaN.
//
// Normalising is NOT repairing the data. A rotation is encoded in the
// DIRECTION of the four-vector; the length is an artefact of how it was
// written down, so dividing canonicalises the representation without choosing
// anything on the caller's behalf. Without it, a quaternion accepted at the
// edge of the band gives |R^T R - I| = 8e-8 (measured), eight times outside
// kOrientationTolerance; with it, 2.22e-15.
RotationMatrix3 toRotationMatrix(const Quaternion& quaternion)
{
    const double norm = std::sqrt(quaternion.w * quaternion.w + quaternion.x * quaternion.x
                                  + quaternion.y * quaternion.y + quaternion.z * quaternion.z);

    const double w = quaternion.w / norm;
    const double x = quaternion.x / norm;
    const double y = quaternion.y / norm;
    const double z = quaternion.z / norm;

    // Quadratic in the components, so R(-q) = R(q) exactly: every product
    // contains an even number of sign-flipped factors.
    return RotationMatrix3{{{{1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z),       2.0 * (x * z + w * y)},
                             {2.0 * (x * y + w * z),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)},
                             {2.0 * (x * z - w * y),       2.0 * (y * z + w * x),       1.0 - 2.0 * (x * x + y * y)}}}};
}

} // namespace

std::expected<PoseTarget, TargetValidationError>
makePoseTargetFromQuaternion(const Vector3& position, const Quaternion& orientation)
{
    if (auto checked = validate(PositionTarget{position}); !checked)
        return std::unexpected(std::move(checked.error()));

    if (auto checked = validate(orientation); !checked)
        return std::unexpected(std::move(checked.error()));

    PoseTarget target{position, toRotationMatrix(orientation)};

    // Not validation of the result -- the conclusion of the design argument,
    // written into the code: after normalising, this matrix MUST pass the check
    // ConstraintBuilder is about to run. It disappears in Release, and that is
    // fine: ProducedOrientationAlwaysPassesTargetValidation pins the property,
    // and build-werror/ keeps assertions live, so the whole suite exercises it.
    //
    // This is NOT the shape removed in R4: there an assert preceded a
    // dereference, so a broken precondition became UB in Release. Nothing
    // follows this one.
    assert(validate(target.orientation).has_value());

    return target;
}

} // namespace kinemaforge::ik
