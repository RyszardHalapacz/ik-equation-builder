#include "ik_equations/builders/JointTransformBuilder.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>

namespace kinemaforge::ik {

namespace {

// An axis that is off unit length by more than this is a broken contract,
// not rounding: the loader normalizes, so real error is a few ULP. The
// bound is loose on purpose — it exists to catch a hand-built joint or a
// regression upstream, not to police the last bits.
constexpr double kUnitAxisTolerance = 1e-9;

bool isUnitLength(const Vector3& axis) noexcept
{
    return std::abs(std::hypot(axis.x, axis.y, axis.z) - 1.0) <= kUnitAxisTolerance;
}

// --- primitive blocks -----------------------------------------------

// Rotation about one principal axis. `negated` folds the sign into the
// sine rather than wrapping the result, since R(-a, q) = R(a, -q) and
// cos is even: without it the cells would carry Negate(Negate(Sin)).
enum class PrincipalAxis { X, Y, Z };

SymbolicRotation buildPrincipalRotation(PrincipalAxis axis,
                                        bool negated,
                                        const Expression& variable,
                                        const ExpressionFactory& factory)
{
    const Expression cosine = factory.cos(variable);
    const Expression rawSine = factory.sin(variable);

    const Expression sine     = negated ? factory.negate(rawSine) : rawSine;
    const Expression minusSine = negated ? rawSine : factory.negate(rawSine);

    auto rotation = SymbolicRotation::identity();
    switch (axis)
    {
    case PrincipalAxis::X:
        rotation(1, 1) = cosine; rotation(1, 2) = minusSine;
        rotation(2, 1) = sine;   rotation(2, 2) = cosine;
        break;
    case PrincipalAxis::Y:
        rotation(0, 0) = cosine; rotation(0, 2) = sine;
        rotation(2, 0) = minusSine; rotation(2, 2) = cosine;
        break;
    case PrincipalAxis::Z:
        rotation(0, 0) = cosine; rotation(0, 1) = minusSine;
        rotation(1, 0) = sine;   rotation(1, 1) = cosine;
        break;
    }
    return rotation;
}

// R_rpy = Rz(yaw) * Ry(pitch) * Rx(roll) — URDF's fixed-axis convention.
//
// Built by composing the three principal rotations rather than writing out
// the nine closed-form entries: every cell here is a constant, so folding
// collapses the products immediately and the code stays readable as the
// formula it implements.
SymbolicRotation buildRpyRotation(const Vector3& rpy, const ExpressionFactory& factory)
{
    const auto roll  = buildPrincipalRotation(PrincipalAxis::X, false, factory.constant(rpy.x), factory);
    const auto pitch = buildPrincipalRotation(PrincipalAxis::Y, false, factory.constant(rpy.y), factory);
    const auto yaw   = buildPrincipalRotation(PrincipalAxis::Z, false, factory.constant(rpy.z), factory);

    return multiply(multiply(yaw, pitch, factory), roll, factory);
}

// True when every cell is exactly the constant it would be in an identity
// matrix. Asks about constants, not about equivalence of expressions.
bool isIdentityRotation(const SymbolicRotation& rotation) noexcept
{
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
        {
            const bool ok = (row == column) ? isOne(rotation(row, column))
                                            : isZero(rotation(row, column));
            if (!ok) return false;
        }
    return true;
}

// Recognises an axis that is exactly a signed principal direction, so the
// caller can emit a canonical Rx/Ry/Rz instead of the general formula.
// The components are constants from the URDF, so exact comparison is the
// right test — a slightly tilted axis is a different axis, not this one.
struct PrincipalAxisMatch { PrincipalAxis axis; bool negated; };

std::optional<PrincipalAxisMatch> matchPrincipalAxis(const Vector3& unitAxis) noexcept
{
    if (unitAxis.y == 0.0 && unitAxis.z == 0.0)
    {
        if (unitAxis.x ==  1.0) return PrincipalAxisMatch{PrincipalAxis::X, false};
        if (unitAxis.x == -1.0) return PrincipalAxisMatch{PrincipalAxis::X, true};
    }
    if (unitAxis.x == 0.0 && unitAxis.z == 0.0)
    {
        if (unitAxis.y ==  1.0) return PrincipalAxisMatch{PrincipalAxis::Y, false};
        if (unitAxis.y == -1.0) return PrincipalAxisMatch{PrincipalAxis::Y, true};
    }
    if (unitAxis.x == 0.0 && unitAxis.y == 0.0)
    {
        if (unitAxis.z ==  1.0) return PrincipalAxisMatch{PrincipalAxis::Z, false};
        if (unitAxis.z == -1.0) return PrincipalAxisMatch{PrincipalAxis::Z, true};
    }
    return std::nullopt;
}

// Rodrigues' rotation formula for a unit axis a = [x, y, z]:
//
//     R = [ t*x^2 + c    t*x*y - s*z   t*x*z + s*y ]
//         [ t*x*y + s*z  t*y^2 + c     t*y*z - s*x ]
//         [ t*x*z - s*y  t*y*z + s*x   t*z^2 + c   ]
//
// with c = cos(q), s = sin(q), t = 1 - c.
//
// A signed principal axis takes a shortcut to the canonical Rx/Ry/Rz. That
// is not an algebraic simplification: the axis is a constant known while
// building, and the general formula would leave cells such as
// ((1-cos(q))*0)*0 + cos(q) that nothing in this project can reduce.
SymbolicRotation buildAxisAngleRotation(const Vector3& unitAxis,
                                        const Expression& variable,
                                        const ExpressionFactory& factory)
{
    if (const auto principal = matchPrincipalAxis(unitAxis))
        return buildPrincipalRotation(principal->axis, principal->negated, variable, factory);

    const Expression cosine = factory.cos(variable);
    const Expression sine   = factory.sin(variable);
    const Expression versine = factory.subtract(factory.constant(1.0), cosine);

    const Expression x = factory.constant(unitAxis.x);
    const Expression y = factory.constant(unitAxis.y);
    const Expression z = factory.constant(unitAxis.z);

    const Expression tx = factory.multiply(versine, x);
    const Expression ty = factory.multiply(versine, y);
    const Expression tz = factory.multiply(versine, z);

    const Expression sx = factory.multiply(sine, x);
    const Expression sy = factory.multiply(sine, y);
    const Expression sz = factory.multiply(sine, z);

    SymbolicRotation rotation;
    rotation(0, 0) = factory.add(factory.multiply(tx, x), cosine);
    rotation(0, 1) = factory.subtract(factory.multiply(tx, y), sz);
    rotation(0, 2) = factory.add(factory.multiply(tx, z), sy);

    rotation(1, 0) = factory.add(factory.multiply(tx, y), sz);
    rotation(1, 1) = factory.add(factory.multiply(ty, y), cosine);
    rotation(1, 2) = factory.subtract(factory.multiply(ty, z), sx);

    rotation(2, 0) = factory.subtract(factory.multiply(tx, z), sy);
    rotation(2, 1) = factory.add(factory.multiply(ty, z), sx);
    rotation(2, 2) = factory.add(factory.multiply(tz, z), cosine);
    return rotation;
}

// component * q, without building a multiplication for a zero component.
//
// Safe even though x * 0 -> 0 is deliberately absent from the factory:
// that rule was rejected because an arbitrary x may have a restricted
// domain, whereas here the other operand is a bare joint variable — total
// on the reals — and the component is a constant known while building.
Expression scaledAxisComponent(double component,
                               const Expression& variable,
                               const ExpressionFactory& factory)
{
    if (component == 0.0)
        return factory.constant(0.0);
    return factory.multiply(factory.constant(component), variable);
}

SymbolicVector3 buildPrismaticDisplacement(const Vector3& unitAxis,
                                           const Expression& variable,
                                           const ExpressionFactory& factory)
{
    SymbolicVector3 displacement;
    displacement(0, 0) = scaledAxisComponent(unitAxis.x, variable, factory);
    displacement(1, 0) = scaledAxisComponent(unitAxis.y, variable, factory);
    displacement(2, 0) = scaledAxisComponent(unitAxis.z, variable, factory);
    return displacement;
}

SymbolicVector3 toSymbolicVector(const Vector3& value, const ExpressionFactory& factory)
{
    SymbolicVector3 vector;
    vector(0, 0) = factory.constant(value.x);
    vector(1, 0) = factory.constant(value.y);
    vector(2, 0) = factory.constant(value.z);
    return vector;
}

SymbolicVector3 addVectors(const SymbolicVector3& lhs,
                           const SymbolicVector3& rhs,
                           const ExpressionFactory& factory)
{
    SymbolicVector3 sum;
    for (std::size_t row = 0; row < 3; ++row)
        sum(row, 0) = factory.add(lhs(row, 0), rhs(row, 0));
    return sum;
}

// Starting from identity leaves the homogeneous last row [0 0 0 1] correct
// by construction. Multiplying two full 4x4 matrices would not: the row
// would become 0*R00 + 0*R10 + 0*R20, which is zero mathematically but not
// a Constant(0) node, so isZero() would stop recognising it.
SymbolicTransform assembleTransform(const SymbolicRotation& rotation,
                                    const SymbolicVector3& translation)
{
    auto transform = SymbolicTransform::identity();
    for (std::size_t row = 0; row < 3; ++row)
    {
        for (std::size_t column = 0; column < 3; ++column)
            transform(row, column) = rotation(row, column);
        transform(row, 3) = translation(row, 0);
    }
    return transform;
}

// R_origin * R_motion, skipping the product when origin contributes no
// rotation. Without this, every joint of a robot whose origins carry no
// rpy — every joint of kr640.urdf — would drag along terms like
// 0 * sin(q), which the factory deliberately does not fold away.
SymbolicRotation composeRotations(const SymbolicRotation& originRotation,
                                  const SymbolicRotation& motionRotation,
                                  const ExpressionFactory& factory)
{
    if (isIdentityRotation(originRotation))
        return motionRotation;
    return multiply(originRotation, motionRotation, factory);
}

SymbolicVector3 rotateVector(const SymbolicRotation& rotation,
                             const SymbolicVector3& vector,
                             const ExpressionFactory& factory)
{
    if (isIdentityRotation(rotation))
        return vector;
    return multiply(rotation, vector, factory);
}

bool isActuated(JointType type) noexcept
{
    switch (type)
    {
    case JointType::Revolute:
    case JointType::Prismatic:
    case JointType::Continuous:
        return true;
    case JointType::Fixed:
        return false;
    }
    return false;
}

} // namespace

JointTransformBuilder::JointTransformBuilder(ExpressionFactory factory)
    : factory_(std::move(factory))
{
}

SymbolicTransform JointTransformBuilder::build(const KinematicJoint& joint) const
{
    assert(isActuated(joint.type) == joint.variable.has_value() &&
           "exactly the actuated joints carry a symbolic variable");

    const SymbolicRotation originRotation = buildRpyRotation(joint.origin.rpy, factory_);
    const SymbolicVector3 originTranslation = toSymbolicVector(joint.origin.translation, factory_);

    if (joint.type == JointType::Fixed)
        return assembleTransform(originRotation, originTranslation);

    assert(isUnitLength(joint.axis) && "actuated joint axis must be a unit vector");

    // The name comes from the chain builder; inventing one here would let
    // forward kinematics and the solver drift onto different symbols.
    const Expression variable = factory_.symbol(joint.variable->name);

    if (joint.type == JointType::Prismatic)
    {
        const auto displacement = buildPrismaticDisplacement(joint.axis, variable, factory_);
        const auto offset = rotateVector(originRotation, displacement, factory_);
        return assembleTransform(originRotation,
                                 addVectors(originTranslation, offset, factory_));
    }

    // Revolute and Continuous are geometrically identical; the difference
    // is a limit, which is not this component's concern.
    const auto motionRotation = buildAxisAngleRotation(joint.axis, variable, factory_);
    return assembleTransform(composeRotations(originRotation, motionRotation, factory_),
                             originTranslation);
}

} // namespace kinemaforge::ik
